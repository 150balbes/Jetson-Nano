/*
 * Semaphore Sync Framework Integration
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <nvgpu/lock.h>

#include <nvgpu/kmem.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/channel.h>
#include "../linux/channel.h"

#include "../drivers/staging/android/sync.h"

#include "sync_sema_android.h"

static const struct sync_timeline_ops gk20a_sync_timeline_ops;

struct gk20a_sync_timeline {
	struct sync_timeline		obj;
	u32				max;
	u32				min;
};

/**
 * The sync framework dups pts when merging fences. We share a single
 * refcounted gk20a_sync_pt for each duped pt.
 */
struct gk20a_sync_pt {
	struct gk20a			*g;
	struct nvgpu_ref			refcount;
	u32				thresh;
	struct nvgpu_semaphore		*sema;
	struct gk20a_sync_timeline	*obj;

	/*
	 * Use a spin lock here since it will have better performance
	 * than a mutex - there should be very little contention on this
	 * lock.
	 */
	struct nvgpu_spinlock			lock;
};

struct gk20a_sync_pt_inst {
	struct sync_pt			pt;
	struct gk20a_sync_pt		*shared;
};

/**
 * Compares sync pt values a and b, both of which will trigger either before
 * or after ref (i.e. a and b trigger before ref, or a and b trigger after
 * ref). Supplying ref allows us to handle wrapping correctly.
 *
 * Returns -1 if a < b (a triggers before b)
 *	    0 if a = b (a and b trigger at the same time)
 *	    1 if a > b (b triggers before a)
 */
static int __gk20a_sync_pt_compare_ref(
	u32 ref,
	u32 a,
	u32 b)
{
	/*
	 * We normalize both a and b by subtracting ref from them.
	 * Denote the normalized values by a_n and b_n. Note that because
	 * of wrapping, a_n and/or b_n may be negative.
	 *
	 * The normalized values a_n and b_n satisfy:
	 * - a positive value triggers before a negative value
	 * - a smaller positive value triggers before a greater positive value
	 * - a smaller negative value (greater in absolute value) triggers
	 *   before a greater negative value (smaller in absolute value).
	 *
	 * Thus we can just stick to unsigned arithmetic and compare
	 * (u32)a_n to (u32)b_n.
	 *
	 * Just to reiterate the possible cases:
	 *
	 *	1A) ...ref..a....b....
	 *	1B) ...ref..b....a....
	 *	2A) ...b....ref..a....              b_n < 0
	 *	2B) ...a....ref..b....     a_n > 0
	 *	3A) ...a....b....ref..     a_n < 0, b_n < 0
	 *	3A) ...b....a....ref..     a_n < 0, b_n < 0
	 */
	u32 a_n = a - ref;
	u32 b_n = b - ref;
	if (a_n < b_n)
		return -1;
	else if (a_n > b_n)
		return 1;
	else
		return 0;
}

static struct gk20a_sync_pt *to_gk20a_sync_pt(struct sync_pt *pt)
{
	struct gk20a_sync_pt_inst *pti =
			container_of(pt, struct gk20a_sync_pt_inst, pt);
	return pti->shared;
}
static struct gk20a_sync_timeline *to_gk20a_timeline(struct sync_timeline *obj)
{
	if (WARN_ON(obj->ops != &gk20a_sync_timeline_ops))
		return NULL;
	return (struct gk20a_sync_timeline *)obj;
}

static void gk20a_sync_pt_free_shared(struct nvgpu_ref *ref)
{
	struct gk20a_sync_pt *pt =
		container_of(ref, struct gk20a_sync_pt, refcount);
	struct gk20a *g = pt->g;

	if (pt->sema)
		nvgpu_semaphore_put(pt->sema);
	nvgpu_kfree(g, pt);
}

static struct gk20a_sync_pt *gk20a_sync_pt_create_shared(
		struct gk20a *g,
		struct gk20a_sync_timeline *obj,
		struct nvgpu_semaphore *sema)
{
	struct gk20a_sync_pt *shared;

	shared = nvgpu_kzalloc(g, sizeof(*shared));
	if (!shared)
		return NULL;

	nvgpu_ref_init(&shared->refcount);
	shared->g = g;
	shared->obj = obj;
	shared->sema = sema;
	shared->thresh = ++obj->max; /* sync framework has a lock */

	nvgpu_spinlock_init(&shared->lock);

	nvgpu_semaphore_get(sema);

	return shared;
}

static struct sync_pt *gk20a_sync_pt_create_inst(
		struct gk20a *g,
		struct gk20a_sync_timeline *obj,
		struct nvgpu_semaphore *sema)
{
	struct gk20a_sync_pt_inst *pti;

	pti = (struct gk20a_sync_pt_inst *)
		sync_pt_create(&obj->obj, sizeof(*pti));
	if (!pti)
		return NULL;

	pti->shared = gk20a_sync_pt_create_shared(g, obj, sema);
	if (!pti->shared) {
		sync_pt_free(&pti->pt);
		return NULL;
	}
	return &pti->pt;
}

static void gk20a_sync_pt_free_inst(struct sync_pt *sync_pt)
{
	struct gk20a_sync_pt *pt = to_gk20a_sync_pt(sync_pt);
	if (pt)
		nvgpu_ref_put(&pt->refcount, gk20a_sync_pt_free_shared);
}

static struct sync_pt *gk20a_sync_pt_dup_inst(struct sync_pt *sync_pt)
{
	struct gk20a_sync_pt_inst *pti;
	struct gk20a_sync_pt *pt = to_gk20a_sync_pt(sync_pt);

	pti = (struct gk20a_sync_pt_inst *)
		sync_pt_create(&pt->obj->obj, sizeof(*pti));
	if (!pti)
		return NULL;
	pti->shared = pt;
	nvgpu_ref_get(&pt->refcount);
	return &pti->pt;
}

/*
 * This function must be able to run on the same sync_pt concurrently. This
 * requires a lock to protect access to the sync_pt's internal data structures
 * which are modified as a side effect of calling this function.
 */
static int gk20a_sync_pt_has_signaled(struct sync_pt *sync_pt)
{
	struct gk20a_sync_pt *pt = to_gk20a_sync_pt(sync_pt);
	struct gk20a_sync_timeline *obj = pt->obj;
	bool signaled = true;

	nvgpu_spinlock_acquire(&pt->lock);
	if (!pt->sema)
		goto done;

	/* Acquired == not realeased yet == active == not signaled. */
	signaled = !nvgpu_semaphore_is_acquired(pt->sema);

	if (signaled) {
		/* Update min if necessary. */
		if (__gk20a_sync_pt_compare_ref(obj->max, pt->thresh,
						obj->min) == 1)
			obj->min = pt->thresh;

		/* Release the semaphore to the pool. */
		nvgpu_semaphore_put(pt->sema);
		pt->sema = NULL;
	}
done:
	nvgpu_spinlock_release(&pt->lock);

	return signaled;
}

static int gk20a_sync_pt_compare(struct sync_pt *a, struct sync_pt *b)
{
	bool a_expired;
	bool b_expired;
	struct gk20a_sync_pt *pt_a = to_gk20a_sync_pt(a);
	struct gk20a_sync_pt *pt_b = to_gk20a_sync_pt(b);

	if (WARN_ON(pt_a->obj != pt_b->obj))
		return 0;

	/* Early out */
	if (a == b)
		return 0;

	a_expired = gk20a_sync_pt_has_signaled(a);
	b_expired = gk20a_sync_pt_has_signaled(b);
	if (a_expired && !b_expired) {
		/* Easy, a was earlier */
		return -1;
	} else if (!a_expired && b_expired) {
		/* Easy, b was earlier */
		return 1;
	}

	/* Both a and b are expired (trigger before min) or not
	 * expired (trigger after min), so we can use min
	 * as a reference value for __gk20a_sync_pt_compare_ref.
	 */
	return __gk20a_sync_pt_compare_ref(pt_a->obj->min,
			pt_a->thresh, pt_b->thresh);
}

static u32 gk20a_sync_timeline_current(struct gk20a_sync_timeline *obj)
{
	return obj->min;
}

static void gk20a_sync_timeline_value_str(struct sync_timeline *timeline,
		char *str, int size)
{
	struct gk20a_sync_timeline *obj =
		(struct gk20a_sync_timeline *)timeline;
	snprintf(str, size, "%d", gk20a_sync_timeline_current(obj));
}

static void gk20a_sync_pt_value_str_for_sema(struct gk20a_sync_pt *pt,
					     char *str, int size)
{
	struct nvgpu_semaphore *s = pt->sema;

	snprintf(str, size, "S: pool=%llu [v=%u,r_v=%u]",
		 s->location.pool->page_idx,
		 nvgpu_semaphore_get_value(s),
		 nvgpu_semaphore_read(s));
}

static void gk20a_sync_pt_value_str(struct sync_pt *sync_pt, char *str,
		int size)
{
	struct gk20a_sync_pt *pt = to_gk20a_sync_pt(sync_pt);

	if (pt->sema) {
		gk20a_sync_pt_value_str_for_sema(pt, str, size);
		return;
	}

	snprintf(str, size, "%d", pt->thresh);
}

static const struct sync_timeline_ops gk20a_sync_timeline_ops = {
	.driver_name = "nvgpu_semaphore",
	.dup = gk20a_sync_pt_dup_inst,
	.has_signaled = gk20a_sync_pt_has_signaled,
	.compare = gk20a_sync_pt_compare,
	.free_pt = gk20a_sync_pt_free_inst,
	.timeline_value_str = gk20a_sync_timeline_value_str,
	.pt_value_str = gk20a_sync_pt_value_str,
};

/* Public API */

struct sync_fence *gk20a_sync_fence_fdget(int fd)
{
	struct sync_fence *fence = sync_fence_fdget(fd);
	int i;

	if (!fence)
		return NULL;

	for (i = 0; i < fence->num_fences; i++) {
		struct sync_pt *spt = sync_pt_from_fence(fence->cbs[i].sync_pt);
		struct sync_timeline *t;

		if (spt == NULL) {
			sync_fence_put(fence);
			return NULL;
		}

		t = sync_pt_parent(spt);
		if (t->ops != &gk20a_sync_timeline_ops) {
			sync_fence_put(fence);
			return NULL;
		}
	}

	return fence;
}

struct nvgpu_semaphore *gk20a_sync_pt_sema(struct sync_pt *spt)
{
	struct gk20a_sync_pt *pt = to_gk20a_sync_pt(spt);
	struct nvgpu_semaphore *sema;

	nvgpu_spinlock_acquire(&pt->lock);
	sema = pt->sema;
	if (sema)
		nvgpu_semaphore_get(sema);
	nvgpu_spinlock_release(&pt->lock);

	return sema;
}

void gk20a_sync_timeline_signal(struct sync_timeline *timeline)
{
	sync_timeline_signal(timeline, 0);
}

void gk20a_sync_timeline_destroy(struct sync_timeline *timeline)
{
	sync_timeline_destroy(timeline);
}

struct sync_timeline *gk20a_sync_timeline_create(
		const char *name)
{
	struct gk20a_sync_timeline *obj;

	obj = (struct gk20a_sync_timeline *)
		sync_timeline_create(&gk20a_sync_timeline_ops,
				     sizeof(struct gk20a_sync_timeline),
				     name);
	if (!obj)
		return NULL;
	obj->max = 0;
	obj->min = 0;
	return &obj->obj;
}

struct sync_fence *gk20a_sync_fence_create(
		struct channel_gk20a *c,
		struct nvgpu_semaphore *sema,
		const char *fmt, ...)
{
	char name[30];
	va_list args;
	struct sync_pt *pt;
	struct sync_fence *fence;
	struct gk20a *g = c->g;

	struct nvgpu_channel_linux *os_channel_priv = c->os_priv;
	struct nvgpu_os_fence_framework *fence_framework = NULL;
	struct gk20a_sync_timeline *timeline = NULL;

	fence_framework = &os_channel_priv->fence_framework;

	timeline = to_gk20a_timeline(fence_framework->timeline);

	pt = gk20a_sync_pt_create_inst(g, timeline, sema);
	if (pt == NULL)
		return NULL;

	va_start(args, fmt);
	vsnprintf(name, sizeof(name), fmt, args);
	va_end(args);

	fence = sync_fence_create(name, pt);
	if (fence == NULL) {
		sync_pt_free(pt);
		return NULL;
	}
	return fence;
}
