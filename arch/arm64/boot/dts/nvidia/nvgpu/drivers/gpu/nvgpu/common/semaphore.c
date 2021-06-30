/*
 * Nvgpu Semaphores
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/sizes.h>
#include <nvgpu/channel.h>
#include <nvgpu/gk20a.h>

#include "gk20a/mm_gk20a.h"

#define pool_to_gk20a(p) ((p)->sema_sea->gk20a)

#define __lock_sema_sea(s)						\
	do {								\
		gpu_sema_verbose_dbg(s->gk20a, "Acquiring sema lock..."); \
		nvgpu_mutex_acquire(&s->sea_lock);			\
		gpu_sema_verbose_dbg(s->gk20a, "Sema lock aquried!");	\
	} while (0)

#define __unlock_sema_sea(s)						\
	do {								\
		nvgpu_mutex_release(&s->sea_lock);			\
		gpu_sema_verbose_dbg(s->gk20a, "Released sema lock");	\
	} while (0)

/*
 * Return the sema_sea pointer.
 */
struct nvgpu_semaphore_sea *nvgpu_semaphore_get_sea(struct gk20a *g)
{
	return g->sema_sea;
}

static int __nvgpu_semaphore_sea_grow(struct nvgpu_semaphore_sea *sea)
{
	int ret = 0;
	struct gk20a *gk20a = sea->gk20a;
	u32 i;

	__lock_sema_sea(sea);

	ret = nvgpu_dma_alloc_sys(gk20a,
				  PAGE_SIZE * SEMAPHORE_POOL_COUNT,
				  &sea->sea_mem);
	if (ret) {
		goto out;
	}

	sea->size = SEMAPHORE_POOL_COUNT;
	sea->map_size = SEMAPHORE_POOL_COUNT * PAGE_SIZE;

	/*
	 * Start the semaphores at values that will soon overflow the 32-bit
	 * integer range. This way any buggy comparisons would start to fail
	 * sooner rather than later.
	 */
	for (i = 0U; i < PAGE_SIZE * SEMAPHORE_POOL_COUNT; i += 4U) {
		nvgpu_mem_wr(gk20a, &sea->sea_mem, i, 0xfffffff0);
	}

out:
	__unlock_sema_sea(sea);
	return ret;
}

void nvgpu_semaphore_sea_destroy(struct gk20a *g)
{
	if (g->sema_sea == NULL) {
		return;
	}

	nvgpu_dma_free(g, &g->sema_sea->sea_mem);
	nvgpu_mutex_destroy(&g->sema_sea->sea_lock);
	nvgpu_kfree(g, g->sema_sea);
	g->sema_sea = NULL;
}

/*
 * Create the semaphore sea. Only create it once - subsequent calls to this will
 * return the originally created sea pointer.
 */
struct nvgpu_semaphore_sea *nvgpu_semaphore_sea_create(struct gk20a *g)
{
	if (g->sema_sea) {
		return g->sema_sea;
	}

	g->sema_sea = nvgpu_kzalloc(g, sizeof(*g->sema_sea));
	if (g->sema_sea == NULL) {
		return NULL;
	}

	g->sema_sea->size = 0;
	g->sema_sea->page_count = 0;
	g->sema_sea->gk20a = g;
	nvgpu_init_list_node(&g->sema_sea->pool_list);
	if (nvgpu_mutex_init(&g->sema_sea->sea_lock)) {
		goto cleanup_free;
	}

	if (__nvgpu_semaphore_sea_grow(g->sema_sea)) {
		goto cleanup_destroy;
	}

	gpu_sema_dbg(g, "Created semaphore sea!");
	return g->sema_sea;

cleanup_destroy:
	nvgpu_mutex_destroy(&g->sema_sea->sea_lock);
cleanup_free:
	nvgpu_kfree(g, g->sema_sea);
	g->sema_sea = NULL;
	gpu_sema_dbg(g, "Failed to creat semaphore sea!");
	return NULL;
}

static int __semaphore_bitmap_alloc(unsigned long *bitmap, unsigned long len)
{
	unsigned long idx = find_first_zero_bit(bitmap, len);

	if (idx == len) {
		return -ENOSPC;
	}

	set_bit(idx, bitmap);

	return (int)idx;
}

/*
 * Allocate a pool from the sea.
 */
int nvgpu_semaphore_pool_alloc(struct nvgpu_semaphore_sea *sea,
			       struct nvgpu_semaphore_pool **pool)
{
	struct nvgpu_semaphore_pool *p;
	unsigned long page_idx;
	int ret;

	p = nvgpu_kzalloc(sea->gk20a, sizeof(*p));
	if (p == NULL) {
		return -ENOMEM;
	}

	__lock_sema_sea(sea);

	ret = nvgpu_mutex_init(&p->pool_lock);
	if (ret) {
		goto fail;
	}

	ret = __semaphore_bitmap_alloc(sea->pools_alloced,
				       SEMAPHORE_POOL_COUNT);
	if (ret < 0) {
		goto fail_alloc;
	}

	page_idx = (unsigned long)ret;

	p->page_idx = page_idx;
	p->sema_sea = sea;
	nvgpu_init_list_node(&p->pool_list_entry);
	nvgpu_ref_init(&p->ref);

	sea->page_count++;
	nvgpu_list_add(&p->pool_list_entry, &sea->pool_list);
	__unlock_sema_sea(sea);

	gpu_sema_dbg(sea->gk20a,
		     "Allocated semaphore pool: page-idx=%llu", p->page_idx);

	*pool = p;
	return 0;

fail_alloc:
	nvgpu_mutex_destroy(&p->pool_lock);
fail:
	__unlock_sema_sea(sea);
	nvgpu_kfree(sea->gk20a, p);
	gpu_sema_dbg(sea->gk20a, "Failed to allocate semaphore pool!");
	return ret;
}

/*
 * Map a pool into the passed vm's address space. This handles both the fixed
 * global RO mapping and the non-fixed private RW mapping.
 */
int nvgpu_semaphore_pool_map(struct nvgpu_semaphore_pool *p,
			     struct vm_gk20a *vm)
{
	int err = 0;
	u64 addr;

	if (p->mapped) {
		return -EBUSY;
	}

	gpu_sema_dbg(pool_to_gk20a(p),
		     "Mapping semaphore pool! (idx=%llu)", p->page_idx);

	/*
	 * Take the sea lock so that we don't race with a possible change to the
	 * nvgpu_mem in the sema sea.
	 */
	__lock_sema_sea(p->sema_sea);

	addr = nvgpu_gmmu_map_fixed(vm, &p->sema_sea->sea_mem,
				    p->sema_sea->gpu_va,
				    p->sema_sea->map_size,
				    0, gk20a_mem_flag_read_only, 0,
				    p->sema_sea->sea_mem.aperture);
	if (addr == 0ULL) {
		err = -ENOMEM;
		goto fail_unlock;
	}

	p->gpu_va_ro = addr;
	p->mapped = true;

	gpu_sema_dbg(pool_to_gk20a(p),
		     "  %llu: GPU read-only  VA = 0x%llx",
		     p->page_idx, p->gpu_va_ro);

	/*
	 * Now the RW mapping. This is a bit more complicated. We make a
	 * nvgpu_mem describing a page of the bigger RO space and then map
	 * that. Unlike above this does not need to be a fixed address.
	 */
	err = nvgpu_mem_create_from_mem(vm->mm->g,
					&p->rw_mem, &p->sema_sea->sea_mem,
					p->page_idx, 1);
	if (err) {
		goto fail_unmap;
	}

	addr = nvgpu_gmmu_map(vm, &p->rw_mem, SZ_4K, 0,
			      gk20a_mem_flag_none, 0,
			      p->rw_mem.aperture);

	if (addr == 0ULL) {
		err = -ENOMEM;
		goto fail_free_submem;
	}

	p->gpu_va = addr;

	__unlock_sema_sea(p->sema_sea);

	gpu_sema_dbg(pool_to_gk20a(p),
		     "  %llu: GPU read-write VA = 0x%llx",
		     p->page_idx, p->gpu_va);
	gpu_sema_dbg(pool_to_gk20a(p),
		     "  %llu: CPU VA            = 0x%p",
		     p->page_idx, p->rw_mem.cpu_va);

	return 0;

fail_free_submem:
	nvgpu_dma_free(pool_to_gk20a(p), &p->rw_mem);
fail_unmap:
	nvgpu_gmmu_unmap(vm, &p->sema_sea->sea_mem, p->gpu_va_ro);
	gpu_sema_dbg(pool_to_gk20a(p),
		     "  %llu: Failed to map semaphore pool!", p->page_idx);
fail_unlock:
	__unlock_sema_sea(p->sema_sea);
	return err;
}

/*
 * Unmap a semaphore_pool.
 */
void nvgpu_semaphore_pool_unmap(struct nvgpu_semaphore_pool *p,
				struct vm_gk20a *vm)
{
	__lock_sema_sea(p->sema_sea);

	nvgpu_gmmu_unmap(vm, &p->sema_sea->sea_mem, p->gpu_va_ro);
	nvgpu_gmmu_unmap(vm, &p->rw_mem, p->gpu_va);
	nvgpu_dma_free(pool_to_gk20a(p), &p->rw_mem);

	p->gpu_va = 0;
	p->gpu_va_ro = 0;
	p->mapped = false;

	__unlock_sema_sea(p->sema_sea);

	gpu_sema_dbg(pool_to_gk20a(p),
		     "Unmapped semaphore pool! (idx=%llu)", p->page_idx);
}

/*
 * Completely free a semaphore_pool. You should make sure this pool is not
 * mapped otherwise there's going to be a memory leak.
 */
static void nvgpu_semaphore_pool_free(struct nvgpu_ref *ref)
{
	struct nvgpu_semaphore_pool *p =
		container_of(ref, struct nvgpu_semaphore_pool, ref);
	struct nvgpu_semaphore_sea *s = p->sema_sea;

	/* Freeing a mapped pool is a bad idea. */
	WARN_ON((p->mapped) ||
		(p->gpu_va != 0ULL) ||
		(p->gpu_va_ro != 0ULL));

	__lock_sema_sea(s);
	nvgpu_list_del(&p->pool_list_entry);
	clear_bit((int)p->page_idx, s->pools_alloced);
	s->page_count--;
	__unlock_sema_sea(s);

	nvgpu_mutex_destroy(&p->pool_lock);

	gpu_sema_dbg(pool_to_gk20a(p),
		     "Freed semaphore pool! (idx=%llu)", p->page_idx);
	nvgpu_kfree(p->sema_sea->gk20a, p);
}

void nvgpu_semaphore_pool_get(struct nvgpu_semaphore_pool *p)
{
	nvgpu_ref_get(&p->ref);
}

void nvgpu_semaphore_pool_put(struct nvgpu_semaphore_pool *p)
{
	nvgpu_ref_put(&p->ref, nvgpu_semaphore_pool_free);
}

/*
 * Get the address for a semaphore_pool - if global is true then return the
 * global RO address instead of the RW address owned by the semaphore's VM.
 */
u64 __nvgpu_semaphore_pool_gpu_va(struct nvgpu_semaphore_pool *p, bool global)
{
	if (!global) {
		return p->gpu_va;
	}

	return p->gpu_va_ro + (PAGE_SIZE * p->page_idx);
}

static int __nvgpu_init_hw_sema(struct channel_gk20a *ch)
{
	int hw_sema_idx;
	int ret = 0;
	struct nvgpu_semaphore_int *hw_sema;
	struct nvgpu_semaphore_pool *p = ch->vm->sema_pool;
	int current_value;

	BUG_ON(p == NULL);

	nvgpu_mutex_acquire(&p->pool_lock);

	/* Find an available HW semaphore. */
	hw_sema_idx = __semaphore_bitmap_alloc(p->semas_alloced,
					       PAGE_SIZE / SEMAPHORE_SIZE);
	if (hw_sema_idx < 0) {
		ret = hw_sema_idx;
		goto fail;
	}

	hw_sema = nvgpu_kzalloc(ch->g, sizeof(struct nvgpu_semaphore_int));
	if (hw_sema == NULL) {
		ret = -ENOMEM;
		goto fail_free_idx;
	}

	ch->hw_sema = hw_sema;
	hw_sema->ch = ch;
	hw_sema->location.pool = p;
	hw_sema->location.offset = SEMAPHORE_SIZE * (u32)hw_sema_idx;
	current_value = nvgpu_mem_rd(ch->g, &p->rw_mem,
			hw_sema->location.offset);
	nvgpu_atomic_set(&hw_sema->next_value, current_value);

	nvgpu_mutex_release(&p->pool_lock);

	return 0;

fail_free_idx:
	clear_bit(hw_sema_idx, p->semas_alloced);
fail:
	nvgpu_mutex_release(&p->pool_lock);
	return ret;
}

/*
 * Free the channel used semaphore index
 */
void nvgpu_semaphore_free_hw_sema(struct channel_gk20a *ch)
{
	struct nvgpu_semaphore_pool *p = ch->vm->sema_pool;
	struct nvgpu_semaphore_int *hw_sema = ch->hw_sema;
	int idx = hw_sema->location.offset / SEMAPHORE_SIZE;

	BUG_ON(p == NULL);

	nvgpu_mutex_acquire(&p->pool_lock);

	clear_bit(idx, p->semas_alloced);

	nvgpu_kfree(ch->g, hw_sema);
	ch->hw_sema = NULL;

	nvgpu_mutex_release(&p->pool_lock);
}

/*
 * Allocate a semaphore from the passed pool.
 *
 * Since semaphores are ref-counted there's no explicit free for external code
 * to use. When the ref-count hits 0 the internal free will happen.
 */
struct nvgpu_semaphore *nvgpu_semaphore_alloc(struct channel_gk20a *ch)
{
	struct nvgpu_semaphore *s;
	int ret;

	if (ch->hw_sema == NULL) {
		ret = __nvgpu_init_hw_sema(ch);
		if (ret) {
			return NULL;
		}
	}

	s = nvgpu_kzalloc(ch->g, sizeof(*s));
	if (s == NULL) {
		return NULL;
	}

	nvgpu_ref_init(&s->ref);
	s->g = ch->g;
	s->location = ch->hw_sema->location;
	nvgpu_atomic_set(&s->value, 0);

	/*
	 * Take a ref on the pool so that we can keep this pool alive for
	 * as long as this semaphore is alive.
	 */
	nvgpu_semaphore_pool_get(s->location.pool);

	gpu_sema_dbg(ch->g, "Allocated semaphore (c=%d)", ch->chid);

	return s;
}

static void nvgpu_semaphore_free(struct nvgpu_ref *ref)
{
	struct nvgpu_semaphore *s =
		container_of(ref, struct nvgpu_semaphore, ref);

	nvgpu_semaphore_pool_put(s->location.pool);

	nvgpu_kfree(s->g, s);
}

void nvgpu_semaphore_put(struct nvgpu_semaphore *s)
{
	nvgpu_ref_put(&s->ref, nvgpu_semaphore_free);
}

void nvgpu_semaphore_get(struct nvgpu_semaphore *s)
{
	nvgpu_ref_get(&s->ref);
}

/*
 * Return the address of a specific semaphore.
 *
 * Don't call this on a semaphore you don't own - the VA returned will make no
 * sense in your specific channel's VM.
 */
u64 nvgpu_semaphore_gpu_rw_va(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_pool_gpu_va(s->location.pool, false) +
		s->location.offset;
}

/*
 * Get the global RO address for the semaphore. Can be called on any semaphore
 * regardless of whether you own it.
 */
u64 nvgpu_semaphore_gpu_ro_va(struct nvgpu_semaphore *s)
{
	return __nvgpu_semaphore_pool_gpu_va(s->location.pool, true) +
		s->location.offset;
}

u64 nvgpu_hw_sema_addr(struct nvgpu_semaphore_int *hw_sema)
{
	return __nvgpu_semaphore_pool_gpu_va(hw_sema->location.pool, true) +
		hw_sema->location.offset;
}

u32 __nvgpu_semaphore_read(struct nvgpu_semaphore_int *hw_sema)
{
	return nvgpu_mem_rd(hw_sema->ch->g, &hw_sema->location.pool->rw_mem,
			hw_sema->location.offset);
}

/*
 * Read the underlying value from a semaphore.
 */
u32 nvgpu_semaphore_read(struct nvgpu_semaphore *s)
{
	return nvgpu_mem_rd(s->g, &s->location.pool->rw_mem,
			s->location.offset);
}

/*
 * Check if "racer" is over "goal" with wraparound handling.
 */
static bool __nvgpu_semaphore_value_released(u32 goal, u32 racer)
{
	/*
	 * Handle wraparound with the same heuristic as the hardware does:
	 * although the integer will eventually wrap around, consider a sema
	 * released against a threshold if its value has passed that threshold
	 * but has not wrapped over half of the u32 range over that threshold;
	 * such wrapping is unlikely to happen during a sema lifetime.
	 *
	 * Values for [goal, goal + 0x7fffffff] are considered signaled; that's
	 * precisely half of the 32-bit space. If racer == goal + 0x80000000,
	 * then it needs 0x80000000 increments to wrap again and signal.
	 *
	 * Unsigned arithmetic is used because it's well-defined. This is
	 * effectively the same as: signed_racer - signed_goal > 0.
	 */

	return racer - goal < 0x80000000;
}

u32 nvgpu_semaphore_get_value(struct nvgpu_semaphore *s)
{
	return (u32)nvgpu_atomic_read(&s->value);
}

bool nvgpu_semaphore_is_released(struct nvgpu_semaphore *s)
{
	u32 sema_val = nvgpu_semaphore_read(s);
	u32 wait_payload = nvgpu_semaphore_get_value(s);

	return __nvgpu_semaphore_value_released(wait_payload, sema_val);
}

bool nvgpu_semaphore_is_acquired(struct nvgpu_semaphore *s)
{
	return !nvgpu_semaphore_is_released(s);
}

/*
 * Fast-forward the hw sema to its tracked max value.
 *
 * Return true if the sema wasn't at the max value and needed updating, false
 * otherwise.
 */
bool nvgpu_semaphore_reset(struct nvgpu_semaphore_int *hw_sema)
{
	u32 threshold = (u32)nvgpu_atomic_read(&hw_sema->next_value);
	u32 current_val = __nvgpu_semaphore_read(hw_sema);

	/*
	 * If the semaphore has already reached the value we would write then
	 * this is really just a NO-OP. However, the sema value shouldn't be
	 * more than what we expect to be the max.
	 */

	if (WARN_ON(__nvgpu_semaphore_value_released(threshold + 1U,
						     current_val)))
		return false;

	if (current_val == threshold)
		return false;

	nvgpu_mem_wr(hw_sema->ch->g, &hw_sema->location.pool->rw_mem,
			hw_sema->location.offset, threshold);

	gpu_sema_verbose_dbg(hw_sema->ch->g, "(c=%d) RESET %u -> %u",
			hw_sema->ch->chid, current_val, threshold);

	return true;
}

/*
 * Update nvgpu-tracked shadow of the value in "hw_sema" and mark the threshold
 * value to "s" which represents the increment that the caller must write in a
 * pushbuf. The same nvgpu_semaphore will also represent an output fence; when
 * nvgpu_semaphore_is_released(s) == true, the gpu is done with this increment.
 */
void nvgpu_semaphore_prepare(struct nvgpu_semaphore *s,
		struct nvgpu_semaphore_int *hw_sema)
{
	int next = nvgpu_atomic_add_return(1, &hw_sema->next_value);

	/* "s" should be an uninitialized sema. */
	WARN_ON(s->incremented);

	nvgpu_atomic_set(&s->value, next);
	s->incremented = true;

	gpu_sema_verbose_dbg(s->g, "INCR sema for c=%d (%u)",
			     hw_sema->ch->chid, next);
}
