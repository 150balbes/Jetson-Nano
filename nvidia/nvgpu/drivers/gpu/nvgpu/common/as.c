/*
 * GK20A Address Spaces
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <trace/events/gk20a.h>

#include <nvgpu/kmem.h>
#include <nvgpu/vm.h>
#include <nvgpu/log2.h>
#include <nvgpu/gk20a.h>

/* dumb allocator... */
static int generate_as_share_id(struct gk20a_as *as)
{
	struct gk20a *g = gk20a_from_as(as);

	nvgpu_log_fn(g, " ");
	return ++as->last_share_id;
}
/* still dumb */
static void release_as_share_id(struct gk20a_as *as, int id)
{
	struct gk20a *g = gk20a_from_as(as);

	nvgpu_log_fn(g, " ");
	return;
}

/* address space interfaces for the gk20a module */
static int gk20a_vm_alloc_share(struct gk20a_as_share *as_share,
				u32 big_page_size, u32 flags)
{
	struct gk20a_as *as = as_share->as;
	struct gk20a *g = gk20a_from_as(as);
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm;
	char name[32];
	const bool userspace_managed =
		(flags & NVGPU_AS_ALLOC_USERSPACE_MANAGED) != 0;

	nvgpu_log_fn(g, " ");

	if (big_page_size == 0) {
		big_page_size = g->ops.mm.get_default_big_page_size();
	} else {
		if (!is_power_of_2(big_page_size)) {
			return -EINVAL;
		}

		if (!(big_page_size &
                      nvgpu_mm_get_available_big_page_sizes(g))) {
			return -EINVAL;
		}
	}

	snprintf(name, sizeof(name), "as_%d", as_share->id);

	vm = nvgpu_vm_init(g, big_page_size,
			   big_page_size << 10,
			   mm->channel.kernel_size,
			   mm->channel.user_size + mm->channel.kernel_size,
			   !mm->disable_bigpage, userspace_managed, name);
	if (!vm) {
		return -ENOMEM;
	}

	as_share->vm = vm;
	vm->as_share = as_share;
	vm->enable_ctag = true;

	return 0;
}

int gk20a_as_alloc_share(struct gk20a *g,
			 u32 big_page_size, u32 flags,
			 struct gk20a_as_share **out)
{
	struct gk20a_as_share *as_share;
	int err = 0;

	nvgpu_log_fn(g, " ");
	g = gk20a_get(g);
	if (!g) {
		return -ENODEV;
	}

	*out = NULL;
	as_share = nvgpu_kzalloc(g, sizeof(*as_share));
	if (!as_share) {
		return -ENOMEM;
	}

	as_share->as = &g->as;
	as_share->id = generate_as_share_id(as_share->as);

	/* this will set as_share->vm. */
	err = gk20a_busy(g);
	if (err) {
		goto failed;
	}
	err = gk20a_vm_alloc_share(as_share, big_page_size, flags);
	gk20a_idle(g);

	if (err) {
		goto failed;
	}

	*out = as_share;
	return 0;

failed:
	nvgpu_kfree(g, as_share);
	return err;
}

int gk20a_vm_release_share(struct gk20a_as_share *as_share)
{
	struct vm_gk20a *vm = as_share->vm;
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_log_fn(g, " ");

	vm->as_share = NULL;
	as_share->vm = NULL;

	nvgpu_vm_put(vm);

	return 0;
}

/*
 * channels and the device nodes call this to release.
 * once the ref_cnt hits zero the share is deleted.
 */
int gk20a_as_release_share(struct gk20a_as_share *as_share)
{
	struct gk20a *g = as_share->vm->mm->g;
	int err;

	nvgpu_log_fn(g, " ");

	err = gk20a_busy(g);

	if (err) {
		goto release_fail;
	}

	err = gk20a_vm_release_share(as_share);

	gk20a_idle(g);

release_fail:
	release_as_share_id(as_share->as, as_share->id);
	gk20a_put(g);
	nvgpu_kfree(g, as_share);

	return err;
}

struct gk20a *gk20a_from_as(struct gk20a_as *as)
{
	return container_of(as, struct gk20a, as);
}
