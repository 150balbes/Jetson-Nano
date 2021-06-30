/*
 * gk20a allocator
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

#include <nvgpu/allocator.h>
#include <nvgpu/gk20a.h>

#include "gk20a/mm_gk20a.h"

u64 nvgpu_alloc_length(struct nvgpu_allocator *a)
{
	if (a->ops->length) {
		return a->ops->length(a);
	}

	return 0;
}

u64 nvgpu_alloc_base(struct nvgpu_allocator *a)
{
	if (a->ops->base) {
		return a->ops->base(a);
	}

	return 0;
}

bool nvgpu_alloc_initialized(struct nvgpu_allocator *a)
{
	if (a->ops == NULL || a->ops->inited == NULL) {
		return false;
	}

	return a->ops->inited(a);
}

u64 nvgpu_alloc_end(struct nvgpu_allocator *a)
{
	if (a->ops->end) {
		return a->ops->end(a);
	}

	return 0;
}

u64 nvgpu_alloc_space(struct nvgpu_allocator *a)
{
	if (a->ops->space) {
		return a->ops->space(a);
	}

	return 0;
}

u64 nvgpu_alloc(struct nvgpu_allocator *a, u64 len)
{
	return a->ops->alloc(a, len);
}

u64 nvgpu_alloc_pte(struct nvgpu_allocator *a, u64 len, u32 page_size)
{
	return a->ops->alloc_pte(a, len, page_size);
}

void nvgpu_free(struct nvgpu_allocator *a, u64 addr)
{
	a->ops->free(a, addr);
}

u64 nvgpu_alloc_fixed(struct nvgpu_allocator *a, u64 base, u64 len,
		      u32 page_size)
{
	if (a->ops->alloc_fixed) {
		return a->ops->alloc_fixed(a, base, len, page_size);
	}

	return 0;
}

void nvgpu_free_fixed(struct nvgpu_allocator *a, u64 base, u64 len)
{
	/*
	 * If this operation is not defined for the allocator then just do
	 * nothing. The alternative would be to fall back on the regular
	 * free but that may be harmful in unexpected ways.
	 */
	if (a->ops->free_fixed) {
		a->ops->free_fixed(a, base, len);
	}
}

int nvgpu_alloc_reserve_carveout(struct nvgpu_allocator *a,
				 struct nvgpu_alloc_carveout *co)
{
	if (a->ops->reserve_carveout) {
		return a->ops->reserve_carveout(a, co);
	}

	return -ENODEV;
}

void nvgpu_alloc_release_carveout(struct nvgpu_allocator *a,
				  struct nvgpu_alloc_carveout *co)
{
	if (a->ops->release_carveout) {
		a->ops->release_carveout(a, co);
	}
}

void nvgpu_alloc_destroy(struct nvgpu_allocator *a)
{
	a->ops->fini(a);
	nvgpu_mutex_destroy(&a->lock);
	memset(a, 0, sizeof(*a));
}

#ifdef __KERNEL__
void nvgpu_alloc_print_stats(struct nvgpu_allocator *na,
			     struct seq_file *s, int lock)
{
	na->ops->print_stats(na, s, lock);
}
#endif

/*
 * Handle the common init stuff for a nvgpu_allocator.
 */
int nvgpu_alloc_common_init(struct nvgpu_allocator *a, struct gk20a *g,
			    const char *name, void *priv, bool dbg,
			    const struct nvgpu_allocator_ops *ops)
{
	int err;

	if (ops == NULL) {
		return -EINVAL;
	}

	/*
	 * This is the bare minimum operations required for a sensible
	 * allocator.
	 */
	if (ops->alloc == NULL || ops->free == NULL || ops->fini == NULL) {
		return -EINVAL;
	}

	err = nvgpu_mutex_init(&a->lock);
	if (err) {
		return err;
	}

	a->g = g;
	a->ops = ops;
	a->priv = priv;
	a->debug = dbg;

	strncpy(a->name, name, sizeof(a->name));
	a->name[sizeof(a->name) - 1U] = '\0';

	return 0;
}
