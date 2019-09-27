/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bug.h>
#include <nvgpu/bitops.h>
#include <nvgpu/comptags.h>
#include <nvgpu/gk20a.h>

int gk20a_comptaglines_alloc(struct gk20a_comptag_allocator *allocator,
			     u32 *offset, u32 len)
{
	unsigned long addr;
	int err = 0;

	if (allocator->size == 0UL) {
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&allocator->lock);
	addr = bitmap_find_next_zero_area(allocator->bitmap, allocator->size,
			0, len, 0);
	if (addr < allocator->size) {
		/* number zero is reserved; bitmap base is 1 */
		*offset = 1U + addr;
		bitmap_set(allocator->bitmap, addr, len);
	} else {
		err = -ENOMEM;
	}
	nvgpu_mutex_release(&allocator->lock);

	return err;
}

void gk20a_comptaglines_free(struct gk20a_comptag_allocator *allocator,
			     u32 offset, u32 len)
{
	/* number zero is reserved; bitmap base is 1 */
	u32 addr = offset - 1U;

	if (allocator->size == 0UL) {
		return;
	}

	WARN_ON(offset == 0U);
	WARN_ON(addr > allocator->size);
	WARN_ON(addr + len > allocator->size);

	nvgpu_mutex_acquire(&allocator->lock);
	bitmap_clear(allocator->bitmap, addr, len);
	nvgpu_mutex_release(&allocator->lock);
}

int gk20a_comptag_allocator_init(struct gk20a *g,
				 struct gk20a_comptag_allocator *allocator,
				 unsigned long size)
{
	int err = nvgpu_mutex_init(&allocator->lock);

	if (err != 0) {
		nvgpu_err(g, "Error in allocator.lock mutex initialization");
		return err;
	}

	/*
	 * 0th comptag is special and is never used. The base for this bitmap
	 * is 1, and its size is one less than the size of comptag store.
	 */
	size--;
	allocator->bitmap = nvgpu_vzalloc(g,
					  BITS_TO_LONGS(size) * sizeof(long));
	if (allocator->bitmap == NULL)
		return -ENOMEM;

	allocator->size = size;

	return 0;
}

void gk20a_comptag_allocator_destroy(struct gk20a *g,
				     struct gk20a_comptag_allocator *allocator)
{
	/*
	 * called only when exiting the driver (gk20a_remove, or unwinding the
	 * init stage); no users should be active, so taking the mutex is
	 * unnecessary here.
	 */
	allocator->size = 0;
	nvgpu_vfree(g, allocator->bitmap);
}
