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

#include <nvgpu/pramin.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/enabled.h>
#include <nvgpu/sizes.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

/*
 * This typedef is for functions that get called during the access_batched()
 * operation.
 */
typedef void (*pramin_access_batch_fn)(struct gk20a *g, u32 start, u32 words,
				       u32 **arg);

/*
 * The PRAMIN range is 1 MB, must change base addr if a buffer crosses that.
 * This same loop is used for read/write/memset. Offset and size in bytes.
 * One call to "loop" is done per range, with "arg" supplied.
 */
static void nvgpu_pramin_access_batched(struct gk20a *g, struct nvgpu_mem *mem,
		u32 offset, u32 size, pramin_access_batch_fn loop, u32 **arg)
{
	struct nvgpu_page_alloc *alloc = NULL;
	struct nvgpu_sgt *sgt;
	struct nvgpu_sgl *sgl;
	u32 byteoff, start_reg, until_end, n;

	/*
	 * TODO: Vidmem is not accesible through pramin on shutdown path.
	 * driver should be refactored to prevent this from happening, but for
	 * now it is ok just to ignore the writes
	 */
	if (!gk20a_io_exists(g) && nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING)) {
		return;
	}

	alloc = mem->vidmem_alloc;
	sgt = &alloc->sgt;

	nvgpu_sgt_for_each_sgl(sgl, sgt) {
		if (offset >= nvgpu_sgt_get_length(sgt, sgl)) {
			offset -= nvgpu_sgt_get_length(sgt, sgl);
		} else {
			break;
		}
	}

	while (size) {
		u32 sgl_len = (u32)nvgpu_sgt_get_length(sgt, sgl);

		nvgpu_spinlock_acquire(&g->mm.pramin_window_lock);
		byteoff = g->ops.bus.set_bar0_window(g, mem, sgt, sgl,
					      offset / sizeof(u32));
		start_reg = g->ops.pramin.data032_r(byteoff / sizeof(u32));
		until_end = SZ_1M - (byteoff & (SZ_1M - 1));

		n = min3(size, until_end, (u32)(sgl_len - offset));

		loop(g, start_reg, n / sizeof(u32), arg);

		/* read back to synchronize accesses */
		(void) gk20a_readl(g, start_reg);

		nvgpu_spinlock_release(&g->mm.pramin_window_lock);

		size -= n;

		if (n == (sgl_len - offset)) {
			sgl = nvgpu_sgt_get_next(sgt, sgl);
			offset = 0;
		} else {
			offset += n;
		}
	}
}

static void nvgpu_pramin_access_batch_rd_n(struct gk20a *g,
					   u32 start, u32 words, u32 **arg)
{
	u32 r = start, *dest_u32 = *arg;

	while (words--) {
		*dest_u32++ = nvgpu_readl(g, r);
		r += sizeof(u32);
	}

	*arg = dest_u32;
}

void nvgpu_pramin_rd_n(struct gk20a *g, struct nvgpu_mem *mem,
					   u32 start, u32 size, void *dest)
{
	u32 *dest_u32 = dest;

	return nvgpu_pramin_access_batched(g, mem, start, size,
			nvgpu_pramin_access_batch_rd_n, &dest_u32);
}

static void nvgpu_pramin_access_batch_wr_n(struct gk20a *g,
					   u32 start, u32 words, u32 **arg)
{
	u32 r = start, *src_u32 = *arg;

	while (words--) {
		nvgpu_writel_relaxed(g, r, *src_u32++);
		r += sizeof(u32);
	}

	*arg = src_u32;
}

void nvgpu_pramin_wr_n(struct gk20a *g, struct nvgpu_mem *mem,
					   u32 start, u32 size, void *src)
{
	u32 *src_u32 = src;

	return nvgpu_pramin_access_batched(g, mem, start, size,
			nvgpu_pramin_access_batch_wr_n, &src_u32);
}

static void nvgpu_pramin_access_batch_set(struct gk20a *g,
					  u32 start, u32 words, u32 **arg)
{
	u32 r = start, repeat = **arg;

	while (words--) {
		nvgpu_writel_relaxed(g, r, repeat);
		r += sizeof(u32);
	}
}

void nvgpu_pramin_memset(struct gk20a *g, struct nvgpu_mem *mem,
			 u32 start, u32 size, u32 w)
{
	u32 *p = &w;

	return nvgpu_pramin_access_batched(g, mem, start, size,
			nvgpu_pramin_access_batch_set, &p);
}
void nvgpu_init_pramin(struct mm_gk20a *mm)
{
	mm->pramin_window = 0;
	nvgpu_spinlock_init(&mm->pramin_window_lock);
}
