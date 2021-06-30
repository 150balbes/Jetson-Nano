/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <stdlib.h>

#include <nvgpu/mm.h>
#include <nvgpu/vm.h>
#include <nvgpu/bug.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/enabled.h>

/*
 * In userspace vidmem vs sysmem is just a difference in what is placed in the
 * aperture field.
 */
static int __nvgpu_do_dma_alloc(struct gk20a *g, unsigned long flags,
				size_t size, struct nvgpu_mem *mem,
				enum nvgpu_aperture ap)
{
	void *memory = malloc(mem->aligned_size);

	if (memory == NULL)
		return -ENOMEM;

	mem->cpu_va       = memory;
	mem->aperture     = ap;
	mem->size         = size;
	mem->aligned_size = PAGE_ALIGN(size);
	mem->gpu_va       = 0ULL;
	mem->skip_wmb     = true;
	mem->vidmem_alloc = NULL;
	mem->allocator    = NULL;

	return 0;
}

bool nvgpu_iommuable(struct gk20a *g)
{
	return false;
}

int nvgpu_dma_alloc_flags_sys(struct gk20a *g, unsigned long flags,
			      size_t size, struct nvgpu_mem *mem)
{
	return __nvgpu_do_dma_alloc(g, flags, size, mem, APERTURE_SYSMEM);
}

int nvgpu_dma_alloc_flags_vid_at(struct gk20a *g, unsigned long flags,
				 size_t size, struct nvgpu_mem *mem, u64 at)
{
	BUG();

	return 0;
}

void nvgpu_dma_free_sys(struct gk20a *g, struct nvgpu_mem *mem)
{
	if (!(mem->mem_flags & NVGPU_MEM_FLAG_SHADOW_COPY))
		free(mem->cpu_va);

	memset(mem, 0, sizeof(*mem));
}

void nvgpu_dma_free_vid(struct gk20a *g, struct nvgpu_mem *mem)
{
	BUG();
}
