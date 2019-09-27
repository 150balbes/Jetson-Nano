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

#include <nvgpu/bug.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>

/*
 * These functions are somewhat meaningless.
 */
u64 nvgpu_mem_get_addr(struct gk20a *g, struct nvgpu_mem *mem)
{
	return (u64)(uintptr_t)mem->cpu_va;
}

u64 nvgpu_mem_get_phys_addr(struct gk20a *g, struct nvgpu_mem *mem)
{
	return (u64)(uintptr_t)mem->cpu_va;
}

static struct nvgpu_sgl *nvgpu_mem_sgl_next(struct nvgpu_sgl *sgl)
{
	return NULL;
}

static u64 nvgpu_mem_sgl_phys(struct gk20a *g, struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem *mem = (struct nvgpu_mem *)sgl;

	return (u64)(uintptr_t)mem->cpu_va;
}

static u64 nvgpu_mem_sgl_dma(struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem *mem = (struct nvgpu_mem *)sgl;

	return (u64)(uintptr_t)mem->cpu_va;
}

static u64 nvgpu_mem_sgl_length(struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem *mem = (struct nvgpu_mem *)sgl;

	return (u64)mem->aligned_size;
}

static u64 nvgpu_mem_sgl_gpu_addr(struct gk20a *g, struct nvgpu_sgl *sgl,
				  struct nvgpu_gmmu_attrs *attrs)
{
	struct nvgpu_mem *mem = (struct nvgpu_mem *)sgl;

	return mem->gpu_va;
}

static bool nvgpu_mem_sgt_iommuable(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	return nvgpu_iommuable(g);
}

static void nvgpu_mem_sgt_free(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	nvgpu_kfree(g, sgt);
}

static struct nvgpu_sgt_ops nvgpu_sgt_posix_ops = {
	.sgl_next	= nvgpu_mem_sgl_next,
	.sgl_phys	= nvgpu_mem_sgl_phys,
	.sgl_dma	= nvgpu_mem_sgl_dma,
	.sgl_length	= nvgpu_mem_sgl_length,
	.sgl_gpu_addr	= nvgpu_mem_sgl_gpu_addr,
	.sgt_iommuable	= nvgpu_mem_sgt_iommuable,
	.sgt_free	= nvgpu_mem_sgt_free,
};

struct nvgpu_sgt *nvgpu_sgt_create_from_mem(struct gk20a *g,
					    struct nvgpu_mem *mem)
{
	struct nvgpu_sgt *sgt = nvgpu_kzalloc(g, sizeof(*sgt));

	if (sgt == NULL)
		return NULL;

	/*
	 * The userspace implementation is simple: a single 'entry' (which we
	 * only need the mem struct to describe). Maybe this could be expanded
	 * to be more interesting some day.
	 */
	sgt->sgl = (struct nvgpu_sgl *)mem;
	sgt->ops = &nvgpu_sgt_posix_ops;

	return sgt;
}

int nvgpu_mem_create_from_mem(struct gk20a *g,
			      struct nvgpu_mem *dest, struct nvgpu_mem *src,
			      u64 start_page, int nr_pages)
{
	u64 start = start_page * PAGE_SIZE;
	u64 size = nr_pages * PAGE_SIZE;

	if (src->aperture != APERTURE_SYSMEM)
		return -EINVAL;

	/* Some silly things a caller might do... */
	if (size > src->size)
		return -EINVAL;
	if ((start + size) > src->size)
		return -EINVAL;

	memset(dest, 0, sizeof(*dest));

	dest->cpu_va    = ((char *)src->cpu_va) + start;
	dest->mem_flags = src->mem_flags | NVGPU_MEM_FLAG_SHADOW_COPY;
	dest->aperture  = src->aperture;
	dest->skip_wmb  = src->skip_wmb;
	dest->size      = size;

	return 0;
}
