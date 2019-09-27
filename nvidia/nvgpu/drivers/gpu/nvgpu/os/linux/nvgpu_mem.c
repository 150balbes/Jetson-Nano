/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/kmem.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/linux/dma.h>

#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include "os_linux.h"
#include "dmabuf_vidmem.h"

#include "gk20a/mm_gk20a.h"
#include "platform_gk20a.h"

static u64 __nvgpu_sgl_phys(struct gk20a *g, struct nvgpu_sgl *sgl)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	u64 ipa = sg_phys((struct scatterlist *)sgl);

	if (platform->phys_addr)
		return platform->phys_addr(g, ipa);

	return ipa;
}

/*
 * Obtain a SYSMEM address from a Linux SGL. This should eventually go away
 * and/or become private to this file once all bad usages of Linux SGLs are
 * cleaned up in the driver.
 */
u64 nvgpu_mem_get_addr_sgl(struct gk20a *g, struct scatterlist *sgl)
{
	if (nvgpu_is_enabled(g, NVGPU_MM_USE_PHYSICAL_SG) ||
	    !nvgpu_iommuable(g))
		return g->ops.mm.gpu_phys_addr(g, NULL,
			__nvgpu_sgl_phys(g, (struct nvgpu_sgl *)sgl));

	if (sg_dma_address(sgl) == 0)
		return g->ops.mm.gpu_phys_addr(g, NULL,
			__nvgpu_sgl_phys(g, (struct nvgpu_sgl *)sgl));

	if (sg_dma_address(sgl) == DMA_ERROR_CODE)
		return 0;

	return nvgpu_mem_iommu_translate(g, sg_dma_address(sgl));
}

/*
 * Obtain the address the GPU should use from the %mem assuming this is a SYSMEM
 * allocation.
 */
static u64 nvgpu_mem_get_addr_sysmem(struct gk20a *g, struct nvgpu_mem *mem)
{
	return nvgpu_mem_get_addr_sgl(g, mem->priv.sgt->sgl);
}

/*
 * Return the base address of %mem. Handles whether this is a VIDMEM or SYSMEM
 * allocation.
 *
 * Note: this API does not make sense to use for _VIDMEM_ buffers with greater
 * than one scatterlist chunk. If there's more than one scatterlist chunk then
 * the buffer will not be contiguous. As such the base address probably isn't
 * very useful. This is true for SYSMEM as well, if there's no IOMMU.
 *
 * However! It _is_ OK to use this on discontiguous sysmem buffers _if_ there's
 * an IOMMU present and enabled for the GPU.
 *
 * %attrs can be NULL. If it is not NULL then it may be inspected to determine
 * if the address needs to be modified before writing into a PTE.
 */
u64 nvgpu_mem_get_addr(struct gk20a *g, struct nvgpu_mem *mem)
{
	struct nvgpu_page_alloc *alloc;

	if (mem->aperture == APERTURE_SYSMEM)
		return nvgpu_mem_get_addr_sysmem(g, mem);

	/*
	 * Otherwise get the vidmem address.
	 */
	alloc = mem->vidmem_alloc;

	/* This API should not be used with > 1 chunks */
	WARN_ON(alloc->nr_chunks != 1);

	return alloc->base;
}

/*
 * This should only be used on contiguous buffers regardless of whether
 * there's an IOMMU present/enabled. This applies to both SYSMEM and
 * VIDMEM.
 */
u64 nvgpu_mem_get_phys_addr(struct gk20a *g, struct nvgpu_mem *mem)
{
	/*
	 * For a VIDMEM buf, this is identical to simply get_addr() so just fall
	 * back to that.
	 */
	if (mem->aperture == APERTURE_VIDMEM)
		return nvgpu_mem_get_addr(g, mem);

	return __nvgpu_sgl_phys(g, (struct nvgpu_sgl *)mem->priv.sgt->sgl);
}

/*
 * Be careful how you use this! You are responsible for correctly freeing this
 * memory.
 */
int nvgpu_mem_create_from_mem(struct gk20a *g,
			      struct nvgpu_mem *dest, struct nvgpu_mem *src,
			      u64 start_page, int nr_pages)
{
	int ret;
	u64 start = start_page * PAGE_SIZE;
	u64 size = nr_pages * PAGE_SIZE;
	dma_addr_t new_iova;

	if (src->aperture != APERTURE_SYSMEM)
		return -EINVAL;

	/* Some silly things a caller might do... */
	if (size > src->size)
		return -EINVAL;
	if ((start + size) > src->size)
		return -EINVAL;

	dest->mem_flags = src->mem_flags | NVGPU_MEM_FLAG_SHADOW_COPY;
	dest->aperture  = src->aperture;
	dest->skip_wmb  = src->skip_wmb;
	dest->size      = size;

	/*
	 * Re-use the CPU mapping only if the mapping was made by the DMA API.
	 *
	 * Bug 2040115: the DMA API wrapper makes the mapping that we should
	 * re-use.
	 */
	if (!(src->priv.flags & NVGPU_DMA_NO_KERNEL_MAPPING) ||
	    nvgpu_is_enabled(g, NVGPU_USE_COHERENT_SYSMEM))
		dest->cpu_va = src->cpu_va + (PAGE_SIZE * start_page);

	dest->priv.pages = src->priv.pages + start_page;
	dest->priv.flags = src->priv.flags;

	new_iova = sg_dma_address(src->priv.sgt->sgl) ?
		sg_dma_address(src->priv.sgt->sgl) + start : 0;

	/*
	 * Make a new SG table that is based only on the subset of pages that
	 * is passed to us. This table gets freed by the dma free routines.
	 */
	if (src->priv.flags & NVGPU_DMA_NO_KERNEL_MAPPING)
		ret = nvgpu_get_sgtable_from_pages(g, &dest->priv.sgt,
						   src->priv.pages + start_page,
						   new_iova, size);
	else
		ret = nvgpu_get_sgtable(g, &dest->priv.sgt, dest->cpu_va,
					new_iova, size);

	return ret;
}

int __nvgpu_mem_create_from_pages(struct gk20a *g, struct nvgpu_mem *dest,
				  struct page **pages, int nr_pages)
{
	struct sg_table *sgt;
	struct page **our_pages =
		nvgpu_kmalloc(g, sizeof(struct page *) * nr_pages);

	if (!our_pages)
		return -ENOMEM;

	memcpy(our_pages, pages, sizeof(struct page *) * nr_pages);

	if (nvgpu_get_sgtable_from_pages(g, &sgt, pages, 0,
					 nr_pages * PAGE_SIZE)) {
		nvgpu_kfree(g, our_pages);
		return -ENOMEM;
	}

	/*
	 * If we are making an SGT from physical pages we can be reasonably
	 * certain that this should bypass the SMMU - thus we set the DMA (aka
	 * IOVA) address to 0. This tells the GMMU mapping code to not make a
	 * mapping directed to the SMMU.
	 */
	sg_dma_address(sgt->sgl) = 0;

	dest->mem_flags  = __NVGPU_MEM_FLAG_NO_DMA;
	dest->aperture   = APERTURE_SYSMEM;
	dest->skip_wmb   = 0;
	dest->size       = PAGE_SIZE * nr_pages;

	dest->priv.flags = 0;
	dest->priv.pages = our_pages;
	dest->priv.sgt   = sgt;

	return 0;
}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
int __nvgpu_mem_create_from_phys(struct gk20a *g, struct nvgpu_mem *dest,
				 u64 src_phys, int nr_pages)
{
	struct page **pages =
		nvgpu_kmalloc(g, sizeof(struct page *) * nr_pages);
	int i, ret = 0;

	if (!pages)
		return -ENOMEM;

	for (i = 0; i < nr_pages; i++)
		pages[i] = phys_to_page(src_phys + PAGE_SIZE * i);

	ret = __nvgpu_mem_create_from_pages(g, dest, pages, nr_pages);
	nvgpu_kfree(g, pages);

	return ret;
}
#endif

static struct nvgpu_sgl *nvgpu_mem_linux_sgl_next(struct nvgpu_sgl *sgl)
{
	return (struct nvgpu_sgl *)sg_next((struct scatterlist *)sgl);
}

static u64 nvgpu_mem_linux_sgl_phys(struct gk20a *g, struct nvgpu_sgl *sgl)
{
	return (u64)__nvgpu_sgl_phys(g, sgl);
}

static u64 nvgpu_mem_linux_sgl_dma(struct nvgpu_sgl *sgl)
{
	return (u64)sg_dma_address((struct scatterlist *)sgl);
}

static u64 nvgpu_mem_linux_sgl_length(struct nvgpu_sgl *sgl)
{
	return (u64)((struct scatterlist *)sgl)->length;
}

static u64 nvgpu_mem_linux_sgl_gpu_addr(struct gk20a *g,
					struct nvgpu_sgl *sgl,
					struct nvgpu_gmmu_attrs *attrs)
{
	if (sg_dma_address((struct scatterlist *)sgl) == 0)
		return g->ops.mm.gpu_phys_addr(g, attrs,
				__nvgpu_sgl_phys(g, sgl));

	if (sg_dma_address((struct scatterlist *)sgl) == DMA_ERROR_CODE)
		return 0;

	return nvgpu_mem_iommu_translate(g,
				sg_dma_address((struct scatterlist *)sgl));
}

static bool nvgpu_mem_linux_sgt_iommuable(struct gk20a *g,
					  struct nvgpu_sgt *sgt)
{
	if (nvgpu_is_enabled(g, NVGPU_MM_USE_PHYSICAL_SG))
		return false;
	return true;
}

static void nvgpu_mem_linux_sgl_free(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	/*
	 * Free this SGT. All we do is free the passed SGT. The actual Linux
	 * SGT/SGL needs to be freed separately.
	 */
	nvgpu_kfree(g, sgt);
}

static const struct nvgpu_sgt_ops nvgpu_linux_sgt_ops = {
	.sgl_next      = nvgpu_mem_linux_sgl_next,
	.sgl_phys      = nvgpu_mem_linux_sgl_phys,
	.sgl_dma       = nvgpu_mem_linux_sgl_dma,
	.sgl_length    = nvgpu_mem_linux_sgl_length,
	.sgl_gpu_addr  = nvgpu_mem_linux_sgl_gpu_addr,
	.sgt_iommuable = nvgpu_mem_linux_sgt_iommuable,
	.sgt_free      = nvgpu_mem_linux_sgl_free,
};

static struct nvgpu_sgt *__nvgpu_mem_get_sgl_from_vidmem(
	struct gk20a *g,
	struct scatterlist *linux_sgl)
{
	struct nvgpu_page_alloc *vidmem_alloc;

	vidmem_alloc = nvgpu_vidmem_get_page_alloc(linux_sgl);
	if (!vidmem_alloc)
		return NULL;

	return &vidmem_alloc->sgt;
}

struct nvgpu_sgt *nvgpu_linux_sgt_create(struct gk20a *g, struct sg_table *sgt)
{
	struct nvgpu_sgt *nvgpu_sgt;
	struct scatterlist *linux_sgl = sgt->sgl;

	if (nvgpu_addr_is_vidmem_page_alloc(sg_dma_address(linux_sgl)))
		return __nvgpu_mem_get_sgl_from_vidmem(g, linux_sgl);

	nvgpu_sgt = nvgpu_kzalloc(g, sizeof(*nvgpu_sgt));
	if (!nvgpu_sgt)
		return NULL;

	nvgpu_log(g, gpu_dbg_sgl, "Making Linux SGL!");

	nvgpu_sgt->sgl = (struct nvgpu_sgl *)linux_sgl;
	nvgpu_sgt->ops = &nvgpu_linux_sgt_ops;

	return nvgpu_sgt;
}

struct nvgpu_sgt *nvgpu_sgt_create_from_mem(struct gk20a *g,
					    struct nvgpu_mem *mem)
{
	return nvgpu_linux_sgt_create(g, mem->priv.sgt);
}
