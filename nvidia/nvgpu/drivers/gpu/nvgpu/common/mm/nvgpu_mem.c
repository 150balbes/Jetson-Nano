/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/dma.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/gk20a.h>
/*
 * Make sure to use the right coherency aperture if you use this function! This
 * will not add any checks. If you want to simply use the default coherency then
 * use nvgpu_aperture_mask().
 */
u32 nvgpu_aperture_mask_raw(struct gk20a *g, enum nvgpu_aperture aperture,
			    u32 sysmem_mask, u32 sysmem_coh_mask,
			    u32 vidmem_mask)
{
	/*
	 * Some iGPUs treat sysmem (i.e SoC DRAM) as vidmem. In these cases the
	 * "sysmem" aperture should really be translated to VIDMEM.
	 */
	if (!nvgpu_is_enabled(g, NVGPU_MM_HONORS_APERTURE)) {
		aperture = APERTURE_VIDMEM;
	}

	switch (aperture) {
	case APERTURE_SYSMEM_COH:
		return sysmem_coh_mask;
	case APERTURE_SYSMEM:
		return sysmem_mask;
	case APERTURE_VIDMEM:
		return vidmem_mask;
	case APERTURE_INVALID:
		WARN_ON("Bad aperture");
	}
	return 0;
}

u32 nvgpu_aperture_mask(struct gk20a *g, struct nvgpu_mem *mem,
			u32 sysmem_mask, u32 sysmem_coh_mask, u32 vidmem_mask)
{
	enum nvgpu_aperture ap = mem->aperture;

	/*
	 * Handle the coherent aperture: ideally most of the driver is not
	 * aware of the difference between coherent and non-coherent sysmem so
	 * we add this translation step here.
	 */
	if (nvgpu_is_enabled(g, NVGPU_USE_COHERENT_SYSMEM) &&
	    ap == APERTURE_SYSMEM) {
		ap = APERTURE_SYSMEM_COH;
	}

	return nvgpu_aperture_mask_raw(g, ap,
				       sysmem_mask,
				       sysmem_coh_mask,
				       vidmem_mask);
}

bool nvgpu_aperture_is_sysmem(enum nvgpu_aperture ap)
{
	return ap == APERTURE_SYSMEM_COH || ap == APERTURE_SYSMEM;
}

bool nvgpu_mem_is_sysmem(struct nvgpu_mem *mem)
{
	return nvgpu_aperture_is_sysmem(mem->aperture);
}

struct nvgpu_sgl *nvgpu_sgt_get_next(struct nvgpu_sgt *sgt,
				     struct nvgpu_sgl *sgl)
{
	return sgt->ops->sgl_next(sgl);
}

u64 nvgpu_sgt_get_phys(struct gk20a *g, struct nvgpu_sgt *sgt,
		       struct nvgpu_sgl *sgl)
{
	return sgt->ops->sgl_phys(g, sgl);
}

u64 nvgpu_sgt_get_dma(struct nvgpu_sgt *sgt, struct nvgpu_sgl *sgl)
{
	return sgt->ops->sgl_dma(sgl);
}

u64 nvgpu_sgt_get_length(struct nvgpu_sgt *sgt, struct nvgpu_sgl *sgl)
{
	return sgt->ops->sgl_length(sgl);
}

u64 nvgpu_sgt_get_gpu_addr(struct gk20a *g, struct nvgpu_sgt *sgt,
			   struct nvgpu_sgl *sgl,
			   struct nvgpu_gmmu_attrs *attrs)
{
	return sgt->ops->sgl_gpu_addr(g, sgl, attrs);
}

bool nvgpu_sgt_iommuable(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	if (sgt->ops->sgt_iommuable) {
		return sgt->ops->sgt_iommuable(g, sgt);
	}
	return false;
}

void nvgpu_sgt_free(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	if (sgt != NULL && sgt->ops->sgt_free != NULL) {
		sgt->ops->sgt_free(g, sgt);
	}
}

u64 nvgpu_mem_iommu_translate(struct gk20a *g, u64 phys)
{
	/* ensure it is not vidmem allocation */
	WARN_ON(nvgpu_addr_is_vidmem_page_alloc(phys));

	if (nvgpu_iommuable(g) && g->ops.mm.get_iommu_bit != NULL) {
		return phys | 1ULL << g->ops.mm.get_iommu_bit(g);
	}

	return phys;
}

/*
 * Determine alignment for a passed buffer. Necessary since the buffer may
 * appear big enough to map with large pages but the SGL may have chunks that
 * are not aligned on a 64/128kB large page boundary. There's also the
 * possibility chunks are odd sizes which will necessitate small page mappings
 * to correctly glue them together into a contiguous virtual mapping.
 */
u64 nvgpu_sgt_alignment(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	u64 align = 0, chunk_align = 0;
	struct nvgpu_sgl *sgl;

	/*
	 * If this SGT is iommuable and we want to use the IOMMU address then
	 * the SGT's first entry has the IOMMU address. We will align on this
	 * and double check length of buffer later. Also, since there's an
	 * IOMMU we know that this DMA address is contiguous.
	 */
	if (nvgpu_iommuable(g) &&
	    nvgpu_sgt_iommuable(g, sgt) &&
	    nvgpu_sgt_get_dma(sgt, sgt->sgl) != 0ULL) {
		return 1ULL << __ffs(nvgpu_sgt_get_dma(sgt, sgt->sgl));
	}

	/*
	 * Otherwise the buffer is not iommuable (VIDMEM, for example) or we are
	 * bypassing the IOMMU and need to use the underlying physical entries
	 * of the SGT.
	 */
	nvgpu_sgt_for_each_sgl(sgl, sgt) {
		chunk_align = 1ULL << __ffs(nvgpu_sgt_get_phys(g, sgt, sgl) |
					    nvgpu_sgt_get_length(sgt, sgl));

		if (align) {
			align = min(align, chunk_align);
		} else {
			align = chunk_align;
		}
	}

	return align;
}

u32 nvgpu_mem_rd32(struct gk20a *g, struct nvgpu_mem *mem, u32 w)
{
	u32 data = 0;

	if (mem->aperture == APERTURE_SYSMEM) {
		u32 *ptr = mem->cpu_va;

		WARN_ON(ptr == NULL);
		data = ptr[w];
	} else if (mem->aperture == APERTURE_VIDMEM) {
		nvgpu_pramin_rd_n(g, mem, w * sizeof(u32), sizeof(u32), &data);
	} else {
		WARN_ON("Accessing unallocated nvgpu_mem");
	}

	return data;
}

u32 nvgpu_mem_rd(struct gk20a *g, struct nvgpu_mem *mem, u32 offset)
{
	WARN_ON((offset & 3U) != 0U);
	return nvgpu_mem_rd32(g, mem, offset / sizeof(u32));
}

void nvgpu_mem_rd_n(struct gk20a *g, struct nvgpu_mem *mem,
		u32 offset, void *dest, u32 size)
{
	WARN_ON((offset & 3U) != 0U);
	WARN_ON((size & 3U) != 0U);

	if (mem->aperture == APERTURE_SYSMEM) {
		u8 *src = (u8 *)mem->cpu_va + offset;

		WARN_ON(mem->cpu_va == NULL);
		memcpy(dest, src, size);
	} else if (mem->aperture == APERTURE_VIDMEM) {
		nvgpu_pramin_rd_n(g, mem, offset, size, dest);
	} else {
		WARN_ON("Accessing unallocated nvgpu_mem");
	}
}

void nvgpu_mem_wr32(struct gk20a *g, struct nvgpu_mem *mem, u32 w, u32 data)
{
	if (mem->aperture == APERTURE_SYSMEM) {
		u32 *ptr = mem->cpu_va;

		WARN_ON(ptr == NULL);
		ptr[w] = data;
	} else if (mem->aperture == APERTURE_VIDMEM) {
		nvgpu_pramin_wr_n(g, mem, w * sizeof(u32), sizeof(u32), &data);
		if (!mem->skip_wmb) {
			nvgpu_wmb();
		}
	} else {
		WARN_ON("Accessing unallocated nvgpu_mem");
	}
}

void nvgpu_mem_wr(struct gk20a *g, struct nvgpu_mem *mem, u32 offset, u32 data)
{
	WARN_ON((offset & 3U) != 0U);
	nvgpu_mem_wr32(g, mem, offset / sizeof(u32), data);
}

void nvgpu_mem_wr_n(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		void *src, u32 size)
{
	WARN_ON((offset & 3U) != 0U);
	WARN_ON((size & 3U) != 0U);

	if (mem->aperture == APERTURE_SYSMEM) {
		u8 *dest = (u8 *)mem->cpu_va + offset;

		WARN_ON(mem->cpu_va == NULL);
		memcpy(dest, src, size);
	} else if (mem->aperture == APERTURE_VIDMEM) {
		nvgpu_pramin_wr_n(g, mem, offset, size, src);
		if (!mem->skip_wmb) {
			nvgpu_wmb();
		}
	} else {
		WARN_ON("Accessing unallocated nvgpu_mem");
	}
}

void nvgpu_memset(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		u32 c, u32 size)
{
	WARN_ON((offset & 3U) != 0U);
	WARN_ON((size & 3U) != 0U);
	WARN_ON((c & ~0xffU) != 0U);

	c &= 0xffU;

	if (mem->aperture == APERTURE_SYSMEM) {
		u8 *dest = (u8 *)mem->cpu_va + offset;

		WARN_ON(mem->cpu_va == NULL);
		memset(dest, c, size);
	} else if (mem->aperture == APERTURE_VIDMEM) {
		u32 repeat_value = c | (c << 8) | (c << 16) | (c << 24);

		nvgpu_pramin_memset(g, mem, offset, size, repeat_value);
		if (!mem->skip_wmb) {
			nvgpu_wmb();
		}
	} else {
		WARN_ON("Accessing unallocated nvgpu_mem");
	}
}
