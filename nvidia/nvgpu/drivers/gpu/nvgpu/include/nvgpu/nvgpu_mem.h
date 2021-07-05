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

#ifndef NVGPU_MEM_H
#define NVGPU_MEM_H

#include <nvgpu/types.h>
#include <nvgpu/list.h>
#include <nvgpu/enabled.h>

#ifdef __KERNEL__
#include <nvgpu/linux/nvgpu_mem.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/nvgpu_mem.h>
#else
#include <nvgpu_rmos/include/nvgpu_mem.h>
#endif

struct page;
struct sg_table;
struct nvgpu_sgt;

struct gk20a;
struct nvgpu_allocator;
struct nvgpu_gmmu_attrs;
struct nvgpu_page_alloc;

#define NVGPU_MEM_DMA_ERROR		(~0ULL)

/*
 * Real location of a buffer - nvgpu_aperture_mask() will deduce what will be
 * told to the gpu about the aperture, but this flag designates where the
 * memory actually was allocated from.
 */
enum nvgpu_aperture {
	APERTURE_INVALID = 0, /* unallocated or N/A */
	APERTURE_SYSMEM,

	 /* Don't use directly. Use APERTURE_SYSMEM, this is used internally. */
	APERTURE_SYSMEM_COH,

	APERTURE_VIDMEM
};

/*
 * Forward declared opaque placeholder type that does not really exist, but
 * helps the compiler help us about getting types right. In reality,
 * implementors of nvgpu_sgt_ops will have some concrete type in place of this.
 */
struct nvgpu_sgl;

struct nvgpu_sgt_ops {
	struct nvgpu_sgl *(*sgl_next)(struct nvgpu_sgl *sgl);
	u64   (*sgl_phys)(struct gk20a *g, struct nvgpu_sgl *sgl);
	u64   (*sgl_dma)(struct nvgpu_sgl *sgl);
	u64   (*sgl_length)(struct nvgpu_sgl *sgl);
	u64   (*sgl_gpu_addr)(struct gk20a *g, struct nvgpu_sgl *sgl,
			      struct nvgpu_gmmu_attrs *attrs);
	/*
	 * If left NULL then iommuable is assumed to be false.
	 */
	bool  (*sgt_iommuable)(struct gk20a *g, struct nvgpu_sgt *sgt);

	/*
	 * Note: this operates on the whole SGT not a specific SGL entry.
	 */
	void  (*sgt_free)(struct gk20a *g, struct nvgpu_sgt *sgt);
};

/*
 * Scatter gather table: this is a list of scatter list entries and the ops for
 * interacting with those entries.
 */
struct nvgpu_sgt {
	/*
	 * Ops for interacting with the underlying scatter gather list entries.
	 */
	const struct nvgpu_sgt_ops *ops;

	/*
	 * The first node in the scatter gather list.
	 */
	struct nvgpu_sgl *sgl;
};

/*
 * This struct holds the necessary information for describing a struct
 * nvgpu_mem's scatter gather list.
 *
 * This is one underlying implementation for nvgpu_sgl. Not all nvgpu_sgt's use
 * this particular implementation. Nor is a given OS required to use this at
 * all.
 */
struct nvgpu_mem_sgl {
	/*
	 * Internally this is implemented as a singly linked list.
	 */
	struct nvgpu_mem_sgl	*next;

	/*
	 * There is both a phys address and a DMA address since some systems,
	 * for example ones with an IOMMU, may see these as different addresses.
	 */
	u64			 phys;
	u64			 dma;
	u64			 length;
};

/*
 * Iterate over the SGL entries in an SGT.
 */
#define nvgpu_sgt_for_each_sgl(__sgl__, __sgt__)		\
	for ((__sgl__) = (__sgt__)->sgl;			\
	     (__sgl__) != NULL;					\
	     (__sgl__) = nvgpu_sgt_get_next(__sgt__, __sgl__))

struct nvgpu_mem {
	/*
	 * Populated for all nvgpu_mem structs - vidmem or system.
	 */
	enum nvgpu_aperture			 aperture;
	size_t					 size;
	size_t					 aligned_size;
	u64					 gpu_va;
	bool					 skip_wmb;
	bool					 free_gpu_va;

	/*
	 * Set when a nvgpu_mem struct is not a "real" nvgpu_mem struct. Instead
	 * the struct is just a copy of another nvgpu_mem struct.
	 */
#define NVGPU_MEM_FLAG_SHADOW_COPY		 (1 << 0)

	/*
	 * Specify that the GVA mapping is a fixed mapping - that is the caller
	 * chose the GPU VA, not the GMMU mapping function. Only relevant for
	 * VIDMEM.
	 */
#define NVGPU_MEM_FLAG_FIXED			 (1 << 1)

	/*
	 * Set for user generated VIDMEM allocations. This triggers a special
	 * cleanup path that clears the vidmem on free. Given that the VIDMEM is
	 * zeroed on boot this means that all user vidmem allocations are
	 * therefor zeroed (to prevent leaking information in VIDMEM buffers).
	 */
#define NVGPU_MEM_FLAG_USER_MEM			 (1 << 2)

	/*
	 * Internal flag that specifies this struct has not been made with DMA
	 * memory and as a result should not try to use the DMA routines for
	 * freeing the backing memory.
	 *
	 * However, this will not stop the DMA API from freeing other parts of
	 * nvgpu_mem in a system specific way.
	 */
#define __NVGPU_MEM_FLAG_NO_DMA			 (1 << 3)
	/*
	 * Some nvgpu_mem objects act as facades to memory buffers owned by
	 * someone else. This internal flag specifies that the sgt field is
	 * "borrowed", and it must not be freed by us.
	 *
	 * Of course the caller will have to make sure that the sgt owner
	 * outlives the nvgpu_mem.
	 */
#define NVGPU_MEM_FLAG_FOREIGN_SGT		 (1 << 4)
	unsigned long				 mem_flags;

	/*
	 * Only populated for a sysmem allocation.
	 */
	void					*cpu_va;

	/*
	 * Fields only populated for vidmem allocations.
	 */
	struct nvgpu_page_alloc			*vidmem_alloc;
	struct nvgpu_allocator			*allocator;
	struct nvgpu_list_node			 clear_list_entry;

	/*
	 * This is defined by the system specific header. It can be empty if
	 * there's no system specific stuff for a given system.
	 */
	struct nvgpu_mem_priv			 priv;
};

static inline struct nvgpu_mem *
nvgpu_mem_from_clear_list_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_mem *)
		((uintptr_t)node - offsetof(struct nvgpu_mem,
					    clear_list_entry));
};

static inline const char *nvgpu_aperture_str(struct gk20a *g,
					     enum nvgpu_aperture aperture)
{
	switch (aperture) {
	case APERTURE_INVALID:
		return "INVAL";
	case APERTURE_SYSMEM:
		return "SYSMEM";
	case APERTURE_SYSMEM_COH:
		return "SYSCOH";
	case APERTURE_VIDMEM:
		return "VIDMEM";
	};
	return "UNKNOWN";
}

bool nvgpu_aperture_is_sysmem(enum nvgpu_aperture ap);
bool nvgpu_mem_is_sysmem(struct nvgpu_mem *mem);

/*
 * Returns true if the passed nvgpu_mem has been allocated (i.e it's valid for
 * subsequent use).
 */
static inline bool nvgpu_mem_is_valid(struct nvgpu_mem *mem)
{
	/*
	 * Internally the DMA APIs must set/unset the aperture flag when
	 * allocating/freeing the buffer. So check that to see if the *mem
	 * has been allocated or not.
	 *
	 * This relies on mem_descs being zeroed before being initialized since
	 * APERTURE_INVALID is equal to 0.
	 */
	return mem->aperture != APERTURE_INVALID;

}

/**
 * nvgpu_mem_sgt_create_from_mem - Create a scatter list from an nvgpu_mem.
 *
 * @g   - The GPU.
 * @mem - The source memory allocation to use.
 *
 * Create a scatter gather table from the passed @mem struct. This list lets the
 * calling code iterate across each chunk of a DMA allocation for when that DMA
 * allocation is not completely contiguous.
 */
struct nvgpu_sgt *nvgpu_sgt_create_from_mem(struct gk20a *g,
					    struct nvgpu_mem *mem);

struct nvgpu_sgl *nvgpu_sgt_get_next(struct nvgpu_sgt *sgt,
					  struct nvgpu_sgl *sgl);
u64 nvgpu_sgt_get_phys(struct gk20a *g, struct nvgpu_sgt *sgt,
		       struct nvgpu_sgl *sgl);
u64 nvgpu_sgt_get_dma(struct nvgpu_sgt *sgt, struct nvgpu_sgl *sgl);
u64 nvgpu_sgt_get_length(struct nvgpu_sgt *sgt, struct nvgpu_sgl *sgl);
u64 nvgpu_sgt_get_gpu_addr(struct gk20a *g, struct nvgpu_sgt *sgt,
			   struct nvgpu_sgl *sgl,
			   struct nvgpu_gmmu_attrs *attrs);
void nvgpu_sgt_free(struct gk20a *g, struct nvgpu_sgt *sgt);

bool nvgpu_sgt_iommuable(struct gk20a *g, struct nvgpu_sgt *sgt);
u64 nvgpu_sgt_alignment(struct gk20a *g, struct nvgpu_sgt *sgt);

/**
 * nvgpu_mem_create_from_mem - Create a new nvgpu_mem struct from an old one.
 *
 * @g          - The GPU.
 * @dest       - Destination nvgpu_mem to hold resulting memory description.
 * @src        - Source memory. Must be valid.
 * @start_page - Starting page to use.
 * @nr_pages   - Number of pages to place in the new nvgpu_mem.
 *
 * Create a new nvgpu_mem struct describing a subsection of the @src nvgpu_mem.
 * This will create an nvpgu_mem object starting at @start_page and is @nr_pages
 * long. This currently only works on SYSMEM nvgpu_mems. If this is called on a
 * VIDMEM nvgpu_mem then this will return an error.
 *
 * There is a _major_ caveat to this API: if the source buffer is freed before
 * the copy is freed then the copy will become invalid. This is a result from
 * how typical DMA APIs work: we can't call free on the buffer multiple times.
 * Nor can we call free on parts of a buffer. Thus the only way to ensure that
 * the entire buffer is actually freed is to call free once on the source
 * buffer. Since these nvgpu_mem structs are not ref-counted in anyway it is up
 * to the caller of this API to _ensure_ that the resulting nvgpu_mem buffer
 * from this API is freed before the source buffer. Otherwise there can and will
 * be memory corruption.
 *
 * The resulting nvgpu_mem should be released with the nvgpu_dma_free() or the
 * nvgpu_dma_unmap_free() function depending on whether or not the resulting
 * nvgpu_mem has been mapped.
 *
 * This will return 0 on success. An error is returned if the resulting
 * nvgpu_mem would not make sense or if a new scatter gather table cannot be
 * created.
 */
int nvgpu_mem_create_from_mem(struct gk20a *g,
			      struct nvgpu_mem *dest, struct nvgpu_mem *src,
			      u64 start_page, int nr_pages);

/*
 * Really free a vidmem buffer. There's a fair amount of work involved in
 * freeing vidmem buffers in the DMA API. This handles none of that - it only
 * frees the underlying vidmem specific structures used in vidmem buffers.
 *
 * This is implemented in the OS specific code. If it's not necessary it can
 * be a noop. But the symbol must at least be present.
 */
void __nvgpu_mem_free_vidmem_alloc(struct gk20a *g, struct nvgpu_mem *vidmem);

/*
 * Buffer accessors. Sysmem buffers always have a CPU mapping and vidmem
 * buffers are accessed via PRAMIN.
 */

/* word-indexed offset */
u32 nvgpu_mem_rd32(struct gk20a *g, struct nvgpu_mem *mem, u32 w);
/* byte offset (32b-aligned) */
u32 nvgpu_mem_rd(struct gk20a *g, struct nvgpu_mem *mem, u32 offset);
/* memcpy to cpu, offset and size in bytes (32b-aligned) */
void nvgpu_mem_rd_n(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		void *dest, u32 size);

/* word-indexed offset */
void nvgpu_mem_wr32(struct gk20a *g, struct nvgpu_mem *mem, u32 w, u32 data);
/* byte offset (32b-aligned) */
void nvgpu_mem_wr(struct gk20a *g, struct nvgpu_mem *mem, u32 offset, u32 data);
/* memcpy from cpu, offset and size in bytes (32b-aligned) */
void nvgpu_mem_wr_n(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		void *src, u32 size);
/* size and offset in bytes (32b-aligned), filled with the constant byte c */
void nvgpu_memset(struct gk20a *g, struct nvgpu_mem *mem, u32 offset,
		u32 c, u32 size);

u64 nvgpu_mem_get_addr(struct gk20a *g, struct nvgpu_mem *mem);
u64 nvgpu_mem_get_phys_addr(struct gk20a *g, struct nvgpu_mem *mem);

u32 nvgpu_aperture_mask_raw(struct gk20a *g, enum nvgpu_aperture aperture,
		u32 sysmem_mask, u32 sysmem_coh_mask, u32 vidmem_mask);
u32 nvgpu_aperture_mask(struct gk20a *g, struct nvgpu_mem *mem,
		u32 sysmem_mask, u32 sysmem_coh_mask, u32 vidmem_mask);

u64 nvgpu_mem_iommu_translate(struct gk20a *g, u64 phys);

#endif /* NVGPU_MEM_H */
