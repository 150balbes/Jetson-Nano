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

#ifndef NVGPU_GMMU_H
#define NVGPU_GMMU_H

#include <nvgpu/types.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/lock.h>
#include <nvgpu/bitops.h>
#include <nvgpu/mm.h>

/*
 * This is the GMMU API visible to blocks outside of the GMMU. Basically this
 * API supports all the different types of mappings that might be done in the
 * GMMU.
 */

struct vm_gk20a;
struct nvgpu_mem;

#define GMMU_PAGE_SIZE_SMALL	0U
#define GMMU_PAGE_SIZE_BIG	1U
#define GMMU_PAGE_SIZE_KERNEL	2U
#define GMMU_NR_PAGE_SIZES	3U

enum gk20a_mem_rw_flag {
	gk20a_mem_flag_none = 0,	/* RW */
	gk20a_mem_flag_read_only = 1,	/* RO */
	gk20a_mem_flag_write_only = 2,	/* WO */
};

/*
 * Minimum size of a cache. The number of different caches in the nvgpu_pd_cache
 * structure is of course depending on this. The MIN_SHIFT define is the right
 * number of bits to shift to determine which list to use in the array of lists.
 *
 * For Linux, limit the use of the cache to entries less than the page size, to
 * avoid potential problems with running out of CMA memory when allocating large,
 * contiguous slabs, as would be required for non-iommmuable chips.
 */
#define NVGPU_PD_CACHE_MIN		256U
#define NVGPU_PD_CACHE_MIN_SHIFT	9U

#ifdef __KERNEL__

#if PAGE_SIZE == 4096
#define NVGPU_PD_CACHE_COUNT		4U
#elif PAGE_SIZE == 65536
#define NVGPU_PD_CACHE_COUNT		8U
#else
#error "Unsupported page size."
#endif

#else
#define NVGPU_PD_CACHE_COUNT		8U
#endif

#define NVGPU_PD_CACHE_SIZE		(NVGPU_PD_CACHE_MIN * (1U << NVGPU_PD_CACHE_COUNT))

struct nvgpu_pd_mem_entry {
	struct nvgpu_mem		mem;

	/*
	 * Size of the page directories (not the mem). alloc_map is a bitmap
	 * showing which PDs have been allocated.
	 *
	 * The size of mem will be NVGPU_PD_CACHE_SIZE
	 * and pd_size will always be a power of 2.
	 *
	 */
	u32				pd_size;
	DECLARE_BITMAP(alloc_map, NVGPU_PD_CACHE_SIZE / NVGPU_PD_CACHE_MIN);

	/* Total number of allocations in this PD. */
	u32				allocs;

	struct nvgpu_list_node		list_entry;
	struct nvgpu_rbtree_node	tree_entry;
};

static inline struct nvgpu_pd_mem_entry *
nvgpu_pd_mem_entry_from_list_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_pd_mem_entry *)
		((uintptr_t)node -
		 offsetof(struct nvgpu_pd_mem_entry, list_entry));
};

static inline struct nvgpu_pd_mem_entry *
nvgpu_pd_mem_entry_from_tree_entry(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_pd_mem_entry *)
		((uintptr_t)node -
		 offsetof(struct nvgpu_pd_mem_entry, tree_entry));
};

/*
 * A cache for allocating PD memory from. This enables smaller PDs to be packed
 * into single pages.
 *
 * This is fairly complex so see the documentation in pd_cache.c for a full
 * description of how this is organized.
 */
struct nvgpu_pd_cache {
	/*
	 * Array of lists of full nvgpu_pd_mem_entries and partially full (or
	 * empty) nvgpu_pd_mem_entries.
	 */
	struct nvgpu_list_node		 full[NVGPU_PD_CACHE_COUNT];
	struct nvgpu_list_node		 partial[NVGPU_PD_CACHE_COUNT];

	/*
	 * Tree of all allocated struct nvgpu_mem's for fast look up.
	 */
	struct nvgpu_rbtree_node	*mem_tree;

	/*
	 * All access to the cache much be locked. This protects the lists and
	 * the rb tree.
	 */
	struct nvgpu_mutex		 lock;
};

/*
 * GMMU page directory. This is the kernel's tracking of a list of PDEs or PTEs
 * in the GMMU.
 */
struct nvgpu_gmmu_pd {
	/*
	 * DMA memory describing the PTEs or PDEs. @mem_offs describes the
	 * offset of the PDE table in @mem. @cached specifies if this PD is
	 * using pd_cache memory.
	 */
	struct nvgpu_mem	*mem;
	u32			 mem_offs;
	bool			 cached;

	/*
	 * List of pointers to the next level of page tables. Does not
	 * need to be populated when this PD is pointing to PTEs.
	 */
	struct nvgpu_gmmu_pd	*entries;
	int			 num_entries;
};

/*
 * Reduce the number of arguments getting passed through the various levels of
 * GMMU mapping functions.
 *
 * The following fields are set statically and do not change throughout the
 * mapping call:
 *
 *   pgsz:        Index into the page size table.
 *   kind_v:      Kind attributes for mapping.
 *   cacheable:   Cacheability of the mapping.
 *   rw_flag:     Flag from enum gk20a_mem_rw_flag
 *   sparse:      Set if the mapping should be sparse.
 *   priv:        Privilidged mapping.
 *   coherent:    Set if the mapping should be IO coherent.
 *   valid:       Set if the PTE should be marked valid.
 *   aperture:    VIDMEM or SYSMEM.
 *   debug:       When set print debugging info.
 *   platform_atomic: True if platform_atomic flag is valid.
 *
 * These fields are dynamically updated as necessary during the map:
 *
 *   ctag:        Comptag line in the comptag cache;
 *                updated every time we write a PTE.
 */
struct nvgpu_gmmu_attrs {
	u32			 pgsz;
	u32			 kind_v;
	u64			 ctag;
	bool			 cacheable;
	enum gk20a_mem_rw_flag	 rw_flag;
	bool			 sparse;
	bool			 priv;
	bool			 valid;
	enum nvgpu_aperture	 aperture;
	bool			 debug;
	bool			 l3_alloc;
	bool			 platform_atomic;
};

struct gk20a_mmu_level {
	int hi_bit[2];
	int lo_bit[2];

	/*
	 * Build map from virt_addr -> phys_addr.
	 */
	void (*update_entry)(struct vm_gk20a *vm,
			     const struct gk20a_mmu_level *l,
			     struct nvgpu_gmmu_pd *pd,
			     u32 pd_idx,
			     u64 phys_addr,
			     u64 virt_addr,
			     struct nvgpu_gmmu_attrs *attrs);
	u32 entry_size;
	/*
	 * Get pde page size
	 */
	u32 (*get_pgsz)(struct gk20a *g, const struct gk20a_mmu_level *l,
				struct nvgpu_gmmu_pd *pd, u32 pd_idx);
};

static inline const char *nvgpu_gmmu_perm_str(enum gk20a_mem_rw_flag p)
{
	switch (p) {
	case gk20a_mem_flag_none:
		return "RW";
	case gk20a_mem_flag_write_only:
		return "WO";
	case gk20a_mem_flag_read_only:
		return "RO";
	default:
		return "??";
	}
}

int nvgpu_gmmu_init_page_table(struct vm_gk20a *vm);

/**
 * nvgpu_gmmu_map - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map(struct vm_gk20a *vm,
		   struct nvgpu_mem *mem,
		   u64 size,
		   u32 flags,
		   enum gk20a_mem_rw_flag rw_flag,
		   bool priv,
		   enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_map_fixed - Map memory into the GMMU.
 *
 * Kernel space.
 */
u64 nvgpu_gmmu_map_fixed(struct vm_gk20a *vm,
			 struct nvgpu_mem *mem,
			 u64 addr,
			 u64 size,
			 u32 flags,
			 enum gk20a_mem_rw_flag rw_flag,
			 bool priv,
			 enum nvgpu_aperture aperture);

/**
 * nvgpu_gmmu_unmap - Unmap a buffer.
 *
 * Kernel space.
 */
void nvgpu_gmmu_unmap(struct vm_gk20a *vm,
		      struct nvgpu_mem *mem,
		      u64 gpu_va);

int nvgpu_pd_alloc(struct vm_gk20a *vm,
		   struct nvgpu_gmmu_pd *pd,
		   u32 bytes);

void nvgpu_pd_free(struct vm_gk20a *vm, struct nvgpu_gmmu_pd *pd);
int nvgpu_pd_cache_alloc_direct(struct gk20a *g,
				  struct nvgpu_gmmu_pd *pd, u32 bytes);
void nvgpu_pd_cache_free_direct(struct gk20a *g, struct nvgpu_gmmu_pd *pd);
int nvgpu_pd_cache_init(struct gk20a *g);
void nvgpu_pd_cache_fini(struct gk20a *g);

/*
 * Some useful routines that are shared across chips.
 */
static inline u32 pd_offset_from_index(const struct gk20a_mmu_level *l,
				       u32 pd_idx)
{
	return (pd_idx * l->entry_size) / sizeof(u32);
}

static inline void pd_write(struct gk20a *g, struct nvgpu_gmmu_pd *pd,
			    size_t w, size_t data)
{
	nvgpu_mem_wr32(g, pd->mem, (pd->mem_offs / sizeof(u32)) + w, data);
}

/**
 * __nvgpu_pte_words - Compute number of words in a PTE.
 *
 * @g  - The GPU.
 *
 * This computes and returns the size of a PTE for the passed chip.
 */
u32 __nvgpu_pte_words(struct gk20a *g);

/**
 * __nvgpu_get_pte - Get the contents of a PTE by virtual address
 *
 * @g     - The GPU.
 * @vm    - VM to look in.
 * @vaddr - GPU virtual address.
 * @pte   - [out] Set to the contents of the PTE.
 *
 * Find a PTE in the passed VM based on the passed GPU virtual address. This
 * will @pte with a copy of the contents of the PTE. @pte must be an array of
 * u32s large enough to contain the PTE. This can be computed using
 * __nvgpu_pte_words().
 *
 * If you wish to write to this PTE then you may modify @pte and then use the
 * __nvgpu_set_pte().
 *
 * This function returns 0 if the PTE is found and -EINVAL otherwise.
 */
int __nvgpu_get_pte(struct gk20a *g, struct vm_gk20a *vm, u64 vaddr, u32 *pte);

/**
 * __nvgpu_set_pte - Set a PTE based on virtual address
 *
 * @g     - The GPU.
 * @vm    - VM to look in.
 * @vaddr - GPU virtual address.
 * @pte   - The contents of the PTE to write.
 *
 * Find a PTE and overwrite the contents of that PTE with the passed in data
 * located in @pte. If the PTE does not exist then no writing will happen. That
 * is this function will not fill out the page tables for you. The expectation
 * is that the passed @vaddr has already been mapped and this is just modifying
 * the mapping (for instance changing invalid to valid).
 *
 * @pte must contain at least the required words for the PTE. See
 * __nvgpu_pte_words().
 *
 * This function returns 0 on success and -EINVAL otherwise.
 */
int __nvgpu_set_pte(struct gk20a *g, struct vm_gk20a *vm, u64 vaddr, u32 *pte);


/*
 * Internal debugging routines. Probably not something you want to use.
 */
#define pte_dbg(g, attrs, fmt, args...)					\
	do {								\
		if ((attrs != NULL) && (attrs->debug))			\
			nvgpu_info(g, fmt, ##args);			\
		else							\
			nvgpu_log(g, gpu_dbg_pte, fmt, ##args);		\
	} while (0)

#endif /* NVGPU_GMMU_H */
