/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef PAGE_ALLOCATOR_PRIV_H
#define PAGE_ALLOCATOR_PRIV_H

#include <nvgpu/allocator.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/kmem.h>
#include <nvgpu/list.h>
#include <nvgpu/rbtree.h>

struct nvgpu_allocator;

/*
 * This allocator implements the ability to do SLAB style allocation since the
 * GPU has two page sizes available - 4k and 64k/128k. When the default
 * granularity is the large page size (64k/128k) small allocations become very
 * space inefficient. This is most notable in PDE and PTE blocks which are 4k
 * in size.
 *
 * Thus we need the ability to suballocate in 64k pages. The way we do this for
 * the GPU is as follows. We have several buckets for sub-64K allocations:
 *
 *   B0 - 4k
 *   B1 - 8k
 *   B3 - 16k
 *   B4 - 32k
 *   B5 - 64k (for when large pages are 128k)
 *
 * When an allocation comes in for less than the large page size (from now on
 * assumed to be 64k) the allocation is satisfied by one of the buckets.
 */
struct page_alloc_slab {
	struct nvgpu_list_node empty;
	struct nvgpu_list_node partial;
	struct nvgpu_list_node full;

	int nr_empty;
	int nr_partial;
	int nr_full;

	u32 slab_size;
};

enum slab_page_state {
	SP_EMPTY,
	SP_PARTIAL,
	SP_FULL,
	SP_NONE
};

struct page_alloc_slab_page {
	unsigned long bitmap;
	u64 page_addr;
	u32 slab_size;

	u32 nr_objects;
	u32 nr_objects_alloced;

	enum slab_page_state state;

	struct page_alloc_slab *owner;
	struct nvgpu_list_node list_entry;
};

static inline struct page_alloc_slab_page *
page_alloc_slab_page_from_list_entry(struct nvgpu_list_node *node)
{
	return (struct page_alloc_slab_page *)
	((uintptr_t)node - offsetof(struct page_alloc_slab_page, list_entry));
};

/*
 * Struct to handle internal management of page allocation. It holds a list
 * of the chunks of pages that make up the overall allocation - much like a
 * scatter gather table.
 */
struct nvgpu_page_alloc {
	/*
	 * nvgpu_sgt for describing the actual allocation. Convenient for
	 * GMMU mapping.
	 */
	struct nvgpu_sgt sgt;

	int nr_chunks;
	u64 length;

	/*
	 * Only useful for the RB tree - since the alloc may have discontiguous
	 * pages the base is essentially irrelevant except for the fact that it
	 * is guarenteed to be unique.
	 */
	u64 base;

	struct nvgpu_rbtree_node tree_entry;

	/*
	 * Set if this is a slab alloc. Points back to the slab page that owns
	 * this particular allocation. nr_chunks will always be 1 if this is
	 * set.
	 */
	struct page_alloc_slab_page *slab_page;
};

static inline struct nvgpu_page_alloc *
nvgpu_page_alloc_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_page_alloc *)
	      ((uintptr_t)node - offsetof(struct nvgpu_page_alloc, tree_entry));
};

struct nvgpu_page_allocator {
	struct nvgpu_allocator *owner;	/* Owner of this allocator. */

	/*
	 * Use a buddy allocator to manage the allocation of the underlying
	 * pages. This lets us abstract the discontiguous allocation handling
	 * out of the annoyingly complicated buddy allocator.
	 */
	struct nvgpu_allocator source_allocator;

	/*
	 * Page params.
	 */
	u64 base;
	u64 length;
	u64 page_size;
	u32 page_shift;

	struct nvgpu_rbtree_node *allocs;	/* Outstanding allocations. */

	struct page_alloc_slab *slabs;
	int nr_slabs;

	struct nvgpu_kmem_cache *alloc_cache;
	struct nvgpu_kmem_cache *slab_page_cache;

	u64 flags;

	/*
	 * Stat tracking.
	 */
	u64 nr_allocs;
	u64 nr_frees;
	u64 nr_fixed_allocs;
	u64 nr_fixed_frees;
	u64 nr_slab_allocs;
	u64 nr_slab_frees;
	u64 pages_alloced;
	u64 pages_freed;
};

static inline struct nvgpu_page_allocator *page_allocator(
	struct nvgpu_allocator *a)
{
	return (struct nvgpu_page_allocator *)(a)->priv;
}

static inline struct nvgpu_allocator *palloc_owner(
	struct nvgpu_page_allocator *a)
{
	return a->owner;
}

#endif
