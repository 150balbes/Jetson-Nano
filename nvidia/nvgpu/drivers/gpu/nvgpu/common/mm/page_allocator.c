/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bitops.h>
#include <nvgpu/allocator.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>
#include <nvgpu/sizes.h>

#include "buddy_allocator_priv.h"

#define palloc_dbg(a, fmt, arg...)			\
	alloc_dbg(palloc_owner(a), fmt, ##arg)

/*
 * Since some Linux headers are still leaked into common code this is necessary
 * for some builds.
 */
#ifdef PAGE_SIZE
#undef PAGE_SIZE
#endif

#ifdef PAGE_ALIGN
#undef PAGE_ALIGN
#endif

/*
 * VIDMEM page size is 4k.
 */
#define PAGE_SIZE		0x1000
#define PAGE_ALIGN(addr)	((addr + (PAGE_SIZE - 1)) &		\
				 ((typeof(addr)) ~(PAGE_SIZE - 1)))

/*
 * Handle the book-keeping for these operations.
 */
static inline void add_slab_page_to_empty(struct page_alloc_slab *slab,
					  struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->empty);
	slab->nr_empty++;
	page->state = SP_EMPTY;
}
static inline void add_slab_page_to_partial(struct page_alloc_slab *slab,
					    struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->partial);
	slab->nr_partial++;
	page->state = SP_PARTIAL;
}
static inline void add_slab_page_to_full(struct page_alloc_slab *slab,
					 struct page_alloc_slab_page *page)
{
	BUG_ON(page->state != SP_NONE);
	nvgpu_list_add(&page->list_entry, &slab->full);
	slab->nr_full++;
	page->state = SP_FULL;
}

static inline void del_slab_page_from_empty(struct page_alloc_slab *slab,
					    struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_empty--;
	page->state = SP_NONE;
}
static inline void del_slab_page_from_partial(struct page_alloc_slab *slab,
					      struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_partial--;
	page->state = SP_NONE;
}
static inline void del_slab_page_from_full(struct page_alloc_slab *slab,
					   struct page_alloc_slab_page *page)
{
	nvgpu_list_del(&page->list_entry);
	slab->nr_full--;
	page->state = SP_NONE;
}

static u64 nvgpu_page_alloc_length(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_length(&va->source_allocator);
}

static u64 nvgpu_page_alloc_base(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_base(&va->source_allocator);
}

static bool nvgpu_page_alloc_inited(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_initialized(&va->source_allocator);
}

static u64 nvgpu_page_alloc_end(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_end(&va->source_allocator);
}

static u64 nvgpu_page_alloc_space(struct nvgpu_allocator *a)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_space(&va->source_allocator);
}

static int nvgpu_page_reserve_co(struct nvgpu_allocator *a,
				 struct nvgpu_alloc_carveout *co)
{
	struct nvgpu_page_allocator *va = a->priv;

	return nvgpu_alloc_reserve_carveout(&va->source_allocator, co);
}

static void nvgpu_page_release_co(struct nvgpu_allocator *a,
				  struct nvgpu_alloc_carveout *co)
{
	struct nvgpu_page_allocator *va = a->priv;

	nvgpu_alloc_release_carveout(&va->source_allocator, co);
}

static struct nvgpu_sgl *nvgpu_page_alloc_sgl_next(struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem_sgl *sgl_impl = (struct nvgpu_mem_sgl *)sgl;

	return (struct nvgpu_sgl *)sgl_impl->next;
}

static u64 nvgpu_page_alloc_sgl_phys(struct gk20a *g, struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem_sgl *sgl_impl = (struct nvgpu_mem_sgl *)sgl;

	return sgl_impl->phys;
}

static u64 nvgpu_page_alloc_sgl_dma(struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem_sgl *sgl_impl = (struct nvgpu_mem_sgl *)sgl;

	return sgl_impl->dma;
}

static u64 nvgpu_page_alloc_sgl_length(struct nvgpu_sgl *sgl)
{
	struct nvgpu_mem_sgl *sgl_impl = (struct nvgpu_mem_sgl *)sgl;

	return sgl_impl->length;
}

static u64 nvgpu_page_alloc_sgl_gpu_addr(struct gk20a *g,
					 struct nvgpu_sgl *sgl,
					 struct nvgpu_gmmu_attrs *attrs)
{
	struct nvgpu_mem_sgl *sgl_impl = (struct nvgpu_mem_sgl *)sgl;

	return sgl_impl->phys;
}

static void nvgpu_page_alloc_sgt_free(struct gk20a *g, struct nvgpu_sgt *sgt)
{
	/*
	 * No-op here. The free is handled by the page_alloc free() functions.
	 */
}

/*
 * These implement the generic scatter gather ops for pages allocated
 * by the page allocator. however, the primary aim for this, is of course,
 * vidmem.
 */
static const struct nvgpu_sgt_ops page_alloc_sgl_ops = {
	.sgl_next = nvgpu_page_alloc_sgl_next,
	.sgl_phys = nvgpu_page_alloc_sgl_phys,
	.sgl_dma = nvgpu_page_alloc_sgl_dma,
	.sgl_length = nvgpu_page_alloc_sgl_length,
	.sgl_gpu_addr = nvgpu_page_alloc_sgl_gpu_addr,
	.sgt_free = nvgpu_page_alloc_sgt_free,
};

/*
 * This actually frees the sgl memory. Used by the page_alloc free() functions.
 */
static void nvgpu_page_alloc_sgl_proper_free(struct gk20a *g,
					     struct nvgpu_mem_sgl *sgl)
{
	struct nvgpu_mem_sgl *next;

	while (sgl) {
		next = sgl->next;
		nvgpu_kfree(g, sgl);
		sgl = next;
	}
}

static void nvgpu_page_alloc_free_pages(struct nvgpu_page_allocator *a,
					struct nvgpu_page_alloc *alloc,
					bool free_buddy_alloc)
{
	struct nvgpu_sgl *sgl = alloc->sgt.sgl;
	struct gk20a *g = a->owner->g;

	if (free_buddy_alloc) {
		while (sgl) {
			nvgpu_free(&a->source_allocator,
				   nvgpu_sgt_get_phys(g, &alloc->sgt, sgl));
			sgl = nvgpu_sgt_get_next(&alloc->sgt, sgl);
		}
	}

	nvgpu_page_alloc_sgl_proper_free(a->owner->g,
			(struct nvgpu_mem_sgl *)sgl);
	nvgpu_kmem_cache_free(a->alloc_cache, alloc);
}

static int insert_page_alloc(struct nvgpu_page_allocator *a,
			     struct nvgpu_page_alloc *alloc)
{
	alloc->tree_entry.key_start = alloc->base;
	alloc->tree_entry.key_end = alloc->base + alloc->length;

	nvgpu_rbtree_insert(&alloc->tree_entry, &a->allocs);
	return 0;
}

static struct nvgpu_page_alloc *find_page_alloc(
	struct nvgpu_page_allocator *a,
	u64 addr)
{
	struct nvgpu_page_alloc *alloc;
	struct nvgpu_rbtree_node *node = NULL;

	nvgpu_rbtree_search(addr, &node, a->allocs);
	if (node == NULL) {
		return NULL;
	}

	alloc = nvgpu_page_alloc_from_rbtree_node(node);

	nvgpu_rbtree_unlink(node, &a->allocs);

	return alloc;
}

static struct page_alloc_slab_page *alloc_slab_page(
	struct nvgpu_page_allocator *a,
	struct page_alloc_slab *slab)
{
	struct page_alloc_slab_page *slab_page;

	slab_page = nvgpu_kmem_cache_alloc(a->slab_page_cache);
	if (slab_page == NULL) {
		palloc_dbg(a, "OOM: unable to alloc slab_page struct!");
		return NULL;
	}

	memset(slab_page, 0, sizeof(*slab_page));

	slab_page->page_addr = nvgpu_alloc(&a->source_allocator, a->page_size);
	if (slab_page->page_addr == 0ULL) {
		nvgpu_kmem_cache_free(a->slab_page_cache, slab_page);
		palloc_dbg(a, "OOM: vidmem is full!");
		return NULL;
	}

	nvgpu_init_list_node(&slab_page->list_entry);
	slab_page->slab_size = slab->slab_size;
	slab_page->nr_objects = (u32)a->page_size / slab->slab_size;
	slab_page->nr_objects_alloced = 0;
	slab_page->owner = slab;
	slab_page->state = SP_NONE;

	a->pages_alloced++;

	palloc_dbg(a, "Allocated new slab page @ 0x%012llx size=%u",
		   slab_page->page_addr, slab_page->slab_size);

	return slab_page;
}

static void free_slab_page(struct nvgpu_page_allocator *a,
			   struct page_alloc_slab_page *slab_page)
{
	palloc_dbg(a, "Freeing slab page @ 0x%012llx", slab_page->page_addr);

	BUG_ON((slab_page->state != SP_NONE && slab_page->state != SP_EMPTY) ||
	       slab_page->nr_objects_alloced != 0U ||
	       slab_page->bitmap != 0U);

	nvgpu_free(&a->source_allocator, slab_page->page_addr);
	a->pages_freed++;

	nvgpu_kmem_cache_free(a->slab_page_cache, slab_page);
}

/*
 * This expects @alloc to have 1 empty sgl_entry ready for usage.
 */
static int do_slab_alloc(struct nvgpu_page_allocator *a,
			 struct page_alloc_slab *slab,
			 struct nvgpu_page_alloc *alloc)
{
	struct page_alloc_slab_page *slab_page = NULL;
	struct nvgpu_mem_sgl *sgl;
	unsigned long offs;

	/*
	 * Check the partial and empty lists to see if we have some space
	 * readily available. Take the slab_page out of what ever list it
	 * was in since it may be put back into a different list later.
	 */
	if (!nvgpu_list_empty(&slab->partial)) {
		slab_page = nvgpu_list_first_entry(&slab->partial,
					     page_alloc_slab_page,
					     list_entry);
		del_slab_page_from_partial(slab, slab_page);
	} else if (!nvgpu_list_empty(&slab->empty)) {
		slab_page = nvgpu_list_first_entry(&slab->empty,
					     page_alloc_slab_page,
					     list_entry);
		del_slab_page_from_empty(slab, slab_page);
	}

	if (slab_page == NULL) {
		slab_page = alloc_slab_page(a, slab);
		if (slab_page == NULL) {
			return -ENOMEM;
		}
	}

	/*
	 * We now have a slab_page. Do the alloc.
	 */
	offs = bitmap_find_next_zero_area(&slab_page->bitmap,
					  slab_page->nr_objects,
					  0, 1, 0);
	if (offs >= slab_page->nr_objects) {
		WARN(1, "Empty/partial slab with no free objects?");

		/* Add the buggy page to the full list... This isn't ideal. */
		add_slab_page_to_full(slab, slab_page);
		return -ENOMEM;
	}

	bitmap_set(&slab_page->bitmap, offs, 1);
	slab_page->nr_objects_alloced++;

	if (slab_page->nr_objects_alloced < slab_page->nr_objects) {
		add_slab_page_to_partial(slab, slab_page);
	} else if (slab_page->nr_objects_alloced == slab_page->nr_objects) {
		add_slab_page_to_full(slab, slab_page);
	} else {
		BUG(); /* Should be impossible to hit this. */
	}

	/*
	 * Handle building the nvgpu_page_alloc struct. We expect one sgl
	 * to be present.
	 */
	alloc->slab_page = slab_page;
	alloc->nr_chunks = 1;
	alloc->length = slab_page->slab_size;
	alloc->base = slab_page->page_addr + (offs * slab_page->slab_size);

	sgl         = (struct nvgpu_mem_sgl *)alloc->sgt.sgl;
	sgl->phys   = alloc->base;
	sgl->dma    = alloc->base;
	sgl->length = alloc->length;
	sgl->next   = NULL;

	return 0;
}

/*
 * Allocate from a slab instead of directly from the page allocator.
 */
static struct nvgpu_page_alloc *nvgpu_alloc_slab(
	struct nvgpu_page_allocator *a, u64 len)
{
	int err, slab_nr;
	struct page_alloc_slab *slab;
	struct nvgpu_page_alloc *alloc = NULL;
	struct nvgpu_mem_sgl *sgl = NULL;

	/*
	 * Align the length to a page and then divide by the page size (4k for
	 * this code). ilog2() of that then gets us the correct slab to use.
	 */
	slab_nr = (int)ilog2(PAGE_ALIGN(len) >> 12);
	slab = &a->slabs[slab_nr];

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	if (alloc == NULL) {
		palloc_dbg(a, "OOM: could not alloc page_alloc struct!");
		goto fail;
	}

	alloc->sgt.ops = &page_alloc_sgl_ops;

	sgl = nvgpu_kzalloc(a->owner->g, sizeof(*sgl));
	if (sgl == NULL) {
		palloc_dbg(a, "OOM: could not alloc sgl struct!");
		goto fail;
	}

	alloc->sgt.sgl = (struct nvgpu_sgl *)sgl;
	err = do_slab_alloc(a, slab, alloc);
	if (err) {
		goto fail;
	}

	palloc_dbg(a, "Alloc 0x%04llx sr=%d id=0x%010llx [slab]",
		   len, slab_nr, alloc->base);
	a->nr_slab_allocs++;

	return alloc;

fail:
	if (alloc) {
		nvgpu_kmem_cache_free(a->alloc_cache, alloc);
	}
	if (sgl) {
		nvgpu_kfree(a->owner->g, sgl);
	}
	return NULL;
}

static void nvgpu_free_slab(struct nvgpu_page_allocator *a,
			    struct nvgpu_page_alloc *alloc)
{
	struct page_alloc_slab_page *slab_page = alloc->slab_page;
	struct page_alloc_slab *slab = slab_page->owner;
	enum slab_page_state new_state;
	int offs;

	offs = (u32)(alloc->base - slab_page->page_addr) / slab_page->slab_size;
	bitmap_clear(&slab_page->bitmap, offs, 1);

	slab_page->nr_objects_alloced--;

	if (slab_page->nr_objects_alloced == 0U) {
		new_state = SP_EMPTY;
	} else {
		new_state = SP_PARTIAL;
	}

	/*
	 * Need to migrate the page to a different list.
	 */
	if (new_state != slab_page->state) {
		/* Delete - can't be in empty. */
		if (slab_page->state == SP_PARTIAL) {
			del_slab_page_from_partial(slab, slab_page);
		} else {
			del_slab_page_from_full(slab, slab_page);
		}

		/* And add. */
		if (new_state == SP_EMPTY) {
			if (nvgpu_list_empty(&slab->empty)) {
				add_slab_page_to_empty(slab, slab_page);
			} else {
				free_slab_page(a, slab_page);
			}
		} else {
			add_slab_page_to_partial(slab, slab_page);
		}
	}

	/*
	 * Now handle the page_alloc.
	 */
	nvgpu_page_alloc_free_pages(a, alloc, false);
	a->nr_slab_frees++;

	return;
}

/*
 * Allocate physical pages. Since the underlying allocator is a buddy allocator
 * the returned pages are always contiguous. However, since there could be
 * fragmentation in the space this allocator will collate smaller non-contiguous
 * allocations together if necessary.
 */
static struct nvgpu_page_alloc *do_nvgpu_alloc_pages(
	struct nvgpu_page_allocator *a, u64 pages)
{
	struct nvgpu_page_alloc *alloc;
	struct nvgpu_mem_sgl *sgl, *prev_sgl = NULL;
	u64 max_chunk_len = pages << a->page_shift;
	int i = 0;

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	if (alloc == NULL) {
		goto fail;
	}

	memset(alloc, 0, sizeof(*alloc));

	alloc->length = pages << a->page_shift;
	alloc->sgt.ops = &page_alloc_sgl_ops;

	while (pages) {
		u64 chunk_addr = 0;
		u64 chunk_pages = (u64)1 << __fls(pages);
		u64 chunk_len = chunk_pages << a->page_shift;

		/*
		 * Take care of the possibility that the allocation must be
		 * contiguous. If this is not the first iteration then that
		 * means the first iteration failed to alloc the entire
		 * requested size. The buddy allocator guarantees any given
		 * single alloc is contiguous.
		 */
		if ((a->flags & GPU_ALLOC_FORCE_CONTIG) != 0ULL && i != 0) {
			goto fail_cleanup;
		}

		if (chunk_len > max_chunk_len) {
			chunk_len = max_chunk_len;
		}

		/*
		 * Keep attempting to allocate in smaller chunks until the alloc
		 * either succeeds or is smaller than the page_size of the
		 * allocator (i.e the allocator is OOM).
		 */
		do {
			chunk_addr = nvgpu_alloc(&a->source_allocator,
						 chunk_len);

			/* Divide by 2 and try again */
			if (chunk_addr == 0ULL) {
				palloc_dbg(a, "balloc failed: 0x%llx",
					   chunk_len);
				chunk_len >>= 1;
				max_chunk_len = chunk_len;
			}
		} while (chunk_addr == 0ULL && chunk_len >= a->page_size);

		chunk_pages = chunk_len >> a->page_shift;

		if (chunk_addr == 0ULL) {
			palloc_dbg(a, "bailing @ 0x%llx", chunk_len);
			goto fail_cleanup;
		}

		sgl = nvgpu_kzalloc(a->owner->g, sizeof(*sgl));
		if (sgl == NULL) {
			nvgpu_free(&a->source_allocator, chunk_addr);
			goto fail_cleanup;
		}

		pages -= chunk_pages;

		sgl->phys   = chunk_addr;
		sgl->dma    = chunk_addr;
		sgl->length = chunk_len;

		/*
		 * Build the singly linked list with a head node that is part of
		 * the list.
		 */
		if (prev_sgl) {
			prev_sgl->next = sgl;
		} else {
			alloc->sgt.sgl = (struct nvgpu_sgl *)sgl;
		}

		prev_sgl = sgl;

		i++;
	}

	alloc->nr_chunks = i;
	alloc->base = ((struct nvgpu_mem_sgl *)alloc->sgt.sgl)->phys;

	return alloc;

fail_cleanup:
	sgl = (struct nvgpu_mem_sgl *)alloc->sgt.sgl;
	while (sgl) {
		struct nvgpu_mem_sgl *next = sgl->next;

		nvgpu_free(&a->source_allocator, sgl->phys);
		nvgpu_kfree(a->owner->g, sgl);

		sgl = next;
	}

	nvgpu_kmem_cache_free(a->alloc_cache, alloc);
fail:
	return NULL;
}

static struct nvgpu_page_alloc *nvgpu_alloc_pages(
	struct nvgpu_page_allocator *a, u64 len)
{
	struct gk20a *g = a->owner->g;
	struct nvgpu_page_alloc *alloc = NULL;
	struct nvgpu_sgl *sgl;
	u64 pages;
	int i = 0;

	pages = ALIGN(len, a->page_size) >> a->page_shift;

	alloc = do_nvgpu_alloc_pages(a, pages);
	if (alloc == NULL) {
		palloc_dbg(a, "Alloc 0x%llx (%llu) (failed)",
			   pages << a->page_shift, pages);
		return NULL;
	}

	palloc_dbg(a, "Alloc 0x%llx (%llu) id=0x%010llx",
		   pages << a->page_shift, pages, alloc->base);
	sgl = alloc->sgt.sgl;
	while (sgl) {
		palloc_dbg(a, "  Chunk %2d: 0x%010llx + 0x%llx",
			   i++,
			   nvgpu_sgt_get_phys(g, &alloc->sgt, sgl),
			   nvgpu_sgt_get_length(&alloc->sgt, sgl));
		sgl = nvgpu_sgt_get_next(&alloc->sgt, sgl);
	}
	palloc_dbg(a, "Alloc done");

	return alloc;
}

/*
 * Allocate enough pages to satisfy @len. Page size is determined at
 * initialization of the allocator.
 *
 * The return is actually a pointer to a struct nvgpu_page_alloc pointer. This
 * is because it doesn't make a lot of sense to return the address of the first
 * page in the list of pages (since they could be discontiguous). This has
 * precedent in the dma_alloc APIs, though, it's really just an annoying
 * artifact of the fact that the nvgpu_alloc() API requires a u64 return type.
 */
static u64 nvgpu_page_alloc(struct nvgpu_allocator *na, u64 len)
{
	struct nvgpu_page_allocator *a = page_allocator(na);
	struct nvgpu_page_alloc *alloc = NULL;
	u64 real_len;

	/*
	 * If we want contig pages we have to round up to a power of two. It's
	 * easier to do that here than in the buddy allocator.
	 */
	real_len = ((a->flags & GPU_ALLOC_FORCE_CONTIG) != 0ULL) ?
		roundup_pow_of_two(len) : len;

	alloc_lock(na);
	if ((a->flags & GPU_ALLOC_4K_VIDMEM_PAGES) != 0ULL &&
	    real_len <= (a->page_size / 2U)) {
		alloc = nvgpu_alloc_slab(a, real_len);
	} else {
		alloc = nvgpu_alloc_pages(a, real_len);
	}

	if (alloc == NULL) {
		alloc_unlock(na);
		return 0;
	}

	insert_page_alloc(a, alloc);

	a->nr_allocs++;
	if (real_len > a->page_size / 2U) {
		a->pages_alloced += alloc->length >> a->page_shift;
	}
	alloc_unlock(na);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		return alloc->base;
	} else {
		return (u64) (uintptr_t) alloc;
	}
}

/*
 * Note: this will remove the nvgpu_page_alloc struct from the RB tree
 * if it's found.
 */
static void nvgpu_page_free(struct nvgpu_allocator *na, u64 base)
{
	struct nvgpu_page_allocator *a = page_allocator(na);
	struct nvgpu_page_alloc *alloc;

	alloc_lock(na);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		alloc = find_page_alloc(a, base);
	} else {
		alloc = find_page_alloc(a,
			((struct nvgpu_page_alloc *)(uintptr_t)base)->base);
	}

	if (alloc == NULL) {
		palloc_dbg(a, "Hrm, found no alloc?");
		goto done;
	}

	a->nr_frees++;

	palloc_dbg(a, "Free  0x%llx id=0x%010llx",
		   alloc->length, alloc->base);

	/*
	 * Frees *alloc.
	 */
	if (alloc->slab_page) {
		nvgpu_free_slab(a, alloc);
	} else {
		a->pages_freed += (alloc->length >> a->page_shift);
		nvgpu_page_alloc_free_pages(a, alloc, true);
	}

done:
	alloc_unlock(na);
}

static struct nvgpu_page_alloc *nvgpu_alloc_pages_fixed(
	struct nvgpu_page_allocator *a, u64 base, u64 length, u32 unused)
{
	struct nvgpu_page_alloc *alloc;
	struct nvgpu_mem_sgl *sgl;

	alloc = nvgpu_kmem_cache_alloc(a->alloc_cache);
	sgl = nvgpu_kzalloc(a->owner->g, sizeof(*sgl));
	if (alloc == NULL || sgl == NULL) {
		goto fail;
	}

	alloc->sgt.ops = &page_alloc_sgl_ops;
	alloc->base = nvgpu_alloc_fixed(&a->source_allocator, base, length, 0);
	if (alloc->base == 0ULL) {
		WARN(1, "nvgpu: failed to fixed alloc pages @ 0x%010llx", base);
		goto fail;
	}

	alloc->nr_chunks = 1;
	alloc->length = length;
	alloc->sgt.sgl = (struct nvgpu_sgl *)sgl;

	sgl->phys   = alloc->base;
	sgl->dma    = alloc->base;
	sgl->length = length;
	sgl->next   = NULL;

	return alloc;

fail:
	if (sgl) {
		nvgpu_kfree(a->owner->g, sgl);
	}
	if (alloc) {
		nvgpu_kmem_cache_free(a->alloc_cache, alloc);
	}
	return NULL;
}

/*
 * @page_size is ignored.
 */
static u64 nvgpu_page_alloc_fixed(struct nvgpu_allocator *na,
				  u64 base, u64 len, u32 page_size)
{
	struct nvgpu_page_allocator *a = page_allocator(na);
	struct nvgpu_page_alloc *alloc = NULL;
	struct nvgpu_sgl *sgl;
	struct gk20a *g = a->owner->g;
	u64 aligned_len, pages;
	int i = 0;

	aligned_len = ALIGN(len, a->page_size);
	pages = aligned_len >> a->page_shift;

	alloc_lock(na);

	alloc = nvgpu_alloc_pages_fixed(a, base, aligned_len, 0);
	if (alloc == NULL) {
		alloc_unlock(na);
		return 0;
	}

	insert_page_alloc(a, alloc);
	alloc_unlock(na);

	palloc_dbg(a, "Alloc [fixed] @ 0x%010llx + 0x%llx (%llu)",
		   alloc->base, aligned_len, pages);
	sgl = alloc->sgt.sgl;
	while (sgl) {
		palloc_dbg(a, "  Chunk %2d: 0x%010llx + 0x%llx",
			   i++,
			   nvgpu_sgt_get_phys(g, &alloc->sgt, sgl),
			   nvgpu_sgt_get_length(&alloc->sgt, sgl));
		sgl = nvgpu_sgt_get_next(&alloc->sgt, sgl);
	}

	a->nr_fixed_allocs++;
	a->pages_alloced += pages;

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		return alloc->base;
	} else {
		return (u64) (uintptr_t) alloc;
	}
}

static void nvgpu_page_free_fixed(struct nvgpu_allocator *na,
				  u64 base, u64 len)
{
	struct nvgpu_page_allocator *a = page_allocator(na);
	struct nvgpu_page_alloc *alloc;

	alloc_lock(na);

	if (a->flags & GPU_ALLOC_NO_SCATTER_GATHER) {
		alloc = find_page_alloc(a, base);
		if (alloc == NULL) {
			goto done;
		}
	} else {
		alloc = (struct nvgpu_page_alloc *) (uintptr_t) base;
	}

	palloc_dbg(a, "Free  [fixed] 0x%010llx + 0x%llx",
		   alloc->base, alloc->length);

	a->nr_fixed_frees++;
	a->pages_freed += (alloc->length >> a->page_shift);

	/*
	 * This works for the time being since the buddy allocator
	 * uses the same free function for both fixed and regular
	 * allocs. This would have to be updated if the underlying
	 * allocator were to change.
	 */
	nvgpu_page_alloc_free_pages(a, alloc, true);

done:
	alloc_unlock(na);
}

static void nvgpu_page_allocator_destroy(struct nvgpu_allocator *na)
{
	struct nvgpu_page_allocator *a = page_allocator(na);

	alloc_lock(na);
	nvgpu_kfree(nvgpu_alloc_to_gpu(na), a);
	na->priv = NULL;
	alloc_unlock(na);
}

#ifdef __KERNEL__
static void nvgpu_page_print_stats(struct nvgpu_allocator *na,
				   struct seq_file *s, int lock)
{
	struct nvgpu_page_allocator *a = page_allocator(na);
	int i;

	if (lock)
		alloc_lock(na);

	__alloc_pstat(s, na, "Page allocator:");
	__alloc_pstat(s, na, "  allocs         %lld", a->nr_allocs);
	__alloc_pstat(s, na, "  frees          %lld", a->nr_frees);
	__alloc_pstat(s, na, "  fixed_allocs   %lld", a->nr_fixed_allocs);
	__alloc_pstat(s, na, "  fixed_frees    %lld", a->nr_fixed_frees);
	__alloc_pstat(s, na, "  slab_allocs    %lld", a->nr_slab_allocs);
	__alloc_pstat(s, na, "  slab_frees     %lld", a->nr_slab_frees);
	__alloc_pstat(s, na, "  pages alloced  %lld", a->pages_alloced);
	__alloc_pstat(s, na, "  pages freed    %lld", a->pages_freed);
	__alloc_pstat(s, na, "");

	__alloc_pstat(s, na, "Page size:       %lld KB",
		      a->page_size >> 10);
	__alloc_pstat(s, na, "Total pages:     %lld (%lld MB)",
		      a->length / a->page_size,
		      a->length >> 20);
	__alloc_pstat(s, na, "Available pages: %lld (%lld MB)",
		      nvgpu_alloc_space(&a->source_allocator) / a->page_size,
		      nvgpu_alloc_space(&a->source_allocator) >> 20);
	__alloc_pstat(s, na, "");

	/*
	 * Slab info.
	 */
	if (a->flags & GPU_ALLOC_4K_VIDMEM_PAGES) {
		__alloc_pstat(s, na, "Slabs:");
		__alloc_pstat(s, na, "  size      empty     partial   full");
		__alloc_pstat(s, na, "  ----      -----     -------   ----");

		for (i = 0; i < a->nr_slabs; i++) {
			struct page_alloc_slab *slab = &a->slabs[i];

			__alloc_pstat(s, na, "  %-9u %-9d %-9u %u",
				      slab->slab_size,
				      slab->nr_empty, slab->nr_partial,
				      slab->nr_full);
		}
		__alloc_pstat(s, na, "");
	}

	__alloc_pstat(s, na, "Source alloc: %s",
		      a->source_allocator.name);
	nvgpu_alloc_print_stats(&a->source_allocator, s, lock);

	if (lock)
		alloc_unlock(na);
}
#endif

static const struct nvgpu_allocator_ops page_ops = {
	.alloc		= nvgpu_page_alloc,
	.free		= nvgpu_page_free,

	.alloc_fixed	= nvgpu_page_alloc_fixed,
	.free_fixed	= nvgpu_page_free_fixed,

	.reserve_carveout	= nvgpu_page_reserve_co,
	.release_carveout	= nvgpu_page_release_co,

	.base		= nvgpu_page_alloc_base,
	.length		= nvgpu_page_alloc_length,
	.end		= nvgpu_page_alloc_end,
	.inited		= nvgpu_page_alloc_inited,
	.space		= nvgpu_page_alloc_space,

	.fini		= nvgpu_page_allocator_destroy,

#ifdef __KERNEL__
	.print_stats	= nvgpu_page_print_stats,
#endif
};

/*
 * nr_slabs is computed as follows: divide page_size by 4096 to get number of
 * 4k pages in page_size. Then take the base 2 log of that to get number of
 * slabs. For 64k page_size that works on like:
 *
 *   1024*64 / 1024*4 = 16
 *   ilog2(16) = 4
 *
 * That gives buckets of 1, 2, 4, and 8 pages (i.e 4k, 8k, 16k, 32k).
 */
static int nvgpu_page_alloc_init_slabs(struct nvgpu_page_allocator *a)
{
	size_t nr_slabs = ilog2(a->page_size >> 12);
	unsigned int i;

	a->slabs = nvgpu_kcalloc(nvgpu_alloc_to_gpu(a->owner),
				 nr_slabs,
				 sizeof(struct page_alloc_slab));
	if (a->slabs == NULL) {
		return -ENOMEM;
	}
	a->nr_slabs = nr_slabs;

	for (i = 0; i < nr_slabs; i++) {
		struct page_alloc_slab *slab = &a->slabs[i];

		slab->slab_size = SZ_4K * (1 << i);
		nvgpu_init_list_node(&slab->empty);
		nvgpu_init_list_node(&slab->partial);
		nvgpu_init_list_node(&slab->full);
		slab->nr_empty = 0;
		slab->nr_partial = 0;
		slab->nr_full = 0;
	}

	return 0;
}

int nvgpu_page_allocator_init(struct gk20a *g, struct nvgpu_allocator *na,
			      const char *name, u64 base, u64 length,
			      u64 blk_size, u64 flags)
{
	struct nvgpu_page_allocator *a;
	char buddy_name[sizeof(na->name)];
	int err;

	if (blk_size < SZ_4K) {
		return -EINVAL;
	}

	a = nvgpu_kzalloc(g, sizeof(struct nvgpu_page_allocator));
	if (a == NULL) {
		return -ENOMEM;
	}

	err = nvgpu_alloc_common_init(na, g, name, a, false, &page_ops);
	if (err) {
		goto fail;
	}

	a->alloc_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct nvgpu_page_alloc));
	a->slab_page_cache = nvgpu_kmem_cache_create(g,
					sizeof(struct page_alloc_slab_page));
	if (a->alloc_cache == NULL || a->slab_page_cache == NULL) {
		err = -ENOMEM;
		goto fail;
	}

	a->base = base;
	a->length = length;
	a->page_size = blk_size;
	a->page_shift = __ffs(blk_size);
	a->allocs = NULL;
	a->owner = na;
	a->flags = flags;

	if ((flags & GPU_ALLOC_4K_VIDMEM_PAGES) != 0ULL &&
	    blk_size > SZ_4K) {
		err = nvgpu_page_alloc_init_slabs(a);
		if (err) {
			goto fail;
		}
	}

	snprintf(buddy_name, sizeof(buddy_name), "%s-src", name);

	err = nvgpu_buddy_allocator_init(g, &a->source_allocator, NULL,
					 buddy_name, base, length, blk_size,
					 0ULL, 0ULL);
	if (err) {
		goto fail;
	}

#ifdef CONFIG_DEBUG_FS
	nvgpu_init_alloc_debug(g, na);
#endif
	palloc_dbg(a, "New allocator: type      page");
	palloc_dbg(a, "               base      0x%llx", a->base);
	palloc_dbg(a, "               size      0x%llx", a->length);
	palloc_dbg(a, "               page_size 0x%llx", a->page_size);
	palloc_dbg(a, "               flags     0x%llx", a->flags);
	palloc_dbg(a, "               slabs:    %d", a->nr_slabs);

	return 0;

fail:
	if (a->alloc_cache) {
		nvgpu_kmem_cache_destroy(a->alloc_cache);
	}
	if (a->slab_page_cache) {
		nvgpu_kmem_cache_destroy(a->slab_page_cache);
	}
	nvgpu_kfree(g, a);
	return err;
}
