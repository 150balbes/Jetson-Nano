/*
 * drivers/video/tegra/nvmap/nvmap_alloc.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/moduleparam.h>
#include <linux/random.h>
#include <soc/tegra/chip-id.h>
#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

bool nvmap_convert_carveout_to_iovmm;
bool nvmap_convert_iovmm_to_carveout;

u32 nvmap_max_handle_count;
u64 nvmap_big_page_allocs;
u64 nvmap_total_page_allocs;

/* handles may be arbitrarily large (16+MiB), and any handle allocated from
 * the kernel (i.e., not a carveout handle) includes its array of pages. to
 * preserve kmalloc space, if the array of pages exceeds PAGELIST_VMALLOC_MIN,
 * the array is allocated using vmalloc. */
#define PAGELIST_VMALLOC_MIN	(PAGE_SIZE)

void *nvmap_altalloc(size_t len)
{
	if (len > PAGELIST_VMALLOC_MIN)
		return vmalloc(len);
	else
		return kmalloc(len, GFP_KERNEL);
}

void nvmap_altfree(void *ptr, size_t len)
{
	if (!ptr)
		return;

	if (len > PAGELIST_VMALLOC_MIN)
		vfree(ptr);
	else
		kfree(ptr);
}

static struct page *nvmap_alloc_pages_exact(gfp_t gfp, size_t size)
{
	struct page *page, *p, *e;
	unsigned int order;

	order = get_order(size);
	page = alloc_pages(gfp, order);

	if (!page)
		return NULL;

	split_page(page, order);
	e = nth_page(page, (1 << order));
	for (p = nth_page(page, (size >> PAGE_SHIFT)); p < e; p++)
		__free_page(p);

	return page;
}

static uint s_nr_colors = 1;
module_param_named(nr_colors, s_nr_colors, uint, 0644);

#define NVMAP_MAX_COLORS 16

struct color_list {
	u32 *counts;
	u32 *heads;
	u32 *list;
	struct page **pages;
	u32 page_count;
	u32 length;
};

static struct color_list *alloc_color_list(u32 nr_pages, u32 nr_colors)
{
	struct color_list *list;
	u32 *temp = NULL;
	u32 nr_u32;

	list = kzalloc(sizeof(struct color_list), GFP_KERNEL);
	if (!list)
		return NULL;

	list->pages = vmalloc(nr_pages * sizeof(struct page *));
	if (!list->pages) {
		kfree(list);
		return NULL;
	}

	/* Allocate counts, heads, and list with a single allocation */
	nr_u32 = nr_pages + 2 * nr_colors;
	temp = vmalloc(nr_u32 * sizeof(u32));
	if (!temp)
		goto fail;

	memset(&temp[0], 0, 2 * nr_colors *  sizeof(u32));
	list->counts = &temp[0];
	list->heads = &temp[nr_colors];
	list->list = &temp[2 * nr_colors];

	list->page_count = nr_pages;

	return list;
fail:
	if (list->pages)
		vfree(list->pages);
	kfree(list);
	return NULL;
}

static void free_color_list(struct color_list *list)
{
	vfree(list->pages);
	vfree(list->counts);	/* Frees counts, heads, and list */
	kfree(list);
}

static struct page *list_pop_page(struct color_list *list, u32 color, char *who)
{
	u32 i;

	/* Debug check */
	if ((list->counts[color] == 0) || (list->counts[color] > 1 << 31)) {
		pr_err("list_pop_page: OVER FREE!\n");
		pr_err(" called from: %s\n", who);
		for (i = 0; i < s_nr_colors; i++)
			pr_err(" color = %d: %d\n", i, list->counts[i]);
		BUG();
	}
	i = list->heads[color];
	list->heads[color] = list->list[i];
	list->counts[color]--;
	return list->pages[i];
}

struct nvmap_alloc_state {
	u32 nr_colors;
	u32 (*addr_to_color)(uintptr_t phys);
	u32 tile;
	u32 output_count;
	u32 nr_pages;
	u32 max_color_per_tile;
	struct color_list *list;
};

#define CHANNEL_MASK_0 0x27af5200
#define CHANNEL_MASK_1 0x563ca400
#define CHANNEL_MASK_2 0x3f264800
#define CHANNEL_MASK_3 0xe2443000
#define BANK_MASK_0 0x5ca78400
#define BANK_MASK_1 0xe5724800
#define BANK_MASK_2 0x973bb000

#define BIT_N(a, n) \
	(((a) >> (n)) & 1)

#define BITS_XOR_9_TO_31(a) \
	(BIT_N((a), 9) ^ BIT_N((a), 10) ^ BIT_N((a), 11) ^ BIT_N((a), 12)  ^ \
	BIT_N((a), 13) ^ BIT_N((a), 14) ^ BIT_N((a), 15) ^ BIT_N((a), 16) ^ \
	BIT_N((a), 17) ^ BIT_N((a), 18) ^ BIT_N((a), 19) ^ BIT_N((a), 20) ^ \
	BIT_N((a), 21) ^ BIT_N((a), 22) ^ BIT_N((a), 23) ^ BIT_N((a), 24) ^ \
	BIT_N((a), 25) ^ BIT_N((a), 26) ^ BIT_N((a), 27) ^ BIT_N((a), 28) ^ \
	BIT_N((a), 29) ^ BIT_N((a), 30) ^ BIT_N((a), 31))

#define BITS_XOR_10_TO_31(a) \
	(BIT_N((a), 10) ^ BIT_N((a), 11) ^ BIT_N((a), 12) ^ \
	BIT_N((a), 13) ^ BIT_N((a), 14) ^ BIT_N((a), 15) ^ BIT_N((a), 16) ^ \
	BIT_N((a), 17) ^ BIT_N((a), 18) ^ BIT_N((a), 19) ^ BIT_N((a), 20) ^ \
	BIT_N((a), 21) ^ BIT_N((a), 22) ^ BIT_N((a), 23) ^ BIT_N((a), 24) ^ \
	BIT_N((a), 25) ^ BIT_N((a), 26) ^ BIT_N((a), 27) ^ BIT_N((a), 28) ^ \
	BIT_N((a), 29) ^ BIT_N((a), 30) ^ BIT_N((a), 31))

static u32 addr_to_color_t19x(uintptr_t phys)
{
	int color, chan, bank;
	u32 addr = (u32)phys;
	u32 xaddr = (u32)(phys >> 4);


	chan =  (BITS_XOR_9_TO_31(addr & CHANNEL_MASK_0) << 0);
	chan |= (BITS_XOR_9_TO_31(addr & CHANNEL_MASK_1) << 1);
	chan |= (BITS_XOR_9_TO_31(addr & CHANNEL_MASK_2) << 2);
	chan |= (BITS_XOR_9_TO_31(addr & CHANNEL_MASK_3) << 3);

	bank = (BITS_XOR_10_TO_31(xaddr & BANK_MASK_0) << 0);
	bank |= (BITS_XOR_10_TO_31(xaddr & BANK_MASK_1) << 1);
	bank |= (BITS_XOR_10_TO_31(xaddr & BANK_MASK_2) << 2);

	WARN_ON(chan > 15);
	WARN_ON(bank > 7);
	/* It is preferable to color pages based on even/odd banks
	 * as well. To limit the number of colors to 16, bank info
	 * is not used in page coloring.
	 */
	color = chan;

	return color;
}

static struct color_list *init_color_list(struct nvmap_page_pool *pool,
					  struct nvmap_alloc_state *state,
					  u32 nr_pages)
{
	struct color_list *list;
	u32 color, i, page_index = 0;
	gfp_t gfp = GFP_NVMAP | __GFP_ZERO;

	list = alloc_color_list(nr_pages, state->nr_colors);
	if (!list)
		return NULL;

#ifdef CONFIG_NVMAP_PAGE_POOLS
	/* Allocated page from nvmap page pool if possible */
	page_index = nvmap_page_pool_alloc_lots(pool, list->pages, nr_pages);
#endif
	/* Fall back to general page allocator */
	for (i = page_index; i < nr_pages; i++) {
		list->pages[i] = nvmap_alloc_pages_exact(gfp, PAGE_SIZE);
		if (!list->pages[i])
			goto fail;
	}
	/* Clean the cache for any page that didn't come from the page pool */
	if (page_index < nr_pages)
		nvmap_clean_cache(&list->pages[page_index],
				  nr_pages - page_index);

	/* Create linked list of colors and compute the histogram */
	for (i = 0; i < nr_pages; i++) {
		color = state->addr_to_color((uintptr_t)
					     page_to_phys(list->pages[i]));
		list->list[i] = list->heads[color];
		list->heads[color] = i;
		list->counts[color]++;
	}
	return list;
fail:
	while (i--)
		__free_page(list->pages[i]);
	free_color_list(list);
	return NULL;
}

static void smooth_pages(struct color_list *list, u32 nr_extra, u32 nr_colors)
{
	u32 i, j, color, max;
	u32 counts[NVMAP_MAX_COLORS] = {0};

	if (nr_extra == 0)
		return;

	/* Determine which colors need to be freed */
	for (i = 0; i < nr_extra; i++) {
		/* Find the max */
		max = 0;
		color = 0;
		for (j = 0; j < nr_colors; j++) {
			if (list->counts[j] - counts[j] > max) {
				color = j;
				max = list->counts[j] - counts[j];
			}
		}
		counts[color]++;
	}

	/* Iterate through 0...nr_extra-1 in psuedorandom order */
	do {
		/* Pop the max off and free it */
		for (color = 0; color < nr_colors; color++) {
			while (counts[color]) {
				__free_page(list_pop_page(list,
						color, "smooth_pages"));
				counts[color]--;
				nr_extra--;
			}
		}
	} while (nr_extra > 0);
}

static void add_perfect(struct nvmap_alloc_state *state, u32 nr_pages,
			struct page **out_pages)
{
	u32 i;
	u32 color;
	struct page *page;
	uintptr_t virt_addr;


	/* create a perfect tile */
	for (i = 0;
	     i < state->nr_colors && state->output_count < nr_pages;
	     i++) {
		virt_addr = (i + (state->tile * state->nr_colors)) * PAGE_SIZE;
		color = state->addr_to_color(virt_addr);
		page = list_pop_page(state->list, color, "perfect");
		out_pages[state->output_count++] = page;
	}
}

static void add_imperfect(struct nvmap_alloc_state *state, u32 nr_pages,
			  struct page **out_pages)
{
	u32 i, j;
	u32 max_count;
	u32 color;
	struct page *page;
	uintptr_t virt_addr;
	u32 counts[NVMAP_MAX_COLORS] = {0};

	/* Determine which colors will go into the tile */
	for (i = 0; i < state->nr_colors; i++) {
		max_count = 0;
		color = 0;
		for (j = 0; j < state->nr_colors; j++) {
			u32 left = state->list->counts[j] - counts[j];

			if (left > max_count &&
			    counts[j] < state->max_color_per_tile) {
				max_count = left;
				color = j;
			}
		}
		counts[color]++;
	}

	/* Arrange the colors into the tile */
	for (i = 0;
	     i < state->nr_colors && state->output_count < nr_pages;
	     i++) {
		virt_addr = (i + (i * state->nr_colors)) * PAGE_SIZE;
		color = state->addr_to_color(virt_addr);
		/* Find a substitute color */
		if (counts[color] == 0) {
			/* Find the color used the most in the tile */
			max_count = 0;
			for (j = 0; j < state->nr_colors; j++) {
				if (counts[j] > max_count) {
					max_count = counts[j];
					color = j;
				}
			}
		}
		page = list_pop_page(state->list, color, "imperfect");
		out_pages[state->output_count++] = page;
		counts[color]--;
	}
}

static int alloc_colored(struct nvmap_page_pool *pool, u32 nr_pages,
			 struct page **out_pages, u32 chipid)
{
	struct nvmap_alloc_state state;
	u32 nr_alloc, max_count, min_count;
	u32 nr_tiles, nr_perfect, nr_imperfect;
	int dither_state;
	u32 i;

	state.nr_colors = s_nr_colors;
	state.addr_to_color = addr_to_color_t19x;

	/* Allocate pages for full 32-page tiles */
	nr_tiles = (nr_pages + state.nr_colors - 1) / state.nr_colors;
	/* Overallocate pages by 1/16th */
	nr_alloc  = state.nr_colors * nr_tiles;
	nr_alloc += nr_alloc >> 4;

	/* Create lists of each page color */
	state.list = init_color_list(pool, &state, nr_alloc);
	if (!state.list)
		return -ENOMEM;

	/* Smooth out the histogram by freeing over allocated pages */
	smooth_pages(state.list, nr_alloc - state.nr_colors * nr_tiles,
		     state.nr_colors);

	max_count = 0;
	min_count = state.list->counts[0];
	for (i = 0; i < state.nr_colors; i++) {
		if (state.list->counts[i] > max_count)
			max_count = state.list->counts[i];
		if (state.list->counts[i] < min_count)
			min_count = state.list->counts[i];
	}

	/* Compute the number of perfect / imperfect tiles and the maximum
	 * number of pages with the same color can be in a tile
	 */
	if (max_count / nr_tiles >= 3) {
		/* It is not possible to create perfect tiles with
		 * max_color_per_tile <= 3
		 */
		nr_perfect = 0;
		state.max_color_per_tile = (max_count + nr_tiles - 1)
					   / nr_tiles;
	} else if (nr_tiles * 2 == max_count) {
		/* All of the tiles can be perfect */
		nr_perfect = nr_tiles;
		state.max_color_per_tile = 2;
	} else {
		/* Some of the tiles can be perfect */
		nr_perfect = nr_tiles - (max_count % nr_tiles);
		state.max_color_per_tile = 3;
	}
	/* Check if the number of perfect tiles is bound by the color with the
	 * minimum count
	 */
	if (nr_perfect * 2 > min_count)
		nr_perfect = min_count / 2;

	nr_imperfect = nr_tiles - nr_perfect;

	/* Output tiles */
	dither_state = nr_perfect - nr_imperfect;
	state.output_count = 0;
	for (state.tile = 0; state.tile < nr_tiles; state.tile++) {
		if (dither_state > 0) {
			add_perfect(&state, nr_pages, out_pages);
			dither_state -= nr_imperfect;
		} else {
			add_imperfect(&state, nr_pages, out_pages);
			dither_state += nr_perfect;
		}
	}

	/* Free extra pages created when the buffer does not
	 * fill the last tile
	 */
	for (i = 0; i < state.nr_colors; i++)
		while (state.list->counts[i] > 0)
			__free_page(list_pop_page(state.list, i, "free"));

	free_color_list(state.list);

	return 0;
}

static int handle_page_alloc(struct nvmap_client *client,
			     struct nvmap_handle *h, bool contiguous)
{
	size_t size = h->size;
	int nr_page = size >> PAGE_SHIFT;
	int i = 0, page_index = 0;
	struct page **pages;
	gfp_t gfp = GFP_NVMAP | __GFP_ZERO;
	int pages_per_big_pg = NVMAP_PP_BIG_PAGE_SIZE >> PAGE_SHIFT;
	static u32 chipid;

	if (!chipid) {
#ifdef CONFIG_NVMAP_COLOR_PAGES
		chipid = tegra_hidrev_get_chipid(tegra_read_chipid());
		if (chipid == TEGRA194)
			s_nr_colors = 16;
#endif
	}

	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (!pages)
		return -ENOMEM;

	if (contiguous) {
		struct page *page;
		page = nvmap_alloc_pages_exact(gfp, size);
		if (!page)
			goto fail;

		for (i = 0; i < nr_page; i++)
			pages[i] = nth_page(page, i);

	} else {
#ifdef CONFIG_NVMAP_PAGE_POOLS
		/* Get as many big pages from the pool as possible. */
		page_index = nvmap_page_pool_alloc_lots_bp(&nvmap_dev->pool, pages,
								 nr_page);
		pages_per_big_pg = nvmap_dev->pool.pages_per_big_pg;
#endif
		/* Try to allocate big pages from page allocator */
		for (i = page_index;
		     i < nr_page && pages_per_big_pg > 1 && (nr_page - i) >= pages_per_big_pg;
		     i += pages_per_big_pg, page_index += pages_per_big_pg) {
			struct page *page;
			int idx;
			/*
			 * set the gfp not to trigger direct/kswapd reclaims and
			 * not to use emergency reserves.
			 */
			gfp_t gfp_no_reclaim = (gfp | __GFP_NOMEMALLOC) & ~__GFP_RECLAIM;

			page = nvmap_alloc_pages_exact(gfp_no_reclaim,
					pages_per_big_pg << PAGE_SHIFT);
			if (!page)
				break;

			for (idx = 0; idx < pages_per_big_pg; idx++)
				pages[i + idx] = nth_page(page, idx);
			nvmap_clean_cache(&pages[i], pages_per_big_pg);
		}
		nvmap_big_page_allocs += page_index;

		if (s_nr_colors <= 1) {
#ifdef CONFIG_NVMAP_PAGE_POOLS
			/* Get as many 4K pages from the pool as possible. */
			page_index += nvmap_page_pool_alloc_lots(
				      &nvmap_dev->pool, &pages[page_index],
				      nr_page - page_index);
#endif

			for (i = page_index; i < nr_page; i++) {
				pages[i] = nvmap_alloc_pages_exact(gfp,
								   PAGE_SIZE);
				if (!pages[i])
					goto fail;
			}
		} else if (page_index < nr_page) {
			if (alloc_colored(&nvmap_dev->pool,
			     nr_page - page_index, &pages[page_index], chipid))
				goto fail;
			page_index = nr_page;
		}
		nvmap_total_page_allocs += nr_page;
	}

	/*
	 * Make sure any data in the caches is cleaned out before
	 * passing these pages to userspace. Many nvmap clients assume that
	 * the buffers are clean as soon as they are allocated. nvmap
	 * clients can pass the buffer to hardware as it is without any
	 * explicit cache maintenance.
	 */
	if (page_index < nr_page)
		nvmap_clean_cache(&pages[page_index], nr_page - page_index);

	h->pgalloc.pages = pages;
	h->pgalloc.contig = contiguous;
	atomic_set(&h->pgalloc.ndirty, 0);
	return 0;

fail:
	while (i--)
		__free_page(pages[i]);
	nvmap_altfree(pages, nr_page * sizeof(*pages));
	wmb();
	return -ENOMEM;
}

static struct device *nvmap_heap_pgalloc_dev(unsigned long type)
{
	int ret = -EINVAL;
	struct device *dma_dev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	ret = 0;
#endif

	if (ret || (type != NVMAP_HEAP_CARVEOUT_VPR))
		return ERR_PTR(-EINVAL);

	dma_dev = dma_dev_from_handle(type);
	if (IS_ERR(dma_dev))
		return dma_dev;

	ret = dma_set_resizable_heap_floor_size(dma_dev, 0);
	if (ret)
		return ERR_PTR(ret);
	return dma_dev;
}

static int nvmap_heap_pgalloc(struct nvmap_client *client,
			struct nvmap_handle *h, unsigned long type)
{
	size_t size = h->size;
	struct page **pages;
	struct device *dma_dev;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t pa;

	dma_dev = nvmap_heap_pgalloc_dev(type);
	if (IS_ERR(dma_dev))
		return PTR_ERR(dma_dev);

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	dma_set_attr(DMA_ATTR_ALLOC_SINGLE_PAGES, __DMA_ATTR(attrs));
#endif

	pages = dma_alloc_attrs(dma_dev, size, &pa,
			GFP_KERNEL, __DMA_ATTR(attrs));
	if (dma_mapping_error(dma_dev, pa))
		return -ENOMEM;

	h->pgalloc.pages = pages;
	h->pgalloc.contig = 0;
	atomic_set(&h->pgalloc.ndirty, 0);
	return 0;
}

static int nvmap_heap_pgfree(struct nvmap_handle *h)
{
	size_t size = h->size;
	struct device *dma_dev;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t pa = ~(dma_addr_t)0;

	dma_dev = nvmap_heap_pgalloc_dev(h->heap_type);
	if (IS_ERR(dma_dev))
		return PTR_ERR(dma_dev);

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
	dma_set_attr(DMA_ATTR_ALLOC_SINGLE_PAGES, __DMA_ATTR(attrs));
#endif

	dma_free_attrs(dma_dev, size, h->pgalloc.pages, pa,
		       __DMA_ATTR(attrs));

	h->pgalloc.pages = NULL;
	return 0;
}
static void alloc_handle(struct nvmap_client *client,
			 struct nvmap_handle *h, unsigned int type)
{
	unsigned int carveout_mask = NVMAP_HEAP_CARVEOUT_MASK;
	unsigned int iovmm_mask = NVMAP_HEAP_IOVMM;
	int ret;

	BUG_ON(type & (type - 1));

	if (nvmap_convert_carveout_to_iovmm) {
		carveout_mask &= ~NVMAP_HEAP_CARVEOUT_GENERIC;
		iovmm_mask |= NVMAP_HEAP_CARVEOUT_GENERIC;
	} else if (nvmap_convert_iovmm_to_carveout) {
		if (type & NVMAP_HEAP_IOVMM) {
			type &= ~NVMAP_HEAP_IOVMM;
			type |= NVMAP_HEAP_CARVEOUT_GENERIC;
		}
	}

	if (type & carveout_mask) {
		struct nvmap_heap_block *b;

		b = nvmap_carveout_alloc(client, h, type, NULL);
		if (b) {
			h->heap_type = type;
			h->heap_pgalloc = false;
			/* barrier to ensure all handle alloc data
			 * is visible before alloc is seen by other
			 * processors.
			 */
			mb();
			h->alloc = true;
			return;
		}
		ret = nvmap_heap_pgalloc(client, h, type);
		if (ret)
			return;
		h->heap_type = NVMAP_HEAP_CARVEOUT_VPR;
		h->heap_pgalloc = true;
		mb();
		h->alloc = true;
	} else if (type & iovmm_mask) {
		ret = handle_page_alloc(client, h,
			h->userflags & NVMAP_HANDLE_PHYS_CONTIG);
		if (ret)
			return;
		h->heap_type = NVMAP_HEAP_IOVMM;
		h->heap_pgalloc = true;
		mb();
		h->alloc = true;
	}
}

static int alloc_handle_from_va(struct nvmap_client *client,
				 struct nvmap_handle *h,
				 ulong vaddr)
{
	int nr_page = h->size >> PAGE_SHIFT;
	struct page **pages;
	int ret = 0;

	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (IS_ERR_OR_NULL(pages))
		return PTR_ERR(pages);

	ret = nvmap_get_user_pages(vaddr & PAGE_MASK, nr_page, pages);
	if (ret) {
		nvmap_altfree(pages, nr_page * sizeof(*pages));
		return ret;
	}

	nvmap_clean_cache(&pages[0], nr_page);
	h->pgalloc.pages = pages;
	atomic_set(&h->pgalloc.ndirty, 0);
	h->heap_type = NVMAP_HEAP_IOVMM;
	h->heap_pgalloc = true;
	h->from_va = true;
	mb();
	h->alloc = true;
	return ret;
}

/* small allocations will try to allocate from generic OS memory before
 * any of the limited heaps, to increase the effective memory for graphics
 * allocations, and to reduce fragmentation of the graphics heaps with
 * sub-page splinters */
static const unsigned int heap_policy_small[] = {
	NVMAP_HEAP_CARVEOUT_VPR,
	NVMAP_HEAP_CARVEOUT_IRAM,
	NVMAP_HEAP_CARVEOUT_MASK,
	NVMAP_HEAP_IOVMM,
	0,
};

static const unsigned int heap_policy_large[] = {
	NVMAP_HEAP_CARVEOUT_VPR,
	NVMAP_HEAP_CARVEOUT_IRAM,
	NVMAP_HEAP_IOVMM,
	NVMAP_HEAP_CARVEOUT_MASK,
	0,
};

static const unsigned int heap_policy_excl[] = {
	NVMAP_HEAP_CARVEOUT_IVM,
	NVMAP_HEAP_CARVEOUT_VIDMEM,
	0,
};

int nvmap_alloc_handle(struct nvmap_client *client,
		       struct nvmap_handle *h, unsigned int heap_mask,
		       size_t align,
		       u8 kind,
		       unsigned int flags,
		       int peer)
{
	const unsigned int *alloc_policy;
	int nr_page;
	int err = -ENOMEM;
	int tag, i;
	bool alloc_from_excl = false;

	h = nvmap_handle_get(h);

	if (!h)
		return -EINVAL;

	if (h->alloc) {
		nvmap_handle_put(h);
		return -EEXIST;
	}

	nvmap_stats_inc(NS_TOTAL, h->size);
	nvmap_stats_inc(NS_ALLOC, h->size);
	trace_nvmap_alloc_handle(client, h,
		h->size, heap_mask, align, flags,
		nvmap_stats_read(NS_TOTAL),
		nvmap_stats_read(NS_ALLOC));
	h->userflags = flags;
	nr_page = ((h->size + PAGE_SIZE - 1) >> PAGE_SHIFT);
	/* Force mapping to uncached for VPR memory. */
	if (heap_mask & (NVMAP_HEAP_CARVEOUT_VPR | ~nvmap_dev->cpu_access_mask))
		h->flags = NVMAP_HANDLE_UNCACHEABLE;
	else
		h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = max_t(size_t, align, L1_CACHE_BYTES);
	h->peer = peer;
	tag = flags >> 16;

	if (!tag && client && !client->tag_warned) {
		char task_comm[TASK_COMM_LEN];
		client->tag_warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: WARNING: "
			"All NvMap Allocations must have a tag "
			"to identify the subsystem allocating memory."
			"Please pass the tag to the API call"
			" NvRmMemHanldeAllocAttr() or relevant. \n",
			client->task->pid, task_comm);
	}

	/*
	 * If user specifies one of the exclusive carveouts, allocation
	 * from no other heap should be allowed.
	 */
	for (i = 0; i < ARRAY_SIZE(heap_policy_excl); i++) {
		if (!(heap_mask & heap_policy_excl[i]))
			continue;

		if (heap_mask & ~(heap_policy_excl[i])) {
			pr_err("%s alloc mixes exclusive heap %d and other heaps\n",
			       current->group_leader->comm, heap_policy_excl[i]);
			err = -EINVAL;
			goto out;
		}
		alloc_from_excl = true;
	}

	if (!heap_mask) {
		err = -EINVAL;
		goto out;
	}

	alloc_policy = alloc_from_excl ? heap_policy_excl :
			(nr_page == 1) ? heap_policy_small : heap_policy_large;

	while (!h->alloc && *alloc_policy) {
		unsigned int heap_type;

		heap_type = *alloc_policy++;
		heap_type &= heap_mask;

		if (!heap_type)
			continue;

		heap_mask &= ~heap_type;

		while (heap_type && !h->alloc) {
			unsigned int heap;

			/* iterate possible heaps MSB-to-LSB, since higher-
			 * priority carveouts will have higher usage masks */
			heap = 1 << __fls(heap_type);
			alloc_handle(client, h, heap);
			heap_type &= ~heap;
		}
	}

out:
	if (h->alloc) {
		if (client->kernel_client)
			nvmap_stats_inc(NS_KALLOC, h->size);
		else
			nvmap_stats_inc(NS_UALLOC, h->size);
		NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(client, h, NULL));
		err = 0;
	} else {
		nvmap_stats_dec(NS_TOTAL, h->size);
		nvmap_stats_dec(NS_ALLOC, h->size);
	}
	nvmap_handle_put(h);
	return err;
}

int nvmap_alloc_handle_from_va(struct nvmap_client *client,
			       struct nvmap_handle *h,
			       ulong addr,
			       unsigned int flags)
{
	int err = -ENOMEM;
	int tag;

	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;

	if (h->alloc) {
		nvmap_handle_put(h);
		return -EEXIST;
	}

	h->userflags = flags;
	h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = PAGE_SIZE;
	tag = flags >> 16;

	if (!tag && client && !client->tag_warned) {
		char task_comm[TASK_COMM_LEN];
		client->tag_warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: WARNING: "
			"All NvMap Allocations must have a tag "
			"to identify the subsystem allocating memory."
			"Please pass the tag to the API call"
			" NvRmMemHanldeAllocAttr() or relevant. \n",
			client->task->pid, task_comm);
	}

	(void)alloc_handle_from_va(client, h, addr);

	if (h->alloc) {
		NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(client, h, NULL));
		err = 0;
	}
	nvmap_handle_put(h);
	return err;
}

void _nvmap_handle_free(struct nvmap_handle *h)
{
	unsigned int i, nr_page, page_index = 0;
	struct nvmap_handle_dmabuf_priv *curr, *next;

	list_for_each_entry_safe(curr, next, &h->dmabuf_priv, list) {
		curr->priv_release(curr->priv);
		list_del(&curr->list);
		kzfree(curr);
	}

	if (nvmap_handle_remove(nvmap_dev, h) != 0)
		return;

	if (!h->alloc)
		goto out;

	nvmap_stats_inc(NS_RELEASE, h->size);
	nvmap_stats_dec(NS_TOTAL, h->size);
	if (!h->heap_pgalloc) {
		if (h->vaddr) {
			struct vm_struct *vm;
			void *addr = h->vaddr;

			addr -= (h->carveout->base & ~PAGE_MASK);
			vm = find_vm_area(addr);
			BUG_ON(!vm);
			free_vm_area(vm);
		}

		nvmap_heap_free(h->carveout);
		nvmap_kmaps_dec(h);
		h->vaddr = NULL;
		goto out;
	} else {
		int ret = nvmap_heap_pgfree(h);
		if (!ret)
			goto out;
	}

	nr_page = DIV_ROUND_UP(h->size, PAGE_SIZE);

	BUG_ON(h->size & ~PAGE_MASK);
	BUG_ON(!h->pgalloc.pages);

	if (h->vaddr) {
		nvmap_kmaps_dec(h);

		vm_unmap_ram(h->vaddr, h->size >> PAGE_SHIFT);
		h->vaddr = NULL;
	}

	for (i = 0; i < nr_page; i++)
		h->pgalloc.pages[i] = nvmap_to_page(h->pgalloc.pages[i]);

#ifdef CONFIG_NVMAP_PAGE_POOLS
	if (!h->from_va)
		page_index = nvmap_page_pool_fill_lots(&nvmap_dev->pool,
					h->pgalloc.pages, nr_page);
#endif

	for (i = page_index; i < nr_page; i++) {
		if (h->from_va)
			put_page(h->pgalloc.pages[i]);
		else
			__free_page(h->pgalloc.pages[i]);
	}

	nvmap_altfree(h->pgalloc.pages, nr_page * sizeof(struct page *));

out:
	NVMAP_TAG_TRACE(trace_nvmap_destroy_handle,
		NULL, get_current()->pid, 0, NVMAP_TP_ARGS_H(h));
	kfree(h);
}

void nvmap_free_handle(struct nvmap_client *client,
		       struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h;

	nvmap_ref_lock(client);

	ref = __nvmap_validate_locked(client, handle);
	if (!ref) {
		nvmap_ref_unlock(client);
		return;
	}

	BUG_ON(!ref->handle);
	h = ref->handle;

	if (atomic_dec_return(&ref->dupes)) {
		NVMAP_TAG_TRACE(trace_nvmap_free_handle,
			NVMAP_TP_ARGS_CHR(client, h, ref));
		nvmap_ref_unlock(client);
		goto out;
	}

	smp_rmb();
	rb_erase(&ref->node, &client->handle_refs);
	client->handle_count--;
	atomic_dec(&ref->handle->share_count);

	nvmap_ref_unlock(client);

	if (h->owner == client)
		h->owner = NULL;

	dma_buf_put(ref->handle->dmabuf);
	NVMAP_TAG_TRACE(trace_nvmap_free_handle,
		NVMAP_TP_ARGS_CHR(client, h, ref));
	kfree(ref);

out:
	BUG_ON(!atomic_read(&h->ref));
	nvmap_handle_put(h);
}
EXPORT_SYMBOL(nvmap_free_handle);

void nvmap_free_handle_fd(struct nvmap_client *client,
			       int fd)
{
	struct nvmap_handle *handle = nvmap_handle_get_from_fd(fd);
	if (handle) {
		nvmap_free_handle(client, handle);
		nvmap_handle_put(handle);
	}
}
