/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <soc/tegra/chip-id.h>

#include <trace/events/nvmap.h>

#include "nvmap_heap_alloc.h"
#include "nvmap_cache.h"
#include "nvmap_misc.h"
#include "nvmap_pp.h"
#include "nvmap_dev.h"

static uint s_nr_colors = 1;
module_param_named(nr_colors, s_nr_colors, uint, 0644);

#define NVMAP_MAX_COLORS 16

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

struct color_list {
	u32 *counts;
	u32 *heads;
	u32 *list;
	struct page **pages;
	u32 page_count;
	u32 length;
};

struct nvmap_alloc_state {
	u32 nr_colors;
	u32 (*addr_to_color)(uintptr_t phys);
	u32 tile;
	u32 output_count;
	u32 nr_pages;
	u32 max_color_per_tile;
	struct color_list *list;
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
		nvmap_cache_clean_pages(&list->pages[page_index],
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

int nvmap_color_is_enabled(void)
{
	return s_nr_colors > 1;
}

int nvmap_color_init(void)
{
	u32 chipid = tegra_hidrev_get_chipid(tegra_read_chipid());

	if (chipid == TEGRA194)
		s_nr_colors = 16;
	return 0;
}

int nvmap_color_alloc(struct nvmap_page_pool *pool, u32 nr_pages,
			 struct page **out_pages)
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
