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

#ifndef __NVMAP_PAGE_COLOR_H
#define __NVMAP_PAGE_COLOR_H

#include "nvmap_structs.h"

#ifdef CONFIG_NVMAP_COLOR_PAGES

int nvmap_color_alloc(struct nvmap_page_pool *pool, u32 nr_pages,
			 struct page **out_pages);
int nvmap_color_init(void);
int nvmap_color_is_enabled(void);

#else

static inline int nvmap_color_alloc(struct nvmap_page_pool *pool, u32 nr_pages,
			 struct page **out_pages)
{
	return -1;
}

static inline int nvmap_color_init(void)
{
	return 0;
}

static inline int nvmap_color_is_enabled(void)
{
	return 0;
}

#endif

#endif
