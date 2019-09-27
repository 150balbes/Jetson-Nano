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

#ifndef __NVMAP_PP_H
#define __NVMAP_PP_H

#ifdef CONFIG_NVMAP_PAGE_POOLS

struct nvmap_device;
/*
 * This is the default ratio defining pool size. It can be thought of as pool
 * size in either MB per GB or KB per MB. That means the max this number can
 * be is 1024 (all physical memory - not a very good idea) or 0 (no page pool
 * at all).
 */
#define NVMAP_PP_POOL_SIZE               (128)

#define NVMAP_PP_BIG_PAGE_SIZE           (0x10000)

struct nvmap_page_pool {
	struct rt_mutex lock;
	u32 count;      /* Number of pages in the page & dirty list. */
	u32 max;        /* Max no. of pages in all lists. */
	u32 to_zero;    /* Number of pages on the zero list */
	u32 under_zero; /* Number of pages getting zeroed */
	u32 big_pg_sz;  /* big page size supported(64k, etc.) */
	u32 big_page_count;   /* Number of zeroed big pages avaialble */
	u32 pages_per_big_pg; /* Number of pages in big page */
	struct list_head page_list;
	struct list_head zero_list;
	struct list_head page_list_bp;

#ifdef CONFIG_NVMAP_PAGE_POOL_DEBUG
	u64 allocs;
	u64 fills;
	u64 hits;
	u64 misses;
#endif
};

int nvmap_page_pool_init(struct nvmap_device *dev);
int nvmap_page_pool_fini(struct nvmap_device *dev);
struct page *nvmap_page_pool_alloc(struct nvmap_page_pool *pool);
int nvmap_page_pool_alloc_lots(struct nvmap_page_pool *pool,
					struct page **pages, u32 nr);
int nvmap_page_pool_alloc_lots_bp(struct nvmap_page_pool *pool,
					struct page **pages, u32 nr);
int nvmap_page_pool_fill_lots(struct nvmap_page_pool *pool,
				       struct page **pages, u32 nr);
int nvmap_page_pool_clear(void);
int nvmap_page_pool_debugfs_init(struct dentry *nvmap_root);
#endif

#endif /* __NVMAP_PP_H */
