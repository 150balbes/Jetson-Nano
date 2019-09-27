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

#ifndef __NVMAP_CACHE_H
#define __NVMAP_CACHE_H

#include "nvmap_structs.h"

#define CALL_CLEAN_CACHE_ON_INIT 1
#define CALL_FLUSH_CACHE_ON_INIT 2

#ifdef CONFIG_ARM64
#define PG_PROT_KERNEL PAGE_KERNEL
#define FLUSH_DCACHE_AREA __flush_dcache_area
#define outer_flush_range(s, e)
#define outer_inv_range(s, e)
#define outer_clean_range(s, e)
#define outer_flush_all()
#define outer_clean_all()
extern void __clean_dcache_page(struct page *);
#else
#define PG_PROT_KERNEL pgprot_kernel
#define FLUSH_DCACHE_AREA __cpuc_flush_dcache_area
extern void __flush_dcache_page(struct address_space *, struct page *);
#endif

/*
 * TODO: put op at beginning of APIS
 * 	- remove the cache_maint_op
 *
 */

struct cache_maint_op;

int nvmap_cache_maint(struct cache_maint_op *cache_work);
void nvmap_cache_maint_inner(unsigned int op, void *vaddr, size_t size);

bool nvmap_cache_can_fast_maint(unsigned long start,
			unsigned long end, unsigned int op);
void nvmap_cache_fast_maint(unsigned int op);

void nvmap_cache_maint_heap_page_outer(struct page **pages,
				unsigned int op,
				unsigned long start, unsigned long end);

void nvmap_cache_clean_pages(struct page **pages, int numpages);

int nvmap_cache_maint_phys_range(unsigned int op, phys_addr_t pstart,
					phys_addr_t pend);

void nvmap_cache_inner_clean_all(void);
void nvmap_cache_inner_flush_all(void);

struct nvmap_chip_cache_op {
	void (*inner_clean_cache_all)(void);
	void (*inner_flush_cache_all)(void);
	const char *name;
	int flags;
};

// TODO: Rename this
void nvmap_handle_get_cacheability(struct nvmap_handle *h,
		bool *inner, bool *outer);

typedef void (*nvmap_setup_chip_cache_fn)(struct nvmap_chip_cache_op *);

extern struct of_device_id __nvmapcache_of_table;

#define NVMAP_CACHE_OF_DECLARE(compat, fn) \
	_OF_DECLARE(nvmapcache, nvmapcache_of, compat, fn, \
			nvmap_setup_chip_cache_fn)

extern size_t cache_maint_inner_threshold;
extern int nvmap_cache_maint_by_set_ways;

int nvmap_cache_debugfs_init(struct dentry *nvmap_root);
void nvmap_override_cache_ops(void);

#endif /* __NVMAP_CACHE_H */
