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

#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

#include <trace/events/nvmap.h>

#include "nvmap_misc.h"

bool nvmap_convert_carveout_to_iovmm;
bool nvmap_convert_iovmm_to_carveout;

u32 nvmap_max_handle_count;

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

struct page **nvmap_alloc_pages(struct page **pg_pages, u32 nr_pages)
{
	struct page **pages;
	int i;

	pages = nvmap_altalloc(sizeof(*pages) * nr_pages);
	if (!pages)
		return NULL;

	for (i = 0; i < nr_pages; i++)
		pages[i] = nvmap_to_page(pg_pages[i]);

	return pages;
}

struct page *nvmap_alloc_pages_exact(gfp_t gfp, size_t size)
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

void nvmap_altfree(void *ptr, size_t len)
{
	if (!ptr)
		return;

	if (len > PAGELIST_VMALLOC_MIN)
		vfree(ptr);
	else
		kfree(ptr);
}

int nvmap_get_user_pages(ulong vaddr, int nr_page, struct page **pages)
{
	int ret = 0;
	int user_pages;
	down_read(&current->mm->mmap_sem);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
        user_pages = get_user_pages(current, current->mm,
			      vaddr & PAGE_MASK, nr_page,
			      1/*write*/, 1, /* force */
			      pages, NULL);
#else
	user_pages = get_user_pages(vaddr & PAGE_MASK, nr_page,
			      FOLL_WRITE | FOLL_FORCE,
			      pages, NULL);
#endif
	up_read(&current->mm->mmap_sem);
	if (user_pages != nr_page) {
		ret = user_pages < 0 ? user_pages : -ENOMEM;
		pr_err("get_user_pages requested/got: %d/%d]\n", nr_page,
				user_pages);
		while (--user_pages >= 0)
			put_page(pages[user_pages]);
	}
	return ret;
}
