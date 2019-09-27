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

#ifndef __NVMAP_MISC_H
#define __NVMAP_MISC_H

#define NVMAP_IVM_INVALID_PEER		(-1)

/* bit 31-29: IVM peer
 * bit 28-16: offset (aligned to 32K)
 * bit 15-00: len (aligned to page_size)
 */
#define NVMAP_IVM_LENGTH_SHIFT (0)
#define NVMAP_IVM_LENGTH_WIDTH (16)
#define NVMAP_IVM_LENGTH_MASK  ((1 << NVMAP_IVM_LENGTH_WIDTH) - 1)
#define NVMAP_IVM_OFFSET_SHIFT (NVMAP_IVM_LENGTH_SHIFT + NVMAP_IVM_LENGTH_WIDTH)
#define NVMAP_IVM_OFFSET_WIDTH (13)
#define NVMAP_IVM_OFFSET_MASK  ((1 << NVMAP_IVM_OFFSET_WIDTH) - 1)
#define NVMAP_IVM_IVMID_SHIFT  (NVMAP_IVM_OFFSET_SHIFT + NVMAP_IVM_OFFSET_WIDTH)
#define NVMAP_IVM_IVMID_WIDTH  (3)
#define NVMAP_IVM_IVMID_MASK   ((1 << NVMAP_IVM_IVMID_WIDTH) - 1)
#define NVMAP_IVM_ALIGNMENT    (SZ_32K)

void *nvmap_altalloc(size_t len);
void nvmap_altfree(void *ptr, size_t len);

static inline size_t nvmap_ivmid_to_size(u64 ivm_id)
{
	return (ivm_id &
			((1ULL << NVMAP_IVM_LENGTH_WIDTH) - 1)) << PAGE_SHIFT;
}

static inline phys_addr_t nvmap_ivmid_to_offset(u64 ivm_id)
{
	return ((ivm_id &
			~((u64)NVMAP_IVM_IVMID_MASK << NVMAP_IVM_IVMID_SHIFT)) >>
			NVMAP_IVM_LENGTH_WIDTH) << (ffs(NVMAP_IVM_ALIGNMENT) - 1);
}

static inline int nvmap_ivmid_to_peer(u64 ivm_id)
{
	return (ivm_id >> NVMAP_IVM_IVMID_SHIFT);

}

static inline int nvmap_calculate_ivm_id(int vm_id, size_t len,
						unsigned int offs)
{
	int ivm_id = 0;

	BUG_ON(offs & (NVMAP_IVM_ALIGNMENT - 1));
	BUG_ON((offs >> ffs(NVMAP_IVM_ALIGNMENT)) &
			~((1 << NVMAP_IVM_OFFSET_WIDTH) - 1));
	BUG_ON(vm_id & ~(NVMAP_IVM_IVMID_MASK));

	BUG_ON(len & ~(PAGE_MASK));

	ivm_id = ((u64)vm_id << NVMAP_IVM_IVMID_SHIFT);
	ivm_id |= (((offs >> (ffs(NVMAP_IVM_ALIGNMENT) - 1)) &
			((1ULL << NVMAP_IVM_OFFSET_WIDTH) - 1)) <<
				NVMAP_IVM_OFFSET_SHIFT);
	ivm_id |= (len >> PAGE_SHIFT);

	return ivm_id;
}

static inline struct page *nvmap_to_page(struct page *page)
{
	return (struct page *)((unsigned long)page & ~3UL);
}

struct page **nvmap_alloc_pages(struct page **pg_pages, u32 nr_pages);
struct page *nvmap_alloc_pages_exact(gfp_t gfp, size_t size);

int nvmap_get_user_pages(ulong vaddr, int nr_page, struct page **pages);

static inline bool nvmap_page_dirty(struct page *page)
{
	return (unsigned long)page & 1UL;
}

static inline bool nvmap_page_mkdirty(struct page **page)
{
	if (nvmap_page_dirty(*page))
		return false;
	*page = (struct page *)((unsigned long)*page | 1UL);
	return true;
}

static inline bool nvmap_page_mkclean(struct page **page)
{
	if (!nvmap_page_dirty(*page))
		return false;
	*page = (struct page *)((unsigned long)*page & ~1UL);
	return true;
}

#endif /* __NVMAP_MISC_H */
