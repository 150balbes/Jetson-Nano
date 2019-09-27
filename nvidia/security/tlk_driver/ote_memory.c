/*
 * Copyright (c) 2016-2018 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 */

#include <linux/atomic.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/printk.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <asm/smp_plat.h>
#include <linux/version.h>

#include "ote_protocol.h"

#define PHYS_PAGELIST_ALIGNED_PA(ent)          ((ent) & 0xFFFFFFFFF000ULL)

#ifndef CONFIG_ARM64
#define PTE_RDONLY	PTE_AP2
#endif

extern uint64_t _tlk_get_mair(void);
static uint64_t te_get_mair_attrs(void) { return _tlk_get_mair(); }
static uint64_t te_get_pte_attrs(uint64_t pte, uint64_t mair)
{
	uint64_t pte_attrs, shareable;
	uint32_t idx;
	uint8_t attrs;

	/* Bits 2:4 holds attributes index */
	idx = (pte >> 2) & 0x7;
	attrs = *((uint8_t *)&mair + idx);

	/* Bits 8:9 holds shareability field */
	shareable = (pte >> 8) & 0x3;

	/*
	 * TLK expects that attributes are stored in the first byte of PTE.
	 * MSB holds inner(4 bits), outer(4 bits) and shareable (2 bits)
	 * attributes respectively.
	 */
	pte_attrs = attrs;
	pte_attrs = pte_attrs << 56;
	pte_attrs |= (shareable << 54);
	return pte_attrs;
}

static void te_put_free_plist(struct te_shmem_desc *shmem_desc)
{
	struct tlk_device *dev = &tlk_dev;
	int idx, nbits;

	idx = shmem_desc->plist_idx;
	nbits = get_count_order(shmem_desc->nr_pages);

	bitmap_release_region(dev->plist_bitmap, idx, nbits);
}

static void te_release_mem_buffer(struct te_shmem_desc *shmem_desc)
{
	uint32_t i;

	list_del(&shmem_desc->list);
	for (i = 0; i < shmem_desc->nr_pages; i++) {
		if ((shmem_desc->type == TE_PARAM_TYPE_MEM_RW) ||
			(shmem_desc->type == TE_PARAM_TYPE_PERSIST_MEM_RW))
			set_page_dirty_lock(shmem_desc->pages[i]);
		put_page(shmem_desc->pages[i]);
	}

	te_put_free_plist(shmem_desc);

	kfree(shmem_desc->pages);
	kfree(shmem_desc);
}

void te_release_mem_buffers(struct list_head *buflist)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;

	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc, buflist, list) {
		te_release_mem_buffer(shmem_desc);
	}
}

static int te_load_page_list(unsigned long start,
			     unsigned int npages,
			     struct page **pages,
			     struct vm_area_struct **vmas)
{
	struct tlk_device *dev = &tlk_dev;
	uint64_t *ptes;
	uint64_t mair;
	int i, idx, nbits;

	nbits = get_count_order(npages);
	idx = bitmap_find_free_region(dev->plist_bitmap,
				TE_PLIST_MAX, nbits);
	if (idx < 0) {
		pr_err("%s: ERROR: plist bitmap is full, order %d max size %lu\n",
						__func__, nbits, TE_PLIST_MAX);
		return -ENOMEM;
	}

	mair = te_get_mair_attrs();
	ptes = dev->plist_addr + idx;

	for (i = 0; i < npages; i++, start += PAGE_SIZE) {
		uint64_t pte;
#ifdef CONFIG_ARM64
		/*
		 * Not performing this sanity check for 32 bit kernel as
		 * PTE_ATTRINDX* macros are not available in 32 bit headers
		 */
		if ((pgprot_val(vmas[i]->vm_page_prot) & PTE_ATTRINDX_MASK) !=
			PTE_ATTRINDX(MT_NORMAL)) {
			pr_err("%s: unsupported memory type: %llx\n",
			       __func__,
			       pgprot_val(vmas[i]->vm_page_prot) & PTE_ATTRINDX_MASK);
			bitmap_release_region(dev->plist_bitmap, idx, nbits);
			return -EINVAL;
		}
#endif

		/*
		 * Recreate the pte of the page - we can't access it
		 * safely here race-free.
		 */

		pte = page_to_phys(pages[i]);
		pte |= pgprot_val(vmas[i]->vm_page_prot);
		if (vmas[i]->vm_flags & VM_WRITE)
			pte &= ~PTE_RDONLY;

		ptes[i]  = PHYS_PAGELIST_ALIGNED_PA(pte);
		ptes[i] |= (start & (PAGE_SIZE-1));
		ptes[i] |= te_get_pte_attrs(pte, mair);
	}
	return idx;
}

static int te_pin_user_pages(struct te_oper_param *param,
			     struct page ***pages,
			     struct vm_area_struct **vmas,
			     uint32_t nr_pages, uint32_t *plist_idx)
{
	struct tlk_device *dev = &tlk_dev;
	int idx, ret = 0;
	int nr_pinned = 0;
	unsigned long start;
	uint32_t length;
	bool writable;
	int i;

	start = (unsigned long)param->u.Mem.base,
	length = param->u.Mem.len;

	*pages = kzalloc(nr_pages * sizeof(struct page **), GFP_KERNEL);
	if (!*pages) {
		pr_err("%s: Error allocating %d pages!\n",
		       __func__, (int)nr_pages);
		return OTE_ERROR_OUT_OF_MEMORY;
	}

	writable = (param->type == TE_PARAM_TYPE_MEM_RW ||
			param->type == TE_PARAM_TYPE_PERSIST_MEM_RW);

	down_read(&current->mm->mmap_sem);
	/*
	 * vmas are valid only when mmap_sem is held. Hence holding the lock
	 * across get user pages and process returned vmas.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	nr_pinned = get_user_pages(current, current->mm, start, nr_pages,
				writable, 0, *pages, vmas);
#else
	nr_pinned = get_user_pages(start, nr_pages,
				writable ? FOLL_WRITE : 0, *pages, vmas);
#endif
	if (nr_pinned != nr_pages) {
		pr_err("%s: Error %d in get_user_pages for buffer 0x%lx\n",
				__func__, nr_pinned, start);
		ret = OTE_ERROR_GENERIC;
		goto error;
	}

	idx = te_load_page_list(start, nr_pages, *pages, vmas);
	if (idx < 0) {
		pr_err("%s: Error loading page list, idx = %d!\n",
		       __func__, idx);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error;
	}
	up_read(&current->mm->mmap_sem);

	/* stores the index from the beginning of shared buffer */
	param->u.Mem.base = (uintptr_t)(dev->plist_addr + idx) -
						(uintptr_t)dev->req_addr;

	param->type |= TE_PARAM_TYPE_FLAGS_PHYS_LIST;

	*plist_idx = idx;

	return OTE_SUCCESS;
error:
	up_read(&current->mm->mmap_sem);
	for (i = 0; i < nr_pinned; i++)
		put_page((*pages)[i]);
	kfree(*pages);
	return ret;
}

static int te_prep_mem_buffer(struct te_oper_param *param,
				struct te_session *session)
{
	struct page **pages = NULL;
	struct te_shmem_desc *shmem_desc = NULL;
	int ret = 0, nr_pages = 0;
	uint32_t buf_type;
	unsigned long start;
	uint32_t length;
	struct vm_area_struct **vmas;
	uint32_t plist_idx = 0;

	/* allocate new shmem descriptor */
	shmem_desc = kzalloc(sizeof(struct te_shmem_desc), GFP_KERNEL);
	if (!shmem_desc) {
		pr_err("%s: out of memory for shmem_desc!\n", __func__);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	/* Need this for vma alloc prior to pin_user_pages */
	start = (unsigned long)param->u.Mem.base,
	length = param->u.Mem.len;
	nr_pages = (((uintptr_t)start & (PAGE_SIZE - 1)) +
			(length + PAGE_SIZE - 1)) >> PAGE_SHIFT;

	vmas = kzalloc(sizeof(*vmas) * nr_pages, GFP_KERNEL);
	if (!vmas) {
		pr_err("%s: out of memory for vmas! (%d pages)\n",
		       __func__, nr_pages);
		ret = OTE_ERROR_OUT_OF_MEMORY;
		goto error_alloc_vmas;
	}

	/* pin pages */
	ret = te_pin_user_pages(param, &pages, vmas, nr_pages, &plist_idx);
	if (ret != OTE_SUCCESS) {
		pr_err("%s: te_pin_user_pages failed (%d)\n", __func__, ret);
		goto error_pin_pages;
	}
	kfree(vmas);

	/* initialize shmem descriptor */
	INIT_LIST_HEAD(&(shmem_desc->list));
	shmem_desc->plist_idx = plist_idx;
	shmem_desc->size = param->u.Mem.len;
	shmem_desc->nr_pages = nr_pages;
	shmem_desc->pages = pages;

	/* just type (no flags) */
	buf_type = param->type & ~TE_PARAM_TYPE_ALL_FLAGS;

	/* add shmem descriptor to proper list */
	if ((buf_type == TE_PARAM_TYPE_MEM_RO) ||
		(buf_type == TE_PARAM_TYPE_MEM_RW))
		list_add_tail(&shmem_desc->list, &session->temp_shmem_list);
	else {
		list_add_tail(&shmem_desc->list,
			&session->inactive_persist_shmem_list);
	}

	return OTE_SUCCESS;

error_pin_pages:
	kfree(vmas);
error_alloc_vmas:
	kfree(shmem_desc);
error:
	return ret;
}

int te_prep_mem_buffers(struct te_request *request,
			struct te_session *session)
{
	uint32_t i;
	int ret = OTE_SUCCESS;
	struct te_oper_param *params;

	params = (struct te_oper_param *)(uintptr_t)request->params;
	for (i = 0; i < request->params_size; i++) {
		switch (params[i].type) {
		case TE_PARAM_TYPE_NONE:
		case TE_PARAM_TYPE_INT_RO:
		case TE_PARAM_TYPE_INT_RW:
			break;
		case TE_PARAM_TYPE_MEM_RO:
		case TE_PARAM_TYPE_MEM_RW:
		case TE_PARAM_TYPE_PERSIST_MEM_RO:
		case TE_PARAM_TYPE_PERSIST_MEM_RW:
			ret = te_prep_mem_buffer(params + i, session);
			if (ret != OTE_SUCCESS) {
				pr_err("%s failed with err (%d)\n",
					__func__, ret);
				ret = OTE_ERROR_BAD_PARAMETERS;
				goto error;
			}
			break;
		default:
			pr_err("%s: OTE_ERROR_BAD_PARAMETERS\n", __func__);
			ret = OTE_ERROR_BAD_PARAMETERS;
			break;
		}
	}
	return OTE_SUCCESS;
error:
	/* release mem buffers */
	te_release_mem_buffers(&session->temp_shmem_list);
	te_release_mem_buffers(&session->inactive_persist_shmem_list);
	return ret;
}

void te_activate_persist_mem_buffers(struct te_session *session)
{
	struct te_shmem_desc *shmem_desc, *tmp_shmem_desc;

	/* move persist mem buffers from inactive list to active list */
	list_for_each_entry_safe(shmem_desc, tmp_shmem_desc,
		&session->inactive_persist_shmem_list, list) {
		list_move_tail(&shmem_desc->list, &session->persist_shmem_list);
	}
}
