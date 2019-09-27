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

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/vmalloc.h>
#include <linux/nvmap.h>
#include <linux/slab.h>
#include <linux/highmem.h>

#include <trace/events/nvmap.h>

#include <asm/pgtable.h>

#include "nvmap_handle.h"
#include "nvmap_handle_priv.h"
#include "nvmap_client.h"
#include "nvmap_dev.h"
#include "nvmap_misc.h"
#include "nvmap_carveout.h"
#include "nvmap_vma.h"
#include "nvmap_cache.h"

int nvmap_handle_owns_vma(struct nvmap_handle *h, struct vm_area_struct *vma)
{
	return nvmap_vma_belongs_to_handle(vma, h);
}

int nvmap_handle_add_vma(struct nvmap_handle *handle,
					struct vm_area_struct *vma)
{
	struct nvmap_vma_list *vma_list;
	struct nvmap_vma_list *tmp;
	struct list_head *tmp_head = NULL;
	bool vma_pos_found = false;
	pid_t current_pid = task_tgid_nr(current);

	// TODO: Just push this code into the vma code
	vma_list = kmalloc(sizeof(*vma_list), GFP_KERNEL);
	if (!vma_list)
		return -ENOMEM;

	vma_list->vma = vma;
	vma_list->pid = current_pid;
	vma_list->save_vm_flags = vma->vm_flags;
	atomic_set(&vma_list->ref, 1);

	mutex_lock(&handle->lock);
	tmp_head = &handle->vmas;

	/* insert vma into handle's vmas list in the increasing order of
	 * handle offsets
	 */
	list_for_each_entry(tmp, &handle->vmas, list) {
		/* if vma exists in list, just increment refcount */
		if (tmp->vma == vma) {
			atomic_inc(&tmp->ref);
			kfree(vma_list);
			goto unlock;
		}

		if (!vma_pos_found && (current_pid == tmp->pid)) {
			if (vma->vm_pgoff < tmp->vma->vm_pgoff) {
				tmp_head = &tmp->list;
				vma_pos_found = true;
			} else {
				tmp_head = tmp->list.next;
			}
		}
	}

	list_add_tail(&vma_list->list, tmp_head);
unlock:
	mutex_unlock(&handle->lock);
	return 0;
}

int nvmap_handle_del_vma(struct nvmap_handle *handle,
					struct vm_area_struct *vma)
{
	struct nvmap_vma_list *vma_list;
	bool vma_found = false;

	mutex_lock(&handle->lock);

	list_for_each_entry(vma_list, &handle->vmas, list) {
		if (vma_list->vma != vma)
			continue;
		if (atomic_dec_return(&vma_list->ref) == 0) {
			list_del(&vma_list->list);
			kfree(vma_list);
		}
		vma_found = true;
		break;
	}
	if (!vma_found)
		return -EFAULT;

	mutex_unlock(&handle->lock);

	return 0;
}

int nvmap_handle_open_vma(struct nvmap_handle *handle)
{
	int nr_page, i;
	mutex_lock(&handle->lock);
	if (!handle->heap_pgalloc) {
		goto finish;
	}

	nr_page = handle->size >> PAGE_SHIFT;
	for (i = 0; i < nr_page; i++) {
		struct page *page = nvmap_to_page(handle->pgalloc.pages[i]);
		/* This is necessry to avoid page being accounted
		 * under NR_FILE_MAPPED. This way NR_FILE_MAPPED would
		 * be fully accounted under NR_FILE_PAGES. This allows
		 * Android low mem killer detect low memory condition
		 * precisely.
		 * This has a side effect of inaccurate pss accounting
		 * for NvMap memory mapped into user space. Android
		 * procrank and NvMap Procrank both would have same
		 * issue. Subtracting NvMap_Procrank pss from
		 * procrank pss would give non-NvMap pss held by process
		 * and adding NvMap memory used by process represents
		 * entire memroy consumption by the process.
		 */
		atomic_inc(&page->_mapcount);
	}
finish:
	mutex_unlock(&handle->lock);
	return 0;
}

int nvmap_handle_close_vma(struct nvmap_handle *handle)
{
	int nr_page, i;

	nr_page = handle->size >> PAGE_SHIFT;

	if (!handle->heap_pgalloc)
		return 0;

	mutex_lock(&handle->lock);

	for (i = 0; i < nr_page; i++) {
		struct page *page;
		page = nvmap_to_page(handle->pgalloc.pages[i]);
		atomic_dec(&page->_mapcount);
	}

	mutex_unlock(&handle->lock);

	return 0;
}

static void page_inner_cache_maint(struct page *page)
{
	void *kaddr;

	/* inner cache maint */
	kaddr  = kmap(page);
	BUG_ON(!kaddr);
	nvmap_cache_maint_inner(NVMAP_CACHE_OP_WB_INV, kaddr, PAGE_SIZE);
	kunmap(page);

}

int nvmap_handle_fault_vma(struct nvmap_handle *handle,
		unsigned long offs, struct page **page_ptr)
{
	struct page *page;

	if (!handle->alloc || offs >= handle->size)
		return VM_FAULT_SIGBUS;

	if (!handle->heap_pgalloc) {
		unsigned long pfn;
		BUG_ON(handle->carveout->base & ~PAGE_MASK);
		pfn = ((handle->carveout->base + offs) >> PAGE_SHIFT);
		if (!pfn_valid(pfn)) {
			*page_ptr = pfn_to_page(pfn);
			return VM_FAULT_NOPAGE;
		}
		/* CMA memory would get here */
		page = pfn_to_page(pfn);
	} else {

		offs >>= PAGE_SHIFT;
		if (atomic_read(&handle->pgalloc.reserved))
			return VM_FAULT_SIGBUS;
		page = nvmap_to_page(handle->pgalloc.pages[offs]);

		if (!nvmap_handle_track_dirty(handle))
			goto finish;

		mutex_lock(&handle->lock);
		if (nvmap_page_dirty(handle->pgalloc.pages[offs])) {
			mutex_unlock(&handle->lock);
			goto finish;
		}

		page_inner_cache_maint(page);

		nvmap_page_mkdirty(&handle->pgalloc.pages[offs]);
		atomic_inc(&handle->pgalloc.ndirty);
		mutex_unlock(&handle->lock);
	}
finish:
	*page_ptr = page;
	return 0;
}

bool nvmap_handle_fixup_prot_vma(struct nvmap_handle *handle,
					unsigned long offs)
{
	struct page *page;

	if (!handle->alloc)
		return false;

	if ((offs >= handle->size) || !handle->heap_pgalloc)
		return false;

	if (atomic_read(&handle->pgalloc.reserved))
		return false;

	if (!nvmap_handle_track_dirty(handle))
		return true;

	mutex_lock(&handle->lock);

	offs >>= PAGE_SHIFT;
	if (nvmap_page_dirty(handle->pgalloc.pages[offs]))
		goto unlock;

	page = nvmap_to_page(handle->pgalloc.pages[offs]);

	page_inner_cache_maint(page);

	nvmap_page_mkdirty(&handle->pgalloc.pages[offs]);
	atomic_inc(&handle->pgalloc.ndirty);

unlock:
	mutex_unlock(&handle->lock);
	return true;

}
