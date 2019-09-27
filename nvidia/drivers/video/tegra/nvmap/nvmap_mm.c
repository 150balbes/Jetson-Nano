/*
 * drivers/video/tegra/nvmap/nvmap_mm.c
 *
 * Some MM related functionality specific to nvmap.
 *
 * Copyright (c) 2013-2018, NVIDIA CORPORATION. All rights reserved.
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

#include <trace/events/nvmap.h>
#include <linux/version.h>

#include <asm/pgtable.h>

#include "nvmap_priv.h"

enum NVMAP_PROT_OP {
	NVMAP_HANDLE_PROT_NONE = 1,
	NVMAP_HANDLE_PROT_RESTORE = 2,
};

void nvmap_zap_handle(struct nvmap_handle *handle, u64 offset, u64 size)
{
	struct list_head *vmas;
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;

	if (!handle->heap_pgalloc)
		return;

	/* if no dirty page is present, no need to zap */
	if (nvmap_handle_track_dirty(handle) && !atomic_read(&handle->pgalloc.ndirty))
		return;

	if (!size) {
		offset = 0;
		size = handle->size;
	}

	size = PAGE_ALIGN((offset & ~PAGE_MASK) + size);

	mutex_lock(&handle->lock);
	vmas = &handle->vmas;
	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		size_t vm_size = size;

		vma = vma_list->vma;
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;

		if (priv->offs || vma->vm_pgoff)
			/* vma mapping starts in the middle of handle memory.
			 * zapping needs special care. zap entire range for now.
			 * FIXME: optimze zapping.
			 */
			zap_page_range(vma, vma->vm_start,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
				vma->vm_end - vma->vm_start);
#else
				vma->vm_end - vma->vm_start, NULL);
#endif
		else
			zap_page_range(vma, vma->vm_start + offset,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
				vm_size);
#else
				vm_size, NULL);
#endif
	}
	mutex_unlock(&handle->lock);
}

static int nvmap_prot_handle(struct nvmap_handle *handle, u64 offset,
		u64 size, int op)
{
	struct list_head *vmas;
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;
	int err = -EINVAL;

	if (!handle->heap_pgalloc)
		return err;

	if ((offset >= handle->size) || (offset > handle->size - size) ||
	    (size > handle->size)) {
		pr_debug("%s offset: %lld h->size: %zu size: %lld\n", __func__,
				offset, handle->size, size);
		return err;
	}

	if (!size)
		size = handle->size;

	size = PAGE_ALIGN((offset & ~PAGE_MASK) + size);

	mutex_lock(&handle->lock);
	vmas = &handle->vmas;
	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		size_t vm_size = size;
		struct vm_area_struct *prev;

		vma = vma_list->vma;
		prev = vma->vm_prev;
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;

		if ((priv->offs || vma->vm_pgoff) ||
		    (size > (vma->vm_end - vma->vm_start)))
			vm_size = vma->vm_end - vma->vm_start;

		if (vma->vm_mm != current->mm)
			down_write(&vma->vm_mm->mmap_sem);
		switch (op) {
		case NVMAP_HANDLE_PROT_NONE:
			vma->vm_flags = vma_list->save_vm_flags;
			(void)vma_set_page_prot(vma);
			if (nvmap_handle_track_dirty(handle) &&
			    !atomic_read(&handle->pgalloc.ndirty)) {
				err = 0;
				break;
			}
			err = mprotect_fixup(vma, &prev, vma->vm_start,
					vma->vm_start + vm_size, VM_NONE);
			if (err)
				goto try_unlock;
			vma->vm_flags = vma_list->save_vm_flags;
			(void)vma_set_page_prot(vma);
			break;
		case NVMAP_HANDLE_PROT_RESTORE:
			vma->vm_flags = VM_NONE;
			(void)vma_set_page_prot(vma);
			err = mprotect_fixup(vma, &prev, vma->vm_start,
					vma->vm_start + vm_size,
					vma_list->save_vm_flags);
			if (err)
				goto try_unlock;
			_nvmap_handle_mkdirty(handle, 0, size);
			break;
		default:
			BUG();
		};
try_unlock:
		if (vma->vm_mm != current->mm)
			up_write(&vma->vm_mm->mmap_sem);
		if (err)
			goto finish;
	}
finish:
	mutex_unlock(&handle->lock);
	return err;
}

static int nvmap_prot_handles(struct nvmap_handle **handles, u64 *offsets,
		       u64 *sizes, u32 nr, int op, bool is_32)
{
	int i, err = 0;
	u32 *offs_32 = (u32 *)offsets, *sizes_32 = (u32 *)sizes;

	down_write(&current->mm->mmap_sem);
	for (i = 0; i < nr; i++) {
		err = nvmap_prot_handle(handles[i],
				is_32 ? offs_32[i] : offsets[i],
				is_32 ? sizes_32[i] : sizes[i], op);
		if (err) {
			pr_debug("%s nvmap_prot_handle failed [%d] is_32 %d\n",
					__func__, err, is_32);
			goto finish;
		}
	}
finish:
	up_write(&current->mm->mmap_sem);
	return err;
}

int nvmap_reserve_pages(struct nvmap_handle **handles, u64 *offsets, u64 *sizes,
			u32 nr, u32 op, bool is_32)
{
	int i, err;
	u32 *offs_32 = (u32 *)offsets, *sizes_32 = (u32 *)sizes;

	for (i = 0; i < nr; i++) {
		u64 size = is_32 ? sizes_32[i] : sizes[i];
		u64 offset = is_32 ? offs_32[i] : offsets[i];

		size = size ?: handles[i]->size;
		offset = offset ?: 0;

		if ((offset != 0) || (size != handles[i]->size)) {
			pr_debug("%s offset: %lld size: %lld h->size %zu\n",
					__func__, offset, size,
					handles[i]->size);
			return -EINVAL;
		}

		if (op == NVMAP_PAGES_PROT_AND_CLEAN)
			continue;

		/*
		 * NOTE: This unreserves the handle even when
		 * NVMAP_PAGES_INSERT_ON_UNRESERVE is called on some portion
		 * of the handle
		 */
		atomic_set(&handles[i]->pgalloc.reserved,
				(op == NVMAP_PAGES_RESERVE) ? 1 : 0);
	}

	if (op == NVMAP_PAGES_PROT_AND_CLEAN)
		op = NVMAP_PAGES_RESERVE;

	switch (op) {
	case NVMAP_PAGES_RESERVE:
		err = nvmap_prot_handles(handles, offsets, sizes, nr,
				NVMAP_HANDLE_PROT_NONE, is_32);
		if (err)
			return err;
		break;
	case NVMAP_INSERT_PAGES_ON_UNRESERVE:
		err = nvmap_prot_handles(handles, offsets, sizes, nr,
				NVMAP_HANDLE_PROT_RESTORE, is_32);
		if (err)
			return err;
		break;
	case NVMAP_PAGES_UNRESERVE:
		for (i = 0; i < nr; i++)
			if (nvmap_handle_track_dirty(handles[i]))
				atomic_set(&handles[i]->pgalloc.ndirty, 0);
		break;
	default:
		return -EINVAL;
	}

	if (!(handles[0]->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
		return 0;

	if (op == NVMAP_PAGES_RESERVE) {
		err = nvmap_do_cache_maint_list(handles, offsets, sizes,
					  NVMAP_CACHE_OP_WB, nr, is_32);
		if (err)
			return err;
		for (i = 0; i < nr; i++)
			nvmap_handle_mkclean(handles[i],
					is_32 ? offs_32[i] : offsets[i],
					is_32 ? sizes_32[i] : sizes[i]);
	} else if ((op == NVMAP_PAGES_UNRESERVE) && handles[0]->heap_pgalloc) {
		/* Do nothing */
	} else {
		err = nvmap_do_cache_maint_list(handles, offsets, sizes,
					  NVMAP_CACHE_OP_WB_INV, nr, is_32);
		if (err)
			return err;
	}
	return 0;
}

