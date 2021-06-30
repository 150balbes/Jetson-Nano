/*
 * drivers/video/tegra/nvmap/nvmap_vma.c
 *
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

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/list.h>
#include <linux/version.h>

#include "nvmap_vma.h"
#include "nvmap_handle.h"

extern struct vm_operations_struct nvmap_vma_ops;

int nvmap_vma_is_nvmap(struct vm_area_struct *vma)
{
	return vma->vm_ops == &nvmap_vma_ops;
}

int nvmap_vma_belongs_to_handle(struct vm_area_struct *vma,
					struct nvmap_handle *h)
{
	struct nvmap_vma_priv *priv;

	priv = (struct nvmap_vma_priv *) vma->vm_private_data;

	return (priv->handle == h);
}

static void nvmap_zap_page_range(struct vm_area_struct *vma,
		unsigned long start, unsigned long size)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	zap_page_range(vma, start, size);
#else
	zap_page_range(vma, start, size, NULL);
#endif
}

void nvmap_vma_zap(struct list_head *vmas, u64 offset, u64 size)
{
	struct nvmap_vma_list *vma_list;
	struct vm_area_struct *vma;

	list_for_each_entry(vma_list, vmas, list) {
		struct nvmap_vma_priv *priv;
		size_t vm_size = size;

		vma = vma_list->vma;
		priv = vma->vm_private_data;
		if ((offset + size) > (vma->vm_end - vma->vm_start))
			vm_size = vma->vm_end - vma->vm_start - offset;

		if (priv->offs || vma->vm_pgoff) {
			/* vma mapping starts in the middle of handle memory.
			 * zapping needs special care. zap entire range for now.
			 * FIXME: optimze zapping.
			 */
			nvmap_zap_page_range(vma, vma->vm_start,
					vma->vm_end - vma->vm_start);
		} else {
			nvmap_zap_page_range(vma, vma->vm_start + offset,
						vm_size);
		}
	}
}

static int nvmap_vma_list_prot_none(struct nvmap_vma_list *vma_list,
				struct vm_area_struct *vma,
				struct vm_area_struct *prev,
				size_t vm_size,
				int handle_is_dirty)
{
	int err = 0;

	vma->vm_flags = vma_list->save_vm_flags;
	(void)vma_set_page_prot(vma);

	if (!handle_is_dirty)
		return 0;

	err = mprotect_fixup(vma, &prev, vma->vm_start,
			vma->vm_start + vm_size, VM_NONE);
	if (err)
		return err;

	vma->vm_flags = vma_list->save_vm_flags;
	(void)vma_set_page_prot(vma);

	return err;
}

static int nvmap_vma_list_prot_restore(struct nvmap_vma_list *vma_list,
				struct vm_area_struct *vma,
				struct vm_area_struct *prev,
				size_t vm_size)
{
	int err = 0;

	vma->vm_flags = VM_NONE;
	(void)vma_set_page_prot(vma);

	err = mprotect_fixup(vma, &prev, vma->vm_start,
			vma->vm_start + vm_size,
			vma_list->save_vm_flags);
	return err;
}

int nvmap_vma_list_prot(struct nvmap_vma_list *vma_list, u64 offset,
					u64 size, int handle_is_dirty, int op)
{
	struct vm_area_struct *vma = vma_list->vma;
	struct nvmap_vma_priv *priv = vma->vm_private_data;
	struct vm_area_struct *prev = vma->vm_prev;
	size_t vm_size;
	int err = 0;

	vm_size = size;

	if ((offset + size) > (vma->vm_end - vma->vm_start))
		vm_size = vma->vm_end - vma->vm_start - offset;

	if ((priv->offs || vma->vm_pgoff) ||
			(size > (vma->vm_end - vma->vm_start)))
		vm_size = vma->vm_end - vma->vm_start;

	if (vma->vm_mm != current->mm)
		down_write(&vma->vm_mm->mmap_sem);

	switch (op) {
		case NVMAP_HANDLE_PROT_NONE:
			err = nvmap_vma_list_prot_none(vma_list, vma,
							prev, vm_size,
							handle_is_dirty);
			break;
		case NVMAP_HANDLE_PROT_RESTORE:
			err = nvmap_vma_list_prot_restore(vma_list, vma,
							prev, vm_size);
			break;
		default:
			BUG();
	};

	if (vma->vm_mm != current->mm)
		up_write(&vma->vm_mm->mmap_sem);

	return err;
}
