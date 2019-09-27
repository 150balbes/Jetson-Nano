/*
 * drivers/video/tegra/nvmap/nvmap_fault.c
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/nvmap.h>

#include "nvmap_vma.h"
#include "nvmap_handle.h"

#include <trace/events/nvmap.h>


/* to ensure that the backing store for the VMA isn't freed while a fork'd
 * reference still exists, nvmap_vma_open increments the reference count on
 * the handle, and nvmap_vma_close decrements it. alternatively, we could
 * disallow copying of the vma, or behave like pmem and zap the pages. FIXME.
*/
void nvmap_vma_open(struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv = vma->vm_private_data;
	struct nvmap_handle *handle = NULL;
	ulong vma_open_count;
	int err;

	if (!priv) {
		WARN(1, "VMA missing priv");
	}

	handle = priv->handle;
	if (!handle) {
		WARN(1, "VMA priv misses handle");
	}

	// TODO: Pretty sure no one uses this ref count
	nvmap_handle_umap_inc(handle);

	vma_open_count = atomic_inc_return(&priv->count);
	if (vma_open_count == 1) {
		err = nvmap_handle_open_vma(handle);
		if (err) {
			WARN(1, "handle_open_vma failed");
		}
	}


	err = nvmap_handle_add_vma(handle, vma);
	if (err) {
		WARN(1, "vma not tracked");
	}
}

static void nvmap_vma_close(struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv = vma->vm_private_data;
	struct nvmap_handle *h;
	ulong vma_open_count;
	int err;

	if (!priv)
		return;

	h = priv->handle;
	BUG_ON(!h);

	err = nvmap_handle_del_vma(h, vma);
	if (err) {
		WARN(1, "Handle del vma failed");
		return;
	}

	nvmap_handle_umap_dec(h);

	vma_open_count = __atomic_add_unless(&priv->count, -1, 0);
	if (vma_open_count == 1) {
		err = nvmap_handle_close_vma(h);
		if (err)
			WARN(1, "Handle close vma failed");

		// TODO: There is NO handle_get in vma_open
		//  This is PROBABLY a bug
		if (priv->handle)
			nvmap_handle_put(priv->handle);
		vma->vm_private_data = NULL;
		kfree(priv);
	}
}

static int nvmap_vma_fault(struct vm_area_struct *vma, struct vm_fault *vmf,
				unsigned long vmf_address)
{
	struct page *page = NULL;
	struct nvmap_vma_priv *priv;
	unsigned long offs;
	int err;

	offs = (unsigned long)(vmf_address - vma->vm_start);
	priv = vma->vm_private_data;
	if (!priv || !priv->handle)
		return VM_FAULT_SIGBUS;

	offs += priv->offs;
	/* if the VMA was split for some reason, vm_pgoff will be the VMA's
	 * offset from the original VMA */
	offs += (vma->vm_pgoff << PAGE_SHIFT);


	err = nvmap_handle_fault_vma(priv->handle, offs, &page);
	if (err){
		if (page != NULL) {
			vm_insert_pfn(vma, (unsigned long)vmf_address,
							page_to_pfn(page));
		}
		return err;
	}

	if (page)
		get_page(page);
	vmf->page = page;
	return (page) ? 0 : VM_FAULT_SIGBUS;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
static int nvmap_vma_fault_k414(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	unsigned long vmf_address = vmf->address;

	return nvmap_vma_fault(vma, vmf, vmf->address);

}
#else
static int nvmap_vma_fault_k409(struct vm_area_struct *vma,
					struct vm_fault *vmf)
{

	return nvmap_vma_fault(vma, vmf, (unsigned long) vmf->virtual_address);
}
#endif

static bool nvmap_fixup_prot(struct vm_area_struct *vma,
		unsigned long addr, pgoff_t pgoff)
{
	struct nvmap_vma_priv *priv;
	unsigned long offs;

	priv = vma->vm_private_data;
	if (!priv || !priv->handle)
		return false;

	offs = pgoff << PAGE_SHIFT;
	offs += priv->offs;

	return nvmap_handle_fixup_prot_vma(priv->handle, offs);
}

struct vm_operations_struct nvmap_vma_ops = {
	.open		= nvmap_vma_open,
	.close		= nvmap_vma_close,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	.fault		= nvmap_vma_fault_k414,
#else
	.fault		= nvmap_vma_fault_k409,
#endif
	.fixup_prot	= nvmap_fixup_prot,
};

int is_nvmap_vma(struct vm_area_struct *vma)
{
	return vma->vm_ops == &nvmap_vma_ops;
}
