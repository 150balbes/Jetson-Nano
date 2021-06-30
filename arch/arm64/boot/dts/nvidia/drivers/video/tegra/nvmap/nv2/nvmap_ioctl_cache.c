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

#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/nvmap.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>
#include <soc/tegra/chip-id.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <asm/uaccess.h>
#include <soc/tegra/common.h>
#include <trace/events/nvmap.h>

#include "nvmap_ioctl.h"
#include "nvmap_heap.h"

#include "nvmap_dev.h"
#include "nvmap_handle.h"
#include "nvmap_cache.h"
#include "nvmap_misc.h"


/*
 * sizes[i] == 0  is a special case which causes handle wide operation,
 */
static int cache_maint_sanitize_size_offset(struct nvmap_handle **handles,
				u64 *sizes, u64 *offsets, int nr)
{
	int err = 0;
	int i = 0;

	for (i = 0; i < nr; i++) {
		if (sizes[i] == 0)
			sizes[i] = nvmap_handle_size(handles[i]);
	}

	return err;
}

static void cache_maint_put_handles(struct nvmap_handle **handles, int nr)
{
	int i;

	for (i = 0; i < nr; i++)
		nvmap_handle_put(handles[i]);
}


static int cache_maint_get_handles(struct nvmap_handle **handles,
					u32 *handle_ptrs, int nr)
{
	int i;
	int sync_count = 0;
	int pgalloc_count = 0;
	int heap_type;

	for (i = 0; i < nr; i++) {
		handles[i] = nvmap_handle_from_fd(handle_ptrs[i]);
		handles[i] = nvmap_handle_get(handles[i]);
		if (!handles[i]) {
			pr_err("invalid handle_ptr[%d] = %u\n",
							i, handle_ptrs[i]);
			cache_maint_put_handles(handles, i);
			return -EINVAL;
		}

		heap_type = nvmap_handle_heap_type(handles[i]);
		if (!(heap_type & nvmap_dev->cpu_access_mask)) {
			pr_err("heap %x can't be accessed from cpu\n",
								heap_type);
			cache_maint_put_handles(handles, i);
			return -EPERM;
		}
	}

	for (i = 0; i < nr; i++) {
		if (nvmap_handle_userflag(handles[i])
					& NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE)
			sync_count++;

		if (nvmap_handle_is_heap(handles[i]))
			pgalloc_count++;
	}

	/*
	 * Either all handles should have NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE
	 * or none should have it.
	 */
	if (!((sync_count == 0) || (sync_count == nr))) {
		pr_err("incorrect CACHE_SYNC_AT_RESERVE mix of handles\n");
		cache_maint_put_handles(handles, nr);
		return -EINVAL;
	}

	/*
	 * when NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE is specified mix can cause
	 * cache WB_INV at unreserve op on iovmm handles increasing overhead.
	 * So, either all handles should have pages from carveout or from iovmm.
	 */
	if (sync_count && !((pgalloc_count == 0) || (pgalloc_count == nr))) {
		pr_err("all or none of the handles should be from heap\n");
		cache_maint_put_handles(handles, nr);
		return -EINVAL;
	}

	return 0;
}

static int cache_maint_copy_args(struct nvmap_cache_op_list *op,
				u32 *handle_ptr, u64 *offset_ptr, u64 *size_ptr)
{
	int is_32 = !(op->op & NVMAP_ELEM_SIZE_U64);
	int i = 0;

	if (!op->handles || !op->offsets || !op->sizes) {
		pr_err("pointers are invalid\n");
		return -EINVAL;
	}

	if (is_32) {
		size_t tmp_size = sizeof(u32) * op->nr;
		u32 *tmp = nvmap_altalloc(tmp_size);
		if (!tmp) {
			pr_err("Failed allocating tmp buffer");
			return -ENOMEM;
		}

		if (copy_from_user(tmp, (void *)op->offsets,
					op->nr * sizeof(u32))) {
			pr_err("Can't copy from user pointer op.offsets\n");
			nvmap_altfree(tmp, tmp_size);
			return -EFAULT;
		}
		for (i = 0; i < op->nr; i++) {
			offset_ptr[i] = tmp[i];
		}

		if (copy_from_user(tmp, (void *)op->sizes,
					op->nr * sizeof(u32))) {
			pr_err("Can't copy from user pointer op.sizes\n");
			nvmap_altfree(tmp, tmp_size);
			return -EFAULT;
		}
		for (i = 0; i < op->nr; i++) {
			size_ptr[i] = tmp[i];
		}

		nvmap_altfree(tmp, tmp_size);
	} else {
		if (copy_from_user(offset_ptr, (void *)op->offsets,
					op->nr * sizeof(u64))) {
			pr_err("Can't copy from user pointer op.offsets\n");
			return -EFAULT;
		}

		if (copy_from_user(size_ptr, (void *)op->sizes,
					op->nr * sizeof(u64))) {
			pr_err("Can't copy from user pointer op.sizes\n");
			return -EFAULT;
		}
	}


	if (copy_from_user(handle_ptr, (void *)op->handles,
		op->nr * sizeof(u32))) {
		pr_err("Can't copy from user pointer op.handles\n");
		return -EFAULT;
	}

	return 0;
}


int nvmap_ioctl_cache_maint_list(struct file *filp, void __user *arg,
				 bool is_reserve_ioctl)
{
	struct nvmap_cache_op_list op;
	struct nvmap_handle **handles;
	u32 *handle_ptr;
	u64 *offset_ptr;
	u64 *size_ptr;
	int err = 0;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.nr || op.nr > UINT_MAX / sizeof(u32))
		return -EINVAL;

	if (!access_ok(VERIFY_READ, op.handles, op.nr * sizeof(u32)))
		return -EFAULT;

	handles		= nvmap_altalloc(sizeof(*handles) * op.nr);
	offset_ptr 	= nvmap_altalloc(sizeof(u64) * op.nr);
	size_ptr 	= nvmap_altalloc(sizeof(u64) * op.nr);
	handle_ptr 	= nvmap_altalloc(sizeof(u32) * op.nr);

	if (!handles || !offset_ptr || !size_ptr || !handle_ptr) {
		if (handles)
			nvmap_altfree(handles, sizeof(*handles) * op.nr);
		if (offset_ptr)
			nvmap_altfree(offset_ptr, sizeof(u64) * op.nr);
		if (size_ptr)
			nvmap_altfree(size_ptr, sizeof(u64) * op.nr);
		if (handle_ptr)
			nvmap_altfree(handle_ptr, sizeof(u32) * op.nr);
		return -ENOMEM;
	}

	err = cache_maint_copy_args(&op, handle_ptr, offset_ptr, size_ptr);
	if (err)
		goto free_mem;

	op.op &= ~NVMAP_ELEM_SIZE_U64;

	err = cache_maint_get_handles(handles, handle_ptr, op.nr);
	if (err)
		goto free_mem;

	err = cache_maint_sanitize_size_offset(handles, size_ptr,
							offset_ptr, op.nr);
	if (err)
		goto free_handles;

	if (is_reserve_ioctl) {
		err = nvmap_handles_reserve(handles, offset_ptr, size_ptr,
							op.op, op.nr);
	} else {
		err = nvmap_handles_cache_maint(handles, offset_ptr, size_ptr,
							op.op, op.nr);
	}

free_handles:
	cache_maint_put_handles(handles, op.nr);
free_mem:
	nvmap_altfree(handles, sizeof(*handles) * op.nr);
	nvmap_altfree(offset_ptr, sizeof(u64) * op.nr);
	nvmap_altfree(size_ptr, sizeof(u64) * op.nr);
	nvmap_altfree(handle_ptr, sizeof(u32) * op.nr);

	return err;
}

