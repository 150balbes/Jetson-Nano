/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.c
 *
 * User-space interface to nvmap
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
#include <linux/syscalls.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <asm/uaccess.h>
#include <soc/tegra/common.h>
#include <trace/events/nvmap.h>

#include "nvmap_heap.h"
#include "nvmap_stats.h"

#include "nvmap_structs.h"
#include "nvmap_ioctl.h"
#include "nvmap_ioctl.h"

#include "nvmap_dmabuf.h"
#include "nvmap_client.h"
#include "nvmap_handle.h"
#include "nvmap_carveout.h"
#include "nvmap_dev.h"
#include "nvmap_tag.h"
#include "nvmap_misc.h"
#include "nvmap_vma.h"

struct nvmap_carveout_node;

/* TODO: Remove this */
extern struct device tegra_vpr_dev;

int ioctl_create_handle_from_fd(struct nvmap_client *client,
				int orig_fd)
{
	struct nvmap_handle *handle = NULL;
	int fd;
	int err;

	handle = nvmap_handle_from_fd(orig_fd);
	/* If we can't get a handle, then we create a new
	 * FD for the non-nvmap buffer
	 */
	if (IS_ERR(handle)) {
		struct dma_buf *dmabuf = dma_buf_get(orig_fd);

		if (IS_ERR(dmabuf) || !dmabuf) {
			return -1;
		}

		if (nvmap_dmabuf_is_nvmap(dmabuf)) {
			return -1;
		}

		fd = nvmap_client_create_fd(client);
		nvmap_dmabuf_install_fd(dmabuf, fd);

		get_dma_buf(dmabuf);
		return fd;
	}

	err = nvmap_client_add_handle(client, handle);
	if (err) {
		return -1;
	}

	fd  = nvmap_client_create_fd(client);
	if (fd < 0) {
		nvmap_client_remove_handle(client, handle);
		return -1;
	}
	nvmap_handle_install_fd(handle, fd);

	return fd;
}

int nvmap_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;
	int err = 0;
	int fd;

	if (!client) {
		return -ENODEV;
	}

	if (copy_from_user(&op, arg, sizeof(op))) {
		return -EFAULT;
	}

	if (cmd == NVMAP_IOC_CREATE) {
		op.size64 = op.size;
	}

	if ((cmd == NVMAP_IOC_CREATE) || (cmd == NVMAP_IOC_CREATE_64)) {
		fd = nvmap_client_create_handle(client, op.size64);
	} else if (cmd == NVMAP_IOC_FROM_FD) {
		fd = ioctl_create_handle_from_fd(client, op.fd);
	} else {
		return -EFAULT;
	}

	if (fd < 0)
		return -EFAULT;

	if (cmd == NVMAP_IOC_CREATE_64) {
		op.handle64 = fd;
	} else {
		op.handle = fd;
	}

	err = copy_to_user(arg, &op, sizeof(op));
	if (err) {
		err = -EFAULT;
		goto failed;
	}

	return 0;
failed:
	// TODO: Find a way to free the handle here
	// Needs to make sure it covers case where FD wasn't nvmap fd
	pr_warn("Need to free handle here!\n");
	return err;
}

int nvmap_ioctl_getfd(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle *handle;
	struct nvmap_client *client = filp->private_data;
	int fd;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = nvmap_handle_from_fd(op.handle);
	/* If there's no handle install a non-nvmap dmabuf fd */
	// TODO: Clean this up
	if (IS_ERR(handle)) {
		struct dma_buf *dmabuf;

		dmabuf = dma_buf_get(op.handle);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);

		fd = nvmap_client_create_fd(client);
		if (fd < 0)
			return fd;
		nvmap_dmabuf_install_fd(dmabuf, fd);
	} else {

		fd = nvmap_client_create_fd(client);
		if (fd < 0)
			return fd;

		nvmap_handle_install_fd(handle, fd);
	}

	op.fd = fd;
	if (copy_to_user(arg, &op, sizeof(op))) {
		put_unused_fd(op.fd);
		return -EFAULT;
	}

	return 0;
}

static int vaddr_and_size_are_in_vma(ulong vaddr, size_t size,
					struct vm_area_struct *vma)
{
	if (!vma) {
		return 0;
	}

	if (vaddr >= vma->vm_start && vaddr + size <= vma->vm_end) {
		return 1;
	} else {
		return 0;
	}
}

int nvmap_ioctl_create_from_va(struct file *filp, void __user *arg)
{
	int fd;
	int err;
	struct nvmap_create_handle_from_va op;
	struct nvmap_handle *handle = NULL;
	struct nvmap_client *client = filp->private_data;
	struct vm_area_struct *vma;
	size_t size;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	/* don't allow non-page aligned addresses. */
	if (op.va & ~PAGE_MASK)
		return -EINVAL;

	if (!client)
		return -ENODEV;

	vma = find_vma(current->mm, op.va);
	size = (op.size) ? op.size: op.size64;

	if (!vaddr_and_size_are_in_vma(op.va, size, vma)) {
		return -EINVAL;
	}

	if (!size) {
		size = vma->vm_end - op.va;
	}

	fd = nvmap_client_create_handle(client, size);
	if (fd < 0) {
		return -EFAULT;
	}

	handle = nvmap_handle_from_fd(fd);
	if (IS_ERR_OR_NULL(handle)) {
		return -EFAULT;
	}

	nvmap_client_warn_if_no_tag(client, op.flags);
	err = nvmap_handle_alloc_from_va(handle, op.va, op.flags);

	NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(client, handle, NULL));
	if (err) {
		nvmap_ioctl_free(filp, fd);
		return err;
	}

	op.handle = fd;
	if (copy_to_user(arg, &op, sizeof(op))) {
		nvmap_ioctl_free(filp, fd);
		return err;
	}

	return 0;
}

int nvmap_ioctl_free(struct file *filp, unsigned long fd)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *handle;

	if (!fd)
		return 0;

	handle  = nvmap_handle_from_fd(fd);
	if (!IS_ERR_OR_NULL(handle)) {
		nvmap_client_remove_handle(client, handle);
	}

	return sys_close(fd);
}

int nvmap_ioctl_alloc(struct file *filp, unsigned int cmd, void __user *arg)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *h;
	unsigned int heap_mask;
	unsigned int align;
	unsigned int flags;
	int peer;
	int err;
	int handle;

	if (cmd == NVMAP_IOC_ALLOC) {
		struct nvmap_alloc_handle op;

		if (copy_from_user(&op, arg, sizeof(op)))
			return -EFAULT;

		peer = NVMAP_IVM_INVALID_PEER;
		align = op.align;
		heap_mask = op.heap_mask;
		handle = op.handle;
		flags = op.flags;
	} else if (cmd == NVMAP_IOC_ALLOC_IVM) {
		struct nvmap_alloc_ivm_handle op;

		if (copy_from_user(&op, arg, sizeof(op)))
			return -EFAULT;
		peer = op.peer;

		align = op.align;
		heap_mask = op.heap_mask;
		handle = op.handle;
		flags = op.flags;
	} else {
		return -EINVAL;
	}

	if (align & (align - 1))
		return -EINVAL;

	h = nvmap_handle_from_fd(handle);
	if (!h)
		return -EINVAL;
	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;
	if (nvmap_handle_is_allocated(h)) {
		nvmap_handle_put(h);
		return -EEXIST;
	}


	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	align = max_t(size_t, align, PAGE_SIZE);

	nvmap_client_warn_if_no_tag(client, flags);

	err = nvmap_handle_alloc(h, heap_mask, align,
				  0, /* no kind */
				  flags & (~NVMAP_HANDLE_KIND_SPECIFIED),
				  peer);

	if (nvmap_handle_is_allocated(h)) {
		size_t size = nvmap_handle_size(h);

		nvmap_stats_inc(NS_TOTAL, size);
		nvmap_stats_inc(NS_ALLOC, size);

		nvmap_client_stats_alloc(client, size);

		NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(client, h, NULL));
		err = 0;
	}

	nvmap_handle_put(h);
	return err;
}

struct nvmap_handle *nvmap_try_duplicate_by_ivmid(
		struct nvmap_client *client, u64 ivm_id)
{
	struct nvmap_handle *handle = NULL;

	handle = nvmap_handle_from_ivmid(ivm_id);
	handle = nvmap_handle_get(handle);
	if (!handle)
		return NULL;

	nvmap_client_add_handle(client, handle);

	return handle;
}

static int ioctl_alloc_handle_by_ivmid(struct nvmap_client *client, u64 ivm_id)
{
	struct nvmap_handle *handle = NULL;
	size_t size = 0;
	int err;
	int fd;

	size = nvmap_ivmid_to_size(ivm_id);

	fd = nvmap_client_create_handle(client, size);
	if (fd < 0) {
		return -1;
	}

	handle = nvmap_handle_from_fd(fd);
	if (!handle) {
		return -1;
	}

	err = nvmap_handle_alloc_from_ivmid(handle, ivm_id);
	if (!err) {
		// TODO: Make sure client create/remove frees handle
		nvmap_client_remove_handle(client, handle);
		return -1;
	}
	NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(NULL, handle, NULL));
	return fd;

}

static int ioctl_handle_from_ivmid(struct nvmap_client *client, int ivm_id)
{
	struct nvmap_handle *handle;
	int fd;

	handle = nvmap_handle_from_ivmid(ivm_id);
	if (!handle)
		return -1;

	fd = nvmap_client_create_fd(client);
	if (fd >= 0)
		nvmap_handle_install_fd(handle, fd);

	return fd;
}

int nvmap_ioctl_create_from_ivc(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;
	int fd;
	int err = 0;

	/* First create a new handle and then fake carveout allocation */
	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	fd = ioctl_handle_from_ivmid(client, op.ivm_id);
	if (fd < 0) {
		fd = ioctl_alloc_handle_by_ivmid(client, op.ivm_id);
	}

	if (fd < 0) {
		err = -1;
		goto fail;
	}

	op.ivm_handle = fd;

	if (copy_to_user(arg, &op, sizeof(op))) {
		err = -EFAULT;
		// TODO: What do we do here to free
		goto fail;
	}

	return err;

fail:
	// TODO: Find a way to free the handle here
	// Needs to make sure it covers case where FD wasn't nvmap fd
	pr_warn("Need to free handle here!\n");
	return err;
}

int nvmap_ioctl_vpr_floor_size(struct file *filp, void __user *arg)
{
	int err=0;
	u32 floor_size;

	if (copy_from_user(&floor_size, arg, sizeof(floor_size)))
		return -EFAULT;

	err = dma_set_resizable_heap_floor_size(&tegra_vpr_dev, floor_size);
	return err;
}

int nvmap_ioctl_get_ivc_heap(struct file *filp, void __user *arg)
{
	struct nvmap_device *dev = nvmap_dev;
	int i;
	unsigned int heap_mask = 0;

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *co_heap = nvmap_dev_to_carveout(dev, i);
		int peer;

		if (!nvmap_carveout_is_ivm(co_heap))
			continue;

		peer = nvmap_carveout_query_peer(co_heap);
		if (peer < 0)
			return -EINVAL;

		heap_mask |= BIT(peer);
	}

	if (copy_to_user(arg, &heap_mask, sizeof(heap_mask)))
		return -EFAULT;

	return 0;
}

int nvmap_ioctl_get_ivcid(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle *h = NULL;

	if (copy_from_user(&op, arg, sizeof(op))) {
		return -EFAULT;
	}

	h = nvmap_handle_from_fd(op.ivm_handle);
	if (!h) {
		return -EINVAL;
	}

	if (!nvmap_handle_is_allocated(h)) {
		return -EFAULT;
	}

	op.ivm_id = nvmap_handle_ivm_id(h);

	return copy_to_user(arg, &op, sizeof(op)) ? -EFAULT : 0;
}

static int ioctl_cache_maint_copy_op_from_user(void __user *arg,
				struct nvmap_cache_op_64 *op64, int op_size)
{
#ifdef CONFIG_COMPAT
	if (op_size == sizeof(struct nvmap_cache_op_32)) {
		struct nvmap_cache_op_32 op32;

		if (copy_from_user(&op32, arg, sizeof(op32)))
			return -EFAULT;

		op64->addr = op32.addr;
		op64->handle = op32.handle;
		op64->len = op32.len;
		op64->op = op32.op;
		return 0;
	}
#endif
	if (op_size == sizeof(struct nvmap_cache_op)) {
		struct nvmap_cache_op op;

		if (copy_from_user(&op, arg, sizeof(op)))
			return -EFAULT;
		op64->addr = op.addr;
		op64->handle = op.handle;
		op64->len = op.len;
		op64->op = op.op;
		return 0;
	}

	if (copy_from_user(&op64, arg, sizeof(op64)))
		return -EFAULT;
	return 0;
}
int nvmap_ioctl_cache_maint(struct file *filp, void __user *arg, int op_size)
{
	struct vm_area_struct *vma;
	struct nvmap_handle *handle;
	struct nvmap_cache_op_64 op;
	int ret = 0;
	unsigned long start;
	unsigned long end;

	ret = ioctl_cache_maint_copy_op_from_user(arg, &op, op_size);
	if (ret)
		return ret;

	handle = nvmap_handle_from_fd(op.handle);
	handle = nvmap_handle_get(handle);
	if (!handle)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->mm, (unsigned long) op.addr);
	if (!nvmap_vma_is_nvmap(vma) ||
			!vaddr_and_size_are_in_vma(op.addr, op.len, vma)) {
		ret = -EADDRNOTAVAIL;
		goto out;
	}

	if (!nvmap_handle_owns_vma(handle, vma)) {
		ret = -EFAULT;
		goto out;
	}

	start = (unsigned long)op.addr - vma->vm_start +
		(vma->vm_pgoff << PAGE_SHIFT);
	end = start + op.len;

	ret = nvmap_handle_cache_maint(handle, start, end, op.op);


out:
	up_read(&current->mm->mmap_sem);
	nvmap_handle_put(handle);
	return ret;
}

/*
 * This function copies the rw_handle arguments into op.
 * Returns 0 if passing, err if failing
 */
static int ioctl_rw_handle_copy_op_from_user(void __user *arg,
				struct nvmap_rw_handle *op, int op_size)
{
	struct nvmap_rw_handle_64 op64;
#ifdef CONFIG_COMPAT
	struct nvmap_rw_handle_32 op32;
#endif

#ifdef CONFIG_COMPAT
	if (op_size == sizeof(op32)) {
		if (copy_from_user(&op32, arg, sizeof(op32)))
			return -EFAULT;
		op->addr = op32.addr;
		op->handle = op32.handle;
		op->offset = op32.offset;
		op->elem_size = op32.elem_size;
		op->hmem_stride = op32.hmem_stride;
		op->user_stride = op32.user_stride;
		op->count = op32.count;
		return 0;
	}
#endif
	if (op_size == sizeof(*op)) {
		if (copy_from_user(op, arg, sizeof(*op)))
			return -EFAULT;
		return 0;
	}

	if (op_size == sizeof(op64)) {
		if (copy_from_user(&op64, arg, sizeof(op64)))
			return -EFAULT;
		op->addr = op64.addr;
		op->handle = op64.handle;
		op->offset = op64.offset;
		op->elem_size = op64.elem_size;
		op->hmem_stride = op64.hmem_stride;
		op->user_stride = op64.user_stride;
		op->count = op64.count;
		return 0;
	}

	pr_warn("nvmap: rw_handle copy size failed\n");
	return -EINVAL;
}

static void ioctl_rw_handle_copy_arg_to_user(void __user *arg,
					ssize_t copied, int op_size)
{
	struct nvmap_rw_handle __user *uarg = arg;
	struct nvmap_rw_handle_64 __user *uarg64 = arg;
#ifdef CONFIG_COMPAT
	struct nvmap_rw_handle_32 __user *uarg32 = arg;
#endif

#ifdef CONFIG_COMPAT
	if (op_size == sizeof(struct nvmap_rw_handle_32)) {
		__put_user(copied, &uarg32->count);
		return;
	}
#endif
	if (op_size == sizeof(struct nvmap_rw_handle)) {
		__put_user(copied, &uarg->count);
		return;
	}

	if (op_size == sizeof(struct nvmap_rw_handle_64)) {
		__put_user(copied, &uarg64->count);
		return;
	}

	pr_warn("nvmap: rw_handle copy to uarg failed\n");
}

/*
 * TODO: Move this function to a better location
 */
static int set_vpr_fail_data(void *user_addr, ulong user_stride,
		       ulong elem_size, ulong count)
{
	int ret = 0;
	void *vaddr;

	vaddr = vmalloc(PAGE_SIZE);
	if (!vaddr)
		return -ENOMEM;
	memset(vaddr, 0xFF, PAGE_SIZE);

	while (!ret && count--) {
		ulong size_to_copy = elem_size;

		while (!ret && size_to_copy) {
			ret = copy_to_user(user_addr, vaddr,
				size_to_copy > PAGE_SIZE ? PAGE_SIZE : size_to_copy);
			size_to_copy -= (size_to_copy > PAGE_SIZE ? PAGE_SIZE : size_to_copy);
		}
		user_addr += user_stride;
	}

	vfree(vaddr);
	return ret;
}

int nvmap_ioctl_rw_handle(struct file *filp, int is_read, void __user *arg,
			  size_t op_size)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *h;
	ssize_t copied;
	struct nvmap_rw_handle op;
	unsigned long addr, offset, elem_size, hmem_stride, user_stride;
	unsigned long count;
	int fd;
	int err = 0;

	err = ioctl_rw_handle_copy_op_from_user(arg, &op, op_size);
	if (err)
		return err;

	addr = op.addr;
	fd = op.handle;
	offset = op.offset;
	elem_size = op.elem_size;
	hmem_stride = op.hmem_stride;
	user_stride = op.user_stride;
	count = op.count;


	if (!addr || !count || !elem_size)
		return -EINVAL;

	h = nvmap_handle_from_fd(fd);
	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;

	if (is_read && soc_is_tegra186_n_later() &&
		nvmap_handle_heap_type(h) == NVMAP_HEAP_CARVEOUT_VPR) {
		int ret;

		/* VPR memory is not readable from CPU.
		 * Memset buffer to all 0xFF's for backward compatibility. */
		ret = set_vpr_fail_data((void *)addr, user_stride, elem_size, count);
		nvmap_handle_put(h);

		if (ret == 0)
			ret = -EPERM;

		return ret;
	}

	nvmap_handle_kmap_inc(h);
	trace_nvmap_ioctl_rw_handle(client, h, is_read, offset,
				    addr, hmem_stride,
				    user_stride, elem_size, count);
	copied = nvmap_handle_rw(h,
				offset, hmem_stride,
				addr, user_stride,
				elem_size, count,
				is_read);
	nvmap_handle_kmap_dec(h);

	if (copied < 0) {
		err = copied;
		copied = 0;
	} else if (copied < (count * elem_size))
		err = -EINTR;

	ioctl_rw_handle_copy_arg_to_user(arg, copied, op_size);

	nvmap_handle_put(h);

	return err;
}

extern struct device tegra_vpr_dev;

int nvmap_ioctl_gup_test(struct file *filp, void __user *arg)
{
	int err = -EINVAL;
	struct nvmap_gup_test op;
	struct vm_area_struct *vma;
	struct nvmap_handle *handle;
	int nr_page;
	struct page **pages;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	op.result = 1;
	vma = find_vma(current->mm, op.va);
	if (unlikely(!vma) || (unlikely(op.va < vma->vm_start )) ||
	    unlikely(op.va >= vma->vm_end))
		goto exit;

	handle = nvmap_handle_from_fd(op.handle);
	nvmap_handle_get(handle);
	if (!handle)
		goto exit;

	if (vma->vm_end - vma->vm_start != nvmap_handle_size(handle)) {
		pr_err("handle size(0x%zx) and vma size(0x%lx) don't match\n",
			 nvmap_handle_size(handle), vma->vm_end - vma->vm_start);
		goto put_handle;
	}

	err = -ENOMEM;
	nr_page = nvmap_handle_size(handle) >> PAGE_SHIFT;
	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (IS_ERR_OR_NULL(pages)) {
		err = PTR_ERR(pages);
		goto put_handle;
	}

	err = nvmap_get_user_pages(op.va & PAGE_MASK, nr_page, pages);
	if (err)
		goto put_user_pages;

	// TODO: Find an easy way to fix this
	/*
	for (i = 0; i < nr_page; i++) {
		if (handle->pgalloc.pages[i] != pages[i]) {
			pr_err("page pointers don't match, %p %p\n",
			       handle->pgalloc.pages[i], pages[i]);
			op.result = 0;
		}
	}
	*/

	if (op.result)
		err = 0;

	if (copy_to_user(arg, &op, sizeof(op)))
		err = -EFAULT;

put_user_pages:
	nvmap_altfree(pages, nr_page * sizeof(*pages));
put_handle:
	nvmap_handle_put(handle);
exit:
	pr_info("GUP Test %s\n", err ? "failed" : "passed");
	return err;
}

int nvmap_ioctl_set_tag_label(struct file *filp, void __user *arg)
{
	struct nvmap_set_tag_label op;
	struct nvmap_device *dev = nvmap_dev;
	int err;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (op.len > NVMAP_TAG_LABEL_MAXLEN)
		op.len = NVMAP_TAG_LABEL_MAXLEN;

	if (op.len)
		err = nvmap_define_tag(dev, op.tag,
			(const char __user *)op.addr, op.len);
	else
		err = nvmap_remove_tag(dev, op.tag);

	return err;
}

int nvmap_ioctl_get_available_heaps(struct file *filp, void __user *arg)
{
	struct nvmap_available_heaps op;
	int i;

	memset(&op, 0, sizeof(op));

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *carveout =
					nvmap_dev_to_carveout(nvmap_dev, i);

		op.heaps |= nvmap_carveout_heap_bit(carveout);
	}

	if (copy_to_user(arg, &op, sizeof(op))) {
		pr_err("copy_to_user failed\n");
		return -EINVAL;
	}

	return 0;
}

int nvmap_ioctl_get_heap_size(struct file *filp, void __user *arg)
{
	struct nvmap_heap_size op;
	int i;
	memset(&op, 0, sizeof(op));

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *carveout =
					nvmap_dev_to_carveout(nvmap_dev, i);

		if (op.heap & nvmap_carveout_heap_bit(carveout)) {
			op.size = nvmap_carveout_query_heap_size(carveout);
			if (copy_to_user(arg, &op, sizeof(op)))
				return -EFAULT;
			return 0;
		}
	}
	return -ENODEV;

}
