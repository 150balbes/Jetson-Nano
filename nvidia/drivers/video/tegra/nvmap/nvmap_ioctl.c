/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.c
 *
 * User-space interface to nvmap
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
#include <linux/mm.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <asm/uaccess.h>
#include <soc/tegra/common.h>
#include <trace/events/nvmap.h>

#include "nvmap_ioctl.h"
#include "nvmap_priv.h"
#include "nvmap_heap.h"


extern struct device tegra_vpr_dev;

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count);

static bool is_allocation_possible(size_t size)
{
	struct sysinfo i;

	si_meminfo(&i);

	if (size >> PAGE_SHIFT >= i.totalram) {
		pr_debug("Requested allocation size is more than system memory");
		return false;
	}
	return true;
}

/* NOTE: Callers of this utility function must invoke nvmap_handle_put after
 * using the returned nvmap_handle.
 */
struct nvmap_handle *nvmap_handle_get_from_fd(int fd)
{
	struct nvmap_handle *h;

	h = nvmap_handle_get_from_dmabuf_fd(NULL, fd);
	if (!IS_ERR(h))
		return h;
	return NULL;
}

static int nvmap_install_fd(struct nvmap_client *client,
	struct nvmap_handle *handle, int fd, void __user *arg,
	void *op, size_t op_size, bool free, struct dma_buf *dmabuf)
{
	int err = 0;

	if (IS_ERR_VALUE((uintptr_t)fd)) {
		err = fd;
		goto fd_fail;
	}

	if (copy_to_user(arg, op, op_size)) {
		err = -EFAULT;
		goto copy_fail;
	}

	fd_install(fd, dmabuf->file);
	return err;

copy_fail:
	put_unused_fd(fd);
fd_fail:
	if (dmabuf)
		dma_buf_put(dmabuf);
	if (free && handle)
		nvmap_free_handle(client, handle);
	return err;
}

int nvmap_ioctl_getfd(struct file *filp, void __user *arg)
{
	struct nvmap_handle *handle;
	struct nvmap_create_handle op;
	struct nvmap_client *client = filp->private_data;
	struct dma_buf *dmabuf;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	handle = nvmap_handle_get_from_fd(op.handle);
	if (handle) {
		op.fd = nvmap_get_dmabuf_fd(client, handle);
		nvmap_handle_put(handle);
		dmabuf = IS_ERR_VALUE((uintptr_t)op.fd) ? NULL : handle->dmabuf;
	} else {
		/* if we get an error, the fd might be non-nvmap dmabuf fd */
		dmabuf = dma_buf_get(op.handle);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		op.fd = nvmap_dmabuf_duplicate_gen_fd(client, dmabuf);
	}

	return nvmap_install_fd(client, handle,
				op.fd, arg, &op, sizeof(op), 0, dmabuf);
}

int nvmap_ioctl_alloc(struct file *filp, void __user *arg)
{
	struct nvmap_alloc_handle op;
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *handle;
	int err;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (op.align & (op.align - 1))
		return -EINVAL;

	handle = nvmap_handle_get_from_fd(op.handle);
	if (!handle)
		return -EINVAL;

	if (!is_allocation_possible(handle->size))
		return -ENOMEM;

	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	op.align = max_t(size_t, op.align, PAGE_SIZE);

	err = nvmap_alloc_handle(client, handle, op.heap_mask, op.align,
				  0, /* no kind */
				  op.flags & (~NVMAP_HANDLE_KIND_SPECIFIED),
				  NVMAP_IVM_INVALID_PEER);
	nvmap_handle_put(handle);
	return err;
}

int nvmap_ioctl_alloc_ivm(struct file *filp, void __user *arg)
{
	struct nvmap_alloc_ivm_handle op;
	struct nvmap_client *client = filp->private_data;
	struct nvmap_handle *handle;
	int err;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (op.align & (op.align - 1))
		return -EINVAL;

	handle = nvmap_handle_get_from_fd(op.handle);
	if (!handle)
		return -EINVAL;

	/* user-space handles are aligned to page boundaries, to prevent
	 * data leakage. */
	op.align = max_t(size_t, op.align, PAGE_SIZE);

	err = nvmap_alloc_handle(client, handle, op.heap_mask, op.align,
				  0, /* no kind */
				  op.flags & (~NVMAP_HANDLE_KIND_SPECIFIED),
				  op.peer);
	nvmap_handle_put(handle);
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

int nvmap_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle_ref *ref = NULL;
	struct nvmap_client *client = filp->private_data;
	struct dma_buf *dmabuf = NULL;
	struct nvmap_handle *handle = NULL;
	int fd;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	if (cmd == NVMAP_IOC_CREATE)
		op.size64 = op.size;

	if ((cmd == NVMAP_IOC_CREATE) || (cmd == NVMAP_IOC_CREATE_64)) {
		ref = nvmap_create_handle(client, op.size64);
		if (!IS_ERR(ref))
			ref->handle->orig_size = op.size64;
	} else if (cmd == NVMAP_IOC_FROM_FD) {
		ref = nvmap_create_handle_from_fd(client, op.fd);

		/* if we get an error, the fd might be non-nvmap dmabuf fd */
		if (IS_ERR(ref)) {
			dmabuf = dma_buf_get(op.fd);
			if (IS_ERR(dmabuf))
				return PTR_ERR(dmabuf);
			fd = nvmap_dmabuf_duplicate_gen_fd(client, dmabuf);
			if (fd < 0)
				return fd;
		}
	} else {
		return -EINVAL;
	}

	if (!IS_ERR(ref)) {
		handle = ref->handle;
		dmabuf = handle->dmabuf;
		fd = nvmap_get_dmabuf_fd(client, ref->handle);
	} else if (!dmabuf) {
		return PTR_ERR(ref);
	}

	if (cmd == NVMAP_IOC_CREATE_64)
		op.handle64 = fd;
	else
		op.handle = fd;
	return nvmap_install_fd(client, handle, fd,
				arg, &op, sizeof(op), 1, dmabuf);
}

int nvmap_ioctl_create_from_va(struct file *filp, void __user *arg)
{
	int fd;
	int err;
	struct nvmap_create_handle_from_va op;
	struct nvmap_handle_ref *ref = NULL;
	struct nvmap_client *client = filp->private_data;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	ref = nvmap_create_handle_from_va(client, op.va,
			op.size ? op.size : op.size64);
	if (IS_ERR(ref))
		return PTR_ERR(ref);

	err = nvmap_alloc_handle_from_va(client, ref->handle,
					 op.va, op.flags);
	if (err) {
		nvmap_free_handle(client, ref->handle);
		return err;
	}

	fd = nvmap_get_dmabuf_fd(client, ref->handle);
	op.handle = fd;
	return nvmap_install_fd(client, ref->handle, fd,
			arg, &op, sizeof(op), 1,  ref->handle->dmabuf);
}

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
	struct nvmap_rw_handle_64 __user *uarg64 = arg;
	struct nvmap_rw_handle_64 op64;
	struct nvmap_rw_handle __user *uarg = arg;
	struct nvmap_rw_handle op;
#ifdef CONFIG_COMPAT
	struct nvmap_rw_handle_32 __user *uarg32 = arg;
	struct nvmap_rw_handle_32 op32;
#endif
	struct nvmap_handle *h;
	ssize_t copied;
	int err = 0;
	unsigned long addr, offset, elem_size, hmem_stride, user_stride;
	unsigned long count;
	int handle;
	int ret;

#ifdef CONFIG_COMPAT
	if (op_size == sizeof(op32)) {
		if (copy_from_user(&op32, arg, sizeof(op32)))
			return -EFAULT;
		addr = op32.addr;
		handle = op32.handle;
		offset = op32.offset;
		elem_size = op32.elem_size;
		hmem_stride = op32.hmem_stride;
		user_stride = op32.user_stride;
		count = op32.count;
	} else
#endif
	{
		if (op_size == sizeof(op)) {
			if (copy_from_user(&op, arg, sizeof(op)))
				return -EFAULT;
			addr = op.addr;
			handle = op.handle;
			offset = op.offset;
			elem_size = op.elem_size;
			hmem_stride = op.hmem_stride;
			user_stride = op.user_stride;
			count = op.count;
		} else {
			if (copy_from_user(&op64, arg, sizeof(op64)))
				return -EFAULT;
			addr = op64.addr;
			handle = op64.handle;
			offset = op64.offset;
			elem_size = op64.elem_size;
			hmem_stride = op64.hmem_stride;
			user_stride = op64.user_stride;
			count = op64.count;
		}
	}

	if (!addr || !count || !elem_size)
		return -EINVAL;

	h = nvmap_handle_get_from_fd(handle);
	if (!h)
		return -EINVAL;

	if (is_read && soc_is_tegra186_n_later() &&
		h->heap_type == NVMAP_HEAP_CARVEOUT_VPR) {
		/* VPR memory is not readable from CPU.
		 * Memset buffer to all 0xFF's for backward compatibility. */
		ret = set_vpr_fail_data((void *)addr, user_stride, elem_size, count);
		nvmap_handle_put(h);
		return ret ?: -EPERM;
	}

	nvmap_kmaps_inc(h);
	trace_nvmap_ioctl_rw_handle(client, h, is_read, offset,
				    addr, hmem_stride,
				    user_stride, elem_size, count);
	copied = rw_handle(client, h, is_read, offset,
			   addr, hmem_stride,
			   user_stride, elem_size, count);
	nvmap_kmaps_dec(h);

	if (copied < 0) {
		err = copied;
		copied = 0;
	} else if (copied < (count * elem_size))
		err = -EINTR;

#ifdef CONFIG_COMPAT
	if (op_size == sizeof(op32))
		__put_user(copied, &uarg32->count);
	else
#endif
		if (op_size == sizeof(op))
			__put_user(copied, &uarg->count);
		else
			__put_user(copied, &uarg64->count);

	nvmap_handle_put(h);

	return err;
}

int nvmap_ioctl_cache_maint(struct file *filp, void __user *arg, int op_size)
{
	struct nvmap_client *client = filp->private_data;
	struct nvmap_cache_op op;
	struct nvmap_cache_op_64 op64;
#ifdef CONFIG_COMPAT
	struct nvmap_cache_op_32 op32;
#endif

#ifdef CONFIG_COMPAT
	if (op_size == sizeof(op32)) {
		if (copy_from_user(&op32, arg, sizeof(op32)))
			return -EFAULT;
		op64.addr = op32.addr;
		op64.handle = op32.handle;
		op64.len = op32.len;
		op64.op = op32.op;
	} else
#endif
	{
		if (op_size == sizeof(op)) {
			if (copy_from_user(&op, arg, sizeof(op)))
				return -EFAULT;
			op64.addr = op.addr;
			op64.handle = op.handle;
			op64.len = op.len;
			op64.op = op.op;
		} else {
			if (copy_from_user(&op64, arg, sizeof(op64)))
				return -EFAULT;
		}
	}

	return __nvmap_cache_maint(client, &op64);
}

int nvmap_ioctl_free(struct file *filp, unsigned long arg)
{
	struct nvmap_client *client = filp->private_data;

	if (!arg)
		return 0;

	nvmap_free_handle_fd(client, arg);
	return sys_close(arg);
}

static ssize_t rw_handle(struct nvmap_client *client, struct nvmap_handle *h,
			 int is_read, unsigned long h_offs,
			 unsigned long sys_addr, unsigned long h_stride,
			 unsigned long sys_stride, unsigned long elem_size,
			 unsigned long count)
{
	ssize_t copied = 0;
	void *addr;
	int ret = 0;

	if (!(h->heap_type & nvmap_dev->cpu_access_mask))
		return -EPERM;

	if (!elem_size || !count)
		return -EINVAL;

	if (!h->alloc)
		return -EFAULT;

	if (elem_size == h_stride && elem_size == sys_stride && (h_offs % 8 == 0)) {
		elem_size *= count;
		h_stride = elem_size;
		sys_stride = elem_size;
		count = 1;
	}

	if (elem_size > h->size ||
		h_offs >= h->size ||
		elem_size > sys_stride ||
		elem_size > h_stride ||
		sys_stride > (h->size - h_offs) / count ||
		h_offs + h_stride * (count - 1) + elem_size > h->size)
		return -EINVAL;

	if (!h->vaddr) {
		if (!__nvmap_mmap(h))
			return -ENOMEM;
		__nvmap_munmap(h, h->vaddr);
	}

	addr = h->vaddr + h_offs;

	while (count--) {
		if (h_offs + elem_size > h->size) {
			pr_warn("read/write outside of handle\n");
			ret = -EFAULT;
			break;
		}
		if (is_read &&
		    !(h->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
			__nvmap_do_cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_INV, false);

		if (is_read)
			ret = copy_to_user((void *)sys_addr, addr, elem_size);
		else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
			if (h->heap_type == NVMAP_HEAP_CARVEOUT_VPR) {
				uaccess_enable();
				memcpy_toio(addr, (void *)sys_addr, elem_size);
				uaccess_disable();
				ret = 0;
			} else
#endif
				ret = copy_from_user(addr, (void *)sys_addr, elem_size);
		}

		if (ret)
			break;

		if (!is_read &&
		    !(h->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
			__nvmap_do_cache_maint(client, h, h_offs,
				h_offs + elem_size, NVMAP_CACHE_OP_WB_INV,
				false);

		copied += elem_size;
		sys_addr += sys_stride;
		h_offs += h_stride;
		addr += h_stride;
	}

	return ret ?: copied;
}

int nvmap_ioctl_get_ivcid(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle *h = NULL;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	h = nvmap_handle_get_from_fd(op.ivm_handle);
	if (!h)
		return -EINVAL;

	if (!h->alloc) { /* || !h->ivm_id) { */
		nvmap_handle_put(h);
		return -EFAULT;
	}

	op.ivm_id = h->ivm_id;

	nvmap_handle_put(h);

	return copy_to_user(arg, &op, sizeof(op)) ? -EFAULT : 0;
}

int nvmap_ioctl_get_ivc_heap(struct file *filp, void __user *arg)
{
	struct nvmap_device *dev = nvmap_dev;
	int i;
	unsigned int heap_mask = 0;

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_carveout_node *co_heap = &dev->heaps[i];
		int peer;

		if (!(co_heap->heap_bit & NVMAP_HEAP_CARVEOUT_IVM))
			continue;

		peer = nvmap_query_heap_peer(co_heap->carveout);
		if (peer < 0)
			return -EINVAL;

		heap_mask |= BIT(peer);
	}

	if (copy_to_user(arg, &heap_mask, sizeof(heap_mask)))
		return -EFAULT;

	return 0;
}

int nvmap_ioctl_create_from_ivc(struct file *filp, void __user *arg)
{
	struct nvmap_create_handle op;
	struct nvmap_handle_ref *ref;
	struct nvmap_client *client = filp->private_data;
	int fd;
	phys_addr_t offs;
	size_t size = 0;
	int peer;
	struct nvmap_heap_block *block = NULL;

	/* First create a new handle and then fake carveout allocation */
	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!client)
		return -ENODEV;

	ref = nvmap_try_duplicate_by_ivmid(client, op.ivm_id, &block);
	if (!ref) {
		/*
		 * See nvmap_heap_alloc() for encoding details.
		 */
		offs = ((op.ivm_id &
		       ~((u64)NVMAP_IVM_IVMID_MASK << NVMAP_IVM_IVMID_SHIFT)) >>
			NVMAP_IVM_LENGTH_WIDTH) << (ffs(NVMAP_IVM_ALIGNMENT) - 1);
		size = (op.ivm_id &
			((1ULL << NVMAP_IVM_LENGTH_WIDTH) - 1)) << PAGE_SHIFT;
		peer = (op.ivm_id >> NVMAP_IVM_IVMID_SHIFT);

		ref = nvmap_create_handle(client, size);
		if (IS_ERR(ref)) {
			nvmap_heap_free(block);
			return PTR_ERR(ref);
		}
		ref->handle->orig_size = size;

		ref->handle->peer = peer;
		if (!block)
			block = nvmap_carveout_alloc(client, ref->handle,
					NVMAP_HEAP_CARVEOUT_IVM, &offs);
		if (!block) {
			nvmap_free_handle(client, ref->handle);
			return -ENOMEM;
		}

		ref->handle->heap_type = NVMAP_HEAP_CARVEOUT_IVM;
		ref->handle->heap_pgalloc = false;
		ref->handle->ivm_id = op.ivm_id;
		ref->handle->carveout = block;
		block->handle = ref->handle;
		mb();
		ref->handle->alloc = true;
		NVMAP_TAG_TRACE(trace_nvmap_alloc_handle_done,
			NVMAP_TP_ARGS_CHR(client, ref->handle, ref));
	}

	fd = nvmap_get_dmabuf_fd(client, ref->handle);
	op.ivm_handle = fd;
	return nvmap_install_fd(client, ref->handle, fd,
				arg, &op, sizeof(op), 1, ref->handle->dmabuf);
}

int nvmap_ioctl_cache_maint_list(struct file *filp, void __user *arg,
				 bool is_reserve_ioctl)
{
	struct nvmap_cache_op_list op;
	u32 *handle_ptr;
	u64 *offset_ptr;
	u64 *size_ptr;
	struct nvmap_handle **refs;
	int err = 0;
	u32 i, n_unmarshal_handles = 0, count = 0;
	size_t bytes;
	size_t elem_size;
	bool is_32;

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	if (!op.nr || op.nr > UINT_MAX / sizeof(u32))
		return -EINVAL;

	bytes = op.nr * sizeof(*refs);
	if (!access_ok(VERIFY_READ, op.handles, op.nr * sizeof(u32)))
		return -EFAULT;

	elem_size  = (op.op & NVMAP_ELEM_SIZE_U64) ?
			sizeof(u64) : sizeof(u32);
	op.op &= ~NVMAP_ELEM_SIZE_U64;
	is_32 = elem_size == sizeof(u32) ? 1 : 0;

	bytes += 2 * op.nr * elem_size;
	bytes += op.nr * sizeof(u32);
	refs = nvmap_altalloc(bytes);
	if (!refs) {
		pr_err("memory allocation failed\n");
		return -ENOMEM;
	}

	offset_ptr = (u64 *)(refs + op.nr);
	size_ptr = (u64 *)(((uintptr_t)offset_ptr) + op.nr * elem_size);
	handle_ptr = (u32 *)(((uintptr_t)size_ptr) + op.nr * elem_size);

	if (!op.handles || !op.offsets || !op.sizes) {
		pr_err("pointers are invalid\n");
		return -EINVAL;
	}

	if (!IS_ALIGNED((ulong)offset_ptr, elem_size) ||
	    !IS_ALIGNED((ulong)size_ptr, elem_size) ||
	    !IS_ALIGNED((ulong)handle_ptr, sizeof(u32))) {
		pr_err("pointers are not properly aligned!!\n");
		return -EINVAL;
	}

	if (copy_from_user(handle_ptr, (void *)op.handles,
		op.nr * sizeof(u32))) {
		pr_err("Can't copy from user pointer op.handles\n");
		return -EFAULT;
	}

	if (copy_from_user(offset_ptr, (void *)op.offsets,
		op.nr * elem_size)) {
		pr_err("Can't copy from user pointer op.offsets\n");
		return -EFAULT;
	}

	if (copy_from_user(size_ptr, (void *)op.sizes,
		op.nr * elem_size)) {
		pr_err("Can't copy from user pointer op.sizes\n");
		return -EFAULT;
	}

	for (i = 0; i < op.nr; i++) {
		refs[i] = nvmap_handle_get_from_fd(handle_ptr[i]);
		if (!refs[i]) {
			pr_err("invalid handle_ptr[%d] = %u\n",
				i, handle_ptr[i]);
			err = -EINVAL;
			goto free_mem;
		}
		if (!(refs[i]->heap_type & nvmap_dev->cpu_access_mask)) {
			pr_err("heap %x can't be accessed from cpu\n",
				refs[i]->heap_type);
			err = -EPERM;
			goto free_mem;
		}

		n_unmarshal_handles++;
	}

	/*
	 * Either all handles should have NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE
	 * or none should have it.
	 */
	for (i = 0; i < op.nr; i++)
		if (refs[i]->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE)
			count++;

	if (op.nr && count % op.nr) {
		pr_err("incorrect CACHE_SYNC_AT_RESERVE mix of handles\n");
		err = -EINVAL;
		goto free_mem;
	}

	/*
	 * when NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE is specified mix can cause
	 * cache WB_INV at unreserve op on iovmm handles increasing overhead.
	 * So, either all handles should have pages from carveout or from iovmm.
	 */
	if (count) {
		for (i = 0; i < op.nr; i++)
			if (refs[i]->heap_pgalloc)
				count++;

		if (op.nr && count % op.nr) {
			pr_err("all or none of the handles should be from heap\n");
			err = -EINVAL;
			goto free_mem;
		}
	}

	if (is_reserve_ioctl)
		err = nvmap_reserve_pages(refs, offset_ptr, size_ptr,
					  op.nr, op.op, is_32);
	else
		err = nvmap_do_cache_maint_list(refs, offset_ptr, size_ptr,
						op.op, op.nr, is_32);

free_mem:
	for (i = 0; i < n_unmarshal_handles; i++)
		nvmap_handle_put(refs[i]);
	nvmap_altfree(refs, bytes);
	return err;
}

int nvmap_ioctl_gup_test(struct file *filp, void __user *arg)
{
	int i, err = -EINVAL;
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

	handle = nvmap_handle_get_from_fd(op.handle);
	if (!handle)
		goto exit;

	if (vma->vm_end - vma->vm_start != handle->size) {
		pr_err("handle size(0x%zx) and vma size(0x%lx) don't match\n",
			 handle->size, vma->vm_end - vma->vm_start);
		goto put_handle;
	}

	err = -ENOMEM;
	nr_page = handle->size >> PAGE_SHIFT;
	pages = nvmap_altalloc(nr_page * sizeof(*pages));
	if (IS_ERR_OR_NULL(pages)) {
		err = PTR_ERR(pages);
		goto put_handle;
	}

	err = nvmap_get_user_pages(op.va & PAGE_MASK, nr_page, pages);
	if (err)
		goto put_user_pages;

	for (i = 0; i < nr_page; i++) {
		if (handle->pgalloc.pages[i] != pages[i]) {
			pr_err("page pointers don't match, %p %p\n",
			       handle->pgalloc.pages[i], pages[i]);
			op.result = 0;
		}
	}

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

	for (i = 0; i < nvmap_dev->nr_carveouts; i++)
		op.heaps |= nvmap_dev->heaps[i].heap_bit;

	if (copy_to_user(arg, &op, sizeof(op))) {
		pr_err("copy_to_user failed\n");
		return -EINVAL;
	}

	return 0;
}

int nvmap_ioctl_get_heap_size(struct file *filp, void __user *arg)
{
	struct nvmap_heap_size op;
	struct nvmap_heap *heap;
	int i;
	memset(&op, 0, sizeof(op));

	if (copy_from_user(&op, arg, sizeof(op)))
		return -EFAULT;

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		if (op.heap & nvmap_dev->heaps[i].heap_bit) {
			heap = nvmap_dev->heaps[i].carveout;
			op.size = nvmap_query_heap_size(heap);
			if (copy_to_user(arg, &op, sizeof(op)))
				return -EFAULT;
			return 0;
		}
	}
	return -ENODEV;

}
