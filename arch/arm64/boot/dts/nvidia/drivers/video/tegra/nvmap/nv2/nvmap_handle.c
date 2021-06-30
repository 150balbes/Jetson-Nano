/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include <soc/tegra/chip-id.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_stats.h"

#include "nvmap_misc.h"
#include "nvmap_cache.h"
#include "nvmap_heap_alloc.h"
#include "nvmap_carveout.h"
#include "nvmap_carveout.h"
#include "nvmap_dev.h"
#include "nvmap_dmabuf.h"
#include "nvmap_vma.h"
#include "nvmap_tag.h"

#include "nvmap_handle.h"
#include "nvmap_handle_priv.h"

// TODO: Add page coloring

static void handle_add_to_dev(struct nvmap_handle *h, struct nvmap_device *dev);
static int handle_remove_from_dev(struct nvmap_handle *h,
						struct nvmap_device *dev);
// TODO: Remove these global variables
extern size_t cache_maint_inner_threshold;
extern int nvmap_cache_maint_by_set_ways;
extern struct static_key nvmap_disable_vaddr_for_cache_maint;

static struct nvmap_handle_info *handle_create_info(struct nvmap_handle *handle)
{

	struct nvmap_handle_info *info = kzalloc(sizeof(*info), GFP_KERNEL);

	if (!info) {
		return ERR_PTR(-ENOMEM);
	}

	info->handle = handle;
	INIT_LIST_HEAD(&info->maps);
	mutex_init(&info->maps_lock);

	return info;
}

struct dma_buf *nvmap_handle_to_dmabuf(struct nvmap_handle *handle)
{
	return handle->dmabuf;
}

void nvmap_handle_install_fd(struct nvmap_handle *handle, int fd)
{
	nvmap_dmabuf_install_fd(handle->dmabuf, fd);
	// TODO: why is this get_dma_buf here?
	get_dma_buf(handle->dmabuf);
}

void nvmap_handle_kmap_inc(struct nvmap_handle *h)
{
	atomic_inc(&h->kmap_count);
}

void nvmap_handle_kmap_dec(struct nvmap_handle *h)
{
	atomic_dec(&h->kmap_count);
}

void nvmap_handle_umap_inc(struct nvmap_handle *h)
{
	atomic_inc(&h->umap_count);
}

void nvmap_handle_umap_dec(struct nvmap_handle *h)
{
	atomic_dec(&h->umap_count);
}

size_t nvmap_handle_size(struct nvmap_handle *h)
{
	return h->size;
}

int nvmap_handle_is_allocated(struct nvmap_handle *h)
{
	return h->alloc;
}

size_t nvmap_handle_ivm_id(struct nvmap_handle *h)
{
	return h->ivm_id;
}

u32 nvmap_handle_heap_type(struct nvmap_handle *h)
{
	return h->heap_type;
}

u32 nvmap_handle_userflag(struct nvmap_handle *h)
{
	return h->userflags;
}

u32 nvmap_handle_flags(struct nvmap_handle *h)
{
	return h->flags;
}


bool nvmap_handle_is_heap(struct nvmap_handle *h)
{
	return h->heap_pgalloc;
}

bool nvmap_handle_track_dirty(struct nvmap_handle *h)
{
	if (!h->heap_pgalloc)
		return false;

	return h->userflags & (NVMAP_HANDLE_CACHE_SYNC |
			       NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE);
}

struct nvmap_handle *nvmap_handle_from_fd(int fd)
{
	struct nvmap_handle *handle = ERR_PTR(-EINVAL);
	struct dma_buf *dmabuf;

	dmabuf = nvmap_dmabuf_from_fd(fd);
	if (IS_ERR(dmabuf))
		return ERR_CAST(dmabuf);

	handle = nvmap_dmabuf_to_handle(dmabuf);
	if (IS_ERR(handle))
		return ERR_CAST(handle);

	return handle;
}

struct list_head *nvmap_handle_lru(struct nvmap_handle *h)
{
	return &h->lru;
}

atomic_t *nvmap_handle_pin(struct nvmap_handle *h)
{
	return &h->pin;
}

/*
 * NOTE: this does not ensure the continued existence of the underlying
 * dma_buf. If you want ensure the existence of the dma_buf you must get an
 * nvmap_handle_ref as that is what tracks the dma_buf refs.
 */
struct nvmap_handle *nvmap_handle_get(struct nvmap_handle *h)
{
	if (WARN_ON(!virt_addr_valid(h))) {
		pr_err("%s: invalid handle\n", current->group_leader->comm);
		return NULL;
	}

	if (unlikely(atomic_inc_return(&h->ref) <= 1)) {
		pr_err("%s: %s attempt to get a freed handle\n",
			__func__, current->group_leader->comm);
		atomic_dec(&h->ref);
		return NULL;
	}
	return h;
}

void nvmap_handle_put(struct nvmap_handle *h)
{
	int cnt;

	if (WARN_ON(!virt_addr_valid(h)))
		return;
	cnt = atomic_dec_return(&h->ref);

	if (WARN_ON(cnt < 0)) {
		pr_err("%s: %s put to negative references\n",
			__func__, current->comm);
	} else if (cnt == 0) {
		nvmap_handle_destroy(h);
	}
}

struct nvmap_handle *nvmap_handle_create(size_t size)
{
	void *err = ERR_PTR(-ENOMEM);
	struct nvmap_handle *h;
	struct nvmap_handle_info *info;

	if (!size)
		return ERR_PTR(-EINVAL);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return ERR_PTR(-ENOMEM);

	/* This reference of 1 is the reference the dmabuf has on the handle
	 * It's removed when the dma_buf is released through
	 *    nvmap_dmabuf_release
	 */
	atomic_set(&h->ref, 1);
	atomic_set(&h->pin, 0);

	h->orig_size = size;
	h->size = PAGE_ALIGN(size);
	h->flags = NVMAP_HANDLE_WRITE_COMBINE;
	h->peer = NVMAP_IVM_INVALID_PEER;
	mutex_init(&h->lock);
	INIT_LIST_HEAD(&h->vmas);
	INIT_LIST_HEAD(&h->lru);
	INIT_LIST_HEAD(&h->dmabuf_priv);


	info = handle_create_info(h);
	if (IS_ERR(info)) {
		err = info;
		goto  make_info_fail;
	}

	h->dmabuf = nvmap_dmabuf_create(info, h->size);
	if (IS_ERR_OR_NULL(h->dmabuf)) {
		err = ERR_PTR(-ENOMEM);
		goto make_dmabuf_fail;
	}

	handle_add_to_dev(h, nvmap_dev);

	return h;

make_dmabuf_fail:
	kfree(info);
make_info_fail:
	kfree(h);
	return err;
}

static void handle_pgalloc_free(struct nvmap_pgalloc *pgalloc, size_t size,
				int from_va)
{
	int i;
	unsigned int nr_page;
	unsigned int page_index = 0;

	nr_page = DIV_ROUND_UP(size, PAGE_SIZE);

	BUG_ON(size & ~PAGE_MASK);
	BUG_ON(!pgalloc->pages);

	for (i = 0; i < nr_page; i++)
		pgalloc->pages[i] = nvmap_to_page(pgalloc->pages[i]);

#ifdef CONFIG_NVMAP_PAGE_POOLS
	if (!from_va)
		page_index = nvmap_page_pool_fill_lots(&nvmap_dev->pool,
					pgalloc->pages, nr_page);
#endif

	for (i = page_index; i < nr_page; i++) {
		if (from_va)
			put_page(pgalloc->pages[i]);
		else
			__free_page(pgalloc->pages[i]);
	}

	nvmap_altfree(pgalloc->pages, nr_page * sizeof(struct page *));
}

static void handle_dealloc(struct nvmap_handle *h)
{
	if (!h->alloc)
		return;

	nvmap_stats_inc(NS_RELEASE, h->size);
	nvmap_stats_dec(NS_TOTAL, h->size);
	if (!h->heap_pgalloc) {
		if (h->vaddr) {
			struct vm_struct *vm;
			void *addr = h->vaddr;

			addr -= (h->carveout->base & ~PAGE_MASK);
			vm = find_vm_area(addr);
			BUG_ON(!vm);
			free_vm_area(vm);
		}

		nvmap_heap_free(h->carveout);
		nvmap_handle_kmap_dec(h);
		h->vaddr = NULL;
		return;
	} else if (nvmap_heap_type_is_dma(h->heap_type)){
		nvmap_heap_dealloc_dma_pages(h->size, h->heap_type,
						h->pgalloc.pages);
	} else {
		if (h->vaddr) {
			nvmap_handle_kmap_dec(h);

			vm_unmap_ram(h->vaddr, h->size >> PAGE_SHIFT);
			h->vaddr = NULL;
		}

		handle_pgalloc_free(&h->pgalloc, h->size, h->from_va);
	}
}

/* adds a newly-created handle to the device master tree */
static void handle_add_to_dev(struct nvmap_handle *h, struct nvmap_device *dev)
{
	struct rb_node **p;
	struct rb_node *parent = NULL;

	spin_lock(&dev->handle_lock);
	p = &dev->handles.rb_node;
	while (*p) {
		struct nvmap_handle *b;

		parent = *p;
		b = rb_entry(parent, struct nvmap_handle, node);
		if (h > b)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&h->node, parent, p);
	rb_insert_color(&h->node, &dev->handles);
	nvmap_lru_add(&h->lru);
	spin_unlock(&dev->handle_lock);
}

/* remove a handle from the device's tree of all handles; called
 * when freeing handles. */
static int handle_remove_from_dev(struct nvmap_handle *h,
						struct nvmap_device *dev)
{
	spin_lock(&dev->handle_lock);

	/* re-test inside the spinlock if the handle really has no clients;
	 * only remove the handle if it is unreferenced */
	if (atomic_add_return(0, &h->ref) > 0) {
		spin_unlock(&dev->handle_lock);
		return -EBUSY;
	}
	smp_rmb();
	BUG_ON(atomic_read(&h->ref) < 0);
	BUG_ON(atomic_read(&h->pin) != 0);

	nvmap_lru_del(&h->lru);
	rb_erase(&h->node, &dev->handles);

	spin_unlock(&dev->handle_lock);
	return 0;
}

void nvmap_handle_add_owner(struct nvmap_handle *handle,
					struct nvmap_client *client)
{
	if (!handle->owner)
		handle->owner = client;
}

void nvmap_handle_destroy(struct nvmap_handle *h)
{
	nvmap_dmabufs_free(&h->dmabuf_priv);

	if (handle_remove_from_dev(h, nvmap_dev) != 0)
		return;

	handle_dealloc(h);

	NVMAP_TAG_TRACE(trace_nvmap_destroy_handle,
		NULL, get_current()->pid, 0, NVMAP_TP_ARGS_H(h));
	kfree(h);
}

static void heap_alloc_and_set_handle(
			 struct nvmap_handle *h, unsigned int orig_heap)
{
	unsigned int heap_type;
	struct page **pages;

	BUG_ON(orig_heap & (orig_heap - 1));

	heap_type = nvmap_heap_type_conversion(orig_heap);

	if (nvmap_heap_type_is_carveout(heap_type)) {
		int err;

		err = nvmap_handle_alloc_carveout(h, heap_type, NULL);
		if (!err) {
			h->heap_type = heap_type;
			h->heap_pgalloc = false;
			goto success;
		}

		pages = nvmap_heap_alloc_dma_pages(h->size, heap_type);
		if (IS_ERR_OR_NULL(pages))
			return;
		h->pgalloc.pages = pages;
		h->pgalloc.contig = 0;

		atomic_set(&h->pgalloc.ndirty, 0);
		h->heap_type = NVMAP_HEAP_CARVEOUT_VPR;
		h->heap_pgalloc = true;
	} else if (nvmap_heap_type_is_iovmm(heap_type)) {
		pages = nvmap_heap_alloc_iovmm_pages(h->size,
			h->userflags & NVMAP_HANDLE_PHYS_CONTIG);
		if (IS_ERR_OR_NULL(pages))
			return;

		h->pgalloc.pages = pages;
		h->pgalloc.contig =
			h->userflags & NVMAP_HANDLE_PHYS_CONTIG;
		atomic_set(&h->pgalloc.ndirty, 0);
		h->heap_type = NVMAP_HEAP_IOVMM;
		h->heap_pgalloc = true;
	}

success:
	/* barrier to ensure all handle alloc data
	 * is visible before alloc is seen by other
	 * processors.
	 */
	mb();
	h->alloc = true;
	return;
}

/* TODO: Change this to return an alloc_handle or something
 *       stop this and the next function from editing handle,
 *       and push the functions back into heap_alloc
 */
static void heap_alloc_handle_from_heaps(
					struct nvmap_handle *handle,
					const unsigned int *alloc_policy,
					unsigned int heap_mask)
{
	while (!handle->alloc && *alloc_policy) {
		unsigned int heap_type;

		heap_type = *alloc_policy++;
		heap_type &= heap_mask;

		if (!heap_type)
			continue;

		heap_mask &= ~heap_type;

		while (heap_type && !handle->alloc) {
			unsigned int heap;

			/* iterate possible heaps MSB-to-LSB, since higher-
			 * priority carveouts will have higher usage masks */
			heap = 1 << __fls(heap_type);
			heap_alloc_and_set_handle(handle, heap);
			heap_type &= ~heap;
		}
	}

}

int nvmap_handle_alloc(
		       struct nvmap_handle *h, unsigned int heap_mask,
		       size_t align,
		       u8 kind,
		       unsigned int flags,
		       int peer)
{
	const unsigned int *alloc_policy;
	int nr_page;
	int err = -ENOMEM;

	if (!h)
		return -EINVAL;

	if (h->alloc) {
		return -EEXIST;
	}

	h->userflags = flags;
	h->peer = NVMAP_IVM_INVALID_PEER;

	nr_page = ((h->size + PAGE_SIZE - 1) >> PAGE_SHIFT);
	/* Force mapping to uncached for VPR memory. */
	if (heap_mask & (NVMAP_HEAP_CARVEOUT_VPR | ~nvmap_dev->cpu_access_mask))
		h->flags = NVMAP_HANDLE_UNCACHEABLE;
	else
		h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = max_t(size_t, align, L1_CACHE_BYTES);

	alloc_policy = nvmap_heap_mask_to_policy(heap_mask, nr_page);
	if (!alloc_policy) {
		err = -EINVAL;
		goto out;
	}

	heap_alloc_handle_from_heaps(h, alloc_policy, heap_mask);

out:
	return err;
}

struct nvmap_handle *nvmap_handle_from_ivmid(u64 ivm_id)
{
	struct nvmap_handle *handle = NULL;
	struct rb_node *n;

	spin_lock(&nvmap_dev->handle_lock);

	n = nvmap_dev->handles.rb_node;
	for (n = rb_first(&nvmap_dev->handles); n; n = rb_next(n)) {
		handle = rb_entry(n, struct nvmap_handle, node);
		if (handle->ivm_id == ivm_id) {
			BUG_ON(!virt_addr_valid(handle));

			spin_unlock(&nvmap_dev->handle_lock);
			return handle;
		}
	}

	spin_unlock(&nvmap_dev->handle_lock);
	return NULL;
}

int nvmap_handle_alloc_from_va(struct nvmap_handle *h,
			       ulong addr,
			       unsigned int flags)
{
	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;

	if (h->alloc) {
		nvmap_handle_put(h);
		return -EEXIST;
	}

	h->userflags = flags;
	h->flags = (flags & NVMAP_HANDLE_CACHE_FLAG);
	h->align = PAGE_SIZE;

	h->pgalloc.pages = nvmap_heap_alloc_from_va(h->size, addr);
	if (!h->pgalloc.pages) {
		nvmap_handle_put(h);
		return -ENOMEM;
	}

	atomic_set(&h->pgalloc.ndirty, 0);
	h->heap_type = NVMAP_HEAP_IOVMM;
	h->heap_pgalloc = true;
	h->from_va = true;

	mb();
	h->alloc = true;

	nvmap_handle_put(h);
	return 0;
}

int nvmap_handle_alloc_carveout(struct nvmap_handle *handle,
					      unsigned long type,
					      phys_addr_t *start)
{
	struct nvmap_carveout_node *co_heap;
	struct nvmap_device *dev = nvmap_dev;
	int i;

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_heap_block *block;
		co_heap = nvmap_dev_to_carveout(dev, i);

		if (!(nvmap_carveout_heap_bit(co_heap) & type))
			continue;

		if (type & NVMAP_HEAP_CARVEOUT_IVM)
			handle->size = ALIGN(handle->size, NVMAP_IVM_ALIGNMENT);

		block = nvmap_carveout_alloc(co_heap, start,
						handle->size,
						handle->align,
						handle->flags,
						handle->peer);
		if (block) {
			handle->carveout = block;
			handle->ivm_id = nvmap_carveout_ivm(co_heap, block,
								handle->size);
			return 0;
		}
	}
	return -1;

}

int nvmap_handle_alloc_from_ivmid(struct nvmap_handle *handle, u64 ivm_id)
{
	phys_addr_t offs = nvmap_ivmid_to_offset(ivm_id);
	int peer = nvmap_ivmid_to_peer(ivm_id);
	int err;

	handle->peer = peer;

	err = nvmap_handle_alloc_carveout(handle, NVMAP_HEAP_CARVEOUT_IVM,
								&offs);
	if (err) {
		return -1;
	}

	handle->heap_type = NVMAP_HEAP_CARVEOUT_IVM;
	handle->heap_pgalloc = false;
	handle->ivm_id = ivm_id;

	mb();
	handle->alloc = true;

	return 0;
}

void nvmap_handle_zap(struct nvmap_handle *handle, u64 offset, u64 size)
{
	if (!handle->heap_pgalloc)
		return;

	/* if no dirty page is present, no need to zap */
	if (nvmap_handle_track_dirty(handle)
			&& !atomic_read(&handle->pgalloc.ndirty))
		return;

	if (!size) {
		offset = 0;
		size = handle->size;
	}

	size = PAGE_ALIGN((offset & ~PAGE_MASK) + size);

	mutex_lock(&handle->lock);
	nvmap_vma_zap(&handle->vmas, offset, size);
	mutex_unlock(&handle->lock);
}

static int handle_cache_maint_heap_page_inner(struct nvmap_handle *handle,
				unsigned int op,
				unsigned long start, unsigned long end)
{
	if (static_key_false(&nvmap_disable_vaddr_for_cache_maint))
		return 0;

	if (!handle->vaddr) {
		/* TODO: We need better naming than mapping and then unmapping */
		if (nvmap_handle_mmap(handle))
			nvmap_handle_munmap(handle, handle->vaddr);
		else
			return 1;
	}
	/* Fast inner cache maintenance using single mapping */
	nvmap_cache_maint_inner(op, handle->vaddr + start, end - start);
	return 0;
}

void nvmap_handle_cache_maint_heap_page(struct nvmap_handle *handle,
				unsigned long op,
				unsigned long start, unsigned long end,
				int outer)
{
	int err;

	if (handle->userflags & NVMAP_HANDLE_CACHE_SYNC) {
		/*
		 * zap user VA->PA mappings so that any access to the pages
		 * will result in a fault and can be marked dirty
		 */
		nvmap_handle_mkclean(handle, start, end-start);
		nvmap_handle_zap(handle, start, end - start);
	}

	err = handle_cache_maint_heap_page_inner(handle, op, start, end);
	if (err && outer) {
		nvmap_cache_maint_heap_page_outer(handle->pgalloc.pages, op,
							start, end);
	}
}

/*
 * This is the equivalent of __nvmap_do_cache_maint
 * The clean_only_dirty flag has been removed because it is always passed as
 * false
 */
int nvmap_handle_cache_maint(struct nvmap_handle *handle, unsigned long start,
		unsigned long end, unsigned int op)
{
	int ret = 0;

	if ((start >= handle->size) || (end > handle->size)) {
		return -EFAULT;
	}

	if (!(handle->heap_type & nvmap_dev->cpu_access_mask)) {
		return -EPERM;
	}

	if (!handle || !handle->alloc)
		return -EFAULT;

	nvmap_handle_kmap_inc(handle);

	if (op == NVMAP_CACHE_OP_INV)
		op = NVMAP_CACHE_OP_WB_INV;

	if (!end)
		end = handle->size;

	wmb();
	if (handle->flags == NVMAP_HANDLE_UNCACHEABLE ||
	    handle->flags == NVMAP_HANDLE_WRITE_COMBINE || start == end)
		goto out;

	if (start > handle->size || end > handle->size) {
		pr_warn("cache maintenance outside handle\n");
		ret = -EINVAL;
		goto out;
	}

	if (nvmap_cache_can_fast_maint(start, end, op)) {
		if (handle->userflags & NVMAP_HANDLE_CACHE_SYNC) {
			nvmap_handle_mkclean(handle, 0, handle->size);
			nvmap_handle_zap(handle, 0, handle->size);
		}
		nvmap_cache_fast_maint(op);
	} else if (handle->heap_pgalloc) {
		nvmap_handle_cache_maint_heap_page(handle, op, start, end,
			(handle->flags != NVMAP_HANDLE_INNER_CACHEABLE));
	} else {
		ret = nvmap_cache_maint_phys_range(op,
				start + handle->carveout->base,
				end   + handle->carveout->base);
	}

out:
	/* TODO: Add more stats counting here */
	nvmap_stats_inc(NS_CFLUSH_RQ, end - start);
	nvmap_handle_kmap_dec(handle);
	return ret;

}

static void cache_maint_large(struct nvmap_handle **handles, u64 total,
					u64 thresh, int op, int nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		if (handles[i]->userflags & NVMAP_HANDLE_CACHE_SYNC) {
			nvmap_handle_mkclean(handles[i], 0, handles[i]->size);
			nvmap_handle_zap(handles[i], 0, handles[i]->size);
		}
	}

	if (op == NVMAP_CACHE_OP_WB)
		nvmap_cache_inner_clean_all();
	else
		nvmap_cache_inner_flush_all();

	nvmap_stats_inc(NS_CFLUSH_RQ, total);
	nvmap_stats_inc(NS_CFLUSH_DONE, thresh);
	trace_nvmap_cache_flush(total,
			nvmap_stats_read(NS_ALLOC),
			nvmap_stats_read(NS_CFLUSH_RQ),
			nvmap_stats_read(NS_CFLUSH_DONE));
}

static int handles_get_total_cache_size(struct nvmap_handle **handles,
			u64 *sizes, int op, int nr)
{
	int i;
	int total = 0;

	for (i = 0; i < nr; i++) {
		bool inner, outer;

		nvmap_handle_get_cacheability(handles[i], &inner, &outer);

		if (!inner && !outer)
			continue;

		if ((op == NVMAP_CACHE_OP_WB)
				&& nvmap_handle_track_dirty(handles[i])) {
			/* TODO: shouldn't this be shifted by page size? */
			total += atomic_read(&handles[i]->pgalloc.ndirty);
		} else {
			total += sizes[i];
		}
	}

	return total;
}

/*
 * Perform cache op on the list of memory regions within passed handles.
 * A memory region within handle[i] is identified by offsets[i], sizes[i]
 *
 * This will optimze the op if it can.
 * In the case that all the handles together are larger than the inner cache
 * maint threshold it is possible to just do an entire inner cache flush.
 *
 * NOTE: this omits outer cache operations which is fine for ARM64
 */
int nvmap_handles_cache_maint(struct nvmap_handle **handles,
				u64 *offsets, u64 *sizes, int op, int nr)
{
	int i;
	int err;
	u64 total = 0;
	u64 thresh = ~0;

	/*
	 * As io-coherency is enabled by default from T194 onwards,
	 * Don't do cache maint from CPU side. The HW, SCF will do.
	 */
	if (tegra_get_chip_id() == TEGRA194)
		return 0;

	WARN(!IS_ENABLED(CONFIG_ARM64),
		"cache list operation may not function properly");

	if (nvmap_cache_maint_by_set_ways)
		thresh = cache_maint_inner_threshold;

	total = handles_get_total_cache_size(handles, sizes, op, nr);
	if (!total)
		return 0;

	/* Full flush in the case the passed list is bigger than our
	 * threshold. */
	if (total >= thresh) {
		cache_maint_large(handles, total, thresh, op, nr);
	} else {
		for (i = 0; i < nr; i++) {
			err = nvmap_handle_cache_maint(handles[i],
							offsets[i],
							offsets[i] + sizes[i],
							op);
			if (err) {
				pr_err("cache maint per handle failed [%d]\n",
						err);
				return err;
			}
		}
	}

	return 0;
}

static int handle_read(struct nvmap_handle *h, unsigned long h_offs,
			 unsigned long sys_addr, void *addr,
			 unsigned long elem_size)
{
	if (!(h->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE)) {
		nvmap_handle_cache_maint(h, h_offs, h_offs + elem_size,
							NVMAP_CACHE_OP_INV);
	}

	return copy_to_user((void *)sys_addr, addr, elem_size);
}

static int handle_write(struct nvmap_handle *h, unsigned long h_offs,
			 unsigned long sys_addr, void *addr,
			 unsigned long elem_size)
{
	int ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
	if (h->heap_type == NVMAP_HEAP_CARVEOUT_VPR) {
		uaccess_enable();
		memcpy_toio(addr, (void *)sys_addr, elem_size);
		uaccess_disable();
		ret = 0;
	} else {
		ret = copy_from_user(addr, (void *)sys_addr, elem_size);
	}
#else
	ret = copy_from_user(addr, (void *)sys_addr, elem_size);
#endif

	if (ret)
		return ret;

	if (!(h->userflags & NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE))
		nvmap_handle_cache_maint(h, h_offs, h_offs + elem_size,
						NVMAP_CACHE_OP_WB_INV);

	return ret;
}

ssize_t nvmap_handle_rw(struct nvmap_handle *h,
			 unsigned long h_offs, unsigned long h_stride,
			 unsigned long sys_addr, unsigned long sys_stride,
			 unsigned long elem_size, unsigned long count,
			 int is_read)
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

	/*
	 * TODO: Add an english description of this
	 * I think it is:
	 * If all of the strides are equal, and the offset is byte aligned,
	 * then do all of the copying in one go
	 */
	if (elem_size == h_stride && elem_size == sys_stride && (h_offs % 8 == 0)) {
		elem_size *= count;
		h_stride = elem_size;
		sys_stride = elem_size;
		count = 1;
	}

	if (elem_size > sys_stride || elem_size > h_stride)
		return -EINVAL;

	if (elem_size > h->size ||
			h_offs >= h->size ||
			h_stride * (count - 1) + elem_size > (h->size - h_offs) ||
			sys_stride * count > (h->size - h_offs))
		return -EINVAL;

	if (!h->vaddr) {
		if (nvmap_handle_mmap(h) == NULL)
			return -ENOMEM;
		nvmap_handle_munmap(h, h->vaddr);
	}

	addr = h->vaddr + h_offs;

	while (count--) {
		if (h_offs + elem_size > h->size) {
			pr_warn("read/write outside of handle\n");
			ret = -EFAULT;
			break;
		}

		if (is_read) {
			ret = handle_read(h, h_offs,
					sys_addr, addr, elem_size);
		} else {
			ret = handle_write(h, h_offs,
					sys_addr, addr, elem_size);
		}

		if (ret)
			break;

		copied += elem_size;
		sys_addr += sys_stride;
		h_offs += h_stride;
		addr += h_stride;
	}

	if (ret)
		return ret;

	return copied;
}

struct nvmap_handle *nvmap_handle_from_node(struct rb_node *n)
{
	return rb_entry(n, struct nvmap_handle, node);
}

struct nvmap_handle *nvmap_handle_from_lru(struct list_head *n)
{
	return list_entry(n, struct nvmap_handle, lru);
}
