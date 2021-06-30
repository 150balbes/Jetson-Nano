/*
 * drivers/video/tegra/nvmap/nvmap_carveout.c
 *
 * Interface with nvmap carveouts
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

#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include "nvmap_handle.h"
#include "nvmap_dev.h"
#include "nvmap_cache.h"
#include "nvmap_misc.h"

extern struct nvmap_device *nvmap_dev;
extern struct kmem_cache *heap_block_cache;

struct list_block {
	struct nvmap_heap_block block;
	struct list_head all_list;
	unsigned int mem_prot;
	phys_addr_t orig_addr;
	size_t size;
	size_t align;
	struct nvmap_heap *heap;
	struct list_head free_list;
};

struct nvmap_carveout_node {
	unsigned int		heap_bit;
	struct nvmap_heap	*carveout;
	int			index;
	phys_addr_t		base;
	size_t			size;
};

struct nvmap_heap {
	struct list_head all_list;
	struct mutex lock;
	const char *name;
	void *arg;
	/* heap base */
	phys_addr_t base;
	/* heap size */
	size_t len;
	struct device *cma_dev;
	struct device *dma_dev;
	bool is_ivm;
	bool can_alloc; /* Used only if is_ivm == true */
	int peer; /* Used only if is_ivm == true */
	int vm_id; /* Used only if is_ivm == true */
	struct nvmap_pm_ops pm_ops;
};

/* TODO fix these global variables */
extern struct nvmap_device *nvmap_dev;
extern const struct file_operations debug_clients_fops;
extern const struct file_operations debug_allocations_fops;
extern const struct file_operations debug_all_allocations_fops;
extern const struct file_operations debug_orphan_handles_fops;
extern const struct file_operations debug_maps_fops;

int nvmap_carveout_is_ivm(struct nvmap_carveout_node *carveout)
{
	return carveout->heap_bit & NVMAP_HEAP_CARVEOUT_IVM;
}

int nvmap_carveout_query_peer(struct nvmap_carveout_node *carveout)
{
	return nvmap_query_heap_peer(carveout->carveout);
}

int nvmap_carveout_heap_bit(struct nvmap_carveout_node *carveout)
{
	return carveout->heap_bit;
}

int nvmap_carveout_query_heap_size(struct nvmap_carveout_node *carveout)
{
	return nvmap_query_heap_size(carveout->carveout);
}

int nvmap_carveout_create(const struct nvmap_platform_carveout *co)
{
	int i, err = 0;
	struct nvmap_carveout_node *node;

	if (!nvmap_dev->heaps) {
		nvmap_dev->nr_carveouts = 0;
		nvmap_dev->nr_heaps = nvmap_dev->plat->nr_carveouts + 1;
		nvmap_dev->heaps = kzalloc(sizeof(struct nvmap_carveout_node) *
				     nvmap_dev->nr_heaps, GFP_KERNEL);
		if (!nvmap_dev->heaps) {
			err = -ENOMEM;
			pr_err("couldn't allocate carveout memory\n");
			goto out;
		}
	} else if (nvmap_dev->nr_carveouts >= nvmap_dev->nr_heaps) {
		node = krealloc(nvmap_dev->heaps,
				sizeof(*node) * (nvmap_dev->nr_carveouts + 1),
				GFP_KERNEL);
		if (!node) {
			err = -ENOMEM;
			pr_err("nvmap heap array resize failed\n");
			goto out;
		}
		nvmap_dev->heaps = node;
		nvmap_dev->nr_heaps = nvmap_dev->nr_carveouts + 1;
	}

	for (i = 0; i < nvmap_dev->nr_heaps; i++)
		if ((co->usage_mask != NVMAP_HEAP_CARVEOUT_IVM) &&
		    (nvmap_dev->heaps[i].heap_bit & co->usage_mask)) {
			pr_err("carveout %s already exists\n", co->name);
			return -EEXIST;
		}

	node = &nvmap_dev->heaps[nvmap_dev->nr_carveouts];

	node->base = round_up(co->base, PAGE_SIZE);
	node->size = round_down(co->size -
				(node->base - co->base), PAGE_SIZE);
	if (!co->size)
		goto out;

	node->carveout = nvmap_heap_create(
			nvmap_dev->dev_user.this_device, co,
			node->base, node->size, node);

	if (!node->carveout) {
		err = -ENOMEM;
		pr_err("couldn't create %s\n", co->name);
		goto out;
	}
	node->index = nvmap_dev->nr_carveouts;
	nvmap_dev->nr_carveouts++;
	node->heap_bit = co->usage_mask;

	if (!IS_ERR_OR_NULL(nvmap_dev->debug_root)) {
		struct dentry *heap_root =
			debugfs_create_dir(co->name, nvmap_dev->debug_root);
		if (!IS_ERR_OR_NULL(heap_root)) {
			debugfs_create_file("clients", S_IRUGO,
				heap_root,
				(void *)(uintptr_t)node->heap_bit,
				&debug_clients_fops);
			debugfs_create_file("allocations", S_IRUGO,
				heap_root,
				(void *)(uintptr_t)node->heap_bit,
				&debug_allocations_fops);
			debugfs_create_file("all_allocations", S_IRUGO,
				heap_root,
				(void *)(uintptr_t)node->heap_bit,
				&debug_all_allocations_fops);
			debugfs_create_file("orphan_handles", S_IRUGO,
				heap_root,
				(void *)(uintptr_t)node->heap_bit,
				&debug_orphan_handles_fops);
			debugfs_create_file("maps", S_IRUGO,
				heap_root,
				(void *)(uintptr_t)node->heap_bit,
				&debug_maps_fops);
			nvmap_heap_debugfs_init(heap_root,
						node->carveout);
		}
	}
out:
	return err;
}

struct device *nvmap_heap_type_to_dev(unsigned long type)
{
	int i;
	struct nvmap_carveout_node *co_heap;

	for (i = 0; i < nvmap_dev->nr_carveouts; i++) {
		co_heap = &nvmap_dev->heaps[i];

		if (!(co_heap->heap_bit & type))
			continue;

		return co_heap->carveout->dma_dev;
	}
	return ERR_PTR(-ENODEV);
}

int heap_alloc_mem_virtualized(struct device *dev, phys_addr_t pa, size_t len)
{
	void *ret;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));

	ret = dma_mark_declared_memory_occupied(dev, pa, len, __DMA_ATTR(attrs));
	if (IS_ERR(ret)) {
		dev_err(dev, "Failed to reserve (%pa) len(%zu)\n", &pa, len);
		return 1;
	} else {
		dev_dbg(dev, "reserved (%pa) len(%zu)\n", &pa, len);
	}
	return 0;
}

static void nvmap_free_mem(struct nvmap_heap *h, phys_addr_t base,
				size_t len)
{
	struct device *dev = h->dma_dev;
	DEFINE_DMA_ATTRS(attrs);

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));
	dev_dbg(dev, "Free base (%pa) size (%zu)\n", &base, len);
#ifdef CONFIG_TEGRA_VIRTUALIZATION
	if (h->is_ivm && !h->can_alloc) {
		dma_mark_declared_memory_unoccupied(dev, base, len, __DMA_ATTR(attrs));
	} else
#endif
	{
		dma_free_attrs(dev, len,
			        (void *)(uintptr_t)base,
			        (dma_addr_t)base, __DMA_ATTR(attrs));
	}
}

static phys_addr_t nvmap_alloc_mem(struct nvmap_heap *h, size_t len,
				   phys_addr_t *start)
{
	phys_addr_t pa;
	DEFINE_DMA_ATTRS(attrs);
	struct device *dev = h->dma_dev;

	dma_set_attr(DMA_ATTR_ALLOC_EXACT_SIZE, __DMA_ATTR(attrs));

#ifdef CONFIG_TEGRA_VIRTUALIZATION
	if (start && h->is_ivm) {
		int err;

		pa = h->base + *start;
		err = heap_alloc_mem_virtualized(dev, pa, len);
		if (err) {
			return DMA_ERROR_CODE;
		}
		return pa;
	}
#endif

	(void)dma_alloc_attrs(dev, len, &pa, GFP_KERNEL, __DMA_ATTR(attrs));
	if (dma_mapping_error(dev, pa)) {
		return pa;
	}

	dev_dbg(dev, "Allocated addr (%pa) len(%zu)\n", &pa, len);

	if (!dma_is_coherent_dev(dev) && h->cma_dev) {
		int ret;
		ret = nvmap_cache_maint_phys_range(NVMAP_CACHE_OP_WB,
							pa, pa + len);
		if (ret) {
			dev_err(dev, "cache WB on (%pa, %zu) failed\n",
								&pa, len);
		}
	}

	return pa;
}

/*
 * This routine is used to flush the carveout memory from cache.
 * Why cache flush is needed for carveout? Consider the case, where a piece of
 * carveout is allocated as cached and released. After this, if the same memory is
 * allocated for uncached request and the memory is not flushed out from cache.
 * In this case, the client might pass this to H/W engine and it could start modify
 * the memory. As this was cached earlier, it might have some portion of it in cache.
 * During cpu request to read/write other memory, the cached portion of this memory
 * might get flushed back to main memory and would cause corruptions, if it happens
 * after H/W writes data to memory.
 *
 * But flushing out the memory blindly on each carveout allocation is redundant.
 *
 * In order to optimize the carveout buffer cache flushes, the following
 * strategy is used.
 *
 * The whole Carveout is flushed out from cache during its initialization.
 * During allocation, carveout buffers are not flused from cache.
 * During deallocation, carveout buffers are flushed, if they were allocated as cached.
 * if they were allocated as uncached/writecombined, no cache flush is needed.
 * Just draining store buffers is enough.
 */
static int heap_block_flush(struct nvmap_heap_block *block, size_t len,
							unsigned int prot)
{
	phys_addr_t phys = block->base;
	phys_addr_t end = block->base + len;
	int ret = 0;

	if (prot == NVMAP_HANDLE_UNCACHEABLE
				|| prot == NVMAP_HANDLE_WRITE_COMBINE)
		goto out;

	ret = nvmap_cache_maint_phys_range(NVMAP_CACHE_OP_WB_INV, phys, end);
	if (ret)
		goto out;
out:
	wmb();
	return ret;
}

void nvmap_heap_block_free(struct nvmap_heap_block *b)
{
	struct nvmap_heap *h;
	struct list_block *lb;

	if (!b)
		return;

	h = nvmap_block_to_heap(b);
	mutex_lock(&h->lock);

	lb = container_of(b, struct list_block, block);
	heap_block_flush(b, lb->size, lb->mem_prot);


	list_del(&lb->all_list);
	nvmap_free_mem(h, b->base, lb->size);
	kmem_cache_free(heap_block_cache, lb);

	/*
	 * If this HEAP has pm_ops defined and powering off the
	 * RAM attached with the HEAP returns error, raise warning.
	 */
	if (h->pm_ops.idle) {
		if (h->pm_ops.idle() < 0)
			WARN_ON(1);
	}

	mutex_unlock(&h->lock);
}

static struct nvmap_heap_block *heap_block_alloc(struct nvmap_heap *heap,
					      size_t len, size_t align,
					      unsigned int mem_prot,
					      phys_addr_t *start)
{
	struct list_block *heap_block = NULL;
	dma_addr_t dev_base;
	struct device *dev = heap->dma_dev;

	align = max_t(size_t, align, L1_CACHE_BYTES);

	/* since pages are only mappable with one cache attribute,
	 * and most allocations from carveout heaps are DMA coherent
	 * (i.e., non-cacheable), round cacheable allocations up to
	 * a page boundary to ensure that the physical pages will
	 * only be mapped one way. */
	if (mem_prot == NVMAP_HANDLE_CACHEABLE ||
	    mem_prot == NVMAP_HANDLE_INNER_CACHEABLE) {
		align = max_t(size_t, align, PAGE_SIZE);
		len = PAGE_ALIGN(len);
	}

	if (heap->is_ivm)
		align = max_t(size_t, align, NVMAP_IVM_ALIGNMENT);

	heap_block = kmem_cache_zalloc(heap_block_cache, GFP_KERNEL);
	if (!heap_block) {
		dev_err(dev, "%s: failed to alloc heap block %s\n",
			__func__, dev_name(dev));
		goto fail_heap_block_alloc;
	}

	dev_base = nvmap_alloc_mem(heap, len, start);
	if (dma_mapping_error(dev, dev_base)) {
		dev_err(dev, "failed to alloc mem of size (%zu)\n",
			len);
		if (dma_is_coherent_dev(dev)) {
			struct dma_coherent_stats stats;

			dma_get_coherent_stats(dev, &stats);
			dev_err(dev, "used:%zu,curr_size:%zu max:%zu\n",
				stats.used, stats.size, stats.max);
		}
		goto fail_dma_alloc;
	}

	heap_block->block.base = dev_base;
	heap_block->orig_addr = dev_base;
	heap_block->size = len;

	list_add_tail(&heap_block->all_list, &heap->all_list);
	heap_block->heap = heap;
	heap_block->mem_prot = mem_prot;
	heap_block->align = align;
	return &heap_block->block;

fail_dma_alloc:
	kmem_cache_free(heap_block_cache, heap_block);
fail_heap_block_alloc:
	return NULL;
}

static int heap_can_allocate(struct nvmap_heap *h, int peer, phys_addr_t *start)
{
	if (h->is_ivm) { /* Is IVM carveout? */
		/* Check if this correct IVM heap */
		if (peer != h->peer) {
			return 0;
		}
		/* If this partition does actual allocation, it
		 * should not specify start_offset.
		 */
		if (h->can_alloc && start) {
			return 0;
		}

		/* If this partition does not do actual
		 * allocation, it should specify start_offset.
		 */
		if (!h->can_alloc && !start) {
			return 0;
		}
	}

	/*
	 * If this HEAP has pm_ops defined and powering on the
	 * RAM attached with the HEAP returns error, don't
	 * allocate from the heap and return NULL.
	 */
	if (h->pm_ops.busy) {
		if (h->pm_ops.busy() < 0) {
			pr_err("Unable to power on the heap device\n");
			return 0;
		}
	}
	return 1;
}

/* nvmap_heap_alloc: allocates a block of memory of len bytes, aligned to
 * align bytes. */
struct nvmap_heap_block *nvmap_carveout_alloc(struct nvmap_carveout_node *co,
					phys_addr_t *start,
					size_t len,
					size_t align,
					unsigned int prot,
					int peer)
{
	struct nvmap_heap *h = co->carveout;
	struct nvmap_heap_block *b;

	mutex_lock(&h->lock);

	if (!heap_can_allocate(h, peer, start)) {
		mutex_unlock(&h->lock);
		return NULL;
	}

	b = heap_block_alloc(h, len, align, prot, start);
	if (!b) {
		mutex_unlock(&h->lock);
		return NULL;
	}

	mutex_unlock(&h->lock);
	return b;
}

u64 nvmap_carveout_ivm(struct nvmap_carveout_node *co,
				struct nvmap_heap_block *b, size_t len)
{
	struct nvmap_heap *h = co->carveout;
	unsigned int offs;

	/* Generate IVM for partition that can alloc */
	if (h->is_ivm && h->can_alloc) {
		offs = (b->base - h->base);
		return nvmap_calculate_ivm_id(h->vm_id, len, offs);
	} else {
		return 0;
	}
}

// This is only needed because dev doesn't have a double pointer
// and carveout.c is the only file that knows the size of the struct
struct nvmap_carveout_node *nvmap_carveout_index(
				struct nvmap_carveout_node *node, int i)
{
	return node + i;
}

void nvmap_carveout_destroy(struct nvmap_carveout_node *node)
{
	nvmap_heap_destroy(node->carveout);
}
