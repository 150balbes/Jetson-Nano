/*
 * drivers/video/tegra/nvmap/nvmap_dev.c
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

#include <linux/debugfs.h>

#include "nvmap_priv.h"

extern struct nvmap_device *nvmap_dev;

extern const struct file_operations debug_clients_fops;
extern const struct file_operations debug_allocations_fops;
extern const struct file_operations debug_all_allocations_fops;
extern const struct file_operations debug_orphan_handles_fops;
extern const struct file_operations debug_maps_fops;

int nvmap_create_carveout(const struct nvmap_platform_carveout *co)
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

static
struct nvmap_heap_block *do_nvmap_carveout_alloc(struct nvmap_client *client,
					      struct nvmap_handle *handle,
					      unsigned long type,
					      phys_addr_t *start)
{
	struct nvmap_carveout_node *co_heap;
	struct nvmap_device *dev = nvmap_dev;
	int i;

	for (i = 0; i < dev->nr_carveouts; i++) {
		struct nvmap_heap_block *block;
		co_heap = &dev->heaps[i];

		if (!(co_heap->heap_bit & type))
			continue;

		if (type & NVMAP_HEAP_CARVEOUT_IVM)
			handle->size = ALIGN(handle->size, NVMAP_IVM_ALIGNMENT);

		block = nvmap_heap_alloc(co_heap->carveout, handle, start);
		if (block)
			return block;
	}
	return NULL;
}

struct nvmap_heap_block *nvmap_carveout_alloc(struct nvmap_client *client,
					      struct nvmap_handle *handle,
					      unsigned long type,
					      phys_addr_t *start)
{
	return do_nvmap_carveout_alloc(client, handle, type, start);
}
