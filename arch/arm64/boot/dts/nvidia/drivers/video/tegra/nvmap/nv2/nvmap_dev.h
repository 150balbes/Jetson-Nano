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

#ifndef __NVMAP_DEV_H
#define __NVMAP_DEV_H

#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/nvmap.h>

#include "nvmap_heap.h"
#include "nvmap_pp.h"
#include "nvmap_structs.h"


struct nvmap_device {
	struct rb_root	handles;
	spinlock_t	handle_lock;
	struct miscdevice dev_user;
	// TODO: heaps should be a double pointer
	struct nvmap_carveout_node *heaps;
	int nr_heaps;
	int nr_carveouts;
#ifdef CONFIG_NVMAP_PAGE_POOLS
	struct nvmap_page_pool pool;
#endif
	struct list_head clients;
	struct rb_root pids;
	struct mutex	clients_lock;
	struct list_head lru_handles;
	spinlock_t	lru_lock;
	struct dentry *handles_by_pid;
	struct dentry *debug_root;
	struct nvmap_platform_data *plat;
	struct rb_root	tags;
	struct mutex	tags_lock;
	u32 dynamic_dma_map_mask;
	u32 cpu_access_mask;
};

int nvmap_probe(struct platform_device *pdev);
int nvmap_remove(struct platform_device *pdev);
int nvmap_init(struct platform_device *pdev);

extern struct nvmap_device *nvmap_dev;

u32 nvmap_cpu_access_mask(void);

struct nvmap_carveout_node *nvmap_dev_to_carveout(struct nvmap_device *dev, int i);

int nvmap_dmabuf_stash_init(void);

#endif /* __NVMAP_DEV_H */
