/*
 * include/linux/nvmap_t19x.h
 *
 * structure declarations for nvmem and nvmap user-space ioctls
 *
 * Copyright (c) 2009-2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/types.h>
#include <linux/device.h>

#ifndef _LINUX_NVMAP_T19x_H
#define _LINUX_NVMAP_T19x_H

#define NVMAP_HEAP_CARVEOUT_CVSRAM  (1ul<<25)
#define NVMAP_HEAP_CARVEOUT_GOS     (1ul<<24)

int nvmap_register_cvsram_carveout(struct device *dma_dev,
		phys_addr_t base, size_t size,
		int (*pmops_busy)(void), int (*pmops_idle)(void));

struct cv_dev_info {
	struct device_node *np;
	struct sg_table *sgt;
	void *cpu_addr;
	struct device offset_dev; /* used to alloc/free semaphore offsets within GoS */
	int idx; /* index to use by firmware to identify the client */
	int count; /* number of sgt */
};

struct cv_dev_info *nvmap_fetch_cv_dev_info(struct device *dev);

struct nvmap_handle_t19x {
	atomic_t nc_pin; /* no. of pins from non io coherent devices */
};

extern bool nvmap_version_t19x;

#endif /* _LINUX_NVMAP_T19x_H */
