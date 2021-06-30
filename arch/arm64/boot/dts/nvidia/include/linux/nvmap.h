/*
 * include/linux/nvmap.h
 *
 * structure declarations for nvmem and nvmap user-space ioctls
 *
 * Copyright (c) 2009-2019, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _LINUX_NVMAP_H
#define _LINUX_NVMAP_H

#include <linux/rbtree.h>
#include <linux/file.h>
#include <linux/dma-buf.h>
#include <linux/device.h>
#include <uapi/linux/nvmap.h>

#define NVMAP_HEAP_IOVMM   (1ul<<30)

/* common carveout heaps */
#define NVMAP_HEAP_CARVEOUT_IRAM    (1ul<<29)
#define NVMAP_HEAP_CARVEOUT_VPR     (1ul<<28)
#define NVMAP_HEAP_CARVEOUT_TSEC    (1ul<<27)
#define NVMAP_HEAP_CARVEOUT_VIDMEM  (1ul<<26)
#define NVMAP_HEAP_CARVEOUT_IVM     (1ul<<1)
#define NVMAP_HEAP_CARVEOUT_GENERIC (1ul<<0)

#define NVMAP_HEAP_CARVEOUT_MASK    (NVMAP_HEAP_IOVMM - 1)

/* allocation flags */
#define NVMAP_HANDLE_UNCACHEABLE     (0x0ul << 0)
#define NVMAP_HANDLE_WRITE_COMBINE   (0x1ul << 0)
#define NVMAP_HANDLE_INNER_CACHEABLE (0x2ul << 0)
#define NVMAP_HANDLE_CACHEABLE       (0x3ul << 0)
#define NVMAP_HANDLE_CACHE_FLAG      (0x3ul << 0)

#define NVMAP_HANDLE_SECURE          (0x1ul << 2)
#define NVMAP_HANDLE_KIND_SPECIFIED  (0x1ul << 3)
#define NVMAP_HANDLE_COMPR_SPECIFIED (0x1ul << 4)
#define NVMAP_HANDLE_ZEROED_PAGES    (0x1ul << 5)
#define NVMAP_HANDLE_PHYS_CONTIG     (0x1ul << 6)
#define NVMAP_HANDLE_CACHE_SYNC      (0x1ul << 7)
#define NVMAP_HANDLE_CACHE_SYNC_AT_RESERVE      (0x1ul << 8)

#ifdef CONFIG_NVMAP_PAGE_POOLS
ulong nvmap_page_pool_get_unused_pages(void);
#else
static inline ulong nvmap_page_pool_get_unused_pages(void)
{
	return 0;
}
#endif

ulong nvmap_iovmm_get_used_pages(void);
int nvmap_register_vidmem_carveout(struct device *dma_dev,
		phys_addr_t base, size_t size);

/*
 * A heap can be mapped to memory other than DRAM.
 * The HW, controls the memory, can be power gated/ungated
 * based upon the clients using the memory.
 * if no client/alloc happens from the memory, the HW needs
 * to be power gated. Similarly it should power ungated if
 * alloc happens from the memory.
 * int (*busy)(void) - trigger runtime power ungate
 * int (*idle)(void) - trigger runtime power gate
 */
struct nvmap_pm_ops {
	int (*busy)(void);
	int (*idle)(void);
};

struct nvmap_platform_carveout {
	const char *name;
	unsigned int usage_mask;
	phys_addr_t base;
	size_t size;
	struct device *cma_dev;
	bool resize;
	struct device *dma_dev;
	struct device dev;
	struct dma_declare_info *dma_info;
	bool is_ivm;
	int peer;
	int vmid;
	int can_alloc;
	bool enable_static_dma_map;
	bool disable_dynamic_dma_map;
	bool no_cpu_access; /* carveout can't be accessed from cpu at all */
	bool init_done;	/* FIXME: remove once all caveouts use reserved-memory */
	struct nvmap_pm_ops pm_ops;
};

struct nvmap_platform_data {
	const struct nvmap_platform_carveout *carveouts;
	unsigned int nr_carveouts;
};

#endif /* _LINUX_NVMAP_H */
