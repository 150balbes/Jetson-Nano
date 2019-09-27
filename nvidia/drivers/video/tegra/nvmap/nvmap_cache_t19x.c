/*
 * drivers/video/tegra/nvmap/nvmap_cache_t19x.c
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/miscdevice.h>
#include <linux/nvmap_t19x.h>

#include "nvmap_priv.h"

struct static_key nvmap_updated_cache_config;

void nvmap_handle_get_cacheability(struct nvmap_handle *h,
		bool *inner, bool *outer)
{
	struct nvmap_handle_t19x *handle_t19x;
	struct device *dev = nvmap_dev->dev_user.parent;

	if (static_key_true(&nvmap_updated_cache_config)) {
		if (nvmap_version_t19x) {
			/* FIX ME: Update correct value after evaluation */
			nvmap_cache_maint_by_set_ways = 0;
			cache_maint_inner_threshold = SZ_2M;
		}
		static_key_slow_dec(&nvmap_updated_cache_config);
	}

	handle_t19x = dma_buf_get_drvdata(h->dmabuf, dev);
	if (handle_t19x && atomic_read(&handle_t19x->nc_pin)) {
		*inner = *outer = false;
		return;
	}

	*inner = h->flags == NVMAP_HANDLE_CACHEABLE ||
		 h->flags == NVMAP_HANDLE_INNER_CACHEABLE;
	*outer = h->flags == NVMAP_HANDLE_CACHEABLE;
}

static void nvmap_t19x_flush_cache(void)
{
	void *unused = NULL;

	tegra_flush_dcache_all(unused);
}

static void nvmap_t19x_clean_cache(void)
{
	void *unused = NULL;

	tegra_clean_dcache_all(unused);
}

static void nvmap_setup_t19x_cache_ops(struct nvmap_chip_cache_op *op)
{
	op->inner_flush_cache_all = nvmap_t19x_flush_cache;
	op->inner_clean_cache_all = nvmap_t19x_clean_cache;
	op->name = kstrdup("scf", GFP_KERNEL);
}
NVMAP_CACHE_OF_DECLARE("nvidia,carveouts-t19x", nvmap_setup_t19x_cache_ops);
