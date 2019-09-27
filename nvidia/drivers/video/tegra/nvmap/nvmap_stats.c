/*
 * drivers/video/tegra/nvmap/nvmap_stats.c
 *
 * Nvmap Stats keeping
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

struct nvmap_stats nvmap_stats;

static int nvmap_stats_reset(void *data, u64 val)
{
	int i;

	if (val) {
		atomic64_set(&nvmap_stats.collect, 0);
		for (i = 0; i < NS_NUM; i++) {
			if (i == NS_TOTAL)
				continue;
			atomic64_set(&nvmap_stats.stats[i], 0);
		}
	}
	return 0;
}

static int nvmap_stats_get(void *data, u64 *val)
{
	atomic64_t *ptr = data;

	*val = atomic64_read(ptr);
	return 0;
}

static int nvmap_stats_set(void *data, u64 val)
{
	atomic64_t *ptr = data;

	atomic64_set(ptr, val);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(reset_stats_fops, NULL, nvmap_stats_reset, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(stats_fops, nvmap_stats_get, nvmap_stats_set, "%llu\n");

void nvmap_stats_init(struct dentry *nvmap_debug_root)
{
	struct dentry *stats_root;

#define CREATE_DF(x, y) \
	debugfs_create_file(#x, S_IRUGO, stats_root, &y, &stats_fops);

	stats_root = debugfs_create_dir("stats", nvmap_debug_root);
	if (!IS_ERR_OR_NULL(stats_root)) {
		CREATE_DF(alloc, nvmap_stats.stats[NS_ALLOC]);
		CREATE_DF(release, nvmap_stats.stats[NS_RELEASE]);
		CREATE_DF(ualloc, nvmap_stats.stats[NS_UALLOC]);
		CREATE_DF(urelease, nvmap_stats.stats[NS_URELEASE]);
		CREATE_DF(kalloc, nvmap_stats.stats[NS_KALLOC]);
		CREATE_DF(krelease, nvmap_stats.stats[NS_KRELEASE]);
		CREATE_DF(cflush_rq, nvmap_stats.stats[NS_CFLUSH_RQ]);
		CREATE_DF(cflush_done, nvmap_stats.stats[NS_CFLUSH_DONE]);
		CREATE_DF(ucflush_rq, nvmap_stats.stats[NS_UCFLUSH_RQ]);
		CREATE_DF(ucflush_done, nvmap_stats.stats[NS_UCFLUSH_DONE]);
		CREATE_DF(kcflush_rq, nvmap_stats.stats[NS_KCFLUSH_RQ]);
		CREATE_DF(kcflush_done, nvmap_stats.stats[NS_KCFLUSH_DONE]);
		CREATE_DF(total_memory, nvmap_stats.stats[NS_TOTAL]);

		debugfs_create_file("collect", S_IRUGO | S_IWUSR,
			stats_root, &nvmap_stats.collect, &stats_fops);
		debugfs_create_file("reset", S_IWUSR,
			stats_root, NULL, &reset_stats_fops);
	}

#undef CREATE_DF
}

void nvmap_stats_inc(enum nvmap_stats_t stat, size_t size)
{
	if (atomic64_read(&nvmap_stats.collect) || stat == NS_TOTAL)
		atomic64_add(size, &nvmap_stats.stats[stat]);
}

void nvmap_stats_dec(enum nvmap_stats_t stat, size_t size)
{
	if (atomic64_read(&nvmap_stats.collect) || stat == NS_TOTAL)
		atomic64_sub(size, &nvmap_stats.stats[stat]);
}

u64 nvmap_stats_read(enum nvmap_stats_t stat)
{
	return atomic64_read(&nvmap_stats.stats[stat]);
}

