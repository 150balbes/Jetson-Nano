/*
 * tegra_l3_cache.h
 *
 * declarations for t19x cache
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __TEGRA_L3_CACHE_H
#define __TEGRA_L3_CACHE_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define  TEGRA_L3_CACHE_IOC_MAGIC 'C'

#if !defined(__KERNEL__)
#define __user
#endif

struct tegra_l3_ioctl_data {
	__u32 igpu_cpu_ways; /* integrated gpu */
	__u32 igpu_only_ways;
	__u32 total_ways;
	__u32 reserved;
	__u64 size;
};

#define TEGRA_L3_CACHE_GET_IOCTL_DATA    \
		_IOR(TEGRA_L3_CACHE_IOC_MAGIC, 1, struct tegra_l3_ioctl_data)

#define TEGRA_L3_CACHE_IOCTL_IOC_MAXNR _IOC_NR(TEGRA_L3_CACHE_GET_IOCTL_DATA)
#define TEGRA_L3_CACHE_IOCTL_MAX_ARG_SIZE  \
		sizeof(struct tegra_l3_ioctl_data)

#endif /*  __TEGRA_L3_CACHE_H */
