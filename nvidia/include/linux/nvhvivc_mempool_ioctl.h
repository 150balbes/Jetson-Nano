/*
 * nvhvivc_mempool_ioctl.h
 *
 * Declarations for Tegra Hypervisor ivc mempool driver ioctls
 *
 * Copyright (c) 2016 NVIDIA CORPORATION.  All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */
#ifndef __NVHVIVC_MEMPOOL_IOCTL_H__
#define __NVHVIVC_MEMPOOL_IOCTL_H__

#include <linux/ioctl.h>

/* ivc mempool IOCTL magic number */
#define TEGRA_MPLUSERSPACE_IOCTL_MAGIC 0xA6


/* IOCTL definitions */

/* query ivc mempool configuration data */
#define TEGRA_MPLUSERSPACE_IOCTL_GET_INFO \
	_IOR(TEGRA_MPLUSERSPACE_IOCTL_MAGIC, 1, struct ivc_mempool)

#define TEGRA_MPLUSERSPACE_IOCTL_NUMBER_MAX 1

#endif /* __NVHVIVC_MEMPOOL_IOCTL_H__ */
