/*
 * drivers/video/tegra/camera/tegra_camera_common.h
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _UAPI_TEGRA_CAMERA_PLATFORM_H_
#define _UAPI_TEGRA_CAMERA_PLATFORM_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define TEGRA_CAMERA_IOCTL_SET_BW _IOW('o', 1, struct bw_info)
#define TEGRA_CAMERA_IOCTL_GET_BW _IOR('o', 2, __u64)
#define TEGRA_CAMERA_IOCTL_GET_CURR_REQ_ISO_BW _IOR('o', 3, __u64)

struct bw_info {
	__u8 is_iso;
	__u64 bw;
};

#endif