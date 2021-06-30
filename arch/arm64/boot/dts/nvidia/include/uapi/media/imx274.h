/**
 * Copyright (c) 2016-2019, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UAPI_IMX274_H__
#define __UAPI_IMX274_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define IMX274_IOCTL_SET_MODE		_IOW('o', 1, struct imx274_mode)
#define IMX274_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define IMX274_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX274_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX274_IOCTL_SET_GAIN		_IOW('o', 5, __u16)
#define IMX274_IOCTL_GET_SENSORDATA	_IOR('o', 6, \
						struct imx274_sensordata)
#define IMX274_IOCTL_SET_GROUP_HOLD	_IOW('o', 7, struct imx274_ae)
#define IMX274_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, \
							struct imx274_hdr)
#define IMX274_IOCTL_SET_POWER		_IOW('o', 20, __u32)

struct imx274_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct imx274_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct imx274_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

#endif  /* __UAPI_IMX274_H__ */
