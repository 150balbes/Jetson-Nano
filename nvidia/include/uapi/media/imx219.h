/**
 * include/uapi/linux/nvhvivc_mempool_ioctl.h
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __UAPI_IMX219_H__
#define __UAPI_IMX219_H__

#include <linux/ioctl.h>
#include <media/nvc.h>

#define IMX219_IOCTL_SET_MODE		_IOW('o', 1, struct imx219_mode)
#define IMX219_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define IMX219_IOCTL_SET_FRAME_LENGTH	_IOW('o', 3, __u32)
#define IMX219_IOCTL_SET_COARSE_TIME	_IOW('o', 4, __u32)
#define IMX219_IOCTL_SET_GAIN		_IOW('o', 5, struct imx219_gain)
#define IMX219_IOCTL_GET_FUSEID		_IOR('o', 6, struct nvc_fuseid)
#define IMX219_IOCTL_SET_GROUP_HOLD	_IOW('o', 7, struct imx219_ae)
#define IMX219_IOCTL_GET_AFDAT		_IOR('o', 8, __u32)
#define IMX219_IOCTL_SET_POWER		_IOW('o', 20, __u32)
#define IMX219_IOCTL_GET_FLASH_CAP	_IOR('o', 30, __u32)
#define IMX219_IOCTL_SET_FLASH_MODE	_IOW('o', 31, \
						struct imx219_flash_control)

/* TODO: revisit these values for IMX219 */
#define IMX219_FRAME_LENGTH_ADDR_MSB		0x0160
#define IMX219_FRAME_LENGTH_ADDR_LSB		0x0161
#define IMX219_COARSE_TIME_ADDR_MSB		0x015a
#define IMX219_COARSE_TIME_ADDR_LSB		0x015b
#define IMX219_GAIN_ADDR			0x0157

struct imx219_flash_control {
	__u8 enable;
	__u8 edge_trig_en;
	__u8 start_edge;
	__u8 repeat;
	__u16 delay_frm;
};

struct imx219_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 gain;
};

struct imx219_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u8  coarse_time_enable;
	__u32 gain;
	__u8  gain_enable;
};

#endif  /* __UAPI_IMX219_H__ */
