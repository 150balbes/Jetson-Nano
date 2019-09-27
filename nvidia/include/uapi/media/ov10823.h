/*
 * ov10823.h - ov10823 sensor driver
 *
 * Copyright (c) 2016-2019 NVIDIA Corporation.  All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef __UAPI_OV10823_H__
#define __UAPI_OV10823_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define OV10823_IOCTL_SET_MODE	_IOW('o', 1, struct ov10823_mode)
#define OV10823_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define OV10823_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define OV10823_IOCTL_SET_GAIN	_IOW('o', 4, __u16)
#define OV10823_IOCTL_GET_STATUS	_IOR('o', 5, __u8)
#define OV10823_IOCTL_SET_GROUP_HOLD	_IOW('o', 6, struct ov10823_grouphold)
#define OV10823_IOCTL_GET_SENSORDATA	_IOR('o', 7, struct ov10823_sensordata)
#define OV10823_IOCTL_SET_FLASH	_IOW('o', 8, struct ov10823_flash_control)
#define OV10823_IOCTL_SET_POWER          _IOW('o', 20, __u32)

#define OV10823_FUSE_ID_OTP_BASE_ADDR	0x6000
#define OV10823_FUSE_ID_SIZE			16
#define OV10823_FUSE_ID_STR_SIZE	(OV10823_FUSE_ID_SIZE * 2)

#define OV10823_GROUP_HOLD_ADDR		0x3208

struct ov10823_sensordata {
	__u32 fuse_id_size;
	__u8 fuse_id[16];
};

struct ov10823_mode {
	__u32 xres;
	__u32 yres;
	__u32 fps;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
	__u8 fsync_master;
};

struct ov10823_grouphold {
	__u32	frame_length;
	__u8	frame_length_enable;
	__u32	coarse_time;
	__u8	coarse_time_enable;
	__s32	gain;
	__u8	gain_enable;
};

struct ov10823_flash_control {
	__u8 enable;
	__u8 edge_trig_en;
	__u8 start_edge;
	__u8 repeat;
	__u16 delay_frm;
};

#endif /* __UAPI_OV10823_H__ */