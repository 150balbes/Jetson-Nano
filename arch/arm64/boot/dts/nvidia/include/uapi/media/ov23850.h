/**
 * Copyright (c) 2014-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef __UAPI_OV23850_H__
#define __UAPI_OV23850_H__

#include <linux/ioctl.h>

#define OV23850_IOCTL_SET_MODE			_IOW('o', 1, \
	 struct ov23850_mode)
#define OV23850_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define OV23850_IOCTL_SET_FRAME_LENGTH		_IOW('o', 3, __u32)
#define OV23850_IOCTL_SET_COARSE_TIME		_IOW('o', 4, __u32)
#define OV23850_IOCTL_SET_GAIN			_IOW('o', 5, __u16)
#define OV23850_IOCTL_GET_SENSORDATA		_IOR('o', 6, \
	 struct ov23850_sensordata)
#define OV23850_IOCTL_SET_GROUP_HOLD		_IOW('o', 7, struct ov23850_ae)
#define OV23850_IOCTL_SET_HDR_COARSE_TIME	_IOW('o', 8, struct ov23850_hdr)
#define OV23850_IOCTL_SET_POWER			_IOW('o', 20, __u32)

#define OV23850_FUSE_ID_OTP_START_ADDR  0x6000
#define OV23850_FUSE_ID_OTP_END_ADDR    0x600F
#define OV23850_FUSE_ID_SIZE \
	 (OV23850_FUSE_ID_OTP_END_ADDR - OV23850_FUSE_ID_OTP_START_ADDR + 1)
#define OV23850_FUSE_ID_STR_SIZE        (OV23850_FUSE_ID_SIZE * 2)

#define OV23850_FRAME_LENGTH_ADDR_MSB		0x380E
#define OV23850_FRAME_LENGTH_ADDR_LSB		0x380F
#define OV23850_COARSE_TIME_ADDR_MSB		0x3501
#define OV23850_COARSE_TIME_ADDR_LSB		0x3502
#define OV23850_COARSE_TIME_SHORT_ADDR_MSB	0x3507
#define OV23850_COARSE_TIME_SHORT_ADDR_LSB	0x3508
#define OV23850_GAIN_ADDR_MSB			0x350A
#define OV23850_GAIN_ADDR_LSB			0x350B
#define OV23850_GAIN_SHORT_ADDR_MSB			0x354E
#define OV23850_GAIN_SHORT_ADDR_LSB			0x354F
#define OV23850_GROUP_HOLD_ADDR			0x3208
#define OV23850_HDR_ADDR	0x3821

struct ov23850_mode {
	__u32 xres;
	__u32 yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct ov23850_hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct ov23850_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct ov23850_sensordata {
	__u32 fuse_id_size;
	__u8  fuse_id[OV23850_FUSE_ID_SIZE];
};

#endif  /* __UAPI_OV23850_H__ */