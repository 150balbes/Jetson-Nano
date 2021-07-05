/*
 * include/uapi/linux/nvhost_nvcsi_ioctl.h
 *
 * Tegra NVCSI Driver
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __UAPI_LINUX_NVHOST_NVCSI_IOCTL_H
#define __UAPI_LINUX_NVHOST_NVCSI_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

/* Bitmap
 *
 * |    PHY_2      |    PHY_1     |    PHY_0     |
 * | 11 10  |  9 8 |  7 6  |  5 4 |  3 2  |  1 0 |
 * |  CILB  | CILA |  CILB | CILA |  CILB | CILA |
 */
#define PHY_0_CIL_A_IO0		0
#define PHY_0_CIL_A_IO1		1
#define PHY_0_CIL_B_IO0		2
#define PHY_0_CIL_B_IO1		3

#define PHY_1_CIL_A_IO0		4
#define PHY_1_CIL_A_IO1		5
#define PHY_1_CIL_B_IO0		6
#define PHY_1_CIL_B_IO1		7

#define PHY_2_CIL_A_IO0		8
#define PHY_2_CIL_A_IO1		9
#define PHY_2_CIL_B_IO0		10
#define PHY_2_CIL_B_IO1		11

#define PHY_3_CIL_A_IO0		12
#define PHY_3_CIL_A_IO1		13
#define PHY_3_CIL_B_IO0		14
#define PHY_3_CIL_B_IO1		15
#define NVCSI_PHY_CIL_NUM_LANE	16

#define PHY_DPHY_MODE		0
#define PHY_CPHY_MODE		1

#define NVCSI_PHY_0_NVCSI_CIL_A_IO0	(0x1 << PHY_0_CIL_A_IO0)
#define NVCSI_PHY_0_NVCSI_CIL_A_IO1	(0x1 << PHY_0_CIL_A_IO1)
#define NVCSI_PHY_0_NVCSI_CIL_B_IO0	(0x1 << PHY_0_CIL_B_IO0)
#define NVCSI_PHY_0_NVCSI_CIL_B_IO1	(0x1 << PHY_0_CIL_B_IO1)

#define NVCSI_PHY_1_NVCSI_CIL_A_IO0	(0x1 << PHY_1_CIL_A_IO0)
#define NVCSI_PHY_1_NVCSI_CIL_A_IO1	(0x1 << PHY_1_CIL_A_IO1)
#define NVCSI_PHY_1_NVCSI_CIL_B_IO0	(0x1 << PHY_1_CIL_B_IO0)
#define NVCSI_PHY_1_NVCSI_CIL_B_IO1	(0x1 << PHY_1_CIL_B_IO1)

#define NVCSI_PHY_2_NVCSI_CIL_A_IO0	(0x1 << PHY_2_CIL_A_IO0)
#define NVCSI_PHY_2_NVCSI_CIL_A_IO1	(0x1 << PHY_2_CIL_A_IO1)
#define NVCSI_PHY_2_NVCSI_CIL_B_IO0	(0x1 << PHY_2_CIL_B_IO0)
#define NVCSI_PHY_2_NVCSI_CIL_B_IO1	(0x1 << PHY_2_CIL_B_IO1)

#define NVCSI_PHY_3_NVCSI_CIL_A_IO0	(0x1 << PHY_3_CIL_A_IO0)
#define NVCSI_PHY_3_NVCSI_CIL_A_IO1	(0x1 << PHY_3_CIL_A_IO1)
#define NVCSI_PHY_3_NVCSI_CIL_B_IO0	(0x1 << PHY_3_CIL_B_IO0)
#define NVCSI_PHY_3_NVCSI_CIL_B_IO1	(0x1 << PHY_3_CIL_B_IO1)

#define NVCSI_PHY_NUM_BRICKS		4
#define NVHOST_NVCSI_IOCTL_MAGIC 'N'

#define NVHOST_NVCSI_IOCTL_DESKEW_SETUP	_IOW(NVHOST_NVCSI_IOCTL_MAGIC, 1, long)
#define NVHOST_NVCSI_IOCTL_DESKEW_APPLY	_IOW(NVHOST_NVCSI_IOCTL_MAGIC, 2, long)
#define NVHOST_NVCSI_IOCTL_PROD_APPLY	_IOW(NVHOST_NVCSI_IOCTL_MAGIC, 3, long)

#endif
