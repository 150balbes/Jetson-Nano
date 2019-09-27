/*
 * Tegra Host Module Class IDs for T194
 *
 * Copyright (c) 2016-2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_CLASS_IDS_T194_H
#define __NVHOST_CLASS_IDS_T194_H

enum {
	NV_VIDEO_ENCODE_NVENC1_CLASS_ID = 0x22,
	NV_VIDEO_STREAMING_VI_FALCON_CLASS_ID = 0x31,
	NV_VIDEO_STREAMING_NVCSI_CLASS_ID = 0x38,

	NV_PVA0_CLASS_ID	= 0xF1,
	NV_PVA1_CLASS_ID	= 0xF2,

	NV_DLA0_CLASS_ID	= 0xF3,
	NV_DLA1_CLASS_ID	= 0xF4,

	NV_NVDEC1_CLASS_ID	= 0xF5,

	NV_SLVSEC_CLASS_ID	= 0x3F,
};

#endif /*__NVHOST_CLASS_IDS_T194_H */
