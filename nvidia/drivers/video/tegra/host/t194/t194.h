/*
 * Tegra Graphics Chip support for T194
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
#ifndef _NVHOST_T194_H_
#define _NVHOST_T194_H_

#include "chip_support.h"

#define T194_NVHOST_NUMCHANNELS 63

extern struct nvhost_device_data t19_host1x_info;
extern struct nvhost_device_data t19_host1x_hv_info;
extern struct nvhost_device_data t19_host1xb_info;
#if defined(CONFIG_VIDEO_TEGRA_VI)
extern struct nvhost_device_data t19_vi_thi_info;
extern struct nvhost_device_data t19_vi5_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVCSI)
extern struct nvhost_device_data t19_nvcsi_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_ISP)
extern struct nvhost_device_data t19_isp_thi_info;
extern struct nvhost_device_data t19_isp5_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_VIC)
extern struct nvhost_device_data t19_vic_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVDEC)
extern struct nvhost_device_data t19_nvdec_info;
extern struct nvhost_device_data t19_nvdec1_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVJPG)
extern struct nvhost_device_data t19_nvjpg_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVENC)
extern struct nvhost_device_data t19_msenc_info;
extern struct nvhost_device_data t19_nvenc1_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_TSEC)
extern struct nvhost_device_data t19_tsec_info;
extern struct nvhost_device_data t19_tsecb_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_PVA)
extern struct nvhost_device_data t19_pva0_info;
extern struct nvhost_device_data t19_pva1_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_NVDLA)
extern struct nvhost_device_data t19_nvdla0_info;
extern struct nvhost_device_data t19_nvdla1_info;
#endif
#if defined(CONFIG_TEGRA_GRHOST_SLVSEC)
extern struct nvhost_device_data t19_slvsec_info;
#endif


int nvhost_init_t194_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op);
int nvhost_init_t194_channel_support(struct nvhost_master *host,
			     struct nvhost_chip_support *op);

#endif /* _NVHOST_T194_H_ */
