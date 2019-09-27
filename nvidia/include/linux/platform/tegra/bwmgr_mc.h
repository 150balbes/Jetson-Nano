/**
 * Copyright (c) 2015-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __BWMGR_MC_H
#define __BWMGR_MC_H

#include <linux/types.h>
#include <linux/platform/tegra/iso_client.h>

unsigned long bwmgr_apply_efficiency(
		unsigned long bw, unsigned long iso_bw,
		unsigned long emc_max, u64 usage_flags,
		unsigned long *iso_bw_min, unsigned long iso_bw_nvdis,
		unsigned long iso_bw_vi);

void bwmgr_eff_init(void);

unsigned long bwmgr_freq_to_bw(unsigned long freq);
unsigned long bwmgr_bw_to_freq(unsigned long bw);
unsigned long bwmgr_get_lowest_iso_emc_freq(long iso_bw,
		long iso_bw_nvdis, long iso_bw_vi);
u32 tegra_bwmgr_get_max_iso_bw(enum tegra_iso_client);

u32 bwmgr_dvfs_latency(u32 ufreq);
int bwmgr_iso_bw_percentage_max(void);
int bwmgr_get_emc_to_dram_freq_factor(void);
#endif /* __BWMGR_MC_H */
