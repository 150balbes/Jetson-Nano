/*
 * arch/arm/mach-tegra/tegra_emc.h
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef _MACH_TEGRA_TEGRA_EMC_H
#define _MACH_TEGRA_TEGRA_EMC_H

#define TEGRA_EMC_ISO_USE_CASES_MAX_NUM		8

extern u8 tegra_emc_bw_efficiency;
extern u8 tegra_emc_iso_share;

enum {
	DRAM_OVER_TEMP_NONE = 0,
	DRAM_OVER_TEMP_REFRESH_X2,
	DRAM_OVER_TEMP_REFRESH_X4,
	DRAM_OVER_TEMP_THROTTLE, /* 4x Refresh + derating. */
};

enum emc_user_id {
	EMC_USER_DC1 = 0,
	EMC_USER_DC2,
	EMC_USER_VI,
	EMC_USER_MSENC,
	EMC_USER_2D,
	EMC_USER_3D,
	EMC_USER_BB,
	EMC_USER_VDE,
	EMC_USER_VI2,
	EMC_USER_ISP1,
	EMC_USER_ISP2,
	EMC_USER_NVDEC,
	EMC_USER_NVJPG,
	EMC_USER_NUM,
};

struct emc_iso_usage {
	u32 emc_usage_flags;
	u8 iso_usage_share;
	u8 (*iso_share_calculator)(unsigned long iso_bw);
};

struct tegra_emc_dvfs_table_ops {
	u32 (*get_dvfs_clk_change_latency_nsec)(unsigned long emc_freq_khz);
};

struct clk;
struct dentry;

void tegra_emc_timer_mr4_start(void);
void tegra_emc_timer_mr4_stop(void);
void tegra_emc_timer_training_start(void);
void tegra_emc_timer_training_stop(void);

void tegra_emc_iso_usage_table_init(struct emc_iso_usage *table, int size);
int  tegra_emc_iso_usage_debugfs_init(struct dentry *emc_debugfs_root);
int tegra_emc_timers_init(struct dentry *parent);
void tegra_emc_dvfs_table_ops_init(
		struct tegra_emc_dvfs_table_ops *dvfs_table_ops_to_copy);
unsigned long tegra_emc_apply_efficiency(unsigned long total_bw,
	unsigned long iso_bw, unsigned long max_rate, u32 usage_flags,
	unsigned long *iso_bw_min);
void tegra_emc_dram_type_init(struct clk *c);
int tegra_emc_get_dram_type(void);
int tegra_emc_get_dram_temperature(void);
int tegra_emc_set_over_temp_state(unsigned long state);
void tegra_emc_mr4_set_freq_thresh(unsigned long thresh);
void tegra_emc_mr4_freq_check(unsigned long freq);

u32 emc_do_periodic_compensation(void);

long tegra_emc_round_rate(unsigned long rate);
long tegra_emc_round_rate_updown(unsigned long rate, bool up);
struct clk *tegra_emc_predict_parent(unsigned long rate, u32 *div_value);
void tegra_emc_timing_invalidate(void);
void tegra_mc_divider_update(struct clk *emc);

u32 tegra_get_dvfs_clk_change_latency_nsec(unsigned long emc_freq_khz);

#endif
