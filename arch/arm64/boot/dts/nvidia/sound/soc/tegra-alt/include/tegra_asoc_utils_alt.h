/*
 * tegra_alt_asoc_utils.h - Definitions for MCLK and DAP Utility driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2011-2018 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TEGRA_ASOC_UTILS_ALT_H_
#define __TEGRA_ASOC_UTILS_ALT_H_

struct clk;
struct device;

enum tegra_asoc_utils_soc {
	TEGRA_ASOC_UTILS_SOC_TEGRA210,
	TEGRA_ASOC_UTILS_SOC_TEGRA186,
	TEGRA_ASOC_UTILS_SOC_TEGRA194,
};

/* Maintain same order in DT entry */
enum tegra_asoc_utils_clkrate {
	PLLA_x11025_RATE,
	AUD_MCLK_x11025_RATE,
	PLLA_OUT0_x11025_RATE,
	AHUB_x11025_RATE,
	PLLA_x8000_RATE,
	AUD_MCLK_x8000_RATE,
	PLLA_OUT0_x8000_RATE,
	AHUB_x8000_RATE,
	MAX_NUM_RATES,
};

struct tegra_asoc_audio_clock_info {
	struct device *dev;
	struct snd_soc_card *card;
	enum tegra_asoc_utils_soc soc;
	struct clk *clk_pll_a;
	struct clk *clk_pll_a_out0;
	struct clk *clk_cdev1;
	struct clk *clk_ahub;
	struct reset_control *clk_cdev1_rst;
	int clk_cdev1_state;
	struct clk *clk_m;
	struct clk *clk_pll_p_out1;
	int set_mclk;
	int lock_count;
	int set_baseclock;
	int num_clk;
	struct clk *clk_mclk_parent;
	u32 set_clk_out_rate;
	u32 mclk_rate;
	u32 mclk_scale;
	u32 clk_rates[MAX_NUM_RATES];
};

int tegra_alt_asoc_utils_set_rate(struct tegra_asoc_audio_clock_info *data,
				int srate,
				int mclk,
				u32 clk_out_rate);
void tegra_alt_asoc_utils_lock_clk_rate(
				struct tegra_asoc_audio_clock_info *data,
				int lock);
int tegra_alt_asoc_utils_init(struct tegra_asoc_audio_clock_info *data,
				struct device *dev, struct snd_soc_card *card);

int tegra_alt_asoc_utils_set_extern_parent(
	struct tegra_asoc_audio_clock_info *data, const char *parent);
int tegra_alt_asoc_utils_set_parent(struct tegra_asoc_audio_clock_info *data,
				int is_i2s_master);
int tegra_alt_asoc_utils_clk_enable(struct tegra_asoc_audio_clock_info *data);
int tegra_alt_asoc_utils_clk_disable(struct tegra_asoc_audio_clock_info *data);
int tegra_alt_asoc_utils_register_ctls(struct tegra_asoc_audio_clock_info *data);

int tegra_alt_asoc_utils_tristate_dap(int id, bool tristate);

#endif
