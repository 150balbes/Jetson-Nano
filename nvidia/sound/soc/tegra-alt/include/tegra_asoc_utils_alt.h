/*
 * tegra_alt_asoc_utils.h - Definitions for MCLK and DAP Utility driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2011-2019 NVIDIA CORPORATION.	All rights reserved.
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

/*
 * Maintain same order in DT entry
 * FIXME: (This would be removed going ahead)
 */
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
	struct clk *clk_pll_base;
	struct clk *clk_pll_out;
	struct clk *clk_aud_mclk;
	struct reset_control *clk_cdev1_rst;
	int clk_cdev1_state;
	unsigned int *pll_base_rate;
	u32 set_pll_base_rate;
	u32 set_pll_out_rate;
	u32 set_aud_mclk_rate;
	u32 mclk_scale;

	/* FIXME: below would be removed going ahead */
	u32 clk_rates[MAX_NUM_RATES];
	u32 num_clk;
};

int tegra_alt_asoc_utils_set_rate(struct tegra_asoc_audio_clock_info *data,
				  unsigned int srate, unsigned int mclk,
				  unsigned int clk_out_rate);
int tegra_alt_asoc_utils_init(struct tegra_asoc_audio_clock_info *data,
			      struct device *dev, struct snd_soc_card *card);
int tegra_alt_asoc_utils_clk_enable(struct tegra_asoc_audio_clock_info *data);
int tegra_alt_asoc_utils_clk_disable(struct tegra_asoc_audio_clock_info *data);

#endif
