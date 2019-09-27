/*
 * hda_dc.h: hd audio dc driver.
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 * Author: Rahul Mittal <rmittal@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_HDA_DC_H__
#define __DRIVERS_VIDEO_TEGRA_DC_HDA_DC_H__

enum tegra_dc_hda_state {
	HDA_UNINITIALIZED = 0,
	HDA_INITIALIZED,
	HDA_ENABLED = 3,
};

struct tegra_dc_hda_data {
	int dev_id;
	struct tegra_dc_sor_data *sor;
	struct tegra_dc *dc;
	struct tegra_edid_hdmi_eld *eld;
	int sink;
	bool null_sample_inject;
	bool *eld_valid;
	bool *enabled;
	u32 audio_freq;
	struct clk *pll_p_clk;
	struct clk *hda_clk;
	struct clk *hda2codec_clk;
	struct clk *hda2hdmi_clk;
	struct clk *maud_clk;
	void *client_data;
	char *audio_switch_name;
};

struct tegra_hda_inst {
	enum tegra_dc_hda_state hda_state; /* hda state */
	struct tegra_dc_hda_data *hda;
	struct mutex hda_inst_lock;
};

void tegra_hda_enable(void *hda_handle);
void tegra_hda_disable(void *hda_handle);
int tegra_hda_get_switch_name(int dev_id, char *name);
int tegra_hdmi_setup_hda_presence(int sor_num);
int tegra_hda_get_dev_id(struct tegra_dc_sor_data *sor);
void tegra_hda_init(struct tegra_dc *dc, void *data);
void tegra_hda_destroy(void *hda_handle);

#endif
