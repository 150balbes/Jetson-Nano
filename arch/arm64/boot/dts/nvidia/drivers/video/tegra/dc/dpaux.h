/*
 * dpaux.h: dpaux headers.
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __DRIVER_VIDEO_TEGRA_DC_DPAUX_H__
#define __DRIVER_VIDEO_TEGRA_DC_DPAUX_H__

#define DP_AUX_MAX_BYTES 16
#define DP_AUX_TIMEOUT_MS 1000
#define DP_AUX_DEFER_MAX_TRIES 7
#define DP_AUX_TIMEOUT_MAX_TRIES 3
#define DP_DPCP_RETRY_SLEEP_US 400

struct tegra_dc_dpaux_data {
	struct tegra_dc *dc;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	struct tegra_prod *prod_list;
	struct device_node *np; /* dpaux@******** */
	atomic_t enable_count;
	struct mutex lock;
	int ctrl_num; /* this should match with sor->ctrl_num */
	int powergate_id;
};

enum tegra_dpaux_pad_mode {
	TEGRA_DPAUX_PAD_MODE_AUX = 0,
	TEGRA_DPAUX_PAD_MODE_I2C = 1,
};

int tegra_dpaux_readl(struct tegra_dc_dpaux_data *dpaux, u32 reg);
void tegra_dpaux_writel(struct tegra_dc_dpaux_data *dpaux, u32 reg, u32 val);
void tegra_dpaux_write_field(struct tegra_dc_dpaux_data *dpaux, u32 reg,
				u32 mask, u32 val);
void tegra_dpaux_get(struct tegra_dc_dpaux_data *dpaux);
void tegra_dpaux_put(struct tegra_dc_dpaux_data *dpaux);
int tegra_dpaux_clk_en(struct tegra_dc_dpaux_data *dpaux);
void tegra_dpaux_clk_dis(struct tegra_dc_dpaux_data *dpaux);
void tegra_dpaux_int_toggle(struct tegra_dc_dpaux_data *dpaux, u32 intr,
				bool enable);
void tegra_dpaux_pad_power(struct tegra_dc_dpaux_data *dpaux, bool on);
void tegra_dpaux_config_pad_mode(struct tegra_dc_dpaux_data *dpaux,
				enum tegra_dpaux_pad_mode mode);
void tegra_dpaux_prod_set(struct tegra_dc_dpaux_data *dpaux);
struct tegra_dc_dpaux_data *tegra_dpaux_init_data(struct tegra_dc *dc,
				struct device_node *sor_np);
void tegra_dpaux_destroy_data(struct tegra_dc_dpaux_data *dpaux);
int tegra_dpaux_get_irq(struct tegra_dc_dpaux_data *dpaux);
int tegra_dc_dpaux_read_chunk_locked(struct tegra_dc_dpaux_data *dpaux,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat);
int tegra_dc_dpaux_write_chunk_locked(struct tegra_dc_dpaux_data *dpaux,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat);
int tegra_dc_dpaux_read(struct tegra_dc_dpaux_data *dpaux, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat);
int tegra_dc_dpaux_write(struct tegra_dc_dpaux_data *dpaux, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat);
int tegra_dc_dpaux_i2c_read(struct tegra_dc_dpaux_data *dpaux, u32 i2c_addr,
				u8 *data, u32 *size, u32 *aux_stat);
int tegra_dc_dpaux_i2c_write(struct tegra_dc_dpaux_data *dpaux, u32 cmd,
			u32 i2c_addr, u8 *data, u32 *size, u32 *aux_stat);
struct clk *tegra_dpaux_get_clk(struct tegra_dc_dpaux_data *dpaux,
				const char *clk_name);
#endif
