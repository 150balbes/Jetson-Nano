/*
 * dsi_padctrl.h: dsi padcontrol function definitions.
 * 
 * Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/of.h>
#include <linux/tegra_prod.h>
#include "dc.h"

#define DSI_MAX_INSTANCES	4

struct tegra_dsi_padctrl {
	void __iomem *base;
	struct reset_control *reset;
	u8 pwr_dwn_mask[DSI_MAX_INSTANCES];
	struct tegra_prod *prod_list;
	bool dsi_pads_enabled;
};

/* Defined in dsi_padctrl.c and used in dsi.c */
struct tegra_dsi_padctrl *tegra_dsi_padctrl_init(struct tegra_dc *dc);
/* Defined in dsi_padctrl.c and used in dsi.c */
void tegra_dsi_padctrl_shutdown(struct tegra_dc *dc);
/* Defined in dsi_padctrl.c and used in dsi.c */
void tegra_dsi_padctrl_enable(struct tegra_dsi_padctrl *dsi_padctrl);
/* Defined in dsi_padctrl.c and used in dsi.c */
void tegra_dsi_padctrl_disable(struct tegra_dsi_padctrl *dsi_padctrl);

