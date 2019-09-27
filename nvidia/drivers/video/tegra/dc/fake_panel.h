/*
 * fake_panel.h: fake panel driver.
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVER_VIDEO_TEGRA_DC_FAKE_PANEL_H__
#define __DRIVER_VIDEO_TEGRA_DC_FAKE_PANEL_H__

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/version.h>

#include <linux/regulator/consumer.h>

#include "dc.h"
#include "sor.h"
#include "dp.h"
#include "dsi.h"

int tegra_dc_init_fakedsi_panel(struct tegra_dc *dc, long dc_outtype);
int tegra_dc_reinit_dsi_resources(struct tegra_dc *dc, long dc_outtype);
int tegra_dc_destroy_dsi_resources(struct tegra_dc *dc, long dc_outtype);

#endif
