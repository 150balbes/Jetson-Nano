/*
 * drivers/video/tegra/dc/ps8625_edp2lvds.h
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Gaurav Singh <gaursingh@nvidia.com>
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_PS8625_EDP2LVDS_H
#define __DRIVERS_VIDEO_TEGRA_DC_PS8625_EDP2LVDS_H

struct tegra_dc_edp2lvds_data {
	struct tegra_dc_dp_data *dp;
	struct i2c_client *client_i2c;
	bool edp2lvds_enabled;
	struct mutex lock;
};

#endif
