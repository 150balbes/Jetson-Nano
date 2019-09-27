/**
 * tegracam_core.h - tegra camera framework core utilities
 *
 * Copyright (c) 2017-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef __TEGRACAM_CORE_H__
#define __TEGRACAM_CORE_H__

#include <media/camera_common.h>

struct tegracam_device {
	struct camera_common_data	*s_data;
	struct media_pad		pad;
	u32 				version;
	bool				is_streaming;
	/* variables to be filled by the driver to register */
	char				name[32];
	struct i2c_client		*client;
	struct device			*dev;
	u32				numctrls;
	const u32			*ctrl_cid_list;
	const struct regmap_config	*dev_regmap_config;
	struct camera_common_sensor_ops		*sensor_ops;
	const struct v4l2_subdev_ops		*v4l2sd_ops;
	const struct v4l2_subdev_internal_ops	*v4l2sd_internal_ops;
	const struct media_entity_operations	*media_ops;
	const struct tegracam_ctrl_ops		*tcctrl_ops;
	void	*priv;
};

u32 tegracam_version(u8 major, u8 minor, u8 patch);
u32 tegracam_query_version(const char *of_dev_name);
struct tegracam_device *to_tegracam_device(struct camera_common_data *data);

void tegracam_set_privdata(struct tegracam_device *tc_dev, void *priv);
void *tegracam_get_privdata(struct tegracam_device *tc_dev);

int tegracam_v4l2subdev_register(struct tegracam_device *tc_dev,
				bool is_sensor);
void tegracam_v4l2subdev_unregister(struct tegracam_device *tc_dev);
int tegracam_device_register(struct tegracam_device *tc_dev);
void tegracam_device_unregister(struct tegracam_device *tc_dev);
#endif
