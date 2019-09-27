/**
 * sensor_common.h - utilities for tegra camera driver
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

#ifndef __sensor_common__
#define __sensor_common__

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include <linux/kernel.h>
#include <linux/debugfs.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <linux/v4l2-mediabus.h>
#include <media/tegra-v4l2-camera.h>
#include <uapi/media/camera_device.h>

struct sensor_properties {
	struct sensor_cfg cfg;
	/* sensor_modes points to an array of mode properties */
	struct sensor_mode_properties *sensor_modes;
	u32 num_modes;
};

int sensor_common_parse_num_modes(const struct device *dev);
int sensor_common_init_sensor_properties(
	struct device *dev, struct device_node *node,
	struct sensor_properties *sensor);

#endif /* __sensor_common__ */
