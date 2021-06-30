/*
 * FPDLink Deserializer driver
 *
 * Copyright (C) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __DRIVERS_VIDEO_DS90UH948_H
#define __DRIVERS_VIDEO_DS90UH948_H

#include <linux/types.h>

#define DS90UH948_DESER_REG_GENCFG			0x03
#define DS90UH948_DESER_REG_GENCFG_I2C_PASSTHRU		3
#define DS90UH948_DESER_REG_GENCFG_FILTER_EN		4
#define DS90UH948_DESER_REG_GPIO1_2			0x1E
#define DS90UH948_DESER_REG_GPIO1_2_GPIO1_EN		0
#define DS90UH948_DESER_REG_GPIO1_2_GPIO1_DIR		1
#define DS90UH948_DESER_REG_GPIO1_2_GPIO1_VAL		3
#define DS90UH948_DESER_REG_GPIO1_2_GPIO2_EN		4
#define DS90UH948_DESER_REG_GPIO1_2_GPIO2_DIR		5
#define DS90UH948_DESER_REG_GPIO1_2_GPIO2_VAL		7
#define DS90UH948_DESER_REG_GPIO3			0x1F
#define DS90UH948_DESER_REG_GPIO3_GPIO3_EN		0
#define DS90UH948_DESER_REG_GPIO3_GPIO3_DIR		1
#define DS90UH948_DESER_REG_GPIO3_GPIO3_VAL		3
#define DS90UH948_DESER_REG_GPIO5_6			0x20
#define DS90UH948_DESER_REG_GPIO5_6_GPIO5_EN		0
#define DS90UH948_DESER_REG_GPIO5_6_GPIO5_DIR		1
#define DS90UH948_DESER_REG_GPIO5_6_GPIO5_VAL		3
#define DS90UH948_DESER_REG_GPIO5_6_GPIO6_EN		4
#define DS90UH948_DESER_REG_GPIO5_6_GPIO6_DIR		5
#define DS90UH948_DESER_REG_GPIO5_6_GPIO6_VAL		7
#define DS90UH948_DESER_REG_GPIO7_8			0x21
#define DS90UH948_DESER_REG_GPIO7_8_GPIO7_EN		0
#define DS90UH948_DESER_REG_GPIO7_8_GPIO7_DIR		1
#define DS90UH948_DESER_REG_GPIO7_8_GPIO7_VAL		3
#define DS90UH948_DESER_REG_GPIO7_8_GPIO8_EN		4
#define DS90UH948_DESER_REG_GPIO7_8_GPIO8_DIR		5
#define DS90UH948_DESER_REG_GPIO7_8_GPIO8_VAL		7

struct ds90uh948_data {
	int en_bchnl_i2c;
	int en_2clk_filter;
	int en_gpio1;
	int en_gpio2;
	int en_gpio3;
	int en_gpio5;
	int en_gpio6;
	int en_gpio7;
	int en_gpio8;
	int gpio1_direction; /* 0 = Output, 1 = Input */
	int gpio2_direction; /* 0 = Output, 1 = Input */
	int gpio3_direction; /* 0 = Output, 1 = Input */
	int gpio5_direction; /* 0 = Output, 1 = Input */
	int gpio6_direction; /* 0 = Output, 1 = Input */
	int gpio7_direction; /* 0 = Output, 1 = Input */
	int gpio8_direction; /* 0 = Output, 1 = Input */
	int gpio1_value;
	int gpio2_value;
	int gpio3_value;
	int gpio5_value;
	int gpio6_value;
	int gpio7_value;
	int gpio8_value;
	struct regmap *regmap;
};

#endif /* __DRIVERS_VIDEO_DS90UH948_H */
