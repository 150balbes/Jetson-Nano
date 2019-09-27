/*
 * ds90ub947-q1.h: lvds to fpdlink ds90ub947 controller driver.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Author:
 *	Deepak Bhosale <dbhosale@nvidia.com>
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DS90UB947_LVDS2FPDLINK_H
#define __DRIVERS_VIDEO_TEGRA_DC_DS90UB947_LVDS2FPDLINK_H

struct ds90ub947_data {
	/* app device */
	struct i2c_client *client;
	struct regmap *regmap;
	u32 *init_regs;
	u32 n_init_regs;
	struct dentry *debugdir;
	struct mutex lock;
	int en_gpio; /* GPIO */
	int en_gpio_flags;
	int en_bchnl_i2c_passthrough;
	int en_bchnl_irq;
	int deser_address;
	int slave_address;
	int slave_alias_address;
};

#define DEV_NAME "ds90ub947-q1"

#define DS90UB947_GENERAL_CONFIG	0x03
#define DS90UB947_DESER_ADDR		0x06
#define DS90UB947_SLAVE_ADDR		0x07
#define DS90UB947_SLAVE_ALIAS		0x08
#define	DS90UB947_GENERAL_STATUS	0x0C
#define DS90UB947_I2C_CONTROL		0x17
#define DS90UB947_INTR_CNTRL_REG	0xC6

#define DS90UB947_I2C_PASSTHROUGH_BIT		0x03
#define DS90UB947_I2C_PASSALL_BIT		0x07
#define DS90UB947_INTR_ENABLE_BIT		0x00
#define DS90UB947_INTR_RX_ENABLE_BIT		0x05
#define DS90UB947_DES_ID_SHIFT_WIDTH		0x01
#define DS90UB947_SLAVE_ADDR_SHIFT_WIDTH	0x01
#define DS90UB947_SLAVE_ALIAS_SHIFT_WIDTH	0x01

#endif
