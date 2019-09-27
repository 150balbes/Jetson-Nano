/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

/* TODO set all define values correctly, copied from ov5693*/
#ifndef __IMX318_H__
#define __IMX318_H__

#include <media/nvc.h>
#include <uapi/media/nvc_image.h>
#include <uapi/media/imx318.h>

#define IMX318_INVALID_COARSE_TIME	-1

#define IMX318_EEPROM_ADDRESS		0x54
#define IMX318_EEPROM_SIZE		256
#define IMX318_EEPROM_STR_SIZE		(IMX318_EEPROM_SIZE * 2)
#define IMX318_EEPROM_BLOCK_SIZE	(1 << 8)
#define IMX318_EEPROM_NUM_BLOCKS \
	(IMX318_EEPROM_SIZE / IMX318_EEPROM_BLOCK_SIZE)

/* Incorrect data, cannot find fuse ID in documentation */
#define IMX318_FUSE_ID_START_ADDR	0x5b
#define IMX318_FUSE_ID_BANK		0
#define IMX318_FUSE_ID_SIZE		8
#define IMX318_FUSE_ID_STR_SIZE		(IMX318_FUSE_ID_SIZE * 2)

/* See notes in the nvc.h file on the GPIO usage */
enum imx318_gpio_type {
	IMX318_GPIO_TYPE_PWRDN = 0,
	IMX318_GPIO_TYPE_RESET,
};

struct imx318_eeprom_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

struct imx318_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *dovdd;
};

struct imx318_regulators {
	const char *avdd;
	const char *dvdd;
	const char *dovdd;
};

struct imx318_platform_data {
	unsigned int cfg;
	unsigned int num;
	const char *dev_name;
	unsigned int gpio_count; /* see nvc.h GPIO notes */
	struct nvc_gpio_pdata *gpio; /* see nvc.h GPIO notes */
	struct nvc_imager_static_nvc *static_info;
	bool use_vcm_vdd;
	int (*probe_clock)(unsigned long);
	int (*power_on)(struct imx318_power_rail *);
	int (*power_off)(struct imx318_power_rail *);
	const char *mclk_name;
	struct nvc_imager_cap *cap;
	struct imx318_regulators regulators;
	bool has_eeprom;
	bool use_cam_gpio;
};

#endif  /* __IMX318_H__ */
