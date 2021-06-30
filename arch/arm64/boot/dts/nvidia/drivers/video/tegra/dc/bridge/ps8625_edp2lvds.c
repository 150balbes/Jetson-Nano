/*
 * drivers/video/tegra/dc/ps8625_edp2lvds.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/swab.h>
#include <linux/module.h>
#include "dc.h"
#include "dc_priv.h"
#include "ps8625_edp2lvds.h"
#include "dp.h"

static struct tegra_dc_edp2lvds_data *edp2lvds;

struct i2c_cmd_for_PS8625 {
	u8 i2c_addr;
	u8 reg_offset;
	u8 data;
};

static struct i2c_cmd_for_PS8625 ps8625_init_sequence[] = {
	/* HPD LOW */
	{0x0a, 0xa1, 0x01},
	{0x0c, 0x14, 0x01},
	{0x0c, 0xe3, 0x20},
	{0x0c, 0xe2, 0x80},
	{0x0c, 0x8a, 0x0c},
	{0x0c, 0x89, 0x08},
	{0x0c, 0x71, 0x2d},
	{0x0c, 0x7d, 0x04},
	{0x0c, 0x7b, 0x00},
	{0x0c, 0x7a, 0xfd},
	{0x0c, 0xc0, 0x12},
	{0x0c, 0xc1, 0x92},
	{0x0c, 0xc2, 0x1c},
	{0x0c, 0x32, 0x80},
	{0x0c, 0x00, 0x30},
	{0x0c, 0x15, 0x40},
	{0x0c, 0x54, 0x10},
	{0x08, 0x52, 0x20},
	{0x08, 0xf1, 0x03},
	{0x08, 0x62, 0x41},
	{0x08, 0xf6, 0x01},
	{0x0a, 0xa1, 0x10},
	{0x08, 0x77, 0x06},
	{0x08, 0x4c, 0x04},
	{0x09, 0xc0, 0x00},
	{0x09, 0xc1, 0x1c},
	{0x09, 0xc2, 0xf8},
	{0x09, 0xc3, 0x44},
	{0x09, 0xc4, 0x32},
	{0x09, 0xc5, 0x44},
	{0x09, 0xc6, 0x4c},
	{0x09, 0xc7, 0x56},
	{0x09, 0xc8, 0x35},
	{0x09, 0xca, 0x01},
	{0x09, 0xcb, 0x07},

/* Customized Setting example Start */

/* Enable internal PWM output */
	{0x09, 0xa5, 0xa0},

/* FFh for 100% PWM of brightness and 00h for 0% PWM of brightness */
	{0x09, 0xa7, 0xff},
/* Set LVDS output as 8bit-VESA mapping  Dual LVDS channel */
	{0x09, 0xcc, 0x14},

/*
 * This will overwrite pin configuration RLV_CFG & RLV_LNK
 * (but this config is actually the same as E2606-A00 strap)
 */

/* Nvidia E2606-A00 board related setting Start */
	{0x0b, 0x5c, 0xff},

/*
 * Nvidia E2606-A00 board related setting End
 * Customized Setting  example End
 */
	{0x0c, 0x59, 0x60},
	{0x0c, 0x54, 0x14},
	/* HPD = High */
	{0x0a, 0xa1, 0x91},
};

static struct i2c_driver tegra_edp2lvds_i2c_slave_driver = {
	.driver = {
		.name = "edp2lvds_bridge",
	},
};

static struct i2c_client *init_i2c_slave(struct tegra_dc_dp_data *dp)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info p_data = {
		.type = "edp2lvds_bridge",
		.addr = 0x0a,
	};
	int err = 0;
	int bus_no = dp->pdata->edp2lvds_i2c_bus_no;

	adapter = i2c_get_adapter(bus_no);
	if (!adapter) {
		dev_err(&dp->dc->ndev->dev,
			"edp2lvds: can't get adpater for bus %d\n", bus_no);
		err = -EBUSY;
		goto err;
	}

	client = i2c_new_device(adapter, &p_data);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&dp->dc->ndev->dev,
			"edp2lvds: can't add i2c slave device\n");
		err = -EBUSY;
		goto err;
	}

	err = i2c_add_driver(&tegra_edp2lvds_i2c_slave_driver);
	if (err) {
		dev_err(&dp->dc->ndev->dev,
			"edp2lvds: can't add i2c slave driver\n");
		goto err_free;
	}

	return client;
err:
	return ERR_PTR(err);
err_free:
	i2c_unregister_device(client);
	return ERR_PTR(err);
}

static int ps8625_edp2lvds_init(struct tegra_dc_dp_data *dp)
{
	int err = 0;

	if (edp2lvds) {
		tegra_dp_set_outdata(dp, edp2lvds);
		return err;
	}

	edp2lvds = devm_kzalloc(&dp->dc->ndev->dev, sizeof(*edp2lvds),
								GFP_KERNEL);
	if (!edp2lvds)
		return -ENOMEM;

	edp2lvds->client_i2c = init_i2c_slave(dp);
	if (IS_ERR_OR_NULL(edp2lvds->client_i2c)) {
		dev_err(&dp->dc->ndev->dev,
			"edp2lvds: i2c slave setup failure\n");
	}

	edp2lvds->dp = dp;

	tegra_dp_set_outdata(dp, edp2lvds);

	mutex_init(&edp2lvds->lock);

	return err;
}

static void ps8625_edp2lvds_destroy(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_edp2lvds_data *edp2lvds = tegra_dp_get_outdata(dp);

	if (!edp2lvds)
		return;

	mutex_lock(&edp2lvds->lock);
	i2c_del_driver(&tegra_edp2lvds_i2c_slave_driver);
	i2c_unregister_device(edp2lvds->client_i2c);
	mutex_unlock(&edp2lvds->lock);
	mutex_destroy(&edp2lvds->lock);
	kfree(edp2lvds);
}

static int ps8625_edp2lvds_enable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_edp2lvds_data *edp2lvds = tegra_dp_get_outdata(dp);
	struct i2c_adapter *adapter;
	struct i2c_msg msg;
	int count;
	int err = 0;
	int bus_no = dp->pdata->edp2lvds_i2c_bus_no;
	u8 buf[2];

	adapter = i2c_get_adapter(bus_no);
	if (!adapter) {
		dev_err(&dp->dc->ndev->dev,
			"edp2lvds: can't get adpater for bus:%d\n", bus_no);
		err = -EBUSY;
		return err;
	}

	if (edp2lvds && edp2lvds->edp2lvds_enabled)
		return err;

	for (count = 0; count < ARRAY_SIZE(ps8625_init_sequence); count++) {
		msg.addr = ps8625_init_sequence[count].i2c_addr;
		msg.flags = 0;
		msg.len = 2;

		buf[0] = ps8625_init_sequence[count].reg_offset;
		buf[1] = ps8625_init_sequence[count].data;

		msg.buf = &buf[0];

		err =  i2c_transfer(adapter, &msg, 1);
		if (err < 0)
			dev_err(&dp->dc->ndev->dev,
			"edp2lvds: i2c_write failed for count:%d addr:%x\n",
			count, msg.addr);
	}
	return err;
}

static void ps8625_edp2lvds_disable(struct tegra_dc_dp_data *dp)
{
	/* To be done */
}

#ifdef CONFIG_PM
static void ps8625_edp2lvds_suspend(struct tegra_dc_dp_data *dp)
{
	/* To be done */
}

static void ps8625_edp2lvds_resume(struct tegra_dc_dp_data *dp)
{
	/* To be done */
}
#endif

struct tegra_dp_out_ops tegra_edp2lvds_ops = {
	.init = ps8625_edp2lvds_init,
	.destroy = ps8625_edp2lvds_destroy,
	.enable = ps8625_edp2lvds_enable,
	.disable = ps8625_edp2lvds_disable,
#ifdef CONFIG_PM
	.suspend = ps8625_edp2lvds_suspend,
	.resume = ps8625_edp2lvds_resume,
#endif
};
