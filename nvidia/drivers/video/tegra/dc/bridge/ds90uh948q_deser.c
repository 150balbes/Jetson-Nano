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
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/nvhost.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#include "ds90uh948q_deser.h"

static struct regmap_config ds90uh948_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8.
};

static int ds90uh948_deser_config(struct i2c_client *client)
{
	int err = 0;
	struct ds90uh948_data *deser_data = i2c_get_clientdata(client);

	/* set backchannel i2c setting */
	err = regmap_update_bits(deser_data->regmap, DS90UH948_DESER_REG_GENCFG,
		BIT(DS90UH948_DESER_REG_GENCFG_I2C_PASSTHRU),
		deser_data->en_bchnl_i2c <<
		DS90UH948_DESER_REG_GENCFG_I2C_PASSTHRU);

	if (err < 0)
		goto err;

	/* set 2 clock filter setting */
	err = regmap_update_bits(deser_data->regmap, DS90UH948_DESER_REG_GENCFG,
		BIT(DS90UH948_DESER_REG_GENCFG_FILTER_EN),
		deser_data->en_2clk_filter <<
		DS90UH948_DESER_REG_GENCFG_FILTER_EN);

	if (err < 0)
		goto err;

	/* GPIO1 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO1_2,
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO1_EN) |
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO1_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO1_VAL),
		deser_data->en_gpio1 <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO1_EN |
		deser_data->gpio1_direction <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO1_DIR |
		deser_data->gpio1_value <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO1_VAL);

	if (err < 0)
		goto err;

	/* GPIO2 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO1_2,
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO2_EN) |
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO2_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO1_2_GPIO2_VAL),
		deser_data->en_gpio2 <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO2_EN |
		deser_data->gpio2_direction <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO2_DIR |
		deser_data->gpio2_value <<
		DS90UH948_DESER_REG_GPIO1_2_GPIO2_VAL);

	if (err < 0)
		goto err;

	/* GPIO3 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO3,
		BIT(DS90UH948_DESER_REG_GPIO3_GPIO3_EN) |
		BIT(DS90UH948_DESER_REG_GPIO3_GPIO3_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO3_GPIO3_VAL),
		deser_data->en_gpio3 <<
		DS90UH948_DESER_REG_GPIO3_GPIO3_EN |
		deser_data->gpio3_direction <<
		DS90UH948_DESER_REG_GPIO3_GPIO3_DIR |
		deser_data->gpio3_value <<
		DS90UH948_DESER_REG_GPIO3_GPIO3_VAL);

	if (err < 0)
		goto err;

	/* GPIO5 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO5_6,
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO5_EN) |
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO5_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO5_VAL),
		deser_data->en_gpio5 <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO5_EN |
		deser_data->gpio5_direction <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO5_DIR |
		deser_data->gpio5_value <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO5_VAL);

	if (err < 0)
		goto err;

	/* GPIO6 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO5_6,
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO6_EN) |
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO6_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO5_6_GPIO6_VAL),
		deser_data->en_gpio6 <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO6_EN |
		deser_data->gpio6_direction <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO6_DIR |
		deser_data->gpio6_value <<
		DS90UH948_DESER_REG_GPIO5_6_GPIO6_VAL);

	if (err < 0)
		goto err;

	/* GPIO7 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO7_8,
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO7_EN) |
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO7_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO7_VAL),
		deser_data->en_gpio7 <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO7_EN |
		deser_data->gpio7_direction <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO7_DIR |
		deser_data->gpio7_value <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO7_VAL);

	if (err < 0)
		goto err;

	/* GPIO8 configuration */
	err = regmap_update_bits(deser_data->regmap,
		DS90UH948_DESER_REG_GPIO7_8,
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO8_EN) |
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO8_DIR) |
		BIT(DS90UH948_DESER_REG_GPIO7_8_GPIO8_VAL),
		deser_data->en_gpio8 <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO8_EN |
		deser_data->gpio8_direction <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO8_DIR |
		deser_data->gpio8_value <<
		DS90UH948_DESER_REG_GPIO7_8_GPIO8_VAL);

	if (err < 0)
		goto err;

err:
	return err;
}

static int of_ds90uh948_parse_platform_data(struct i2c_client *client)
{
	int err = 0;
	struct device_node *np = client->dev.of_node;
	struct ds90uh948_data *deser_data = i2c_get_clientdata(client);
	u32 temp;

	if (!of_property_read_u32(np, "ti,enable-bchnl-i2c", &temp))
		deser_data->en_bchnl_i2c = temp;

	if (!of_property_read_u32(np, "ti,enable-2clock-filter", &temp))
		deser_data->en_2clk_filter = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio1", &temp))
		deser_data->en_gpio1 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio2", &temp))
		deser_data->en_gpio2 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio3", &temp))
		deser_data->en_gpio3 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio5", &temp))
		deser_data->en_gpio5 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio6", &temp))
		deser_data->en_gpio6 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio7", &temp))
		deser_data->en_gpio7 = temp;

	if (!of_property_read_u32(np, "ti,enable-gpio8", &temp))
		deser_data->en_gpio8 = temp;

	if (!of_property_read_u32(np, "ti,gpio1-dir", &temp))
		deser_data->gpio1_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio2-dir", &temp))
		deser_data->gpio2_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio3-dir", &temp))
		deser_data->gpio3_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio5-dir", &temp))
		deser_data->gpio5_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio6-dir", &temp))
		deser_data->gpio6_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio7-dir", &temp))
		deser_data->gpio7_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio8-dir", &temp))
		deser_data->gpio8_direction = temp;

	if (!of_property_read_u32(np, "ti,gpio1-val", &temp))
		deser_data->gpio1_value = temp;

	if (!of_property_read_u32(np, "ti,gpio2-val", &temp))
		deser_data->gpio2_value = temp;

	if (!of_property_read_u32(np, "ti,gpio3-val", &temp))
		deser_data->gpio3_value = temp;

	if (!of_property_read_u32(np, "ti,gpio5-val", &temp))
		deser_data->gpio5_value = temp;

	if (!of_property_read_u32(np, "ti,gpio6-val", &temp))
		deser_data->gpio6_value = temp;

	if (!of_property_read_u32(np, "ti,gpio7-val", &temp))
		deser_data->gpio7_value = temp;

	if (!of_property_read_u32(np, "ti,gpio8-val", &temp))
		deser_data->gpio8_value = temp;

	return err;
}

static int ds90uh948_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int err;
	struct ds90uh948_data *deser_data;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	deser_data = devm_kzalloc(&client->dev,
			sizeof(*deser_data), GFP_KERNEL);
	if (!deser_data)
		return -ENOMEM;

	deser_data->regmap = devm_regmap_init_i2c(client,
				&ds90uh948_regmap_config);

	if (IS_ERR(deser_data->regmap)) {
		err = PTR_ERR(deser_data->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", err);
		return err;
	}

	i2c_set_clientdata(client, deser_data);

	err = of_ds90uh948_parse_platform_data(client);
	if (err < 0)
		return err;

	err = ds90uh948_deser_config(client);
	if (err < 0)
		return err;

	dev_info(&client->dev, "probe complete");

	return 0;
}

static int ds90uh948_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ds90uh948_id[] = {
	{ "ds90uh948", 0 },
	{ "ds90uh926", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds90uh948_id);

#ifdef CONFIG_OF
static const struct of_device_id ds90uh948_of_match[] = {
	{.compatible = "ti,ds90uh948", },
	{.compatible = "ti,ds90uh926", },
	{ },
};
MODULE_DEVICE_TABLE(of, ds90uh948_of_match);
#endif

static struct i2c_driver ds90uh948_driver = {
	.driver = {
		.name = "ds90uh948",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(ds90uh948_of_match),
#endif
	},
	.probe	= ds90uh948_probe,
	.remove   = ds90uh948_remove,
	.id_table = ds90uh948_id,
};

module_i2c_driver(ds90uh948_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ds90uh948 FPDLink Deserializer driver");
MODULE_ALIAS("i2c:ds90uh948_ser");

