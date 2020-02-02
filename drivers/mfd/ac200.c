// SPDX-License-Identifier: GPL-2.0-only
/*
 * MFD core driver for X-Powers' AC200 IC
 *
 * The AC200 is a chip which is co-packaged with Allwinner H6 SoC and
 * includes analog audio codec, analog TV encoder, ethernet PHY, eFuse
 * and RTC.
 *
 * Copyright (c) 2019 Jernej Skrabec <jernej.skrabec@siol.net>
 *
 * Based on AC100 driver with following copyrights:
 * Copyright (2016) Chen-Yu Tsai
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ac200.h>
#include <linux/module.h>
#include <linux/of.h>

static const struct regmap_range_cfg ac200_range_cfg[] = {
	{
		.range_min = AC200_SYS_VERSION,
		.range_max = AC200_IC_CHARA1,
		.selector_reg = AC200_TWI_REG_ADDR_H,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 256,
	}
};

static const struct regmap_config ac200_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 16,
	.ranges		= ac200_range_cfg,
	.num_ranges	= ARRAY_SIZE(ac200_range_cfg),
	.max_register	= AC200_IC_CHARA1,
};

static struct mfd_cell ac200_cells[] = {
	{
		.name		= "ac200-codec",
		.of_compatible	= "x-powers,ac200-codec",
	}, {
		.name		= "ac200-efuse",
		.of_compatible	= "x-powers,ac200-efuse",
	}, {
		.name		= "ac200-ephy",
		.of_compatible	= "x-powers,ac200-ephy",
	}, {
		.name		= "ac200-rtc",
		.of_compatible	= "x-powers,ac200-rtc",
	}, {
		.name		= "ac200-tve",
		.of_compatible	= "x-powers,ac200-tve",
	},
};

static int ac200_i2c_probe(struct i2c_client *i2c,
			   const struct i2c_device_id *id)
{
	struct device *dev = &i2c->dev;
	struct ac200_dev *ac200;
	int ret;

	ac200 = devm_kzalloc(dev, sizeof(*ac200), GFP_KERNEL);
	if (!ac200)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ac200);

	ac200->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ac200->clk)) {
		ret = PTR_ERR(ac200->clk);
		dev_err(dev, "Can't obtain the clock: %d\n", ret);
		return ret;
	}

	ac200->regmap = devm_regmap_init_i2c(i2c, &ac200_regmap_config);
	if (IS_ERR(ac200->regmap)) {
		ret = PTR_ERR(ac200->regmap);
		dev_err(dev, "Regmap init failed: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ac200->clk);
	if (ret)
		return ret;

	ret = regmap_write(ac200->regmap, AC200_SYS_CONTROL, 0);
	if (ret)
		goto err;

	ret = regmap_write(ac200->regmap, AC200_SYS_CONTROL, 1);
	if (ret)
		goto err;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, ac200_cells,
				   ARRAY_SIZE(ac200_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to add MFD devices: %d\n", ret);
		goto err;
	}

	return 0;

err:
	clk_disable_unprepare(ac200->clk);
	return ret;
}

static int ac200_i2c_remove(struct i2c_client *i2c)
{
	struct ac200_dev *ac200 = i2c_get_clientdata(i2c);

	regmap_write(ac200->regmap, AC200_SYS_CONTROL, 0);

	clk_disable_unprepare(ac200->clk);

	return 0;
}

static const struct i2c_device_id ac200_ids[] = {
	{ "ac200", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ac200_ids);

static const struct of_device_id ac200_of_match[] = {
	{ .compatible = "x-powers,ac200" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ac200_of_match);

static struct i2c_driver ac200_i2c_driver = {
	.driver = {
		.name	= "ac200",
		.of_match_table	= of_match_ptr(ac200_of_match),
	},
	.probe	= ac200_i2c_probe,
	.remove = ac200_i2c_remove,
	.id_table = ac200_ids,
};
module_i2c_driver(ac200_i2c_driver);

MODULE_DESCRIPTION("MFD core driver for AC200");
MODULE_AUTHOR("Jernej Skrabec <jernej.skrabec@siol.net>");
MODULE_LICENSE("GPL v2");
