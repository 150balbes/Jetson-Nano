/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static int update_clocks(struct device_node *np, bool enable)
{
	struct clk *clk;
	int err = 0, index = 0;

	do {
		clk = of_clk_get(np, index);
		if (IS_ERR(clk)) {
			err = PTR_ERR(clk);
			if (err == -ENOENT)
				err = 0;
			break;
		}
		err = clk_prepare(clk);
		if (err)
			break;
		if (!enable)
			clk_unprepare(clk);
		index++;
	} while (!IS_ERR(clk));

	return err;
}

/**
 * Look for enable/disable nodes and enable/disable the clocks respectively.
 */
static int tegra_clocks_config_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cnp;

	cnp = of_get_child_by_name(np, "enable");
	if (cnp) {
		err = update_clocks(cnp, true);
		if (err)
			goto err_ret;
	}

	cnp = of_get_child_by_name(np, "disable");
	if (cnp)
		err = update_clocks(cnp, false);

err_ret:
	if (err) {
		pr_err("tegra_clocks_config: initialization failed: error %d\n",
			err);
	} else {
		pr_info("Clocks initialized successfully\n");
	}

	return err;
}

static const struct of_device_id tegra_clocks_config_of_match[] = {
	{ .compatible = "nvidia,clocks-config", NULL },
	{ },
};

static struct platform_driver tegra_clocks_config_driver = {
	.driver	= {
		.name   = "tegra-clocks-config",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_clocks_config_of_match,
	},
	.probe	= tegra_clocks_config_probe,
};

static int __init tegra_clocks_config_init(void)
{
	return platform_driver_register(&tegra_clocks_config_driver);
}

static void __exit tegra_clocks_config_exit(void)
{
	platform_driver_unregister(&tegra_clocks_config_driver);
}

module_init(tegra_clocks_config_init);
module_exit(tegra_clocks_config_exit);
MODULE_AUTHOR("Vishruth Jain <vishruthj@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra194 driver to configure clock states");
MODULE_LICENSE("GPL v2");
