/*
 * drivers/i2c/busses/i2c-rtcpu.c
 *
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
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

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "i2c-rtcpu-clk-config.h"
#include "i2c-ivc-single.h"
#include "i2c-rtcpu-common.h"

/*
 * I2C bus driver internal data structure
 */
struct tegra_i2c_rtcpu {
	struct device *dev;
	struct i2c_adapter adapter;
	struct tegra_i2c_clk_config i2c_clk_config;
	u32 i2c_base;
	bool is_suspended;
};

/*
 * I2C bus driver interface
 */
static int tegra_i2c_rtcpu_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_rtcpu *i2c_dev = i2c_get_adapdata(adap);
	int ret;

	if (i2c_dev->is_suspended)
		return -EBUSY;

	pm_runtime_get_sync(&adap->dev);
	ret = tegra_i2c_ivc_single_xfer(i2c_dev->i2c_base, msgs, num);
	pm_runtime_put(&adap->dev);

	return ret;
}

static u32 tegra_i2c_rtcpu_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_NOSTART |
		I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm tegra_i2c_rtcpu_algo = {
	.master_xfer = tegra_i2c_rtcpu_xfer,
	.functionality = tegra_i2c_rtcpu_func,
};

/*
 * Probe and initialization
 */
static int tegra_i2c_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_i2c_rtcpu *i2c_dev;
	struct resource *res;
	struct clk *div_clk;
	struct clk *parent_clk;
	struct clk *slow_clk;
	u32 i2c_base;
	void __iomem *base;
	int ret = 0;
	struct tegra_i2c_clk_config *i2c_clk_config;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	i2c_base = tegra_i2c_get_reg_base(pdev->dev.of_node);
	if (i2c_base == 0)
		return -ENODEV;

	if (tegra_i2c_ivc_single_xfer(i2c_base, NULL, 0) != 0)
		return -EPROBE_DEFER;

	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	div_clk = devm_clk_get(&pdev->dev, "div-clk");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock");
		return PTR_ERR(div_clk);
	}

	parent_clk = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(parent_clk)) {
		dev_err(&pdev->dev, "Unable to get parent_clk err:%ld\n",
				PTR_ERR(parent_clk));
	} else {
		ret = clk_set_parent(div_clk, parent_clk);
		if (ret < 0)
			dev_warn(&pdev->dev, "Couldn't set parent clock : %d\n",
				ret);
	}

	slow_clk = devm_clk_get(&pdev->dev, "slow-clk");
	if (IS_ERR(slow_clk)) {
		dev_err(&pdev->dev, "missing slow clock\n");
		slow_clk = NULL;
	}

	/* Create a private data */
	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2c_dev);

	i2c_dev->dev = &pdev->dev;
	i2c_dev->i2c_base = i2c_base;
	i2c_dev->is_suspended = false;

	i2c_clk_config = &i2c_dev->i2c_clk_config;
	i2c_clk_config->base = base;
	i2c_clk_config->div_clk = div_clk;
	i2c_clk_config->slow_clk = slow_clk;
	i2c_clk_config->rst = devm_reset_control_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c_clk_config->rst)) {
		dev_err(i2c_dev->dev, "missing controller reset");
		return PTR_ERR(i2c_clk_config->rst);
	}

	i2c_clk_config->bus_clk_rate = tegra_i2c_get_clk_freq(
			i2c_dev->dev->of_node);

	i2c_clk_config->is_clkon_always =
		of_property_read_bool(pdev->dev.of_node,
				"nvidia,clock-always-on");

	i2c_clk_config->clk_divisor_non_hs_mode = 0x19;
	if (i2c_clk_config->bus_clk_rate == I2C_FAST_MODE_PLUS)
		i2c_clk_config->clk_divisor_non_hs_mode = 0x10;

	ret = clk_prepare(i2c_clk_config->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
		goto fail;
	}
	if (i2c_clk_config->slow_clk) {
		ret = clk_prepare(i2c_clk_config->slow_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "slow clk prep failed %d\n", ret);
			goto unprepare_div_clk;
		}
	}

	ret = tegra_i2c_rtcpu_clock_enable(i2c_clk_config);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "div_clk enable failed %d\n",
			ret);
		goto unprepare_slow_clk;
	}

	ret = tegra_i2c_rtcpu_clock_init(i2c_clk_config);
	if (ret) {
		dev_err(i2c_dev->dev, "Failed to initialize i2c controller");
		goto disable_clk;
	}

	/* Probe the I2C hardware */
	pm_runtime_enable(&pdev->dev);

	/* Register I2C bus */
	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	strlcpy(i2c_dev->adapter.name, "Tegra CAMRTC I2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.algo = &tegra_i2c_rtcpu_algo;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(i2c_dev->dev, "Cannot add I2C adapter: %d\n", ret);
		goto pm_disable;
	}

	return 0;

pm_disable:
	pm_runtime_disable(&pdev->dev);

disable_clk:
	tegra_i2c_rtcpu_clock_disable(i2c_clk_config);

unprepare_slow_clk:
	if (i2c_clk_config->slow_clk)
		clk_unprepare(i2c_clk_config->slow_clk);

unprepare_div_clk:
	clk_unprepare(i2c_clk_config->div_clk);

fail:
	return ret;
}

static int tegra_i2c_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_i2c_rtcpu *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static void tegra_i2c_rtcpu_shutdown(struct platform_device *pdev)
{
	struct tegra_i2c_rtcpu *i2c_dev = platform_get_drvdata(pdev);

	dev_info(i2c_dev->dev, "Bus is shutdown down..\n");
	i2c_shutdown_adapter(&i2c_dev->adapter);
}

/*
 * I2C bus driver PM interface
 */

#ifdef CONFIG_PM_SLEEP
static int tegra_i2c_rtcpu_suspend(struct device *dev)
{
	struct tegra_i2c_rtcpu *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = true;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static int tegra_i2c_rtcpu_resume(struct device *dev)
{
	struct tegra_i2c_rtcpu *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = false;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tegra_i2c_rtcpu_pm,
			tegra_i2c_rtcpu_suspend,
			tegra_i2c_rtcpu_resume);
#define TEGRA_I2C_PM	(&tegra_i2c_rtcpu_pm)
#else
#define TEGRA_I2C_PM	NULL
#endif

/*
 * I2C bus driver interface
 */

static const struct of_device_id tegra_i2c_rtcpu_of_match[] = {
	{ .compatible = "nvidia,tegra-i2c-rtcpu", .data = NULL, },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_i2c_rtcpu_of_match);

static struct platform_driver tegra_i2c_rtcpu_driver = {
	.probe   = tegra_i2c_rtcpu_probe,
	.remove  = tegra_i2c_rtcpu_remove,
	.late_shutdown = tegra_i2c_rtcpu_shutdown,
	.driver  = {
		.name  = "tegra-i2c-rtcpu",
		.of_match_table = tegra_i2c_rtcpu_of_match,
		.pm    = TEGRA_I2C_PM,
	},
};

static int __init tegra_i2c_rtcpu_init_driver(void)
{
	return platform_driver_register(&tegra_i2c_rtcpu_driver);
}

subsys_initcall(tegra_i2c_rtcpu_init_driver);
