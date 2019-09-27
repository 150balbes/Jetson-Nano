/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Suresh Mangipudi <smangipudi@nvidia.com>
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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/reset.h>
#include <linux/tegra-powergate.h>

#include <dt-bindings/soc/tegra186-powergate.h>
#include <dt-bindings/soc/tegra194-powergate.h>

#include "core.h"
#include "pinctrl-utils.h"

#define DPAUX_HYBRID_PADCTL		0x124
#define I2C_SDA_INPUT			BIT(15)
#define I2C_SCL_INPUT			BIT(14)
#define MODE				BIT(0)

#define DPAUX_HYBRID_SPARE		0x134
#define PAD_PWR				BIT(0)

struct tegra_dpaux_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
};

struct tegra_dpaux_pingroup {
	const char *name;
	const unsigned int pins[1];
	u8 npins;
	u8 funcs[2];
};

struct dpaux_context {
	u32 val_padctl;
	u32 val_spare;
};

struct tegra_dpaux_pinctl {
	struct device *dev;
	void __iomem *regs;
	struct platform_device *pdev;

	struct pinctrl_desc desc;
	struct pinctrl_dev *pinctrl;

	const struct pinctrl_pin_desc *pins;
	unsigned npins;
	const struct tegra_dpaux_function *functions;
	unsigned int nfunctions;
	const struct tegra_dpaux_pingroup *groups;
	unsigned ngroups;
	int powergate_id;
	struct clk *dpaux_clk;
	struct dpaux_context dpaux_context;
};

struct tegra_dpaux_chip_data {
	const struct pinctrl_pin_desc *pins;
	u32 npins;
	const struct tegra_dpaux_pingroup *pin_group;
	u32 npin_groups;
	struct tegra_dpaux_function *functions;
	u32 nfunctions;
	int powergate_id;
};

#define TEGRA_PIN_DPAUX_0 0
#define TEGRA_PIN_DPAUX1_1 1
#define TEGRA_PIN_DPAUX2_2 2
#define TEGRA_PIN_DPAUX3_3 3

static const struct pinctrl_pin_desc tegra186_dpaux_pins[] = {
	PINCTRL_PIN(TEGRA_PIN_DPAUX_0, "dpaux-0"),
	PINCTRL_PIN(TEGRA_PIN_DPAUX1_1, "dpaux1-1"),
};

static const struct pinctrl_pin_desc tegra194_dpaux_pins[] = {
	PINCTRL_PIN(TEGRA_PIN_DPAUX_0, "dpaux-0"),
	PINCTRL_PIN(TEGRA_PIN_DPAUX1_1, "dpaux1-1"),
	PINCTRL_PIN(TEGRA_PIN_DPAUX2_2, "dpaux2-2"),
	PINCTRL_PIN(TEGRA_PIN_DPAUX3_3, "dpaux3-3"),
};

enum tegra_dpaux_mux {
	TEGRA_DPAUX_MUX_I2C,
	TEGRA_DPAUX_MUX_DISPLAY,
};

#define TEGRA186_PIN_NAMES "dpaux-0", "dpaux1-1"

static const char * const tegra186_dpaux_pin_groups[] = {
	TEGRA186_PIN_NAMES
};

#define TEGRA194_PIN_NAMES "dpaux-0", "dpaux1-1", "dpaux2-2", "dpaux3-3"

static const char * const tegra194_dpaux_pin_groups[] = {
	TEGRA194_PIN_NAMES
};

#define FUNCTION(fname, group)			\
	{					\
		.name = #fname,			\
		.groups = group,		\
		.ngroups = ARRAY_SIZE(group),	\
	}					\

static struct tegra_dpaux_function tegra186_dpaux_functions[] = {
	FUNCTION(i2c, tegra186_dpaux_pin_groups),
	FUNCTION(display, tegra186_dpaux_pin_groups),
};

static struct tegra_dpaux_function tegra194_dpaux_functions[] = {
	FUNCTION(i2c, tegra194_dpaux_pin_groups),
	FUNCTION(display, tegra194_dpaux_pin_groups),
};

#define PINGROUP(pg_name, pin_id, f0, f1)		\
	{						\
		.name = #pg_name,			\
		.pins = {TEGRA_PIN_##pin_id},		\
		.npins = 1,				\
		.funcs = {				\
			TEGRA_DPAUX_MUX_##f0,		\
			TEGRA_DPAUX_MUX_##f1,		\
		},					\
	}

static const struct tegra_dpaux_pingroup tegra186_dpaux_groups[] = {
	PINGROUP(dpaux_0, DPAUX_0, I2C, DISPLAY),
	PINGROUP(dpaux1_1, DPAUX1_1, I2C, DISPLAY),
};

static const struct tegra_dpaux_pingroup tegra194_dpaux_groups[] = {
	PINGROUP(dpaux_0, DPAUX_0, I2C, DISPLAY),
	PINGROUP(dpaux1_1, DPAUX1_1, I2C, DISPLAY),
	PINGROUP(dpaux2_2, DPAUX2_2, I2C, DISPLAY),
	PINGROUP(dpaux3_3, DPAUX3_3, I2C, DISPLAY),
};

static struct tegra_dpaux_chip_data tegra186_dpaux_chip_data[] = {
	{
		.pins = tegra186_dpaux_pins,
		.npins = ARRAY_SIZE(tegra186_dpaux_pins),
		.pin_group = tegra186_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra186_dpaux_groups),
		.functions = tegra186_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra186_dpaux_functions),
		.powergate_id = TEGRA186_POWER_DOMAIN_DISP,
	},
	{
		.pins = tegra186_dpaux_pins,
		.npins = ARRAY_SIZE(tegra186_dpaux_pins),
		.pin_group = tegra186_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra186_dpaux_groups),
		.functions = tegra186_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra186_dpaux_functions),
		.powergate_id = TEGRA186_POWER_DOMAIN_DISP,
	},
};

static struct tegra_dpaux_chip_data tegra194_dpaux_chip_data[] = {
	{
		.pins = tegra194_dpaux_pins,
		.npins = ARRAY_SIZE(tegra194_dpaux_pins),
		.pin_group = tegra194_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra194_dpaux_groups),
		.functions = tegra194_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra194_dpaux_functions),
		.powergate_id = TEGRA194_POWER_DOMAIN_DISP,
	},
	{
		.pins = tegra194_dpaux_pins,
		.npins = ARRAY_SIZE(tegra194_dpaux_pins),
		.pin_group = tegra194_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra194_dpaux_groups),
		.functions = tegra194_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra194_dpaux_functions),
		.powergate_id = TEGRA194_POWER_DOMAIN_DISP,
	},
	{
		.pins = tegra194_dpaux_pins,
		.npins = ARRAY_SIZE(tegra194_dpaux_pins),
		.pin_group = tegra194_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra194_dpaux_groups),
		.functions = tegra194_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra194_dpaux_functions),
		.powergate_id = TEGRA194_POWER_DOMAIN_DISP,
	},
	{
		.pins = tegra194_dpaux_pins,
		.npins = ARRAY_SIZE(tegra194_dpaux_pins),
		.pin_group = tegra194_dpaux_groups,
		.npin_groups = ARRAY_SIZE(tegra194_dpaux_groups),
		.functions = tegra194_dpaux_functions,
		.nfunctions = ARRAY_SIZE(tegra194_dpaux_functions),
		.powergate_id = TEGRA194_POWER_DOMAIN_DISP,
	},
};

static void tegra_dpaux_update(struct tegra_dpaux_pinctl *tdp_aux,
				u32 reg_offset, u32 mask, u32 val)
{
	u32 rval;

	rval = __raw_readl(tdp_aux->regs + reg_offset);
	rval = (rval & ~mask) | (val & mask);
	__raw_writel(rval, tdp_aux->regs + reg_offset);
}

static int tegra_dpaux_pinctrl_set_mode(struct tegra_dpaux_pinctl *tdpaux_ctl,
					unsigned function)
{
	int ret = 0;
	u32 mask;

	ret =  clk_prepare_enable(tdpaux_ctl->dpaux_clk);
	if (ret < 0) {
		dev_err(tdpaux_ctl->dev, "clock enabled failed: %d\n", ret);
		return ret;
	}

	mask = I2C_SDA_INPUT | I2C_SCL_INPUT | MODE;

	if (function == TEGRA_DPAUX_MUX_DISPLAY)
		tegra_dpaux_update(tdpaux_ctl, DPAUX_HYBRID_PADCTL, mask, 0);
	else if (function == TEGRA_DPAUX_MUX_I2C)
		tegra_dpaux_update(tdpaux_ctl, DPAUX_HYBRID_PADCTL, mask, mask);
	tegra_dpaux_update(tdpaux_ctl, DPAUX_HYBRID_SPARE, 0x1, 0);

	clk_disable_unprepare(tdpaux_ctl->dpaux_clk);

	return ret;
}

static int tegra_dpaux_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);

	return padctl->npins;
}

static const char *tegra_dpaux_pinctrl_get_group_name(
	struct pinctrl_dev *pctldev, unsigned group)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);

	return padctl->pins[group].name;
}

static const struct pinctrl_ops tegra_dpaux_pinctrl_ops = {
	.get_groups_count = tegra_dpaux_pinctrl_get_groups_count,
	.get_group_name = tegra_dpaux_pinctrl_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int tegra186_dpaux_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);

	return padctl->nfunctions;
}

static const char *tegra186_dpaux_get_function_name(struct pinctrl_dev *pctldev,
						    unsigned int function)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);

	return padctl->functions[function].name;
}

static int tegra186_dpaux_get_function_groups(struct pinctrl_dev *pctldev,
					      unsigned int function,
					      const char * const **groups,
					      unsigned * const num_groups)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);

	*num_groups = padctl->functions[function].ngroups;
	*groups = padctl->functions[function].groups;

	return 0;
}

static int tegra_dpaux_pinctrl_set_mux(struct pinctrl_dev *pctldev,
				       unsigned function, unsigned group)
{
	struct tegra_dpaux_pinctl *padctl = pinctrl_dev_get_drvdata(pctldev);
	const struct tegra_dpaux_pingroup *g;
	int i;

	g = &padctl->groups[group];
	for (i = 0; i < ARRAY_SIZE(g->funcs); i++) {
		if (g->funcs[i] == function)
			break;
	}
	if (i == ARRAY_SIZE(g->funcs))
		return -EINVAL;

	return tegra_dpaux_pinctrl_set_mode(padctl, function);
}

static const struct pinmux_ops tegra_dpaux_pinmux_ops = {
	.get_functions_count = tegra186_dpaux_get_functions_count,
	.get_function_name = tegra186_dpaux_get_function_name,
	.get_function_groups = tegra186_dpaux_get_function_groups,
	.set_mux = tegra_dpaux_pinctrl_set_mux,
};

static int tegra186_dpaux_pinctrl_probe(struct platform_device *pdev)
{
	struct tegra_dpaux_chip_data *cdata;
	struct tegra_dpaux_pinctl *tdpaux_ctl;
	struct reset_control *rst;
	int ret;

	tdpaux_ctl = devm_kzalloc(&pdev->dev, sizeof(*tdpaux_ctl), GFP_KERNEL);
	if (!tdpaux_ctl)
		return -ENOMEM;

	tdpaux_ctl->dev = &pdev->dev;
	cdata = (struct tegra_dpaux_chip_data *)
				of_device_get_match_data(&pdev->dev);

	tdpaux_ctl->pdev = pdev;
	tdpaux_ctl->regs = devm_ioremap_resource(&pdev->dev,
			 platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (!tdpaux_ctl->regs) {
		dev_err(&pdev->dev, "Unable to map resource");
		return -EINVAL;
	}

	tdpaux_ctl->pins = cdata->pins;
	tdpaux_ctl->npins = cdata->npins;
	tdpaux_ctl->functions = cdata->functions;
	tdpaux_ctl->nfunctions = cdata->nfunctions;
	tdpaux_ctl->groups = cdata->pin_group;
	tdpaux_ctl->ngroups = cdata->npin_groups;

	memset(&tdpaux_ctl->desc, 0, sizeof(tdpaux_ctl->desc));
	tdpaux_ctl->desc.name = dev_name(&pdev->dev);
	tdpaux_ctl->desc.pins = tdpaux_ctl->pins;
	tdpaux_ctl->desc.npins = tdpaux_ctl->npins;
	tdpaux_ctl->desc.pctlops = &tegra_dpaux_pinctrl_ops;
	tdpaux_ctl->desc.pmxops = &tegra_dpaux_pinmux_ops;
	tdpaux_ctl->desc.owner = THIS_MODULE;
	tdpaux_ctl->powergate_id = cdata->powergate_id;
	platform_set_drvdata(pdev, tdpaux_ctl);

	ret = tegra_unpowergate_partition(tdpaux_ctl->powergate_id);
	if (ret < 0) {
		dev_err(tdpaux_ctl->dev, "unpowergate failed: %d\n", ret);
		return ret;
	}

	tdpaux_ctl->dpaux_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(tdpaux_ctl->dpaux_clk)) {
		dev_err(&pdev->dev, "can not get clock\n");
		return PTR_ERR(tdpaux_ctl->dpaux_clk);
	}

	rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(rst)) {
		dev_err(&pdev->dev, "can not get reset\n");
		return PTR_ERR(rst);
	}

	reset_control_deassert(rst);
	tdpaux_ctl->pinctrl = devm_pinctrl_register(&pdev->dev,
				 &tdpaux_ctl->desc, tdpaux_ctl);
	if (IS_ERR(tdpaux_ctl->pinctrl)) {
		ret = PTR_ERR(tdpaux_ctl->pinctrl);
		dev_err(&pdev->dev, "Failed to register dpaux pinctrl: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static int tegra_dpaux_remove(struct platform_device *pdev)
{
	struct tegra_dpaux_pinctl *tdpaux_ctl = platform_get_drvdata(pdev);

	tegra_powergate_partition(tdpaux_ctl->powergate_id);

	return 0;
}

static void tegra186_dpaux_save(struct tegra_dpaux_pinctl *dpaux_ctl)
{
	dpaux_ctl->dpaux_context.val_padctl =
		__raw_readl(dpaux_ctl->regs + DPAUX_HYBRID_PADCTL);

	dpaux_ctl->dpaux_context.val_spare =
		__raw_readl(dpaux_ctl->regs + DPAUX_HYBRID_SPARE);
}

static void tegra186_dpaux_restore(struct tegra_dpaux_pinctl *dpaux_ctl)
{
	__raw_writel(dpaux_ctl->dpaux_context.val_padctl,
			dpaux_ctl->regs + DPAUX_HYBRID_PADCTL);
	__raw_writel(dpaux_ctl->dpaux_context.val_spare,
			dpaux_ctl->regs + DPAUX_HYBRID_SPARE);
}

static int tegra186_dpaux_suspend(struct device *dev)
{
	int ret;
	struct tegra_dpaux_pinctl *dpaux_ctl = dev_get_drvdata(dev);

	ret = clk_prepare_enable(dpaux_ctl->dpaux_clk);
	if (ret < 0) {
		dev_err(dpaux_ctl->dev, "clock enable failed: %d\n", ret);
		return ret;
	}

	tegra186_dpaux_save(dpaux_ctl);

	clk_disable_unprepare(dpaux_ctl->dpaux_clk);

	return 0;
}

static int tegra186_dpaux_resume(struct device *dev)
{
	int ret;
	struct tegra_dpaux_pinctl *dpaux_ctl = dev_get_drvdata(dev);

	ret =  clk_prepare_enable(dpaux_ctl->dpaux_clk);
	if (ret < 0) {
		dev_err(dpaux_ctl->dev, "clock enabled failed: %d\n", ret);
		return ret;
	}

	tegra186_dpaux_restore(dpaux_ctl);

	clk_disable_unprepare(dpaux_ctl->dpaux_clk);

	return 0;
}

static const struct dev_pm_ops tegra186_dpaux_pm_ops = {
	.suspend = tegra186_dpaux_suspend,
	.resume = tegra186_dpaux_resume,
};

static struct of_device_id tegra_dpaux_pinctl_of_match[] = {
	{.compatible = "nvidia,tegra186-dpaux-padctl",
		.data = &tegra186_dpaux_chip_data[0]},
	{.compatible = "nvidia,tegra186-dpaux1-padctl",
		.data = &tegra186_dpaux_chip_data[1]},
	{.compatible = "nvidia,tegra194-dpaux-padctl",
		.data = &tegra194_dpaux_chip_data[0]},
	{.compatible = "nvidia,tegra194-dpaux1-padctl",
		.data = &tegra194_dpaux_chip_data[1]},
	{.compatible = "nvidia,tegra194-dpaux2-padctl",
		.data = &tegra194_dpaux_chip_data[2]},
	{.compatible = "nvidia,tegra194-dpaux3-padctl",
		.data = &tegra194_dpaux_chip_data[3]},
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_dpaux_pinctl_of_match);

static struct platform_driver tegra186_dpaux_pinctrl = {
	.driver = {
		.name = "tegra186-dpaux-pinctrl",
		.of_match_table = tegra_dpaux_pinctl_of_match,
		.pm = &tegra186_dpaux_pm_ops,
	},
	.probe = tegra186_dpaux_pinctrl_probe,
	.remove = tegra_dpaux_remove,
};


module_platform_driver(tegra186_dpaux_pinctrl);

MODULE_DESCRIPTION("NVIDIA Tegra dpaux pinctrl driver");
MODULE_AUTHOR("Suresh Mangipudi <smangipudi@nvidia.com>");
MODULE_ALIAS("platform:tegra186-dpaux");
MODULE_LICENSE("GPL v2");
