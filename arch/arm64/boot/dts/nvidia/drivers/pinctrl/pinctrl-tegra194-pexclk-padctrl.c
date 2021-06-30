/*
 * Pinctrl driver for Tegra194 PCIE PEXCLK Padcontrol.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/tegra_prod.h>

#include "core.h"
#include "pinctrl-tegra.h"
#include "pinctrl-utils.h"

#define PCIE_PEXCLK_PADCTL1_REFCLK_OVRD_0	0x8
#define CFG2TMC_SW_CTL				BIT(0)

enum t194_pexclk_pins {
	T194_PEXCLK_PIN_REFCLK,
};

enum t194_pexclk_pinconf_param {
	T194_PEXCLK_SINGLE_ENDED = PIN_CONFIG_END + 1,
};

static const struct pinconf_generic_params t194_pexclk_cfg_params[] = {
	{
		.property = "nvidia,pexclk-single-end",
		.param = T194_PEXCLK_SINGLE_ENDED,
	},
};

static const struct pinctrl_pin_desc t194_pexclk_pins_desc[] = {
	PINCTRL_PIN(T194_PEXCLK_PIN_REFCLK, "pexclk"),
};

struct t194_pexclk_pads {
	const char *name;
	const unsigned int pins[1];
	unsigned int npins;
};

static struct t194_pexclk_pads t194_pexclk_pads[] = {
	{
		.name = "pexclk",
		.pins = {
				T194_PEXCLK_PIN_REFCLK,
			},
		.npins = 1,
	},
};

struct t194_pexclk_padctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *regs[2];
	struct tegra_prod *prod_list;
	struct t194_pexclk_pads *pads;
	int num_pads;
	struct pinctrl_desc pinctrl_desc;
};

static int t194_pexclk_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct t194_pexclk_padctrl *pexclk = pinctrl_dev_get_drvdata(pctldev);

	return pexclk->num_pads;
}

static const char *t194_pexclk_get_group_name(struct pinctrl_dev *pctldev,
					      unsigned int group)
{
	struct t194_pexclk_padctrl *pexclk = pinctrl_dev_get_drvdata(pctldev);

	return pexclk->pads[group].name;
}

static int t194_pexclk_get_group_pins(struct pinctrl_dev *pctldev,
				      unsigned int group,
				      const unsigned int **pins,
				      unsigned int *num_pins)
{
	struct t194_pexclk_padctrl *pexclk = pinctrl_dev_get_drvdata(pctldev);

	*pins = pexclk->pads[group].pins;
	*num_pins = pexclk->pads[group].npins;

	return 0;
}

static const struct pinctrl_ops t194_pexclk_ops = {
	.get_groups_count = t194_pexclk_get_groups_count,
	.get_group_name	= t194_pexclk_get_group_name,
	.get_group_pins	= t194_pexclk_get_group_pins,
	.dt_node_to_map	= pinconf_generic_dt_node_to_map_pin,
	.dt_free_map	= pinconf_generic_dt_free_map,
};

static int t194_pexclk_pinconf_get(struct pinctrl_dev *pctldev,
				   unsigned int pin,
				   unsigned long *config)
{
	struct t194_pexclk_padctrl *pexclk = pinctrl_dev_get_drvdata(pctldev);
	u16 param = pinconf_to_config_param(*config);
	const struct t194_pexclk_pads *pad = &pexclk->pads[pin];
	u16 arg = 0;
	u32 rval;

	switch (param) {
	case T194_PEXCLK_SINGLE_ENDED:
		rval = readl(pexclk->regs[1] +
			     PCIE_PEXCLK_PADCTL1_REFCLK_OVRD_0);
		arg = !!(rval & CFG2TMC_SW_CTL);
		break;

	default:
		dev_dbg(pexclk->dev, "Pin %s does not support param %d\n",
			pad->name, param);
		return -EINVAL;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int t194_pexclk_pinconf_set(struct pinctrl_dev *pctldev,
				   unsigned int pin,
				   unsigned long *configs,
				   unsigned int num_configs)
{
	struct t194_pexclk_padctrl *pexclk = pinctrl_dev_get_drvdata(pctldev);
	const struct t194_pexclk_pads *pad = &pexclk->pads[pin];
	unsigned int i;
	u32 rval;

	for (i = 0; i < num_configs; i++) {
		u16 param_val = pinconf_to_config_argument(configs[i]);
		u16 param = pinconf_to_config_param(configs[i]);

		switch (param) {
		case T194_PEXCLK_SINGLE_ENDED:
			rval = readl(pexclk->regs[1] +
				     PCIE_PEXCLK_PADCTL1_REFCLK_OVRD_0);
			if (param_val)
				rval |= CFG2TMC_SW_CTL;
			else
				rval &= ~CFG2TMC_SW_CTL;
			writel(rval, pexclk->regs[1] +
			       PCIE_PEXCLK_PADCTL1_REFCLK_OVRD_0);
			break;

		default:
			dev_err(pexclk->dev, "Pin %s does not support param %d\n",
				pad->name, param);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct pinconf_ops t194_pexclk_pinconf_ops = {
	.pin_config_get = t194_pexclk_pinconf_get,
	.pin_config_set = t194_pexclk_pinconf_set,
	.is_generic = true,
};

static int t194_pexclk_padctrl_probe(struct platform_device *pdev)
{
	struct t194_pexclk_padctrl *pexclk;
	struct resource *res;
	int ret, i;

	pexclk = devm_kzalloc(&pdev->dev, sizeof(*pexclk), GFP_KERNEL);
	if (!pexclk)
		return -ENOMEM;

	pexclk->dev = &pdev->dev;

	for (i = 0; i < 2; ++i) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res) {
			dev_err(pexclk->dev, "Failed to get PEXCLK%d Address\n",
				i);
			return -ENOENT;
		}

		pexclk->regs[i] = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(pexclk->regs[i])) {
			ret = PTR_ERR(pexclk->regs[i]);
			dev_err(pexclk->dev, "Failed to IO map of PEXCLK%d: %d\n",
				i, ret);
			return ret;
		}
	}

	pexclk->prod_list = devm_tegra_prod_get(&pdev->dev);
	if (IS_ERR(pexclk->prod_list)) {
		dev_dbg(&pdev->dev, "Prod-settngs not available\n");
		pexclk->prod_list = NULL;
	} else {
		ret = tegra_prod_set_by_name(pexclk->regs, "prod",
					     pexclk->prod_list);
		if (ret < 0) {
			dev_err(&pdev->dev, "Prod config failed: %d\n", ret);
			return ret;
		}
	}

	pexclk->pads = t194_pexclk_pads;
	pexclk->num_pads = ARRAY_SIZE(t194_pexclk_pads);
	pexclk->pinctrl_desc.name = "pinctrl-pexclk-padctrl";
	pexclk->pinctrl_desc.pctlops = &t194_pexclk_ops;
	pexclk->pinctrl_desc.confops = &t194_pexclk_pinconf_ops;
	pexclk->pinctrl_desc.pins = t194_pexclk_pins_desc;
	pexclk->pinctrl_desc.npins = ARRAY_SIZE(t194_pexclk_pins_desc);
	pexclk->pinctrl_desc.custom_params = t194_pexclk_cfg_params;
	pexclk->pinctrl_desc.num_custom_params =
				ARRAY_SIZE(t194_pexclk_cfg_params);

	pexclk->pctl = devm_pinctrl_register(pexclk->dev,
					     &pexclk->pinctrl_desc,
					     pexclk);
	if (IS_ERR(pexclk->pctl)) {
		ret = PTR_ERR(pexclk->pctl);
		dev_err(pexclk->dev, "Failed to register pinctrl: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, pexclk);

	return 0;
}

static const struct of_device_id t194_pexclk_of_match[] = {
	{
		.compatible = "nvidia,tegra194-pexclk-padctl",
	}, {
	},
};
MODULE_DEVICE_TABLE(of, t194_pexclk_of_match);

static struct platform_driver tegra194_pexclk_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-tegra194-pexclk-padctl",
		.of_match_table = t194_pexclk_of_match,
	},
	.probe = t194_pexclk_padctrl_probe,
};

static int __init tegra194_pexclk_padctrl_init(void)
{
	return platform_driver_register(&tegra194_pexclk_pinctrl_driver);
}
postcore_initcall(tegra194_pexclk_padctrl_init);

MODULE_DESCRIPTION("NVIDIA TEGRA194 PCIE PEXCLK Padcontrol Driver");
MODULE_AUTHOR("Laxman Dewangan<ldewangan@nvidia.com>");
MODULE_LICENSE("GPL v2");
