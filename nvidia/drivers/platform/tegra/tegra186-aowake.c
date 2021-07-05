/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <iomap.h>
#include "tegra186-aowake.h"

#define WAKE_AOWAKE_CNTRL_0	0x0
#define WAKE_AOWAKE_CTRL_0	0x4F4
#define WAKE_AOWAKE_CNTRL_24	0x60
#define WAKE24_WAKE_LEVEL_MASK	BIT(3)

struct tegra_aowake_cntrl_filters {
	u32 wake;
	u32 filter_mask;
	u32 filter_val;
};

struct tegra_aowake_chip_data {
	u32 num_wakes_filter;
	const struct tegra_aowake_cntrl_filters *cntrl_filters;
};

struct tegra_aowake_info {
	struct device *dev;
	void __iomem *aobase;
	bool invert_pmic_interrupt;
	const struct tegra_aowake_chip_data *cdata;
};

static struct tegra_aowake_info *tegra186_aowake;

unsigned long tegra_aowake_read(unsigned int reg_offset)
{
	if (!tegra186_aowake) {
		WARN_ON(!tegra186_aowake);
		return 0;
	}

	return readl(tegra186_aowake->aobase + reg_offset);
}
EXPORT_SYMBOL(tegra_aowake_read);

int tegra_aowake_write(unsigned long val, unsigned int reg_offset)
{
	if (!tegra186_aowake) {
		WARN_ON(!tegra186_aowake);
		return -EINVAL;
	}

	writel(val, tegra186_aowake->aobase + reg_offset);
	return 0;
}
EXPORT_SYMBOL(tegra_aowake_write);

int tegra_aowake_update(unsigned int reg_offset,
		unsigned long mask, unsigned long val)
{
	unsigned long rval;

	if (!tegra186_aowake) {
		WARN_ON(!tegra186_aowake);
		return -EINVAL;
	}

	rval = readl(tegra186_aowake->aobase + reg_offset);
	rval = (rval & ~mask) | (val & mask);
	writel(rval, tegra186_aowake->aobase + reg_offset);
	return 0;
}
EXPORT_SYMBOL(tegra_aowake_update);

static void aowake_configure_pmic_polarity(struct device *dev,
		struct tegra_aowake_info *taowake)
{
	struct device_node *np = dev->of_node;
	unsigned long reg;

	taowake->invert_pmic_interrupt = of_property_read_bool(np,
						"nvidia,invert-interrupt");
	if (!taowake->invert_pmic_interrupt)
		taowake->invert_pmic_interrupt = of_property_read_bool(np,
				"nvidia,invert-pmic-interrupt-polarity");

	reg = readl(taowake->aobase + WAKE_AOWAKE_CTRL_0);
	if (taowake->invert_pmic_interrupt)
		reg |= 0x1;
	else
		reg &= ~0x1;
	writel(reg, taowake->aobase + WAKE_AOWAKE_CTRL_0);
	dev_info(dev, "WAKE_AOWAKE_CTRL_0 = %lu\n", reg);

	reg = readl(taowake->aobase + WAKE_AOWAKE_CNTRL_24);
	if (taowake->invert_pmic_interrupt)
		reg &= ~(WAKE24_WAKE_LEVEL_MASK);
	else
		reg |= WAKE24_WAKE_LEVEL_MASK;
	writel(reg, taowake->aobase + WAKE_AOWAKE_CNTRL_24);
	dev_info(dev, "WAKE_AOWAKE_CNTRL_24(PMU_INT) = %lu\n", reg);
}

static void aowake_apply_cdata(struct device *dev,
		struct tegra_aowake_info *taowake)
{
	const struct tegra_aowake_cntrl_filters *cntrl_filters;
	u32 reg;
	int i;

	if (!taowake->cdata)
		return;

	cntrl_filters = taowake->cdata->cntrl_filters;
	for (i = 0; i < taowake->cdata->num_wakes_filter; i++) {
		reg = WAKE_AOWAKE_CNTRL_0 + (cntrl_filters[i].wake * 4);
		tegra_aowake_update(reg,
				cntrl_filters[i].filter_mask,
				cntrl_filters[i].filter_val);
		dev_info(dev, "WAKE_AOWAKE_CNTRL_%u = 0x%lx\n",
			cntrl_filters[i].wake, tegra_aowake_read(reg));
	}
}

static int tegra_aowake_probe(struct platform_device *pdev)
{
	struct device *dev  = &pdev->dev;
	struct resource *r;
	void __iomem *aobase;
	struct tegra_aowake_info *taowake;
	int ret;

	taowake = devm_kzalloc(dev, sizeof(*taowake), GFP_KERNEL);
	if (!taowake)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(dev, "No IO memory resource\n");
		return -ENODEV;
	}

	aobase = devm_ioremap_resource(dev, r);
	if (IS_ERR(aobase)) {
		ret = PTR_ERR(aobase);
		dev_err(dev, "Cannot iomap aowake register: %d\n", ret);
		return ret;
	}

	taowake->dev = dev;
	taowake->aobase = aobase;
	taowake->cdata = of_device_get_match_data(&pdev->dev);
	tegra186_aowake = taowake;

	aowake_apply_cdata(dev, taowake);
	aowake_configure_pmic_polarity(dev, taowake);

	return 0;
}

static const struct tegra_aowake_cntrl_filters tegra186_cntrl_filters[1] = {
	/* SW Wake (wake83) needs SR_CAPTURE filter to be enabled*/
	{ .wake = 83,
	  .filter_mask = 0x7,
	  .filter_val = 0x2,
	},
};

static const struct tegra_aowake_chip_data tegra186_cdata = {
	.num_wakes_filter = ARRAY_SIZE(tegra186_cntrl_filters),
	.cntrl_filters = tegra186_cntrl_filters,
};

static struct of_device_id tegra_aowake_of_match[] = {
	{ .compatible = "nvidia,tegra194-aowake", .data = &tegra186_cdata },
	{ .compatible = "nvidia,tegra186-aowake", .data = &tegra186_cdata },
	{ },
};

static struct platform_driver tegra_aowake_driver = {
	.driver	= {
		.name   = "tegra186-aowake",
		.owner  = THIS_MODULE,
		.of_match_table = tegra_aowake_of_match,
	},
	.probe	= tegra_aowake_probe,
};

static int __init tegra_aowake_init(void)
{
	pm_irq_init();

	return platform_driver_register(&tegra_aowake_driver);
}
postcore_initcall(tegra_aowake_init);
