/*
 * tegra-dfll-action.c - connect Tegra DFLL clock driver to thermal framework
 *
 * Copyright (C) 2012-2014 NVIDIA Corporation.  All rights reserved.
 *
 * Aleksandr Frid <afrid@nvidia.com>
 * Paul Walmsley <pwalmsley@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include <soc/tegra/tegra-dfll.h>

#define DFLL_CDEV_TYPE_FLOOR	"DFLL-floor"
#define DFLL_CDEV_TYPE_CAP	"DFLL-cap"
/**
 * struct tegra_dfll_cdev_data - DFLL cooling device info
 * @cdev: pointer to the cooling device (cap or floor)
 * @dfll: pointer to the DFLL instance associated with this cooling device
 */
struct tegra_dfll_cdev_data {
	struct thermal_cooling_device *cdev;
	struct tegra_dfll *dfll;
};

/*
 * Thermal cooling device interface
 */

static int
tegra_dfll_cdev_floor_get_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;
	int num;

	num = tegra_dfll_count_thermal_states(dfll_cdev_data->dfll,
					      TEGRA_DFLL_THERMAL_FLOOR);
	if (num <= 0)
		*max_state = 0;
	else
		*max_state = num - 1;

	return 0;
}

static int
tegra_dfll_cdev_floor_get_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;

	*cur_state = tegra_dfll_get_thermal_index(dfll_cdev_data->dfll,
						  TEGRA_DFLL_THERMAL_FLOOR);
	return 0;
}

static int
tegra_dfll_cdev_floor_set_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;

	return tegra_dfll_update_thermal_index(dfll_cdev_data->dfll,
					       TEGRA_DFLL_THERMAL_FLOOR,
					       cur_state);
}

static const struct thermal_cooling_device_ops tegra_dfll_floor_cooling_ops = {
	.get_max_state = tegra_dfll_cdev_floor_get_max_state,
	.get_cur_state = tegra_dfll_cdev_floor_get_cur_state,
	.set_cur_state = tegra_dfll_cdev_floor_set_state,
};

static int tegra_dfll_register_therm_floor(struct platform_device *pdev)
{
	struct thermal_cooling_device *tcd;
	struct device *dev = &pdev->dev;
	struct tegra_dfll_cdev_data *dfll_cdev_data = dev_get_drvdata(dev);

	tcd = thermal_of_cooling_device_register(dev->of_node,
						 DFLL_CDEV_TYPE_FLOOR,
						 dfll_cdev_data,
						 &tegra_dfll_floor_cooling_ops);
	if (IS_ERR(tcd)) {
		dev_err(dev, "failed to register cooling device\n");
		return PTR_ERR(tcd);
	}
	dfll_cdev_data->cdev = tcd;

	dev_info(dev, "Tegra DFLL 'floor cooling device' registered\n");

	return 0;
}

static int
tegra_dfll_cdev_cap_get_max_state(struct thermal_cooling_device *cdev,
				     unsigned long *max_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;
	int num;

	num = tegra_dfll_count_thermal_states(dfll_cdev_data->dfll,
					      TEGRA_DFLL_THERMAL_CAP);
	if (num <= 0)
		*max_state = 0;
	else
		*max_state = num - 1;

	return 0;
}

static int
tegra_dfll_cdev_cap_get_cur_state(struct thermal_cooling_device *cdev,
				     unsigned long *cur_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;

	*cur_state = tegra_dfll_get_thermal_index(dfll_cdev_data->dfll,
						  TEGRA_DFLL_THERMAL_CAP);
	return 0;
}

static int
tegra_dfll_cdev_cap_set_state(struct thermal_cooling_device *cdev,
				 unsigned long cur_state)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = cdev->devdata;

	return tegra_dfll_update_thermal_index(dfll_cdev_data->dfll,
					       TEGRA_DFLL_THERMAL_CAP,
					       cur_state);
}

static const struct thermal_cooling_device_ops tegra_dfll_cap_cooling_ops = {
	.get_max_state = tegra_dfll_cdev_cap_get_max_state,
	.get_cur_state = tegra_dfll_cdev_cap_get_cur_state,
	.set_cur_state = tegra_dfll_cdev_cap_set_state,
};

static int tegra_dfll_register_therm_cap(struct platform_device *pdev)
{
	struct thermal_cooling_device *tcd;
	struct device *dev = &pdev->dev;
	struct tegra_dfll_cdev_data *dfll_cdev_data = dev_get_drvdata(dev);

	tcd = thermal_of_cooling_device_register(dev->of_node,
						 DFLL_CDEV_TYPE_CAP,
						 dfll_cdev_data,
						 &tegra_dfll_cap_cooling_ops);
	if (IS_ERR(tcd)) {
		dev_err(dev, "failed to register cooling device\n");
		return PTR_ERR(tcd);
	}
	dfll_cdev_data->cdev = tcd;

	dev_info(dev, "Tegra DFLL 'cap cooling device' registered\n");

	return 0;
}

static int tegra_dfll_cdev_probe(struct platform_device *pdev)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data;
	struct tegra_dfll *dfll;
	struct device *dev = &pdev->dev;
	const char *cdev_type;
	int ret;

	dfll_cdev_data = devm_kzalloc(dev, sizeof(*dfll_cdev_data),
				      GFP_KERNEL);
	if (!dfll_cdev_data)
		return -ENOMEM;

	dfll = tegra_dfll_get_by_phandle(dev->of_node, "act-dev");
	if (IS_ERR(dfll)) {
		ret = PTR_ERR(dfll);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get DFLL device\n");
		return ret;
	}
	dfll_cdev_data->dfll = dfll;

	cdev_type = of_get_property(dev->of_node, "cdev-type", NULL);
	if (!cdev_type) {
		dev_err(dev, "missing cdev type\n");
		return -EINVAL;
	}

	dev_set_drvdata(dev, dfll_cdev_data);

	if (!strcmp(cdev_type, DFLL_CDEV_TYPE_FLOOR)) {
		ret = tegra_dfll_register_therm_floor(pdev);
	} else if (!strcmp(cdev_type, DFLL_CDEV_TYPE_CAP)) {
		ret = tegra_dfll_register_therm_cap(pdev);
	} else {
		dev_err(dev, "incorrect cdev type\n");
		return -EINVAL;
	}

	return ret;
}

static int tegra_dfll_cdev_remove(struct platform_device *pdev)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = dev_get_drvdata(&pdev->dev);

	thermal_cooling_device_unregister(dfll_cdev_data->cdev);

	return 0;
}

static const struct of_device_id tegra_dfll_cdev_match[] = {
	{ .compatible = "nvidia,tegra-dfll-cdev-action", },
	{},
};

#ifdef CONFIG_PM_SLEEP
static int tegra_dfll_cdev_resume(struct device *dev)
{
	struct tegra_dfll_cdev_data *dfll_cdev_data = dev_get_drvdata(dev);
	dfll_cdev_data->cdev->updated = false;

	return 0;
}

static const struct dev_pm_ops tegra_dfll_cdev_pm_ops = {
	.resume_noirq = tegra_dfll_cdev_resume,
};
#define DFLL_CDEV_PM (&tegra_dfll_cdev_pm_ops)
#else
#define DFLL_CDEV_PM (NULL)
#endif

static struct platform_driver tegra_dfll_cdev_driver = {
	.driver = {
		.name   = "tegra_dfll_action",
		.of_match_table = tegra_dfll_cdev_match,
		.pm = DFLL_CDEV_PM,
	},
	.probe = tegra_dfll_cdev_probe,
	.remove = tegra_dfll_cdev_remove,
};
module_platform_driver(tegra_dfll_cdev_driver);

MODULE_DESCRIPTION("Tegra DFLL thermal reaction driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksandr Frid <afrid@nvidia.com>");
MODULE_AUTHOR("Paul Walmsley <pwalmsley@nvidia.com>");
