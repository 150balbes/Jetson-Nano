/*
 * Copyright (c) 2016 - 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Navneet Kumar <navneetk@nvidia.com>
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>

struct tj_tzs {
	struct thermal_zone_device *tzd;
	int offset;
};

struct tj {
	struct device *dev;
	struct thermal_zone_device *self;
	int num_tzs;
	struct tj_tzs *tzs;
};

static int tegra_tj_thermal_get_temp(void *data, int *out_temp)
{
	int i;
	struct tj *ptj = (struct tj *)data;

	*out_temp = THERMAL_TEMP_INVALID;

	for (i = 0; i < ptj->num_tzs; i++) {
		int temp, ret;

		if (ptj->tzs[i].tzd) {
			ret = thermal_zone_get_temp(ptj->tzs[i].tzd, &temp);
			if (ret < 0) {
				dev_err(ptj->dev, "get_temp failed (%d) [%d]\n",
						i, ret);
				return ret;
			}
			temp += ptj->tzs[i].offset;
			*out_temp = *out_temp > temp ? *out_temp : temp;
		}
	}
	return 0;
}

static struct thermal_zone_of_device_ops of_thermal_ops = {
	.get_temp = tegra_tj_thermal_get_temp,
};

static const struct of_device_id tegra_tj_thermal_of_match[] = {
	{
		.compatible = "nvidia,tegra-tj-thermal",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_tj_thermal_of_match);

static int match(struct thermal_zone_device *tzd, void *data)
{
	return !strncmp(tzd->type, (char *)data, 32);
}

static int tegra_tj_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int i, ret = 0, j = 0;
	struct thermal_zone_device *tzd;
	int temp;

	struct tj *ptj = devm_kzalloc(dev, sizeof(*ptj), GFP_KERNEL);
	if (IS_ERR_OR_NULL(ptj))
		return -ENOMEM;

	platform_set_drvdata(pdev, ptj);
	ptj->dev = dev;

	ret = of_count_phandle_with_args(dev->of_node, "data", NULL);
	if (ret < 0) {
		dev_err(dev, "failed to parse sensors data\n");
		return ret;
	}

	if (ret % 2) {
		dev_err(dev, "invalid sensors data\n");
		return -ENODEV;
	}

	ptj->num_tzs = ret / 2;
	ptj->tzs = devm_kzalloc(dev, ptj->num_tzs * sizeof(struct tj_tzs), GFP_KERNEL);
	if (!ptj->tzs)
		return -ENOMEM;

	for (i = 0; i < ptj->num_tzs; i++) {
		struct of_phandle_args args;
		struct thermal_zone_device *tzd;

		ret = of_parse_phandle_with_fixed_args(dev->of_node, "data", 1,
				i, &args);
		if (ret) {
			dev_err(dev, "parse handle failed for %d [%d]\n", i,
					ret);
			continue;
		}
		if (args.args_count != 1)
			continue;

		tzd = thermal_zone_device_find((void *)args.np->name, match);
		if (IS_ERR_OR_NULL(tzd)) {
			dev_err(dev, "failed to find thermal zone for %s\n",
					args.np->name);
			ptj->tzs[i].tzd = NULL;
			continue;
		}
		dev_info(dev, "found thermal zone %s + offset %d\n", tzd->type,
				args.args[0]);

		/*
		 * To check if thermal sensor of thermal zone driver is already
		 * initialized, need to check with error code -EINVAL.
		 * get_temp call returns -EINVAL when thermal sensor driver is
		 * not initialied.
		 */
		if (thermal_zone_get_temp(tzd, &temp) == -EINVAL) {
			dev_info(dev, "%s driver not initialized.. Deferring the probe\n",
					tzd->type);
			return -EPROBE_DEFER;
		}

		ptj->tzs[i].tzd = tzd;
		ptj->tzs[i].offset = args.args[0];
		j++;
	}

	if (!j) {
		dev_err(dev, "could not add any thermal zone\n");
		return -ENODEV;
	}

	tzd = thermal_zone_of_sensor_register(dev, 0, ptj, &of_thermal_ops);
	if (IS_ERR_OR_NULL(tzd)) {
		ret = PTR_ERR(tzd);
		dev_err(dev, "failed to register sensor %d\n", ret);
	} else {
		ptj->self = tzd;
	}

	return ret;
}

static int tegra_tj_thermal_remove(struct platform_device *pdev)
{
	struct tj *ptj = platform_get_drvdata(pdev);
	thermal_zone_of_sensor_unregister(&pdev->dev, ptj->self);
	return 0;
}

static struct platform_driver tegra_tj_thermal_driver = {
	.probe = tegra_tj_thermal_probe,
	.remove = tegra_tj_thermal_remove,
	.driver = {
		.name = "tegra-tj-thermal",
		.of_match_table = tegra_tj_thermal_of_match,
	},
};
module_platform_driver(tegra_tj_thermal_driver);

MODULE_AUTHOR("Navneet Kumar <navneetk@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Tj sensor driver");
MODULE_LICENSE("GPL v2");
