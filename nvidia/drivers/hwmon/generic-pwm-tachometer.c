/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/therm_est.h>

struct pwm_hwmon_tach {
	struct device		*dev;
	struct pwm_device	*pwm;
	struct device		*hwmon;
};

struct device *pwm_tach_dev;

int pwm_tach_capture_rpm(struct device *dev)
{
	int ret;
	struct pwm_hwmon_tach *ptt = dev_get_drvdata(dev);
	struct pwm_device *pwm = ptt->pwm;
	struct pwm_capture result;
	unsigned int rpm = 0;

	ret = pwm_capture(pwm, &result, 0);
	if (ret < 0) {
		dev_err(ptt->dev, "Failed to capture PWM: %d\n", ret);
		return ret;
	}

	if (result.period)
		rpm = DIV_ROUND_CLOSEST_ULL(60ULL * NSEC_PER_SEC,
					    result.period);

	return rpm;
}
EXPORT_SYMBOL(pwm_tach_capture_rpm);

static ssize_t show_rpm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int rpm;

	rpm = pwm_tach_capture_rpm(dev);
	if (rpm < 0)
		rpm = 0;

	return sprintf(buf, "%u\n", rpm);
}

static SENSOR_DEVICE_ATTR(rpm, 0444, show_rpm, NULL, 0);

static struct attribute *pwm_tach_attrs[] = {
	&sensor_dev_attr_rpm.dev_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(pwm_tach);

struct device *pwm_get_tach_dev(void)
{
	return pwm_tach_dev;
}
EXPORT_SYMBOL(pwm_get_tach_dev);

static int pwm_tach_probe(struct platform_device *pdev)
{
	struct pwm_hwmon_tach *ptt;

	ptt = devm_kzalloc(&pdev->dev, sizeof(*ptt), GFP_KERNEL);
	if (!ptt)
		return -ENOMEM;

	ptt->dev = &pdev->dev;

	platform_set_drvdata(pdev, ptt);
	dev_set_drvdata(&pdev->dev, ptt);

	ptt->pwm = devm_of_pwm_get(&pdev->dev, pdev->dev.of_node, NULL);
	if (IS_ERR(ptt->pwm)) {
		dev_err(&pdev->dev, "Failed to get pwm:  %ld\n",
			PTR_ERR(ptt->pwm));
		return PTR_ERR(ptt->pwm);
	}

	ptt->hwmon = devm_hwmon_device_register_with_groups(&pdev->dev,
					 "pwm_tach", ptt, pwm_tach_groups);
	if (IS_ERR(ptt->hwmon)) {
		dev_err(&pdev->dev, "Failed to register hwmon device: %d\n",
			PTR_ERR_OR_ZERO(ptt->hwmon));
		return PTR_ERR_OR_ZERO(ptt->hwmon);
	}

	pwm_tach_dev = ptt->dev;
	return 0;
}

static const struct of_device_id pwm_tach_of_match[] = {
	{ .compatible = "generic-pwm-tachometer" },
	{}
};
MODULE_DEVICE_TABLE(of, tach_of_match);

static struct platform_driver pwm_tach_driver = {
	.driver = {
		.name = "generic-pwm-tachometer",
		.of_match_table = pwm_tach_of_match,
	},
	.probe = pwm_tach_probe,
};

module_platform_driver(pwm_tach_driver);

MODULE_DESCRIPTION("PWM based Generic Tachometer driver");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_AUTHOR("R Raj Kumar <rrajk@nvidia.com>");
MODULE_LICENSE("GPL v2");
