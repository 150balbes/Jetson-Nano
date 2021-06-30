/*
 * isc_pwm.c - ISC PWM driver.
 *
 * Copyright (c) 2016-2018 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/atomic.h>

#include "isc-pwm-priv.h"

/*
 * Below values are configured during suspend.
 * Invalid values are chosen so that PWM
 * configuration in resume works fine.
 * Period is chosen as least non-zero value
 * and duty-ratio zero.
 */
#define PWM_SUSPEND_PERIOD		1
#define PWM_SUSPEND_DUTY_RATIO		0

static const struct of_device_id isc_pwm_of_match[] = {
	{ .compatible = "nvidia, isc-pwm", .data = NULL },
	{},
};

static inline struct isc_pwm_info *to_isc_pwm_info(struct pwm_chip *chip)
{
	return container_of(chip, struct isc_pwm_info, chip);
}

static int isc_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct isc_pwm_info *info = to_isc_pwm_info(chip);
	int err = 0;

	if (!chip || !pwm)
		return -EINVAL;

	if (info->force_on)
		return err;

	mutex_lock(&info->mutex);

	if (atomic_inc_return(&info->in_use) == 1)
		err = pwm_enable(info->pwm);

	mutex_unlock(&info->mutex);

	return err;
}

static void isc_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct isc_pwm_info *info = to_isc_pwm_info(chip);
	int atomic_val;

	mutex_lock(&info->mutex);

	atomic_val = atomic_read(&info->in_use);
	if (atomic_val > 0) {
		if (atomic_dec_and_test(&info->in_use))
			pwm_disable(info->pwm);
	}

	mutex_unlock(&info->mutex);
}

static int isc_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			int duty_ns, int period_ns)
{
	struct isc_pwm_info *info = to_isc_pwm_info(chip);
	int err = 0;

	if (info->force_on)
		return err;

	mutex_lock(&info->mutex);

	err = pwm_config(info->pwm, duty_ns, period_ns);

	mutex_unlock(&info->mutex);

	return err;
}

static struct pwm_device *of_isc_pwm_xlate(struct pwm_chip *pc,
			const struct of_phandle_args *args)
{
	struct pwm_device *pwm;
	struct isc_pwm_info *info = to_isc_pwm_info(pc);
	int err = 0;

	pwm = pwm_request_from_chip(pc, args->args[0], NULL);
	if (!args->args[1]) {
		dev_err(pc->dev, "Period should be larger than 0\n");
		return NULL;
	}

	if (info->force_on) {
		err = pwm_config(info->pwm, args->args[1]/4, args->args[1]);
		if (err) {
			dev_err(pc->dev, "can't config PWM: %d\n", err);
			return NULL;
		}

		err = pwm_enable(info->pwm);
		if (err) {
			dev_err(pc->dev, "can't enable PWM: %d\n", err);
			return NULL;
		}
	} else {
		err = pwm_config(pwm, args->args[1]/4, args->args[1]);
		if (err) {
			dev_err(pc->dev, "can't config PWM: %d\n", err);
			return NULL;
		}
	}

	return pwm;
}

static const struct pwm_ops isc_pwm_ops = {
	.config = isc_pwm_config,
	.enable = isc_pwm_enable,
	.disable = isc_pwm_disable,
	.owner = THIS_MODULE,
};

static int isc_pwm_probe(struct platform_device *pdev)
{
	struct isc_pwm_info *info = NULL;
	int err = 0, npwm;
	bool force_on = false;

	dev_info(&pdev->dev, "%sing...\n", __func__);

	info = devm_kzalloc(
		&pdev->dev, sizeof(struct isc_pwm_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	atomic_set(&info->in_use, 0);
	mutex_init(&info->mutex);

	err = of_property_read_u32(pdev->dev.of_node, "npwm", &npwm);
	if (err < 0) {
		dev_err(&pdev->dev, "npwm is not defined: %d\n", err);
		return err;
	}

	force_on = of_property_read_bool(pdev->dev.of_node, "force_on");

	info->chip.dev = &pdev->dev;
	info->chip.ops = &isc_pwm_ops;
	info->chip.base = -1;
	info->chip.npwm = npwm;
	info->chip.of_xlate = of_isc_pwm_xlate;
	info->chip.of_pwm_n_cells = 2;
	info->force_on = force_on;

	err = pwmchip_add(&info->chip);
	if (err < 0) {
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", err);
		return err;
	}

	platform_set_drvdata(pdev, info);

	info->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (!IS_ERR(info->pwm)) {
		pwm_disable(info->pwm);
		dev_info(&pdev->dev, "%s success to get PWM\n", __func__);
	} else {
		pwmchip_remove(&info->chip);
		err = PTR_ERR(info->pwm);
		if (err != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"%s: fail to get PWM\n", __func__);
	}

	return err;
}

static int isc_pwm_remove(struct platform_device *pdev)
{
	struct isc_pwm_info *info = platform_get_drvdata(pdev);

	return pwmchip_remove(&info->chip);
}

static int isc_pwm_suspend(struct device *dev)
{
	int err = 0;
	struct isc_pwm_info *info = dev_get_drvdata(dev);

	pwm_disable(info->pwm);
	err = pwm_config(info->pwm, PWM_SUSPEND_DUTY_RATIO, PWM_SUSPEND_PERIOD);
	return 0;
}

static int isc_pwm_resume(struct device *dev)
{
	/* Do nothing */
	return 0;
}

static const struct dev_pm_ops isc_pwm_pm_ops = {
	.suspend = isc_pwm_suspend,
	.resume = isc_pwm_resume,
	.runtime_suspend = isc_pwm_suspend,
	.runtime_resume = isc_pwm_resume,
};

static struct platform_driver isc_pwm_driver = {
	.driver = {
		.name = "isc-pwm",
		.owner = THIS_MODULE,
		.of_match_table = isc_pwm_of_match,
		.pm = &isc_pwm_pm_ops,
	},
	.probe = isc_pwm_probe,
	.remove = isc_pwm_remove,
};

module_platform_driver(isc_pwm_driver);

MODULE_AUTHOR("Junghyun Kim <juskim@nvidia.com>");
MODULE_DESCRIPTION("ISC PWM driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, isc_pwm_of_match);
