/*
 * extcon-disp-state - extcon driver for display accessory detection
 *		compatible with switch-mid
 *
 * Copyright (c) 2018-2020, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/extcon/extcon-disp.h>
#include <linux/extcon.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#if KERNEL_VERSION(4, 14, 0) <= LINUX_VERSION_CODE
#include "extcon.h"
#endif

#define EXTCON_DISP_AUX_BASE		2
#define EXTCON_DISP_MAX_AUX_CONNECTORS	4

struct disp_state_extcon_info {
	struct device *dev;
	struct extcon_dev *edev;
};

static const unsigned int disp_state_extcon_cables[] = {
	EXTCON_DISP_HDMI,
	EXTCON_DISP_DP,
	EXTCON_DISP_AUDIO_AUX0,
	EXTCON_DISP_AUDIO_AUX1,
	EXTCON_DISP_AUDIO_AUX2,
	EXTCON_DISP_AUDIO_AUX3,
	EXTCON_DISP_DSIHPD,
	EXTCON_DISP_HDMI2,
	EXTCON_NONE,
};

static struct disp_state_extcon_info *disp_extcon_info;
static struct class_compat *switch_class;

void disp_state_extcon_switch_report(const unsigned int cable, bool state)
{
	if ((!disp_extcon_info) || (!disp_extcon_info->edev) ||
		(!disp_extcon_info->dev))
		return;

#if KERNEL_VERSION(4, 9, 0) > LINUX_VERSION_CODE
	if (extcon_get_cable_state_(disp_extcon_info->edev, cable) == state) {
#else
	if (extcon_get_state(disp_extcon_info->edev, cable) == state) {
#endif
		dev_info(disp_extcon_info->dev, "cable %d state %d already set.\n",
			cable, state);
		return;
	}

#if KERNEL_VERSION(4, 9, 0) > LINUX_VERSION_CODE
	extcon_set_cable_state_(disp_extcon_info->edev, cable, state);
#else
	extcon_set_state_sync(disp_extcon_info->edev, cable, state);
#endif
	dev_info(disp_extcon_info->dev, "cable %d state %d\n", cable, state);
}

void disp_state_extcon_aux_report(const unsigned int aux_idx, bool state)
{
	if ((!disp_extcon_info) || (!disp_extcon_info->edev) ||
		(!disp_extcon_info->dev))
		return;

	if (aux_idx < EXTCON_DISP_MAX_AUX_CONNECTORS) {
		disp_state_extcon_switch_report(
			disp_state_extcon_cables[EXTCON_DISP_AUX_BASE +
			aux_idx], state);
	} else {
		dev_info(disp_extcon_info->dev, "AUX%d is invalid\n", aux_idx);
		return;
	}
}

static int disp_state_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct disp_state_extcon_info *info;
	int retval = 0;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->edev = devm_extcon_dev_allocate(dev, disp_state_extcon_cables);
	if (IS_ERR(info->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	switch_class = class_compat_register("switch_disp_state");
	if (WARN(!switch_class, "cannot allocate"))
		return -ENOMEM;

	info->edev->name = "disp_state_ext";

	retval = devm_extcon_dev_register(dev, info->edev);
	if (retval < 0) {
		class_compat_unregister(switch_class);
		dev_err(dev, "failed to register extcon device\n");
		return retval;
	}

	retval = class_compat_create_link(switch_class, dev, NULL);
	if (retval)
		dev_warn(dev, "Create compatibility class link failed\n");

	platform_set_drvdata(pdev, info);
	disp_extcon_info = info;

	return 0;
}

static int disp_state_extcon_remove(struct platform_device *pdev)
{
	struct disp_state_extcon_info *extcon_data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	class_compat_remove_link(switch_class, dev, NULL);
	class_compat_unregister(switch_class);
	extcon_dev_unregister(extcon_data->edev);
	disp_extcon_info = NULL;
	return 0;
}

static const struct of_device_id of_extcon_disp_state_tbl[] = {
	{ .compatible = "extcon-disp-state", },
	{ /* end */ }
};

MODULE_DEVICE_TABLE(of, of_extcon_disp_state_tbl);

static struct platform_driver disp_state_extcon_driver = {
	.probe		= disp_state_extcon_probe,
	.remove		= disp_state_extcon_remove,
	.driver		= {
		.name	= "extcon-disp-state",
		.owner	= THIS_MODULE,
		.of_match_table = of_extcon_disp_state_tbl,
	},
};

static int __init disp_state_extcon_driver_init(void)
{
	return platform_driver_register(&disp_state_extcon_driver);
}
subsys_initcall_sync(disp_state_extcon_driver_init);

static void __exit disp_state_extcon_driver_exit(void)
{
	platform_driver_unregister(&disp_state_extcon_driver);
}
module_exit(disp_state_extcon_driver_exit);

MODULE_AUTHOR("Dara Ramesh (dramesh@nvidia.com)");
MODULE_DESCRIPTION("EXTCON Extcon driver");
MODULE_LICENSE("GPL v2");
