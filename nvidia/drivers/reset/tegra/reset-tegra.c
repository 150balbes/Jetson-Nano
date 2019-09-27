/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <soc/tegra/tegra_bpmp.h>
#include <dt-bindings/reset/tegra186-reset.h>
#include "reset.h"

static int tegra18x_reset_probe(struct platform_device *pdev)
{
	uint32_t max_id;
	int r;

	r = bpmp_get_max_reset_id(&max_id);
	if (r) {
		/* for backward compatibility */
		return bpmp_register_reset(TEGRA186_RESET_SIZE, pdev);
	}

	return bpmp_register_reset(max_id + 1, pdev);
}

static int tegra_reset_probe(struct platform_device *pdev)
{
	int (*probe)(struct platform_device *);

	probe = of_device_get_match_data(&pdev->dev);
	if (!probe)
		return -ENODEV;

	return probe(pdev);
}

static const struct of_device_id tegra_reset_match[] = {
	{ .compatible = "nvidia,tegra18x-car", .data = tegra18x_reset_probe },
	{ .compatible = "nvidia,bpmp-resets", .data = tegra18x_reset_probe },
	{ },
};

static struct platform_driver tegra_reset_driver = {
	.probe = tegra_reset_probe,
	.driver = {
		.name		= "tegra-reset",
		.owner		= THIS_MODULE,
		.of_match_table	= tegra_reset_match,
	},
};

static int __init tegra_reset_driver_init(void)
{
	return platform_driver_register(&tegra_reset_driver);
}

static void __exit tegra_reset_driver_exit(void)
{
	platform_driver_unregister(&tegra_reset_driver);
}

arch_initcall(tegra_reset_driver_init);
module_exit(tegra_reset_driver_exit);

MODULE_AUTHOR("Peter De Schrijver <pdeschrijver@nvidia.com>");
MODULE_DESCRIPTION("Tegra Reset Controller Driver");
MODULE_LICENSE("GPL");
