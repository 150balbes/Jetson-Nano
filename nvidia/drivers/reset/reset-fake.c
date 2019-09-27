/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/reset-controller.h>
#include <linux/slab.h>

static int fake_reset_op(struct reset_controller_dev *rcdev, unsigned long id)
{
	return 0;
}

static struct reset_control_ops fake_reset_ops = {
	.assert		= fake_reset_op,
	.deassert	= fake_reset_op,
	.reset		= fake_reset_op,
};

int register_fake_reset(int num_resets, struct platform_device *pdev)
{
	struct reset_controller_dev *rcdev;

	rcdev = devm_kzalloc(&pdev->dev, sizeof(*rcdev), GFP_KERNEL);
	if (!rcdev)
		return -ENOMEM;

	rcdev->owner = THIS_MODULE;
	rcdev->nr_resets = num_resets;
	rcdev->ops = &fake_reset_ops;
	rcdev->of_node = pdev->dev.of_node;

	dev_info(&pdev->dev, "registered %d fake resets\n", num_resets);

	return reset_controller_register(rcdev);
}

static int fake_reset_probe(struct platform_device *pdev)
{
	return register_fake_reset(512, pdev);
}

static const struct of_device_id fake_reset_match[] = {
	{ .compatible = "fake-resets" },
	{ },
};

static struct platform_driver fake_reset_driver = {
	.probe = fake_reset_probe,
	.driver = {
		.name		= "fake-reset",
		.owner		= THIS_MODULE,
		.of_match_table	= fake_reset_match,
	},
};

static int __init fake_reset_driver_init(void)
{
	return platform_driver_register(&fake_reset_driver);
}
arch_initcall(fake_reset_driver_init);
