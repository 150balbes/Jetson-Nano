/*
 * drivers/power/reset/tmpm32xi2c-poweroff.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/mfd/tmpm32xi2c.h>
#ifdef CONFIG_TEGRA_HV_PM_CTL
#include <soc/tegra/virt/tegra_hv_pm_ctl.h>
#endif

struct tmpm32xi2c_poweroff_data {
	struct device *dev;
	struct notifier_block reboot_nb;
};

static int __tmpm32xi2c_send_shutdown_prepare(
		struct tmpm32xi2c_poweroff_data *data, unsigned char val)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	unsigned char tx_buf[2];
	int ret;

	tx_buf[0] = CMD_SHUTDOWN_PREPARE;
	tx_buf[1] = val;

	/* Send a CMD_SHUTDOWN_PREPARE command to MCU */
	ret = chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
	if (ret < 0)
		dev_err(data->dev,
			"Failed to send SHUTDOWN_PREPARE_%s command, %d\n",
			(val == SHUTDOWN_PREPARE_SET) ? "SET" : "CLEAR", ret);
	else
		dev_info(data->dev,
			"Sent SHUTDOWN_PREPARE_%s command.\n",
			(val == SHUTDOWN_PREPARE_SET) ? "SET" : "CLEAR");

	return ret;
}

#ifdef CONFIG_TEGRA_HV_PM_CTL
static struct tmpm32xi2c_poweroff_data *poweroff_data;

static int tmpm32xi2c_set_shutdown_prepare(void)
{
	if (!poweroff_data)
		return -ENXIO;

	return __tmpm32xi2c_send_shutdown_prepare(poweroff_data,
						  SHUTDOWN_PREPARE_SET);
}
#endif

static int tmpm32xi2c_poweroff_reboot_notify(struct notifier_block *nb,
					     unsigned long mode, void *cmd)
{
	struct tmpm32xi2c_poweroff_data *data =
		container_of(nb, struct tmpm32xi2c_poweroff_data, reboot_nb);
	int ret;

	if ((mode == SYS_HALT) || (mode == SYS_POWER_OFF)) {
		/*
		 * Send a CMD_SHUTDOWN_PREPARE set command to notify MCU that
		 * the system is going to shutdown.
		 */
		ret = __tmpm32xi2c_send_shutdown_prepare(data,
							 SHUTDOWN_PREPARE_SET);
		if (ret < 0)
			dev_err(data->dev,
				"Failed to send SHUTDOWN_PREPARE_SET, %d\n",
				ret);
		else
			dev_dbg(data->dev, "Sent SHUTDOWN_PREPARE_SET\n");
	}

	return NOTIFY_DONE;
}

static int tmpm32xi2c_poweroff_probe(struct platform_device *pdev)
{
	struct tmpm32xi2c_poweroff_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev,
			    sizeof(struct tmpm32xi2c_poweroff_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;

	/*
	 * Send a CMD_SHUTDOWN_PREPARE clear command to clear a flag of
	 * shutdown prepare in MCU F/W.
	 */
	ret = __tmpm32xi2c_send_shutdown_prepare(data, SHUTDOWN_PREPARE_CLEAR);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to send SHUTDOWN_PREPARE_CLEAR, %d\n", ret);
		return ret;
	}
	dev_dbg(data->dev, "Sent SHUTDOWN_PREPARE_CLEAR\n");

	data->reboot_nb.notifier_call = tmpm32xi2c_poweroff_reboot_notify;
	ret = register_reboot_notifier(&data->reboot_nb);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Failed to register reboot notifier, %d\n", ret);
		return ret;
	}

#ifdef CONFIG_TEGRA_HV_PM_CTL
	poweroff_data = data;
	tegra_hv_pm_ctl_prepare_shutdown = tmpm32xi2c_set_shutdown_prepare;
#endif

	return 0;
}

static int tmpm32xi2c_poweroff_remove(struct platform_device *pdev)
{
	struct tmpm32xi2c_poweroff_data *data = platform_get_drvdata(pdev);

#ifdef CONFIG_TEGRA_HV_PM_CTL
	poweroff_data = NULL;
#endif
	unregister_reboot_notifier(&data->reboot_nb);

	return 0;
}

static const struct platform_device_id tmpm32xi2c_poweroff_id[] = {
	{ "tmpm32xi2c-poweroff", 0, },
	{ }
};
MODULE_DEVICE_TABLE(platform, tmpm32xi2c_poweroff_id);

static struct platform_driver tmpm32xi2c_poweroff_driver = {
	.driver = {
		.name	= "tmpm32xi2c-poweroff",
	},
	.probe		= tmpm32xi2c_poweroff_probe,
	.remove		= tmpm32xi2c_poweroff_remove,
	.id_table	= tmpm32xi2c_poweroff_id,
};

static int __init tmpm32xi2c_poweroff_init(void)
{
	return platform_driver_register(&tmpm32xi2c_poweroff_driver);
}
/* register after postcore initcall and subsys initcall that may rely on I2C. */
subsys_initcall_sync(tmpm32xi2c_poweroff_init);

static void __exit tmpm32xi2c_poweroff_exit(void)
{
	platform_driver_unregister(&tmpm32xi2c_poweroff_driver);
}
module_exit(tmpm32xi2c_poweroff_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Power off driver for TMPM32x I2C");
MODULE_LICENSE("GPL");
