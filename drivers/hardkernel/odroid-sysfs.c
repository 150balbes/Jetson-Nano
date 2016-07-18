/*
 * ODROID sysfs support for extra feature enhancement
 *
 * Copyright (C) 2014, Hardkernel Co,.Ltd
 * Author: Charles Park <charles.park@hardkernel.com>
 * Author: Dongjin Kim <tobetter@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <asm/setup.h>

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
#include <linux/amlogic/iomap.h>
#endif

MODULE_AUTHOR("Hardkernel Co,.Ltd");
MODULE_DESCRIPTION("SYSFS driver for ODROID hardware");
MODULE_LICENSE("GPL");

static struct hrtimer input_timer;
static struct input_dev *input_dev;
static int keycode[] = { KEY_POWER, };
static int key_release_seconds;

static ssize_t set_poweroff_trigger(struct class *class,
		struct class_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;

	if (0 == sscanf(buf, "%d\n", &val))
		return  -EINVAL;

	/* Emulate power button by software */
	if ((val != 0) && (val < 5)) {
		if (!key_release_seconds) {
			key_release_seconds = val;
			input_report_key(input_dev, KEY_POWER, 1);

			hrtimer_start(&input_timer,
					ktime_set(key_release_seconds, 0),
					HRTIMER_MODE_REL);

			input_sync(input_dev);
		}
	}

	return count;
}

static const char *product;
static const char *serialno;
static const char *mac_addr;

#define	REV_GPIOY_0	211	/* GPIO Number (GPIOY.0) */
#define	REV_GPIOY_1	212	/* GPIO Number (GPIOY.1) */

static ssize_t show_boardrev(struct class *class,
		struct class_attribute *attr, char *buf)
{
	int	rev;

	if (gpio_request_one(REV_GPIOY_0, GPIOF_IN, "rev")) {
		pr_err("%s : REV_GPIOY_0 request error!\n", __func__);
		return	0;
	}
	gpio_set_pullup(REV_GPIOY_0, 1);

	rev = gpio_get_value(REV_GPIOY_0) ? 0x01 : 0x00;

	gpio_free(REV_GPIOY_0);

	if (gpio_request_one(REV_GPIOY_1, GPIOF_IN, "rev")) {
		pr_err("%s : REV_GPIOY_1 request error!\n", __func__);
		return	0;
	}
	gpio_set_pullup(REV_GPIOY_1, 1);

	rev |= gpio_get_value(REV_GPIOY_1) ? 0x02 : 0x00;

	gpio_free(REV_GPIOY_1);

	return snprintf(buf, PAGE_SIZE, "%d\n", rev);
}

static ssize_t show_product(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", product);
}

static ssize_t show_serialno(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", serialno);
}

static ssize_t show_mac_addr(struct class *class,
		struct class_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", mac_addr);
}

#if defined(CONFIG_ARCH_MESON64_ODROIDC2)
/*
 * Discover the boot device within MicroSD or eMMC
 * and return 1 for eMMC, otherwise 0.
 */
enum {
	BOOT_DEVICE_RESERVED = 0,
	BOOT_DEVICE_EMMC = 1,
	BOOT_DEVICE_NAND = 2,
	BOOT_DEVICE_SPI = 3,
	BOOT_DEVICE_SD = 4,
	BOOT_DEVICE_USB = 5,
	BOOT_DEVICE_MAX,
};

static int get_boot_device(void)
{
	int bootdev = aml_read_aobus(0x90 << 2) & 0xf;

	if (bootdev >= BOOT_DEVICE_MAX)
		return BOOT_DEVICE_RESERVED;

	return bootdev;
}

int board_boot_from_emmc(void)
{
	return !!(get_boot_device() == BOOT_DEVICE_EMMC);
}
EXPORT_SYMBOL(board_boot_from_emmc);

static ssize_t show_bootdev(struct class *class,
		struct class_attribute *attr, char *buf)
{
	const char *boot_dev_name[BOOT_DEVICE_MAX] = {
		"unknown",	/* reserved boot device treated as 'unknown' */
		"emmc",
		"nand",
		"spi",
		"sd",
		"usb"
	};

	return snprintf(buf, PAGE_SIZE, "%s\n",
			boot_dev_name[get_boot_device()]);
}
#endif

static struct class_attribute odroid_class_attrs[] = {
	__ATTR(poweroff_trigger, 0222, NULL, set_poweroff_trigger),
	__ATTR(product, 0444, show_product, NULL),
	__ATTR(serialno, 0444, show_serialno, NULL),
	__ATTR(mac_addr, 0444, show_mac_addr, NULL),
	__ATTR(bootdev, 0444, show_bootdev, NULL),
	__ATTR(boardrev, 0444, show_boardrev, NULL),
	__ATTR_NULL,
};

static struct class odroid_class = {
	.name = "odroid",
	.owner = THIS_MODULE,
	.class_attrs = odroid_class_attrs,
};

static enum hrtimer_restart input_timer_function(struct hrtimer *timer)
{
	key_release_seconds = 0;
	input_report_key(input_dev, KEY_POWER, 0);
	input_sync(input_dev);

	return HRTIMER_NORESTART;
}

static int odroid_sysfs_probe(struct platform_device *pdev)
{
	int error = 0;
#ifdef CONFIG_USE_OF
	struct device_node *node;
#endif

#if defined(SLEEP_DISABLE_FLAG)
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock(&sleep_wake_lock);
#endif
#endif
/***********************************************************************
 * virtual key init (Power Off Key)
***********************************************************************/
	input_dev = input_allocate_device();
	if (!input_dev) {
		error = -ENOMEM;
		goto err_out;
	}

	input_dev->name = "vt-input";
	input_dev->phys = "vt-input/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x16B4;
	input_dev->id.product = 0x0701;
	input_dev->id.version = 0x0001;
	input_dev->keycode = keycode;
	input_dev->keycodesize = sizeof(keycode[0]);
	input_dev->keycodemax = ARRAY_SIZE(keycode);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER & KEY_MAX, input_dev->keybit);

	error = input_register_device(input_dev);
	if (error) {
		input_free_device(input_dev);
		goto err_out;
	}

	pr_info(KERN_INFO "%s input driver registered!!\n", "Virtual-Key");

	hrtimer_init(&input_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	input_timer.function = input_timer_function;

#ifdef CONFIG_USE_OF
	if (pdev->dev.of_node) {
		node = pdev->dev.of_node;
		of_property_read_string(node, "product", &product);
		of_property_read_string(node, "serialno", &serialno);
		of_property_read_string(node, "mac_addr", &mac_addr);
	}
#endif

err_out:
	return error;
}

static  int odroid_sysfs_remove(struct platform_device *pdev)
{
#if defined(SLEEP_DISABLE_FLAG)
#if defined(CONFIG_HAS_WAKELOCK)
	wake_unlock(&sleep_wake_lock);
#endif
#endif
	return 0;
}

static int odroid_sysfs_suspend(struct platform_device *dev, pm_message_t state)
{
#if defined(DEBUG_PM_MSG)
	pr_info(KERN_INFO "%s\n", __func__);
#endif

	return 0;
}

static int odroid_sysfs_resume(struct platform_device *dev)
{
#if defined(DEBUG_PM_MSG)
	pr_info(KERN_INFO "%s\n", __func__);
#endif

	return  0;
}

#if defined(CONFIG_OF)
static const struct of_device_id odroid_sysfs_dt[] = {
	{ .compatible = "odroid-sysfs" },
	{ },
};
MODULE_DEVICE_TABLE(of, odroid_sysfs_dt);
#endif

static struct platform_driver odroid_sysfs_driver = {
	.driver = {
		.name = "odroid-sysfs",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(odroid_sysfs_dt),
#endif
	},
	.probe = odroid_sysfs_probe,
	.remove = odroid_sysfs_remove,
	.suspend = odroid_sysfs_suspend,
	.resume = odroid_sysfs_resume,
};

static int __init odroid_sysfs_init(void)
{
	int error = class_register(&odroid_class);
	if (0 > error)
		return error;

	pr_info(KERN_INFO "--------------------------------------------------------\n");
#if defined(SLEEP_DISABLE_FLAG)
#if defined(CONFIG_HAS_WAKELOCK)
	pr_info(KERN_INFO "%s(%d) : Sleep Disable Flag SET!!(Wake_lock_init)\n",
			__func__, __LINE__);

	wake_lock_init(&sleep_wake_lock, WAKE_LOCK_SUSPEND, "sleep_wake_lock");
#endif
#else
	pr_info(KERN_INFO "%s(%d) : Sleep Enable !!\n", __func__, __LINE__);
#endif
	pr_info(KERN_INFO "--------------------------------------------------------\n");

	return platform_driver_register(&odroid_sysfs_driver);
}

static void __exit odroid_sysfs_exit(void)
{
#if defined(SLEEP_DISABLE_FLAG)
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_destroy(&sleep_wake_lock);
#endif
#endif
	platform_driver_unregister(&odroid_sysfs_driver);
	class_unregister(&odroid_class);
}

module_init(odroid_sysfs_init);
module_exit(odroid_sysfs_exit);
