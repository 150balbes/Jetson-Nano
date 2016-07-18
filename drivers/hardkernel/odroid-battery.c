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

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>

static struct platform_device *odroid_pdev;

static int odroid_ac_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int odroid_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
		int ret = 0;

		switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				break;
		case POWER_SUPPLY_PROP_HEALTH:
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
				break;
		case POWER_SUPPLY_PROP_PRESENT:
				val->intval = 1;
				break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
				val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
				break;
		case POWER_SUPPLY_PROP_CAPACITY:
				val->intval = 100;
				break;
		case POWER_SUPPLY_PROP_TEMP:
				val->intval = 20;
				break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
				val->intval = 5;
				break;
		default:
				ret = -EINVAL;
				break;
		}

		return ret;
}

static enum power_supply_property odroid_battery_props[] = {
		POWER_SUPPLY_PROP_STATUS,
		POWER_SUPPLY_PROP_HEALTH,
		POWER_SUPPLY_PROP_PRESENT,
		POWER_SUPPLY_PROP_TECHNOLOGY,
		POWER_SUPPLY_PROP_VOLTAGE_NOW,
		POWER_SUPPLY_PROP_TEMP,
		POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property odroid_ac_props[] = {
		POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply odroid_bat = {
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = odroid_battery_props,
		.num_properties = ARRAY_SIZE(odroid_battery_props),
		.get_property = odroid_bat_get_property,
		.use_for_apm = 1,
};

static struct power_supply odroid_ac = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.properties =  odroid_ac_props,
		.num_properties = ARRAY_SIZE(odroid_ac_props),
		.get_property = odroid_ac_get_property,
};

static int __init odroid_init(void)
{
		int ret = 0;

		odroid_pdev = platform_device_register_simple(
				"battery", 0, NULL, 0);
		if (IS_ERR(odroid_pdev))
				return PTR_ERR(odroid_pdev);

		ret = power_supply_register(&odroid_pdev->dev, &odroid_bat);
		if (ret)
				goto bat_failed;

		ret = power_supply_register(&odroid_pdev->dev, &odroid_ac);
		if (ret)
				goto ac_failed;

		pr_info("adbattery: android dummy battery driver loaded\n");
		goto success;

bat_failed:
		power_supply_unregister(&odroid_bat);
ac_failed:
		power_supply_unregister(&odroid_ac);
		platform_device_unregister(odroid_pdev);
success:
		return ret;
}

static void __exit odroid_exit(void)
{
		power_supply_unregister(&odroid_bat);
		power_supply_unregister(&odroid_ac);
		platform_device_unregister(odroid_pdev);
		pr_info("adbattery: android dummy battery driver unloaded\n");
}

module_init(odroid_init);
module_exit(odroid_exit);

MODULE_AUTHOR("Dongjin Kim tobetter@gmail.com");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Android dummy battery driver");
