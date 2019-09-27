/* Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/relay.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include "nvs_sysfs.h"

#define NVS_RELAY_DRIVER_VERSION	(1)
#define NVS_RELAY_DRIVER_NAME		"relay"
#define NVS_DEV_MAX			(256)
#define NVS_MAX_GROUPS			(2)

static DEFINE_IDA(nvs_ida);

static dev_t nvs_devt;

struct bus_type nvs_bus_type = {
	.name = "nvs",
};

struct nvs_state_relay {
	struct device dev;
	struct device_type dev_type;
	struct cdev cdev;
	unsigned int dev_id;
	char link_name[16];
	const struct attribute_group *groups[NVS_MAX_GROUPS + 1];
};


static int nvs_relay_push(struct nvs_state *st)
{
	return 0;
}

static void nvs_relay_remove(struct nvs_state *st)
{
	struct nvs_state_relay *sr = nvs_st_kif(st);

	if (st->cfg->flags & SENSOR_FLAG_DYNAMIC_SENSOR)
		nvs_dsm_relay(sr->dev_id, false, st->snsr_type, st->cfg->uuid);
}

static void nvs_dev_type_release(struct device *dev)
{
}

static int nvs_cdev_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int nvs_cdev_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations nvs_fops = {
	.owner = THIS_MODULE,
	.llseek = noop_llseek,
	.open = nvs_cdev_open,
	.release = nvs_cdev_release,
};

static int nvs_relay_init(struct nvs_state *st)
{
	struct nvs_state_relay *sr = nvs_st_kif(st);
	int ret;

	ret = ida_simple_get(&nvs_ida, 0, 0, GFP_KERNEL);
	if (ret < 0) {
		sr->dev_id = -1;
		dev_err(st->dev, "%s ida_simple_get ERR=%d\n", __func__, ret);
		return ret;
	}

	sr->dev_id = ret;
	snprintf(sr->link_name, sizeof(sr->link_name),
		 "nvs_device_%u", sr->dev_id);
	sr->dev_type.name = sr->link_name;
	sr->dev_type.release = nvs_dev_type_release;
	sr->dev.type = &sr->dev_type;
	sr->dev.bus = &nvs_bus_type;
	sr->groups[0] = &st->attr_grp;
	sr->dev.groups = sr->groups;
	device_initialize(&sr->dev);
	dev_set_drvdata(&sr->dev, st);
	dev_set_name(&sr->dev, "nvs:device%d", sr->dev_id);
	sr->dev.parent = st->dev;
	sr->dev.of_node = st->dev->of_node;
	sr->dev.devt = MKDEV(MAJOR(nvs_devt), sr->dev_id);
	ret = device_add(&sr->dev);
	if (ret < 0)
		return ret;

	ret = sysfs_create_link(&st->dev->kobj, &sr->dev.kobj,
				sr->dev_type.name);
	if (ret)
		return ret;

	cdev_init(&sr->cdev, &nvs_fops);
	sr->cdev.owner = THIS_MODULE;
	ret = cdev_add(&sr->cdev, sr->dev.devt, 1);
	if (ret < 0)
		return ret;

	if (st->cfg->flags & SENSOR_FLAG_DYNAMIC_SENSOR)
		nvs_dsm_relay(sr->dev_id, true, st->snsr_type, st->cfg->uuid);
	return 0;
}

static struct nvs_kif_fn nvs_kif_fn_relay = {
	.name				= NVS_RELAY_DRIVER_NAME,
	.driver_version			= NVS_RELAY_DRIVER_VERSION,
	.init				= nvs_relay_init,
	.remove				= nvs_relay_remove,
	.push				= nvs_relay_push,
};

static int nvs_probe_relay(void **handle, void *dev_client, struct device *dev,
			   struct nvs_fn_dev *fn_dev,
			   struct sensor_cfg *snsr_cfg)
{
	return nvs_probe(handle, dev_client, dev, fn_dev, snsr_cfg,
			 &nvs_kif_fn_relay, sizeof(struct nvs_state_relay));
}

static struct nvs_fn_if nvs_fn_if_relay = {
	.probe				= nvs_probe_relay,
	.remove				= nvs_remove,
	.shutdown			= nvs_shutdown,
	.nvs_mutex_lock			= nvs_mutex_lock,
	.nvs_mutex_unlock		= nvs_mutex_unlock,
	.suspend			= nvs_suspend,
	.resume				= nvs_resume,
	.handler			= nvs_handler,
};

struct nvs_fn_if *nvs_relay(void)
{
	return &nvs_fn_if_relay;
}
EXPORT_SYMBOL_GPL(nvs_relay);

static int __init nvs_init(void)
{
	int ret;

	ret = bus_register(&nvs_bus_type);
	if (ret < 0)
		return ret;

	ret = alloc_chrdev_region(&nvs_devt, 0, NVS_DEV_MAX, "nvs");
	if (ret < 0) {
		bus_unregister(&nvs_bus_type);
		return ret;
	}

	return 0;
}

static void __exit nvs_exit(void)
{
	if (nvs_devt)
		unregister_chrdev_region(nvs_devt, NVS_DEV_MAX);
	bus_unregister(&nvs_bus_type);
}

subsys_initcall(nvs_init);
module_exit(nvs_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor Relay module");
MODULE_AUTHOR("NVIDIA Corporation");

