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

#include "nvs_sysfs.h"
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/input.h>

#define NVS_INPUT_DRIVER_VERSION	(1)
#define NVS_INPUT_DRIVER_NAME		"input"

struct nvs_state_input {
	struct input_dev *idev;
	unsigned long input_no;
	unsigned int code_n;
	unsigned int code_n_mod;
};


static int nvs_input_push(struct nvs_state *st)
{
	struct nvs_state_input *si = nvs_st_kif(st);
	unsigned int val;
	unsigned int i;

	for (i = 0; i < si->code_n; i++) {
		memcpy(&val, &st->buf[i * sizeof(val)], sizeof(val));
		input_report_rel(si->idev, i, val);
	}

	if (si->code_n_mod) {
		val = 0;
		memcpy(&val, &st->buf[i * sizeof(val)], si->code_n_mod);
		input_report_rel(si->idev, i, val);
	}
	input_sync(si->idev);
	return 0;
}

static void nvs_input_remove(struct nvs_state *st)
{
	struct nvs_state_input *si = nvs_st_kif(st);

	if (si->idev) {
		if (si->input_no != -1 && (st->cfg->flags &
					   SENSOR_FLAG_DYNAMIC_SENSOR))
			nvs_dsm_input(si->input_no, false, st->snsr_type,
				      st->cfg->uuid);
		input_unregister_device(si->idev);
	}
}

static int nvs_input_init(struct nvs_state *st)
{
	struct nvs_state_input *si = nvs_st_kif(st);
	struct input_dev *idev;
	unsigned int i;
	unsigned int n;
	int ret;

	si->input_no = -1;
	idev = devm_input_allocate_device(st->dev);
	if (!idev) {
		dev_err(st->dev, "ERR: devm_input_allocate_device\n");
		return -ENOMEM;
	}

	idev->name = st->cfg->name;
	idev->id.version = st->cfg->version;
	idev->dev.parent = st->dev;
	/* input can only handle 32-bit data so we'll break up all the channel
	 * data, regardless of size, into 32-bit chunks.
	 */
	/* get total number of bytes we'll be sending per event */
	n = 0;
	for (i = 0; i < st->ch_n; i++)
		n += st->buf_ch[i].byte_n;
	si->code_n_mod = n % 4;
	n /= 4;
	si->code_n = n;
	if (si->code_n_mod)
		n++;
	for (i = 0; i < n; i++)
		input_set_capability(idev, EV_REL, i);

	ret = input_register_device(idev);
	if (ret) {
		dev_err(st->dev, "%s input_register_device ERR=%d\n",
			__func__, ret);
		return ret;
	}

	si->idev = idev;
	dev_set_drvdata(&idev->dev, st);
	st->attr_grp.name = "attr";
	ret = sysfs_create_group(&idev->dev.kobj,
				(const struct attribute_group *)&st->attr_grp);
	if (ret) {
		st->attr_grp.attrs = NULL;
		nvs_input_remove(st);
		return ret;
	}

	if (st->cfg->flags & SENSOR_FLAG_DYNAMIC_SENSOR) {
		sscanf(idev->dev.kobj.name, "input%lu", &si->input_no);
		nvs_dsm_input(si->input_no, true, st->snsr_type, st->cfg->uuid);
	}
	return 0;
}

static struct nvs_kif_fn nvs_kif_fn_input = {
	.name				= NVS_INPUT_DRIVER_NAME,
	.driver_version			= NVS_INPUT_DRIVER_VERSION,
	.init				= nvs_input_init,
	.remove				= nvs_input_remove,
	.push				= nvs_input_push,
};

static int nvs_probe_input(void **handle, void *dev_client, struct device *dev,
			   struct nvs_fn_dev *fn_dev,
			   struct sensor_cfg *snsr_cfg)
{
	return nvs_probe(handle, dev_client, dev, fn_dev, snsr_cfg,
			 &nvs_kif_fn_input, sizeof(struct nvs_state_input));
}

static struct nvs_fn_if nvs_fn_if_input = {
	.probe				= nvs_probe_input,
	.remove				= nvs_remove,
	.shutdown			= nvs_shutdown,
	.nvs_mutex_lock			= nvs_mutex_lock,
	.nvs_mutex_unlock		= nvs_mutex_unlock,
	.suspend			= nvs_suspend,
	.resume				= nvs_resume,
	.handler			= nvs_handler,
};

struct nvs_fn_if *nvs_input(void)
{
	return &nvs_fn_if_input;
}
EXPORT_SYMBOL_GPL(nvs_input);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor Input module");
MODULE_AUTHOR("NVIDIA Corporation");

