/*
 * Copyright (c) 2016-2019 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/ote_protocol.h>
#include <linux/delay.h>

extern phys_addr_t tegra_vpr_start;
extern phys_addr_t tegra_vpr_size;
extern bool tegra_vpr_resize;
static DEFINE_MUTEX(vpr_lock);

static int tegra_vpr_arg(char *options)
{
	char *p = options;

	tegra_vpr_size = memparse(p, &p);
	if (*p == '@')
		tegra_vpr_start = memparse(p+1, &p);
	pr_info("Found vpr, start=0x%llx size=%llx",
		(u64)tegra_vpr_start, (u64)tegra_vpr_size);
	return 0;
}
early_param("vpr", tegra_vpr_arg);

static int tegra_vpr_resize_arg(char *options)
{
	tegra_vpr_resize = true;
	return 0;
}
early_param("vpr_resize", tegra_vpr_resize_arg);

#define NUM_MODULES_IDLE_VPR_RESIZE 3
static struct vpr_user_module_info {
	int (*do_idle)(void *);
	int (*do_unidle)(void *);
	void *data;
} vpr_user_module[NUM_MODULES_IDLE_VPR_RESIZE];
static int _tegra_set_vpr_params(void *vpr_base, size_t vpr_size);

static int tegra_update_resize_cfg(phys_addr_t base , size_t size)
{
	int i = 0, err = 0;
#define MAX_RETRIES 6
	int retries = MAX_RETRIES;
	mutex_lock(&vpr_lock);
retry:
	for (; i < NUM_MODULES_IDLE_VPR_RESIZE; i++) {
		if (vpr_user_module[i].do_idle) {
			err = vpr_user_module[i].do_idle(
					vpr_user_module[i].data);
			if (err) {
				pr_err("%s:%d: %pF failed err:%d\n",
					 __func__, __LINE__,
					vpr_user_module[i].do_idle, err);
				break;
			}
		}
	}
	if (!err) {
		/* Config VPR_BOM/_SIZE in MC */
		err = _tegra_set_vpr_params((void *)(uintptr_t)base, size);
		if (err)
			pr_err("vpr resize to (%p, %zu) failed. err=%d\n",
				(void *)(uintptr_t)base, size, err);
		else
			retries = 0; /* finish */
	}
	if (retries--) {
		pr_err("%s:%d: fail retry=%d",
			__func__, __LINE__, MAX_RETRIES - retries);
		msleep(1);
		goto retry;
	}
	while (--i >= 0) {
		if (!vpr_user_module[i].do_unidle)
			continue;

		err = vpr_user_module[i].do_unidle(
				vpr_user_module[i].data);
		if (!err)
			continue;
		pr_err("%s:%d: %pF failed err:%d. Could be fatal!!\n",
			 __func__, __LINE__,
			vpr_user_module[i].do_unidle, err);
		/* vpr resize is success, so return 0 on unidle failure */
		err = 0;
	}
	mutex_unlock(&vpr_lock);
	return err;
}

struct dma_resize_notifier_ops vpr_dev_ops = {
	.resize = tegra_update_resize_cfg
};
EXPORT_SYMBOL(vpr_dev_ops);

bool tegra_is_vpr_resize_supported(void)
{
	return tegra_vpr_resize;
}
EXPORT_SYMBOL(tegra_is_vpr_resize_supported);

/* SMC Definitions*/
#define TE_SMC_PROGRAM_VPR 0x82000003

uint32_t invoke_smc(uint32_t arg0, uintptr_t arg1, uintptr_t arg2);

static int _tegra_set_vpr_params(void *vpr_base, size_t vpr_size)
{
	int retval = -EINVAL;

	retval = invoke_smc(TE_SMC_PROGRAM_VPR,
				(uintptr_t)vpr_base, vpr_size);

	if (retval != 0) {
		pr_err("%s: smc failed, base 0x%p size %zx, err (0x%x)\n",
			__func__, vpr_base, vpr_size, retval);
		return -EINVAL;
	}
	return 0;
}

int tegra_set_vpr_params(void *vpr_base, size_t vpr_size)
{
	int ret;

	mutex_lock(&vpr_lock);
	ret = _tegra_set_vpr_params(vpr_base, vpr_size);
	mutex_unlock(&vpr_lock);
	return ret;
}
EXPORT_SYMBOL(tegra_set_vpr_params);

void tegra_register_idle_unidle(int (*do_idle)(void *),
				int (*do_unidle)(void *),
				void *data)
{
	int i;

	mutex_lock(&vpr_lock);
	for (i = 0; i < NUM_MODULES_IDLE_VPR_RESIZE; i++) {
		if (do_idle == vpr_user_module[i].do_idle) {
			vpr_user_module[i].data = data;
			goto unlock;
		}
	}

	for (i = 0; i < NUM_MODULES_IDLE_VPR_RESIZE; i++) {
		if (!vpr_user_module[i].do_idle) {
			vpr_user_module[i].do_idle = do_idle;
			vpr_user_module[i].do_unidle = do_unidle;
			vpr_user_module[i].data = data;
			break;
		}
	}
unlock:
	mutex_unlock(&vpr_lock);

	if (i != NUM_MODULES_IDLE_VPR_RESIZE)
		return;

	pr_err("%pF,%pF failed to register to be called before vpr resize!!\n",
		do_idle, do_unidle);
}
EXPORT_SYMBOL(tegra_register_idle_unidle);

void tegra_unregister_idle_unidle(int (*do_idle)(void *))
{
	int i;

	mutex_lock(&vpr_lock);
	for (i = 0; i < NUM_MODULES_IDLE_VPR_RESIZE; i++) {
		if (vpr_user_module[i].do_idle == do_idle) {
			vpr_user_module[i].do_idle = NULL;
			vpr_user_module[i].do_unidle = NULL;
			vpr_user_module[i].data = NULL;
			break;
		}
	}
	mutex_unlock(&vpr_lock);
}
EXPORT_SYMBOL(tegra_unregister_idle_unidle);
