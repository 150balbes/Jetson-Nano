/*
 * Copyright (c) 2019-2020 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA_HV_VM_ERR_H_
#define __TEGRA_HV_VM_ERR_H_

#if IS_ENABLED(CONFIG_TEGRA_VM_ERR_HANDLER)
#include <linux/errinfo.h>

struct tegra_hv_vm_err_handlers {
	/* return true, if error needs kernel to enter bad mode and reboot.
	 * return false, if error doesn't need reboot.
	 */
	bool (*fn_self_async)(const struct err_data_t *const err_data);
	bool (*fn_self_sync)(const struct err_data_t *const err_data);
	bool (*fn_peer)(const struct err_data_t *const err_data);
};

struct tegra_hv_config {
	unsigned int guest_id_self;
	unsigned int num_guests;
};

static const char * const tegra_hv_err_reason_desc[] = {
	"Undefined",
	"SMMU Context Bank",
	"SMMU Global",
	"Bridge",
	"Memory Controller",
	"Memory Controller T19X",
	"Central Back Bone",
	"Synchronous exception",
};

int tegra_hv_register_vm_err_hooks(struct tegra_hv_vm_err_handlers *handlers);
void tegra_hv_get_config(struct tegra_hv_config *config);

#else
static inline int tegra_hv_register_vm_err_hooks(
	struct tegra_hv_vm_err_handlers *custom_handlers)
{
	pr_err("Can you please enable CONFIG_TEGRA_VM_ERR_HANDLER?");
	return -EINVAL;
}
#endif

#endif	/* __TEGRA_HV_VM_ERR_H_ */
