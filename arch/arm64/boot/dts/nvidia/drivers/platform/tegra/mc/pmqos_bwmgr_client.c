/**
 * Copyright (c) 2015-2016, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/err.h>
#include <linux/pm_qos.h>


#if defined(CONFIG_TEGRA_BWMGR)

/* Static global handle for PMQoS client */
static struct tegra_bwmgr_client *pmqos_bwmgr_handle;

static int __init register_pmqos_bwmgr_client(void)
{
	int ret = 0;

	pmqos_bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_PMQOS);
	if (IS_ERR_OR_NULL(pmqos_bwmgr_handle)) {
		pr_err("emc bwmgr registration failed for pmqos client\n");
		ret = -ENODEV;
	} else {
		pr_debug("emc bwmgr registration successful for pmqos client\n");
	}
	return ret;
}

static int pmqos_emc_freq_min_notify(struct notifier_block *b,
			unsigned long l, void *v)
{
	int ret = 0;
	unsigned long floor_freq;

	floor_freq = (unsigned long)pm_qos_request(PM_QOS_EMC_FREQ_MIN);
	/* pm_qos_request() returns frequency in Khz */
	floor_freq *= 1000;
	ret = tegra_bwmgr_set_emc(pmqos_bwmgr_handle, floor_freq,
						TEGRA_BWMGR_SET_EMC_FLOOR);
	if (ret)
		return ret;

	return NOTIFY_OK;
}

static struct notifier_block pmqos_emc_freq_min_nb = {
	.notifier_call = pmqos_emc_freq_min_notify,
};

int __init pmqos_bwmgr_init(void)
{
	int ret = 0;

	ret = register_pmqos_bwmgr_client();
	if (ret)
		pr_err("pmqos init in bandwidth manager failed\n");
	else
		ret = pm_qos_add_notifier(PM_QOS_EMC_FREQ_MIN,
						 &pmqos_emc_freq_min_nb);

	return ret;
}
#endif /* CONFIG_TEGRA_BWMGR */
