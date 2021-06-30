/*
 * drivers/net/wireless/bcmdhd/dhd_custom_net_diag_tegra.h
 *
 * NVIDIA Tegra Network Diagnostics for BCMDHD driver
 *
 * Copyright (C) 2016 NVIDIA Corporation. All rights reserved.
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

#ifndef _dhd_custom_net_diag_tegra_h_
#define _dhd_custom_net_diag_tegra_h_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include "bcmutils.h"
#include "wlioctl.h"
#include "wldev_common.h"

#define TEGRA_NET_DIAG_DEBUG(...) \
	do { if (tegra_net_diag_debug) pr_err(__VA_ARGS__); } while (0)

typedef struct tegra_net_diag_data {
	char assoc_mode[32];
	int assoc_channel;
	int assoc_channel_width;
	int rssi;
	unsigned long bw_est;
} tegra_net_diag_data_t;

void tegra_net_diag_get_value(tegra_net_diag_data_t *net_diag_data);

int tegra_net_diag_register(struct device *dev);

void tegra_net_diag_unregister(struct device *dev);

#endif  /* _dhd_custom_net_diag_tegra_h_ */
