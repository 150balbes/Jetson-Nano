/*
 * drivers/net/wireless/bcmdhd/dhd_custom_net_bw_est_tegra.h
 *
 * NVIDIA Tegra Network Bandwidth Estimator for BCMDHD driver
 *
 * Copyright (C) 2015 NVIDIA Corporation. All rights reserved.
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

#ifndef _dhd_custom_net_bw_est_tegra_h_
#define _dhd_custom_net_bw_est_tegra_h_

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/atomic.h>
#include <linux/random.h>
#include "bcmutils.h"
#include "wlioctl.h"
#include "wldev_common.h"

#define TEGRA_NET_BW_EST_DEBUG(...)\
	do { if (tegra_net_bw_est_debug) pr_err(__VA_ARGS__); } while (0)

int tegra_net_bw_est_register(struct device *dev);

void tegra_net_bw_est_unregister(struct device *dev);

void tegra_net_bw_est_set_src_macaddr(unsigned char *macaddr);

void tegra_net_bw_est_set_dst_macaddr(unsigned char *macaddr);

/* private interface for network diagnostics */

unsigned long tegra_net_bw_est_get_value(void);

#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
extern int tegra_sysfs_wifi_on;
#endif /* CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA */

#endif  /* _dhd_custom_net_bw_est_tegra_h_ */
