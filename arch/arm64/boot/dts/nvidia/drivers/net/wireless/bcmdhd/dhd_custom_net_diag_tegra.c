/*
 * drivers/net/wireless/bcmdhd/dhd_custom_net_diag_tegra.c
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

#include "dhd_custom_net_diag_tegra.h"
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
#include "dhd_custom_net_bw_est_tegra.h"
#endif
#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
#include "dhd_custom_sysfs_tegra.h"
#endif
#include "dhd_custom_sysfs_tegra_scan.h"
#include <wl_cfg80211.h>

static int tegra_net_diag_debug;

static tegra_net_diag_data_t tegra_net_diag_data;

/* network diagnostics work */

static unsigned int tegra_net_diag_work_rate;

static struct workqueue_struct *tegra_net_diag_wq;

static void tegra_net_diag_work_func(struct work_struct *work);

static DECLARE_DELAYED_WORK(tegra_net_diag_work, tegra_net_diag_work_func);

static void tegra_net_diag_work_func(struct work_struct *work)
{
#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
	extern struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct net_device *net
		= dhd_custom_sysfs_tegra_histogram_stat_netdev;
#else
	struct net_device *net
		= NULL;
#endif
	struct wireless_dev *wdev = NULL;
	struct bcm_cfg80211 *cfg = NULL;
	int err;

	/* check input */
	if (!net)
		return;

	/* Abort ongoing scan and acquire scan lock */
	if (wifi_scan_sem_lock() < 0)
		return;

	wdev = net->ieee80211_ptr;
	if (wdev != NULL && wdev->wiphy != NULL) {
		cfg = wiphy_priv(wdev->wiphy);
		wl_cfg80211_cancel_scan(cfg);
	}

	/* get assoc mode (802.11 mode a/b/g/n/ac) */
	{
		char cap[32];
		err = wldev_get_mode(net, cap);
		if (err == 0)
			strcpy(tegra_net_diag_data.assoc_mode, cap);
		else
			strcpy(tegra_net_diag_data.assoc_mode, "");
	}

	/* get assoc channel / channel width */
	{
		extern chanspec_t
			wl_chspec_driver_to_host(chanspec_t chanspec);
		s32 chsp;
		chanspec_t chanspec;
		uint16 channel;
		uint16 band;
		uint16 bw;
		u32 sb;
		err = wldev_iovar_getint(net, "chanspec", &chsp);
		chanspec = wl_chspec_driver_to_host(chsp);
		channel = chanspec & WL_CHANSPEC_CHAN_MASK;
		band = chanspec & WL_CHANSPEC_BAND_MASK;
		switch (chanspec & WL_CHANSPEC_BW_MASK) {
		case WL_CHANSPEC_BW_80:
			bw = 80;
			sb = chanspec & WL_CHANSPEC_CTL_SB_MASK;
			if (sb == WL_CHANSPEC_CTL_SB_LL) {
				channel -= (CH_10MHZ_APART + CH_20MHZ_APART);
			} else if (sb == WL_CHANSPEC_CTL_SB_LU) {
				channel -= CH_10MHZ_APART;
			} else if (sb == WL_CHANSPEC_CTL_SB_UL) {
				channel += CH_10MHZ_APART;
			} else {
				/* WL_CHANSPEC_CTL_SB_UU */
				channel += (CH_10MHZ_APART + CH_20MHZ_APART);
			}
			break;
		case WL_CHANSPEC_BW_40:
			bw = 40;
			if (CHSPEC_SB_UPPER(chanspec)) {
				channel += CH_10MHZ_APART;
			} else {
				channel -= CH_10MHZ_APART;
			}
			break;
		case WL_CHANSPEC_BW_20:
			bw = 20;
			break;
		default:
			bw = 20;
			break;
		}
		if (err == 0) {
			tegra_net_diag_data.assoc_channel = channel;
			tegra_net_diag_data.assoc_channel_width = bw;
		} else {
			tegra_net_diag_data.assoc_channel = 0;
			tegra_net_diag_data.assoc_channel_width = 0;
		}
	}

	/* get rssi */
	{
		scb_val_t scb_val;
		memset(&scb_val, 0, sizeof(scb_val));
		err = wldev_get_rssi(net, &scb_val);
		if (err == 0)
			tegra_net_diag_data.rssi = scb_val.val;
		else
			tegra_net_diag_data.rssi = 0;
	}

	/* get bandwidth estimate */
	tegra_net_diag_data.bw_est = 0;
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
	tegra_net_diag_data.bw_est = tegra_net_bw_est_get_value();
#endif
	wifi_scan_sem_unlock();

}

static void tegra_net_diag_work_start(void)
{
	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	/* check work queue */
	if (!tegra_net_diag_wq) {
		TEGRA_NET_DIAG_DEBUG("%s: !workqueue\n", __func__);
		return;
	}

	/* schedule network diagnostics work */
	queue_delayed_work(tegra_net_diag_wq,
		&tegra_net_diag_work,
		msecs_to_jiffies(0));
}

static void tegra_net_diag_work_stop(void)
{
	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	/* check work queue */
	if (!tegra_net_diag_wq) {
		TEGRA_NET_DIAG_DEBUG("%s: !workqueue\n", __func__);
		return;
	}

	/* cancel network diagnostics work */
	tegra_net_diag_work_rate = 0;
	cancel_delayed_work_sync(&tegra_net_diag_work);
}

void tegra_net_diag_get_value(tegra_net_diag_data_t *net_diag_data)
{
	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	memset(&tegra_net_diag_data, 0, sizeof(tegra_net_diag_data_t));

	/* start network diagnostics work */
	tegra_net_diag_work_start();

	/* wait for network diagnostics work to finish */
	flush_delayed_work(&tegra_net_diag_work);

	memcpy(net_diag_data, &tegra_net_diag_data,
			sizeof(tegra_net_diag_data_t));

	/* save a copy of the diag data in bcmdhd tcpdump */
#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
	tcpdump_pkt_save('D', "", __func__, __LINE__,
		(void *) &tegra_net_diag_data, sizeof(tegra_net_diag_data), 0);
#endif
}


/* network diagnostics sysfs */

static ssize_t
tegra_net_diag_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *s = buf;

	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	tegra_net_diag_get_value(&tegra_net_diag_data);

	/* show assoc mode (802.11 mode a/b/g/n/ac) */
	snprintf(s, PAGE_SIZE - (s - buf),
		"802.11 Mode: %s\n",
		tegra_net_diag_data.assoc_mode);
	s += strlen(s);

	/* show assoc channel */
	snprintf(s, PAGE_SIZE - (s - buf),
		"Channel: %d\n",
		tegra_net_diag_data.assoc_channel);
	s += strlen(s);

	/* show assoc channel width */
	snprintf(s, PAGE_SIZE - (s - buf),
		"Channel width: %d\n",
		tegra_net_diag_data.assoc_channel_width);
	s += strlen(s);

	/* show rssi */
	snprintf(s, PAGE_SIZE - (s - buf),
		"RSSI: %d\n",
		tegra_net_diag_data.rssi);
	s += strlen(s);

	/* show network bandwidth estimate (bps) */
	snprintf(s, PAGE_SIZE - (s - buf),
		"Bandwidth: %ld\n",
		tegra_net_diag_data.bw_est);
	s += strlen(s);

	return strlen(buf);
}

static ssize_t
tegra_net_diag_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int uint;

	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	if (strncmp(buf, "debug", 5) == 0) {
		tegra_net_diag_debug = !tegra_net_diag_debug;
	} else if (strncmp(buf, "enable", 6) == 0) {
		TEGRA_NET_DIAG_DEBUG("%s: starting diag delayed work...\n",
			__func__);
		tegra_net_diag_work_start();
	} else if (strncmp(buf, "disable", 7) == 0) {
		TEGRA_NET_DIAG_DEBUG("%s: stopping diag delayed work...\n",
			__func__);
		tegra_net_diag_work_stop();
	} else if (strncmp(buf, "rate ", 5) == 0) {
		err = kstrtouint(buf + 5, 0, &uint);
		if (err < 0) {
			TEGRA_NET_DIAG_DEBUG("%s: invalid diag rate"
				" (ms)\n",
				__func__);
			return count;
		}
		TEGRA_NET_DIAG_DEBUG("%s: set diag rate (ms) %u\n",
			__func__, uint);
		tegra_net_diag_work_rate = uint;
	} else {
		TEGRA_NET_DIAG_DEBUG("%s: unknown command\n", __func__);
	}
	return count;
}

static DEVICE_ATTR(net_diag, S_IRUGO | S_IWUSR,
	tegra_net_diag_show,
	tegra_net_diag_store);

/* network diagnostics initialization */

int tegra_net_diag_register(struct device *dev)
{
	int err;

	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	/* create sysfs */
	err = sysfs_create_file(&dev->kobj, &dev_attr_net_diag.attr);
	if (err) {
		TEGRA_NET_DIAG_DEBUG("%s: failed to create sysfs file"
			" - %d\n",
			__func__, err);
		return err;
	}

	/* create work queue */
	tegra_net_diag_wq = create_workqueue("tegra_net_diag_wq");
	if (!tegra_net_diag_wq) {
		TEGRA_NET_DIAG_DEBUG("%s: failed to create workqueue\n",
			__func__);
	}

	/* start work */
#if 0
	tegra_net_diag_work_start();
#endif

	return 0;
}

void tegra_net_diag_unregister(struct device *dev)
{
	TEGRA_NET_DIAG_DEBUG("%s\n", __func__);

	/* stop work */
	tegra_net_diag_work_stop();

	/* destroy workqueue */
	if (tegra_net_diag_wq) {
		destroy_workqueue(tegra_net_diag_wq);
		tegra_net_diag_wq = NULL;
	}

}
