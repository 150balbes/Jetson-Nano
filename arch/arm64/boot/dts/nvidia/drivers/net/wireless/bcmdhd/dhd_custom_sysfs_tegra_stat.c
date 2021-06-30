/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra_stat.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2014-2019 NVIDIA Corporation. All rights reserved.
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

#include "dhd_custom_sysfs_tegra.h"
#include "dhd_custom_sysfs_tegra_stat.h"
#include "dhd_custom_net_diag_tegra.h"

/* Flags */
int wifi_stat_debug;
int aggr_not_assoc_err_set;

int eapol_message_1_retry = -1;
int eapol_message_2_retry = -1;
int eapol_message_3_retry = -1;
int eapol_message_4_retry = -1;

struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
struct tegra_sysfs_histogram_stat bcmdhd_stat;
struct tegra_sysfs_histogram_stat bcmdhd_stat_saved;
struct timespec dhdstats_ts;
unsigned short cur_drv_state = DRV_STATE_SUSPEND;
unsigned short cur_pm_state = DRV_PM_MODE_INIT;

static void
stat_work_func(struct work_struct *work);

static unsigned int stat_delay_ms;
static unsigned int stat_rate_ms = 60 * 1000;
/* overall bcmdhd stat rate in msec */
static unsigned int bcmdhd_stat_rate_ms = 15 * 60 * 1000;

static DECLARE_DELAYED_WORK(stat_work, stat_work_func);

int bcmdhd_resume_trigger;
int resume_done;

void tegra_sysfs_histogram_driver_stat_suspend(void)
{
	/* No-op */
}

void tegra_sysfs_histogram_driver_stat_resume(void)
{
	/* No-op */
}

void
tegra_sysfs_histogram_stat_set_channel(int channel)
{
	int i, n = -1;

	/* stop collecting channel stat(s) */
	if (channel < 0) {
		return;
	}

	/* allocate array index for collecting channel stat(s) */
	bcmdhd_stat.gen_stat.channel_stat = NULL;

	for (i = 0; i < sizeof(bcmdhd_stat.gen_stat.channel_stat_list) /
		sizeof(bcmdhd_stat.gen_stat.channel_stat_list[0]); i++) {
		if ((n == -1) && !bcmdhd_stat.gen_stat.channel_stat_list[i].channel) {
			n = i;
			continue;
		}
		if (bcmdhd_stat.gen_stat.channel_stat_list[i].channel == channel) {
			n = i;
			break;
		}
	}
	if (n != -1) {
		bcmdhd_stat.gen_stat.channel_stat = &bcmdhd_stat.gen_stat.channel_stat_list[n];
		bcmdhd_stat.gen_stat.channel_stat->channel = channel;
	}
}

void
tegra_sysfs_histogram_stat_work_run(unsigned int ms)
{
	stat_delay_ms = ms;
}

void
tegra_sysfs_histogram_stat_work_start(void)
{
//	pr_info("%s\n", __func__);
	if (!resume_done) {
		TEGRA_SYSFS_HISTOGRAM_DRV_STATE_UPDATE(DRV_STATE_ACTIVE);
		resume_done = 1;
	}
	if (stat_rate_ms > 0)
		schedule_delayed_work(&stat_work,
			msecs_to_jiffies(stat_rate_ms));
}

void
tegra_sysfs_histogram_stat_work_stop(void)
{
//	pr_info("%s\n", __func__);
	cancel_delayed_work_sync(&stat_work);

	if (resume_done) {
		TEGRA_SYSFS_HISTOGRAM_DRV_STATE_UPDATE(DRV_STATE_SUSPEND);
		resume_done = 0;
	}
}

static void
stat_work_func(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	char *netif = net ? net->name : "";
	wl_cnt_t *cnt;
	int i;
	struct timespec now;
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
	tegra_net_diag_data_t net_diag_data;
	int bwValue;
	int roundOffBw;
#endif
	get_monotonic_boottime(&now);

	UNUSED_PARAMETER(dwork);

//	pr_info("%s\n", __func__);

	/* create stat request */
	cnt = kmalloc(sizeof(wl_cnt_t), GFP_KERNEL);
	if (!cnt) {
//		pr_err("%s: kmalloc(wl_cnt_t) failed\n", __func__);
		goto fail;
	}

	/* send stat request */
	if (wldev_iovar_getbuf(net, "counters", NULL, 0,
		(void *) cnt, sizeof(wl_cnt_t), NULL) != BCME_OK) {
//		pr_err("%s: wldev_iovar_getbuf() failed\n", __func__);
		kfree(cnt);
		goto fail;
	}

	/* update statistics */
	TEGRA_SYSFS_HISTOGRAM_STAT_SET(fw_tx_err, cnt->txerror);
	TEGRA_SYSFS_HISTOGRAM_STAT_SET(fw_tx_retry, cnt->txretrans);
	TEGRA_SYSFS_HISTOGRAM_STAT_SET(fw_rx_err, cnt->rxerror);

	/* log stat request */
	for (i = 0; i < sizeof(wl_cnt_t); i += 64) {
		tcpdump_pkt_save('a' + i / 64,
			netif,
			__func__,
			__LINE__,
			((unsigned char *) cnt) + i,
			(i + 64) <= sizeof(wl_cnt_t)
				? 64 : sizeof(wl_cnt_t) - i,
			0);
	}

	/* Update the overall bcmdhd stats */
	if (MSEC(now) - MSEC(bcmdhd_stat.time) > bcmdhd_stat_rate_ms) {
		bcmdhd_stat.time = now;
		TEGRA_SYSFS_HISTOGRAM_AGGR_DRV_STATE(now);
		TEGRA_SYSFS_HISTOGRAM_AGGR_PM_STATE(now);
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
		memset(&net_diag_data, 0, sizeof(tegra_net_diag_data_t));
		tegra_net_diag_get_value(&net_diag_data);
		bcmdhd_stat.driver_stat.cur_bw_est = net_diag_data.bw_est;
		bwValue = net_diag_data.bw_est;
		roundOffBw = 0;
		if (bwValue < 1024) {
			roundOffBw = (bwValue * 100) / 100;
		} else if (bwValue < 1048576) {
			roundOffBw = ((bwValue/1024) * 100) / 100;
		} else if (bwValue < 1073741824) {
			roundOffBw = ((bwValue/1048576) * 100) / 100;
		} else {
			roundOffBw = ((bwValue/1073741824) * 100) / 100;
		}
		if (roundOffBw <= 10) {
			bcmdhd_stat.driver_stat.bw_est_level_0++;
		} else if (roundOffBw <= 50) {
			bcmdhd_stat.driver_stat.bw_est_level_1++;
		} else if (roundOffBw <= 100) {
			bcmdhd_stat.driver_stat.bw_est_level_2++;
		} else if (roundOffBw <= 300) {
			bcmdhd_stat.driver_stat.bw_est_level_3++;
		} else if (roundOffBw > 300) {
			bcmdhd_stat.driver_stat.bw_est_level_4++;
		}
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
		memcpy(&bcmdhd_stat.fw_stat, cnt, sizeof(wl_cnt_t));
		tcpdump_pkt_save(TCPDUMP_TAG_STAT,
			netif,
			__func__,
			__LINE__,
			(unsigned char *) &bcmdhd_stat,
			sizeof(struct tegra_sysfs_histogram_stat),
			0);
	}

	kfree(cnt);

	/* schedule next stat */
fail:
	if (stat_delay_ms) {
		stat_delay_ms = 0;
		msleep(stat_delay_ms);
		schedule_delayed_work(&stat_work, 0);
		return;
	}
	schedule_delayed_work(&stat_work,
		msecs_to_jiffies(stat_rate_ms));

}

ssize_t
tegra_sysfs_histogram_stat_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
#if 0
	static int i;

//	pr_info("%s\n", __func__);

	if (!i) {
		i++;
		strcpy(buf, "dummy stat!");
		return strlen(buf);
	} else {
		i = 0;
		return 0;
	}
#else
	struct timespec now;
	int i, n, comma;
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
	tegra_net_diag_data_t net_diag_data;
#endif
	get_monotonic_boottime(&now);
	TEGRA_SYSFS_HISTOGRAM_AGGR_DRV_STATE(now);
	TEGRA_SYSFS_HISTOGRAM_AGGR_PM_STATE(now);
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
	memset(&net_diag_data, 0, sizeof(tegra_net_diag_data_t));
	tegra_net_diag_get_value(&net_diag_data);
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
	/* print statistics head */
	n = 0;
	snprintf(buf + n, PAGE_SIZE - n,
		"{\n"
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
		"\"version\": 3.2,\n"
#else
		"\"version\": 3,\n"
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
		"\"start_time\": %lu,\n"
		"\"end_time\": %lu,\n"
		"\"wifi_on_success\": %lu,\n"
		"\"wifi_on_retry\": %lu,\n"
		"\"wifi_on_fail\": %lu,\n"
		"\"connect_success\": %lu,\n"
		"\"connect_fail\": %lu,\n"
		"\"connect_fail_reason_15\": %lu,\n"
		"\"connect_fail_set_ssid\": %lu,\n"
		"\"disconnect_rssi_low\": %lu,\n"
		"\"disconnect_rssi_high\": %lu,\n"
		"\"fw_tx_err\": %lu,\n"
		"\"fw_tx_retry\": %lu,\n"
		"\"fw_rx_err\": %lu,\n"
		"\"hang\": %lu,\n"
		"\"ago_start\": %lu,\n"
		"\"connect_on_2g_channel\": %lu,\n"
		"\"connect_on_5g_channel\": %lu,\n",
		MSEC(dhdstats_ts),
		MSEC(now),
		PRINT_DIFF(gen_stat.wifi_on_success),
		PRINT_DIFF(gen_stat.wifi_on_retry),
		PRINT_DIFF(gen_stat.wifi_on_fail),
		PRINT_DIFF(gen_stat.connect_success),
		PRINT_DIFF(gen_stat.connect_fail),
		PRINT_DIFF(gen_stat.connect_fail_reason_15),
		PRINT_DIFF(gen_stat.connect_fail_set_ssid),
		PRINT_DIFF(gen_stat.disconnect_rssi_low),
		PRINT_DIFF(gen_stat.disconnect_rssi_high),
		PRINT_DIFF(gen_stat.fw_tx_err),
		PRINT_DIFF(gen_stat.fw_tx_retry),
		PRINT_DIFF(gen_stat.fw_rx_err),
		PRINT_DIFF(gen_stat.hang),
		PRINT_DIFF(gen_stat.ago_start),
		PRINT_DIFF(gen_stat.connect_on_2g_channel),
		PRINT_DIFF(gen_stat.connect_on_5g_channel));

	/* print statistics */
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"channel_stat\": [");
	comma = 0;
	for (i = 0; i < sizeof(bcmdhd_stat.gen_stat.channel_stat_list) /
		sizeof(bcmdhd_stat.gen_stat.channel_stat_list[0]); i++) {
		if (!bcmdhd_stat.gen_stat.channel_stat_list[i].channel)
			continue;
		if (comma) {
			n = strlen(buf);
			snprintf(buf + n, PAGE_SIZE - n,
				",");
		}
		n = strlen(buf);
		snprintf(buf + n, PAGE_SIZE - n,
			"\n"
			"  [%d,%lu,%lu,%lu]",
			bcmdhd_stat.gen_stat.channel_stat_list[i].channel,
			PRINT_DIFF(gen_stat.channel_stat_list[i].connect_count),
			PRINT_DIFF(gen_stat.channel_stat_list[i].rssi_low),
			PRINT_DIFF(gen_stat.channel_stat_list[i].rssi_high));
		comma = 1;
	}
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\n"
		"],\n");

	/* print statistics tail */
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"sdio_tx_err\": %lu,\n"
		"\"rssi\": %d,\n"
		"\"rssi_low\": %lu,\n"
		"\"rssi_high\": %lu,\n"
		"\"rssi_level_0\": %lu,\n"
		"\"rssi_level_1\": %lu,\n"
		"\"rssi_level_2\": %lu,\n"
		"\"rssi_level_3\": %lu,\n"
		"\"rssi_level_4\": %lu,\n"
		"\"eapol_message_1_retry\": %lu,\n"
		"\"eapol_message_2_retry\": %lu,\n"
		"\"eapol_message_3_retry\": %lu,\n"
		"\"eapol_message_4_retry\": %lu,\n",
		PRINT_DIFF(gen_stat.sdio_tx_err),
		bcmdhd_stat.gen_stat.rssi,
		PRINT_DIFF(gen_stat.rssi_low),
		PRINT_DIFF(gen_stat.rssi_high),
		PRINT_DIFF(gen_stat.rssi_level_0),
		PRINT_DIFF(gen_stat.rssi_level_1),
		PRINT_DIFF(gen_stat.rssi_level_2),
		PRINT_DIFF(gen_stat.rssi_level_3),
		PRINT_DIFF(gen_stat.rssi_level_4),
		PRINT_DIFF(gen_stat.eapol_message_1_retry),
		PRINT_DIFF(gen_stat.eapol_message_2_retry),
		PRINT_DIFF(gen_stat.eapol_message_3_retry),
		PRINT_DIFF(gen_stat.eapol_message_4_retry));

	/* print framework stats */
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"ccode_sig_fail\": [");
	for (i = 0; i < SIG_FAIL_REASONS; i++) {
		n = strlen(buf);
		snprintf(buf + n, PAGE_SIZE - n,
			"%d",
			PRINT_DIFF(gen_stat.ccode_sig_fail[i]));
		if (SIG_FAIL_REASONS - (i+1)) {
			n = strlen(buf);
			snprintf(buf + n, PAGE_SIZE - n,
				",");
		}
	}
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"],\n");

	/* print driver statistics */
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"aggr_num_sta_scans\": %lu,\n"
		"\"aggr_num_p2p_scans\": %lu,\n"
		"\"aggr_time_drv_suspend\": %lu,\n"
		"\"aggr_time_drv_active\": %lu,\n"
		"\"aggr_num_rssi_ioctl\": %lu,\n"
		"\"aggr_num_ioctl\": %lu,\n",
		PRINT_DIFF(driver_stat.aggr_num_sta_scans),
		PRINT_DIFF(driver_stat.aggr_num_p2p_scans),
		PRINT_DIFF(driver_stat.aggr_time_drv_suspend),
		PRINT_DIFF(driver_stat.aggr_time_drv_active),
		PRINT_DIFF(driver_stat.aggr_num_rssi_ioctl),
		PRINT_DIFF(driver_stat.aggr_num_ioctl));

	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"aggr_PM_time\": [");
	for (i = 0; i < NUM_PM_MODES; i++) {
		n = strlen(buf);
		snprintf(buf + n, PAGE_SIZE - n,
			"%lu",
			PRINT_DIFF(driver_stat.aggr_PM_time[i]));
		if (NUM_PM_MODES - (i+1)) {
			n = strlen(buf);
			snprintf(buf + n, PAGE_SIZE - n,
				",");
		}
	}
	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"],\n");

	n = strlen(buf);
	snprintf(buf + n, PAGE_SIZE - n,
		"\"aggr_num_wowlan\": %lu,\n"
		"\"aggr_num_wowlan_unicast\": %lu,\n"
		"\"aggr_num_wowlan_multicast\": %lu,\n"
		"\"aggr_num_wowlan_broadcast\": %lu,\n"
		"\"aggr_bus_credit_unavail\": %lu,\n"
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
		"\"cur_bw_est\": %lu,\n"
		"\"bw_est_level_0\": %lu,\n"
		"\"bw_est_level_1\": %lu,\n"
		"\"bw_est_level_2\": %lu,\n"
		"\"bw_est_level_3\": %lu,\n"
		"\"bw_est_level_4\": %lu,\n"
		"\"cur_mode\": \"%s\",\n"
		"\"cur_channel_width\": %d,\n"
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
		"\"aggr_not_assoc_err\": %lu,\n"
		"\"cur_country_code\": \"%s\"\n"
		"}\n",
		PRINT_DIFF(driver_stat.aggr_num_wowlan),
		PRINT_DIFF(driver_stat.aggr_num_wowlan_unicast),
		PRINT_DIFF(driver_stat.aggr_num_wowlan_multicast),
		PRINT_DIFF(driver_stat.aggr_num_wowlan_broadcast),
		PRINT_DIFF(driver_stat.aggr_bus_credit_unavail),
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
		net_diag_data.bw_est,
		PRINT_DIFF(driver_stat.bw_est_level_0),
		PRINT_DIFF(driver_stat.bw_est_level_1),
		PRINT_DIFF(driver_stat.bw_est_level_2),
		PRINT_DIFF(driver_stat.bw_est_level_3),
		PRINT_DIFF(driver_stat.bw_est_level_4),
		net_diag_data.assoc_mode,
		net_diag_data.assoc_channel_width,
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
		PRINT_DIFF(driver_stat.aggr_not_assoc_err),
		bcmdhd_stat.fw_stat.cur_country_code);

	/* update statistics end time */
	dhdstats_ts = now;
	bcmdhd_stat_saved = bcmdhd_stat;
	/* success */
	return strlen(buf);

#endif
}

extern unsigned long dpc_sleep_cnt;
extern atomic_t dpc_bound;
extern atomic_t dpc_frame_time;

ssize_t
tegra_sysfs_histogram_stat_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int uint;
	struct net_device *net = dhd_custom_sysfs_tegra_histogram_stat_netdev;
	char *netif = net ? net->name : "";

//	pr_info("%s\n", __func__);

	if (strncmp(buf, "debug", 5) == 0) {
		wifi_stat_debug = !wifi_stat_debug;
	} else if (strncmp(buf, "enable", 6) == 0) {
		pr_info("%s: starting stat delayed work...\n", __func__);
		tegra_sysfs_histogram_stat_work_start();
	} else if (strncmp(buf, "disable", 7) == 0) {
		pr_info("%s: stopping stat delayed work...\n", __func__);
		tegra_sysfs_histogram_stat_work_stop();
	} else if (strncmp(buf, "rate ", 5) == 0) {
		err = kstrtouint(buf + 5, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid stat rate (ms)\n", __func__);
			return count;
		}
		pr_info("%s: set stat rate (ms) %u\n", __func__, uint);
		stat_rate_ms = uint;
	} else if (strncmp(buf, "bcmdhd_stat_rate ", 17) == 0) {
		err = kstrtouint(buf + 17, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid bcmdhd_stat rate (ms)\n", __func__);
			return count;
		}
		pr_info("%s: set bcmdhd_stat rate (ms) %u\n", __func__, uint);
		bcmdhd_stat_rate_ms = uint;
	} else if (strncmp(buf, "framework_stat ", 15) == 0) {
		if (strncmp(buf + 15, "sig ", 4) == 0) {
			err = kstrtouint(buf + 19, 0, &uint);
			if (err < 0 ||
				(uint < 0 || uint > (SIG_FAIL_REASONS - 1))) {
				pr_err("%s: invalid framework_stat\n", __func__);
				return count;
			}
			TEGRA_SYSFS_HISTOGRAM_STAT_INC(ccode_sig_fail[uint]);
		}
		tcpdump_pkt_save('E',
			netif,
			__func__,
			__LINE__,
			buf + 15,
			strlen(buf+15),
			0);
	} else if (strncmp(buf, "dpc_sleep_cnt", 13) == 0) {
		pr_err("dpc_sleep_cnt: %ld\n", dpc_sleep_cnt);
	} else if (strncmp(buf, "dpc_frame_time ", 15) == 0) {
		err = kstrtouint(buf + 15, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid dpc_frame_time (ms)\n", __func__);
			return count;
		} else if (uint < 0 || uint < atomic_read(&dpc_bound)) {
			pr_err("%s: invalid dpc_frame_time (ms)\n", __func__);
		    return count;
		}
		pr_info("%s: set dpc_frame_time (ms) %u\n", __func__, uint);
		atomic_set(&dpc_frame_time, uint);
	} else if (strncmp(buf, "dpc_bound ", 10) == 0) {
		err = kstrtouint(buf + 10, 0, &uint);
		if (err < 0) {
			pr_err("%s: invalid dpc_bound (ms)\n", __func__);
			return count;
		} else if (uint < 0 || uint > atomic_read(&dpc_frame_time)) {
			pr_err("%s: invalid dpc_bound (ms)\n", __func__);
			return count;
		}
		pr_info("%s: set dpc_bound (ms) %u\n", __func__, uint);
		atomic_set(&dpc_bound, uint);
	} else {
		pr_err("%s: unknown command\n", __func__);
	}

	return count;
}
