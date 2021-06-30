/*
 * drivers/net/wireless/bcmdhd/include/dhd_custom_sysfs_tegra_stat.h
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

#ifndef _dhd_custom_sysfs_tegra_stat_h_
#define _dhd_custom_sysfs_tegra_stat_h_

#include <linux/kernel.h>
#include <linux/stat.h>
#include <linux/ktime.h>
#include "bcmutils.h"
#include "wlioctl.h"
#include "wldev_common.h"
#include <linux/module.h>

/* Overall stats collection rate in ms */
#define BCMDHD_STAT_RATE (15*60*1000)
#define NUM_PM_MODES 3

/* Reason codes for ccode sig verification failure.
 * 0 - TXT_FILE_NOT_FOUND
 * 1 - SIG_FILE_NOT_FOUND
 * 2 - FILE_READ_FAILED
 * 3 - SIG_VERIFICATION_FAILED
 */
#define SIG_FAIL_REASONS 4

#define DRV_STATE_ACTIVE 1
#define DRV_STATE_SUSPEND 0
#define DRV_PM_MODE_INIT 0

#define MSEC(x) ((x).tv_sec * MSEC_PER_SEC + (x).tv_nsec / NSEC_PER_MSEC)
#define PRINT_DIFF(x) ((bcmdhd_stat).x - (bcmdhd_stat_saved).x)

#define DRV_STAT_SET(x) x##_set
#define SET_DRV_STAT(x, val) { x##_set = val; }

#define TEGRA_SYSFS_HISTOGRAM_STAT_INC(var)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("wifi stat: %s(%d): increment variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		((bcmdhd_stat.gen_stat).var)++\
	)\

#define TEGRA_SYSFS_HISTOGRAM_STAT_SET(var, value)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("wifi stat: %s(%d): set variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		(bcmdhd_stat.gen_stat).var = value\
	)\

#define TEGRA_SYSFS_HISTOGRAM_STAT_UPDATE_4WHS()\
	do {\
		if (wifi_stat_debug) \
			pr_info("wifi stat: 4WHS counter update \n"); \
		if (eapol_message_3_retry != -1) { \
			if (eapol_message_1_retry > 0) \
				bcmdhd_stat.gen_stat.eapol_message_1_retry += eapol_message_1_retry;\
			if (eapol_message_2_retry > 0) \
				bcmdhd_stat.gen_stat.eapol_message_2_retry += eapol_message_2_retry;\
			if (eapol_message_3_retry > 0) \
				bcmdhd_stat.gen_stat.eapol_message_3_retry += eapol_message_3_retry;\
			if (eapol_message_4_retry > 0) \
				bcmdhd_stat.gen_stat.eapol_message_4_retry += eapol_message_4_retry;\
		} \
		TEGRA_SYSFS_HISTOGRAM_STAT_RESET_4WHS(); \
	} while (0)

#define TEGRA_SYSFS_HISTOGRAM_STAT_RESET_4WHS()\
	do {\
		if (wifi_stat_debug) \
			pr_info("wifi stat:4WHS counter reset \n"); \
		eapol_message_1_retry = -1;\
		eapol_message_2_retry = -1;\
		eapol_message_3_retry = -1;\
		eapol_message_4_retry = -1;\
	} while (0)


#define TEGRA_SYSFS_HISTOGRAM_SCAN_CNT_INC(cfg)\
	do {\
		if (!cfg->p2p_supported || !p2p_scan(cfg))\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_sta_scans);\
		else if (p2p_is_on(cfg) && p2p_scan(cfg))\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_p2p_scans);\
	} while (0)

extern int bcmdhd_resume_trigger;
#define TEGRA_SYSFS_HISTOGRAM_WAKE_CNT_INC(skb)\
	do {\
		if (bcmdhd_resume_trigger) {\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_wowlan);\
			switch (skb->pkt_type) {\
			case PACKET_HOST:\
				TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_wowlan_unicast);\
				break;\
			case PACKET_BROADCAST:\
				TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_wowlan_broadcast);\
				break;\
			case PACKET_MULTICAST:\
				TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(aggr_num_wowlan_multicast);\
				break;\
			default:\
				break;\
			} \
			bcmdhd_resume_trigger = 0;\
		} \
	} while (0)

#define TEGRA_SYSFS_HISTOGRAM_AGGR_DRV_STATE(now)\
	do {\
		struct timespec diff;\
		diff = timespec_sub(now, bcmdhd_stat.driver_stat.cur_drv_state_update_time);\
		if (cur_drv_state)\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC_VAL(aggr_time_drv_active,\
				diff.tv_sec * MSEC_PER_SEC + diff.tv_nsec / NSEC_PER_MSEC);\
		else \
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC_VAL(aggr_time_drv_suspend,\
				diff.tv_sec * MSEC_PER_SEC + diff.tv_nsec / NSEC_PER_MSEC);\
		TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_SET(cur_drv_state_update_time, now);\
	} while (0)

extern unsigned short cur_drv_state;
#define TEGRA_SYSFS_HISTOGRAM_DRV_STATE_UPDATE(newstate)\
	do {\
		struct timespec now;\
		if (newstate != cur_drv_state) {\
			get_monotonic_boottime(&now);\
			TEGRA_SYSFS_HISTOGRAM_AGGR_DRV_STATE(now);\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_SET(cur_drv_state_update_time, now);\
			cur_drv_state = newstate;\
		} \
	} while (0)

#define TEGRA_SYSFS_HISTOGRAM_AGGR_PM_STATE(now)\
	do {\
		struct timespec diff;\
		diff = timespec_sub(now, bcmdhd_stat.driver_stat.cur_pm_state_update_time);\
		if (cur_pm_state < NUM_PM_MODES)\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC_VAL(aggr_PM_time[cur_pm_state],\
				diff.tv_sec * MSEC_PER_SEC + diff.tv_nsec / NSEC_PER_MSEC);\
		TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_SET(cur_pm_state_update_time, now);\
	} while (0)

extern unsigned short cur_pm_state;
#define TEGRA_SYSFS_HISTOGRAM_PM_STATE_UPDATE(newstate)\
	do {\
		struct timespec now;\
		if (newstate != cur_pm_state) {\
			get_monotonic_boottime(&now);\
			TEGRA_SYSFS_HISTOGRAM_AGGR_PM_STATE(now);\
			TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_SET(cur_pm_state_update_time, now);\
			cur_pm_state = newstate;\
		} \
	} while (0)

#define TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC(var)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("driver stat: %s(%d): increment variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		((bcmdhd_stat.driver_stat).var)++\
	)\

#define TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_SET(var, value)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("driver stat: %s(%d): set variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		(bcmdhd_stat.driver_stat).var = (value)\
	)\

#define TEGRA_SYSFS_HISTOGRAM_DRIVER_STAT_INC_VAL(var, value)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("driver stat: %s(%d): inc variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		(bcmdhd_stat.driver_stat).var += (value)\
	)\

#define TEGRA_SYSFS_HISTOGRAM_FW_STAT_SET(var, value)\
	(\
		(bcmdhd_stat.fw_stat).var = value\
	)\

#define TEGRA_SYSFS_HISTOGRAM_FRAMEWORK_STAT_INC(var)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("framework stat: %s(%d): increment variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		((bcmdhd_stat.framework_stat).var)++\
	)\

#define TEGRA_SYSFS_HISTOGRAM_FRAMEWORK_STAT_SET(var, value)\
	(\
		(\
		wifi_stat_debug\
		? pr_info("framework stat: %s(%d): set variable " #var "\n",\
			__func__, __LINE__)\
		: ((void) 0)\
		),\
		(bcmdhd_stat.framework_stat).var = value\
	)\

extern int wifi_stat_debug;

typedef struct tegra_sysfs_stat_generic {
	/* power up failure statistics */
	unsigned long wifi_on_success;
	unsigned long wifi_on_retry;
	unsigned long wifi_on_fail;
	/* connection statistics */
	unsigned long connect_success;
	unsigned long connect_fail;
	unsigned long connect_fail_reason_15;
	unsigned long connect_fail_set_ssid;
	unsigned long disconnect_rssi_low;
	unsigned long disconnect_rssi_high;
	/* firmware statistics */
	unsigned long fw_tx_err;
	unsigned long fw_tx_retry;
	unsigned long fw_rx_err;
	/* hang statistics */
	unsigned long hang;
	/* AGO statistics */
	unsigned long ago_start;
	/* channel statistics */
	unsigned long connect_on_2g_channel;
	unsigned long connect_on_5g_channel;
	struct {
		int channel;
		unsigned long connect_count;
		unsigned long rssi_low;
		unsigned long rssi_high;
	} channel_stat_list[40], *channel_stat;
	/* bus statistics */
	unsigned long sdio_tx_err;
	/* last rssi value */
	int rssi;
	unsigned long rssi_low;
	unsigned long rssi_high;
	/* signal level bucket */
	unsigned long rssi_level_0;
	unsigned long rssi_level_1;
	unsigned long rssi_level_2;
	unsigned long rssi_level_3;
	unsigned long rssi_level_4;
	/* 4WHS retry count */
	unsigned long eapol_message_1_retry;
	unsigned long eapol_message_2_retry;
	unsigned long eapol_message_3_retry;
	unsigned long eapol_message_4_retry;

	int ccode_sig_fail[SIG_FAIL_REASONS];
} tegra_sysfs_stat_generic_t;

typedef struct tegra_sysfs_stat_firmware {
	wl_cnt_t fw_counters;
	char cur_country_code[WLC_CNTRY_BUF_SZ];
} tegra_sysfs_stat_firmware_t;

typedef struct tegra_sysfs_stat_driver {
	struct timespec cur_drv_state_update_time;
	struct timespec cur_pm_state_update_time;
	unsigned long aggr_num_sta_scans;
	unsigned long aggr_num_p2p_scans;
	unsigned long aggr_time_drv_suspend;
	unsigned long aggr_time_drv_active;
	unsigned long aggr_num_rssi_ioctl;
	unsigned long aggr_num_ioctl;
	unsigned long aggr_PM_time[NUM_PM_MODES];
	unsigned long aggr_num_wowlan;
	unsigned long aggr_num_wowlan_unicast;
	unsigned long aggr_num_wowlan_multicast;
	unsigned long aggr_num_wowlan_broadcast;
	unsigned long aggr_bus_credit_unavail;
	/* Per connection-lifetime stats
	 * based on global flag <stat_name>_set.
	 * Use macros DRV_STAT_SET, SET_DRV_STAT
	 * to modify flag.
     */
#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
	unsigned long cur_bw_est; /* bps bw estimator*/
	unsigned long bw_est_level_0;
	unsigned long bw_est_level_1;
	unsigned long bw_est_level_2;
	unsigned long bw_est_level_3;
	unsigned long bw_est_level_4;
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */
	unsigned long aggr_not_assoc_err;
} tegra_sysfs_stat_driver_t;

struct tegra_sysfs_histogram_stat {
	/* Timestamp of record */
	struct timespec time;
	/* Generic stats reported in Shieldstats */
	tegra_sysfs_stat_generic_t gen_stat;
	/* Specific driver stats */
	tegra_sysfs_stat_driver_t driver_stat;
	/* Specific firmware stats */
	tegra_sysfs_stat_firmware_t fw_stat;
};

extern struct tegra_sysfs_histogram_stat bcmdhd_stat;
/* Previous saved snapshot for taking diffs when required */
extern struct tegra_sysfs_histogram_stat bcmdhd_stat_saved;
/* Last time stats node is shown */
extern struct timespec dhdstats_ts;
/* Flags */
extern int aggr_not_assoc_err_set;

extern int eapol_message_1_retry;
extern int eapol_message_2_retry;
extern int eapol_message_3_retry;
extern int eapol_message_4_retry;

#ifdef CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA
extern unsigned long tegra_net_bw_est_get_value(void);
#endif /* CONFIG_BCMDHD_CUSTOM_NET_BW_EST_TEGRA */

void tegra_sysfs_histogram_driver_stat_suspend(void);
void tegra_sysfs_histogram_driver_stat_resume(void);
#endif  /* _dhd_custom_sysfs_tegra_stat_h_ */
