/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp.h
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVER_VIDEO_TEGRA_DC_NVDISP_H
#define __DRIVER_VIDEO_TEGRA_DC_NVDISP_H

#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>

extern struct mutex tegra_nvdisp_lock;
extern struct clk *hubclk;

#define NVDISP_TEGRA_POLL_TIMEOUT_MS	50

#define NVDISP_HEAD_ENABLE_DISABLE_TIMEOUT_HZ	(2 * HZ)

struct nvdisp_request_wq {
	wait_queue_head_t	wq;
	atomic_t		nr_pending;
	int			timeout_per_entry;
};

struct nvdisp_common_imp_data {
	struct list_head imp_settings_queue;

	struct nvdisp_request_wq common_channel_reservation_wq;
	struct nvdisp_request_wq common_channel_promotion_wq;

	struct mrq_emc_dvfs_latency_response emc_dvfs_table;

	struct tegra_dc_ext_imp_mc_caps mc_caps;
	bool reg_caps_initialized;

	/*
	 * If this flag is set, the driver will assume that the frequencies of
	 * the following clocks have been locked by an external client:
	 * - EMC
	 * - hubclk
	 * - dispclk
	 */
	bool lock_mode_enabled;
};

int tegra_nvdisp_assign_win(struct tegra_dc *dc, unsigned idx);
int tegra_nvdisp_detach_win(struct tegra_dc *dc, unsigned idx);
int tegra_nvdisp_get_degamma_config(struct tegra_dc *dc,
	struct tegra_dc_win *win);

int tegra_nvdisp_set_win_csc(struct tegra_dc_win *win,
			struct tegra_dc_nvdisp_win_csc *nvdisp_win_csc);

void tegra_nvdisp_set_common_channel_pending(struct tegra_dc *dc);
struct tegra_nvdisp_imp_settings *tegra_nvdisp_get_current_imp_settings(void);
void tegra_nvdisp_program_imp_settings(struct tegra_dc *dc);
void _tegra_nvdisp_bandwidth_unregister(void);
int tegra_nvdisp_program_bandwidth(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk, bool before_win_update);
int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk);
void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc);
void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc);
u32 tegra_nvdisp_get_max_pending_bw(struct tegra_dc *dc);
void tegra_nvdisp_get_max_bw_cfg(struct nvdisp_bandwidth_config *max_cfg);

int __attribute__((weak)) tegra_nvdisp_set_control_t19x(struct tegra_dc *dc);
#endif
