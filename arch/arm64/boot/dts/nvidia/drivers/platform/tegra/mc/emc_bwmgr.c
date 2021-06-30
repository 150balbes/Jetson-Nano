/**
 * Copyright (c) 2015-2019, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/version.h>
#include <soc/tegra/chip-id.h>

#define CREATE_TRACE_POINTS
#include <trace/events/bwmgr.h>

u8 bwmgr_dram_efficiency;
u8 bwmgr_dram_num_channels;
/* flag to determine supported memory and channel configuration */
u8 bwmgr_dram_config_supported;
u32 *bwmgr_dram_iso_eff_table;
u32 *bwmgr_dram_noniso_eff_table;
u32 *bwmgr_max_nvdis_bw_reqd;
u32 *bwmgr_max_vi_bw_reqd;
int *bwmgr_slope;
u32 *bwmgr_vi_bw_reqd_offset;
int bwmgr_iso_bw_percentage;
enum bwmgr_dram_types bwmgr_dram_type;
int emc_to_dram_freq_factor;

#define IS_HANDLE_VALID(x) ((x >= bwmgr.bwmgr_client) && \
		(x < bwmgr.bwmgr_client + TEGRA_BWMGR_CLIENT_COUNT))

struct tegra_bwmgr_client {
	unsigned long bw;
	unsigned long iso_bw;
	unsigned long cap;
	unsigned long iso_cap;
	unsigned long floor;
	int refcount;
};

/* TODO: Manage client state in a dynamic list */
static struct {
	struct tegra_bwmgr_client bwmgr_client[TEGRA_BWMGR_CLIENT_COUNT];
	struct mutex lock;
	unsigned long emc_min_rate;
	unsigned long emc_max_rate;
	struct clk *emc_clk;
	struct task_struct *task;
	bool status;
	struct bwmgr_ops *ops;
	bool override;
} bwmgr;

static struct dram_refresh_alrt {
	unsigned long cur_state;
	u32 max_cooling_state;
	struct thermal_cooling_device *cdev;
} *galert_data;

static bool clk_update_disabled;

static struct {
	unsigned long bw;
	unsigned long iso_bw;
	unsigned long non_iso_cap;
	unsigned long iso_cap;
	unsigned long floor;
	unsigned long total_bw_aftr_eff;
	unsigned long iso_bw_aftr_eff;
	unsigned long calc_freq;
	unsigned long req_freq;
} debug_info;

static void bwmgr_debugfs_init(void);

/* keep in sync with tegra_bwmgr_client_id */
static const char * const tegra_bwmgr_client_names[] = {
	"cpu_cluster_0",
	"cpu_cluster_1",
	"cpu_cluster_2",
	"cpu_cluster_3",
	"disp_0",
	"disp_1",
	"disp_2",
	"disp1_la_emc",
	"disp2_la_emc",
	"usbd",
	"xhci",
	"sdmmc1",
	"sdmmc2",
	"sdmmc3",
	"sdmmc4",
	"mon",
	"gpu",
	"msenc",
	"nvenc1",
	"nvjpg",
	"nvdec",
	"nvdec1",
	"tsec",
	"tsecb",
	"vi",
	"ispa",
	"ispb",
	"camera",
	"camera_non_iso",
	"camrtc",
	"isomgr",
	"thermal",
	"vic",
	"adsp",
	"adma",
	"pcie",
	"pcie_1",
	"pcie_2",
	"pcie_3",
	"pcie_4",
	"pcie_5",
	"bbc_0",
	"eqos",
	"se0",
	"se1",
	"se2",
	"se3",
	"se4",
	"pmqos",
	"nvpmodel",
	"debug",
	"nvdla0",
	"nvdla1",
	"null",
};

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_TRACEPOINTS)
static const char *bwmgr_req_to_name(enum tegra_bwmgr_request_type req)
{
	/* Keep in sync with enum tegra_bwmgr_request_type. */
	switch (req) {
	case TEGRA_BWMGR_SET_EMC_FLOOR:
		return "TEGRA_BWMGR_SET_EMC_FLOOR";
	case TEGRA_BWMGR_SET_EMC_CAP:
		return "TEGRA_BWMGR_SET_EMC_CAP";
	case TEGRA_BWMGR_SET_EMC_ISO_CAP:
		return "TEGRA_BWMGR_SET_EMC_ISO_CAP";
	case TEGRA_BWMGR_SET_EMC_SHARED_BW:
		return "TEGRA_BWMGR_SET_EMC_SHARED_BW";
	case TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO:
		return "TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO";
	default:
		return "INVALID_REQUEST";
	}
}
#endif /* defined(CONFIG_DEBUG_FS) && defined(CONFIG_TRACEPOINTS) */

static inline bool bwmgr_lock(void)
{
	/* disallow rentrance, avoid deadlock */
	if (unlikely(bwmgr.task == current)) {
		pr_err("bwmgr: %s deadlock ?\n", __func__);
		dump_stack();
		return false;
	}
	mutex_lock(&bwmgr.lock);
	bwmgr.task = current;
	return true;
}

static inline bool bwmgr_unlock(void)
{
	/* detect mismatched calls */
	if (unlikely(bwmgr.task != current)) {
		pr_err("bwmgr: %s mismatch ?\n", __func__);
		dump_stack();
		return false;
	}
	bwmgr.task = NULL;
	mutex_unlock(&bwmgr.lock);
	return true;
}

/* call with bwmgr lock held except during init*/
static void purge_client(struct tegra_bwmgr_client *handle)
{
	handle->bw = 0;
	handle->iso_bw = 0;
	handle->cap = bwmgr.emc_max_rate;
	handle->iso_cap = bwmgr.emc_max_rate;
	handle->floor = 0;
	handle->refcount = 0;
}

static unsigned long tegra_bwmgr_apply_efficiency(
		unsigned long total_bw, unsigned long iso_bw,
		unsigned long max_rate, u64 usage_flags,
		unsigned long *iso_bw_min, unsigned long iso_bw_nvdis,
		unsigned long iso_bw_vi)
{
	return bwmgr.ops->bwmgr_apply_efficiency(total_bw, iso_bw,
			max_rate, usage_flags, iso_bw_min,
			iso_bw_nvdis, iso_bw_vi);
}

/* call with bwmgr lock held */
static int bwmgr_update_clk(void)
{
	int i;
	unsigned long bw = 0;
	unsigned long iso_bw = 0; // iso_bw_guarantee
	unsigned long iso_bw_nvdis = 0; //DISP0 + DISP1 + DISP2
	unsigned long iso_bw_vi = 0; //CAMERA
	unsigned long iso_bw_other_clients = 0; //Other ISO clients
	unsigned long non_iso_cap = bwmgr.emc_max_rate;
	unsigned long iso_cap = bwmgr.emc_max_rate;
	unsigned long floor = 0;
	unsigned long iso_bw_min;
	u64 iso_client_flags = 0;
	int ret = 0;

	/* sizeof(iso_client_flags) */
	BUILD_BUG_ON(TEGRA_BWMGR_CLIENT_COUNT > 64);
	/* check that lock is held */
	if (unlikely(bwmgr.task != current)) {
		pr_err("bwmgr: %s called without lock\n", __func__);
		return -EINVAL;
	}

	if (bwmgr.override)
		return 0;

	for (i = 0; i < TEGRA_BWMGR_CLIENT_COUNT; i++) {
		bw += bwmgr.bwmgr_client[i].bw;
		bw = min(bw, bwmgr.emc_max_rate);

		if (bwmgr.bwmgr_client[i].iso_bw > 0) {
			iso_client_flags |= BIT(i);

			if ((i == TEGRA_BWMGR_CLIENT_DISP0) ||
					(i == TEGRA_BWMGR_CLIENT_DISP1) ||
					(i == TEGRA_BWMGR_CLIENT_DISP2)) {
				iso_bw_nvdis += bwmgr.bwmgr_client[i].iso_bw;
				iso_bw_nvdis = min(iso_bw_nvdis,
							bwmgr.emc_max_rate);
			} else if (i == TEGRA_BWMGR_CLIENT_CAMERA) {
				iso_bw_vi += bwmgr.bwmgr_client[i].iso_bw;
				iso_bw_vi = min(iso_bw_vi, bwmgr.emc_max_rate);
			} else {
				iso_bw_other_clients +=
						bwmgr.bwmgr_client[i].iso_bw;
				iso_bw_other_clients = min(iso_bw_other_clients,
							bwmgr.emc_max_rate);
			}

			iso_bw = iso_bw_nvdis + iso_bw_vi +
						iso_bw_other_clients;
			iso_bw = min(iso_bw, bwmgr.emc_max_rate);
		}

		non_iso_cap = min(non_iso_cap, bwmgr.bwmgr_client[i].cap);
		iso_cap = min(iso_cap, bwmgr.bwmgr_client[i].iso_cap);
		floor = max(floor, bwmgr.bwmgr_client[i].floor);
	}
	debug_info.bw = bw;
	debug_info.iso_bw = iso_bw;
	debug_info.floor = floor;
	debug_info.iso_cap = iso_cap;
	debug_info.non_iso_cap = non_iso_cap;
	bw += iso_bw;
	bw = tegra_bwmgr_apply_efficiency(
			bw, iso_bw, bwmgr.emc_max_rate,
			iso_client_flags, &iso_bw_min,
			iso_bw_nvdis, iso_bw_vi);
	debug_info.total_bw_aftr_eff = bw;
	debug_info.iso_bw_aftr_eff = iso_bw_min;
	floor = min(floor, bwmgr.emc_max_rate);
	bw = max(bw, floor);
	bw = min(bw, min(iso_cap, max(non_iso_cap, iso_bw_min)));
	debug_info.calc_freq = bw;
	debug_info.req_freq = bw;

	ret = clk_set_rate(bwmgr.emc_clk, bw);
	if (ret)
		pr_err
		("bwmgr: clk_set_rate failed for freq %lu Hz with errno %d\n",
				bw, ret);

	return ret;
}

struct tegra_bwmgr_client *tegra_bwmgr_register(
		enum tegra_bwmgr_client_id client)
{
	if (!bwmgr_dram_config_supported) {
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		return ERR_PTR(-EINVAL);
	}

	if ((client >= TEGRA_BWMGR_CLIENT_COUNT) || (client < 0)) {
		pr_err("bwmgr: invalid client id %d tried to register",
				client);
		WARN_ON(true);
		return ERR_PTR(-EINVAL);
	}

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__, tegra_bwmgr_client_names[client]);
		return ERR_PTR(-EINVAL);
	}

	(bwmgr.bwmgr_client + client)->refcount++;

	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__, tegra_bwmgr_client_names[client]);
		return ERR_PTR(-EINVAL);
	}
	return (bwmgr.bwmgr_client + client);
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_register);

void tegra_bwmgr_unregister(struct tegra_bwmgr_client *handle)
{
	if (!IS_HANDLE_VALID(handle)) {
		WARN_ON(true);
		return;
	}

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return;
	}
	handle->refcount--;

	if (handle->refcount <= 0) {
		if (handle->refcount < 0) {
			pr_err("bwmgr: Mismatched unregister call, client %ld\n",
				handle - bwmgr.bwmgr_client);
			WARN_ON(true);
		}
		purge_client(handle);
	}

	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return;
	}
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_unregister);

u8 tegra_bwmgr_get_dram_num_channels(void)
{
	return bwmgr_dram_num_channels;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_dram_num_channels);

unsigned long tegra_bwmgr_get_max_emc_rate(void)
{
	return bwmgr.emc_max_rate;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_max_emc_rate);

/* Returns the ratio between dram and emc freq based on the type of dram */
int bwmgr_get_emc_to_dram_freq_factor(void)
{
	return emc_to_dram_freq_factor;
}
EXPORT_SYMBOL_GPL(bwmgr_get_emc_to_dram_freq_factor);

/* Returns the actual emc frequency calculated using the dram
 * frequency and emc_to_dram conversion factor
 */
unsigned long tegra_bwmgr_get_core_emc_rate(void)
{
	return (unsigned long)(tegra_bwmgr_get_emc_rate() /
		bwmgr_get_emc_to_dram_freq_factor());
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_core_emc_rate);

unsigned long tegra_bwmgr_round_rate(unsigned long bw)
{
	if (bwmgr.emc_clk)
		return clk_round_rate(bwmgr.emc_clk, bw);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_round_rate);

int tegra_bwmgr_set_emc(struct tegra_bwmgr_client *handle, unsigned long val,
		enum tegra_bwmgr_request_type req)
{
	int ret = 0;
	bool update_clk = false;

	if (!bwmgr.emc_clk)
		return 0;

	if (!bwmgr.status)
		return 0;

	if (!bwmgr_dram_config_supported) {
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		return -EINVAL;
	}

	if (!IS_HANDLE_VALID(handle)) {
		pr_err("bwmgr: client sent bad handle %p\n",
				handle);
		WARN_ON(true);
		return -EINVAL;
	}

	if (req >= TEGRA_BWMGR_SET_EMC_REQ_COUNT) {
		pr_err("bwmgr: client %ld sent bad request type %d\n",
				handle - bwmgr.bwmgr_client, req);
		WARN_ON(true);
		return -EINVAL;
	}

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return -EINVAL;
	}

#ifdef CONFIG_TRACEPOINTS
	trace_tegra_bwmgr_set_emc(
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client],
			val, bwmgr_req_to_name(req));
#endif /* CONFIG_TRACEPOINTS */

	switch (req) {
	case TEGRA_BWMGR_SET_EMC_FLOOR:
		if (handle->floor != val) {
			handle->floor = val;
			update_clk = true;
		}
		break;

	case TEGRA_BWMGR_SET_EMC_CAP:
		if (val == 0)
			val = bwmgr.emc_max_rate;

		if (handle->cap != val) {
			handle->cap = val;
			update_clk = true;
		}
		break;

	case TEGRA_BWMGR_SET_EMC_ISO_CAP:
		if (val == 0)
			val = bwmgr.emc_max_rate;

		if (handle->iso_cap != val) {
			handle->iso_cap = val;
			update_clk = true;
		}
		break;

	case TEGRA_BWMGR_SET_EMC_SHARED_BW:
		if (handle->bw != val) {
			handle->bw = val;
			update_clk = true;
		}
		break;

	case TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO:
		if (handle->iso_bw != val) {
			handle->iso_bw = val;
			update_clk = true;
		}
		break;

	default:
		WARN_ON(true);
		if (!bwmgr_unlock()) {
			pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
			return -EINVAL;
		}
		return -EINVAL;
	}

	if (update_clk && !clk_update_disabled)
		ret = bwmgr_update_clk();

	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_set_emc);

int tegra_bwmgr_get_client_info(struct tegra_bwmgr_client *handle,
		unsigned long *out_val,
		enum tegra_bwmgr_request_type req)
{
	if (!bwmgr.emc_clk)
		return 0;

	if (!bwmgr.status)
		return 0;

	if (!bwmgr_dram_config_supported) {
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		return -EINVAL;
	}

	if (!IS_HANDLE_VALID(handle)) {
		pr_err("bwmgr: client sent bad handle %p\n",
				handle);
		WARN_ON(true);
		return -EINVAL;
	}

	if (req >= TEGRA_BWMGR_SET_EMC_REQ_COUNT) {
		pr_err("bwmgr: client %ld sent bad request type %d\n",
				handle - bwmgr.bwmgr_client, req);
		WARN_ON(true);
		return -EINVAL;
	}

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return -EINVAL;
	}

	switch (req) {
	case TEGRA_BWMGR_SET_EMC_FLOOR:
		*out_val = handle->floor;
		break;

	case TEGRA_BWMGR_SET_EMC_CAP:
		*out_val = handle->cap;
		break;

	case TEGRA_BWMGR_SET_EMC_ISO_CAP:
		*out_val = handle->iso_cap;
		break;

	case TEGRA_BWMGR_SET_EMC_SHARED_BW:
		*out_val = handle->bw;
		break;

	case TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO:
		*out_val = handle->iso_bw;
		break;

	default:
		WARN_ON(true);
		if (!bwmgr_unlock()) {
			pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
			return -EINVAL;
		}
		return -EINVAL;
	}

	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed for client %s\n",
			__func__,
			tegra_bwmgr_client_names[handle - bwmgr.bwmgr_client]);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_client_info);

int tegra_bwmgr_notifier_register(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;

	if (bwmgr.emc_clk)
		return clk_notifier_register(bwmgr.emc_clk, nb);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_notifier_register);

int tegra_bwmgr_notifier_unregister(struct notifier_block *nb)
{
	if (!nb)
		return -EINVAL;

	if (bwmgr.emc_clk)
		return clk_notifier_unregister(bwmgr.emc_clk, nb);
	else
		return -ENODEV;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_notifier_unregister);

unsigned long tegra_bwmgr_get_emc_rate(void)
{
	if (bwmgr.emc_clk)
		return clk_get_rate(bwmgr.emc_clk);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_emc_rate);

/* bwmgr_get_lowest_iso_emc_freq
 * bwmgr_apply_efficiency function will use this api to calculate
 * the lowest emc frequency that satisfies the requests of ISO clients.
 *
 * A return value of 0 indicates that the requested
 * iso bandwidth cannot be supported.
 */
unsigned long bwmgr_get_lowest_iso_emc_freq(long iso_bw,
				long iso_bw_nvdis, long iso_bw_vi)
{
	return bwmgr.ops->get_best_iso_freq(iso_bw, iso_bw_nvdis, iso_bw_vi);
}
EXPORT_SYMBOL_GPL(bwmgr_get_lowest_iso_emc_freq);

/* tegra_bwmgr_get_max_iso_bw
 * This function returns the max iso bw.
 * This is applicable from t19x onwards, where max_iso is different
 * based on clients requesting.
 * This should not be called on pre t19x and returns 0 in those cases
 */
u32 tegra_bwmgr_get_max_iso_bw(enum tegra_iso_client client)
{
	if (bwmgr.ops->get_max_iso_bw)
		return bwmgr.ops->get_max_iso_bw(client);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_bwmgr_get_max_iso_bw);

int bwmgr_iso_bw_percentage_max(void)
{
	return bwmgr_iso_bw_percentage;
}
EXPORT_SYMBOL_GPL(bwmgr_iso_bw_percentage_max);

unsigned long bwmgr_freq_to_bw(unsigned long freq)
{
	return bwmgr.ops->freq_to_bw(freq);
}
EXPORT_SYMBOL_GPL(bwmgr_freq_to_bw);

unsigned long bwmgr_bw_to_freq(unsigned long bw)
{
	return bwmgr.ops->bw_to_freq(bw);
}
EXPORT_SYMBOL_GPL(bwmgr_bw_to_freq);

u32 bwmgr_dvfs_latency(u32 ufreq)
{
	return bwmgr.ops->dvfs_latency(ufreq);
}
EXPORT_SYMBOL_GPL(bwmgr_dvfs_latency);

/* Get maximum throttle state supported by bwmgr cooling device. */
static int dram_ref_alert_cdev_max_state(struct thermal_cooling_device *tcd,
			unsigned long *state)
{
	struct dram_refresh_alrt *alert_data = tcd->devdata;

	if (!alert_data)
		return -EINVAL;

	*state = (unsigned long)alert_data->max_cooling_state;
	return 0;
}

/* Get current throttle state of the bwmgr cooling device. */
static int dram_ref_alert_cdev_cur_state(struct thermal_cooling_device *tcd,
			unsigned long *state)
{
	struct dram_refresh_alrt *alert_data = tcd->devdata;

	if (!alert_data)
		return -EINVAL;

	*state = alert_data->cur_state;
	return 0;
}

static int tegra_bwmgr_update_efficiency(unsigned long cur_state,
				unsigned long prev_state)
{
	int ret = 0;

#ifdef CONFIG_TRACEPOINTS
	trace_tegra_bwmgr_update_efficiency(
			cur_state, prev_state);
#endif /* CONFIG_TRACEPOINTS */

	/* At this point, the thermal framework has indicated that
	 * the trip point has been crossed for AO-therm zone. However,
	 * this does not guarantee dram refresh rate has been changed.
	 * talk to bpmp at this point to find out if dram refresh rate
	 * has changed or not. This will be implemented once the bpmp
	 * IPC's are ready.For now update efficiency.
	 */
	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed\n", __func__);
		return -EINVAL;
	}

	if (bwmgr.override) {
		bwmgr_unlock();
		return 0;
	}

	if (bwmgr.ops->update_efficiency)
		bwmgr.ops->update_efficiency(cur_state);

	if (!clk_update_disabled)
		ret = bwmgr_update_clk();

	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed.\n", __func__);
		return -EINVAL;
	}

	return ret;
}

/* Set current throttle state of the bwmgr cooling device. */
static int dram_ref_alert_cdev_set_state(struct thermal_cooling_device *tcd,
				unsigned long state)
{
	int ret = 0;
	unsigned long prev_state;
	struct dram_refresh_alrt *alert_data = tcd->devdata;

	if (!alert_data)
		return -EINVAL;

	prev_state = alert_data->cur_state;
	alert_data->cur_state = state;

	ret = tegra_bwmgr_update_efficiency(state, prev_state);

	return ret;
}

/*
 * Cooling device operations.
 */
static struct thermal_cooling_device_ops dram_ref_alert_cdev_ops = {
	.get_max_state = dram_ref_alert_cdev_max_state,
	.get_cur_state = dram_ref_alert_cdev_cur_state,
	.set_cur_state = dram_ref_alert_cdev_set_state,
};

int __init bwmgr_init(void)
{
	int i;
	struct device_node *dn;
	long round_rate;
	struct clk *emc_master_clk;

	mutex_init(&bwmgr.lock);

	if (tegra_get_chip_id() == TEGRA210)
		bwmgr.ops = bwmgr_eff_init_t21x();
	else if (tegra_get_chip_id() == TEGRA186)
		bwmgr.ops = bwmgr_eff_init_t18x();
	else if (tegra_get_chip_id() == TEGRA194)
		bwmgr.ops = bwmgr_eff_init_t19x();
	else
		/*
		 * Fall back to t19x if we are running on a new chip.
		 */
		bwmgr.ops = bwmgr_eff_init_t19x();

	dn = of_find_compatible_node(NULL, NULL, "nvidia,bwmgr");
	if (dn == NULL) {
		pr_err("bwmgr: dt node not found.\n");
		return -ENODEV;
	}

	bwmgr.emc_clk = of_clk_get(dn, 0);
	if (IS_ERR_OR_NULL(bwmgr.emc_clk)) {
		pr_err("bwmgr: couldn't find emc clock.\n");
		bwmgr.emc_clk = NULL;
		WARN_ON(true);
		return -ENODEV;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	clk_prepare_enable(bwmgr.emc_clk);
#endif

	emc_master_clk = bwmgr.emc_clk;
	if (of_property_read_bool(dn, "nvidia,bwmgr-use-shared-master"))
		emc_master_clk = clk_get_parent(emc_master_clk);

	round_rate = clk_round_rate(emc_master_clk, 0);
	if (round_rate < 0) {
		bwmgr.emc_min_rate = 0;
		pr_err("bwmgr: couldn't get emc clock min rate.\n");
	} else
		bwmgr.emc_min_rate = (unsigned long)round_rate;

	/* Use LONG_MAX as downstream functions treats rate arg as signed */
	round_rate = clk_round_rate(emc_master_clk, LONG_MAX);
	if (round_rate < 0) {
		bwmgr.emc_max_rate = 0;
		pr_err("bwmgr: couldn't get emc clock max rate.\n");
	} else
		bwmgr.emc_max_rate = (unsigned long)round_rate;

	/* On some pre-si platforms max rate is acquired via DT */
	if (tegra_platform_is_sim() || tegra_platform_is_fpga()) {
		if (of_property_read_u64(dn, "max_rate_Hz",
					(u64 *) &round_rate) == 0) {
			pr_err("bwmgr: using max rate from device tree.\n");
			bwmgr.emc_max_rate = round_rate;
		}
	}

	for (i = 0; i < TEGRA_BWMGR_CLIENT_COUNT; i++)
		purge_client(bwmgr.bwmgr_client + i);

	bwmgr_debugfs_init();
	pmqos_bwmgr_init();

	/* Check status property is okay or not. */
	if (of_device_is_available(dn))
		bwmgr.status = true;
	else
		bwmgr.status = false;

	return 0;
}
subsys_initcall_sync(bwmgr_init);

void __exit bwmgr_exit(void)
{
	int i;

	for (i = 0; i < TEGRA_BWMGR_CLIENT_COUNT; i++)
		purge_client(bwmgr.bwmgr_client + i);

	bwmgr.emc_clk = NULL;
	mutex_destroy(&bwmgr.lock);

	if (galert_data)
		thermal_cooling_device_unregister(galert_data->cdev);

	kfree(galert_data);
}
module_exit(bwmgr_exit);

static int __init bwmgr_cooling_init(void)
{
	struct device_node *dn;
	char *cdev_type;
	const char *str;
	int ret = 0;

	galert_data = kzalloc(sizeof(struct dram_refresh_alrt),
					GFP_KERNEL);
	if (!galert_data)
		return -ENOMEM;

	dn = of_find_compatible_node(NULL, NULL, "nvidia,bwmgr");
	if (dn == NULL) {
		pr_err("bwmgr: dt node not found.\n");
		ret = -ENODEV;
		goto error;
	}

	if (of_property_read_string(dn, "cdev-type", &str) == 0) {
		cdev_type = (char *)str;
	} else {
		pr_info("bwmgr: missing cdev-type property\n");
		goto error;
	}

	if (of_property_read_u32(dn, "cooling-max-state",
				&(galert_data->max_cooling_state))) {
		pr_err("bwmgr: missing max_cooling_state property\n");
		ret = -EINVAL;
		goto error;
	}

	galert_data->cur_state = 0;
	galert_data->cdev = thermal_of_cooling_device_register(dn, cdev_type,
				galert_data, &dram_ref_alert_cdev_ops);

	if (IS_ERR(galert_data->cdev)) {
		ret = PTR_ERR(galert_data->cdev);
		pr_err("bwmgr: failed to register to thermal framework\n");
		goto error;
	}

	return 0;

error:
	kfree(galert_data);
	return ret;
}
late_initcall(bwmgr_cooling_init);
/* thermal_init is fs_init, make bwmgr_cooling_init late_init as it
 * depends on thermal_init
 */

#ifdef CONFIG_DEBUG_FS
static struct tegra_bwmgr_client *bwmgr_debugfs_client_handle;
static struct dentry *debugfs_dir;
static struct dentry *debugfs_node_floor;
static struct dentry *debugfs_node_cap;
static struct dentry *debugfs_node_iso_cap;
static struct dentry *debugfs_node_bw;
static struct dentry *debugfs_node_iso_bw;
static struct dentry *debugfs_node_emc_rate;
static struct dentry *debugfs_node_emc_min;
static struct dentry *debugfs_node_emc_max;
static struct dentry *debugfs_node_core_emc_rate;
static struct dentry *debugfs_node_clients_info;
static struct dentry *debugfs_node_dram_channels;

static int bwmgr_debugfs_emc_rate_set(void *data, u64 val)
{
	int ret = 0;

	if (!bwmgr_lock())
		return -EPERM;

	if (val == 0) {
		bwmgr.override = false;
		bwmgr_update_clk();
	} else if (bwmgr.emc_clk) {
		bwmgr.override = true;
		ret = clk_set_rate(bwmgr.emc_clk, val);
	}

	if (!bwmgr_unlock())
		return -EPERM;

	return ret;
}

static int bwmgr_debugfs_emc_rate_get(void *data, u64 *val)
{
	*val = tegra_bwmgr_get_emc_rate();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_emc_rate, bwmgr_debugfs_emc_rate_get,
		bwmgr_debugfs_emc_rate_set, "%llu\n");

static int bwmgr_debugfs_core_emc_rate_get(void *data, u64 *val)
{
	*val = tegra_bwmgr_get_core_emc_rate();
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_core_emc_rate,
	bwmgr_debugfs_core_emc_rate_get, NULL, "%llu\n");

static int bwmgr_debugfs_dram_channels_get(void *data, u64 *val)
{
	*val = bwmgr_dram_num_channels;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_dram_channels,
	bwmgr_debugfs_dram_channels_get, NULL, "%llu\n");

static int bwmgr_debugfs_floor_set(void *data, u64 val)
{
	tegra_bwmgr_set_emc(bwmgr_debugfs_client_handle, val,
			TEGRA_BWMGR_SET_EMC_FLOOR);
	return 0;
}

static int bwmgr_debugfs_floor_get(void *data, u64 *val)
{
	*val = bwmgr_debugfs_client_handle->floor;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_floor, bwmgr_debugfs_floor_get,
		bwmgr_debugfs_floor_set, "%llu\n");

static int bwmgr_debugfs_cap_set(void *data, u64 val)
{
	tegra_bwmgr_set_emc(bwmgr_debugfs_client_handle, val,
			TEGRA_BWMGR_SET_EMC_CAP);
	return 0;
}

static int bwmgr_debugfs_cap_get(void *data, u64 *val)
{
	*val = bwmgr_debugfs_client_handle->cap;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_cap, bwmgr_debugfs_cap_get,
		bwmgr_debugfs_cap_set, "%llu\n");

static int bwmgr_debugfs_iso_cap_set(void *data, u64 val)
{
	tegra_bwmgr_set_emc(bwmgr_debugfs_client_handle, val,
			TEGRA_BWMGR_SET_EMC_ISO_CAP);
	return 0;
}

static int bwmgr_debugfs_iso_cap_get(void *data, u64 *val)
{
	*val = bwmgr_debugfs_client_handle->iso_cap;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_iso_cap, bwmgr_debugfs_iso_cap_get,
		bwmgr_debugfs_iso_cap_set, "%llu\n");

static int bwmgr_debugfs_bw_set(void *data, u64 val)
{
	tegra_bwmgr_set_emc(bwmgr_debugfs_client_handle, val,
			TEGRA_BWMGR_SET_EMC_SHARED_BW);
	return 0;
}

static int bwmgr_debugfs_bw_get(void *data, u64 *val)
{
	*val = bwmgr_debugfs_client_handle->bw;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_bw, bwmgr_debugfs_bw_get,
		bwmgr_debugfs_bw_set, "%llu\n");

static int bwmgr_debugfs_iso_bw_set(void *data, u64 val)
{
	tegra_bwmgr_set_emc(bwmgr_debugfs_client_handle, val,
			TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO);
	return 0;
}

static int bwmgr_debugfs_iso_bw_get(void *data, u64 *val)
{
	*val = bwmgr_debugfs_client_handle->iso_bw;
	return 0;
}

static int bwmgr_clients_info_show(struct seq_file *s, void *data)
{
	int i;

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed\n", __func__);
		return -EINVAL;
	}
	seq_printf(s, "%15s%15s%15s%15s%15s%15s (Khz)\n", "Client",
			"Floor", "SharedBw", "SharedIsoBw", "Cap",
			"IsoCap");
	for (i = 0; i < TEGRA_BWMGR_CLIENT_COUNT; i++) {
		seq_printf(s, "%14s%s%15lu%15lu%15lu%15lu%15lu\n",
				tegra_bwmgr_client_names[i],
				bwmgr.bwmgr_client[i].refcount ? "*" : " ",
				bwmgr.bwmgr_client[i].floor / 1000,
				bwmgr.bwmgr_client[i].bw / 1000,
				bwmgr.bwmgr_client[i].iso_bw / 1000,
				bwmgr.bwmgr_client[i].cap / 1000,
				bwmgr.bwmgr_client[i].iso_cap / 1000);
	}
	seq_printf(s, "Total BW requested                              : %lu (Khz)\n",
				 debug_info.bw / 1000);
	seq_printf(s, "Total ISO_BW requested                          : %lu (Khz)\n",
				 debug_info.iso_bw / 1000);
	seq_printf(s, "Effective floor request                         : %lu (Khz)\n",
				 debug_info.floor / 1000);
	seq_printf(s, "Effective NON_ISO_CAP                           : %lu (Khz)\n",
				 debug_info.non_iso_cap / 1000);
	seq_printf(s, "Effective ISO_CAP                               : %lu (Khz)\n",
				 debug_info.iso_cap / 1000);
	seq_printf(s, "Total BW + ISO_BW                               : %lu (Khz)\n",
				 (debug_info.bw + debug_info.iso_bw) / 1000);
	seq_printf(s, "Total BW+ISO_BW after applying efficieny numbers: %lu (Khz)\n",
				 debug_info.total_bw_aftr_eff / 1000);
	seq_printf(s, "Total ISO_BW after applying efficiency          : %lu (Khz)\n",
				 debug_info.iso_bw_aftr_eff / 1000);
	seq_printf(s, "EMC calculated rate                             : %lu (Khz)\n",
				 debug_info.calc_freq / 1000);
	seq_printf(s, "EMC requested(rounded) rate                     : %lu (Khz)\n",
				 debug_info.req_freq / 1000);
	seq_printf(s, "EMC current rate                                : %lu (Khz)\n",
				 tegra_bwmgr_get_emc_rate() / 1000);
	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed\n", __func__);
		return -EINVAL;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_debugfs_iso_bw, bwmgr_debugfs_iso_bw_get,
		bwmgr_debugfs_iso_bw_set, "%llu\n");

static int bwmgr_clients_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, bwmgr_clients_info_show, inode->i_private);
}

static const struct file_operations fops_bwmgr_clients_info = {
	.open = bwmgr_clients_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void bwmgr_debugfs_init(void)
{
	bwmgr_debugfs_client_handle =
		tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_DEBUG);

	if (IS_ERR_OR_NULL(bwmgr_debugfs_client_handle)) {
		bwmgr_debugfs_client_handle = NULL;
		pr_err("bwmgr: could not register bwmgr debugfs client\n");
		return;
	}

	debugfs_dir = debugfs_create_dir("tegra_bwmgr", NULL);
	if (debugfs_dir) {
		debugfs_create_bool(
			"clk_update_disabled", S_IRWXU, debugfs_dir,
			&clk_update_disabled);
		debugfs_node_emc_min = debugfs_create_u64(
			"emc_min_rate", S_IRUSR, debugfs_dir,
			(u64 *) &bwmgr.emc_min_rate);
		debugfs_node_emc_max = debugfs_create_u64(
			"emc_max_rate", S_IRUSR, debugfs_dir,
			(u64 *) &bwmgr.emc_max_rate);
		debugfs_node_core_emc_rate = debugfs_create_file(
			"core_emc_rate", S_IRUSR, debugfs_dir, NULL,
			 &fops_debugfs_core_emc_rate);
		debugfs_node_emc_rate = debugfs_create_file
			("emc_rate", S_IRUSR, debugfs_dir, NULL,
			 &fops_debugfs_emc_rate);
		debugfs_node_floor = debugfs_create_file
			("debug_client_floor", S_IRWXU, debugfs_dir, NULL,
			 &fops_debugfs_floor);
		debugfs_node_cap = debugfs_create_file
			("debug_client_cap", S_IRWXU, debugfs_dir, NULL,
			 &fops_debugfs_cap);
		debugfs_node_iso_cap = debugfs_create_file
			("debug_client_iso_cap", S_IRWXU, debugfs_dir, NULL,
			 &fops_debugfs_iso_cap);
		debugfs_node_bw = debugfs_create_file
			("debug_client_bw", S_IRWXU, debugfs_dir, NULL,
			 &fops_debugfs_bw);
		debugfs_node_iso_bw = debugfs_create_file
			("debug_client_iso_bw", S_IRWXU, debugfs_dir, NULL,
			 &fops_debugfs_iso_bw);
		debugfs_node_clients_info = debugfs_create_file
			("bwmgr_clients_info", S_IRUGO, debugfs_dir, NULL,
			 &fops_bwmgr_clients_info);
		debugfs_node_dram_channels = debugfs_create_file(
			"num_dram_channels", S_IRUSR, debugfs_dir, NULL,
			 &fops_debugfs_dram_channels);
	} else
		pr_err("bwmgr: error creating bwmgr debugfs dir.\n");

	if (!bwmgr_lock()) {
		pr_err("bwmgr: %s failed\n", __func__);
		return;
	}
	debug_info.non_iso_cap = bwmgr.emc_max_rate;
	debug_info.iso_cap = bwmgr.emc_max_rate;
	if (!bwmgr_unlock()) {
		pr_err("bwmgr: %s failed\n", __func__);
		return;
	}
}

#else
static void bwmgr_debugfs_init(void) {};
#endif /* CONFIG_DEBUG_FS */
