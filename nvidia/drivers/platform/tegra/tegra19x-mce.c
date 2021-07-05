/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/t194_nvg.h>
#include <linux/tegra-mce.h>

#include <asm/smp_plat.h>
#include <soc/tegra/chip-id.h>
#include "tegra19x-mce.h"

#include "dmce_perfmon.h"

/* Issue a NVG request with data */
static noinline notrace void nvg_send_req_data(uint64_t req, uint64_t data)
{
	asm volatile (
		"msr s3_0_c15_c1_2, %0	\n"
		"msr s3_0_c15_c1_3, %1	\n"
		:: "r" (req), "r" (data));
}

/* Issue a NVG request with no data */
static noinline notrace void nvg_send_req(uint64_t req)
{
	asm volatile ("msr s3_0_c15_c1_2, %0	\n" :: "r" (req));
}

/* Issue a NVG request to read the command response */
static noinline notrace uint64_t nvg_get_response(void)
{
	uint64_t ret;

	asm volatile ("mrs %0, s3_0_c15_c1_3" : "=r" (ret));

	return ret;
}

int tegra19x_mce_enter_cstate(u32 state, u32 wake_time)
{
	/* use PSCI interface instead */
	return 0;
}

int tegra19x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
				    u8 force, u32 wake_mask, bool valid)
{
	nvg_cstate_info_channel_t cstate_info = { 0 };

	/* disable preemption */
	preempt_disable();

	/* update CLUSTER_CSTATE? */
	if (cluster) {
		cstate_info.bits.cluster_state = cluster;
		cstate_info.bits.update_cluster = 1;
	}

	/* update CCPLEX_CSTATE? */
	if (ccplex) {
		cstate_info.bits.cg_cstate = ccplex;
		cstate_info.bits.update_cg = 1;
	}

	/* update SYSTEM_CSTATE? */
	if (system) {
		cstate_info.bits.system_cstate = system;
		cstate_info.bits.update_system = 1;
	}

	/* update wake mask value? */
	if (valid)
		cstate_info.bits.update_wake_mask = 1;

	/* set the wake mask */
	cstate_info.bits.wake_mask = wake_mask;

	/* set the updated cstate info */
	nvg_send_req_data(TEGRA_NVG_CHANNEL_CSTATE_INFO,
				cstate_info.flat);

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_update_crossover_time(u32 type, u32 time)
{
	if ((type != TEGRA_NVG_CHANNEL_CROSSOVER_C6_LOWER_BOUND) &&
	    (type != TEGRA_NVG_CHANNEL_CROSSOVER_CC6_LOWER_BOUND) &&
	    (type != TEGRA_NVG_CHANNEL_CROSSOVER_CG7_LOWER_BOUND)) {
		pr_err("%s: unknown crossover type (%d)\n", __func__, type);
		return -EINVAL;
	}

	/* disable pre-emption*/
	preempt_disable();

	nvg_send_req_data(type, (uint64_t)time);

	/* enable pre-emption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_cstate_stats(u32 state, u64 *stats)
{
	if (!stats)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(TEGRA_NVG_CHANNEL_CSTATE_STAT_QUERY_REQUEST,
				(uint64_t)state);
	nvg_send_req(TEGRA_NVG_CHANNEL_CSTATE_STAT_QUERY_VALUE);
	*stats = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable)
{
	nvg_cc3_control_channel_t cc3_ctrl;

	/* disable preemption */
	preempt_disable();

	/*
	 * If the enable bit is cleared, Auto-CC3 will be disabled by setting
	 * the SW visible frequency request registers for all non
	 * floorswept cores valid independent of StandbyWFI and disabling
	 * the IDLE frequency request register. If set, Auto-CC3
	 * will be enabled by setting the ARM SW visible frequency
	 * request registers for all non floorswept cores to be enabled by
	 * StandbyWFI or the equivalent signal, and always keeping the IDLE
	 * frequency request register enabled.
	 */
	cc3_ctrl.bits.freq_req = ndiv;
	cc3_ctrl.bits.enable = !!enable;

	nvg_send_req_data(TEGRA_NVG_CHANNEL_CC3_CTRL, cc3_ctrl.flat);

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_versions(u32 *major, u32 *minor)
{
	uint64_t version;

	if (!major || !minor)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req(TEGRA_NVG_CHANNEL_VERSION);
	version = nvg_get_response();
	*minor = (u32)version;
	*major = (u32)(version >> 32);

	/* enable preemption */
	preempt_enable();

	return 0;
}

/* Check for valid dda channel id.*/
static int tegra19x_check_dda_channel_id(u32 index) {
	if ((index < TEGRA_NVG_CHANNEL_DDA_SNOC_MCF) ||
		(index > TEGRA_NVG_CHANNEL_DDA_SNOC_CLIENT_REPLENTISH_CTRL)) {
		pr_err("mce: invalid dda channel id: %u\n", index);
		return -EINVAL;
	}
	return 0;
}

int tegra19x_mce_write_dda_ctrl(u32 index, u64 value)
{
	if (tegra19x_check_dda_channel_id(index))
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(index, value);

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_dda_ctrl(u32 index, u64* value)
{
	if (tegra19x_check_dda_channel_id(index))
		return -EINVAL;

	if (!value)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req(index);
	*value = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_l3_cache_ways(u64 *value)
{
	/* disable preemption */
	preempt_disable();

	nvg_send_req(TEGRA_NVG_CHANNEL_CCPLEX_CACHE_CONTROL);
	*value = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_write_l3_cache_ways(u64 data, u64 *value)
{
	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(TEGRA_NVG_CHANNEL_CCPLEX_CACHE_CONTROL, data);
	*value = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_rt_safe_mask(u64 *rt_safe_mask)
{
	if (!rt_safe_mask)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req(TEGRA_NVG_CHANNEL_RT_SAFE_MASK);
	*rt_safe_mask = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_write_rt_safe_mask(u64 rt_safe_mask)
{
	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(TEGRA_NVG_CHANNEL_RT_SAFE_MASK, rt_safe_mask);

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_read_rt_window_us(u64 *rt_window_us)
{
	if (!rt_window_us)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req(TEGRA_NVG_CHANNEL_RT_WINDOW_US);
	*rt_window_us = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_write_rt_window_us(u64 rt_window_us)
{
	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(TEGRA_NVG_CHANNEL_RT_WINDOW_US, rt_window_us);

	/* enable preemption */
	preempt_enable();

	return 0;
}


int tegra19x_mce_read_rt_fwd_progress_us(u64 *rt_fwd_progress_us)
{
	if (!rt_fwd_progress_us)
		return -EINVAL;

	/* disable preemption */
	preempt_disable();

	nvg_send_req(TEGRA_NVG_CHANNEL_RT_FWD_PROGRESS_US);
	*rt_fwd_progress_us = nvg_get_response();

	/* enable preemption */
	preempt_enable();

	return 0;
}

int tegra19x_mce_write_rt_fwd_progress_us(u64 rt_fwd_progress_us)
{
	/* disable preemption */
	preempt_disable();

	nvg_send_req_data(TEGRA_NVG_CHANNEL_RT_FWD_PROGRESS_US,
		rt_fwd_progress_us);

	/* enable preemption */
	preempt_enable();

	return 0;
}


#ifdef CONFIG_DEBUG_FS
/* Dummy functions below */
int tegra19x_mce_features_get(void *data, u64 *val) { return -ENOTSUPP; }
int tegra19x_mce_enable_latic_set(void *data, u64 val) { return -ENOTSUPP; }
int tegra19x_mce_coresight_cg_set(void *data, u64 val) { return -ENOTSUPP; }
int tegra19x_mce_edbgreq_set(void *data, u64 val) { return -ENOTSUPP; }

#define NVG_STAT_MAX_ENTRIES	10
#define MCE_STAT_ID_SHIFT	16UL

#define UNITGROUP_IGNORED	0
#define UNITGROUP_CORE		1
#define UNITGROUP_CLUSTER	2
#define UNITGROUP_CLUSTERGROUP	3

#define CSTAT_ENTRY(stat) NVG_STAT_QUERY_##stat

struct cstats_info {
	char	*name; /* name of the cstats */
	int	id;   /* NVG id */
	int	units;/* No of cores; No of clusters; No of cluster groups */
	int	unit_group; /* 0:Ignored; 1:core; 2:cluster; 3:cluster group */
};

static struct cstats_info cstats_table[] = {
	{ "SC7_ENTRIES", CSTAT_ENTRY(SC7_ENTRIES), 1, UNITGROUP_IGNORED},
	{ "SC7_RESIDENCY_SUM",
		CSTAT_ENTRY(SC7_RESIDENCY_SUM), 1, UNITGROUP_IGNORED},
	{ "CG7_ENTRIES", CSTAT_ENTRY(CG7_ENTRIES), 2, UNITGROUP_CLUSTERGROUP},
	{ "CG7_RESIDENCY_SUM",
		CSTAT_ENTRY(CG7_RESIDENCY_SUM), 2, UNITGROUP_CLUSTERGROUP},
	{ "CC6_ENTRIES", CSTAT_ENTRY(CC6_ENTRIES), 4, UNITGROUP_CLUSTER},
	{ "CC6_RESIDENCY_SUM",
		CSTAT_ENTRY(CC6_RESIDENCY_SUM), 4, UNITGROUP_CLUSTER},
	{ "C7_ENTRIES", CSTAT_ENTRY(C7_ENTRIES), 8, UNITGROUP_CORE},
	{ "C7_RESIDENCY_SUM", CSTAT_ENTRY(C7_RESIDENCY_SUM), 8, UNITGROUP_CORE},
	{ "C6_ENTRIES", CSTAT_ENTRY(C6_ENTRIES), 8, UNITGROUP_CORE},
	{ "C6_RESIDENCY_SUM", CSTAT_ENTRY(C6_RESIDENCY_SUM), 8, UNITGROUP_CORE},
};

int tegra19x_mce_dbg_cstats_show(struct seq_file *s, void *data)
{
	int nr_cpus = num_present_cpus();
	int st, unit;
	u64 val;
	u32 mce_index;

	seq_printf(s, "%-25s%-15s%-10s\n", "name", "unit-id", "count/time");
	seq_puts(s, "---------------------------------------------------\n");
	for (st = 0; st < NVG_STAT_MAX_ENTRIES; st++) {
		switch (cstats_table[st].unit_group) {
		case UNITGROUP_IGNORED:
			cstats_table[st].units = 1;
			break;
		case UNITGROUP_CLUSTERGROUP:
			cstats_table[st].units = 2;
			break;
		case UNITGROUP_CLUSTER:
			/* Divide by 2 to get num of clusters
			 * as t19x has 2 cores per cluster
			 */
			cstats_table[st].units = nr_cpus / 2;
			break;
		case UNITGROUP_CORE:
			cstats_table[st].units = nr_cpus;
			break;
		default:
			return -EINVAL;
		}

		for (unit = 0; unit < cstats_table[st].units; unit++) {
			mce_index = ((u32)cstats_table[st].id <<
					MCE_STAT_ID_SHIFT) + (u32)unit;
			if (tegra19x_mce_read_cstate_stats(mce_index, &val))
				pr_err("mce: failed to read cstat: %s, %x\n",
					cstats_table[st].name, mce_index);
			else
				seq_printf(s, "%-25s%-15d%-20lld\n",
					cstats_table[st].name, unit, val);
		}
	}
	return 0;
}
#endif /* CONFIG_DEBUG_FS */
