/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/tegra-mce.h>

#include <asm/cacheflush.h>
#include <asm/smp_plat.h>
#include <soc/tegra/chip-id.h>

#include "tegra18x-mce.h"
#include "tegra19x-mce.h"

#define SMC_SIP_INVOKE_MCE	0xC2FFFF00

#define NR_SMC_REGS		6

enum {
	TEGRA_MCE_ID_186,
	TEGRA_MCE_ID_194,
	TEGRA_MCE_ID_MAX,
};

static int tegra_mce_id = TEGRA_MCE_ID_MAX;

static int (*_tegra_mce_enter_cstate)(u32, u32);
static int (*_tegra_mce_update_cstate_info)(u32, u32, u32, u8, u32, bool);
static int (*_tegra_mce_update_crossover_time)(u32, u32);
static int (*_tegra_mce_read_cstate_stats)(u32, u64 *);
static int (*_tegra_mce_write_cstate_stats)(u32, u32);
static int (*_tegra_mce_is_sc7_allowed)(u32, u32, u32 *);
static int (*_tegra_mce_online_core)(int);
static int (*_tegra_mce_cc3_ctrl)(u32, u32, u8);
static int (*_tegra_mce_echo_data)(u32, int *);
static int (*_tegra_mce_read_versions)(u32 *, u32 *);
static int (*_tegra_mce_enum_features)(u64 *);
static int (*_tegra_mce_read_uncore_mca)(mca_cmd_t, u64 *, u32 *);
static int (*_tegra_mce_write_uncore_mca)(mca_cmd_t, u64, u32 *);
static int (*_tegra_mce_read_uncore_perfmon)(u32, u32 *);
static int (*_tegra_mce_write_uncore_perfmon)(u32, u32);
static int (*_tegra_mce_enable_latic)(void);
static int (*_tegra_mce_write_dda_ctrl)(u32 index, u64 value);
static int (*_tegra_mce_read_dda_ctrl)(u32 index, u64 *value);
static int (*_tegra_mce_read_l3_cache_ways)(u64 *value);
static int (*_tegra_mce_write_l3_cache_ways)(u64 data, u64 *value);
static int (*_tegra_mce_read_rt_safe_mask)(u64 *);
static int (*_tegra_mce_write_rt_safe_mask)(u64);
static int (*_tegra_mce_read_rt_window_us)(u64 *);
static int (*_tegra_mce_write_rt_window_us)(u64);
static int (*_tegra_mce_read_rt_fwd_progress_us)(u64 *);
static int (*_tegra_mce_write_rt_fwd_progress_us)(u64);
/**
 * Specify power state and wake time for entering upon STANDBYWFI
 *
 * @state:		requested core power state
 * @wake_time:	wake time in TSC ticks
 *
 * Returns 0 if success.
 */
int tegra_mce_enter_cstate(u32 state, u32 wake_time)
{
	if (!_tegra_mce_enter_cstate)
		return -ENOTSUPP;
	return _tegra_mce_enter_cstate(state, wake_time);
}
EXPORT_SYMBOL_GPL(tegra_mce_enter_cstate);

/**
 * Specify deepest cluster/ccplex/system states allowed.
 *
 * @cluster:	deepest cluster-wide state
 * @ccplex:		deepest ccplex-wide state
 * @system:		deepest system-wide state
 * @force:		forced system state
 * @wake_mask:	wake mask to be updated
 * @valid:		is wake_mask applicable?
 *
 * Returns 0 if success.
 */
int tegra_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
				 u8 force, u32 wake_mask, bool valid)
{
	if (!_tegra_mce_update_cstate_info)
		return -ENOTSUPP;
	return _tegra_mce_update_cstate_info(cluster, ccplex, system,
					     force, wake_mask, valid);
}
EXPORT_SYMBOL_GPL(tegra_mce_update_cstate_info);

/**
 * Update threshold for one specific c-state crossover
 *
 * @type: type of state crossover.
 * @time: idle time threshold.
 *
 * Returns 0 if success.
 */
int tegra_mce_update_crossover_time(u32 type, u32 time)
{
	if (!_tegra_mce_update_crossover_time)
		return -ENOTSUPP;
	return _tegra_mce_update_crossover_time(type, time);
}
EXPORT_SYMBOL_GPL(tegra_mce_update_crossover_time);

/**
 * Query the runtime stats of a specific cstate
 *
 * @state: c-state of the stats.
 * @stats: output integer to hold the stats.
 *
 * Returns 0 if success.
 */
int tegra_mce_read_cstate_stats(u32 state, u64 *stats)
{
	if (!_tegra_mce_read_cstate_stats)
		return -ENOTSUPP;
	return _tegra_mce_read_cstate_stats(state, stats);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_cstate_stats);

/**
 * Overwrite the runtime stats of a specific c-state
 *
 * @state: c-state of the stats.
 * @stats: integer represents the new stats.
 *
 * Returns 0 if success.
 */
int tegra_mce_write_cstate_stats(u32 state, u32 stats)
{
	if (!_tegra_mce_write_cstate_stats)
		return -ENOTSUPP;
	return _tegra_mce_write_cstate_stats(state, stats);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_cstate_stats);

/**
 * Query MCE to determine if SC7 is allowed
 * given a target core's C-state and wake time
 *
 * @state: c-state of the stats.
 * @stats: integer represents the new stats.
 * @allowed: pointer to result
 *
 * Returns 0 if success.
 */
int tegra_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	if (!_tegra_mce_is_sc7_allowed)
		return -ENOTSUPP;
	return _tegra_mce_is_sc7_allowed(state, wake, allowed);
}
EXPORT_SYMBOL_GPL(tegra_mce_is_sc7_allowed);

/**
 * Bring another offlined core back online to C0 state.
 *
 * @cpu: logical cpuid from smp_processor_id()
 *
 * Returns 0 if success.
 */
int tegra_mce_online_core(int cpu)
{
	if (!_tegra_mce_online_core)
		return -ENOTSUPP;
	return _tegra_mce_online_core(cpu);
}
EXPORT_SYMBOL_GPL(tegra_mce_online_core);

/**
 * Program Auto-CC3 feature.
 *
 * @ndiv:		ndiv of IDLE voltage/freq register
 * @vindex:		vindex of IDLE voltage/freq register
 *			(Not used on tegra19x)
 * @enable:		enable bit for Auto-CC3
 *
 * Returns 0 if success.
 */
int tegra_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable)
{
	if (!_tegra_mce_cc3_ctrl)
		return -ENOTSUPP;
	return _tegra_mce_cc3_ctrl(ndiv, vindex, enable);
}
EXPORT_SYMBOL_GPL(tegra_mce_cc3_ctrl);

/**
 * Send data to MCE which echoes it back.
 *
 * @data: data to be sent to MCE.
 * @out: output data to hold the response.
 * @matched: pointer to matching result
 *
 * Returns 0 if success.
 */
int tegra_mce_echo_data(u32 data, int *matched)
{
	if (!_tegra_mce_echo_data)
		return -ENOTSUPP;
	return _tegra_mce_echo_data(data, matched);
}
EXPORT_SYMBOL_GPL(tegra_mce_echo_data);

/**
 * Read out MCE API major/minor versions
 *
 * @major: output for major number.
 * @minor: output for minor number.
 *
 * Returns 0 if success.
 */
int tegra_mce_read_versions(u32 *major, u32 *minor)
{
	if (!_tegra_mce_read_versions)
		return -ENOTSUPP;
	return _tegra_mce_read_versions(major, minor);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_versions);

/**
 * Read out RT Safe Mask
 *
 * @rt_safe_mask: output for rt safe mask.
 *
 * Returns 0 if success.
 */
int tegra_mce_read_rt_safe_mask(u64 *rt_safe_mask)
{
	if (!_tegra_mce_read_rt_safe_mask)
		return -ENOTSUPP;
	return _tegra_mce_read_rt_safe_mask(rt_safe_mask);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_rt_safe_mask);

/**
 * Write RT Safe Mask
 *
 * @rt_safe_mask: rt safe mask value to be written
 *
 * Returns 0 if success.
 */
int tegra_mce_write_rt_safe_mask(u64 rt_safe_mask)
{
	if (!_tegra_mce_write_rt_safe_mask)
		return -ENOTSUPP;
	return _tegra_mce_write_rt_safe_mask(rt_safe_mask);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_rt_safe_mask);

/**
 * Read out RT Window US
 *
 * @rt_window_us: output for rt window us
 *
 * Returns 0 if success.
 */
int tegra_mce_read_rt_window_us(u64 *rt_window_us)
{
	if (!_tegra_mce_read_rt_window_us)
		return -ENOTSUPP;
	return _tegra_mce_read_rt_window_us(rt_window_us);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_rt_window_us);


/**
 * Write RT Window US
 *
 * @rt_window_us: rt window us value to be written
 *
 * Returns 0 if success.
 */
int tegra_mce_write_rt_window_us(u64 rt_window_us)
{
	if (!_tegra_mce_write_rt_window_us)
		return -ENOTSUPP;
	return _tegra_mce_write_rt_window_us(rt_window_us);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_rt_window_us);


/**
 * Read out RT Fwd Progress US
 *
 * @rt_fwd_progress_us: output for rt fwd progress us
 *
 * Returns 0 if success.
 */
int tegra_mce_read_rt_fwd_progress_us(u64 *rt_fwd_progress_us)
{
	if (!_tegra_mce_read_rt_fwd_progress_us)
		return -ENOTSUPP;
	return _tegra_mce_read_rt_fwd_progress_us(rt_fwd_progress_us);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_rt_fwd_progress_us);


/**
 * Write RT Fwd Progress US
 *
 * @rt_fwd_progress_us: rt fwd progress us value to be written
 *
 * Returns 0 if success.
 */
int tegra_mce_write_rt_fwd_progress_us(u64 rt_fwd_progress_us)
{
	if (!_tegra_mce_write_rt_fwd_progress_us)
		return -ENOTSUPP;
	return _tegra_mce_write_rt_fwd_progress_us(rt_fwd_progress_us);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_rt_fwd_progress_us);


/**
 * Enumerate MCE API features
 *
 * @features: output feature vector (4bits each)
 *
 * Returns 0 if success.
 */
int tegra_mce_enum_features(u64 *features)
{
	if (!_tegra_mce_enum_features)
		return -ENOTSUPP;
	return _tegra_mce_enum_features(features);
}
EXPORT_SYMBOL_GPL(tegra_mce_enum_features);

/**
 * Read uncore MCA errors.
 *
 * @cmd: MCA command
 * @data: output data for the command
 * @error: error from MCA
 *
 * Returns 0 if success.
 */
int tegra_mce_read_uncore_mca(mca_cmd_t cmd, u64 *data, u32 *error)
{
	if (!_tegra_mce_read_uncore_mca)
		return -ENOTSUPP;
	return _tegra_mce_read_uncore_mca(cmd, data, error);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_uncore_mca);

/**
 * Write uncore MCA errors.
 *
 * @cmd: MCA command
 * @data: input data for the command
 * @error: error from MCA
 *
 * Returns 0 if success.
 */
int tegra_mce_write_uncore_mca(mca_cmd_t cmd, u64 data, u32 *error)
{
	if (!_tegra_mce_write_uncore_mca)
		return -ENOTSUPP;
	return _tegra_mce_write_uncore_mca(cmd, data, error);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_uncore_mca);

/**
 * Query PMU for uncore perfmon counter
 *
 * @req input command and counter index
 * @data output counter value
 *
 * Returns status of read request.
 */
int tegra_mce_read_uncore_perfmon(u32 req, u32 *data)
{
	if (!_tegra_mce_read_uncore_perfmon)
		return -ENOTSUPP;
	return _tegra_mce_read_uncore_perfmon(req, data);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_uncore_perfmon);

/**
 * Write PMU reg for uncore perfmon counter
 *
 * @req input command and counter index
 * @data data to be written
 *
 * Returns status of write request.
 */
int tegra_mce_write_uncore_perfmon(u32 req, u32 data)
{
	if (!_tegra_mce_write_uncore_perfmon)
		return -ENOTSUPP;
	return _tegra_mce_write_uncore_perfmon(req, data);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_uncore_perfmon);

int tegra_mce_enable_latic(void)
{
	if (!_tegra_mce_enable_latic)
		return -ENOTSUPP;
	return _tegra_mce_enable_latic();
}
EXPORT_SYMBOL_GPL(tegra_mce_enable_latic);

/**
 * Write to NVG DDA registers
 *
 * @index:   NVG communication channel id
 * @value:   Register value to be written
 *
 * Returns 0 on success
 */
int tegra_mce_write_dda_ctrl(u32 index, u64 value)
{
	if(!_tegra_mce_write_dda_ctrl)
		return -ENOTSUPP;
	return _tegra_mce_write_dda_ctrl(index, value);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_dda_ctrl);

/**
 * Read NVG DDA registers
 *
 * @index:   NVG communication channel id
 * @value:   Associated register value read
 *
 * Returns 0 on success
 */
int tegra_mce_read_dda_ctrl(u32 index, u64 *value)
{
	if(!_tegra_mce_read_dda_ctrl)
		return -ENOTSUPP;
	return _tegra_mce_read_dda_ctrl(index, value);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_dda_ctrl);

/**
 * Read NVG L3 cache control register
 *
 * @value:   Fill L3 cache ways
 *
 * Returns 0 on success
 */
int tegra_mce_read_l3_cache_ways(u64 *value)
{
	if (!_tegra_mce_read_l3_cache_ways)
		return -ENOTSUPP;
	return _tegra_mce_read_l3_cache_ways(value);
}
EXPORT_SYMBOL_GPL(tegra_mce_read_l3_cache_ways);

/**
 * Write L3 cache ways and read back the l3 cache ways written
 *
 * @data:   L3 cache ways to be writtein
 * @value:  L3 cache ways returrned back
 *
 * Returns 0 on success
 */
int tegra_mce_write_l3_cache_ways(u64 data, u64 *value)
{
	if (!_tegra_mce_write_l3_cache_ways)
		return -ENOTSUPP;
	return _tegra_mce_write_l3_cache_ways(data, value);
}
EXPORT_SYMBOL_GPL(tegra_mce_write_l3_cache_ways);

#ifdef CONFIG_DEBUG_FS
static struct dentry *mce_debugfs;

static int tegra_mce_echo_set(void *data, u64 val)
{
	u32 matched;
	int ret;

	ret = tegra_mce_echo_data((u32)val, &matched);
	if (ret && ret != -ENOTSUPP)
		return -EINVAL;
	return 0;
}

static int tegra_mce_versions_get(void *data, u64 *val)
{
	u32 major, minor;
	int ret;

	ret = tegra_mce_read_versions(&major, &minor);
	if (!ret)
		*val = ((u64)major << 32) | minor;
	return ret;
}

static int tegra_mce_rt_safe_mask_get(void *data, u64 *val)
{
	u64 rt_safe_mask;
	int ret;

	ret = tegra_mce_read_rt_safe_mask(&rt_safe_mask);
	if (!ret)
		*val = rt_safe_mask;
	return ret;
}

static int tegra_mce_rt_safe_mask_set(void *data, u64 val)
{
	int ret;

	ret = tegra_mce_write_rt_safe_mask(val);
	return ret;
}

static int tegra_mce_rt_window_us_get(void *data, u64 *val)
{
	u64 rt_window_us;
	int ret;

	ret = tegra_mce_read_rt_window_us(&rt_window_us);
	if (!ret)
		*val = rt_window_us;
	return ret;
}

static int tegra_mce_rt_window_us_set(void *data, u64 val)
{
	int ret;

	ret = tegra_mce_write_rt_window_us(val);
	return ret;
}

static int tegra_mce_rt_fwd_progress_us_get(void *data, u64 *val)
{
	u64 rt_fwd_progress_us;
	int ret;

	ret = tegra_mce_read_rt_fwd_progress_us(&rt_fwd_progress_us);
	if (!ret)
		*val = rt_fwd_progress_us;
	return ret;
}

static int tegra_mce_rt_fwd_progress_us_set(void *data, u64 val)
{
	int ret;

	ret = tegra_mce_write_rt_fwd_progress_us(val);
	return ret;
}

#define TEGRA_MCE_DBGFS_FUNC(name, type1, param1, type2, param2)	\
static int tegra_##name(type1 param1, type2 param2)			\
{									\
	int (*f)(type1, type2) = NULL;					\
									\
	switch (tegra_mce_id) {						\
	case TEGRA_MCE_ID_186:						\
		f = tegra18x_##name;					\
		break;							\
	case TEGRA_MCE_ID_194:						\
		f = tegra19x_##name;					\
		break;							\
	default:							\
		return -ENOTSUPP;					\
	}								\
	return f ? f(param1, param2) : -ENOTSUPP;			\
}

TEGRA_MCE_DBGFS_FUNC(mce_features_get, void *, data, u64 *, val);
TEGRA_MCE_DBGFS_FUNC(mce_enable_latic_set, void *, data, u64, val);
TEGRA_MCE_DBGFS_FUNC(mce_coresight_cg_set, void *, data, u64, val);
TEGRA_MCE_DBGFS_FUNC(mce_edbgreq_set, void *, data, u64, val);

static int tegra_mce_dbg_cstats_open(struct inode *inode, struct file *file)
{
	int (*f)(struct seq_file *, void *);

	switch (tegra_mce_id) {
	case TEGRA_MCE_ID_186:
		f = tegra18x_mce_dbg_cstats_show;
		break;
	case TEGRA_MCE_ID_194:
		f = tegra19x_mce_dbg_cstats_show;
		break;
	default:
		return -EINVAL;
	}

	return single_open(file, f, inode->i_private);
}

static const struct file_operations tegra_mce_cstats_fops = {
	.open = tegra_mce_dbg_cstats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_echo_fops, NULL,
			tegra_mce_echo_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_versions_fops, tegra_mce_versions_get,
			NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_features_fops, tegra_mce_features_get,
			NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_enable_latic_fops, NULL,
			tegra_mce_enable_latic_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_coresight_cg_fops, NULL,
			tegra_mce_coresight_cg_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_edbgreq_fops, NULL,
			tegra_mce_edbgreq_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_rt_safe_mask_fops, tegra_mce_rt_safe_mask_get,
			tegra_mce_rt_safe_mask_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_rt_window_us_fops, tegra_mce_rt_window_us_get,
			tegra_mce_rt_window_us_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tegra_mce_rt_fwd_progress_us_fops,
			tegra_mce_rt_fwd_progress_us_get,
			tegra_mce_rt_fwd_progress_us_set, "%llu\n");

struct debugfs_entry {
	const char *name;
	const struct file_operations *fops;
	mode_t mode;
};

/* Make sure to put an NULL entry at the end of each group */
static struct debugfs_entry tegra18x_mce_attrs[] = {
	{ "echo", &tegra_mce_echo_fops, 0200 },
	{ "versions", &tegra_mce_versions_fops, 0444 },
	{ "features", &tegra_mce_features_fops, 0444 },
	{ "cstats", &tegra_mce_cstats_fops, 0444 },
	{ "enable-latic", &tegra_mce_enable_latic_fops, 0200 },
	{ "coresight_cg_enable", &tegra_mce_coresight_cg_fops, 0200 },
	{ "edbgreq", &tegra_mce_edbgreq_fops, 0200 },
	{ NULL, NULL, 0 },
};

static struct debugfs_entry tegra19x_mce_attrs[] = {
	{ "versions", &tegra_mce_versions_fops, 0444 },
	{ "cstats", &tegra_mce_cstats_fops, 0444 },
	{ "rt_safe_mask", &tegra_mce_rt_safe_mask_fops, 0644 },
	{ "rt_window_us", &tegra_mce_rt_window_us_fops, 0644 },
	{ "rt_fwd_progress_us", &tegra_mce_rt_fwd_progress_us_fops, 0644 },
	{ NULL, NULL, 0 }
};

static struct debugfs_entry *tegra_mce_attrs[TEGRA_MCE_ID_MAX] = {
	tegra18x_mce_attrs,
	tegra19x_mce_attrs,
};

static __init int tegra_mce_init(void)
{
	struct debugfs_entry *fent;
	struct dentry *dent;
	int ret;

	if (tegra_mce_id >= TEGRA_MCE_ID_MAX)
		return -ENOTSUPP;

	mce_debugfs = debugfs_create_dir("tegra_mce", NULL);
	if (!mce_debugfs)
		return -ENOMEM;

	for (fent = tegra_mce_attrs[tegra_mce_id]; fent->name; fent++) {
		dent = debugfs_create_file(fent->name, fent->mode,
					   mce_debugfs, NULL, fent->fops);
		if (IS_ERR_OR_NULL(dent)) {
			ret = dent ? PTR_ERR(dent) : -EINVAL;
			pr_err("%s: failed to create debugfs (%s): %d\n",
			       __func__, fent->name, ret);
			goto err;
		}
	}

	pr_debug("%s: init finished\n", __func__);

	return 0;

err:
	debugfs_remove_recursive(mce_debugfs);

	return ret;
}

static void __exit tegra_mce_exit(void)
{
	if (tegra_mce_id >= TEGRA_MCE_ID_MAX)
		return;

	debugfs_remove_recursive(mce_debugfs);
}
module_init(tegra_mce_init);
module_exit(tegra_mce_exit);
#endif /* CONFIG_DEBUG_FS */

/*
 * Tegra cache functions
 *
 * Return 0 if success or -ENOTSUPP.
 *
 */
int tegra_flush_cache_all(void)
{
	int ret = 0;

	switch (tegra_get_chip_id()) {
	case TEGRA186:
		ret = tegra18x_roc_flush_cache();
		break;
	case TEGRA194:
		ret = t19x_flush_cache_all();
		/* Fallback to VA flush cache all if not support or failed */
		if (ret)
			flush_cache_all();
		break;
	default:
		flush_cache_all();
		break;
	}

	/* CRITICAL: failed to flush all cache */
	WARN_ON(ret && ret != -ENOTSUPP);

	return ret;
}
EXPORT_SYMBOL(tegra_flush_cache_all);

int tegra_flush_dcache_all(void *__maybe_unused unused)
{
	int ret = 0;

	switch (tegra_get_chip_id()) {
	case TEGRA186:
		ret = tegra18x_roc_flush_cache_only();
		break;
	case TEGRA194:
		ret = t19x_flush_dcache_all();
		/* Fallback to VA flush dcache if not support or failed */
		if (ret)
			__flush_dcache_all(unused);
		break;
	default:
		__flush_dcache_all(unused);
		break;
	}

	/* CRITICAL: failed to flush dcache */
	WARN_ON(ret && ret != -ENOTSUPP);

	return ret;
}
EXPORT_SYMBOL(tegra_flush_dcache_all);

int tegra_clean_dcache_all(void *__maybe_unused unused)
{
	int ret = 0;

	switch (tegra_get_chip_id()) {
	case TEGRA186:
		ret = tegra18x_roc_clean_cache();
		break;
	case TEGRA194:
		ret = t19x_clean_dcache_all();
		/* Fallback to VA clean if not support or failed */
		if (ret)
			__clean_dcache_all(unused);
		break;
	default:
		__clean_dcache_all(unused);
		break;
	}

	/* CRITICAL: failed to clean dcache */
	WARN_ON(ret && ret != -ENOTSUPP);

	return ret;
}
EXPORT_SYMBOL(tegra_clean_dcache_all);

/* Make sure functions will be available for other drivers */
static __init int tegra_mce_early_init(void)
{
	switch (tegra_get_chip_id()) {
	case TEGRA186:
		tegra_mce_id = TEGRA_MCE_ID_186;
		_tegra_mce_enter_cstate = tegra18x_mce_enter_cstate;
		_tegra_mce_update_cstate_info = tegra18x_mce_update_cstate_info;
		_tegra_mce_update_crossover_time =
			tegra18x_mce_update_crossover_time;
		_tegra_mce_read_cstate_stats = tegra18x_mce_read_cstate_stats;
		_tegra_mce_write_cstate_stats = tegra18x_mce_write_cstate_stats;
		_tegra_mce_is_sc7_allowed = tegra18x_mce_is_sc7_allowed;
		_tegra_mce_online_core = tegra18x_mce_online_core;
		_tegra_mce_cc3_ctrl = tegra18x_mce_cc3_ctrl;
		_tegra_mce_echo_data = tegra18x_mce_echo_data;
		_tegra_mce_read_versions = tegra18x_mce_read_versions;
		_tegra_mce_enum_features = tegra18x_mce_enum_features;
		_tegra_mce_read_uncore_mca = tegra18x_mce_read_uncore_mca;
		_tegra_mce_write_uncore_mca = tegra18x_mce_write_uncore_mca;
		_tegra_mce_read_uncore_perfmon =
			tegra18x_mce_read_uncore_perfmon;
		_tegra_mce_write_uncore_perfmon =
			tegra18x_mce_write_uncore_perfmon;
		_tegra_mce_enable_latic = tegra18x_mce_enable_latic;
		break;
	case TEGRA194:
		tegra_mce_id = TEGRA_MCE_ID_194;
		_tegra_mce_enter_cstate = tegra19x_mce_enter_cstate;
		_tegra_mce_update_cstate_info = tegra19x_mce_update_cstate_info;
		_tegra_mce_update_crossover_time =
			tegra19x_mce_update_crossover_time;
		_tegra_mce_read_cstate_stats = tegra19x_mce_read_cstate_stats;
		_tegra_mce_cc3_ctrl = tegra19x_mce_cc3_ctrl;
		_tegra_mce_read_versions = tegra19x_mce_read_versions;
		_tegra_mce_write_dda_ctrl = tegra19x_mce_write_dda_ctrl;
		_tegra_mce_read_dda_ctrl = tegra19x_mce_read_dda_ctrl;
		_tegra_mce_read_l3_cache_ways = tegra19x_mce_read_l3_cache_ways;
		_tegra_mce_write_l3_cache_ways =
			tegra19x_mce_write_l3_cache_ways;
#ifdef CONFIG_DEBUG_FS
		_tegra_mce_read_rt_safe_mask = tegra19x_mce_read_rt_safe_mask;
		_tegra_mce_write_rt_safe_mask = tegra19x_mce_write_rt_safe_mask;
		_tegra_mce_read_rt_window_us = tegra19x_mce_read_rt_window_us;
		_tegra_mce_write_rt_window_us = tegra19x_mce_write_rt_window_us;
		_tegra_mce_read_rt_fwd_progress_us =
			tegra19x_mce_read_rt_fwd_progress_us;
		_tegra_mce_write_rt_fwd_progress_us =
			tegra19x_mce_write_rt_fwd_progress_us;
#endif /* CONFIG_DEBUG_FS */
		break;
	default:
		/* Do not support any other platform */
		return -ENOTSUPP;
	}

	return 0;
}
early_initcall(tegra_mce_early_init);

MODULE_DESCRIPTION("NVIDIA Tegra MCE driver");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL v2");
