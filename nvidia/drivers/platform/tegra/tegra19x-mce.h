/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_TEGRA19X_MCE_H
#define _LINUX_TEGRA19X_MCE_H

#ifdef CONFIG_ARCH_TEGRA_19x_SOC
int tegra19x_mce_enter_cstate(u32 state, u32 wake_time);
int tegra19x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
				    u8 force, u32 wake_mask, bool valid);
int tegra19x_mce_update_crossover_time(u32 type, u32 time);
int tegra19x_mce_read_cstate_stats(u32 state, u64 *stats);
int tegra19x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable);
int tegra19x_mce_read_versions(u32 *major, u32 *minor);
int tegra19x_mce_write_dda_ctrl(u32 index, u64 value);
int tegra19x_mce_read_dda_ctrl(u32 index, u64 *value);

#ifdef CONFIG_DEBUG_FS
int tegra19x_mce_features_get(void *data, u64 *val);
int tegra19x_mce_enable_latic_set(void *data, u64 val);
int tegra19x_mce_coresight_cg_set(void *data, u64 val);
int tegra19x_mce_edbgreq_set(void *data, u64 val);
int tegra19x_mce_dbg_cstats_show(struct seq_file *s, void *data);
int tegra19x_mce_read_rt_safe_mask(u64 *rt_safe_mask);
int tegra19x_mce_write_rt_safe_mask(u64 rt_safe_mask);
int tegra19x_mce_read_rt_window_us(u64 *rt_window_us);
int tegra19x_mce_write_rt_window_us(u64 rt_window_us);
int tegra19x_mce_read_rt_fwd_progress_us(u64 *rt_fwd_progress_us);
int tegra19x_mce_write_rt_fwd_progress_us(u64 rt_fwd_progress_us);
#endif

/* Tegra19x cache functions */
int t19x_flush_cache_all(void);
int t19x_flush_dcache_all(void);
int t19x_clean_dcache_all(void);

/* Return L3 cache ways */
int tegra19x_mce_read_l3_cache_ways(u64 *value);
/* Write L3 cache ways and return L3 cache ways actually written */
int tegra19x_mce_write_l3_cache_ways(u64 data, u64 *value);
#else /* CONFIG_ARCH_TEGRA_19x_SOC */
static int tegra19x_mce_enter_cstate(u32 state, u32 wake_time)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
					   u8 force, u32 wake_mask, bool valid)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_update_crossover_time(u32 type, u32 time)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_read_cstate_stats(u32 state, u64 *stats)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_read_versions(u32 *major, u32 *minor)
{
	return -ENOTSUPP;
}

static int tegra19x_mce_write_dda_ctrl(u32 index, u64 value)
{
	return -ENOTSUPP;
}

static int tegra19x_mce_read_dda_ctrl(u32 index, u64 *value)
{
	return -ENOTSUPP;
}

#ifdef CONFIG_DEBUG_FS
static int tegra19x_mce_features_get(void *data, u64 *val)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_enable_latic_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_coresight_cg_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_edbgreq_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_dbg_cstats_show(struct seq_file *s, void *data)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_DEBUG_FS */

static int t19x_flush_cache_all(void)
{
	return -ENOTSUPP;
}
static int t19x_flush_dcache_all(void)
{
	return -ENOTSUPP;
}
static int t19x_clean_dcache_all(void)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_write_l3_cache_ways(u64 data, u64 *value)
{
	return -ENOTSUPP;
}
static int tegra19x_mce_read_l3_cache_ways(u64 *value)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_ARCH_TEGRA_19x_SOC */
#endif /* _LINUX_TEGRA19X_MCE_H */
