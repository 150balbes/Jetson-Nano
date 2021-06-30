/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _LINUX_TEGRA18X_MCE_H
#define _LINUX_TEGRA18X_MCE_H

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
int tegra18x_mce_enter_cstate(u32 state, u32 wake_time);
int tegra18x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
				    u8 force, u32 wake_mask, bool valid);
int tegra18x_mce_update_crossover_time(u32 type, u32 time);
int tegra18x_mce_read_cstate_stats(u32 state, u64 *stats);
int tegra18x_mce_write_cstate_stats(u32 state, u32 stats);
int tegra18x_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed);
int tegra18x_mce_online_core(int cpu);
int tegra18x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable);
int tegra18x_mce_echo_data(u32 data, int *matched);
int tegra18x_mce_read_versions(u32 *major, u32 *minor);
int tegra18x_mce_enum_features(u64 *features);
int tegra18x_mce_read_uncore_mca(mca_cmd_t cmd, u64 *data, u32 *error);
int tegra18x_mce_write_uncore_mca(mca_cmd_t cmd, u64 data, u32 *error);
int tegra18x_mce_read_uncore_perfmon(u32 req, u32 *data);
int tegra18x_mce_write_uncore_perfmon(u32 req, u32 data);
int tegra18x_mce_enable_latic(void);

#ifdef CONFIG_DEBUG_FS
int tegra18x_mce_features_get(void *data, u64 *val);
int tegra18x_mce_enable_latic_set(void *data, u64 val);
int tegra18x_mce_coresight_cg_set(void *data, u64 val);
int tegra18x_mce_edbgreq_set(void *data, u64 val);
int tegra18x_mce_dbg_cstats_show(struct seq_file *s, void *data);
#endif /* CONFIG_DEBUG_FS */

/* Tegra18x cache functions */
int tegra18x_roc_flush_cache(void);
int tegra18x_roc_flush_cache_only(void);
int tegra18x_roc_clean_cache(void);
#else /* CONFIG_ARCH_TEGRA_18x_SOC */
static int tegra18x_mce_enter_cstate(u32 state, u32 wake_time)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
					   u8 force, u32 wake_mask, bool valid)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_update_crossover_time(u32 type, u32 time)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_read_cstate_stats(u32 state, u64 *stats)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_write_cstate_stats(u32 state, u32 stats)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_online_core(int cpu)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_echo_data(u32 data, int *matched)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_read_versions(u32 *major, u32 *minor)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_enum_features(u64 *features)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_read_uncore_mca(mca_cmd_t cmd, u64 *data, u32 *error)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_write_uncore_mca(mca_cmd_t cmd, u64 data, u32 *error)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_read_uncore_perfmon(u32 req, u32 *data)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_write_uncore_perfmon(u32 req, u32 data)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_enable_latic(void)
{
	return -ENOTSUPP;
}

#ifdef CONFIG_DEBUG_FS
static int tegra18x_mce_features_get(void *data, u64 *val)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_enable_latic_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_coresight_cg_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_edbgreq_set(void *data, u64 val)
{
	return -ENOTSUPP;
}
static int tegra18x_mce_dbg_cstats_show(struct seq_file *s, void *data)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_DEBUG_FS */

/* Tegra18x cache functions */
static int tegra18x_roc_flush_cache(void)
{
	return -ENOTSUPP;
}
static int tegra18x_roc_flush_cache_only(void)
{
	return -ENOTSUPP;
}
static int tegra18x_roc_clean_cache(void)
{
	return -ENOTSUPP;
}
#endif /* CONFIG_ARCH_TEGRA_18x_SOC */
#endif /* _LINUX_TEGRA18X_MCE_H */
