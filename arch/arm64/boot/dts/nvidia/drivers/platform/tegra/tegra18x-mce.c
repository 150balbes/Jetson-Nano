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
 */

#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/t18x_ari.h>
#include <linux/tegra-mce.h>

#include <asm/smp_plat.h>
#include <soc/tegra/chip-id.h>
#include "tegra18x-mce.h"

#define SMC_SIP_INVOKE_MCE	0xC2FFFF00

#define NR_SMC_REGS		6

/* MCE command enums for SMC calls */
enum {
	MCE_SMC_ENTER_CSTATE = 0,
	MCE_SMC_UPDATE_CSTATE_INFO = 1,
	MCE_SMC_UPDATE_XOVER_TIME = 2,
	MCE_SMC_READ_CSTATE_STATS = 3,
	MCE_SMC_WRITE_CSTATE_STATS = 4,
	MCE_SMC_IS_SC7_ALLOWED = 5,
	MCE_SMC_ONLINE_CORE = 6,
	MCE_SMC_CC3_CTRL = 7,
	MCE_SMC_ECHO_DATA = 8,
	MCE_SMC_READ_VERSIONS = 9,
	MCE_SMC_ENUM_FEATURES = 10,
	MCE_SMC_ROC_FLUSH_CACHE = 11,
	MCE_SMC_ENUM_READ_MCA = 12,
	MCE_SMC_ENUM_WRITE_MCA = 13,
	MCE_SMC_ROC_FLUSH_CACHE_ONLY = 14,
	MCE_SMC_ROC_CLEAN_CACHE_ONLY = 15,
	MCE_SMC_ENABLE_LATIC = 16,
	MCE_SMC_UNCORE_PERFMON_REQ = 17,
	MCE_SMC_MISC_CCPLEX = 18,
	MCE_SMC_ENUM_MAX = 0xFF,	/* enums cannot exceed this value */
};

struct tegra_mce_regs {
	u64 args[NR_SMC_REGS];
};

static noinline notrace int __send_smc(u8 func, struct tegra_mce_regs *regs)
{
	u32 ret = SMC_SIP_INVOKE_MCE | (func & MCE_SMC_ENUM_MAX);

	asm volatile (
	"	mov	x0, %0\n"
	"	ldp	x1, x2, [%1, #16 * 0]\n"
	"	ldp	x3, x4, [%1, #16 * 1]\n"
	"	ldp	x5, x6, [%1, #16 * 2]\n"
	"	isb\n"
	"	smc	#0\n"
	"	mov	%0, x0\n"
	"	stp	x0, x1, [%1, #16 * 0]\n"
	"	stp	x2, x3, [%1, #16 * 1]\n"
	: "+r" (ret)
	: "r" (regs)
	: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8",
	"x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");

	return ret;
}

#define send_smc(func, regs)						\
({									\
	int __ret = __send_smc(func, regs);				\
									\
	if (__ret)							\
		pr_err("%s: failed (ret=%d)\n", __func__, __ret);	\
	__ret;								\
})

int tegra18x_mce_enter_cstate(u32 state, u32 wake_time)
{
	struct tegra_mce_regs regs;

	regs.args[0] = state;
	regs.args[1] = wake_time;

	return send_smc(MCE_SMC_ENTER_CSTATE, &regs);
}

int tegra18x_mce_update_cstate_info(u32 cluster, u32 ccplex, u32 system,
				    u8 force, u32 wake_mask, bool valid)
{
	struct tegra_mce_regs regs;

	regs.args[0] = cluster;
	regs.args[1] = ccplex;
	regs.args[2] = system;
	regs.args[3] = force;
	regs.args[4] = wake_mask;
	regs.args[5] = valid;

	return send_smc(MCE_SMC_UPDATE_CSTATE_INFO, &regs);
}

int tegra18x_mce_update_crossover_time(u32 type, u32 time)
{
	struct tegra_mce_regs regs;

	regs.args[0] = type;
	regs.args[1] = time;

	return send_smc(MCE_SMC_UPDATE_XOVER_TIME, &regs);
}

int tegra18x_mce_read_cstate_stats(u32 state, u64 *stats)
{
	struct tegra_mce_regs regs;

	regs.args[0] = state;
	send_smc(MCE_SMC_READ_CSTATE_STATS, &regs);
	*stats = regs.args[2];

	return 0;
}

int tegra18x_mce_write_cstate_stats(u32 state, u32 stats)
{
	struct tegra_mce_regs regs;

	regs.args[0] = state;
	regs.args[1] = stats;

	return send_smc(MCE_SMC_WRITE_CSTATE_STATS, &regs);
}

int tegra18x_mce_is_sc7_allowed(u32 state, u32 wake, u32 *allowed)
{
	struct tegra_mce_regs regs;

	regs.args[0] = state;
	regs.args[1] = wake;
	send_smc(MCE_SMC_IS_SC7_ALLOWED, &regs);
	*allowed = (u32)regs.args[3];

	return 0;
}

int tegra18x_mce_online_core(int cpu)
{
	struct tegra_mce_regs regs;

	regs.args[0] = cpu_logical_map(cpu);

	return send_smc(MCE_SMC_ONLINE_CORE, &regs);
}

int tegra18x_mce_cc3_ctrl(u32 ndiv, u32 vindex, u8 enable)
{
	struct tegra_mce_regs regs;

	regs.args[0] = ndiv;
	regs.args[1] = vindex;
	regs.args[2] = enable;

	return send_smc(MCE_SMC_CC3_CTRL, &regs);
}

int tegra18x_mce_echo_data(u32 data, int *matched)
{
	struct tegra_mce_regs regs;

	regs.args[0] = data;
	send_smc(MCE_SMC_ECHO_DATA, &regs);
	*matched = (u32)regs.args[2];

	return 0;
}

int tegra18x_mce_read_versions(u32 *major, u32 *minor)
{
	struct tegra_mce_regs regs;

	send_smc(MCE_SMC_READ_VERSIONS, &regs);
	*major = (u32)regs.args[1];
	*minor = (u32)regs.args[2];

	return 0;
}

int tegra18x_mce_enum_features(u64 *features)
{
	struct tegra_mce_regs regs;

	send_smc(MCE_SMC_ENUM_FEATURES, &regs);
	*features = (u32)regs.args[1];

	return 0;
}

int tegra18x_mce_read_uncore_mca(mca_cmd_t cmd, u64 *data, u32 *error)
{
	struct tegra_mce_regs regs;

	regs.args[0] = cmd.data;
	regs.args[1] = 0;
	send_smc(MCE_SMC_ENUM_READ_MCA, &regs);
	*data = regs.args[2];
	*error = (u32)regs.args[3];

	return 0;
}

int tegra18x_mce_write_uncore_mca(mca_cmd_t cmd, u64 data, u32 *error)
{
	struct tegra_mce_regs regs;

	regs.args[0] = cmd.data;
	regs.args[1] = data;
	send_smc(MCE_SMC_ENUM_WRITE_MCA, &regs);
	*error = (u32)regs.args[3];

	return 0;
}

int tegra18x_mce_read_uncore_perfmon(u32 req, u32 *data)
{
	struct tegra_mce_regs regs;
	u32 status;

	if (data == NULL)
		return -EINVAL;

	regs.args[0] = req;
	status = send_smc(MCE_SMC_UNCORE_PERFMON_REQ, &regs);
	*data = (u32)regs.args[1];

	return status;
}

int tegra18x_mce_write_uncore_perfmon(u32 req, u32 data)
{
	struct tegra_mce_regs regs;
	u32 status = 0;

	regs.args[0] = req;
	regs.args[1] = data;
	status = send_smc(MCE_SMC_UNCORE_PERFMON_REQ, &regs);

	return status;
}

int tegra18x_mce_enable_latic(void)
{
	struct tegra_mce_regs regs;

	return send_smc(MCE_SMC_ENABLE_LATIC, &regs);
}

#ifdef CONFIG_DEBUG_FS
int tegra18x_mce_features_get(void *data, u64 *val)
{
	return tegra_mce_enum_features(val);
}

int tegra18x_mce_enable_latic_set(void *data, u64 val)
{
	if (tegra_mce_enable_latic())
		return -EINVAL;
	return 0;
}

/* Enable/disable coresight clock gating */
int tegra18x_mce_coresight_cg_set(void *data, u64 val)
{
	struct tegra_mce_regs regs;

	/* Enable - 1, disable - 0 are the only valid values */
	if (val > 1) {
		pr_err("mce: invalid enable value.\n");
		return -EINVAL;
	}

	regs.args[0] = TEGRA_ARI_MISC_CCPLEX_CORESIGHT_CG_CTRL;
	regs.args[1] = (u32)val;
	send_smc(MCE_SMC_MISC_CCPLEX, &regs);

	return 0;
}

/* Enable external debug on MCA */
int tegra18x_mce_edbgreq_set(void *data, u64 val)
{
	struct tegra_mce_regs regs;

	regs.args[0] = TEGRA_ARI_MISC_CCPLEX_EDBGREQ;
	send_smc(MCE_SMC_MISC_CCPLEX, &regs);

	return 0;
}

#define CSTAT_ENTRY(stat)[TEGRA_ARI_CSTATE_STATS_##stat] = #stat

static const char * const cstats_table[] = {
	CSTAT_ENTRY(SC7_ENTRIES),
	CSTAT_ENTRY(A57_CC6_ENTRIES),
	CSTAT_ENTRY(A57_CC7_ENTRIES),
	CSTAT_ENTRY(D15_CC6_ENTRIES),
	CSTAT_ENTRY(D15_CC7_ENTRIES),
	CSTAT_ENTRY(D15_0_C6_ENTRIES),
	CSTAT_ENTRY(D15_1_C6_ENTRIES),
	CSTAT_ENTRY(D15_0_C7_ENTRIES),
	CSTAT_ENTRY(D15_1_C7_ENTRIES),
	CSTAT_ENTRY(A57_0_C7_ENTRIES),
	CSTAT_ENTRY(A57_1_C7_ENTRIES),
	CSTAT_ENTRY(A57_2_C7_ENTRIES),
	CSTAT_ENTRY(A57_3_C7_ENTRIES),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_D15_0),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_D15_1),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_A57_0),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_A57_1),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_A57_2),
	CSTAT_ENTRY(LAST_CSTATE_ENTRY_A57_3),
};

int tegra18x_mce_dbg_cstats_show(struct seq_file *s, void *data)
{
	int st;
	u64 val;

	seq_printf(s, "%-30s%-10s\n", "name", "count");
	seq_puts(s, "----------------------------------------\n");
	for (st = 1; st <= TEGRA_ARI_CSTATE_STATS_MAX; st++) {
		if (!cstats_table[st])
			continue;
		if (tegra18x_mce_read_cstate_stats(st, &val))
			pr_err("mce: failed to read cstat: %d\n", st);
		else
			seq_printf(s, "%-30s%-10lld\n", cstats_table[st], val);
	}

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

/* Tegra18x Cache functions */
__always_inline int tegra18x_roc_flush_cache(void)
{
	struct tegra_mce_regs regs;

	return send_smc(MCE_SMC_ROC_FLUSH_CACHE, &regs);
}

__always_inline int tegra18x_roc_flush_cache_only(void)
{
	struct tegra_mce_regs regs;

	return send_smc(MCE_SMC_ROC_FLUSH_CACHE_ONLY, &regs);
}

__always_inline int tegra18x_roc_clean_cache(void)
{
	struct tegra_mce_regs regs;

	return send_smc(MCE_SMC_ROC_CLEAN_CACHE_ONLY, &regs);
}
