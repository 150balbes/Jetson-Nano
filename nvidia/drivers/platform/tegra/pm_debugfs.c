/*
 * drivers/platform/tegra/pm_debugfs.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
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

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/tegra-pm.h>

/* Debugfs handle for /d/system_states directory */
static struct dentry *system_states_debugfs;

/* Suspend debug flags used in /d/system_states/suspend_debug_flags */
static u32 suspend_debug_flags;

/*
 * Helper function for send_smc that actually makes the smc call
 */
static noinline notrace int __send_smc(u32 smc_func, struct pm_regs *regs)
{
	u32 ret = smc_func;

	asm volatile (
	"       mov     x0, %0\n"
	"       ldp     x1, x2, [%1, #16 * 0]\n"
	"       ldp     x3, x4, [%1, #16 * 1]\n"
	"       ldp     x5, x6, [%1, #16 * 2]\n"
	"       isb\n"
	"       smc     #0\n"
	"       mov     %0, x0\n"
	"       stp     x0, x1, [%1, #16 * 0]\n"
	"       stp     x2, x3, [%1, #16 * 1]\n"
	: "+r" (ret)
	: "r" (regs)
	: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7", "x8",
	"x9", "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17");
	return ret;
}

/*
 * Make an SMC call. Takes in the SMC function to be invoked & registers to be
 * passed along as args.
 */
int send_smc(u32 smc_func, struct pm_regs *regs)
{
	int __ret = __send_smc(smc_func, regs);

	if (__ret) {
		pr_err("%s: failed (ret=%d)\n", __func__, __ret);
		return __ret;
	}

	return __ret;
}

/*
 * Specify debug flags for system suspend.
 */
static int tegra_set_suspend_debug_flags(u32 debug_flags)
{
	struct pm_regs regs;
	u32 smc_func = SMC_FAKE_SYS_SUSPEND |
				(FAKE_SYSTEM_SUSPEND_MODE & SMC_ENUM_MAX);
	regs.args[0] = debug_flags;
	return send_smc(smc_func, &regs);
}
EXPORT_SYMBOL(tegra_set_suspend_debug_flags);

/*
 * Get suspend debug flags. It is used by debugfs ops.
 */
static int suspend_debug_flags_get(void *data, u64 *val)
{
	*val = suspend_debug_flags;
	return 0;
}

/*
 * Set suspend debug flags. It is used by debugfs ops.
 */
static int suspend_debug_flags_set(void *data, u64 val)
{
	int ret;

	if (val == FAKE_SYSTEM_SUSPEND_USER_ARG) {
		suspend_debug_flags = FAKE_SYSTEM_SUSPEND_MODE;
		ret = tegra_set_suspend_debug_flags(suspend_debug_flags);
	} else {
		pr_err("Invalid suspend debug flags\n");
		ret = -1;
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(suspend_debug_flags_fops,
		suspend_debug_flags_get, suspend_debug_flags_set, "%llu\n");

/*
 * Interface that returns the debugfs handle for /d/system_states directory.
 * In case its not initialized, it creates it.
 */
struct dentry *return_system_states_dir(void)
{
	if (system_states_debugfs == NULL) {
		system_states_debugfs =
				debugfs_create_dir("system_states", NULL);

		if (system_states_debugfs == NULL)
			pr_err("Cannot create system_states debugfs dir\n");
	}

	return system_states_debugfs;
}
EXPORT_SYMBOL(return_system_states_dir);

static int __init fake_system_suspend_debugfs_init(void)
{
	struct dentry *dfs_file = NULL;
	struct dentry *debugfs_dir = return_system_states_dir();

	if (!debugfs_dir) {
		pr_err("/d/system_states was not created. Aborting\n");
		return -ENOENT;
	}

	dfs_file = debugfs_create_file("suspend_debug_flags", 0644,
			debugfs_dir, NULL, &suspend_debug_flags_fops);

	if (!dfs_file) {
		pr_err("Not able to create suspend_debug_flags debugfs node\n");
		return -ENOENT;
	}

	return 0;
}

late_initcall(fake_system_suspend_debugfs_init);

