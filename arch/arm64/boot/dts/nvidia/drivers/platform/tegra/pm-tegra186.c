/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/tegra-pm.h>

static u32 shutdown_state;

#define SMC_PM_FUNC	0xC2FFFE00
#define SMC_SET_SHUTDOWN_MODE 0x1
#define SYSTEM_SHUTDOWN_STATE_FULL_POWER_OFF 0
#define SYSTEM_SHUTDOWN_STATE_SC8 8
#define SMC_GET_CLK_COUNT 0x2

/**
 * Specify state for SYSTEM_SHUTDOWN
 *
 * @shutdown_state:	Specific shutdown state to set
 *
 */
static int tegra_set_shutdown_mode(u32 shutdown_state)
{
	struct pm_regs regs;
	u32 smc_func = SMC_PM_FUNC | (SMC_SET_SHUTDOWN_MODE & SMC_ENUM_MAX);
	regs.args[0] = shutdown_state;
	return send_smc(smc_func, &regs);
}
EXPORT_SYMBOL(tegra_set_shutdown_mode);

/**
 * read core clk and ref clk counters under EL3
 *
 * @mpidr: MPIDR of target core
 * @midr: MIDR of target core
 * @coreclk: core clk counter
 * @refclk : ref clk counter
 *
 * Returns 0 if success.
 */
int tegra_get_clk_counter(u32 mpidr, u32 midr, u32 *coreclk,
	u32 *refclk)
{
	struct pm_regs regs;
	int ret;
	u32 smc_func = SMC_PM_FUNC | (SMC_GET_CLK_COUNT & SMC_ENUM_MAX);

	regs.args[0] = mpidr;
	regs.args[1] = midr;

	ret = send_smc(smc_func, &regs);

	*coreclk = (u32)regs.args[1];
	*refclk = (u32)regs.args[2];

	return ret;
}
EXPORT_SYMBOL(tegra_get_clk_counter);

static void tegra186_power_off_prepare(void)
{
	disable_nonboot_cpus();
}

static int __init tegra186_pm_init(void)
{
	pm_power_off_prepare = tegra186_power_off_prepare;

	return 0;
}
core_initcall(tegra186_pm_init);

static int shutdown_state_get(void *data, u64 *val)
{
	*val = shutdown_state;
	return 0;
}

static int shutdown_state_set(void *data, u64 val)
{
	int ret;
	if (val == SYSTEM_SHUTDOWN_STATE_FULL_POWER_OFF ||
					val == SYSTEM_SHUTDOWN_STATE_SC8) {
		shutdown_state = val;
		ret = tegra_set_shutdown_mode(shutdown_state);
	}
	else {
		printk("Invalid Shutdown state\n");
		ret = -1;
	}

	return ret;
}

DEFINE_SIMPLE_ATTRIBUTE(shutdown_state_fops, shutdown_state_get, shutdown_state_set, "%llu\n");

static int __init tegra18_suspend_debugfs_init(void)
{
	struct dentry *dfs_file, *system_state_debugfs;

	system_state_debugfs = return_system_states_dir();
	if (!system_state_debugfs)
		goto err_out;

	dfs_file = debugfs_create_file("shutdown", 0644,
					system_state_debugfs, NULL, &shutdown_state_fops);
	if (!dfs_file)
		goto err_out;

	return 0;

err_out:
	pr_err("%s: Couldn't create debugfs node for shutdown\n", __func__);
	return -ENOMEM;

}
module_init(tegra18_suspend_debugfs_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Tegra T18x Suspend Mode debugfs");
