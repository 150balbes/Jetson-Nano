/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/debugfs.h>

#include "os_linux.h"

#include "pmgr/pmgr.h"

static int pmgr_pwr_devices_get_power_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_power(g, &val);
	*p = val;

	return err;
}

static int pmgr_pwr_devices_get_current_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_current(g, &val);
	*p = val;

	return err;
}

static int pmgr_pwr_devices_get_voltage_u64(void *data, u64 *p)
{
	struct gk20a *g = (struct gk20a *)data;
	int err;
	u32 val;

	err = pmgr_pwr_devices_get_voltage(g, &val);
	*p = val;

	return err;
}

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_power_ctrl_fops, pmgr_pwr_devices_get_power_u64, NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_current_ctrl_fops, pmgr_pwr_devices_get_current_u64, NULL, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(
		pmgr_voltage_ctrl_fops, pmgr_pwr_devices_get_voltage_u64, NULL, "%llu\n");

static void pmgr_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *dbgentry;

	dbgentry = debugfs_create_file(
				"power", S_IRUGO, l->debugfs, g, &pmgr_power_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for power");

	dbgentry = debugfs_create_file(
				"current", S_IRUGO, l->debugfs, g, &pmgr_current_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for current");

	dbgentry = debugfs_create_file(
				"voltage", S_IRUGO, l->debugfs, g, &pmgr_voltage_ctrl_fops);
	if (!dbgentry)
		nvgpu_err(g, "debugfs entry create failed for voltage");
}

int nvgpu_pmgr_init_debugfs_linux(struct nvgpu_os_linux *l)
{
	struct gk20a *g = &l->g;
	int ret = 0;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PSTATE))
		return ret;

	if (!g->ops.clk.support_pmgr_domain)
		return ret;

	pmgr_debugfs_init(g);

	return ret;
}

