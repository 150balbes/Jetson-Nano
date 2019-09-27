/*
 * Copyright (c) 2018, NVIDIA Corporation. All rights reserved.
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

#include <nvgpu/clk.h>

#include "os_linux.h"

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock);

static int gp106_get_rate_show(void *data , u64 *val)
{
	struct namemap_cfg *c = (struct namemap_cfg *)data;
	struct gk20a *g = c->g;

	if (!g->ops.clk.get_rate_cntr)
		return -EINVAL;

	*val = c->is_counter ? (u64)c->scale * g->ops.clk.get_rate_cntr(g, c) :
		0 /* TODO PLL read */;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(get_rate_fops, gp106_get_rate_show, NULL, "%llu\n");

static int sys_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_SYS);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int sys_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
					CTRL_CLK_CLK_FREQ_CONTROLLER_ID_SYS);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(sys_cfc_fops, sys_cfc_read, sys_cfc_write, "%llu\n");

static int ltc_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_LTC);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int ltc_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
					CTRL_CLK_CLK_FREQ_CONTROLLER_ID_LTC);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(ltc_cfc_fops, ltc_cfc_read, ltc_cfc_write, "%llu\n");

static int xbar_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_XBAR);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int xbar_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
			CTRL_CLK_CLK_FREQ_CONTROLLER_ID_XBAR);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(xbar_cfc_fops, xbar_cfc_read,
			xbar_cfc_write, "%llu\n");

static int gpc_cfc_read(void *data , u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	bool bload = boardobjgrpmask_bitget(
		&g->clk_pmu.clk_freq_controllers.freq_ctrl_load_mask.super,
		CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0);

	/* val = 1 implies CLFC is loaded or enabled */
	*val = bload ? 1 : 0;
	return 0;
}
static int gpc_cfc_write(void *data , u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	int status;
	/* val = 1 implies load or enable the CLFC */
	bool bload = val ? true : false;

	nvgpu_clk_arb_pstate_change_lock(g, true);
	status = clk_pmu_freq_controller_load(g, bload,
			CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0);
	nvgpu_clk_arb_pstate_change_lock(g, false);

	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(gpc_cfc_fops, gpc_cfc_read, gpc_cfc_write, "%llu\n");

int gp106_clk_init_debugfs(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;
	struct dentry *clocks_root, *clk_freq_ctlr_root;
	struct dentry *d;
	unsigned int i;

	if (NULL == (clocks_root = debugfs_create_dir("clocks", gpu_root)))
		return -ENOMEM;

	clk_freq_ctlr_root = debugfs_create_dir("clk_freq_ctlr", gpu_root);
	if (clk_freq_ctlr_root == NULL)
		return -ENOMEM;

	d = debugfs_create_file("sys", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &sys_cfc_fops);
	d = debugfs_create_file("ltc", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &ltc_cfc_fops);
	d = debugfs_create_file("xbar", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &xbar_cfc_fops);
	d = debugfs_create_file("gpc", S_IRUGO | S_IWUSR, clk_freq_ctlr_root,
				g, &gpc_cfc_fops);

	nvgpu_log(g, gpu_dbg_info, "g=%p", g);

	for (i = 0; i < g->clk.namemap_num; i++) {
		if (g->clk.clk_namemap[i].is_enable) {
			d = debugfs_create_file(
				g->clk.clk_namemap[i].name,
				S_IRUGO,
				clocks_root,
				&g->clk.clk_namemap[i],
				&get_rate_fops);
			if (!d)
				goto err_out;
		}
	}
	return 0;

err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	debugfs_remove_recursive(clocks_root);
	return -ENOMEM;
}
