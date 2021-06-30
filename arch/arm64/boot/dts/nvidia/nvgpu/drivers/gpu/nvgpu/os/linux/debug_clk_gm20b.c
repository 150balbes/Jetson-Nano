/*
 * Copyright (C) 2017-2018 NVIDIA Corporation.  All rights reserved.
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

#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <nvgpu/io.h>
#include <nvgpu/clk_arb.h>

#include "gm20b/clk_gm20b.h"
#include "os_linux.h"
#include "platform_gk20a.h"

static int rate_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	struct clk_gk20a *clk = &g->clk;

	*val = (u64)rate_gpc2clk_to_gpu(clk->gpc_pll.freq);
	return 0;
}
static int rate_set(void *data, u64 val)
{
	struct gk20a *g = (struct gk20a *)data;
	if (nvgpu_clk_arb_has_active_req(g))
		return 0;
	return g->ops.clk.set_rate(g, CTRL_CLK_DOMAIN_GPCCLK, (u32)val);
}
DEFINE_SIMPLE_ATTRIBUTE(rate_fops, rate_get, rate_set, "%llu\n");

static int pll_reg_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct nvgpu_clk_pll_debug_data d;
	u32 reg, m, n, pl, f;
	int err = 0;

	if (g->ops.clk.get_pll_debug_data) {
		err = g->ops.clk.get_pll_debug_data(g, &d);
		if (err)
			return err;
	} else {
		return -EINVAL;
	}

	seq_printf(s, "bypassctrl = %s, ",
			d.trim_sys_bypassctrl_val ? "bypass" : "vco");
	seq_printf(s, "sel_vco = %s, ",
			d.trim_sys_sel_vco_val ? "vco" : "bypass");

	seq_printf(s, "cfg  = 0x%x : %s : %s : %s\n", d.trim_sys_gpcpll_cfg_val,
		d.trim_sys_gpcpll_cfg_enabled ? "enabled" : "disabled",
		d.trim_sys_gpcpll_cfg_locked ? "locked" : "unlocked",
		d.trim_sys_gpcpll_cfg_sync_on ? "sync_on" : "sync_off");

	reg = d.trim_sys_gpcpll_coeff_val;
	m = d.trim_sys_gpcpll_coeff_mdiv;
	n = d.trim_sys_gpcpll_coeff_ndiv;
	pl = d.trim_sys_gpcpll_coeff_pldiv;
	f = g->clk.gpc_pll.clk_in * n / (m * nvgpu_pl_to_div(pl));
	seq_printf(s, "coef = 0x%x : m = %u : n = %u : pl = %u", reg, m, n, pl);
	seq_printf(s, " : pll_f(gpu_f) = %u(%u) kHz\n", f, f/2);

	seq_printf(s, "dvfs0 = 0x%x : d = %u : dmax = %u : doffs = %u\n",
		d.trim_sys_gpcpll_dvfs0_val,
		d.trim_sys_gpcpll_dvfs0_dfs_coeff,
		d.trim_sys_gpcpll_dvfs0_dfs_det_max,
		d.trim_sys_gpcpll_dvfs0_dfs_dc_offset);

	return 0;
}

static int pll_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_reg_show, inode->i_private);
}

static const struct file_operations pll_reg_fops = {
	.open		= pll_reg_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int pll_reg_raw_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct nvgpu_clk_pll_debug_data d;
	u32 reg;
	int err = 0;

	if (g->ops.clk.get_pll_debug_data) {
		err = g->ops.clk.get_pll_debug_data(g, &d);
		if (err)
			return err;
	} else {
		return -EINVAL;
	}

	seq_puts(s, "GPCPLL REGISTERS:\n");
	for (reg = d.trim_sys_gpcpll_cfg_reg;
	     reg < d.trim_sys_gpcpll_dvfs2_reg;
	     reg += sizeof(u32))
		seq_printf(s, "[0x%02x] = 0x%08x\n", reg, gk20a_readl(g, reg));

	reg = d.trim_bcast_gpcpll_dvfs2_reg;
	if (reg)
		seq_printf(s, "[0x%02x] = 0x%08x\n", reg, gk20a_readl(g, reg));

	seq_puts(s, "\nGPC CLK OUT REGISTERS:\n");

	seq_printf(s, "[0x%02x] = 0x%08x\n", d.trim_sys_sel_vco_reg,
					     d.trim_sys_sel_vco_val);
	seq_printf(s, "[0x%02x] = 0x%08x\n", d.trim_sys_gpc2clk_out_reg,
					     d.trim_sys_gpc2clk_out_val);
	seq_printf(s, "[0x%02x] = 0x%08x\n", d.trim_sys_bypassctrl_reg,
					     d.trim_sys_bypassctrl_val);

	return 0;
}

static int pll_reg_raw_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_reg_raw_show, inode->i_private);
}

static ssize_t pll_reg_raw_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct gk20a *g = file->f_path.dentry->d_inode->i_private;
	char buf[80];
	u32 reg, val;
	int err = 0;

	if (sizeof(buf) <= count)
		return -EINVAL;

	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count] = '\0';
	strim(buf);

	if (sscanf(buf, "[0x%x] = 0x%x", &reg, &val) != 2)
		return -EINVAL;

	if (g->ops.clk.pll_reg_write(g, reg, val))
		err = g->ops.clk.pll_reg_write(g, reg, val);
	else
		err = -EINVAL;

	return err;
}

static const struct file_operations pll_reg_raw_fops = {
	.open		= pll_reg_raw_open,
	.read		= seq_read,
	.write		= pll_reg_raw_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int monitor_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	int err = 0;

	if (g->ops.clk.get_gpcclk_clock_counter)
		err = g->ops.clk.get_gpcclk_clock_counter(&g->clk, val);
	else
		err = -EINVAL;

	return err;
}
DEFINE_SIMPLE_ATTRIBUTE(monitor_fops, monitor_get, NULL, "%llu\n");

static int voltage_get(void *data, u64 *val)
{
	struct gk20a *g = (struct gk20a *)data;
	int err = 0;

	if (g->ops.clk.get_voltage)
		err = g->ops.clk.get_voltage(&g->clk, val);
	else
		err = -EINVAL;

	return err;
}
DEFINE_SIMPLE_ATTRIBUTE(voltage_fops, voltage_get, NULL, "%llu\n");

static int pll_param_show(struct seq_file *s, void *data)
{
	struct pll_parms *gpc_pll_params = gm20b_get_gpc_pll_parms();

	seq_printf(s, "ADC offs = %d uV, ADC slope = %d uV, VCO ctrl = 0x%x\n",
		   gpc_pll_params->uvdet_offs, gpc_pll_params->uvdet_slope,
		   gpc_pll_params->vco_ctrl);
	return 0;
}

static int pll_param_open(struct inode *inode, struct file *file)
{
	return single_open(file, pll_param_show, inode->i_private);
}

static const struct file_operations pll_param_fops = {
	.open		= pll_param_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int gm20b_clk_init_debugfs(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *d;

	if (!l->debugfs)
		return -EINVAL;

	d = debugfs_create_file(
		"rate", S_IRUGO|S_IWUSR, l->debugfs, g, &rate_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"pll_reg", S_IRUGO, l->debugfs, g, &pll_reg_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file("pll_reg_raw",
		S_IRUGO, l->debugfs, g, &pll_reg_raw_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"monitor", S_IRUGO, l->debugfs, g, &monitor_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"voltage", S_IRUGO, l->debugfs, g, &voltage_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"pll_param", S_IRUGO, l->debugfs, g, &pll_param_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_u32("pll_na_mode", S_IRUGO, l->debugfs,
			       (u32 *)&g->clk.gpc_pll.mode);
	if (!d)
		goto err_out;

	d = debugfs_create_u32("fmax2x_at_vmin_safe_t", S_IRUGO,
		       l->debugfs, (u32 *)&g->clk.dvfs_safe_max_freq);
	if (!d)
		goto err_out;

	return 0;

err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	return -ENOMEM;
}
