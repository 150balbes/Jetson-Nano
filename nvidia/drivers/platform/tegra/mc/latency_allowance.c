/*
 * arch/arm/mach-tegra/latency_allowance.c
 *
 * Copyright (C) 2011-2020, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/err.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/stringify.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/syscore_ops.h>
#include <linux/platform/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/chip-id.h>
#include <asm/bug.h>
#include <asm/io.h>
#include <asm/string.h>

#include <linux/platform/tegra/mc-regs-t21x.h>
#include <linux/platform/tegra/latency_allowance.h>
#include <linux/platform/tegra/mc.h>

#include "la_priv.h"

#define TEST_LA_CODE		0
/* Bug 995270 */
#define HACK_LA_FIFO 1
static int default_set_la(enum tegra_la_id id, unsigned int bw_mbps);

static struct la_chip_specific cs;
module_param_named(disable_la, cs.disable_la, bool, S_IRUGO | S_IWUSR);
module_param_named(disable_ptsa, cs.disable_ptsa, bool, S_IRUGO | S_IWUSR);
module_param_named(disable_disp_ptsa,
	cs.disable_disp_ptsa, bool, S_IRUGO | S_IWUSR);
module_param_named(disable_bbc_ptsa,
	cs.disable_bbc_ptsa, bool, S_IRUGO | S_IWUSR);

#ifdef CONFIG_DEBUG_FS
static int la_ptsa_debugfs_init(void);
#endif

static void init_chip_specific(void)
{
	int cid;

	if (!tegra_platform_is_silicon())
		return;

	cs.set_init_la = default_set_la;
	memset(&cs.id_to_index[0], 0xFF, sizeof(cs.id_to_index));
	spin_lock_init(&cs.lock);

	cid = tegra_get_chip_id();

	switch (cid) {
	case TEGRA194:
		tegra_la_get_t19x_specific(&cs);
		break;
	case TEGRA186:
		tegra_la_get_t18x_specific(&cs);
		break;
	case TEGRA210:
		tegra_la_get_t21x_specific(&cs);
		break;
	default:
		cs.set_init_la = NULL;
	}
#ifdef CONFIG_DEBUG_FS
	la_ptsa_debugfs_init();
#endif
}

struct la_to_dc_params tegra_get_la_to_dc_params(void)
{
	return cs.la_params;
}

static void set_la(struct la_client_info *ci, int la)
{
	unsigned long reg_read;
	unsigned long reg_write;
	int idx = cs.id_to_index[ci->id];

	spin_lock(&cs.lock);
	reg_read = mc_readl(ci->reg_addr);
	reg_write = (reg_read & ~ci->mask) |
			(la << ci->shift);
	mc_writel(reg_write, ci->reg_addr);
	cs.scaling_info[idx].la_set = la;
	ci->la_set = la;
	la_debug("name=%s, reg=0x%x, read=0x%x, write=0x%x\n", ci->name,
		(u32)ci->reg_addr, (u32)reg_read, (u32)reg_write);
	spin_unlock(&cs.lock);
}

static int default_set_la(enum tegra_la_id id, unsigned int bw_mbps)
{
	int ideal_la;
	int la_to_set;
	unsigned int fifo_size_in_atoms;
	int bytes_per_atom = cs.atom_size;
	const int fifo_scale = 4;		/* 25% of the FIFO */
	struct la_client_info *ci;
	int idx = cs.id_to_index[id];

	if (!tegra_platform_is_silicon())
		return 0;

	VALIDATE_ID(id, &cs);
	VALIDATE_BW(bw_mbps);

	ci = &cs.la_info_array[idx];
	fifo_size_in_atoms = ci->fifo_size_in_atoms;

#ifdef CONFIG_TEGRA_MC_PTSA
	if (id >= TEGRA_LA_DISPLAY_0A && id <= TEGRA_LA_DISPLAY_HCB) {
		cs.disp_bw_array[id - TEGRA_LA_DISPLAY_0A] = bw_mbps;
		if (cs.update_display_ptsa_rate)
			cs.update_display_ptsa_rate(cs.disp_bw_array);
	}
#endif
#if HACK_LA_FIFO
	/* pretend that our FIFO is only as deep as the lowest fullness
	 * we expect to see */
	if (id >= ID(DISPLAY_0A) && id <= ID(DISPLAY_HCB))
		fifo_size_in_atoms /= fifo_scale;
#endif

	if (bw_mbps == 0) {
		la_to_set = cs.la_max_value;
	} else {
		ideal_la = (fifo_size_in_atoms * bytes_per_atom * 1000) /
			   (bw_mbps * cs.ns_per_tick);
		la_to_set = ideal_la -
				(ci->expiration_in_ns / cs.ns_per_tick) - 1;
	}

	la_debug("\n%s:id=%d,idx=%d, bw=%dmbps, la_to_set=%d\n",
		__func__, id, idx, bw_mbps, la_to_set);
	la_to_set = (la_to_set < 0) ? 0 : la_to_set;
	cs.scaling_info[idx].actual_la_to_set = la_to_set;
	la_to_set = (la_to_set > cs.la_max_value) ? cs.la_max_value : la_to_set;

	set_la(ci, la_to_set);
	return 0;
}

static void program_scaled_la(struct la_client_info *ci, int la)
{
	if (tegra_get_chip_id() == TEGRA210)
		program_scaled_la_t21x(ci, la);
}

void program_la(struct la_client_info *ci, int la)
{
	u32 reg_read;
	u32 reg_write;

	if (la > cs.la_max_value) {
		pr_err("la > cs.la_max_value\n");
		WARN_ON(1);
		return;
	}

	spin_lock(&cs.lock);
	reg_read = mc_readl(ci->reg_addr);
	reg_write = (reg_read & ~ci->mask) |
			(la << ci->shift);
	mc_writel(reg_write, ci->reg_addr);
	ci->la_set = la;
	la_debug("name=%s, reg_addr=0x%x, read=0x%x, write=0x%x\n", ci->name,
		(u32)(uintptr_t)ci->reg_addr, (u32)reg_read, (u32)reg_write);

	program_scaled_la(ci, la);

	spin_unlock(&cs.lock);
}

int la_suspend(void)
{
	int i = 0;
	struct la_client_info *ci = NULL;

	/* stashing LA and PTSA from registers is necessary
	 * in order to get latest values programmed by DVFS.
	 */
	for (i = 0; i < cs.la_info_array_size; i++) {
		ci = &cs.la_info_array[i];
		ci->la_set = (mc_readl(ci->reg_addr) & ci->mask) >>
				ci->shift;
	}

	cs.save_ptsa();

	if (cs.save_non_la_ptsa)
		cs.save_non_la_ptsa();

	return 0;
}

void la_resume(void)
{
	int i;

	for (i = 0; i < cs.la_info_array_size; i++) {
		if (cs.la_info_array[i].la_set)
			program_la(&cs.la_info_array[i],
					cs.la_info_array[i].la_set);
	}

	cs.program_ptsa();

	if (cs.program_non_la_ptsa)
		cs.program_non_la_ptsa();
}

int tegra_set_disp_latency_allowance(enum tegra_la_id id,
					unsigned long emc_freq_hz,
					unsigned int bw_mbps,
					struct dc_to_la_params disp_params) {
	if (cs.set_disp_la)
		return cs.set_disp_la(id, emc_freq_hz, bw_mbps, disp_params);
	else if (cs.set_dynamic_la)
		return cs.set_dynamic_la(id, bw_mbps);
	return 0;
}

/*
 * Check if the passed bandwidth is possible.
 *
 * Returns zero if there is a possible LA value that can satifsy @bw_mbps at
 * @emc_freq_hz. If no function has been defined for the active chip then this
 * this function returns true (i.e 0).
 */
int tegra_check_disp_latency_allowance(enum tegra_la_id id,
					   unsigned long emc_freq_hz,
					   unsigned int bw_mbps,
					   struct dc_to_la_params disp_params) {
	if (cs.check_disp_la)
		return cs.check_disp_la(id, emc_freq_hz, bw_mbps, disp_params);
	return 0;
}

/* Sets latency allowance based on clients memory bandwitdh requirement.
 * Bandwidth passed is in mega bytes per second.
 */
int tegra_set_latency_allowance(enum tegra_la_id id, unsigned int bw_mbps)
{
	if (cs.set_dynamic_la)
		return cs.set_dynamic_la(id, bw_mbps);
	return 0;
}
EXPORT_SYMBOL(tegra_set_latency_allowance);

int tegra_set_camera_ptsa(enum tegra_la_id id,
			unsigned int bw_mbps,
			int is_hiso)
{
	if (cs.update_camera_ptsa_rate)
		return cs.update_camera_ptsa_rate(id, bw_mbps, is_hiso);
	else if (cs.set_dynamic_la)
		return cs.set_dynamic_la(id, bw_mbps);
	return 0;
}

/* Thresholds for scaling are specified in % of fifo freeness.
 * If threshold_low is specified as 20%, it means when the fifo free
 * between 0 to 20%, use la as programmed_la.
 * If threshold_mid is specified as 50%, it means when the fifo free
 * between 20 to 50%, use la as programmed_la/2 .
 * If threshold_high is specified as 80%, it means when the fifo free
 * between 50 to 80%, use la as programmed_la/4.
 * When the fifo is free between 80 to 100%, use la as 0(highest priority).
 */
int tegra_enable_latency_scaling(enum tegra_la_id id,
					unsigned int threshold_low,
					unsigned int threshold_mid,
					unsigned int threshold_high)
{
	if (cs.enable_la_scaling)
		return cs.enable_la_scaling(id, threshold_low,
			threshold_mid, threshold_high);
	return 0;
}

void tegra_disable_latency_scaling(enum tegra_la_id id)
{
	if (cs.disable_la_scaling) {
		cs.disable_la_scaling(id);
	}
}

void tegra_latency_allowance_update_tick_length(unsigned int new_ns_per_tick)
{
	int i = 0;
	int la;
	unsigned long reg_read;
	unsigned long reg_write;
	unsigned long scale_factor = new_ns_per_tick / cs.ns_per_tick;

	if (scale_factor > 1) {
		spin_lock(&cs.lock);
		cs.ns_per_tick = new_ns_per_tick;
		for (i = 0; i < cs.la_info_array_size - 1; i++) {
			reg_read = mc_readl(cs.la_info_array[i].reg_addr);
			la = ((reg_read & cs.la_info_array[i].mask) >>
				cs.la_info_array[i].shift) / scale_factor;

			reg_write = (reg_read & ~cs.la_info_array[i].mask) |
					(la << cs.la_info_array[i].shift);
			mc_writel(reg_write, cs.la_info_array[i].reg_addr);
			cs.scaling_info[i].la_set = la;
		}
		spin_unlock(&cs.lock);
	}
}

static int la_regs_show(struct seq_file *s, void *unused)
{
	int i;
	unsigned long la;

	/* iterate the list, but don't print MAX_ID */
	for (i = 0; i < cs.la_info_array_size - 1; i++) {
		la = (mc_readl(cs.la_info_array[i].reg_addr) &
			cs.la_info_array[i].mask) >> cs.la_info_array[i].shift;
		seq_printf(s, "%-16s: %4lu\n", cs.la_info_array[i].name, la);
	}

	return 0;
}

static int dbg_la_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, la_regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open           = dbg_la_regs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init tegra_latency_allowance_debugfs_init(void)
{
	if (cs.latency_debug_dir)
		return 0;

	cs.latency_debug_dir = debugfs_create_dir("tegra_latency", NULL);

	debugfs_create_file("la_info", S_IRUGO, cs.latency_debug_dir, NULL,
		&regs_fops);

	return 0;
}

static int tegra_la_suspend(void)
{
	if (cs.suspend)
		return cs.suspend();
	return 0;
}

static void tegra_la_resume(void)
{
	int i;

	if (cs.resume) {
		cs.resume();
		return;
	}
	for (i = 0; i < cs.la_info_array_size; i++) {
		if (cs.la_info_array[i].la_set)
			set_la(&cs.la_info_array[i],
				cs.la_info_array[i].la_set);
	}
	if (cs.init_ptsa)
		cs.init_ptsa();
}

static struct syscore_ops tegra_la_syscore_ops = {
	.suspend = tegra_la_suspend,
	.resume = tegra_la_resume,
};

static __init int tegra_la_syscore_init(void)
{
	register_syscore_ops(&tegra_la_syscore_ops);
	return 0;
}

static int __init tegra_latency_allowance_init(void)
{
	unsigned int i;
	int ret = 0;

	init_chip_specific();

	for (i = 0; i < cs.la_info_array_size; i++)
		cs.id_to_index[cs.la_info_array[i].id] = i;

	for (i = 0; i < cs.la_info_array_size; i++) {
		if (cs.set_init_la) {
			ret = cs.set_init_la(cs.la_info_array[i].id, 0);
			if (ret < 0) {
				if (cs.la_cleanup)
					cs.la_cleanup();
				return -1;
			}
		} else if (cs.la_info_array[i].init_la) {
			set_la(&cs.la_info_array[i],
				cs.la_info_array[i].init_la);
		}
	}

	if (cs.init_ptsa)
		cs.init_ptsa();

	pr_info("la/ptsa driver initialized.\n");
	return 0;
}

late_initcall(tegra_latency_allowance_debugfs_init);
subsys_initcall(tegra_la_syscore_init);

/* Must happen after MC init which is done by device tree. */
fs_initcall(tegra_latency_allowance_init);

static void __exit tegra_latency_allowance_exit(void)
{
	if (cs.la_cleanup)
		cs.la_cleanup();
}
module_exit(tegra_latency_allowance_exit);

/* Must be called after LA/PTSA init */
void mc_pcie_init(void)
{
	if (cs.mc_pcie_init)
		cs.mc_pcie_init();
}
EXPORT_SYMBOL(mc_pcie_init);

#ifdef CONFIG_DEBUG_FS

#include <linux/debugfs.h>
#include <linux/uaccess.h>

static unsigned long debugfs_display_emc_freq_hz;
static unsigned int debugfs_display_bw_mbps;
static int debugfs_camera_la_id;
static unsigned int debugfs_camera_bw_mbps;
static int debugfs_camera_is_hiso;
static int debugfs_other_la_id;
static unsigned int debugfs_other_bw_mbps;

static int display_emc_freq_hz_get(void *data, u64 *val)
{
	*val = (u64) debugfs_display_emc_freq_hz;
	return 0;
}

static int display_emc_freq_hz_set(void *data, u64 val)
{
	debugfs_display_emc_freq_hz = val;
	return 0;
}

static int display_bw_mbps_get(void *data, u64 *val)
{
	*val = (u64) debugfs_display_bw_mbps;
	return 0;
}

static int display_bw_mbps_set(void *data, u64 val)
{
	debugfs_display_bw_mbps = (u32) val;
	return 0;
}

static int display_set_la_ptsa_set(void *data, u64 val)
{
	struct dc_to_la_params disp_params = {0};

	if (cs.set_disp_la(ID(NVDISPLAYR),
				debugfs_display_emc_freq_hz,
				debugfs_display_bw_mbps,
				disp_params))
		return -1;
	return 0;
}

static int camera_la_id_get(void *data, u64 *val)
{
	*val = (u64) debugfs_camera_la_id;
	return 0;
}

static int camera_la_id_set(void *data, u64 val)
{
	debugfs_camera_la_id = (u32) val;
	return 0;
}

static int camera_bw_mbps_get(void *data, u64 *val)
{
	*val = (u64) debugfs_camera_bw_mbps;
	return 0;
}

static int camera_bw_mbps_set(void *data, u64 val)
{
	debugfs_camera_bw_mbps = (u32) val;
	return 0;
}

static int camera_is_hiso_get(void *data, u64 *val)
{
	*val = (u64) debugfs_camera_is_hiso;
	return 0;
}

static int camera_is_hiso_set(void *data, u64 val)
{
	debugfs_camera_is_hiso = (u32) val;
	return 0;
}

static int camera_set_ptsa_set(void *data, u64 val)
{
	if (cs.update_camera_ptsa_rate(debugfs_camera_la_id,
				debugfs_camera_bw_mbps,
				debugfs_camera_is_hiso))
		return -1;
	return 0;
}

static int other_la_id_get(void *data, u64 *val)
{
	*val = (u64) debugfs_other_la_id;
	return 0;
}

static int other_la_id_set(void *data, u64 val)
{
	debugfs_other_la_id = (u32) val;
	return 0;
}

static int other_bw_mbps_get(void *data, u64 *val)
{
	*val = (u64) debugfs_other_bw_mbps;
	return 0;
}

static int other_bw_mbps_set(void *data, u64 val)
{
	debugfs_other_bw_mbps = (u32) val;
	return 0;
}

static int other_set_la_ptsa_set(void *data, u64 val)
{
	if (cs.set_dynamic_la(debugfs_other_la_id,
				debugfs_other_bw_mbps))
		return -1;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(display_emc_freq_hz_fops,
		display_emc_freq_hz_get,
		display_emc_freq_hz_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(display_bw_mbps_fops,
		display_bw_mbps_get,
		display_bw_mbps_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(display_set_la_ptsa_fops,
		NULL,
		display_set_la_ptsa_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(camera_la_id_fops,
		camera_la_id_get,
		camera_la_id_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(camera_bw_mbps_fops,
		camera_bw_mbps_get,
		camera_bw_mbps_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(camera_is_hiso_fops,
		camera_is_hiso_get,
		camera_is_hiso_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(camera_set_ptsa_fops,
		NULL,\
		camera_set_ptsa_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(other_la_id_fops,
		other_la_id_get,
		other_la_id_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(other_bw_mbps_fops,
		other_bw_mbps_get,
		other_bw_mbps_set,
		"%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(other_set_la_ptsa_fops,
		NULL,
		other_set_la_ptsa_set,
		"%llu\n");

static int la_ptsa_debugfs_init(void)
{
	struct dentry *la_ptsa_debugfs_root;
	struct dentry *display_dir;
	struct dentry *camera_dir;
	struct dentry *other_dir;

	la_ptsa_debugfs_root = debugfs_create_dir("tegra_la_ptsa", NULL);
	if (!la_ptsa_debugfs_root) {
		pr_err("%s: Couldn't create the LA\\PTSA root debugfs node.\n",
				__func__);
		return -1;
	}

	/* Display nodes*/
	display_dir = debugfs_create_dir("display", la_ptsa_debugfs_root);
	if (!display_dir) {
		pr_err("%s: Couldn't create the \"display\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("emc_freq_hz",
				S_IRUGO | S_IWUSR,
				display_dir,
				NULL,
				&display_emc_freq_hz_fops)) {
		pr_err("%s: Couldn't create the display \"emc_freq_hz\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("bw_mbps",
				S_IRUGO | S_IWUSR,
				display_dir,
				NULL,
				&display_bw_mbps_fops)) {
		pr_err("%s: Couldn't create the display \"bw_mbps\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("set_la_ptsa",
				S_IWUSR,
				display_dir,
				NULL,
				&display_set_la_ptsa_fops)) {
		pr_err("%s: Couldn't create the display \"set_la_ptsa\" debugfs node.\n",
				__func__);
		return -1;
	}

	/* Camera nodes*/
	camera_dir = debugfs_create_dir("camera", la_ptsa_debugfs_root);
	if (!camera_dir) {
		pr_err("%s: Couldn't create the \"camera\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("la_id",
				S_IRUGO | S_IWUSR,
				camera_dir,
				NULL,
				&camera_la_id_fops)) {
		pr_err("%s: Couldn't create the camera \"la_id\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("bw_mbps",
				S_IRUGO | S_IWUSR,
				camera_dir,
				NULL,
				&camera_bw_mbps_fops)) {
		pr_err("%s: Couldn't create the camera \"bw_mbps\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("is_hiso",
				S_IRUGO | S_IWUSR,
				camera_dir,
				NULL,
				&camera_is_hiso_fops)) {
		pr_err("%s: Couldn't create the camera \"is_hiso\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("set_ptsa",
				S_IWUSR,
				camera_dir,
				NULL,
				&camera_set_ptsa_fops)) {
		pr_err("%s: Couldn't create the camera \"set_ptsa\" debugfs node.\n",
				__func__);
		return -1;
	}

	/* Non-display nodes */
	other_dir = debugfs_create_dir("other", la_ptsa_debugfs_root);
	if (!other_dir) {
		pr_err("%s: Couldn't create the \"other\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("la_id",
				S_IRUGO | S_IWUSR,
				other_dir,
				NULL,
				&other_la_id_fops)) {
		pr_err("%s: Couldn't create the other \"la_id\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("bw_mbps",
				S_IRUGO | S_IWUSR,
				other_dir,
				NULL,
				&other_bw_mbps_fops)) {
		pr_err("%s: Couldn't create the other \"bw_mbps\" debugfs node.\n",
				__func__);
		return -1;
	}
	if (!debugfs_create_file("set_la_ptsa",
				S_IWUSR,
				other_dir,
				NULL,
				&other_set_la_ptsa_fops)) {
		pr_err("%s: Couldn't create the other \"set_la_ptsa\" debugfs node.\n",
				__func__);
		return -1;
	}

	return 0;
}

#endif // "CONFIG_DEBUG_FS"


#if TEST_LA_CODE
#define PRINT_ID_IDX_MAPPING 0
static int __init test_la(void)
{
	int i;
	int err;
	enum tegra_la_id id = 0;
	int repeat_count = 5;

#if PRINT_ID_IDX_MAPPING
	for (i = 0; i < ID(MAX_ID); i++)
		pr_info("ID=0x%x, Idx=0x%x", i, cs.id_to_index[i]);
#endif

	do {
		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			err = tegra_set_latency_allowance(id, 200);
			if (err)
				la_debug("\n***tegra_set_latency_allowance,"
						" err=%d", err);
		}

		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			if (id >= ID(DISPLAY_0AB) && id <= ID(DISPLAY_HCB))
				continue;
			if (id >= ID(VI_WSB) && id <= ID(VI_WY))
				continue;
			err = tegra_enable_latency_scaling(id, 20, 50, 80);
			if (err)
				la_debug("\n***tegra_enable_latency_scaling,"
						" err=%d", err);
		}

		la_debug("la_scaling_enable_count =%d",
				cs.la_scaling_enable_count);
		for (id = 0; id < TEGRA_LA_MAX_ID; id++) {
			if (id >= ID(DISPLAY_0AB) && id <= ID(DISPLAY_HCB))
				continue;
			if (id >= ID(VI_WSB) && id <= ID(VI_WY))
				continue;
			tegra_disable_latency_scaling(id);
		}
		la_debug("la_scaling_enable_count=%d",
				cs.la_scaling_enable_count);
	} while (--repeat_count);
	return 0;
}

late_initcall(test_la);
#endif
