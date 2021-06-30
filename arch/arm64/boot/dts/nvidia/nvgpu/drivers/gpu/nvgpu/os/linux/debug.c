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

#include "debug_cde.h"
#include "debug_ce.h"
#include "debug_fifo.h"
#include "debug_gr.h"
#include "debug_allocator.h"
#include "debug_kmem.h"
#include "debug_pmu.h"
#include "debug_sched.h"
#include "debug_hal.h"
#include "debug_xve.h"
#include "debug_ltc.h"
#include "debug_bios.h"
#include "os_linux.h"
#include "platform_gk20a.h"

#include <nvgpu/gk20a.h>

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include <nvgpu/debug.h>

unsigned int gk20a_debug_trace_cmdbuf;

static inline void gk20a_debug_write_printk(void *ctx, const char *str,
					    size_t len)
{
	pr_info("%s", str);
}

static inline void gk20a_debug_write_to_seqfile(void *ctx, const char *str,
						size_t len)
{
	seq_write((struct seq_file *)ctx, str, len);
}

void gk20a_debug_output(struct gk20a_debug_output *o,
					const char *fmt, ...)
{
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(o->buf, sizeof(o->buf), fmt, args);
	va_end(args);
	o->fn(o->ctx, o->buf, len);
}

static int gk20a_gr_dump_regs(struct gk20a *g,
		struct gk20a_debug_output *o)
{
	if (g->ops.gr.dump_gr_regs)
		gr_gk20a_elpg_protected_call(g, g->ops.gr.dump_gr_regs(g, o));

	return 0;
}

int gk20a_gr_debug_dump(struct gk20a *g)
{
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	gk20a_gr_dump_regs(g, &o);

	return 0;
}

static int gk20a_gr_debug_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct gk20a *g = gk20a_get_platform(dev)->g;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	int err;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu: %d", err);
		return -EINVAL;
	}

	gk20a_gr_dump_regs(g, &o);

	gk20a_idle(g);

	return 0;
}

void gk20a_debug_dump(struct gk20a *g)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev_from_gk20a(g));
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_printk
	};

	if (platform->dump_platform_dependencies)
		platform->dump_platform_dependencies(dev_from_gk20a(g));

	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);
}

static int gk20a_debug_show(struct seq_file *s, void *unused)
{
	struct device *dev = s->private;
	struct gk20a_debug_output o = {
		.fn = gk20a_debug_write_to_seqfile,
		.ctx = s,
	};
	struct gk20a *g;
	int err;

	g = gk20a_get_platform(dev)->g;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu: %d", err);
		return -EFAULT;
	}

	/* HAL only initialized after 1st power-on */
	if (g->ops.debug.show_dump)
		g->ops.debug.show_dump(g, &o);

	gk20a_idle(g);
	return 0;
}

static int gk20a_gr_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_gr_debug_show, inode->i_private);
}

static int gk20a_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_debug_show, inode->i_private);
}

static const struct file_operations gk20a_gr_debug_fops = {
	.open		= gk20a_gr_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations gk20a_debug_fops = {
	.open		= gk20a_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o)
{
	g->ops.fifo.dump_pbdma_status(g, o);
	g->ops.fifo.dump_eng_status(g, o);

	gk20a_debug_dump_all_channel_status_ramfc(g, o);
}

static ssize_t disable_bigpage_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[3];
	struct gk20a *g = file->private_data;

	if (g->mm.disable_bigpage)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t disable_bigpage_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[32];
	int buf_size;
	bool bv;
	struct gk20a *g = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (strtobool(buf, &bv) == 0) {
		g->mm.disable_bigpage = bv;
		gk20a_init_gpu_characteristics(g);
	}

	return count;
}

static struct file_operations disable_bigpage_fops = {
	.open =		simple_open,
	.read =		disable_bigpage_read,
	.write =	disable_bigpage_write,
};

static int railgate_residency_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct gk20a_platform *platform = dev_get_drvdata(dev_from_gk20a(g));
	unsigned long time_since_last_state_transition_ms;
	unsigned long total_rail_gate_time_ms;
	unsigned long total_rail_ungate_time_ms;

	if (platform->is_railgated(dev_from_gk20a(g))) {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_gate_complete);
		total_rail_ungate_time_ms = g->pstats.total_rail_ungate_time_ms;
		total_rail_gate_time_ms =
					g->pstats.total_rail_gate_time_ms +
					time_since_last_state_transition_ms;
	} else {
		time_since_last_state_transition_ms =
				jiffies_to_msecs(jiffies -
				g->pstats.last_rail_ungate_complete);
		total_rail_gate_time_ms = g->pstats.total_rail_gate_time_ms;
		total_rail_ungate_time_ms =
					g->pstats.total_rail_ungate_time_ms +
					time_since_last_state_transition_ms;
	}

	seq_printf(s, "Time with Rails Gated: %lu ms\n"
			"Time with Rails UnGated: %lu ms\n"
			"Total railgating cycles: %lu\n",
			total_rail_gate_time_ms,
			total_rail_ungate_time_ms,
			g->pstats.railgating_cycle_count - 1);
	return 0;

}

static int railgate_residency_open(struct inode *inode, struct file *file)
{
	return single_open(file, railgate_residency_show, inode->i_private);
}

static const struct file_operations railgate_residency_fops = {
	.open		= railgate_residency_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int gk20a_railgating_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *d;

	d = debugfs_create_file(
		"railgate_residency", S_IRUGO|S_IWUSR, l->debugfs, g,
						&railgate_residency_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
static ssize_t timeouts_enabled_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[3];
	struct gk20a *g = file->private_data;

	if (nvgpu_is_timeouts_enabled(g))
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t timeouts_enabled_write(struct file *file,
			const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[3];
	int buf_size;
	bool timeouts_enabled;
	struct gk20a *g = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (strtobool(buf, &timeouts_enabled) == 0) {
		nvgpu_mutex_acquire(&g->dbg_sessions_lock);
		if (timeouts_enabled == false) {
			/* requesting to disable timeouts */
			if (g->timeouts_disabled_by_user == false) {
				nvgpu_atomic_inc(&g->timeouts_disabled_refcount);
				g->timeouts_disabled_by_user = true;
			}
		} else {
			/* requesting to enable timeouts */
			if (g->timeouts_disabled_by_user == true) {
				nvgpu_atomic_dec(&g->timeouts_disabled_refcount);
				g->timeouts_disabled_by_user = false;
			}
		}
		nvgpu_mutex_release(&g->dbg_sessions_lock);
	}

	return count;
}

static const struct file_operations timeouts_enabled_fops = {
	.open =		simple_open,
	.read =		timeouts_enabled_read,
	.write =	timeouts_enabled_write,
};

void gk20a_debug_init(struct gk20a *g, const char *debugfs_symlink)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct device *dev = dev_from_gk20a(g);

	l->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	if (!l->debugfs)
		return;

	if (debugfs_symlink)
		l->debugfs_alias =
			debugfs_create_symlink(debugfs_symlink,
					NULL, dev_name(dev));

	debugfs_create_file("status", S_IRUGO, l->debugfs,
		dev, &gk20a_debug_fops);
	debugfs_create_file("gr_status", S_IRUGO, l->debugfs,
		dev, &gk20a_gr_debug_fops);
	debugfs_create_u32("trace_cmdbuf", S_IRUGO|S_IWUSR,
		l->debugfs, &gk20a_debug_trace_cmdbuf);

	debugfs_create_u32("ch_wdt_timeout_ms", S_IRUGO|S_IWUSR,
		l->debugfs, &g->ch_wdt_timeout_ms);

	debugfs_create_u32("disable_syncpoints", S_IRUGO,
		l->debugfs, &g->disable_syncpoints);

	/* New debug logging API. */
	debugfs_create_u64("log_mask", S_IRUGO|S_IWUSR,
		l->debugfs, &g->log_mask);
	debugfs_create_u32("log_trace", S_IRUGO|S_IWUSR,
		l->debugfs, &g->log_trace);

	l->debugfs_ltc_enabled =
			debugfs_create_bool("ltc_enabled", S_IRUGO|S_IWUSR,
				 l->debugfs,
				 &g->mm.ltc_enabled_target);

	l->debugfs_gr_idle_timeout_default =
			debugfs_create_u32("gr_idle_timeout_default_us",
					S_IRUGO|S_IWUSR, l->debugfs,
					 &g->gr_idle_timeout_default);
	l->debugfs_timeouts_enabled =
			debugfs_create_file("timeouts_enabled",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					g,
					&timeouts_enabled_fops);

	l->debugfs_disable_bigpage =
			debugfs_create_file("disable_bigpage",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					g,
					&disable_bigpage_fops);

	l->debugfs_timeslice_low_priority_us =
			debugfs_create_u32("timeslice_low_priority_us",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					&g->timeslice_low_priority_us);
	l->debugfs_timeslice_medium_priority_us =
			debugfs_create_u32("timeslice_medium_priority_us",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					&g->timeslice_medium_priority_us);
	l->debugfs_timeslice_high_priority_us =
			debugfs_create_u32("timeslice_high_priority_us",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					&g->timeslice_high_priority_us);
	l->debugfs_runlist_interleave =
			debugfs_create_bool("runlist_interleave",
					S_IRUGO|S_IWUSR,
					l->debugfs,
					&g->runlist_interleave);
	l->debugfs_force_preemption_gfxp =
		debugfs_create_bool("force_preemption_gfxp", S_IRUGO|S_IWUSR,
		l->debugfs,
		&g->gr.ctx_vars.force_preemption_gfxp);

	l->debugfs_force_preemption_cilp =
		debugfs_create_bool("force_preemption_cilp", S_IRUGO|S_IWUSR,
		l->debugfs,
		&g->gr.ctx_vars.force_preemption_cilp);

	l->debugfs_dump_ctxsw_stats =
		debugfs_create_bool("dump_ctxsw_stats_on_channel_close",
			S_IRUGO|S_IWUSR, l->debugfs,
			&g->gr.ctx_vars.dump_ctxsw_stats_on_channel_close);

	gr_gk20a_debugfs_init(g);
	gk20a_pmu_debugfs_init(g);
	gk20a_railgating_debugfs_init(g);
#ifdef CONFIG_NVGPU_SUPPORT_CDE
	gk20a_cde_debugfs_init(g);
#endif
	gk20a_ce_debugfs_init(g);
	nvgpu_alloc_debugfs_init(g);
	nvgpu_hal_debugfs_init(g);
	gk20a_fifo_debugfs_init(g);
	gk20a_sched_debugfs_init(g);
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	nvgpu_kmem_debugfs_init(g);
#endif
	nvgpu_ltc_debugfs_init(g);
	if (g->pci_vendor_id) {
		nvgpu_xve_debugfs_init(g);
		nvgpu_bios_debugfs_init(g);
	}
}

void gk20a_debug_deinit(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (!l->debugfs)
		return;

	gk20a_fifo_debugfs_deinit(g);

	debugfs_remove_recursive(l->debugfs);
	debugfs_remove(l->debugfs_alias);
}
