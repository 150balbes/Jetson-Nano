/*
 * Copyright (C) 2017-2019 NVIDIA Corporation.  All rights reserved.
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

#include <nvgpu/enabled.h>
#include "debug_pmu.h"
#include "os_linux.h"

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

static int lpwr_debug_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;

	if (g->ops.pmu.pmu_pg_engines_feature_list &&
		g->ops.pmu.pmu_pg_engines_feature_list(g,
		PMU_PG_ELPG_ENGINE_ID_GRAPHICS) !=
		NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING) {
		seq_printf(s, "PSTATE: %u\n"
			"RPPG Enabled: %u\n"
			"RPPG ref count: %u\n"
			"RPPG state: %u\n"
			"MSCG Enabled: %u\n"
			"MSCG pstate state: %u\n"
			"MSCG transition state: %u\n",
			g->ops.clk_arb.get_current_pstate(g),
			g->elpg_enabled, g->pmu.elpg_refcnt,
			g->pmu.elpg_stat, g->mscg_enabled,
			g->pmu.mscg_stat, g->pmu.mscg_transition_state);

	} else
		seq_printf(s, "ELPG Enabled: %u\n"
			"ELPG ref count: %u\n"
			"ELPG state: %u\n",
			g->elpg_enabled, g->pmu.elpg_refcnt,
			g->pmu.elpg_stat);

	return 0;

}

static int lpwr_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, lpwr_debug_show, inode->i_private);
}

static const struct file_operations lpwr_debug_fops = {
	.open		= lpwr_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mscg_stat_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	u64 total_ingating, total_ungating, residency, divisor, dividend;
	struct pmu_pg_stats_data pg_stat_data = { 0 };
	int err;

	/* Don't unnecessarily power on the device */
	if (g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		nvgpu_pmu_get_pg_stats(g,
			PMU_PG_ELPG_ENGINE_ID_MS, &pg_stat_data);
		gk20a_idle(g);
	}
	total_ingating = g->pg_ingating_time_us +
			(u64)pg_stat_data.ingating_time;
	total_ungating = g->pg_ungating_time_us +
			(u64)pg_stat_data.ungating_time;

	divisor = total_ingating + total_ungating;

	/* We compute the residency on a scale of 1000 */
	dividend = total_ingating * 1000;

	if (divisor)
		residency = div64_u64(dividend, divisor);
	else
		residency = 0;

	seq_printf(s,
			"Time in MSCG: %llu us\n"
			"Time out of MSCG: %llu us\n"
			"MSCG residency ratio: %llu\n"
			"MSCG Entry Count: %u\n"
			"MSCG Avg Entry latency %u\n"
			"MSCG Avg Exit latency %u\n",
			total_ingating, total_ungating,
			residency, pg_stat_data.gating_cnt,
			pg_stat_data.avg_entry_latency_us,
			pg_stat_data.avg_exit_latency_us);
	return 0;

}

static int mscg_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, mscg_stat_show, inode->i_private);
}

static const struct file_operations mscg_stat_fops = {
	.open		= mscg_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mscg_transitions_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct pmu_pg_stats_data pg_stat_data = { 0 };
	u32 total_gating_cnt;
	int err;

	if (g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		nvgpu_pmu_get_pg_stats(g,
			PMU_PG_ELPG_ENGINE_ID_MS, &pg_stat_data);
		gk20a_idle(g);
	}
	total_gating_cnt = g->pg_gating_cnt + pg_stat_data.gating_cnt;

	seq_printf(s, "%u\n", total_gating_cnt);
	return 0;

}

static int mscg_transitions_open(struct inode *inode, struct file *file)
{
	return single_open(file, mscg_transitions_show, inode->i_private);
}

static const struct file_operations mscg_transitions_fops = {
	.open		= mscg_transitions_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int elpg_stat_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct pmu_pg_stats_data pg_stat_data = { 0 };
	u64 total_ingating, total_ungating, residency, divisor, dividend;
	int err;

	/* Don't unnecessarily power on the device */
	if (g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		nvgpu_pmu_get_pg_stats(g,
			PMU_PG_ELPG_ENGINE_ID_GRAPHICS, &pg_stat_data);
		gk20a_idle(g);
	}
	total_ingating = g->pg_ingating_time_us +
			(u64)pg_stat_data.ingating_time;
	total_ungating = g->pg_ungating_time_us +
			(u64)pg_stat_data.ungating_time;
	divisor = total_ingating + total_ungating;

	/* We compute the residency on a scale of 1000 */
	dividend = total_ingating * 1000;

	if (divisor)
		residency = div64_u64(dividend, divisor);
	else
		residency = 0;

	seq_printf(s,
			"Time in ELPG: %llu us\n"
			"Time out of ELPG: %llu us\n"
			"ELPG residency ratio: %llu\n"
			"ELPG Entry Count: %u\n"
			"ELPG Avg Entry latency %u us\n"
			"ELPG Avg Exit latency %u us\n",
			total_ingating, total_ungating,
			residency, pg_stat_data.gating_cnt,
			pg_stat_data.avg_entry_latency_us,
			pg_stat_data.avg_exit_latency_us);
	return 0;

}

static int elpg_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, elpg_stat_show, inode->i_private);
}

static const struct file_operations elpg_stat_fops = {
	.open		= elpg_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int elpg_transitions_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct pmu_pg_stats_data pg_stat_data = { 0 };
	u32 total_gating_cnt;
	int err;

	if (g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		nvgpu_pmu_get_pg_stats(g,
			PMU_PG_ELPG_ENGINE_ID_GRAPHICS, &pg_stat_data);
		gk20a_idle(g);
	}
	total_gating_cnt = g->pg_gating_cnt + pg_stat_data.gating_cnt;

	seq_printf(s, "%u\n", total_gating_cnt);
	return 0;

}

static int elpg_transitions_open(struct inode *inode, struct file *file)
{
	return single_open(file, elpg_transitions_show, inode->i_private);
}

static const struct file_operations elpg_transitions_fops = {
	.open		= elpg_transitions_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int falc_trace_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;
	struct nvgpu_pmu *pmu = &g->pmu;
	u32 i = 0, j = 0, k, l, m;
	char part_str[40];
	void *tracebuffer;
	char *trace;
	u32 *trace1;

	/* allocate system memory to copy pmu trace buffer */
	tracebuffer = nvgpu_kzalloc(g, GK20A_PMU_TRACE_BUFSIZE);
	if (tracebuffer == NULL)
		return -ENOMEM;

	/* read pmu traces into system memory buffer */
	nvgpu_mem_rd_n(g, &pmu->trace_buf,
		       0, tracebuffer, GK20A_PMU_TRACE_BUFSIZE);

	trace = (char *)tracebuffer;
	trace1 = (u32 *)tracebuffer;

	for (i = 0; i < GK20A_PMU_TRACE_BUFSIZE; i += 0x40) {
		for (j = 0; j < 0x40; j++)
			if (trace1[(i / 4) + j])
				break;
		if (j == 0x40)
			break;
		seq_printf(s, "Index %x: ", trace1[(i / 4)]);
		l = 0;
		m = 0;
		while (nvgpu_find_hex_in_string((trace+i+20+m), g, &k)) {
			if (k >= 40)
				break;
			strncpy(part_str, (trace+i+20+m), k);
			part_str[k] = 0;
			seq_printf(s, "%s0x%x", part_str,
					trace1[(i / 4) + 1 + l]);
			l++;
			m += k + 2;
		}
		seq_printf(s, "%s", (trace+i+20+m));
	}

	nvgpu_kfree(g, tracebuffer);
	return 0;
}

static int falc_trace_open(struct inode *inode, struct file *file)
{
	return single_open(file, falc_trace_show, inode->i_private);
}

static const struct file_operations falc_trace_fops = {
	.open		= falc_trace_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int perfmon_events_enable_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;

	seq_printf(s, "%u\n", g->pmu.perfmon_sampling_enabled ? 1 : 0);
	return 0;

}

static int perfmon_events_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmon_events_enable_show, inode->i_private);
}

static ssize_t perfmon_events_enable_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct gk20a *g = s->private;
	unsigned long val = 0;
	char buf[40];
	int buf_size;
	int err;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, (sizeof(buf)-1));

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	/* Don't turn on gk20a unnecessarily */
	if (g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		if (val && !g->pmu.perfmon_sampling_enabled &&
				nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
			g->pmu.perfmon_sampling_enabled = true;
			g->ops.pmu.pmu_perfmon_start_sampling(&(g->pmu));
		} else if (!val && g->pmu.perfmon_sampling_enabled &&
				nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
			g->pmu.perfmon_sampling_enabled = false;
			g->ops.pmu.pmu_perfmon_stop_sampling(&(g->pmu));
		}
		gk20a_idle(g);
	} else {
		g->pmu.perfmon_sampling_enabled = val ? true : false;
	}

	return count;
}

static const struct file_operations perfmon_events_enable_fops = {
	.open		= perfmon_events_enable_open,
	.read		= seq_read,
	.write		= perfmon_events_enable_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int perfmon_events_count_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;

	seq_printf(s, "%lu\n", g->pmu.perfmon_events_cnt);
	return 0;

}

static int perfmon_events_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, perfmon_events_count_show, inode->i_private);
}

static const struct file_operations perfmon_events_count_fops = {
	.open		= perfmon_events_count_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int security_show(struct seq_file *s, void *data)
{
	struct gk20a *g = s->private;

	seq_printf(s, "%d\n", g->pmu.pmu_mode);
	return 0;

}

static int security_open(struct inode *inode, struct file *file)
{
	return single_open(file, security_show, inode->i_private);
}

static const struct file_operations security_fops = {
	.open		= security_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int gk20a_pmu_debugfs_init(struct gk20a *g)
{
	struct dentry *d;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	d = debugfs_create_file(
		"lpwr_debug", S_IRUGO|S_IWUSR, l->debugfs, g,
						&lpwr_debug_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"mscg_residency", S_IRUGO|S_IWUSR, l->debugfs, g,
						&mscg_stat_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"mscg_transitions", S_IRUGO, l->debugfs, g,
						&mscg_transitions_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"elpg_residency", S_IRUGO|S_IWUSR, l->debugfs, g,
						&elpg_stat_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"elpg_transitions", S_IRUGO, l->debugfs, g,
						&elpg_transitions_fops);
	if (!d)
		goto err_out;

	d = debugfs_create_file(
		"pmu_security", S_IRUGO, l->debugfs, g,
						&security_fops);
	if (!d)
		goto err_out;

	/* No access to PMU if virtual */
	if (!g->is_virtual) {
		d = debugfs_create_file(
			"falc_trace", S_IRUGO, l->debugfs, g,
						&falc_trace_fops);
		if (!d)
			goto err_out;

		d = debugfs_create_file(
			"perfmon_events_enable", S_IRUGO, l->debugfs, g,
						&perfmon_events_enable_fops);
		if (!d)
			goto err_out;

		d = debugfs_create_file(
			"perfmon_events_count", S_IRUGO, l->debugfs, g,
						&perfmon_events_count_fops);
		if (!d)
			goto err_out;
	}
	return 0;
err_out:
	pr_err("%s: Failed to make debugfs node\n", __func__);
	return -ENOMEM;
}
