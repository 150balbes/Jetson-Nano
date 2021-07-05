/*
 * Copyright (C) 2017-2020 NVIDIA Corporation.  All rights reserved.
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

#include "debug_fifo.h"
#include "os_linux.h"

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <nvgpu/sort.h>
#include <nvgpu/timers.h>
#include <nvgpu/channel.h>

void __gk20a_fifo_profile_free(struct nvgpu_ref *ref);

static void *gk20a_fifo_sched_debugfs_seq_start(
		struct seq_file *s, loff_t *pos)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;

	if (*pos >= f->num_channels)
		return NULL;

	return &f->channel[*pos];
}

static void *gk20a_fifo_sched_debugfs_seq_next(
		struct seq_file *s, void *v, loff_t *pos)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;

	++(*pos);
	if (*pos >= f->num_channels)
		return NULL;

	return &f->channel[*pos];
}

static void gk20a_fifo_sched_debugfs_seq_stop(
		struct seq_file *s, void *v)
{
}

static int gk20a_fifo_sched_debugfs_seq_show(
		struct seq_file *s, void *v)
{
	struct gk20a *g = s->private;
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch = v;
	struct tsg_gk20a *tsg = NULL;

	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_runlist_info_gk20a *runlist;
	u32 runlist_id;
	int ret = SEQ_SKIP;
	u32 engine_id;

	engine_id = gk20a_fifo_get_gr_engine_id(g);
	engine_info = (f->engine_info + engine_id);
	runlist_id = engine_info->runlist_id;
	runlist = &f->runlist_info[runlist_id];

	if (ch == f->channel) {
		seq_puts(s, "chid     tsgid    pid      timeslice  timeout  interleave graphics_preempt compute_preempt\n");
		seq_puts(s, "                            (usecs)   (msecs)\n");
		ret = 0;
	}

	if (!test_bit(ch->chid, runlist->active_channels))
		return ret;

	if (gk20a_channel_get(ch)) {
		tsg = tsg_gk20a_from_ch(ch);

		if (tsg)
			seq_printf(s, "%-8d %-8d %-8d %-9d %-8d %-10d %-8d %-8d\n",
				ch->chid,
				ch->tsgid,
				ch->tgid,
				tsg->timeslice_us,
				ch->timeout_ms_max,
				tsg->interleave_level,
				tsg->gr_ctx.graphics_preempt_mode,
				tsg->gr_ctx.compute_preempt_mode);
		gk20a_channel_put(ch);
	}
	return 0;
}

static const struct seq_operations gk20a_fifo_sched_debugfs_seq_ops = {
	.start = gk20a_fifo_sched_debugfs_seq_start,
	.next = gk20a_fifo_sched_debugfs_seq_next,
	.stop = gk20a_fifo_sched_debugfs_seq_stop,
	.show = gk20a_fifo_sched_debugfs_seq_show
};

static int gk20a_fifo_sched_debugfs_open(struct inode *inode,
	struct file *file)
{
	struct gk20a *g = inode->i_private;
	int err;

	err = seq_open(file, &gk20a_fifo_sched_debugfs_seq_ops);
	if (err)
		return err;

	nvgpu_log(g, gpu_dbg_info, "i_private=%p", inode->i_private);

	((struct seq_file *)file->private_data)->private = inode->i_private;
	return 0;
};

/*
 * The file operations structure contains our open function along with
 * set of the canned seq_ ops.
 */
static const struct file_operations gk20a_fifo_sched_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = gk20a_fifo_sched_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

static int gk20a_fifo_profile_enable(void *data, u64 val)
{
	struct gk20a *g = (struct gk20a *) data;
	struct fifo_gk20a *f = &g->fifo;


	nvgpu_mutex_acquire(&f->profile.lock);
	if (val == 0) {
		if (f->profile.enabled) {
			f->profile.enabled = false;
			nvgpu_ref_put(&f->profile.ref,
				__gk20a_fifo_profile_free);
		}
	} else {
		if (!f->profile.enabled) {
			/* not kref init as it can have a running condition if
			 * we enable/disable/enable while kickoff is happening
			 */
			if (!nvgpu_ref_get_unless_zero(&f->profile.ref)) {
				f->profile.data = nvgpu_vzalloc(g,
					FIFO_PROFILING_ENTRIES *
					sizeof(struct fifo_profile_gk20a));
				f->profile.sorted  = nvgpu_vzalloc(g,
					FIFO_PROFILING_ENTRIES *
					sizeof(u64));
				if (!(f->profile.data && f->profile.sorted)) {
					nvgpu_vfree(g, f->profile.data);
					nvgpu_vfree(g, f->profile.sorted);
					nvgpu_mutex_release(&f->profile.lock);
					return -ENOMEM;
				}
				nvgpu_ref_init(&f->profile.ref);
			}
			atomic_set(&f->profile.get.atomic_var, 0);
			f->profile.enabled = true;
		}
	}
	nvgpu_mutex_release(&f->profile.lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(
	gk20a_fifo_profile_enable_debugfs_fops,
	NULL,
	gk20a_fifo_profile_enable,
	"%llu\n"
);

static int __profile_cmp(const void *a, const void *b)
{
	return *((unsigned long long *) a) - *((unsigned long long *) b);
}

/*
 * This uses about 800b in the stack, but the function using it is not part
 * of a callstack where much memory is being used, so it is fine
 */
#define PERCENTILE_WIDTH	5
#define PERCENTILE_RANGES	(100/PERCENTILE_WIDTH)

static unsigned int __gk20a_fifo_create_stats(struct gk20a *g,
		u64 *percentiles, u32 index_end, u32 index_start)
{
	unsigned int nelem = 0;
	unsigned int index;
	struct fifo_profile_gk20a *profile;

	for (index = 0; index < FIFO_PROFILING_ENTRIES; index++) {
		profile = &g->fifo.profile.data[index];

		if (profile->timestamp[index_end] >
				profile->timestamp[index_start]) {
			/* This is a valid element */
			g->fifo.profile.sorted[nelem] =
						profile->timestamp[index_end] -
						profile->timestamp[index_start];
			nelem++;
		}
	}

	/* sort it */
	sort(g->fifo.profile.sorted, nelem, sizeof(unsigned long long),
		__profile_cmp, NULL);

	/* build ranges */
	for (index = 0; index < PERCENTILE_RANGES; index++) {
		percentiles[index] = nelem < PERCENTILE_RANGES ? 0 :
			g->fifo.profile.sorted[(PERCENTILE_WIDTH * (index + 1) *
						nelem)/100 - 1];
	}
	return nelem;
}

static int gk20a_fifo_profile_stats(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	unsigned int get, nelem, index;
	/*
	 * 800B in the stack, but function is declared statically and only
	 * called from debugfs handler
	 */
	u64 percentiles_ioctl[PERCENTILE_RANGES];
	u64 percentiles_kickoff[PERCENTILE_RANGES];
	u64 percentiles_jobtracking[PERCENTILE_RANGES];
	u64 percentiles_append[PERCENTILE_RANGES];
	u64 percentiles_userd[PERCENTILE_RANGES];

	if (!nvgpu_ref_get_unless_zero(&g->fifo.profile.ref)) {
		seq_printf(s, "Profiling disabled\n");
		return 0;
	}

	get = atomic_read(&g->fifo.profile.get.atomic_var);

	__gk20a_fifo_create_stats(g, percentiles_ioctl,
		PROFILE_IOCTL_EXIT, PROFILE_IOCTL_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_kickoff,
		PROFILE_END, PROFILE_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_jobtracking,
		PROFILE_JOB_TRACKING, PROFILE_IOCTL_ENTRY);
	__gk20a_fifo_create_stats(g, percentiles_append,
		PROFILE_APPEND, PROFILE_JOB_TRACKING);
	nelem = __gk20a_fifo_create_stats(g, percentiles_userd,
		PROFILE_END, PROFILE_APPEND);

	seq_printf(s, "Number of kickoffs: %d\n", nelem);
	seq_printf(s, "Perc \t ioctl(ns) \t kickoff(ns) \t pbcopy(ns) \t jobtrack(ns) \t userd(ns)\n");

	for (index = 0; index < PERCENTILE_RANGES; index++)
		seq_printf(s, "[%2dpc]\t%8lld\t%8lld\t%8lld\t%8lld\t%8lld\n",
			PERCENTILE_WIDTH * (index+1),
			percentiles_ioctl[index],
			percentiles_kickoff[index],
			percentiles_append[index],
			percentiles_jobtracking[index],
			percentiles_userd[index]);

	nvgpu_ref_put(&g->fifo.profile.ref, __gk20a_fifo_profile_free);

	return 0;
}

static int gk20a_fifo_profile_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, gk20a_fifo_profile_stats, inode->i_private);
}

static const struct file_operations gk20a_fifo_profile_stats_debugfs_fops = {
	.open		= gk20a_fifo_profile_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


void gk20a_fifo_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;
	struct dentry *fifo_root;
	struct dentry *profile_root;

	fifo_root = debugfs_create_dir("fifo", gpu_root);
	if (IS_ERR_OR_NULL(fifo_root))
		return;

	nvgpu_log(g, gpu_dbg_info, "g=%p", g);

	debugfs_create_file("sched", 0600, fifo_root, g,
		&gk20a_fifo_sched_debugfs_fops);

	profile_root = debugfs_create_dir("profile", fifo_root);
	if (IS_ERR_OR_NULL(profile_root))
		return;

	nvgpu_mutex_init(&g->fifo.profile.lock);
	g->fifo.profile.enabled = false;
	atomic_set(&g->fifo.profile.get.atomic_var, 0);
	atomic_set(&g->fifo.profile.ref.refcount.atomic_var, 0);

	debugfs_create_file("enable", 0600, profile_root, g,
		&gk20a_fifo_profile_enable_debugfs_fops);

	debugfs_create_file("stats", 0600, profile_root, g,
		&gk20a_fifo_profile_stats_debugfs_fops);

}

void gk20a_fifo_profile_snapshot(struct fifo_profile_gk20a *profile, int idx)
{
	if (profile)
		profile->timestamp[idx] = nvgpu_current_time_ns();
}

void __gk20a_fifo_profile_free(struct nvgpu_ref *ref)
{
	struct fifo_gk20a *f = container_of(ref, struct fifo_gk20a,
						profile.ref);
	nvgpu_vfree(f->g, f->profile.data);
	nvgpu_vfree(f->g, f->profile.sorted);
}

/* Get the next element in the ring buffer of profile entries
 * and grab a reference to the structure
 */
struct fifo_profile_gk20a *gk20a_fifo_profile_acquire(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_profile_gk20a *profile;
	unsigned int index;

	/* If kref is zero, profiling is not enabled */
	if (!nvgpu_ref_get_unless_zero(&f->profile.ref))
		return NULL;
	index = atomic_inc_return(&f->profile.get.atomic_var);
	profile = &f->profile.data[index % FIFO_PROFILING_ENTRIES];

	return profile;
}

/* Free the reference to the structure. This allows deferred cleanups */
void gk20a_fifo_profile_release(struct gk20a *g,
					struct fifo_profile_gk20a *profile)
{
	nvgpu_ref_put(&g->fifo.profile.ref, __gk20a_fifo_profile_free);
}

void gk20a_fifo_debugfs_deinit(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_mutex_acquire(&f->profile.lock);
	if (f->profile.enabled) {
		f->profile.enabled = false;
		nvgpu_ref_put(&f->profile.ref, __gk20a_fifo_profile_free);
	}
	nvgpu_mutex_release(&f->profile.lock);
}
