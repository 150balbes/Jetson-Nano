/*
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#include <nvgpu/fecs_trace.h>

#include "os_linux.h"

/*
 * The sequence iterator functions.  We simply use the count of the
 * next line as our internal position.
 */
static void *gk20a_fecs_trace_debugfs_ring_seq_start(
		struct seq_file *s, loff_t *pos)
{
	if (*pos >= GK20A_FECS_TRACE_NUM_RECORDS)
		return NULL;

	return pos;
}

static void *gk20a_fecs_trace_debugfs_ring_seq_next(
		struct seq_file *s, void *v, loff_t *pos)
{
	++(*pos);
	if (*pos >= GK20A_FECS_TRACE_NUM_RECORDS)
		return NULL;
	return pos;
}

static void gk20a_fecs_trace_debugfs_ring_seq_stop(
		struct seq_file *s, void *v)
{
}

static int gk20a_fecs_trace_debugfs_ring_seq_show(
		struct seq_file *s, void *v)
{
	loff_t *pos = (loff_t *) v;
	struct gk20a *g = *(struct gk20a **)s->private;
	struct gk20a_fecs_trace_record *r =
		gk20a_fecs_trace_get_record(g, *pos);
	int i;
	const u32 invalid_tag = gk20a_fecs_trace_record_ts_tag_invalid_ts_v();
	u32 tag;
	u64 timestamp;

	seq_printf(s, "record #%lld (%p)\n", *pos, r);
	seq_printf(s, "\tmagic_lo=%08x\n", r->magic_lo);
	seq_printf(s, "\tmagic_hi=%08x\n", r->magic_hi);
	if (gk20a_fecs_trace_is_valid_record(r)) {
		seq_printf(s, "\tcontext_ptr=%08x\n", r->context_ptr);
		seq_printf(s, "\tcontext_id=%08x\n", r->context_id);
		seq_printf(s, "\tnew_context_ptr=%08x\n", r->new_context_ptr);
		seq_printf(s, "\tnew_context_id=%08x\n", r->new_context_id);
		for (i = 0; i < gk20a_fecs_trace_num_ts(); i++) {
			tag = gk20a_fecs_trace_record_ts_tag_v(r->ts[i]);
			if (tag == invalid_tag)
				continue;
			timestamp = gk20a_fecs_trace_record_ts_timestamp_v(r->ts[i]);
			timestamp <<= GK20A_FECS_TRACE_PTIMER_SHIFT;
			seq_printf(s, "\ttag=%02x timestamp=%012llx\n", tag, timestamp);
		}
	}
	return 0;
}

/*
 * Tie them all together into a set of seq_operations.
 */
static const struct seq_operations gk20a_fecs_trace_debugfs_ring_seq_ops = {
	.start = gk20a_fecs_trace_debugfs_ring_seq_start,
	.next = gk20a_fecs_trace_debugfs_ring_seq_next,
	.stop = gk20a_fecs_trace_debugfs_ring_seq_stop,
	.show = gk20a_fecs_trace_debugfs_ring_seq_show
};

/*
 * Time to set up the file operations for our /proc file.  In this case,
 * all we need is an open function which sets up the sequence ops.
 */

static int gk20a_ctxsw_debugfs_ring_open(struct inode *inode,
	struct file *file)
{
	struct gk20a **p;

	p = __seq_open_private(file, &gk20a_fecs_trace_debugfs_ring_seq_ops,
		sizeof(struct gk20a *));
	if (!p)
		return -ENOMEM;

	*p = (struct gk20a *)inode->i_private;
	return 0;
};

/*
 * The file operations structure contains our open function along with
 * set of the canned seq_ ops.
 */
static const struct file_operations gk20a_fecs_trace_debugfs_ring_fops = {
	.owner = THIS_MODULE,
	.open = gk20a_ctxsw_debugfs_ring_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_private
};

static int gk20a_fecs_trace_debugfs_read(void *arg, u64 *val)
{
	*val = gk20a_fecs_trace_get_read_index((struct gk20a *)arg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gk20a_fecs_trace_debugfs_read_fops,
	gk20a_fecs_trace_debugfs_read, NULL, "%llu\n");

static int gk20a_fecs_trace_debugfs_write(void *arg, u64 *val)
{
	*val = gk20a_fecs_trace_get_write_index((struct gk20a *)arg);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(gk20a_fecs_trace_debugfs_write_fops,
	gk20a_fecs_trace_debugfs_write, NULL, "%llu\n");

int nvgpu_fecs_trace_init_debugfs(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	debugfs_create_file("ctxsw_trace_read", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_read_fops);
	debugfs_create_file("ctxsw_trace_write", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_write_fops);
	debugfs_create_file("ctxsw_trace_ring", 0600, l->debugfs, g,
		&gk20a_fecs_trace_debugfs_ring_fops);

	return 0;
}
