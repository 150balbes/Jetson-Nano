/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include "os_linux.h"
#include "debug_kmem.h"
#include "kmem_priv.h"

/**
 * to_human_readable_bytes - Determine  suffix for passed size.
 *
 * @bytes - Number of bytes to generate a suffix for.
 * @hr_bytes [out] - The human readable number of bytes.
 * @hr_suffix [out] - The suffix for the HR number of bytes.
 *
 * Computes a human readable decomposition of the passed number of bytes. The
 * suffix for the bytes is passed back through the @hr_suffix pointer. The right
 * number of bytes is then passed back in @hr_bytes. This returns the following
 * ranges:
 *
 *   0 - 1023 B
 *   1 - 1023 KB
 *   1 - 1023 MB
 *   1 - 1023 GB
 *   1 - 1023 TB
 *   1 - ...  PB
 */
static void __to_human_readable_bytes(u64 bytes, u64 *hr_bytes,
				      const char **hr_suffix)
{
	static const char *suffixes[] =
		{ "B", "KB", "MB", "GB", "TB", "PB" };

	u64 suffix_ind = 0;

	while (suffix_ind < ARRAY_SIZE(suffixes) && bytes >= 1024) {
		bytes >>= 10;
		suffix_ind++;
	}

	/*
	 * Handle case where bytes > 1023PB.
	 */
	suffix_ind = suffix_ind < ARRAY_SIZE(suffixes) ?
		suffix_ind : ARRAY_SIZE(suffixes) - 1;

	*hr_bytes = bytes;
	*hr_suffix = suffixes[suffix_ind];
}

/**
 * print_hr_bytes - Print human readable bytes
 *
 * @s - A seq_file to print to. May be NULL.
 * @msg - A message to print before the bytes.
 * @bytes - Number of bytes.
 *
 * Print @msg followed by the human readable decomposition of the passed number
 * of bytes.
 *
 * If @s is NULL then this prints will be made to the kernel log.
 */
static void print_hr_bytes(struct seq_file *s, const char *msg, u64 bytes)
{
	u64 hr_bytes;
	const char *hr_suffix;

	__to_human_readable_bytes(bytes, &hr_bytes, &hr_suffix);
	__pstat(s, "%s%lld %s\n", msg, hr_bytes, hr_suffix);
}

/**
 * print_histogram - Build a histogram of the memory usage.
 *
 * @tracker The tracking to pull data from.
 * @s       A seq_file to dump info into.
 */
static void print_histogram(struct nvgpu_mem_alloc_tracker *tracker,
			    struct seq_file *s)
{
	int i;
	u64 pot_min, pot_max;
	u64 nr_buckets;
	unsigned int *buckets;
	unsigned int total_allocs;
	struct nvgpu_rbtree_node *node;
	static const char histogram_line[] =
		"++++++++++++++++++++++++++++++++++++++++";

	/*
	 * pot_min is essentially a round down to the nearest power of 2. This
	 * is the start of the histogram. pot_max is just a round up to the
	 * nearest power of two. Each histogram bucket is one power of two so
	 * the histogram buckets are exponential.
	 */
	pot_min = (u64)rounddown_pow_of_two(tracker->min_alloc);
	pot_max = (u64)roundup_pow_of_two(tracker->max_alloc);

	nr_buckets = __ffs(pot_max) - __ffs(pot_min);

	buckets = kzalloc(sizeof(*buckets) * nr_buckets, GFP_KERNEL);
	if (!buckets) {
		__pstat(s, "OOM: could not allocate bucket storage!?\n");
		return;
	}

	/*
	 * Iterate across all of the allocs and determine what bucket they
	 * should go in. Round the size down to the nearest power of two to
	 * find the right bucket.
	 */
	nvgpu_rbtree_enum_start(0, &node, tracker->allocs);
	while (node) {
		int b;
		u64 bucket_min;
		struct nvgpu_mem_alloc *alloc =
			nvgpu_mem_alloc_from_rbtree_node(node);

		bucket_min = (u64)rounddown_pow_of_two(alloc->size);
		if (bucket_min < tracker->min_alloc)
			bucket_min = tracker->min_alloc;

		b = __ffs(bucket_min) - __ffs(pot_min);

		/*
		 * Handle the one case were there's an alloc exactly as big as
		 * the maximum bucket size of the largest bucket. Most of the
		 * buckets have an inclusive minimum and exclusive maximum. But
		 * the largest bucket needs to have an _inclusive_ maximum as
		 * well.
		 */
		if (b == (int)nr_buckets)
			b--;

		buckets[b]++;

		nvgpu_rbtree_enum_next(&node, node);
	}

	total_allocs = 0;
	for (i = 0; i < (int)nr_buckets; i++)
		total_allocs += buckets[i];

	__pstat(s, "Alloc histogram:\n");

	/*
	 * Actually compute the histogram lines.
	 */
	for (i = 0; i < (int)nr_buckets; i++) {
		char this_line[sizeof(histogram_line) + 1];
		u64 line_length;
		u64 hr_bytes;
		const char *hr_suffix;

		memset(this_line, 0, sizeof(this_line));

		/*
		 * Compute the normalized line length. Cant use floating point
		 * so we will just multiply everything by 1000 and use fixed
		 * point.
		 */
		line_length = (1000 * buckets[i]) / total_allocs;
		line_length *= sizeof(histogram_line);
		line_length /= 1000;

		memset(this_line, '+', line_length);

		__to_human_readable_bytes(1 << (__ffs(pot_min) + i),
					  &hr_bytes, &hr_suffix);
		__pstat(s, "  [%-4lld %-4lld] %-2s %5u | %s\n",
			hr_bytes, hr_bytes << 1,
			hr_suffix, buckets[i], this_line);
	}
}

/**
 * nvgpu_kmem_print_stats - Print kmem tracking stats.
 *
 * @tracker The tracking to pull data from.
 * @s       A seq_file to dump info into.
 *
 * Print stats from a tracker. If @s is non-null then seq_printf() will be
 * used with @s. Otherwise the stats are pr_info()ed.
 */
void nvgpu_kmem_print_stats(struct nvgpu_mem_alloc_tracker *tracker,
			    struct seq_file *s)
{
	nvgpu_lock_tracker(tracker);

	__pstat(s, "Mem tracker: %s\n\n", tracker->name);

	__pstat(s, "Basic Stats:\n");
	__pstat(s,        "  Number of allocs        %lld\n",
		tracker->nr_allocs);
	__pstat(s,        "  Number of frees         %lld\n",
		tracker->nr_frees);
	print_hr_bytes(s, "  Smallest alloc          ", tracker->min_alloc);
	print_hr_bytes(s, "  Largest alloc           ", tracker->max_alloc);
	print_hr_bytes(s, "  Bytes allocated         ", tracker->bytes_alloced);
	print_hr_bytes(s, "  Bytes freed             ", tracker->bytes_freed);
	print_hr_bytes(s, "  Bytes allocated (real)  ",
		       tracker->bytes_alloced_real);
	print_hr_bytes(s, "  Bytes freed (real)      ",
		       tracker->bytes_freed_real);
	__pstat(s, "\n");

	print_histogram(tracker, s);

	nvgpu_unlock_tracker(tracker);
}

static int __kmem_tracking_show(struct seq_file *s, void *unused)
{
	struct nvgpu_mem_alloc_tracker *tracker = s->private;

	nvgpu_kmem_print_stats(tracker, s);

	return 0;
}

static int __kmem_tracking_open(struct inode *inode, struct file *file)
{
	return single_open(file, __kmem_tracking_show, inode->i_private);
}

static const struct file_operations __kmem_tracking_fops = {
	.open = __kmem_tracking_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __kmem_traces_dump_tracker(struct gk20a *g,
				      struct nvgpu_mem_alloc_tracker *tracker,
				      struct seq_file *s)
{
	struct nvgpu_rbtree_node *node;

	nvgpu_rbtree_enum_start(0, &node, tracker->allocs);
	while (node) {
		struct nvgpu_mem_alloc *alloc =
			nvgpu_mem_alloc_from_rbtree_node(node);

		kmem_print_mem_alloc(g, alloc, s);

		nvgpu_rbtree_enum_next(&node, node);
	}

	return 0;
}

static int __kmem_traces_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;

	nvgpu_lock_tracker(g->vmallocs);
	seq_puts(s, "Oustanding vmallocs:\n");
	__kmem_traces_dump_tracker(g, g->vmallocs, s);
	seq_puts(s, "\n");
	nvgpu_unlock_tracker(g->vmallocs);

	nvgpu_lock_tracker(g->kmallocs);
	seq_puts(s, "Oustanding kmallocs:\n");
	__kmem_traces_dump_tracker(g, g->kmallocs, s);
	nvgpu_unlock_tracker(g->kmallocs);

	return 0;
}

static int __kmem_traces_open(struct inode *inode, struct file *file)
{
	return single_open(file, __kmem_traces_show, inode->i_private);
}

static const struct file_operations __kmem_traces_fops = {
	.open = __kmem_traces_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void nvgpu_kmem_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *node;

	l->debugfs_kmem = debugfs_create_dir("kmem_tracking", l->debugfs);
	if (IS_ERR_OR_NULL(l->debugfs_kmem))
		return;

	node = debugfs_create_file(g->vmallocs->name, S_IRUGO,
				   l->debugfs_kmem,
				   g->vmallocs, &__kmem_tracking_fops);
	node = debugfs_create_file(g->kmallocs->name, S_IRUGO,
				   l->debugfs_kmem,
				   g->kmallocs, &__kmem_tracking_fops);
	node = debugfs_create_file("traces", S_IRUGO,
				   l->debugfs_kmem,
				   g, &__kmem_traces_fops);
}
