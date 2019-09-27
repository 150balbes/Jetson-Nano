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
 *
 */

#include "debug_hal.h"
#include "os_linux.h"

#include <linux/debugfs.h>
#include <linux/seq_file.h>

/* Format and print a single function pointer to the specified seq_file. */
static void __hal_print_op(struct seq_file *s, void *op_ptr)
{
	seq_printf(s, "%pF\n", op_ptr);
}

/*
 * Prints an array of function pointer addresses in op_ptrs to the
 * specified seq_file
 */
static void __hal_print_ops(struct seq_file *s, void **op_ptrs, int num_ops)
{
	int i;

	for (i = 0; i < num_ops; i++)
		__hal_print_op(s, op_ptrs[i]);
}

/*
 * Show file operation, which generates content of the file once. Prints a list
 * of gpu operations as defined by gops and the corresponding function pointer
 * destination addresses. Relies on no compiler reordering of struct fields and
 * assumption that all members are function pointers.
 */
static int __hal_show(struct seq_file *s, void *unused)
{
	struct gpu_ops *gops = s->private;

	__hal_print_ops(s, (void **)gops, sizeof(*gops) / sizeof(void *));

	return 0;
}

static int __hal_open(struct inode *inode, struct file *file)
{
	return single_open(file, __hal_show, inode->i_private);
}

static const struct file_operations __hal_fops = {
	.open = __hal_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void nvgpu_hal_debugfs_fini(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (!(l->debugfs_hal == NULL))
		debugfs_remove_recursive(l->debugfs_hal);
}

void nvgpu_hal_debugfs_init(struct gk20a *g)
{
	struct dentry *d;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (!l->debugfs)
		return;
	l->debugfs_hal = debugfs_create_dir("hal", l->debugfs);
	if (IS_ERR_OR_NULL(l->debugfs_hal)) {
		l->debugfs_hal = NULL;
		return;
	}

	/* Pass along reference to the gpu_ops struct as private data */
	d = debugfs_create_file("gops", S_IRUGO, l->debugfs_hal,
		&g->ops, &__hal_fops);
	if (!d) {
		nvgpu_err(g, "%s: Failed to make debugfs node\n", __func__);
		debugfs_remove_recursive(l->debugfs_hal);
		return;
	}
}
