/*
 * Copyright (C) 2018 NVIDIA Corporation.  All rights reserved.
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

#include "debug_ltc.h"
#include "os_linux.h"

#include <nvgpu/gk20a.h>

#include <linux/debugfs.h>
#include <linux/uaccess.h>

static ssize_t ltc_intr_illegal_compstat_read(struct file *file,
			char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[3];
	struct gk20a *g = file->private_data;

	if (g->ltc_intr_en_illegal_compstat)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;

	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t ltc_intr_illegal_compstat_write(struct file *file,
			const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[3];
	int buf_size;
	bool intr_illegal_compstat_enabled;
	struct gk20a *g = file->private_data;
	int err;

	if (!g->ops.ltc.intr_en_illegal_compstat)
		return -EINVAL;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	err = gk20a_busy(g);
	if (err)
		return err;

	if (strtobool(buf, &intr_illegal_compstat_enabled) == 0) {
		g->ops.ltc.intr_en_illegal_compstat(g,
				intr_illegal_compstat_enabled);
		g->ltc_intr_en_illegal_compstat = intr_illegal_compstat_enabled;
	}

	gk20a_idle(g);

	return buf_size;
}

static const struct file_operations ltc_intr_illegal_compstat_fops = {
	.open = simple_open,
	.read = ltc_intr_illegal_compstat_read,
	.write = ltc_intr_illegal_compstat_write,
};

int nvgpu_ltc_debugfs_init(struct gk20a *g)
{
	struct dentry *d;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;

	l->debugfs_ltc = debugfs_create_dir("ltc", gpu_root);
	if (IS_ERR_OR_NULL(l->debugfs_ltc))
		return -ENODEV;

	/* Debug fs node to enable/disable illegal_compstat */
	d = debugfs_create_file("intr_illegal_compstat_enable", 0600,
			    l->debugfs_ltc, g,
			    &ltc_intr_illegal_compstat_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}
