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

#include <nvgpu/types.h>
#include <nvgpu/xve.h>
#include <nvgpu/timers.h>

#include "debug_xve.h"
#include "os_linux.h"

#include <linux/debugfs.h>
#include <linux/uaccess.h>

static ssize_t xve_link_speed_write(struct file *filp,
				    const char __user *buff,
				    size_t len, loff_t *off)
{
	struct gk20a *g = ((struct seq_file *)filp->private_data)->private;
	char kbuff[16];
	u32 buff_size, check_len;
	u32 link_speed = 0;
	int ret;

	buff_size = min_t(size_t, 16, len);

	memset(kbuff, 0, 16);
	if (copy_from_user(kbuff, buff, buff_size))
		return -EFAULT;

	check_len = strlen("Gen1");
	if (strncmp(kbuff, "Gen1", check_len) == 0)
		link_speed = GPU_XVE_SPEED_2P5;
	else if (strncmp(kbuff, "Gen2", check_len) == 0)
		link_speed = GPU_XVE_SPEED_5P0;
	else if (strncmp(kbuff, "Gen3", check_len) == 0)
		link_speed = GPU_XVE_SPEED_8P0;
	else
		nvgpu_err(g, "%s: Unknown PCIe speed: %s",
			  __func__, kbuff);

	if (!link_speed)
		return -EINVAL;

	/* Brief pause... To help rate limit this. */
	nvgpu_msleep(250);

	/*
	 * And actually set the speed. Yay.
	 */
	ret = g->ops.xve.set_speed(g, link_speed);
	if (ret)
		return ret;

	return len;
}

static int xve_link_speed_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	u32 speed;
	int err;

	err = g->ops.xve.get_speed(g, &speed);
	if (err)
		return err;

	seq_printf(s, "Current PCIe speed:\n  %s\n", xve_speed_to_str(speed));

	return 0;
}

static int xve_link_speed_open(struct inode *inode, struct file *file)
{
	return single_open(file, xve_link_speed_show, inode->i_private);
}

static const struct file_operations xve_link_speed_fops = {
	.open = xve_link_speed_open,
	.read = seq_read,
	.write = xve_link_speed_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int xve_available_speeds_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	u32 available_speeds;

	g->ops.xve.available_speeds(g, &available_speeds);

	seq_puts(s, "Available PCIe bus speeds:\n");
	if (available_speeds & GPU_XVE_SPEED_2P5)
		seq_puts(s, "  Gen1\n");
	if (available_speeds & GPU_XVE_SPEED_5P0)
		seq_puts(s, "  Gen2\n");
	if (available_speeds & GPU_XVE_SPEED_8P0)
		seq_puts(s, "  Gen3\n");

	return 0;
}

static int xve_available_speeds_open(struct inode *inode, struct file *file)
{
	return single_open(file, xve_available_speeds_show, inode->i_private);
}

static const struct file_operations xve_available_speeds_fops = {
	.open = xve_available_speeds_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int xve_link_control_status_show(struct seq_file *s, void *unused)
{
	struct gk20a *g = s->private;
	u32 link_status;

	link_status = g->ops.xve.get_link_control_status(g);
	seq_printf(s, "0x%08x\n", link_status);

	return 0;
}

static int xve_link_control_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, xve_link_control_status_show, inode->i_private);
}

static const struct file_operations xve_link_control_status_fops = {
	.open = xve_link_control_status_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int nvgpu_xve_debugfs_init(struct gk20a *g)
{
	int err = -ENODEV;

	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct dentry *gpu_root = l->debugfs;

	l->debugfs_xve = debugfs_create_dir("xve", gpu_root);
	if (IS_ERR_OR_NULL(l->debugfs_xve))
		goto fail;

	/*
	 * These are just debug nodes. If they fail to get made it's not worth
	 * worrying the higher level SW.
	 */
	debugfs_create_file("link_speed", S_IRUGO,
			    l->debugfs_xve, g,
			    &xve_link_speed_fops);
	debugfs_create_file("available_speeds", S_IRUGO,
			    l->debugfs_xve, g,
			    &xve_available_speeds_fops);
	debugfs_create_file("link_control_status", S_IRUGO,
			    l->debugfs_xve, g,
			    &xve_link_control_status_fops);

	err = 0;
fail:
	return err;
}
