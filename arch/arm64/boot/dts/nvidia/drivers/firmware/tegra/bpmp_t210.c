/*
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <soc/tegra/bpmp_t210_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include "bpmp.h"

struct bpmp_cpuidle_state {
	int id;
	const char *name;
};

static struct bpmp_cpuidle_state plat_cpuidle_state[] = {
	{ TEGRA_PM_CC4, "CC4" },
	{ TEGRA_PM_CC6, "CC6" },
	{ TEGRA_PM_CC7, "CC7" },
	{ TEGRA_PM_SC2, "SC2" },
	{ TEGRA_PM_SC3, "SC3" },
	{ TEGRA_PM_SC4, "SC4" },
	{ TEGRA_PM_SC7, "SC7" },
	{ 0, NULL }
};

static int bpmp_cpuidle_name_show(struct seq_file *file, void *data)
{
	struct bpmp_cpuidle_state *state = file->private;

	seq_printf(file, "%s\n", state->name);
	return 0;
}

static int bpmp_cpuidle_name_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_name_show, inode->i_private);
}

static const struct file_operations cpuidle_name_fops = {
	.open = bpmp_cpuidle_name_open,
	.read = seq_read,
	.release = single_release
};

static int __bpmp_cpuidle_show(struct seq_file *file, bool show_time)
{
	struct bpmp_cpuidle_state *state = file->private;
	struct { uint64_t usage; uint64_t time; } __packed mb[3];
	uint32_t id = state->id;
	int ret;

	ret = tegra_bpmp_send_receive(MRQ_CPUIDLE_USAGE, &id, sizeof(id),
			&mb, sizeof(mb));
	if (ret) {
		seq_printf(file, "%d\n", ret);
		return ret;
	}

	if (show_time) {
		seq_printf(file, "%llu (%llu, %llu)\n",
				mb[0].time, mb[1].time, mb[2].time);
	} else {
		seq_printf(file, "%llu (%llu, %llu)\n",
				mb[0].usage, mb[1].usage, mb[2].usage);
	}

	return 0;
}

static int bpmp_cpuidle_usage_show(struct seq_file *file, void *data)
{
	return __bpmp_cpuidle_show(file, false);
}

static int bpmp_cpuidle_usage_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_usage_show, inode->i_private);
}

static const struct file_operations cpuidle_usage_fops = {
	.open = bpmp_cpuidle_usage_open,
	.read = seq_read,
	.release = single_release
};

static int bpmp_cpuidle_time_show(struct seq_file *file, void *data)
{
	return __bpmp_cpuidle_show(file, true);
}

static int bpmp_cpuidle_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, bpmp_cpuidle_time_show, inode->i_private);
}

static const struct file_operations cpuidle_time_fops = {
	.open = bpmp_cpuidle_time_open,
	.read = seq_read,
	.release = single_release
};

static const struct fops_entry cpuidle_attrs[] = {
	{ "name", &cpuidle_name_fops, S_IRUGO },
	{ "usage", &cpuidle_usage_fops, S_IRUGO },
	{ "time", &cpuidle_time_fops, S_IRUGO },
	{ NULL, NULL, 0 }
};

static int bpmp_create_cpuidle_debug(int index, struct dentry *parent,
		struct bpmp_cpuidle_state *state)
{
	struct dentry *top;
	char name[16];

	sprintf(name, "state%d", index);
	top = debugfs_create_dir(name, parent);
	if (IS_ERR_OR_NULL(top))
		return -EFAULT;

	return bpmp_create_attrs(cpuidle_attrs, top, state);
}

int bpmp_init_cpuidle_debug(struct dentry *root)
{
	struct bpmp_cpuidle_state *state;
	struct dentry *d;
	unsigned int i;

	d = debugfs_create_dir("cpuidle", root);
	if (IS_ERR_OR_NULL(d))
		return -EFAULT;

	for (i = 0, state = plat_cpuidle_state; state->name; i++, state++) {
		if (bpmp_create_cpuidle_debug(i, d, state))
			return -EFAULT;
	}

	return 0;
}
