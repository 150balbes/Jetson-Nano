/*
 * t19x-nvlink-endpt-debugfs.c:
 * This file adds various debugfs nodes for the Tegra NVLINK controller.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"
#include <linux/uaccess.h>

static int nvlink_refclk_rate_file_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t nvlink_refclk_rate_file_read(struct file *file,
				char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct tnvlink_dev *tdev = file->private_data;
	char buf[5];
	int str_len;

	switch (tdev->refclk) {
	case NVLINK_REFCLK_150:
		strcpy(buf, "150");
		break;
	case NVLINK_REFCLK_156:
	default:
		strcpy(buf, "156");
		break;
	}
	strcat(buf, "\n");
	str_len = strlen(buf);
	return simple_read_from_buffer(ubuf, count, offp, buf, str_len);
}

static ssize_t nvlink_refclk_rate_file_write(struct file *file,
				const char __user *ubuf,
				size_t count, loff_t *offp)
{
	struct tnvlink_dev *tdev = file->private_data;
	struct nvlink_device *ndev = tdev->ndev;
	char tmp[3];
	int ret;
	enum init_state state = NVLINK_DEV_OFF;

	ret = nvlink_get_init_state(ndev, &state);
	if (ret < 0) {
		nvlink_err("Error retriving the init state!");
		return ret;
	}
	if (NVLINK_DEV_OFF != state)
		return -EINVAL;

	ret = copy_from_user(tmp, ubuf, count);

	if (!strncmp(tmp, "150", 3))
		tdev->refclk = NVLINK_REFCLK_150;
	else if (!strncmp(tmp, "156", 3))
		tdev->refclk = NVLINK_REFCLK_156;
	else {
		nvlink_err("Invalid refclk rate request!");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations  nvlink_refclk_rate_fops = {
	.open	= nvlink_refclk_rate_file_open,
	.read	= nvlink_refclk_rate_file_read,
	.write	= nvlink_refclk_rate_file_write,
	.owner	= THIS_MODULE,
};

static int nvlink_speedcontrol_file_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t nvlink_speedcontrol_file_read(struct file *file,
					     char __user *ubuf,
					     size_t count,
					     loff_t *offp)
{
	struct tnvlink_dev *tdev = file->private_data;
	struct nvlink_device *ndev = tdev->ndev;
	char buf[4];
	int str_len;

	switch (ndev->speed) {
	case NVLINK_SPEED_16:
		strcpy(buf, "16");
		break;
	case NVLINK_SPEED_20:
		strcpy(buf, "20");
		break;
	default:
		nvlink_err("Unsupported ndev speed!");
		strcpy(buf, "-1");
		break;
	}
	strcat(buf, "\n");
	str_len = strlen(buf);
	return simple_read_from_buffer(ubuf, count, offp, buf, str_len);
}

static ssize_t nvlink_speedcontrol_file_write(struct file *file,
						const char __user *ubuf,
						size_t count, loff_t *offp)
{
	struct tnvlink_dev *tdev = file->private_data;
	struct nvlink_device *ndev = tdev->ndev;
	char tmp[2];
	int ret;
	enum init_state state = NVLINK_DEV_OFF;

	ret = nvlink_get_init_state(ndev, &state);
	if (ret < 0) {
		nvlink_err("Error retriving the device state!");
		return ret;
	}
	if (NVLINK_DEV_OFF != state)
		return -EINVAL;

	ret = copy_from_user(tmp, ubuf, count);

	if (!strncmp(tmp, "20", 2))
		ndev->speed = NVLINK_SPEED_20;
	else if (!strncmp(tmp, "16", 2))
		ndev->speed = NVLINK_SPEED_16;
	else {
		nvlink_err("Invalid speed request!");
		return -EINVAL;
	}
	return count;
}

static const struct file_operations  nvlink_speedcontrol_fops = {
	.open	= nvlink_speedcontrol_file_open,
	.read	= nvlink_speedcontrol_file_read,
	.write	= nvlink_speedcontrol_file_write,
	.owner	= THIS_MODULE,
};

static int nvlink_shutdown_file_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t nvlink_shutdown_file_write(struct file *file,
					const char __user *ubuf,
					size_t count, loff_t *offp)
{
#if IS_ENABLED(CONFIG_PM_SLEEP)
	struct tnvlink_dev *tdev = file->private_data;
#endif
	char tmp[3];
	int ret = 0;

	ret = copy_from_user(tmp, ubuf, count);

	if (!strncmp(tmp, "1", 1)) {
		nvlink_dbg("nvlink shutdown request from debugfs node");
#if IS_ENABLED(CONFIG_PM_SLEEP)
		ret = t19x_nvlink_suspend(tdev->dev);
		if (ret < 0)
			nvlink_err("t19x_nvlink_suspend failed");
#endif
	} else {
		nvlink_err("Unsupported value. Write 1 to shutdown nvlink");
		ret = -EINVAL;
	}

	if (ret)
		return ret;
	else
		return count;
}

static const struct file_operations nvlink_shutdown_fops = {
	.open	= nvlink_shutdown_file_open,
	.write	= nvlink_shutdown_file_write,
	.owner	= THIS_MODULE,
};

static int nvlink_single_lane_debugfs_init(struct tnvlink_dev *tdev)
{
	struct dentry *tegra_sl_debugfs;
	int ret = 0;

	tegra_sl_debugfs = debugfs_create_dir("single_lane_params",
						tdev->tegra_debugfs);
	if (!tegra_sl_debugfs) {
		nvlink_err("Failed to create Tegra NVLINK endpoint driver's"
		" single lane debugfs directory");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u16("fb_ic_inc", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.fb_ic_inc)) {
		nvlink_err("Unable to create debugfs node for fb_ic_inc");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u16("lp_ic_inc", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.lp_ic_inc)) {
		nvlink_err("Unable to create debugfs node for lp_ic_inc");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u16("fb_ic_dec", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.fb_ic_dec)) {
		nvlink_err("Unable to create debugfs node for fb_ic_dec");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u16("lp_ic_dec", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.lp_ic_dec)) {
		nvlink_err("Unable to create debugfs node for lp_ic_dec");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u32("enter_thresh", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.enter_thresh)) {
		nvlink_err("Unable to create debugfs node for enter_thresh");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u32("exit_thresh", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.exit_thresh)) {
		nvlink_err("Unable to create debugfs node for exit_thresh");
		ret = -1;
		goto fail;
	}

	if (!debugfs_create_u32("ic_limit", (S_IWUSR | S_IRUGO),
					tegra_sl_debugfs,
					&tdev->tlink.sl_params.ic_limit)) {
		nvlink_err("Unable to create debugfs node for ic_limit");
		ret = -1;
		goto fail;
	}

fail:
	return ret;
}

static int start_tp_cntrs_read(void *data, u64 *val)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)data;
	bool start = tdev->is_tp_cntr_running;

	/* val = 1 implies tp_cntrs are running */
	*val = start ? 1 : 0;
	return 0;
}

static int start_tp_cntrs_write(void *data, u64 val)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)data;
	int status;
	/* val = 1 implies start the tp_cntrs */
	bool freeze = val ? false : true;

	status = t19x_nvlink_freeze_tp_counters(tdev, freeze);
	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(start_tp_cntrs_fops, start_tp_cntrs_read,
					start_tp_cntrs_write, "%llu\n");

static int reset_tp_cntrs_write(void *data, u64 val)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)data;
	int status;

	status = t19x_nvlink_reset_tp_counters(tdev);
	return status;
}
DEFINE_SIMPLE_ATTRIBUTE(reset_tp_cntrs_fops, NULL,
					reset_tp_cntrs_write, "%llu\n");

static int tp_cntrs_show(struct seq_file *s, void *unused)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)s->private;
	int ret = 0;
	u64 tx0cnt, tx1cnt;
	u64 rx0cnt, rx1cnt;

	t19x_nvlink_get_tp_counters(tdev, &tx0cnt, &tx1cnt, &rx0cnt, &rx1cnt);
	seq_printf(s, "TX packets: %llu\n", tx0cnt);
	seq_printf(s, "TX idle cycles : %llu\n", tx1cnt);
	seq_printf(s, "RX packets: %llu\n", rx0cnt);
	seq_printf(s, "RX idle cycles : %llu\n", rx1cnt);

	return ret;
}

static int tp_cntrs_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_cntrs_show, inode->i_private);
}

static const struct file_operations tp_cntrs_fops = {
	.open		= tp_cntrs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int nvlink_tlc_debugfs_init(struct tnvlink_dev *tdev)
{
	int ret = 0;
	struct dentry *tlc_root;
	struct dentry *d;

	tlc_root = debugfs_create_dir("tlc", tdev->tegra_debugfs);
	if (!tlc_root) {
		nvlink_err("Failed to create Tegra TLC debugfs root dir");
		return -ENOMEM;
	}

	d = debugfs_create_file("tp_cntrs", 0444, tlc_root, tdev,
							&tp_cntrs_fops);
	if (!d) {
		nvlink_err(
		"Unable to create debugfs node for tp_cntrs");
		return -ENOMEM;
	}

	d = debugfs_create_file("start_tp_cntrs", 0644, tlc_root, tdev,
							&start_tp_cntrs_fops);
	if (!d) {
		nvlink_err(
		"Unable to create debugfs node for start_tp_cntrs");
		return -ENOMEM;
	}

	d = debugfs_create_file("reset_tp_cntrs", 0244, tlc_root, tdev,
							&reset_tp_cntrs_fops);
	if (!d) {
		nvlink_err(
		"Unable to create debugfs node for reset_tp_cntrs");
		return -ENOMEM;
	}
	return ret;
}

void t19x_nvlink_endpt_debugfs_init(struct tnvlink_dev *tdev)
{
	if (!nvlink_debugfs_root) {
		nvlink_err("Root NVLINK debugfs directory doesn't exist");
		goto fail;
	}

	tdev->tegra_debugfs = debugfs_create_dir(NVLINK_MODULE_NAME,
						nvlink_debugfs_root);
	if (!tdev->tegra_debugfs) {
		nvlink_err("Failed to create Tegra NVLINK endpoint driver's"
			" debugfs directory");
		goto fail;
	}

	/* nvlink_rate_config: to switch and set different NVLINK RefCLK rate */
	tdev->tegra_debugfs_file = debugfs_create_file("refclk_rate",
				(S_IWUSR | S_IRUGO), tdev->tegra_debugfs,
				tdev, &nvlink_refclk_rate_fops);
	if (IS_ERR_OR_NULL(tdev->tegra_debugfs_file)) {
		tdev->tegra_debugfs_file = NULL;
		nvlink_dbg("debugfs_create_file() for nvlink_refclk_rate failed");
		goto fail;
	}
	/* nvlink_rate_config: to switch and set different NVLINK Speed Control */
	tdev->tegra_debugfs_file = debugfs_create_file("speed_control",
				(S_IWUSR | S_IRUGO), tdev->tegra_debugfs,
				tdev, &nvlink_speedcontrol_fops);
	if (IS_ERR_OR_NULL(tdev->tegra_debugfs_file)) {
		tdev->tegra_debugfs_file = NULL;
		nvlink_dbg("debugfs_create_file() for nvlink_rate_config failed");
		goto fail;
	}

	/* nvlink_shutdown: to shutdown the nvlink */
	tdev->tegra_debugfs_file = debugfs_create_file("shutdown",
				S_IWUSR, tdev->tegra_debugfs,
				tdev, &nvlink_shutdown_fops);
	if (IS_ERR_OR_NULL(tdev->tegra_debugfs_file)) {
		tdev->tegra_debugfs_file = NULL;
		nvlink_err("debugfs_create_file() for nvlink_shutdown failed");
		goto fail;
	}

	if (nvlink_single_lane_debugfs_init(tdev) < 0)
		goto fail;

	if (nvlink_tlc_debugfs_init(tdev) < 0)
		goto fail;

	if (!debugfs_create_bool("is_nea", (S_IWUSR | S_IRUGO),
					tdev->tegra_debugfs,
					&tdev->is_nea)) {
		nvlink_err("Unable to create debugfs node for is_nea");
		goto fail;
	}

	return;

fail:
	nvlink_err("Failed to create debugfs nodes");
	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}

void t19x_nvlink_endpt_debugfs_deinit(struct tnvlink_dev *tdev)
{
	debugfs_remove_recursive(tdev->tegra_debugfs);
	tdev->tegra_debugfs = NULL;
}
