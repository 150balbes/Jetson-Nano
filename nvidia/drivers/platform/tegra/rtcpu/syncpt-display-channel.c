/*
 * SYNCPT DISPLAY CHANNEL DRIVER
 *
 * Copyright (c) 2015-2017 NVIDIA Corporation.  All rights reserved.
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

#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#endif
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>

struct tegra_ivc_syncpt_display {
	wait_queue_head_t waitq;
	struct dentry *root;
	struct completion echo_complete;
};

#define TEGRA_IVC_DC_SYNCPT_TIMEOUT_MS_DEFAULT	100

static int camrtc_syncpt_display_channel_echo(struct seq_file *file, void *data);

#define INIT_OPEN_FOPS(_open) { \
	.open = _open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release \
}

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
static int _fops_ ## _open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, _show_, inode->i_private); \
} \
static const struct file_operations _fops_ = INIT_OPEN_FOPS(_fops_ ## _open)

static int camrtc_show_rtcpu_utils_echo(struct seq_file *file, void *data)
{
	return camrtc_syncpt_display_channel_echo(file, data);
}
DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_syncpt_display_channel_echo, camrtc_show_rtcpu_utils_echo);

static int camrtc_syncpt_display_debugfs_init(struct tegra_ivc_channel *ch)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(ch);
	struct dentry *dir;
	char const *name = "rtcpu-utils";

	drvData->root = dir = debugfs_create_dir(name, NULL);
	if (dir == NULL){
		return -ENOMEM;
	}

	if (!debugfs_create_file("syncpt-echo", S_IRUGO, dir, ch, &camrtc_dbgfs_fops_syncpt_display_channel_echo)) {
		return -ENOMEM;
	}

	return 0;
}

static void tegra_ivc_channel_syncpt_display_recv(struct tegra_ivc_channel *chan,
					const void *data, size_t len)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(chan);

	complete(&drvData->echo_complete);
}

static void tegra_ivc_channel_syncpt_display_notify(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(chan);

	wake_up(&drvData->waitq);

	while (tegra_ivc_can_read(&chan->ivc)) {
		const void* data = tegra_ivc_read_get_next_frame(&chan->ivc);
		int length = chan->ivc.frame_size;

		tegra_ivc_channel_syncpt_display_recv(chan, data, length);
		tegra_ivc_read_advance(&chan->ivc);
	}
}

static int tegra_ivc_syncpt_display_channel_send(struct tegra_ivc_channel *chan,
					void *req, uint32_t req_length)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(chan);
	int ret = 0;
	long timeout = 0;

	timeout = wait_event_interruptible_timeout(drvData->waitq, tegra_ivc_can_write(&chan->ivc), timeout);
	if (timeout <= 0) {
		ret = timeout ?: -ETIMEDOUT;
		pr_err("%s: timeout (%ldms) for writing expired.\n", __func__, timeout);
		return ret;
	}

	ret = tegra_ivc_write(&chan->ivc, req, req_length);
	if (ret < 0) {
		pr_err("%s: tegra_ivc_write failed with %d.\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int tegra_ivc_syncpt_display_channel_prepare(struct tegra_ivc_channel *chan)
{
	int err = 0;

	err = tegra_ivc_channel_runtime_get(chan);

	if (err)
		tegra_ivc_channel_runtime_put(chan);

	return err;
}

static void tegra_ivc_syncpt_display_channel_complete(struct tegra_ivc_channel *chan)
{
	tegra_ivc_channel_runtime_put(chan);
}

static int camrtc_syncpt_display_channel_echo(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(ch);

	int ret = 0;
	u64 req = 0xDEADDEAD;
	u64 sent,recv;

	ret = tegra_ivc_syncpt_display_channel_prepare(ch);
	if(ret)
		return ret;

	sent = sched_clock();

	init_completion(&drvData->echo_complete);

	ret = tegra_ivc_syncpt_display_channel_send(ch,&req,sizeof(req));

	tegra_ivc_syncpt_display_channel_complete(ch);

	if(ret) {
		complete(&drvData->echo_complete);
	}
	else {
		ret = wait_for_completion_timeout(&drvData->echo_complete, TEGRA_IVC_DC_SYNCPT_TIMEOUT_MS_DEFAULT);
	}

	if (ret <= 0) {
		seq_printf(file,
	 	"Syncpt channel echo timeout expired ");
	} else {
		recv = sched_clock();

		seq_printf(file,
	 	"roundtrip=%llu.%03llu us "
	 	"(sent=%llu.%09llu recv=%llu.%09llu)\n",
	 	(recv - sent) / 1000, (recv - sent) % 1000,
	 	sent / 1000000000, sent % 1000000000,
	 	recv / 1000000000, recv % 1000000000);

	}
	return ret;
}

static int tegra_ivc_channel_syncpt_display_probe(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(chan);

	drvData = devm_kzalloc(&chan->dev, sizeof(*drvData), GFP_KERNEL);
	if (unlikely(drvData == NULL))
		return -ENOMEM;

	init_waitqueue_head(&drvData->waitq);

	tegra_ivc_channel_set_drvdata(chan, drvData);

	return camrtc_syncpt_display_debugfs_init(chan);

}

static void tegra_ivc_channel_syncpt_display_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_syncpt_display *drvData = tegra_ivc_channel_get_drvdata(chan);
	debugfs_remove_recursive(drvData->root);
}

static struct of_device_id tegra_ivc_channel_syncpt_display_of_match[] = {
	{ .compatible = "nvidia,tegra-ivc-syncpt-disp" },
	{ },
};

static const struct tegra_ivc_channel_ops tegra_ivc_channel_syncpt_display_ops = {
	.probe	    = tegra_ivc_channel_syncpt_display_probe,
	.remove	    = tegra_ivc_channel_syncpt_display_remove,
	.notify 	= tegra_ivc_channel_syncpt_display_notify,
};

static struct tegra_ivc_driver tegra_ivc_channel_syncpt_display_driver = {
	.driver = {
		.name	= "tegra-ivc-syncpt-display",
		.bus	= &tegra_ivc_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = tegra_ivc_channel_syncpt_display_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_ivc_channel_syncpt_display_ops,
};
tegra_ivc_module_driver(tegra_ivc_channel_syncpt_display_driver);
