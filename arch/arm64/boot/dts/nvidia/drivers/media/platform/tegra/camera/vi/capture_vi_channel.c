/*
 * VI channel driver for T186/T194
 *
 * Copyright (c) 2017-2018 NVIDIA Corporation.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
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
#include <asm/ioctls.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/of_platform.h>
#include <linux/module.h>
#include <linux/nvhost.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <media/capture.h>
#include <media/capture_vi_channel.h>

#include "nvhost_acm.h"

/** TODO: Get it from  DT */
#define MAX_VI_CHANNELS 64

#define VI_CAPTURE_SETUP	_IOW('I', 1, struct vi_capture_setup)
#define VI_CAPTURE_RELEASE	_IOW('I', 2, __u32)
#define VI_CAPTURE_SET_CONFIG	_IOW('I', 3, struct vi_capture_control_msg)
#define VI_CAPTURE_RESET	_IOW('I', 4, __u32)
#define VI_CAPTURE_GET_INFO	_IOR('I', 5, struct vi_capture_info)
#define VI_CAPTURE_REQUEST	_IOW('I', 6, struct vi_capture_req)
#define VI_CAPTURE_STATUS	_IOW('I', 7, __u32)
#define VI_CAPTURE_SET_COMPAND	_IOW('I', 8, struct vi_capture_compand)
#define VI_CAPTURE_SET_PROGRESS_STATUS_NOTIFIER \
	_IOW('I', 9, struct vi_capture_progress_status_req)

struct vi_channel_drv {
	struct device *dev;
	struct platform_device *ndev;
	struct mutex lock;
	u8 num_channels;
	const struct vi_channel_drv_ops *ops;
	struct tegra_vi_channel __rcu *channels[];
};

void vi_capture_request_unpin(struct tegra_vi_channel *chan,
		uint32_t buffer_index)
{
	struct vi_capture *capture = chan->capture_data;
	struct capture_common_unpins *unpins;
	int i = 0;

	mutex_lock(&capture->unpins_list_lock);
	unpins = capture->unpins_list[buffer_index];
	if (unpins != NULL) {
		for (i = 0; i < unpins->num_unpins; i++)
			capture_common_unpin_memory(&unpins->data[i]);
		capture->unpins_list[buffer_index] = NULL;
		kfree(unpins);
	}
	mutex_unlock(&capture->unpins_list_lock);
}
EXPORT_SYMBOL(vi_capture_request_unpin);

static long vi_channel_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct tegra_vi_channel *chan = file->private_data;
	struct vi_capture *capture = chan->capture_data;
	void __user *ptr = (void __user *)arg;
	int err = -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(VI_CAPTURE_SETUP): {
		struct vi_capture_setup setup;

		if (copy_from_user(&setup, ptr, sizeof(setup)))
			break;

		/* pin the capture descriptor ring buffer */
		err = capture_common_pin_memory(capture->rtcpu_dev,
				setup.mem, &capture->requests);
		if (err < 0) {
			dev_err(chan->dev, "%s: memory setup failed\n", __func__);
			return -EFAULT;
		}

		/* allocate for unpin list based on queue depth */
		capture->unpins_list = kcalloc(setup.queue_depth,
				sizeof(struct capture_common_unpins *),
				GFP_KERNEL);
		if (unlikely(capture->unpins_list == NULL)) {
			dev_err(chan->dev, "failed to allocate unpins array\n");
			capture_common_unpin_memory(&capture->requests);
			return -ENOMEM;
		}

		setup.iova = capture->requests.iova;
		err = vi_capture_setup(chan, &setup);
		if (err < 0) {
			dev_err(chan->dev, "vi capture setup failed\n");
			kfree(capture->unpins_list);
			capture_common_unpin_memory(&capture->requests);
			return err;
		}
		break;
	}

	case _IOC_NR(VI_CAPTURE_RESET): {
		uint32_t reset_flags;
		int i;

		if (copy_from_user(&reset_flags, ptr, sizeof(reset_flags)))
			break;

		err = vi_capture_reset(chan, reset_flags);
		if (err < 0)
			dev_err(chan->dev, "vi capture reset failed\n");
		else {
			for (i = 0; i < capture->queue_depth; i++)
				vi_capture_request_unpin(chan, i);
		}
		break;
	}

	case _IOC_NR(VI_CAPTURE_RELEASE): {
		uint32_t reset_flags;
		int i;

		if (copy_from_user(&reset_flags, ptr, sizeof(reset_flags)))
			break;

		err = vi_capture_release(chan, reset_flags);
		if (err < 0)
			dev_err(chan->dev, "vi capture release failed\n");
		else {
			for (i = 0; i < capture->queue_depth; i++)
				vi_capture_request_unpin(chan, i);
			capture_common_unpin_memory(&capture->requests);
			kfree(capture->unpins_list);
		}
		break;
	}

	case _IOC_NR(VI_CAPTURE_GET_INFO): {
		struct vi_capture_info info;

		err = vi_capture_get_info(chan, &info);
		if (err < 0)
			dev_err(chan->dev, "vi capture get info failed\n");
		if (copy_to_user(ptr, &info, sizeof(info)))
			err = -EFAULT;
		break;
	}

	case _IOC_NR(VI_CAPTURE_SET_CONFIG): {
		struct vi_capture_control_msg msg;

		if (copy_from_user(&msg, ptr, sizeof(msg)))
			break;
		err = vi_capture_control_message(chan, &msg);
		if (err < 0)
			dev_err(chan->dev, "vi capture set config failed\n");
		break;
	}

	case _IOC_NR(VI_CAPTURE_REQUEST): {
		struct vi_capture_req req;
		struct capture_common_pin_req cap_common_req;

		if (copy_from_user(&req, ptr, sizeof(req)))
			break;

		if (req.num_relocs == 0) {
			dev_err(chan->dev, "request must have non-zero relocs\n");
			return -EINVAL;
		}

		/* pin and reloc */
		memset(&cap_common_req, 0, sizeof(cap_common_req));
		cap_common_req.dev = chan->dev;
		cap_common_req.rtcpu_dev = capture->rtcpu_dev;
		cap_common_req.unpins = NULL;
		cap_common_req.requests = &capture->requests;
		cap_common_req.requests_dev = NULL;
		cap_common_req.request_size = capture->request_size;
		cap_common_req.request_offset = req.buffer_index *
				capture->request_size;
		cap_common_req.num_relocs = req.num_relocs;
		cap_common_req.reloc_user = (uint32_t __user *)
				(uintptr_t)req.reloc_relatives;

		err = capture_common_request_pin_and_reloc(&cap_common_req);
		if (err < 0) {
			dev_err(chan->dev, "request relocation failed\n");
			return err;
		}

		/* assign the unpins list to the capture to be unpinned and */
		/* freed at capture completion (vi_capture_request_unpin) */
		mutex_lock(&capture->unpins_list_lock);
		capture->unpins_list[req.buffer_index] = cap_common_req.unpins;
		mutex_unlock(&capture->unpins_list_lock);

		err = vi_capture_request(chan, &req);
		if (err < 0) {
			dev_err(chan->dev,
				"vi capture request submit failed\n");
			vi_capture_request_unpin(chan, req.buffer_index);
		}
		break;
	}

	case _IOC_NR(VI_CAPTURE_STATUS): {
		uint32_t timeout_ms;

		if (copy_from_user(&timeout_ms, ptr, sizeof(timeout_ms)))
			break;
		err = vi_capture_status(chan, timeout_ms);
		if (err < 0)
			dev_err(chan->dev,
				"vi capture get status failed\n");
		break;
	}

	case _IOC_NR(VI_CAPTURE_SET_COMPAND): {
		struct vi_capture_compand compand;

		if (copy_from_user(&compand, ptr, sizeof(compand)))
			break;
		err = vi_capture_set_compand(chan, &compand);
		if (err < 0)
			dev_err(chan->dev,
				"setting compand failed\n");
		break;
	}

	case _IOC_NR(VI_CAPTURE_SET_PROGRESS_STATUS_NOTIFIER): {
		struct vi_capture_progress_status_req req;

		if (copy_from_user(&req, ptr, sizeof(req)))
			break;
		err = vi_capture_set_progress_status_notifier(chan, &req);
		if (err < 0)
			dev_err(chan->dev,
					"setting progress status buffer failed\n");
		break;
	}
	default: {
		dev_err(chan->dev, "%s:Unknown ioctl\n", __func__);
		return -ENOIOCTLCMD;
	}
	}

	return err;
}

static struct vi_channel_drv *chdrv_;
static DEFINE_MUTEX(chdrv_lock);

static int vi_channel_power_on_vi_device(struct tegra_vi_channel *chan)
{
	int ret = 0;

	dev_dbg(chan->dev, "vi_channel_power_on_vi_device\n");

	ret = nvhost_module_add_client(chan->ndev, chan->capture_data);
	if (ret < 0) {
		dev_err(chan->dev, "%s: failed to add vi client\n", __func__);
		return ret;
	}

	ret = nvhost_module_busy(chan->ndev);
	if (ret < 0) {
		dev_err(chan->dev, "%s: failed to power on vi\n", __func__);
		return ret;
	}

	return 0;
}

static void vi_channel_power_off_vi_device(struct tegra_vi_channel *chan)
{
	dev_dbg(chan->dev, "vi_channel_power_off_vi_device\n");

	nvhost_module_idle(chan->ndev);
	nvhost_module_remove_client(chan->ndev, chan->capture_data);
}

struct tegra_vi_channel *vi_channel_open_ex(unsigned channel, bool is_mem_pinned)
{
	struct tegra_vi_channel *chan;
	struct vi_channel_drv *chan_drv;
	int err;

	if (mutex_lock_interruptible(&chdrv_lock))
		return ERR_PTR(-ERESTARTSYS);

	chan_drv = chdrv_;

	if (chan_drv == NULL || channel >= chan_drv->num_channels) {
		mutex_unlock(&chdrv_lock);
		return ERR_PTR(-ENODEV);
	}
	mutex_unlock(&chdrv_lock);

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL))
		return ERR_PTR(-ENOMEM);

	chan->drv = chan_drv;
	chan->dev = chan_drv->dev;
	chan->ndev = chan_drv->ndev;
	chan->ops = chan_drv->ops;

	err = vi_channel_power_on_vi_device(chan);
	if (err < 0)
		goto error;

	err = vi_capture_init(chan, is_mem_pinned);
	if (err < 0)
		goto error;

	mutex_lock(&chan_drv->lock);
	if (rcu_access_pointer(chan_drv->channels[channel]) != NULL) {
		mutex_unlock(&chan_drv->lock);
		err = -EBUSY;
		goto rcu_err;
	}

	rcu_assign_pointer(chan_drv->channels[channel], chan);
	mutex_unlock(&chan_drv->lock);

	return chan;

rcu_err:
	vi_channel_power_off_vi_device(chan);
	vi_capture_shutdown(chan);
error:
	kfree(chan);
	return ERR_PTR(err);
}
EXPORT_SYMBOL(vi_channel_open_ex);

int vi_channel_close_ex(unsigned channel, struct tegra_vi_channel *chan)
{
	struct vi_channel_drv *chan_drv = chan->drv;

	vi_capture_shutdown(chan);
	vi_channel_power_off_vi_device(chan);

	mutex_lock(&chan_drv->lock);

	WARN_ON(rcu_access_pointer(chan_drv->channels[channel]) != chan);
	RCU_INIT_POINTER(chan_drv->channels[channel], NULL);

	mutex_unlock(&chan_drv->lock);
	kfree_rcu(chan, rcu);

	return 0;
}
EXPORT_SYMBOL(vi_channel_close_ex);

static int vi_channel_open(struct inode *inode, struct file *file)
{
	unsigned channel = iminor(inode);
	struct tegra_vi_channel *chan;

	chan = vi_channel_open_ex(channel, true);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	file->private_data = chan;

	return nonseekable_open(inode, file);
}

static int vi_channel_release(struct inode *inode, struct file *file)
{
	struct tegra_vi_channel *chan = file->private_data;
	unsigned channel = iminor(inode);

	vi_channel_close_ex(channel, chan);

	return 0;
}

static const struct file_operations vi_channel_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = vi_channel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vi_channel_ioctl,
#endif
	.open = vi_channel_open,
	.release = vi_channel_release,
};

/* Character device */
static struct class *vi_channel_class;
static int vi_channel_major;

int vi_channel_drv_register(struct platform_device *ndev,
	const struct vi_channel_drv_ops *ops)
{
	struct vi_channel_drv *chan_drv;
	int err = 0;
	unsigned i;

	chan_drv = devm_kzalloc(&ndev->dev, sizeof(*chan_drv) +
			MAX_VI_CHANNELS * sizeof(struct tegra_vi_channel *),
			GFP_KERNEL);
	if (unlikely(chan_drv == NULL))
		return -ENOMEM;

	chan_drv->dev = &ndev->dev;
	chan_drv->ndev = ndev;
	chan_drv->ops = ops;
	chan_drv->num_channels = MAX_VI_CHANNELS;
	mutex_init(&chan_drv->lock);

	mutex_lock(&chdrv_lock);
	if (chdrv_ != NULL) {
		mutex_unlock(&chdrv_lock);
		WARN_ON(1);
		err = -EBUSY;
		goto error;
	}
	chdrv_ = chan_drv;
	mutex_unlock(&chdrv_lock);

	for (i = 0; i < chan_drv->num_channels; i++) {
		dev_t devt = MKDEV(vi_channel_major, i);

		device_create(vi_channel_class, chan_drv->dev, devt, NULL,
				"capture-vi-channel%u", i);
	}

	return 0;

error:
	return err;
}
EXPORT_SYMBOL(vi_channel_drv_register);

void vi_channel_drv_unregister(struct device *dev)
{
	struct vi_channel_drv *chan_drv;
	unsigned i;

	mutex_lock(&chdrv_lock);
	chan_drv = chdrv_;
	chdrv_ = NULL;
	WARN_ON(chan_drv->dev != dev);
	mutex_unlock(&chdrv_lock);

	for (i = 0; i < chan_drv->num_channels; i++) {
		dev_t devt = MKDEV(vi_channel_major, i);

		device_destroy(vi_channel_class, devt);
	}

	devm_kfree(chan_drv->dev, chan_drv);
}
EXPORT_SYMBOL(vi_channel_drv_unregister);

static int __init vi_channel_drv_init(void)
{
	vi_channel_class = class_create(THIS_MODULE, "capture-vi-channel");
	if (IS_ERR(vi_channel_class))
		return PTR_ERR(vi_channel_class);

	vi_channel_major = register_chrdev(0, "capture-vi-channel",
						&vi_channel_fops);
	if (vi_channel_major < 0) {
		class_destroy(vi_channel_class);
		return vi_channel_major;
	}

	return 0;
}

static void __exit vi_channel_drv_exit(void)
{
	unregister_chrdev(vi_channel_major, "capture-vi-channel");
	class_destroy(vi_channel_class);
}

subsys_initcall(vi_channel_drv_init);
module_exit(vi_channel_drv_exit);
