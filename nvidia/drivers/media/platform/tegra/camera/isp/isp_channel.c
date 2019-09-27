/*
 * ISP channel driver for T186
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
#include <linux/stddef.h>
#include <linux/uaccess.h>

#include <media/capture_isp.h>
#include <media/isp_channel.h>

#include "nvhost_acm.h"

#define MAX_ISP_CHANNELS 64

#define ISP_CAPTURE_SETUP	\
		_IOW('I', 1, struct isp_capture_setup)
#define ISP_CAPTURE_RELEASE	\
		_IOW('I', 2, __u32)
#define ISP_CAPTURE_RESET	\
		_IOW('I', 3, __u32)
#define ISP_CAPTURE_GET_INFO	\
		_IOR('I', 4, struct isp_capture_info)
#define ISP_CAPTURE_REQUEST	\
		_IOW('I', 5, struct isp_capture_req)
#define ISP_CAPTURE_STATUS	\
		_IOW('I', 6, __u32)
#define ISP_CAPTURE_PROGRAM_REQUEST	\
		_IOW('I', 7, struct isp_program_req)
#define ISP_CAPTURE_PROGRAM_STATUS	\
		_IOW('I', 8, __u32)
#define ISP_CAPTURE_REQUEST_EX \
		_IOW('I', 9, struct isp_capture_req_ex)
#define ISP_CAPTURE_SET_PROGRESS_STATUS_NOTIFIER \
		_IOW('I', 10, struct isp_capture_progress_status_req)

struct isp_channel_drv {
	struct device *dev;
	dev_t major;
	u8 num_channels;
	struct mutex lock;
	struct platform_device *ndev;
	const struct isp_channel_drv_ops *ops;
	struct tegra_isp_channel *channels[];
};

static long isp_channel_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct tegra_isp_channel *chan = file->private_data;
	void __user *ptr = (void __user *)arg;
	long err = -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(ISP_CAPTURE_SETUP): {
		struct isp_capture_setup setup;
		if (copy_from_user(&setup, ptr, sizeof(setup)))
			break;
		err = isp_capture_setup(chan, &setup);
		if (err)
			dev_err(chan->isp_dev, "isp capture setup failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_RESET): {
		uint32_t rst;
		if (copy_from_user(&rst, ptr, sizeof(rst)))
			break;
		err = isp_capture_reset(chan, rst);
		if (err)
			dev_err(chan->isp_dev, "isp capture reset failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_RELEASE): {
		uint32_t rel;
		if (copy_from_user(&rel, ptr, sizeof(rel)))
			break;
		err = isp_capture_release(chan, rel);
		if (err)
			dev_err(chan->isp_dev, "isp capture release failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_GET_INFO): {
		struct isp_capture_info info;
		err = isp_capture_get_info(chan, &info);
		if (err)
			dev_err(chan->isp_dev, "isp capture get info failed\n");
		if (copy_to_user(ptr, &info, sizeof(info)))
			err = -EFAULT;
		break;
	}

	case _IOC_NR(ISP_CAPTURE_REQUEST): {
		struct isp_capture_req req;
		if (copy_from_user(&req, ptr, sizeof(req)))
			break;
		err = isp_capture_request(chan, &req);
		if (err)
			dev_err(chan->isp_dev,
				"isp capture request submit failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_STATUS): {
		uint32_t status;
		if (copy_from_user(&status, ptr, sizeof(status)))
			break;
		err = isp_capture_status(chan, status);
		if (err)
			dev_err(chan->isp_dev,
				"isp capture get status failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_PROGRAM_REQUEST): {
		struct isp_program_req program_req;
		if (copy_from_user(&program_req, ptr, sizeof(program_req)))
			break;
		err = isp_capture_program_request(chan, &program_req);
		if (err)
			dev_err(chan->isp_dev,
				"isp program request submit failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_PROGRAM_STATUS): {
		err = isp_capture_program_status(chan);
		if (err)
			dev_err(chan->isp_dev,
				"isp program get status failed\n");
		break;
	}

	case _IOC_NR(ISP_CAPTURE_REQUEST_EX): {
		struct isp_capture_req_ex req;

		if (copy_from_user(&req, ptr, sizeof(req)))
			break;
		err = isp_capture_request_ex(chan, &req);
		if (err)
			dev_err(chan->isp_dev,
					"isp capture request extended submit failed\n");
		break;
	}
	case _IOC_NR(ISP_CAPTURE_SET_PROGRESS_STATUS_NOTIFIER): {
		struct isp_capture_progress_status_req req;

		if (copy_from_user(&req, ptr, sizeof(req)))
			break;
		err = isp_capture_set_progress_status_notifier(chan, &req);
		if (err)
			dev_err(chan->isp_dev,
					"isp capture set progress status buffers failed\n");
		break;
	}
	default: {
		dev_err(chan->isp_dev, "%s:Unknown ioctl\n", __func__);
		return -ENOIOCTLCMD;
	}
	}

	return err;
}

static int isp_channel_power_on(struct tegra_isp_channel *chan)
{
	int ret = 0;

	dev_dbg(chan->isp_dev, "isp_channel_power_on\n");

	ret = nvhost_module_add_client(chan->ndev, chan->priv);
	if (ret < 0) {
		dev_err(chan->isp_dev, "%s: failed to add isp client\n", __func__);
		return ret;
	}

	ret = nvhost_module_busy(chan->ndev);
	if (ret < 0) {
		dev_err(chan->isp_dev, "%s: failed to power on isp\n", __func__);
		return ret;
	}

	return 0;
}

static void isp_channel_power_off(struct tegra_isp_channel *chan)
{
	dev_dbg(chan->isp_dev, "isp_channel_power_off\n");

	nvhost_module_idle(chan->ndev);
	nvhost_module_remove_client(chan->ndev, chan->priv);
}

static struct isp_channel_drv *chdrv_;
static DEFINE_MUTEX(chdrv_lock);

static int isp_channel_open(struct inode *inode, struct file *file)
{
	struct tegra_isp_channel *chan;
	unsigned channel = iminor(inode);
	struct isp_channel_drv *chan_drv;
	int err;

	if (mutex_lock_interruptible(&chdrv_lock))
		return -ERESTARTSYS;

	chan_drv = chdrv_;

	if (chan_drv == NULL || channel >= chan_drv->num_channels) {
		mutex_unlock(&chdrv_lock);
		return -ENODEV;
	}
	mutex_unlock(&chdrv_lock);

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL))
		return -ENOMEM;

	chan->drv = chan_drv;
	chan->isp_dev = chan_drv->dev;
	chan->ndev = chan_drv->ndev;
	chan->ops = chan_drv->ops;
	chan->priv = file;

	err = isp_channel_power_on(chan);
	if (err < 0)
		goto error;

	err = isp_capture_init(chan);
	if (err < 0)
		goto init_err;

	mutex_lock(&chan_drv->lock);
	if (chan_drv->channels[channel] != NULL) {
		mutex_unlock(&chan_drv->lock);
		err = -EBUSY;
		goto chan_err;
	}

	chan_drv->channels[channel] = chan;
	mutex_unlock(&chan_drv->lock);

	file->private_data = chan;

	return nonseekable_open(inode, file);

chan_err:
	isp_capture_shutdown(chan);
init_err:
	isp_channel_power_off(chan);
error:
	kfree(chan);
	return err;
}

static int isp_channel_release(struct inode *inode, struct file *file)
{
	struct tegra_isp_channel *chan = file->private_data;
	unsigned channel = iminor(inode);
	struct isp_channel_drv *chan_drv = chan->drv;

	isp_capture_shutdown(chan);
	isp_channel_power_off(chan);

	mutex_lock(&chan_drv->lock);

	WARN_ON(chan_drv->channels[channel] != chan);
	chan_drv->channels[channel] = NULL;

	mutex_unlock(&chan_drv->lock);
	kfree(chan);

	return 0;
}

static const struct file_operations isp_channel_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.unlocked_ioctl = isp_channel_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = isp_channel_ioctl,
#endif
	.open = isp_channel_open,
	.release = isp_channel_release,
};

/* Character device */
static struct class *isp_channel_class;
static int isp_channel_major;

int isp_channel_drv_register(struct platform_device *ndev,
		const struct isp_channel_drv_ops *ops)
{
	struct isp_channel_drv *chan_drv;
	unsigned i;

	chan_drv = kzalloc(offsetof(struct isp_channel_drv,
			channels[MAX_ISP_CHANNELS]), GFP_KERNEL);
	if (unlikely(chan_drv == NULL))
		return -ENOMEM;

	chan_drv->dev = &ndev->dev;
	chan_drv->ndev = ndev;
	chan_drv->ops = ops;
	chan_drv->num_channels = MAX_ISP_CHANNELS;
	mutex_init(&chan_drv->lock);

	mutex_lock(&chdrv_lock);
	if (WARN_ON(chdrv_ != NULL)) {
		mutex_unlock(&chdrv_lock);
		kfree(chan_drv);
		return -EBUSY;
	}
	chdrv_ = chan_drv;
	mutex_unlock(&chdrv_lock);

	for (i = 0; i < chan_drv->num_channels; i++) {
		dev_t devt = MKDEV(isp_channel_major, i);

		device_create(isp_channel_class, chan_drv->dev, devt, NULL,
				"capture-isp-channel%u", i);
	}

	return 0;
}
EXPORT_SYMBOL(isp_channel_drv_register);

void isp_channel_drv_unregister(struct device *dev)
{
	struct isp_channel_drv *chan_drv;
	unsigned i;

	mutex_lock(&chdrv_lock);
	chan_drv = chdrv_;
	chdrv_ = NULL;
	WARN_ON(chan_drv->dev != dev);
	mutex_unlock(&chdrv_lock);

	for (i = 0; i < chan_drv->num_channels; i++) {
		dev_t devt = MKDEV(isp_channel_major, i);

		device_destroy(isp_channel_class, devt);
	}

	kfree(chan_drv);
}
EXPORT_SYMBOL(isp_channel_drv_unregister);

static int __init isp_channel_drv_init(void)
{
	isp_channel_class = class_create(THIS_MODULE, "capture-isp-channel");
	if (IS_ERR(isp_channel_class))
		return PTR_ERR(isp_channel_class);

	isp_channel_major = register_chrdev(0, "capture-isp-channel",
						&isp_channel_fops);
	if (isp_channel_major < 0) {
		class_destroy(isp_channel_class);
		return isp_channel_major;
	}

	return 0;
}

static void __exit isp_channel_drv_exit(void)
{
	unregister_chrdev(isp_channel_major, "capture-isp-channel");
	class_destroy(isp_channel_class);
}

subsys_initcall(isp_channel_drv_init);
module_exit(isp_channel_drv_exit);
