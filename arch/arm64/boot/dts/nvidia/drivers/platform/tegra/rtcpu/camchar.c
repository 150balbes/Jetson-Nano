/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.	See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/bitmap.h>
#include <linux/cdev.h>
#include <linux/dcache.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#endif
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/wait.h>
#include <asm/ioctls.h>
#include <asm/uaccess.h>

#define CCIOGNFRAMES _IOR('c', 1, int)
#define CCIOGNBYTES _IOR('c', 2, int)

struct tegra_camchar_data {
	struct cdev cdev;
	struct tegra_ivc_channel *ch;
	struct mutex io_lock;
	wait_queue_head_t waitq;
	bool is_open;
	bool is_established;
};

#define DEVICE_COUNT (128)

static DECLARE_BITMAP(tegra_camchar_minor_map, DEVICE_COUNT);
static DEFINE_SPINLOCK(tegra_camchar_lock);
static dev_t tegra_camchar_major_number;
static struct class *tegra_camchar_class;

static int tegra_camchar_open(struct inode *in, struct file *f)
{
	struct tegra_camchar_data *data;
	int ret;

	data = container_of(in->i_cdev, struct tegra_camchar_data, cdev);
	if (data->is_open)
		return -EBUSY;

	ret = tegra_ivc_channel_runtime_get(data->ch);
	if (ret < 0)
		return ret;

	data->is_open = true;
	data->is_established = false;
	f->private_data = data->ch;

	return nonseekable_open(in, f);
}

static int tegra_camchar_release(struct inode *in, struct file *fp)
{
	struct tegra_ivc_channel *ch = fp->private_data;
	struct tegra_camchar_data *data;

	data = tegra_ivc_channel_get_drvdata(ch);
	tegra_ivc_channel_runtime_put(ch);
	data->is_open = false;

	return 0;
}

static unsigned int tegra_camchar_poll(struct file *fp, poll_table *pt)
{
	unsigned int ret = 0;
	struct tegra_ivc_channel *ch = fp->private_data;
	struct tegra_camchar_data *dev_data = tegra_ivc_channel_get_drvdata(ch);

	poll_wait(fp, &dev_data->waitq, pt);

	mutex_lock(&dev_data->io_lock);
	if (tegra_ivc_can_read(&ch->ivc))
		ret |= (POLLIN | POLLRDNORM);
	if (tegra_ivc_can_write(&ch->ivc))
		ret |= (POLLOUT | POLLWRNORM);
	mutex_unlock(&dev_data->io_lock);

	return ret;
}

static ssize_t tegra_camchar_read(struct file *fp, char __user *buffer, size_t len,
					loff_t *offset)
{
	struct tegra_ivc_channel *ch = fp->private_data;
	struct tegra_camchar_data *dev_data = tegra_ivc_channel_get_drvdata(ch);
	DEFINE_WAIT(wait);
	ssize_t ret;

	if (WARN_ON(!ch->is_ready))
		return -EIO;

	len = min_t(size_t, len, ch->ivc.frame_size);
	if (len == 0)
		return 0;

	do {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			break;
		prepare_to_wait(&dev_data->waitq, &wait, TASK_INTERRUPTIBLE);

		ret = tegra_ivc_read_user(&ch->ivc, buffer, len);
		mutex_unlock(&dev_data->io_lock);

		if (ret != -ENOMEM)
			;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();

		finish_wait(&dev_data->waitq, &wait);

	} while (ret == -ENOMEM);

	return ret;
}

static ssize_t tegra_camchar_write(struct file *fp, const char __user *buffer,
					size_t len, loff_t *offset)
{
	struct tegra_ivc_channel *ch = fp->private_data;
	struct tegra_camchar_data *dev_data = tegra_ivc_channel_get_drvdata(ch);
	DEFINE_WAIT(wait);
	ssize_t ret;

	if (WARN_ON(!ch->is_ready))
		return -EIO;

	len = min_t(size_t, len, ch->ivc.frame_size);
	if (len == 0)
		return 0;

	do {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			break;

		prepare_to_wait(&dev_data->waitq, &wait, TASK_INTERRUPTIBLE);
		ret = tegra_ivc_write_user(&ch->ivc, buffer, len);
		mutex_unlock(&dev_data->io_lock);

		if (ret > 0)
			dev_data->is_established = true;

		if (ret != -ENOMEM && ret != ECONNRESET)
			;
		else if (ret == ECONNRESET && dev_data->is_established)
			;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();

		finish_wait(&dev_data->waitq, &wait);

		if (ret == ECONNRESET && dev_data->is_established)
			break;

	} while (ret == -ENOMEM || ret == -ECONNRESET);

	return ret;
}

static long tegra_camchar_ioctl(struct file *fp, unsigned int cmd,
				unsigned long arg)
{
	struct tegra_ivc_channel *ch = fp->private_data;
	struct tegra_camchar_data *dev_data = tegra_ivc_channel_get_drvdata(ch);
	long ret;
	int val = 0;

	mutex_lock(&dev_data->io_lock);

	switch (cmd) {
	/* generic serial port ioctls */
	case FIONREAD:
		ret = 0;
		if (tegra_ivc_can_read(&ch->ivc))
			val = ch->ivc.frame_size;
		ret = put_user(val, (int __user *)arg);
		break;
	/* ioctls specific to this device */
	case CCIOGNFRAMES:
		val = ch->ivc.nframes;
		ret = put_user(val, (int __user *)arg);
		break;
	case CCIOGNBYTES:
		val = ch->ivc.frame_size;
		ret = put_user(val, (int __user *)arg);
		break;

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&dev_data->io_lock);
	return ret;
}

static const struct file_operations tegra_camchar_fops = {
	.open = tegra_camchar_open,
	.poll = tegra_camchar_poll,
	.read = tegra_camchar_read,
	.write = tegra_camchar_write,
	.release = tegra_camchar_release,
	.unlocked_ioctl = tegra_camchar_ioctl,
	.compat_ioctl = tegra_camchar_ioctl,
	.llseek = no_llseek,
};

static int __init tegra_camchar_init(struct tegra_ivc_driver *drv)
{
	int ret;
	dev_t start;

	ret = alloc_chrdev_region(&start, 0, DEVICE_COUNT, "camchar");
	if (ret) {
		pr_alert("camchar: failed to allocate device numbers\n");
		return ret;
	}
	tegra_camchar_major_number = MAJOR(start);

	tegra_camchar_class = class_create(THIS_MODULE, "camchar_class");
	if (IS_ERR(tegra_camchar_class)) {
		pr_alert("camchar: failed to create class\n");
		ret = PTR_ERR(tegra_camchar_class);
		goto init_err_class;
	}

	ret = tegra_ivc_driver_register(drv);
	if (ret) {
		pr_alert("camchar: ivc driver registration failed\n");
		goto init_err_ivc;
	}

	pr_info("camchar: rtcpu character device driver loaded\n");
	return 0;

init_err_ivc:
	class_destroy(tegra_camchar_class);
init_err_class:
	unregister_chrdev_region(start, DEVICE_COUNT);
	return ret;
}

static void __exit tegra_camchar_exit(struct tegra_ivc_driver *drv)
{
	dev_t num = MKDEV(tegra_camchar_major_number, 0);

	tegra_ivc_driver_unregister(drv);
	class_destroy(tegra_camchar_class);
	unregister_chrdev_region(num, DEVICE_COUNT);
	tegra_camchar_major_number = 0;

	pr_info("camchar: unloaded rtcpu character device driver\n");
}

static void tegra_camchar_notify(struct tegra_ivc_channel *ch)
{
	struct tegra_camchar_data *dev_data = tegra_ivc_channel_get_drvdata(ch);

	wake_up_interruptible(&dev_data->waitq);
}

static int tegra_camchar_get_minor(void)
{
	int minor;

	spin_lock(&tegra_camchar_lock);

	minor = find_first_zero_bit(tegra_camchar_minor_map, DEVICE_COUNT);
	if (minor < DEVICE_COUNT)
		set_bit(minor, tegra_camchar_minor_map);
	else
		minor = -ENODEV;

	spin_unlock(&tegra_camchar_lock);

	return minor;
}

static void tegra_camchar_put_minor(unsigned minor)
{
	spin_lock(&tegra_camchar_lock);

	if (minor < DEVICE_COUNT)
		clear_bit(minor, tegra_camchar_minor_map);

	spin_unlock(&tegra_camchar_lock);
}

static int tegra_camchar_probe(struct tegra_ivc_channel *ch)
{
	const char *devname;
	struct tegra_camchar_data *data;
	int ret, minor;
	dev_t num;
	struct device *dummy;

	devname = of_device_get_match_data(&ch->dev);
	if (devname == NULL) {
		ret = of_property_read_string(ch->dev.of_node,
					"nvidia,devname", &devname);
		if (ret != 0)
			return ret;
	}

	dev_dbg(&ch->dev, "probing /dev/%s", devname);

	data = devm_kzalloc(&ch->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ch = ch;
	cdev_init(&data->cdev, &tegra_camchar_fops);
	data->cdev.owner = THIS_MODULE;
	init_waitqueue_head(&data->waitq);
	mutex_init(&data->io_lock);

	tegra_ivc_channel_set_drvdata(ch, data);

	minor = tegra_camchar_get_minor();
	if (minor < 0)
		return minor;

	num = MKDEV(tegra_camchar_major_number, minor);
	ret = cdev_add(&data->cdev, num, 1);
	if (ret) {
		dev_warn(&ch->dev, "cannot add /dev/%s\n", devname);
		tegra_camchar_put_minor(minor);
		return ret;
	}

	dummy = device_create(tegra_camchar_class, &ch->dev, num, NULL,
			"%s", devname);
	if (IS_ERR(dummy)) {
		dev_err(&ch->dev, "cannot create /dev/%s\n", devname);
		tegra_camchar_put_minor(minor);
		return PTR_ERR(dummy);
	}

	return ret;
}

static void tegra_camchar_remove(struct tegra_ivc_channel *ch)
{
	struct tegra_camchar_data *data = tegra_ivc_channel_get_drvdata(ch);
	dev_t num = data->cdev.dev;

	device_destroy(tegra_camchar_class, num);
	cdev_del(&data->cdev);
	tegra_camchar_put_minor(MINOR(num));
}

static const struct tegra_ivc_channel_ops tegra_ivc_channel_chardev_ops = {
	.probe	= tegra_camchar_probe,
	.remove	= tegra_camchar_remove,
	.notify	= tegra_camchar_notify,
};

static const struct of_device_id camchar_of_match[] = {
	{ .compatible = "nvidia,tegra-ivc-cdev" },
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-echo",
		.data = (void *)"camchar-echo", },
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-dbg",
		.data = (void *)"camchar-dbg", },
	{ },
};

static struct tegra_ivc_driver camchar_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.bus	= &tegra_ivc_bus_type,
		.name	= "tegra-ivc-cdev",
		.of_match_table = camchar_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_ivc_channel_chardev_ops,
};

tegra_ivc_subsys_driver(camchar_driver, tegra_camchar_init, tegra_camchar_exit);
MODULE_AUTHOR("Jan Solanti <jsolanti@nvidia.com>");
MODULE_DESCRIPTION("The character device for ivc-bus");
MODULE_LICENSE("GPL v2");
