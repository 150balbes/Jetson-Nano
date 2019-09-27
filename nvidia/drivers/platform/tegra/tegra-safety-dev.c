/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/wait.h>
#include <asm/ioctls.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#endif

#include <linux/tegra-safety-ivc.h>

#define CCIOGNFRAMES _IOR('c', 1, int)
#define CCIOGNBYTES _IOR('c', 2, int)

struct tegra_safety_dev_data {
	struct tegra_safety_ivc_chan *ivc_chan;
	struct cdev cdev;
	struct mutex io_lock;
	wait_queue_head_t read_waitq;
	wait_queue_head_t write_waitq;
	int open_count;
};

static struct tegra_safety_dev_data *safety_dev_data[MAX_SAFETY_CHANNELS];
static DEFINE_MUTEX(tegra_safety_dev_lock_open);
static int tegra_safety_dev_major_number;
static struct class *tegra_safety_dev_class;
extern int ivc_chan_count;

static inline struct tegra_safety_dev_data *get_file_to_devdata(struct file *fp)
{
	return ((struct tegra_safety_dev_data *)fp->private_data);
}

static inline struct ivc *get_file_to_ivc(struct file *fp)
{
	struct tegra_safety_dev_data *dev_data = get_file_to_devdata(fp);

	return &dev_data->ivc_chan->ivc;
}

static int tegra_safety_dev_open(struct inode *in, struct file *f)
{
	unsigned int minor = iminor(in);
	struct tegra_safety_dev_data *dev_data = safety_dev_data[minor];
	int ret = -1;

	if (minor >= ivc_chan_count)
		return -EBADFD;

	ret = mutex_lock_interruptible(&tegra_safety_dev_lock_open);
	if (ret)
		return ret;

	/* For CmdRsp restrict device open to 2 */
	if ((minor == 0) && (dev_data->open_count >= 2)) {
		ret = -1;
		goto error;
	}

	/* For HeartBeat restrict device open to 1 */
	if ((minor == 1) && (dev_data->open_count >= 1)) {
		ret = -1;
		goto error;
	}

	dev_data->open_count++;
	f->private_data = dev_data;
	nonseekable_open(in, f);

error:
	mutex_unlock(&tegra_safety_dev_lock_open);

	return ret;
}

static int tegra_safety_dev_release(struct inode *in, struct file *fp)
{
	unsigned int minor = iminor(in);
	struct tegra_safety_dev_data *dev_data = safety_dev_data[minor];

	mutex_lock(&tegra_safety_dev_lock_open);
	dev_data->open_count--;
	mutex_unlock(&tegra_safety_dev_lock_open);

	return 0;
}

static unsigned int tegra_safety_dev_poll(struct file *fp, poll_table *pt)
{
	struct tegra_safety_dev_data *dev_data = get_file_to_devdata(fp);
	struct ivc *ivc = get_file_to_ivc(fp);
	unsigned int ret = 0;

	poll_wait(fp, &dev_data->read_waitq, pt);
	poll_wait(fp, &dev_data->write_waitq, pt);

	mutex_lock(&dev_data->io_lock);
	if (tegra_ivc_can_read(ivc))
		ret |= (POLLIN | POLLRDNORM);
	if (tegra_ivc_can_write(ivc))
		ret |= (POLLOUT | POLLWRNORM);
	mutex_unlock(&dev_data->io_lock);

	return ret;
}

static ssize_t tegra_safety_dev_read(struct file *fp, char __user *buffer,
		size_t len, loff_t *offset)
{
	struct tegra_safety_dev_data *dev_data = get_file_to_devdata(fp);
	struct ivc *ivc = get_file_to_ivc(fp);
	DEFINE_WAIT(wait);
	size_t maxbytes = len > ivc->frame_size ? ivc->frame_size : len;
	ssize_t ret = 0;
	bool done = false;

	/*
	 * here we are reading maxbytes of data from IVC. If data is
	 * present we will read it, otherwise do the wait.
	 */
	while (!ret && maxbytes) {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			return ret;
		prepare_to_wait(&dev_data->read_waitq, &wait,
				TASK_INTERRUPTIBLE);

		done = tegra_ivc_can_read(ivc);
		if (done)
			ret = tegra_ivc_read_user(ivc, buffer, maxbytes);
		mutex_unlock(&dev_data->io_lock);

		if (done)
			goto finish;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();
finish:
		finish_wait(&dev_data->read_waitq, &wait);
	};

	return ret;
}

static ssize_t tegra_safety_dev_write(struct file *fp,
		const char __user *buffer, size_t len, loff_t *offset)
{
	struct tegra_safety_dev_data *dev_data = get_file_to_devdata(fp);
	struct ivc *ivc = get_file_to_ivc(fp);
	DEFINE_WAIT(wait);
	size_t maxbytes = len > ivc->frame_size ? ivc->frame_size : len;
	ssize_t ret = 0;
	int done = false;

	/*
	 * here we are writing maxbytes of data to IVC. If space is
	 * available we will write it, otherwise do the wait.
	 */
	while (!ret && maxbytes) {
		ret = mutex_lock_interruptible(&dev_data->io_lock);
		if (ret)
			return ret;
		prepare_to_wait(&dev_data->write_waitq, &wait,
				TASK_INTERRUPTIBLE);

		done = tegra_ivc_can_write(ivc);
		if (done)
			ret = tegra_ivc_write_user(ivc, buffer, maxbytes);
		mutex_unlock(&dev_data->io_lock);

		if (done)
			goto finish;
		else if (signal_pending(current))
			ret = -EINTR;
		else if (fp->f_flags & O_NONBLOCK)
			ret = -EAGAIN;
		else
			schedule();
finish:
		finish_wait(&dev_data->write_waitq, &wait);
	}

	return ret;
}

static long tegra_safety_dev_ioctl(struct file *fp, unsigned int cmd,
		unsigned long arg)
{
	struct tegra_safety_dev_data *dev_data = get_file_to_devdata(fp);
	struct ivc *ivc = get_file_to_ivc(fp);
	int val = 0;
	long ret;

	mutex_lock(&dev_data->io_lock);

	switch (cmd) {
	case CCIOGNFRAMES:
		val = ivc->nframes;
		ret = put_user(val, (int __user *)arg);
		break;
	case CCIOGNBYTES:
		val = ivc->frame_size;
		ret = put_user(val, (int __user *)arg);
		break;

	default:
		ret = -ENOTTY;
	}

	mutex_unlock(&dev_data->io_lock);

	return ret;
}

static const struct file_operations tegra_safety_dev_fops = {
	.open = tegra_safety_dev_open,
	.poll = tegra_safety_dev_poll,
	.read = tegra_safety_dev_read,
	.write = tegra_safety_dev_write,
	.release = tegra_safety_dev_release,
	.unlocked_ioctl = tegra_safety_dev_ioctl,
	.compat_ioctl = tegra_safety_dev_ioctl,
	.llseek = no_llseek,
};

void tegra_safety_dev_notify(void)
{
	struct tegra_safety_dev_data *dev_data;
	struct ivc *ivc;
	int can_read, can_write;
	int i;

	if (!safety_dev_data[0])
		return;

	for (i = 0; i < ivc_chan_count; i++) {
		dev_data = safety_dev_data[i];
		if (!dev_data)
			return;

		ivc = &dev_data->ivc_chan->ivc;

		mutex_lock(&dev_data->io_lock);
		can_read = tegra_ivc_can_read(ivc);
		can_write = tegra_ivc_can_write(ivc);
		mutex_unlock(&dev_data->io_lock);

		if (can_read)
			wake_up_interruptible(&dev_data->read_waitq);

		if (can_write)
			wake_up_interruptible(&dev_data->write_waitq);
	}
}

void tegra_safety_class_exit(struct device *dev)
{
	dev_t num = MKDEV(tegra_safety_dev_major_number, 0);

	if (!tegra_safety_dev_class)
		return;

	class_destroy(tegra_safety_dev_class);
	unregister_chrdev_region(num, ivc_chan_count);
	tegra_safety_dev_class = NULL;
}

int tegra_safety_class_init(struct device *dev)
{
	dev_t start;
	int ret;

	ret = alloc_chrdev_region(&start, 0, ivc_chan_count, "safety");
	if (ret) {
		dev_alert(dev, "safety: failed to allocate device numbers\n");
		goto error;
	}
	tegra_safety_dev_major_number = MAJOR(start);

	tegra_safety_dev_class = class_create(THIS_MODULE, "safety_class");
	if (IS_ERR(tegra_safety_dev_class)) {
		dev_alert(dev, "safety: failed to create class\n");
		ret = PTR_ERR(tegra_safety_dev_class);
		goto error;
	}

	return 0;

error:
	tegra_safety_class_exit(dev);

	return ret;
}

int tegra_safety_dev_init(struct device *dev, int index)
{
	struct tegra_safety_ivc *safety_ivc = dev_get_drvdata(dev);
	struct tegra_safety_dev_data *dev_data;
	struct device *char_dev;
	dev_t num;
	int ret;

	if (!tegra_safety_dev_class) {
		ret = tegra_safety_class_init(dev);
		if (ret) {
			dev_err(dev, "safety: class init failed\n");
			goto error;
		}
	}

	dev_data = devm_kzalloc(dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		dev_alert(dev, "safety: failed to allocate memory\n");
		ret = -ENOMEM;
		goto error;
	}

	cdev_init(&dev_data->cdev, &tegra_safety_dev_fops);
	dev_data->cdev.owner = THIS_MODULE;
	init_waitqueue_head(&dev_data->read_waitq);
	init_waitqueue_head(&dev_data->write_waitq);
	mutex_init(&dev_data->io_lock);

	dev_data->ivc_chan = safety_ivc->ivc_chan[index];
	safety_dev_data[index] = dev_data;
	num = MKDEV(tegra_safety_dev_major_number, index);

	ret = cdev_add(&dev_data->cdev, num, 1);
	if (ret) {
		dev_err(dev, "safety: unable to add character device\n");
		goto error;
	}

	char_dev = device_create(tegra_safety_dev_class, dev, num,
			NULL, "safety%d", index);
	if (IS_ERR(char_dev)) {
		dev_err(dev, "safety: could not create device\n");
		ret = PTR_ERR(char_dev);
		goto error;
	}

	dev_info(dev, "safety: character device %d registered\n", index);

	return ret;

error:
	tegra_safety_dev_exit(dev, index);
	tegra_safety_class_exit(dev);

	return ret;
}

void tegra_safety_dev_exit(struct device *dev, int index)
{
	struct tegra_safety_dev_data *dev_data = safety_dev_data[index];
	dev_t num = MKDEV(tegra_safety_dev_major_number, index);

	if (!dev_data)
		return;

	device_destroy(tegra_safety_dev_class, num);
	cdev_del(&dev_data->cdev);

	safety_dev_data[index] = NULL;

	if (index == (ivc_chan_count-1))
		tegra_safety_class_exit(dev);

	dev_info(dev, "safety: character device %d unregistered\n", index);
}

