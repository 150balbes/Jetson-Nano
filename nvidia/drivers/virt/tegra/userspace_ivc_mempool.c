/*
 * Userspace ivc mempool companion device driver
 *
 * This companion driver implements ivc mempool support services required
 * by the userspace ivc library that cannot be implemented inside
 * lower-privelged userspace code, e.g. mapping ivc mempool memory to
 * user-space.
 *
 * To provide complete ivc mempool functionality this driver must be paired
 * with a userspace daemon linked against ivc library code, i.e. it  is not
 * a stand-alone driver.
 *
 * Copyright (C) 2016-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-instance.h>
#include <uapi/linux/nvhvivc_mempool_ioctl.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
#include <asm/uaccess.h>
#else
#include <linux/uaccess.h>
#endif

#include "tegra_hv.h"

/* userspace ivc mempool device */
struct ivc_mempool_dev {
	int			minor;
	dev_t			dev;
	struct cdev		cdev;
	struct device		*device;
	char			name[32];
	/* config data for this mempool */
	const struct ivc_mempool *mempoolcfg;
	struct tegra_hv_ivm_cookie *mpool_cookie;
};

/* maximum ivc mempool id from all ivc mempools assigned to this guest */
static uint32_t max_mempool_id;

/* sysfs ivc mempool device class */
static struct class *ivc_mempool_class;

/* array of all ivc mempool devices managed by this driver */
static struct ivc_mempool_dev *ivc_mempool_dev_array;

/* first reserved ivc mempool device device */
static dev_t ivc_mempool_first_cdev;

/* mempool configuration data for all mempools assigned to this guest */
static const struct ivc_info_page *guest_ivc_info;

/* sysfs ivc mempool attributes */
/* mempool id */
static ssize_t id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_mempool_dev *mempooldev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			mempooldev->mempoolcfg->id);
}
/* mempool size */
static ssize_t size_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_mempool_dev *mempooldev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%llu\n",
			mempooldev->mempoolcfg->size);
}
/* peer guest id */
static ssize_t peer_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ivc_mempool_dev *mempooldev = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			mempooldev->mempoolcfg->peer_vmid);
}
static DEVICE_ATTR_RO(id);
static DEVICE_ATTR_RO(size);
static DEVICE_ATTR_RO(peer);
struct attribute *ivc_mempool_attrs[] = {
	&dev_attr_id.attr,
	&dev_attr_size.attr,
	&dev_attr_peer.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ivc_mempool);
/* end of sysfs properties */


static int ivc_mempool_open(struct inode *inode, struct file *filp)
{

	struct tegra_hv_ivm_cookie *mpool_cookie;
	struct ivc_mempool_dev *mempool_dev =
		container_of(inode->i_cdev, struct ivc_mempool_dev, cdev);

	/* Reserve the ivc mempool to prevent a userspace client from
	 * incorrectly opening a mempool assigned to a kernel driver and
	 * provide exclusive access to the pool
	 */
	mpool_cookie = tegra_hv_mempool_reserve(mempool_dev->minor);
	if (IS_ERR(mpool_cookie))
		return PTR_ERR(mpool_cookie);

	mempool_dev->mpool_cookie = mpool_cookie;

	filp->private_data = mempool_dev;
	return 0;
}

static int ivc_mempool_release(struct inode *inode, struct file *filp)
{
	int ret;
	struct ivc_mempool_dev *mempooldev = filp->private_data;

	if (WARN_ON(!((mempooldev != NULL) &&
			(mempooldev->mpool_cookie != NULL))))
		return -EFAULT;

	/* Unreserve the mempool */
	ret = tegra_hv_mempool_unreserve(mempooldev->mpool_cookie);
	if (ret < 0)
		pr_err("user_ivc_mempool: ### unable to release mempool\n");

	mempooldev->mpool_cookie = NULL;
	filp->private_data = NULL;

	return 0;
}

static ssize_t ivc_mempool_read(struct file *filp, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ivc_mempool_dev *mempooldev = filp->private_data;

	if (WARN_ON(!((mempooldev != NULL) &&
			(mempooldev->mpool_cookie != NULL))))
		return -EFAULT;

	/* ivc mempool read operation not exposed to client */
	dev_err(mempooldev->device, "read() syscall not supported\n");
	return -ENOTSUPP;
}

static ssize_t ivc_mempool_write(struct file *filp,
		const char __user *buf, size_t count, loff_t *pos)
{
	struct ivc_mempool_dev *mempooldev = filp->private_data;

	if (WARN_ON(!((mempooldev != NULL) &&
			(mempooldev->mpool_cookie != NULL))))
		return -EFAULT;

	/* ivc mempool write operation not exposed to client */
	dev_err(mempooldev->device, "write() syscall not supported\n");
	return -ENOTSUPP;
}

static int ivc_mempool_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ivc_mempool_dev *mempooldev = filp->private_data;
	uint64_t map_region_sz;
	unsigned long mpool_ipa_pfn;
	int ret = -EFAULT;

	if (WARN_ON(!((mempooldev != NULL) &&
			(mempooldev->mpool_cookie != NULL))))
		return -EFAULT;

	/* fail if userspace attempts to partially map the mempool */
	map_region_sz = vma->vm_end - vma->vm_start;

	if (((vma->vm_pgoff == 0) &&
		(map_region_sz == mempooldev->mempoolcfg->size))) {

		mpool_ipa_pfn =
			(mempooldev->mempoolcfg->pa >> PAGE_SHIFT);

		if (remap_pfn_range(vma, vma->vm_start,
					mpool_ipa_pfn,
					map_region_sz,
					vma->vm_page_prot)) {
			ret = -EAGAIN;
		} else {
			/* success! */
			ret = 0;
		}
	}

	return ret;
}

static long ivc_mempool_dev_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	struct ivc_mempool_dev *mempooldev = filp->private_data;
	long ret = 0;

	/* validate the cmd */
	if (_IOC_TYPE(cmd) != TEGRA_MPLUSERSPACE_IOCTL_MAGIC) {
		ret = -ENOTTY;
	} else if (_IOC_NR(cmd) > TEGRA_MPLUSERSPACE_IOCTL_NUMBER_MAX) {
		ret = -ENOTTY;
	} else {
		switch (cmd) {

		case TEGRA_MPLUSERSPACE_IOCTL_GET_INFO:
			if (copy_to_user((void __user *) arg,
					mempooldev->mempoolcfg,
					sizeof(*mempooldev->mempoolcfg))) {
				ret = -EFAULT;
			}
			break;
		default:
			/* The ioctl cmd number was validated against
			 * TEGRA_MPLUSERSPACE_IOCTL_NUMBER_MAX so execution
			 * should * never get here.
			 */
			ret = -ENOTTY;
		}
	}

	return ret;
}

static const struct file_operations ivc_mempool_fops = {
	.owner		= THIS_MODULE,
	.open		= ivc_mempool_open,
	.release	= ivc_mempool_release,
	.read		= ivc_mempool_read,
	.write		= ivc_mempool_write,
	.unlocked_ioctl	= ivc_mempool_dev_ioctl,
	.mmap		= ivc_mempool_mmap,
};


static int __init add_ivc_mempool_dev(struct ivc_mempool_dev *mempooldev,
		const struct ivc_mempool *mempoolcfg)
{
	int ret;

	/* TODO - validate mempool data */
	mempooldev->mempoolcfg = mempoolcfg;
	mempooldev->minor = mempooldev->mempoolcfg->id;
	mempooldev->dev = MKDEV(MAJOR(ivc_mempool_first_cdev),
			mempooldev->minor);

	/* register/add device */
	cdev_init(&mempooldev->cdev, &ivc_mempool_fops);
	ret = cdev_add(&mempooldev->cdev, mempooldev->dev, 1);
	if (ret != 0) {
		pr_err("user_ivc_mempool: ### device add failed\n");
		return ret;
	}
	snprintf(mempooldev->name, sizeof(mempooldev->name) - 1,
			"uivcmpool%d", mempooldev->mempoolcfg->id);
	mempooldev->device = device_create(ivc_mempool_class, NULL,
			mempooldev->dev, mempooldev, mempooldev->name);
	if (IS_ERR(mempooldev->device)) {
		pr_err("user_ivc_mempool: ### device create failed for %s\n",
			mempooldev->name);
		return PTR_ERR(mempooldev->device);
	}

	dev_set_drvdata(mempooldev->device, mempooldev);


	return 0;
}

static int __init setup_ivc_mempool(void)
{
	const struct ivc_mempool *ivc_info_mpool_array;
	const struct ivc_mempool *mempoolcfg;
	struct ivc_mempool_dev *mempooldev;
	uint32_t i;
	int result;

	/* extract the mempool data for this guest from the ivc info page */
	ivc_info_mpool_array = ivc_info_mempool_array(guest_ivc_info);

	/* device minor number = mempool id. Allocate a range of devices
	 * from (0 .. mempool id(max))
	 */
	max_mempool_id = 0;
	for (i = 0; i < guest_ivc_info->nr_mempools; i++) {
		mempoolcfg = &ivc_info_mpool_array[i];
		if (mempoolcfg->id > max_mempool_id)
			max_mempool_id = mempoolcfg->id;
	}

	result = alloc_chrdev_region(&ivc_mempool_first_cdev, 0, max_mempool_id,
			"ivc_mempool");
	if (result < 0) {
		pr_err("user_ivc_mempool: ### alloc_chrdev_region() failed\n");
		return result;
	}

	/* register ivc_user class with sysfs */
	ivc_mempool_class = class_create(THIS_MODULE, "tegra_uivc_mpool");
	if (IS_ERR(ivc_mempool_class)) {
		pr_err("user_ivc_mempool: ### failed create sysfs class: %ld\n",
				PTR_ERR(ivc_mempool_class));
		return PTR_ERR(ivc_mempool_class);
	}
	ivc_mempool_class->dev_groups = ivc_mempool_groups;

	/* allocate array to hold all ivc channels */
	ivc_mempool_dev_array = kcalloc(guest_ivc_info->nr_mempools,
			sizeof(struct ivc_mempool_dev), GFP_KERNEL);
	if (!ivc_mempool_dev_array) {
		pr_err("user_ivc_mempool: ### device array alloc failure\n");
		return -ENOMEM;
	}

	/* Add device for each ivc mempool */
	for (i = 0; i < guest_ivc_info->nr_mempools; i++) {
		mempooldev = &ivc_mempool_dev_array[i];
		mempoolcfg = &ivc_info_mpool_array[i];
		result = add_ivc_mempool_dev(mempooldev, mempoolcfg);
		if (result != 0)
			return result;
	}

	return 0;
}

static void __init cleanup_ivc_mempool(void)
{
	uint32_t i;

	if (ivc_mempool_dev_array) {
		for (i = 0; i < guest_ivc_info->nr_mempools; i++) {
			struct ivc_mempool_dev *ivcmempooldev =
				&ivc_mempool_dev_array[i];
			if (ivcmempooldev->device) {
				cdev_del(&ivcmempooldev->cdev);
				device_del(ivcmempooldev->device);
			}
		}
		kfree(ivc_mempool_dev_array);
		ivc_mempool_dev_array = NULL;
	}

	if (!IS_ERR_OR_NULL(ivc_mempool_class)) {
		class_destroy(ivc_mempool_class);
		ivc_mempool_class = NULL;
	}

	if (ivc_mempool_first_cdev) {
		unregister_chrdev_region(ivc_mempool_first_cdev,
				max_mempool_id);
		ivc_mempool_first_cdev = 0;
	}
}

static int __init userspace_ivc_mempool_init(void)
{
	int result;

	if (is_tegra_hypervisor_mode() == false) {
		pr_info("user_ivc_mempool: hypervisor not present\n");
		return -ENODEV;
	}

	/* get ivc configuration data for this  guest */
	guest_ivc_info = tegra_hv_get_ivc_info();
	if (IS_ERR(guest_ivc_info)) {
		pr_err("user_ivc_mempool: ### failed hyp get ivc info\n");
		return -ENODEV;
	}

	result = setup_ivc_mempool();
	if (result != 0)
		cleanup_ivc_mempool();

	return result;
}
module_init(userspace_ivc_mempool_init);
