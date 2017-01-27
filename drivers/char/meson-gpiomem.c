/*
 * linux/drivers/char/meson-gpiomem.c
 *
 * GPIO memory device driver
 *
 * Creates a chardev /dev/gpiomem which will provide user access to
 * the Meson GPIO registers when it is mmap()'d.
 * No longer need root for user GPIO access, but without relaxing permissions
 * on /dev/mem.
 *
 * Copyright (c) 2017 Hardkernel Co., Ltd.
 *
 * This driver is based on bcm2835-gpiomem.c in Raspberrypi's linux kernel 4.4:
 *	Written by Luke Wren <luke@raspberrypi.org>
 *	Copyright (c) 2015, Raspberry Pi (Trading) Ltd.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2, as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/pagemap.h>

#define DEVICE_NAME "meson-gpiomem"
#define DRIVER_NAME "gpiomem-meson"
#define DEVICE_MINOR 0

struct meson_gpiomem_instance {
	unsigned long gpio_regs_phys;
	struct device *dev;
};

static struct cdev meson_gpiomem_cdev;
static dev_t meson_gpiomem_devid;
static struct class *meson_gpiomem_class;
static struct device *meson_gpiomem_dev;
static struct meson_gpiomem_instance *inst;

static int meson_gpiomem_open(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;

	dev_info(inst->dev, "gpiomem device opened.");

	if (dev != DEVICE_MINOR) {
		dev_err(inst->dev, "Unknown minor device: %d", dev);
		ret = -ENXIO;
	}
	return ret;
}

static int meson_gpiomem_release(struct inode *inode, struct file *file)
{
	int dev = iminor(inode);
	int ret = 0;

	if (dev != DEVICE_MINOR) {
		dev_err(inst->dev, "Unknown minor device %d", dev);
		ret = -ENXIO;
	}
	return ret;
}

static const struct vm_operations_struct meson_gpiomem_vm_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};

static int meson_gpiomem_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long gpio_page = inst->gpio_regs_phys >> PAGE_SHIFT;

	vma->vm_page_prot = phys_mem_access_prot(file, gpio_page,
						 PAGE_SIZE,
						 vma->vm_page_prot);

	vma->vm_ops = &meson_gpiomem_vm_ops;
	if (remap_pfn_range(vma, vma->vm_start,
				gpio_page,
				PAGE_SIZE,
				vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations
meson_gpiomem_fops = {
	.owner = THIS_MODULE,
	.open = meson_gpiomem_open,
	.release = meson_gpiomem_release,
	.mmap = meson_gpiomem_mmap,
};

static int meson_gpiomem_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *dev = &pdev->dev;
	struct resource *res = NULL;

	/* Allocate buffers and instance data */
	inst = kzalloc(sizeof(struct meson_gpiomem_instance), GFP_KERNEL);
	if (!inst) {
		err = -ENOMEM;
		goto failed_inst_alloc;
	}

	inst->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		inst->gpio_regs_phys = res->start;
	} else {
		dev_err(inst->dev, "failed to get IO resource");
		err = -ENOENT;
		goto failed_get_resource;
	}

	/* Create character device entries */
	err = alloc_chrdev_region(&meson_gpiomem_devid,
				  DEVICE_MINOR, 1, DEVICE_NAME);
	if (err != 0) {
		dev_err(inst->dev, "unable to allocate device number");
		goto failed_alloc_chrdev;
	}
	cdev_init(&meson_gpiomem_cdev, &meson_gpiomem_fops);
	meson_gpiomem_cdev.owner = THIS_MODULE;
	err = cdev_add(&meson_gpiomem_cdev, meson_gpiomem_devid, 1);
	if (err != 0) {
		dev_err(inst->dev, "unable to register device");
		goto failed_cdev_add;
	}

	/* Create sysfs entries */
	meson_gpiomem_class = class_create(THIS_MODULE, DEVICE_NAME);
	err = IS_ERR(meson_gpiomem_class);
	if (err)
		goto failed_class_create;

	meson_gpiomem_dev = device_create(meson_gpiomem_class, NULL,
					meson_gpiomem_devid, NULL,
					"gpiomem");
	err = IS_ERR(meson_gpiomem_dev);
	if (err)
		goto failed_device_create;

	dev_info(inst->dev, "Initialised: Registers at 0x%08lx",
		 inst->gpio_regs_phys);

	return 0;

failed_device_create:
	class_destroy(meson_gpiomem_class);
failed_class_create:
	cdev_del(&meson_gpiomem_cdev);
failed_cdev_add:
	unregister_chrdev_region(meson_gpiomem_devid, 1);
failed_alloc_chrdev:
failed_get_resource:
	kfree(inst);
failed_inst_alloc:
	dev_err(inst->dev, "could not load meson_gpiomem");
	return err;
}

static int meson_gpiomem_remove(struct platform_device *pdev)
{
	struct device *dev = inst->dev;

	kfree(inst);
	device_destroy(meson_gpiomem_class, meson_gpiomem_devid);
	class_destroy(meson_gpiomem_class);
	cdev_del(&meson_gpiomem_cdev);
	unregister_chrdev_region(meson_gpiomem_devid, 1);

	dev_info(dev, "GPIO mem driver removed - OK");
	return 0;
}

static const struct of_device_id meson_gpiomem_of_match[] = {
	{.compatible = "amlogic,meson-gpiomem",},
	{ },
};
MODULE_DEVICE_TABLE(of, meson_gpiomem_of_match);

static struct platform_driver meson_gpiomem_driver = {
	.driver			= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= meson_gpiomem_of_match,
	},
	.probe			= meson_gpiomem_probe,
	.remove			= meson_gpiomem_remove,
};

module_platform_driver(meson_gpiomem_driver);

MODULE_ALIAS("platform:gpiomem-meson");
MODULE_DESCRIPTION("MESON gpiomem driver for accessing GPIO from userspace");
MODULE_AUTHOR("Brian Kim <brian.kim@hardkernel.com>");
MODULE_LICENSE("GPL");
