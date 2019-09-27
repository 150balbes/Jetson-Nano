/*
 * Tegra Graphics ISP
 *
 * Copyright (c) 2012-2019, NVIDIA Corporation.  All rights reserved.
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

#include <linux/export.h>
#include <linux/resource.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/tegra_pm_domains.h>
#include <linux/version.h>
#include <soc/tegra/fuse.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t124/t124.h"
#include "t210/t210.h"

#ifdef CONFIG_ARCH_TEGRA_18x_SOC
#include "t186/t186.h"
#endif

#include <linux/uaccess.h>
#include <linux/fs.h>
#include <uapi/linux/nvhost_isp_ioctl.h>
#include <linux/platform/tegra/latency_allowance.h>
#include "isp.h"
#include <media/tegra_camera_platform.h>

#define T12_ISP_CG_CTRL		0x74
#define T12_CG_2ND_LEVEL_EN	1

#define	ISP_MAX_BPP		2

#define ISPA_DEV_ID		0
#define ISPB_DEV_ID		1
#define ISP_PPC_T186		2
#define ISP_OVERHEAD_T210	10
/* 15% sw + 2% hw */
#define ISP_OVERHEAD_T186	17

static struct of_device_id tegra_isp_of_match[] = {
#ifdef TEGRA_12X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra124-isp",
		.data = (struct nvhost_device_data *)&t124_isp_info },
#endif
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{ .compatible = "nvidia,tegra210-isp",
		.data = (struct nvhost_device_data *)&t21_isp_info },
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{ .compatible = "nvidia,tegra186-isp",
		.data = (struct nvhost_device_data *)&t18_isp_info },
#endif
	{ },
};

static void (*mfi_callback)(void *);
static void *mfi_callback_arg;
static DEFINE_MUTEX(isp_isr_lock);

static int __init init_tegra_isp_isr_callback(void)
{
	mutex_init(&isp_isr_lock);
	return 0;
}

pure_initcall(init_tegra_isp_isr_callback);

int nvhost_isp_t124_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct isp *tegra_isp = pdata->private_data;

	disable_irq(tegra_isp->irq);

	return 0;
}

int nvhost_isp_t124_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct isp *tegra_isp = pdata->private_data;

	host1x_writel(pdev, T12_ISP_CG_CTRL, T12_CG_2ND_LEVEL_EN);
	enable_irq(tegra_isp->irq);

	return 0;
}

int nvhost_isp_t210_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct isp *tegra_isp = pdata->private_data;

	enable_irq(tegra_isp->irq);

	return 0;
}

static inline u32 __maybe_unused
tegra_isp_read(struct isp *tegra_isp, u32 offset)
{
	return readl(tegra_isp->base + offset);
}

static inline void __maybe_unused
tegra_isp_write(struct isp *tegra_isp, u32 offset, u32 data)
{
	writel(data, tegra_isp->base + offset);
}

int tegra_isp_register_mfi_cb(isp_callback cb, void *cb_arg)
{
	if (mfi_callback || mfi_callback_arg) {
		pr_err("cb already registered\n");
		return -1;
	}

	mutex_lock(&isp_isr_lock);
	mfi_callback = cb;
	mfi_callback_arg = cb_arg;
	mutex_unlock(&isp_isr_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_isp_register_mfi_cb);

int tegra_isp_unregister_mfi_cb(void)
{
	mutex_lock(&isp_isr_lock);
	mfi_callback = NULL;
	mfi_callback_arg = NULL;
	mutex_unlock(&isp_isr_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_isp_unregister_mfi_cb);

static void isp_isr_work(struct work_struct *isp_work)
{
	if (mfi_callback == NULL) {
		pr_debug("NULL callback\n");
		return;
	}

	mutex_lock(&isp_isr_lock);
	mfi_callback(mfi_callback_arg);
	mutex_unlock(&isp_isr_lock);
	return;
}

void nvhost_isp_queue_isr_work(struct isp *tegra_isp)
{
	queue_work(tegra_isp->isp_workqueue, &tegra_isp->my_isr_work->work);
}

static int isp_probe(struct platform_device *dev)
{
	int err = 0;
	int dev_id = 0;

	struct isp *tegra_isp;
	struct nvhost_device_data *pdata = NULL;
	struct tegra_camera_dev_info isp_info;

	memset(&isp_info, 0, sizeof(isp_info));

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		if (nvhost_is_186()) {
			/* T186 only has one ISP */
			match = of_match_device(tegra_isp_of_match, &dev->dev);
			if (match)
				pdata = (struct nvhost_device_data *)
					match->data;
			/* 2% hw + 15% sw overhead */
			isp_info.overhead = ISP_OVERHEAD_T186;
			isp_info.ppc = ISP_PPC_T186;
		} else {
			/*
			* For older kernels, we use "isp.0" for ispa
			* and "isp.1" for ispb
			*
			* For newer kernels, we use "54600000.isp" for ispa
			* and "54680000.isp" for ispb
			*
			*/
			if (strcmp(dev->name, "isp.0") == 0 ||
				strcmp(dev->name, "54600000.isp") == 0)
				dev_id = ISPA_DEV_ID;
			else if (strcmp(dev->name, "isp.1") == 0 ||
				strcmp(dev->name, "54680000.isp") == 0)
				dev_id = ISPB_DEV_ID;
			else
				return -EINVAL;

			if (nvhost_is_124()) {
				if (dev_id == ISPB_DEV_ID)
					pdata = &t124_ispb_info;
				if (dev_id == ISPA_DEV_ID)
					pdata = &t124_isp_info;
			} else if (nvhost_is_210()) {
				if (dev_id == ISPB_DEV_ID)
					pdata = &t21_ispb_info;
				if (dev_id == ISPA_DEV_ID)
					pdata = &t21_isp_info;
				/* 10% overhead */
				isp_info.overhead = ISP_OVERHEAD_T210;
				isp_info.use_max = true;
			}
		}
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	tegra_isp = devm_kzalloc(&dev->dev, sizeof(struct isp), GFP_KERNEL);
	if (!tegra_isp) {
		dev_err(&dev->dev, "can't allocate memory for isp\n");
		return -ENOMEM;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto camera_isp_unregister;

	tegra_isp->dev_id = dev_id;
	tegra_isp->ndev = dev;

	pdata->private_data = tegra_isp;

	/* init ispa isr */
	tegra_isp->base = pdata->aperture[0];
	if (!tegra_isp->base) {
		pr_err("%s: can't ioremap gnt_base\n", __func__);
		err = -ENOMEM;
	}

	/* creating workqueue */
	if (dev_id == 0)
		tegra_isp->isp_workqueue = alloc_workqueue("ispa_workqueue",
						 WQ_HIGHPRI | WQ_UNBOUND, 1);
	else
		tegra_isp->isp_workqueue = alloc_workqueue("ispb_workqueue",
						 WQ_HIGHPRI | WQ_UNBOUND, 1);

	if (!tegra_isp->isp_workqueue) {
		pr_err("failed to allocate isp_workqueue\n");
		goto camera_isp_unregister;
	}

	tegra_isp->my_isr_work =
		kmalloc(sizeof(struct tegra_isp_mfi), GFP_KERNEL);

	if (!tegra_isp->my_isr_work) {
		err = -ENOMEM;
		goto camera_isp_unregister;
	}

	INIT_WORK((struct work_struct *)tegra_isp->my_isr_work, isp_isr_work);

	nvhost_module_init(dev);

	err = nvhost_client_device_init(dev);
	if (err)
		goto free_isr;

	isp_info.hw_type = HWTYPE_ISPA;
	if (dev_id == ISPB_DEV_ID)
		isp_info.hw_type = HWTYPE_ISPB;
	isp_info.pdev = dev;
	err = tegra_camera_device_register(&isp_info, tegra_isp);
	if (err)
		goto free_isr;

	return 0;
free_isr:
	kfree(tegra_isp->my_isr_work);
camera_isp_unregister:
	dev_err(&dev->dev, "%s: failed\n", __func__);

	return err;
}

static int __exit isp_remove(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct isp *tegra_isp = (struct isp *)pdata->private_data;

	tegra_camera_device_unregister(tegra_isp);

	nvhost_client_device_release(dev);
	disable_irq(tegra_isp->irq);
	kfree(tegra_isp->my_isr_work);
	flush_workqueue(tegra_isp->isp_workqueue);
	destroy_workqueue(tegra_isp->isp_workqueue);
	tegra_isp = NULL;
	return 0;
}

static struct platform_driver isp_driver = {
	.probe = isp_probe,
	.remove = __exit_p(isp_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "isp",
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = tegra_isp_of_match,
#endif
	}
};

#ifdef CONFIG_TEGRA_MC
static int isp_set_la(struct isp *tegra_isp, u32 isp_bw, u32 la_client)
{
	int ret = 0;
	int la_id;

	if (tegra_isp->dev_id == ISPB_DEV_ID)
		la_id = TEGRA_LA_ISP_WAB;
	else
		la_id = TEGRA_LA_ISP_WA;

	ret = tegra_set_camera_ptsa(la_id, isp_bw, la_client);
	if (!ret) {
		if (nvhost_is_124() || nvhost_is_210()) {
			/* T186 ISP/VI LA programming is changed.
			* Check tegra18x_la.c
			*/
			ret = tegra_set_latency_allowance(la_id, isp_bw);
			if (ret)
				pr_err("%s: set latency failed for ISP %d: %d\n",
					__func__, tegra_isp->dev_id, ret);
		}
	} else {
		pr_err("%s: set ptsa failed for ISP %d: %d\n", __func__,
			tegra_isp->dev_id, ret);
	}

	return ret;
}
#else
static int isp_set_la(struct isp *tegra_isp, u32 isp_bw, u32 la_client)
{
	return 0;
}
#endif

static long isp_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct isp *tegra_isp = file->private_data;

	if (_IOC_TYPE(cmd) != NVHOST_ISP_IOCTL_MAGIC)
		return -EFAULT;

	switch (_IOC_NR(cmd)) {
	case _IOC_NR(NVHOST_ISP_IOCTL_SET_ISP_LA_BW): {
		u32 ret = 0;
		struct isp_la_bw isp_info;

		if (copy_from_user(&isp_info,
			(const void __user *)arg,
				sizeof(struct isp_la_bw))) {
			dev_err(&tegra_isp->ndev->dev,
				"%s: Failed to copy arg from user\n", __func__);
			return -EFAULT;
			}

		/* Set latency allowance for ISP, BW is in MBps */
		ret = isp_set_la(tegra_isp,
			isp_info.isp_la_bw,
			(isp_info.is_iso) ?
				ISP_HARD_ISO_CLIENT : ISP_SOFT_ISO_CLIENT);
		if (ret) {
			dev_err(&tegra_isp->ndev->dev,
			"%s: failed to set la isp_bw %u MBps\n",
			__func__, isp_info.isp_la_bw);
			return -ENOMEM;
		}

		return 0;

	}
	default:
		dev_err(&tegra_isp->ndev->dev,
			"%s: Unknown ISP ioctl.\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int isp_open(struct inode *inode, struct file *file)
{
	struct nvhost_device_data *pdata;
	struct isp *tegra_isp;

	pdata = container_of(inode->i_cdev,
		struct nvhost_device_data, ctrl_cdev);
	if (WARN_ONCE(pdata == NULL, "pdata not found, %s failed\n", __func__))
		return -ENODEV;

	tegra_isp = pdata->private_data;
	if (WARN_ONCE(tegra_isp == NULL,
		"tegra_isp not found, %s failed\n", __func__))
		return -ENODEV;

	file->private_data = tegra_isp;

	return 0;
}

static int isp_release(struct inode *inode, struct file *file)
{
	return 0;
}

const struct file_operations tegra_isp_ctrl_ops = {
	.owner = THIS_MODULE,
	.open = isp_open,
	.unlocked_ioctl = isp_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = isp_ioctl,
#endif
	.release = isp_release,
};

static struct of_device_id tegra_isp_domain_match[] = {
	{.compatible = "nvidia,tegra124-ve-pd",
	 .data = (struct nvhost_device_data *)&t124_isp_info},
	{.compatible = "nvidia,tegra132-ve-pd",
	 .data = (struct nvhost_device_data *)&t124_isp_info},
#ifdef TEGRA_21X_OR_HIGHER_CONFIG
	{.compatible = "nvidia,tegra210-ve-pd",
	 .data = (struct nvhost_device_data *)&t21_isp_info},
	{.compatible = "nvidia,tegra210-ve2-pd",
	 .data = (struct nvhost_device_data *)&t21_ispb_info},
#endif
#ifdef CONFIG_ARCH_TEGRA_18x_SOC
	{.compatible = "nvidia,tegra186-ispa-pd",
	 .data = (struct nvhost_device_data *)&t18_isp_info},
#endif
	{},
};

static int __init isp_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_isp_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&isp_driver);
}

static void __exit isp_exit(void)
{
	platform_driver_unregister(&isp_driver);
}

late_initcall(isp_init);
module_exit(isp_exit);
