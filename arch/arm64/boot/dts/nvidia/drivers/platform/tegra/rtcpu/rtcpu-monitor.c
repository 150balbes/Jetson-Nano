/*
 * Copyright (c) 2016-2017 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-rtcpu-monitor.h>

#include "drivers/video/tegra/host/vi/vi_notify.h"
#include "vi-notify.h"

struct tegra_camrtc_mon {
	struct device *rce_dev;
	int wdt_irq;
	struct work_struct wdt_work;
};

int tegra_camrtc_mon_restore_rtcpu(struct tegra_camrtc_mon *cam_rtcpu_mon)
{
	/* (Re)boot the rtcpu */
	/* rtcpu-down and rtcpu-up events are broadcast to all ivc channels */
	return tegra_camrtc_reboot(cam_rtcpu_mon->rce_dev);
}
EXPORT_SYMBOL(tegra_camrtc_mon_restore_rtcpu);

static void tegra_camrtc_mon_wdt_worker(struct work_struct *work)
{
	struct tegra_camrtc_mon *cam_rtcpu_mon = container_of(work,
					struct tegra_camrtc_mon, wdt_work);

	dev_info(cam_rtcpu_mon->rce_dev,
		"Alert: Camera RTCPU gone bad! restoring it immediately!!\n");

	tegra_camrtc_mon_restore_rtcpu(cam_rtcpu_mon);

	/* Enable WDT IRQ */
	enable_irq(cam_rtcpu_mon->wdt_irq);
}

static irqreturn_t tegra_camrtc_mon_wdt_remote_isr(int irq, void *data)
{
	struct tegra_camrtc_mon *cam_rtcpu_mon = data;

	disable_irq_nosync(irq);

	schedule_work(&cam_rtcpu_mon->wdt_work);

	return IRQ_HANDLED;
}

static int tegra_camrtc_mon_wdt_irq_setup(
		struct tegra_camrtc_mon *cam_rtcpu_mon)
{
	struct platform_device *pdev =
			to_platform_device(cam_rtcpu_mon->rce_dev);
	int ret;

	cam_rtcpu_mon->wdt_irq = platform_get_irq_byname(pdev, "wdt-remote");
	if (cam_rtcpu_mon->wdt_irq < 0) {
		dev_warn(&pdev->dev, "missing irq wdt-remote\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(&pdev->dev, cam_rtcpu_mon->wdt_irq,
			NULL, tegra_camrtc_mon_wdt_remote_isr, IRQF_ONESHOT,
			dev_name(cam_rtcpu_mon->rce_dev), cam_rtcpu_mon);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "using cam RTCPU IRQ (%d)\n",
			cam_rtcpu_mon->wdt_irq);

	return 0;
}

struct tegra_camrtc_mon *tegra_camrtc_mon_create(struct device *dev)
{
	struct tegra_camrtc_mon *cam_rtcpu_mon;

	cam_rtcpu_mon = devm_kzalloc(dev, sizeof(*cam_rtcpu_mon), GFP_KERNEL);
	if (unlikely(cam_rtcpu_mon == NULL))
		return ERR_PTR(-ENOMEM);

	cam_rtcpu_mon->rce_dev = dev;

	/* Initialize wdt_work */
	INIT_WORK(&cam_rtcpu_mon->wdt_work, tegra_camrtc_mon_wdt_worker);

	tegra_camrtc_mon_wdt_irq_setup(cam_rtcpu_mon);

	dev_info(dev, "tegra_camrtc_mon_create is successful\n");

	return cam_rtcpu_mon;
}
EXPORT_SYMBOL(tegra_camrtc_mon_create);

int tegra_cam_rtcpu_mon_destroy(struct tegra_camrtc_mon *cam_rtcpu_mon)
{
	if (IS_ERR_OR_NULL(cam_rtcpu_mon))
		return -EINVAL;

	devm_kfree(cam_rtcpu_mon->rce_dev, cam_rtcpu_mon);

	return 0;
}
EXPORT_SYMBOL(tegra_cam_rtcpu_mon_destroy);

MODULE_DESCRIPTION("CAMERA RTCPU monitor driver");
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_LICENSE("GPL v2");
