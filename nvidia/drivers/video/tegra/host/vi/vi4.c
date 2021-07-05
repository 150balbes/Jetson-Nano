/*
 * VI driver for T186
 *
 * Copyright (c) 2015-2020 NVIDIA Corporation.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/tegra_pm_domains.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <media/capture_vi_channel.h>
#include <soc/tegra/camrtc-capture.h>

#include "dev.h"
#include "nvhost_acm.h"
#include "vi_notify.h"
#include <video/vi4.h>
#include "t186/t186.h"
#include <uapi/linux/nvhost_vi_ioctl.h>
#include <media/mc_common.h>
#include <media/tegra_camera_platform.h>
#include "camera/vi/vi4_fops.h"

#define VI_CFG_INTERRUPT_STATUS_0		0x0044
#define VI_CFG_INTERRUPT_MASK_0			0x0048
#define VI_ISPBUFA_ERROR_0			0x1000
#define VI_FMLITE_ERROR_0			0x313C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_0		0x6008
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0	0x600C
#define VI_NOTIFY_TAG_CLASSIFY_SAFETY_TEST_0	0x6010
#define VI_NOTIFY_ERROR_0			0x6020

#define VI_HOST_PKTINJECT_STALL_ERR_MASK	0x00000080
#define VI_CSIMUX_FIFO_OVFL_ERR_MASK		0x00000040
#define VI_ATOMP_PACKER_OVFL_ERR_MASK		0x00000020
#define VI_FMLITE_BUF_OVFL_ERR_MASK		0x00000010
#define VI_NOTIFY_FIFO_OVFL_ERR_MASK		0x00000008
#define VI_ISPBUFA_ERR_MASK			0x00000001

/* HW capability, pixels per clock */
#define NUM_PPC					8
#define VI_OVERHEAD				10

/* Interrupt handler */
/* NOTE: VI4 has three interrupt lines. This handler is for the master/error
 * line. The other lines are dedicated to VI NOTIFY and handled elsewhere.
 */
static irqreturn_t nvhost_vi4_error_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);
	u32 r;

	r = host1x_readl(pdev, VI_NOTIFY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_ERROR_0, 1);
		dev_err_ratelimited(&pdev->dev, "notify buffer overflow\n");
		atomic_inc(&vi->notify_overflow);

		/* FIXME: does not work with RTCPU-based VI Notify */
		if (vi->hvnd != NULL)
			nvhost_vi_notify_error(pdev);
	}

	r = host1x_readl(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_NOTIFY_TAG_CLASSIFY_SAFETY_ERROR_0, r);
		dev_err_ratelimited(&pdev->dev, "safety error mask 0x%08X\n", r);
	}

	r = host1x_readl(pdev, VI_FMLITE_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_FMLITE_ERROR_0, 1);
		dev_err_ratelimited(&pdev->dev, "FM-Lite buffer overflow\n");
		atomic_inc(&vi->fmlite_overflow);
	}

	r = host1x_readl(pdev, VI_ISPBUFA_ERROR_0);
	if (r) {
		host1x_writel(pdev, VI_ISPBUFA_ERROR_0, 1);
		dev_err_ratelimited(&pdev->dev, "ISPBUF buffer overflow\n");
	}

	r = host1x_readl(pdev, VI_CFG_INTERRUPT_STATUS_0);
	if (r) {
		host1x_writel(pdev, VI_CFG_INTERRUPT_STATUS_0, 1);
		dev_err_ratelimited(&pdev->dev, "master error\n");
		atomic_inc(&vi->overflow);
	}

	return IRQ_HANDLED;
}

/* NV host device */
int nvhost_vi4_prepare_poweroff(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0, 0x00000000);
	if (vi->error_irq >= 0)
		disable_irq(vi->error_irq);
	return 0;
}

int nvhost_vi4_finalize_poweron(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	host1x_writel(pdev, VI_CFG_INTERRUPT_MASK_0,
			VI_HOST_PKTINJECT_STALL_ERR_MASK |
			VI_CSIMUX_FIFO_OVFL_ERR_MASK |
			VI_ATOMP_PACKER_OVFL_ERR_MASK |
			VI_FMLITE_BUF_OVFL_ERR_MASK |
			VI_NOTIFY_FIFO_OVFL_ERR_MASK |
			VI_ISPBUFA_ERR_MASK);
	if (vi->error_irq >= 0)
		enable_irq(vi->error_irq);
	return 0;
}

int nvhost_vi4_aggregate_constraints(struct platform_device *dev,
				int clk_index,
				unsigned long floor_rate,
				unsigned long pixelrate,
				unsigned long bw_constraint)
{
	struct nvhost_device_data *pdata = nvhost_get_devdata(dev);

	if (!pdata) {
		dev_err(&dev->dev,
			"No platform data, fall back to default policy\n");
		return 0;
	}
	if (!pixelrate || clk_index != 0)
		return 0;
	/* SCF send request using NVHOST_CLK, which is calculated
	 * in floor_rate, so we need to aggregate its request
	 * with V4L2 pixelrate request
	 */
	return floor_rate + (pixelrate / pdata->num_ppc);
}

void nvhost_vi4_idle(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	vi->busy = false;
}

void nvhost_vi4_busy(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	vi->busy = true;
}

void nvhost_vi4_reset(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	if (vi->busy)
		return;
	if (!IS_ERR(vi->vi_reset))
		reset_control_reset(vi->vi_reset);
	if (!IS_ERR(vi->vi_tsc_reset))
		reset_control_reset(vi->vi_tsc_reset);
}

static struct tegra_vi_data t18_vi_data = {
	.info = (struct nvhost_device_data *)&t18_vi_info,
	.vi_fops = &vi4_fops,
};

/* Platform device */
static struct of_device_id tegra_vi4_of_match[] = {
	{
		.compatible = "nvidia,tegra186-vi",
		.data = &t18_vi_data
	},
	{ },
};

static bool tegra_camera_rtcpu_available(void)
{
	struct device_node *dn;

	dn = of_find_node_by_path("tegra-camera-rtcpu");
	/* Note: false if dn is NULL */
	return of_device_is_available(dn);
}

static int vi4_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id)
{
	uint32_t id;

	if (syncpt_id == NULL) {
		dev_err(&pdev->dev, "%s: null argument\n", __func__);
		return -EINVAL;
	}

	id = nvhost_get_syncpt_client_managed(pdev, name);
	if (id == 0)
		return -ENODEV;

	*syncpt_id = id;

	return 0;
}

static void vi4_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	nvhost_syncpt_put_ref_ext(pdev, id);
}

static void vi4_get_gos_table(struct platform_device *pdev, int *count,
			const dma_addr_t **table)
{
	*table = NULL;
	*count = 0;
}

static int vi4_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset)
{
	if (syncpt_addr == NULL || gos_index == NULL || gos_offset == NULL) {
		dev_err(&pdev->dev, "%s: null arguments\n", __func__);
		return -EINVAL;
	}

	*syncpt_addr = 0;
	*gos_index = GOS_INDEX_INVALID;
	*gos_offset = 0;

	return 0;
}

static struct vi_channel_drv_ops vi4_channel_drv_ops = {
	.alloc_syncpt = vi4_alloc_syncpt,
	.release_syncpt = vi4_release_syncpt,
	.get_gos_table = vi4_get_gos_table,
	.get_syncpt_gos_backing = vi4_get_syncpt_gos_backing,
};

static int tegra_vi4_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvhost_device_data *pdata;
	struct nvhost_vi_dev *vi;
	struct tegra_vi_data *data = NULL;
	int err;
	struct tegra_camera_dev_info vi_info;
	unsigned int num_channels = 0;

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(39));
	memset(&vi_info, 0, sizeof(vi_info));

	match = of_match_device(tegra_vi4_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "No match found on probe...\n");
		return -EINVAL;
	}

	data = (struct tegra_vi_data *) match->data;

	pdata = (struct nvhost_device_data *)data->info;
	if (!pdata) {
		dev_err(&pdev->dev, "No device data!\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "nvidia,num-vi-channels",
				&num_channels)) {
		dev_warn(&pdev->dev,
			"using default number of vi channels,%d\n",
			pdata->num_channels);
	} else {
		if (num_channels < pdata->num_channels) {
			pdata->num_channels = num_channels;
		} else {
			dev_warn(&pdev->dev,
				"num vi-channels are out of range, use default num\n");
		}
	}
	dev_dbg(&pdev->dev, "num vi channels : %d\n", pdata->num_channels);

	pdata->pdev = pdev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(pdev, pdata);

	vi = devm_kzalloc(&pdev->dev, sizeof(*vi), GFP_KERNEL);
	if (unlikely(vi == NULL))
		return -ENOMEM;

	vi->vi_reset = devm_reset_control_get(&pdev->dev, "vi");
	vi->vi_tsc_reset = devm_reset_control_get(&pdev->dev, "tsctnvi");
	vi->debug_dir = debugfs_create_dir("tegra_vi", NULL);

	if (vi->debug_dir != NULL) {
		debugfs_create_atomic_t("overflow", S_IRUGO, vi->debug_dir,
					&vi->overflow);
		debugfs_create_atomic_t("notify-overflow", S_IRUGO,
					vi->debug_dir, &vi->notify_overflow);
		debugfs_create_atomic_t("fmlite-overflow", S_IRUGO,
					vi->debug_dir, &vi->fmlite_overflow);
	}

	nvhost_set_private_data(pdev, vi);
	err = nvhost_client_device_get_resources(pdev);
	if (err)
		return err;

	nvhost_module_init(pdev);
	err = nvhost_client_device_init(pdev);
	if (err) {
		nvhost_module_deinit(pdev);
		return err;
	}

	vi->error_irq = platform_get_irq(pdev, 0);
	if (vi->error_irq >= 0) {
		err = devm_request_threaded_irq(&pdev->dev, vi->error_irq,
						NULL, nvhost_vi4_error_isr,
						IRQF_ONESHOT,
						dev_name(&pdev->dev), pdev);
		if (err) {
			dev_err(&pdev->dev, "cannot get master IRQ %d: %d\n",
				vi->error_irq, err);
			vi->error_irq = -ENXIO;
		} else
			disable_irq(vi->error_irq);
	} else
		dev_warn(&pdev->dev, "missing master IRQ\n");

	if (!tegra_camera_rtcpu_available()) {
		err = vi_notify_register(&nvhost_vi_notify_driver, &pdev->dev,
						12);
		if (err) {
			nvhost_client_device_release(pdev);
			return err;
		}
	}

	vi->mc_vi.ndev = pdev;
	vi->mc_vi.fops = data->vi_fops;
	err = tegra_vi_media_controller_init(&vi->mc_vi, pdev);
	if (err) {
		if (vi->hvnd != NULL)
			vi_notify_unregister(&nvhost_vi_notify_driver,
						&pdev->dev);
		nvhost_client_device_release(pdev);
		return err;
	}

	err = vi_channel_drv_register(pdev, &vi4_channel_drv_ops);
	if (err)
		return err;

	vi_info.pdev = pdev;
	vi_info.hw_type = HWTYPE_VI;
	vi_info.ppc = NUM_PPC;
	vi_info.overhead = VI_OVERHEAD;
	/* 4 uS latency allowed for memory freq switch */
	vi_info.memory_latency = 4;
	err = tegra_camera_device_register(&vi_info, vi);

	return err;
}

static int tegra_vi4_remove(struct platform_device *pdev)
{
	struct nvhost_vi_dev *vi = nvhost_get_private_data(pdev);

	tegra_camera_device_unregister(vi);
	vi_channel_drv_unregister(&pdev->dev);
	tegra_vi_media_controller_cleanup(&vi->mc_vi);
	if (vi->hvnd != NULL)
		vi_notify_unregister(&nvhost_vi_notify_driver, &pdev->dev);
	nvhost_client_device_release(pdev);
	/* ^ includes call to nvhost_module_deinit() */
#ifdef CONFIG_PM_GENERIC_DOMAINS
	tegra_pd_remove_device(&pdev->dev);
#endif
	debugfs_remove_recursive(vi->debug_dir);
	return 0;
}

static struct platform_driver tegra_vi4_driver = {
	.probe = tegra_vi4_probe,
	.remove = tegra_vi4_remove,
	.driver = {
		.name = "tegra-vi4",
		.owner = THIS_MODULE,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#ifdef CONFIG_OF
		.of_match_table = tegra_vi4_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
		.suppress_bind_attrs = true,
	},
};

static struct of_device_id tegra_vi4_domain_match[] = {
	{ .compatible = "nvidia,tegra186-ve-pd",
		.data = (struct nvhost_device_data *)&t18_vi_info },
	{ },
};

static int __init tegra_vi4_init(void)
{
	int err;

	err = nvhost_domain_init(tegra_vi4_domain_match);
	if (err)
		return err;

	return platform_driver_register(&tegra_vi4_driver);
}

static void __exit tegra_vi4_exit(void)
{
	platform_driver_unregister(&tegra_vi4_driver);
}

late_initcall(tegra_vi4_init);
module_exit(tegra_vi4_exit);
