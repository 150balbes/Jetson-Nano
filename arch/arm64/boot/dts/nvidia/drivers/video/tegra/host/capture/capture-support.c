/*
 * Device driver for owning the separate Stream-ID used for GoS
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#include "capture/capture-support.h"

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <soc/tegra/camrtc-capture.h>
#include <soc/tegra/chip-id.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "nvhost_syncpt_unit_interface.h"
#include "t194/t194.h"

int t194_capture_alloc_syncpt(struct platform_device *pdev,
			const char *name,
			uint32_t *syncpt_id)
{
	uint32_t id;

	if (syncpt_id == NULL) {
		dev_err(&pdev->dev, "%s: null argument\n", __func__);
		return -EINVAL;
	}

	id = nvhost_get_syncpt_client_managed(pdev, name);
	if (id == 0) {
		dev_err(&pdev->dev, "%s: syncpt allocation failed\n", __func__);
		return -ENODEV;
	}

	*syncpt_id = id;

	return 0;
}
EXPORT_SYMBOL_GPL(t194_capture_alloc_syncpt);

void t194_capture_release_syncpt(struct platform_device *pdev, uint32_t id)
{
	dev_dbg(&pdev->dev, "%s: id=%u\n", __func__, id);
	nvhost_syncpt_put_ref_ext(pdev, id);
}
EXPORT_SYMBOL_GPL(t194_capture_release_syncpt);

void t194_capture_get_gos_table(struct platform_device *pdev,
				int *gos_count,
				const dma_addr_t **gos_table)
{
	int count = 0;
	dma_addr_t *table = NULL;

	/* Using information cached during the probe, this should never fail */
	(void)nvhost_syncpt_get_cv_dev_address_table(pdev, &count, &table);

	*gos_count = count;
	*gos_table = table;
}
EXPORT_SYMBOL_GPL(t194_capture_get_gos_table);

int t194_capture_get_syncpt_gos_backing(struct platform_device *pdev,
			uint32_t id,
			dma_addr_t *syncpt_addr,
			uint32_t *gos_index,
			uint32_t *gos_offset)
{
	uint32_t index = GOS_INDEX_INVALID;
	uint32_t offset = 0;
	dma_addr_t addr;
	int err = 0;

	if (id == 0) {
		dev_err(&pdev->dev, "%s: syncpt id is invalid\n", __func__);
		return -EINVAL;
	}

	if (syncpt_addr == NULL || gos_index == NULL || gos_offset == NULL) {
		dev_err(&pdev->dev, "%s: null arguments\n", __func__);
		return -EINVAL;
	}

	addr = nvhost_syncpt_address(pdev, id);
	err = nvhost_syncpt_get_gos(pdev, id, &index, &offset);
	if (err < 0) {
		dev_dbg(&pdev->dev, "%s: failed to get GoS backing: %d\n",
			__func__, err);
	}

	*syncpt_addr = addr;
	*gos_index = index;
	*gos_offset = offset;

	dev_dbg(&pdev->dev, "%s: id=%u addr=0x%llx gos_idx=%u gos_offset=%u\n",
		__func__, id, addr, index, offset);

	return 0;
}
EXPORT_SYMBOL_GPL(t194_capture_get_syncpt_gos_backing);

static int t194_capture_support_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	int err = 0;

	info = (void *)of_device_get_match_data(dev);
	if (WARN_ON(info == NULL))
		return -ENODATA;

	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);

	(void) dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto error;

	err = nvhost_module_init(pdev);
	if (err)
		goto error;

	err = nvhost_client_device_init(pdev);
	if (err) {
		nvhost_module_deinit(pdev);
		goto error;
	}

	err = nvhost_syncpt_unit_interface_init(pdev);
	if (err)
		goto device_release;

	return 0;

device_release:
	nvhost_client_device_release(pdev);
error:
	if (err != -EPROBE_DEFER)
		dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static int t194_capture_support_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id t194_capture_support_match[] = {
	{
		.compatible = "nvidia,tegra194-isp-thi",
		.data = &t19_isp_thi_info,
	},
	{
		.compatible = "nvidia,tegra194-vi-thi",
		.data = &t19_vi_thi_info,
	},
	{ },
};

static struct platform_driver t194_capture_support_driver = {
	.probe = t194_capture_support_probe,
	.remove = t194_capture_support_remove,
	.driver = {
		/* Only suitable name for dummy falcon driver */
		.name = "scare-pigeon",
		.of_match_table = t194_capture_support_match,
		.pm = &nvhost_module_pm_ops,
	},
};

module_platform_driver(t194_capture_support_driver);
