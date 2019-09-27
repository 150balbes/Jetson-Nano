/*
 * util.c: Utility functions for tegradc ext interface.
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION, All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/err.h>
#include <linux/types.h>
#include <linux/dma-buf.h>

#include "../dc.h"
#include "tegra_dc_ext_priv.h"


int tegra_dc_ext_pin_window(struct tegra_dc_ext_user *user, u32 fd,
			    struct tegra_dc_dmabuf **dc_buf,
			    dma_addr_t *phys_addr)
{
	struct tegra_dc_ext *ext = user->ext;
	struct tegra_dc_dmabuf *dc_dmabuf;
	dma_addr_t dma_addr;

	*dc_buf = NULL;
	*phys_addr = -1;
	if (!fd)
		return 0;

	dc_dmabuf = kzalloc(sizeof(*dc_dmabuf), GFP_KERNEL);
	if (!dc_dmabuf)
		return -ENOMEM;

	dc_dmabuf->buf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dc_dmabuf->buf))
		goto buf_fail;

	dc_dmabuf->attach = dma_buf_attach(dc_dmabuf->buf, ext->dev->parent);
	if (IS_ERR_OR_NULL(dc_dmabuf->attach))
		goto attach_fail;

	dc_dmabuf->sgt = dma_buf_map_attachment(dc_dmabuf->attach,
						DMA_TO_DEVICE);
	if (IS_ERR_OR_NULL(dc_dmabuf->sgt))
		goto sgt_fail;

	if (!device_is_iommuable(ext->dev->parent) &&
			sg_nents(dc_dmabuf->sgt->sgl) > 1) {
		dev_err(ext->dev->parent,
			"Cannot use non-contiguous buffer w/ IOMMU disabled\n");
		goto iommu_fail;
	}

	dma_addr = sg_dma_address(dc_dmabuf->sgt->sgl);
	if (dma_addr)
		*phys_addr = dma_addr;
	else
		*phys_addr = sg_phys(dc_dmabuf->sgt->sgl);

	*dc_buf = dc_dmabuf;

	return 0;
iommu_fail:
	dma_buf_unmap_attachment(dc_dmabuf->attach, dc_dmabuf->sgt,
		DMA_TO_DEVICE);
sgt_fail:
	dma_buf_detach(dc_dmabuf->buf, dc_dmabuf->attach);
attach_fail:
	dma_buf_put(dc_dmabuf->buf);
buf_fail:
	kfree(dc_dmabuf);
	return -ENOMEM;
}

int tegra_dc_ext_cpy_caps_from_user(void __user *user_arg,
				struct tegra_dc_ext_caps **caps_ptr,
				u32 *nr_elements_ptr)
{
	unsigned int nr_elements = 0;
	struct tegra_dc_ext_get_cap_info args;
	struct tegra_dc_ext_caps *caps = NULL;

	if (copy_from_user(&args, user_arg, sizeof(args))) {
		pr_err("%s: Failed to copy cap info from user\n", __func__);
		return -EFAULT;
	}

	nr_elements = args.nr_elements;
	if (nr_elements > 0) {
		caps = kzalloc(sizeof(*caps)
				* nr_elements, GFP_KERNEL);
		if (!caps)
			return -ENOMEM;

		if (copy_from_user(caps,
			(void __user *) (uintptr_t)args.data,
			sizeof(*caps) * nr_elements)) {
			pr_err("%s: Failed to copy caps from user\n", __func__);
			kfree(caps);

			return -EFAULT;
		}
	}

	*caps_ptr = caps;
	*nr_elements_ptr = nr_elements;

	return 0;
}
