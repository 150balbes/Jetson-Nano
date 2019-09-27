/*
 * drivers/video/tegra/nvmap/nvmap_dmabuf_t19x.c
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/nvmap_t19x.h>

#include "nvmap_priv.h"

extern bool of_dma_is_coherent(struct device_node *np);

static void nvmap_handle_t19x_free(void *ptr)
{
	struct nvmap_handle_t19x *handle_t19x =
		(struct nvmap_handle_t19x *)ptr;
	int outstanding_nc_pin = atomic_read(&handle_t19x->nc_pin);

	WARN(outstanding_nc_pin,
		"outstanding dma maps from %d coherent devices",
		outstanding_nc_pin);
	kfree(handle_t19x);
}

struct sg_table *nvmap_dmabuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;
	struct nvmap_handle *handle = info->handle;
	struct nvmap_handle_t19x *handle_t19x = NULL;
	struct device *dev = nvmap_dev->dev_user.parent;
	struct sg_table *sg_table;

	if (!nvmap_version_t19x)
		goto dmabuf_map;

	handle_t19x = dma_buf_get_drvdata(handle->dmabuf, dev);
	if (!handle_t19x && !of_dma_is_coherent(attach->dev->of_node)) {
		handle_t19x = kmalloc(sizeof(*handle_t19x), GFP_KERNEL);
		if (WARN(!handle_t19x, "No memory!!"))
			return ERR_PTR(-ENOMEM);

		atomic_set(&handle_t19x->nc_pin, 0);
		dma_buf_set_drvdata(handle->dmabuf, dev,
				handle_t19x, nvmap_handle_t19x_free);
	}

	if (!of_dma_is_coherent(attach->dev->of_node))
		atomic_inc(&handle_t19x->nc_pin);

dmabuf_map:
	sg_table = _nvmap_dmabuf_map_dma_buf(attach, dir);
	/* no need to free handle_t19x, it is freed with handle */
	if (IS_ERR(sg_table))
		if (handle_t19x)
			atomic_dec(&handle_t19x->nc_pin);

	return sg_table;
}

void nvmap_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
	 struct sg_table *sgt, enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;
	struct nvmap_handle *handle = info->handle;
	struct device *dev = nvmap_dev->dev_user.parent;
	struct nvmap_handle_t19x *handle_t19x;

	_nvmap_dmabuf_unmap_dma_buf(attach, sgt, dir);

	handle_t19x = dma_buf_get_drvdata(handle->dmabuf, dev);
	if (handle_t19x && !of_dma_is_coherent(attach->dev->of_node))
		atomic_dec(&handle_t19x->nc_pin);
}

