/*
 * dma_buf exporter for nvmap
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/nvmap.h>
#include <linux/dma-buf.h>
#include <linux/version.h>

#include <trace/events/nvmap.h>

#include "nvmap_dmabuf.h"

extern bool dmabuf_is_nvmap(struct dma_buf *dmabuf);
extern struct dma_buf_ops nvmap_dma_buf_ops;

int nvmap_dmabuf_is_nvmap(struct dma_buf *dmabuf)
{
	return dmabuf_is_nvmap(dmabuf);
}

struct dma_buf *nvmap_dmabuf_from_fd(int fd)
{
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return dmabuf;
	dma_buf_put(dmabuf);
	return dmabuf;
}

struct nvmap_handle * nvmap_dmabuf_to_handle(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info;
	if (!dmabuf_is_nvmap(dmabuf)) {
		return ERR_PTR(-EINVAL);
	}

	info = dmabuf->priv;
	return info->handle;
}

void nvmap_dmabuf_install_fd(struct dma_buf *dmabuf, int fd)
{
	fd_install(fd, dmabuf->file);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0)
static struct dma_buf *__dma_buf_export(void * priv,
					size_t size)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	exp_info.priv = priv;
	exp_info.ops = &nvmap_dma_buf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.exp_flags = DMABUF_CAN_DEFER_UNMAP |
				DMABUF_SKIP_CACHE_SYNC;

	return dma_buf_export(&exp_info);
}
#else
#define __dma_buf_export(priv, size) \
	dma_buf_export(priv, &nvmap_dma_buf_ops, size, O_RDWR, NULL)
#endif
/*
 * Make a dmabuf object for an nvmap handle.
 */
struct dma_buf *nvmap_dmabuf_create(void * priv, size_t size)
{
	return __dma_buf_export(priv, size);
}
