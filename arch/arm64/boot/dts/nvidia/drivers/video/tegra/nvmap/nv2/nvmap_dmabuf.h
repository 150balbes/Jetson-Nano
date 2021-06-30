/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVMAP_DMABUF_H
#define __NVMAP_DMABUF_H

#include "nvmap_structs.h"

struct nvmap_handle_info {
	struct nvmap_handle *handle;
	struct list_head maps;
	struct mutex maps_lock;
};

struct dma_buf *nvmap_dmabuf_create(void * priv, size_t size);
void nvmap_dmabuf_install_fd(struct dma_buf *dmabuf, int fd);
struct nvmap_handle * nvmap_dmabuf_to_handle(struct dma_buf *dmabuf);
struct dma_buf *nvmap_dmabuf_from_fd(int fd);
int nvmap_dmabuf_is_nvmap(struct dma_buf *dmabuf);

void nvmap_dmabufs_free(struct list_head *dmabuf_list);

#endif /* __NVMAP_DMABUF_H */
