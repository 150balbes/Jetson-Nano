/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_LINUX_DMABUF_VIDMEM_H__
#define __NVGPU_LINUX_DMABUF_VIDMEM_H__

#include <nvgpu/types.h>

struct dma_buf;

struct gk20a;
struct scatterlist;

#ifdef CONFIG_GK20A_VIDMEM

struct gk20a *nvgpu_vidmem_buf_owner(struct dma_buf *dmabuf);
int nvgpu_vidmem_export_linux(struct gk20a *g, size_t bytes);

void nvgpu_vidmem_set_page_alloc(struct scatterlist *sgl, u64 addr);
struct nvgpu_page_alloc *nvgpu_vidmem_get_page_alloc(struct scatterlist *sgl);

int nvgpu_vidmem_buf_access_memory(struct gk20a *g, struct dma_buf *dmabuf,
		void *buffer, u64 offset, u64 size, u32 cmd);

#else /* !CONFIG_GK20A_VIDMEM */

static inline struct gk20a *nvgpu_vidmem_buf_owner(struct dma_buf *dmabuf)
{
	return NULL;
}

static inline int nvgpu_vidmem_export_linux(struct gk20a *g, size_t bytes)
{
	return -ENOSYS;
}

static inline void nvgpu_vidmem_set_page_alloc(struct scatterlist *sgl,
					       u64 addr)
{
}

static inline struct nvgpu_page_alloc *nvgpu_vidmem_get_page_alloc(
	struct scatterlist *sgl)
{
	return NULL;
}

static inline int nvgpu_vidmem_buf_access_memory(struct gk20a *g,
						 struct dma_buf *dmabuf,
						 void *buffer, u64 offset,
						 u64 size, u32 cmd)
{
	return -ENOSYS;
}

#endif


struct nvgpu_vidmem_linux {
	struct dma_buf	*dmabuf;
	void		*dmabuf_priv;
	void		(*dmabuf_priv_delete)(void *);
};

#endif
