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

#ifndef __COMMON_LINUX_DMABUF_H__
#define __COMMON_LINUX_DMABUF_H__

#include <nvgpu/comptags.h>
#include <nvgpu/list.h>
#include <nvgpu/lock.h>
#include <nvgpu/gmmu.h>

struct sg_table;
struct dma_buf;
struct dma_buf_attachment;
struct device;

struct gk20a;
struct gk20a_buffer_state;

struct gk20a_dmabuf_priv {
	struct nvgpu_mutex lock;

	struct gk20a *g;

	struct gk20a_comptag_allocator *comptag_allocator;
	struct gk20a_comptags comptags;

	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	int pin_count;

	struct nvgpu_list_node states;

	u64 buffer_id;
};

struct sg_table *gk20a_mm_pin(struct device *dev, struct dma_buf *dmabuf,
			      struct dma_buf_attachment **attachment);
void gk20a_mm_unpin(struct device *dev, struct dma_buf *dmabuf,
		    struct dma_buf_attachment *attachment,
		    struct sg_table *sgt);

int gk20a_dmabuf_alloc_drvdata(struct dma_buf *dmabuf, struct device *dev);

int gk20a_dmabuf_get_state(struct dma_buf *dmabuf, struct gk20a *g,
			   u64 offset, struct gk20a_buffer_state **state);

#endif
