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

#include <linux/dma-buf.h>

#include <nvgpu/comptags.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/linux/vm.h>

#include "dmabuf.h"

void gk20a_get_comptags(struct nvgpu_os_buffer *buf,
			struct gk20a_comptags *comptags)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);

	if (!comptags)
		return;

	if (!priv) {
		memset(comptags, 0, sizeof(*comptags));
		return;
	}

	nvgpu_mutex_acquire(&priv->lock);
	*comptags = priv->comptags;
	nvgpu_mutex_release(&priv->lock);
}

int gk20a_alloc_or_get_comptags(struct gk20a *g,
				struct nvgpu_os_buffer *buf,
				struct gk20a_comptag_allocator *allocator,
				struct gk20a_comptags *comptags)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);
	u32 offset;
	int err;
	unsigned int ctag_granularity;
	u32 lines;

	if (!priv)
		return -ENOSYS;

	nvgpu_mutex_acquire(&priv->lock);

	if (priv->comptags.allocated) {
		/*
		 * already allocated
		 */
		*comptags = priv->comptags;

		err = 0;
		goto exit_locked;
	}

	ctag_granularity = g->ops.fb.compression_page_size(g);
	lines = DIV_ROUND_UP_ULL(buf->dmabuf->size, ctag_granularity);

	/* 0-sized buffer? Shouldn't occur, but let's check anyways. */
	if (lines < 1) {
		err = -EINVAL;
		goto exit_locked;
	}

	/* store the allocator so we can use it when we free the ctags */
	priv->comptag_allocator = allocator;
	err = gk20a_comptaglines_alloc(allocator, &offset, lines);
	if (!err) {
		priv->comptags.offset = offset;
		priv->comptags.lines = lines;
		priv->comptags.needs_clear = true;
	} else {
		priv->comptags.offset = 0;
		priv->comptags.lines = 0;
		priv->comptags.needs_clear = false;
	}

	/*
	 * We don't report an error here if comptag alloc failed. The
	 * caller will simply fallback to incompressible kinds. It
	 * would not be safe to re-allocate comptags anyways on
	 * successive calls, as that would break map aliasing.
	 */
	err = 0;
	priv->comptags.allocated = true;

	*comptags = priv->comptags;

exit_locked:
	nvgpu_mutex_release(&priv->lock);

	return err;
}

bool gk20a_comptags_start_clear(struct nvgpu_os_buffer *buf)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);
	bool clear_started = false;

	if (priv) {
		nvgpu_mutex_acquire(&priv->lock);

		clear_started = priv->comptags.needs_clear;

		if (!clear_started)
			nvgpu_mutex_release(&priv->lock);
	}

	return clear_started;
}

void gk20a_comptags_finish_clear(struct nvgpu_os_buffer *buf,
				 bool clear_successful)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(buf->dmabuf,
							     buf->dev);
	if (priv) {
		if (clear_successful)
			priv->comptags.needs_clear = false;

		nvgpu_mutex_release(&priv->lock);
	}
}
