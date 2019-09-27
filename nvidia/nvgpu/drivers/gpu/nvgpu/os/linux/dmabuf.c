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

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/scatterlist.h>

#include <nvgpu/comptags.h>
#include <nvgpu/enabled.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/linux/vm.h>

#include "gk20a/fence_gk20a.h"

#include "platform_gk20a.h"
#include "dmabuf.h"
#include "os_linux.h"
#include "dmabuf_vidmem.h"

static void gk20a_mm_delete_priv(void *_priv)
{
	struct gk20a_buffer_state *s, *s_tmp;
	struct gk20a_dmabuf_priv *priv = _priv;
	struct gk20a *g;

	if (!priv)
		return;

	g = priv->g;

	if (priv->comptags.allocated && priv->comptags.lines) {
		BUG_ON(!priv->comptag_allocator);
		gk20a_comptaglines_free(priv->comptag_allocator,
				priv->comptags.offset,
				priv->comptags.lines);
	}

	/* Free buffer states */
	nvgpu_list_for_each_entry_safe(s, s_tmp, &priv->states,
				gk20a_buffer_state, list) {
		gk20a_fence_put(s->fence);
		nvgpu_list_del(&s->list);
		nvgpu_kfree(g, s);
	}

	nvgpu_kfree(g, priv);
}

enum nvgpu_aperture gk20a_dmabuf_aperture(struct gk20a *g,
					  struct dma_buf *dmabuf)
{
	struct gk20a *buf_owner = nvgpu_vidmem_buf_owner(dmabuf);
	bool unified_memory = nvgpu_is_enabled(g, NVGPU_MM_UNIFIED_MEMORY);

	if (buf_owner == NULL) {
		/* Not nvgpu-allocated, assume system memory */
		return APERTURE_SYSMEM;
	} else if (WARN_ON(buf_owner == g && unified_memory)) {
		/* Looks like our video memory, but this gpu doesn't support
		 * it. Warn about a bug and bail out */
		nvgpu_warn(g,
			"dmabuf is our vidmem but we don't have local vidmem");
		return APERTURE_INVALID;
	} else if (buf_owner != g) {
		/* Someone else's vidmem */
		return APERTURE_INVALID;
	} else {
		/* Yay, buf_owner == g */
		return APERTURE_VIDMEM;
	}
}

struct sg_table *gk20a_mm_pin(struct device *dev, struct dma_buf *dmabuf,
			      struct dma_buf_attachment **attachment)
{
	struct gk20a_dmabuf_priv *priv;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return ERR_PTR(-EINVAL);

	nvgpu_mutex_acquire(&priv->lock);

	if (priv->pin_count == 0) {
		priv->attach = dma_buf_attach(dmabuf, dev);
		if (IS_ERR(priv->attach)) {
			nvgpu_mutex_release(&priv->lock);
			return (struct sg_table *)priv->attach;
		}

		priv->sgt = dma_buf_map_attachment(priv->attach,
						   DMA_BIDIRECTIONAL);
		if (IS_ERR(priv->sgt)) {
			dma_buf_detach(dmabuf, priv->attach);
			nvgpu_mutex_release(&priv->lock);
			return priv->sgt;
		}
	}

	priv->pin_count++;
	nvgpu_mutex_release(&priv->lock);
	*attachment = priv->attach;
	return priv->sgt;
}

void gk20a_mm_unpin(struct device *dev, struct dma_buf *dmabuf,
		    struct dma_buf_attachment *attachment,
		    struct sg_table *sgt)
{
	struct gk20a_dmabuf_priv *priv = dma_buf_get_drvdata(dmabuf, dev);
	dma_addr_t dma_addr;

	if (IS_ERR(priv) || !priv)
		return;

	nvgpu_mutex_acquire(&priv->lock);
	WARN_ON(priv->sgt != sgt);
	WARN_ON(priv->attach != attachment);
	priv->pin_count--;
	WARN_ON(priv->pin_count < 0);
	dma_addr = sg_dma_address(priv->sgt->sgl);
	if (priv->pin_count == 0) {
		dma_buf_unmap_attachment(priv->attach, priv->sgt,
					 DMA_BIDIRECTIONAL);
		dma_buf_detach(dmabuf, priv->attach);
	}
	nvgpu_mutex_release(&priv->lock);
}

int gk20a_dmabuf_alloc_drvdata(struct dma_buf *dmabuf, struct device *dev)
{
	struct gk20a *g = gk20a_get_platform(dev)->g;
	struct gk20a_dmabuf_priv *priv;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (likely(priv))
		return 0;

	nvgpu_mutex_acquire(&g->mm.priv_lock);
	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (priv)
		goto priv_exist_or_err;

	priv = nvgpu_kzalloc(g, sizeof(*priv));
	if (!priv) {
		priv = ERR_PTR(-ENOMEM);
		goto priv_exist_or_err;
	}

	nvgpu_mutex_init(&priv->lock);
	nvgpu_init_list_node(&priv->states);
	priv->g = g;
	dma_buf_set_drvdata(dmabuf, dev, priv, gk20a_mm_delete_priv);

priv_exist_or_err:
	nvgpu_mutex_release(&g->mm.priv_lock);
	if (IS_ERR(priv))
		return -ENOMEM;

	return 0;
}

int gk20a_dmabuf_get_state(struct dma_buf *dmabuf, struct gk20a *g,
			   u64 offset, struct gk20a_buffer_state **state)
{
	int err = 0;
	struct gk20a_dmabuf_priv *priv;
	struct gk20a_buffer_state *s;
	struct device *dev = dev_from_gk20a(g);

	if (WARN_ON(offset >= (u64)dmabuf->size))
		return -EINVAL;

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev);
	if (err)
		return err;

	priv = dma_buf_get_drvdata(dmabuf, dev);
	if (WARN_ON(!priv))
		return -ENOSYS;

	nvgpu_mutex_acquire(&priv->lock);

	nvgpu_list_for_each_entry(s, &priv->states, gk20a_buffer_state, list)
		if (s->offset == offset)
			goto out;

	/* State not found, create state. */
	s = nvgpu_kzalloc(g, sizeof(*s));
	if (!s) {
		err = -ENOMEM;
		goto out;
	}

	s->offset = offset;
	nvgpu_init_list_node(&s->list);
	nvgpu_mutex_init(&s->lock);
	nvgpu_list_add_tail(&s->list, &priv->states);

out:
	nvgpu_mutex_release(&priv->lock);
	if (!err)
		*state = s;
	return err;
}
