/*
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/bug.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/channel.h>

#include "vgpu/gm20b/vgpu_gr_gm20b.h"

#include "gp10b/gr_gp10b.h"
#include "vgpu_gr_gp10b.h"

#include <nvgpu/hw/gp10b/hw_gr_gp10b.h>

int vgpu_gr_gp10b_alloc_gr_ctx(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm,
				u32 class,
				u32 flags)
{
	u32 graphics_preempt_mode = 0;
	u32 compute_preempt_mode = 0;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int err;

	nvgpu_log_fn(g, " ");

	err = vgpu_gr_alloc_gr_ctx(g, gr_ctx, vm, class, flags);
	if (err)
		return err;

	if (flags & NVGPU_OBJ_CTX_FLAGS_SUPPORT_GFXP)
		graphics_preempt_mode = NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP;
	if (flags & NVGPU_OBJ_CTX_FLAGS_SUPPORT_CILP)
		compute_preempt_mode = NVGPU_PREEMPTION_MODE_COMPUTE_CILP;

	if (priv->constants.force_preempt_mode && !graphics_preempt_mode &&
		!compute_preempt_mode) {
		graphics_preempt_mode = g->ops.gr.is_valid_gfx_class(g, class) ?
					NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP : 0;
		compute_preempt_mode =
			g->ops.gr.is_valid_compute_class(g, class) ?
			NVGPU_PREEMPTION_MODE_COMPUTE_CTA : 0;
	}

	if (graphics_preempt_mode || compute_preempt_mode) {
		if (g->ops.gr.set_ctxsw_preemption_mode) {
			err = g->ops.gr.set_ctxsw_preemption_mode(g, gr_ctx, vm,
			    class, graphics_preempt_mode, compute_preempt_mode);
			if (err) {
				nvgpu_err(g,
					"set_ctxsw_preemption_mode failed");
				goto fail;
			}
		} else {
			err = -ENOSYS;
			goto fail;
		}
	}

	nvgpu_log_fn(g, "done");
	return err;

fail:
	vgpu_gr_free_gr_ctx(g, vm, gr_ctx);
	return err;
}

int vgpu_gr_gp10b_set_ctxsw_preemption_mode(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gr_bind_ctxsw_buffers_params *p =
				&msg.params.gr_bind_ctxsw_buffers;
	int err = 0;

	if (g->ops.gr.is_valid_gfx_class(g, class) &&
			g->gr.ctx_vars.force_preemption_gfxp)
		graphics_preempt_mode = NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP;

	if (g->ops.gr.is_valid_compute_class(g, class) &&
			g->gr.ctx_vars.force_preemption_cilp)
		compute_preempt_mode = NVGPU_PREEMPTION_MODE_COMPUTE_CILP;

	/* check for invalid combinations */
	if ((graphics_preempt_mode == 0) && (compute_preempt_mode == 0))
		return -EINVAL;

	if ((graphics_preempt_mode == NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP) &&
		   (compute_preempt_mode == NVGPU_PREEMPTION_MODE_COMPUTE_CILP))
		return -EINVAL;

	/* set preemption modes */
	switch (graphics_preempt_mode) {
	case NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP:
	{
		u32 spill_size =
			gr_gpc0_swdx_rm_spill_buffer_size_256b_default_v() *
			gr_gpc0_swdx_rm_spill_buffer_size_256b_byte_granularity_v();
		u32 pagepool_size = g->ops.gr.pagepool_default_size(g) *
			gr_scc_pagepool_total_pages_byte_granularity_v();
		u32 betacb_size = g->gr.attrib_cb_default_size +
				  (gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
				   gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
		u32 attrib_cb_size = (betacb_size + g->gr.alpha_cb_size) *
				  gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
				  g->gr.max_tpc_count;
		struct nvgpu_mem *desc;

		attrib_cb_size = ALIGN(attrib_cb_size, 128);

		nvgpu_log_info(g, "gfxp context preempt size=%d",
			g->gr.ctx_vars.preempt_image_size);
		nvgpu_log_info(g, "gfxp context spill size=%d", spill_size);
		nvgpu_log_info(g, "gfxp context pagepool size=%d", pagepool_size);
		nvgpu_log_info(g, "gfxp context attrib cb size=%d",
			attrib_cb_size);

		/* Only allocate buffers the first time through */
		if (!nvgpu_mem_is_valid(&gr_ctx->preempt_ctxsw_buffer)) {
			err = gr_gp10b_alloc_buffer(vm,
					g->gr.ctx_vars.preempt_image_size,
					&gr_ctx->preempt_ctxsw_buffer);
			if (err) {
				err = -ENOMEM;
				goto fail;
			}

			err = gr_gp10b_alloc_buffer(vm,
					spill_size,
					&gr_ctx->spill_ctxsw_buffer);
			if (err) {
				err = -ENOMEM;
				goto fail;
			}
			err = gr_gp10b_alloc_buffer(vm,
					pagepool_size,
					&gr_ctx->pagepool_ctxsw_buffer);
			if (err) {
				err = -ENOMEM;
				goto fail;
			}
			err = gr_gp10b_alloc_buffer(vm,
					attrib_cb_size,
					&gr_ctx->betacb_ctxsw_buffer);
			if (err) {
				err = -ENOMEM;
				goto fail;
			}
		}

		desc = &gr_ctx->preempt_ctxsw_buffer;
		p->gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAIN] = desc->gpu_va;
		p->size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAIN] = desc->size;

		desc = &gr_ctx->spill_ctxsw_buffer;
		p->gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_SPILL] = desc->gpu_va;
		p->size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_SPILL] = desc->size;

		desc = &gr_ctx->pagepool_ctxsw_buffer;
		p->gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_PAGEPOOL] =
			desc->gpu_va;
		p->size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_PAGEPOOL] = desc->size;

		desc = &gr_ctx->betacb_ctxsw_buffer;
		p->gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_BETACB] =
			desc->gpu_va;
		p->size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_BETACB] = desc->size;

		gr_ctx->graphics_preempt_mode = NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP;
		p->mode = TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_GFX_GFXP;
		break;
	}
	case NVGPU_PREEMPTION_MODE_GRAPHICS_WFI:
		gr_ctx->graphics_preempt_mode = graphics_preempt_mode;
		break;

	default:
		break;
	}

	if (g->ops.gr.is_valid_compute_class(g, class)) {
		switch (compute_preempt_mode) {
		case NVGPU_PREEMPTION_MODE_COMPUTE_WFI:
			gr_ctx->compute_preempt_mode =
				NVGPU_PREEMPTION_MODE_COMPUTE_WFI;
			p->mode = TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_WFI;
			break;
		case NVGPU_PREEMPTION_MODE_COMPUTE_CTA:
			gr_ctx->compute_preempt_mode =
				NVGPU_PREEMPTION_MODE_COMPUTE_CTA;
			p->mode =
				TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_COMPUTE_CTA;
			break;
		case NVGPU_PREEMPTION_MODE_COMPUTE_CILP:
			gr_ctx->compute_preempt_mode =
				NVGPU_PREEMPTION_MODE_COMPUTE_CILP;
			p->mode =
				TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_COMPUTE_CILP;
			break;
		default:
			break;
		}
	}

	if (gr_ctx->graphics_preempt_mode || gr_ctx->compute_preempt_mode) {
		msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTXSW_BUFFERS;
		msg.handle = vgpu_get_handle(g);
		p->gr_ctx_handle = gr_ctx->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		if (err || msg.ret) {
			err = -ENOMEM;
			goto fail;
		}
	}

	return err;

fail:
	nvgpu_err(g, "%s failed %d", __func__, err);
	return err;
}

int vgpu_gr_gp10b_set_preemption_mode(struct channel_gk20a *ch,
					u32 graphics_preempt_mode,
					u32 compute_preempt_mode)
{
	struct nvgpu_gr_ctx *gr_ctx;
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg;
	struct vm_gk20a *vm;
	u32 class;
	int err;

	class = ch->obj_class;
	if (!class)
		return -EINVAL;

	tsg = tsg_gk20a_from_ch(ch);
	if (!tsg)
		return -EINVAL;

	vm = tsg->vm;
	gr_ctx = &tsg->gr_ctx;

	/* skip setting anything if both modes are already set */
	if (graphics_preempt_mode &&
	   (graphics_preempt_mode == gr_ctx->graphics_preempt_mode))
		graphics_preempt_mode = 0;

	if (compute_preempt_mode &&
	   (compute_preempt_mode == gr_ctx->compute_preempt_mode))
		compute_preempt_mode = 0;

	if (graphics_preempt_mode == 0 && compute_preempt_mode == 0)
		return 0;

	if (g->ops.gr.set_ctxsw_preemption_mode) {
		err = g->ops.gr.set_ctxsw_preemption_mode(g, gr_ctx, vm, class,
						graphics_preempt_mode,
						compute_preempt_mode);
		if (err) {
			nvgpu_err(g, "set_ctxsw_preemption_mode failed");
			return err;
		}
	} else {
		err = -ENOSYS;
	}

	return err;
}

int vgpu_gr_gp10b_init_ctx_state(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int err;

	nvgpu_log_fn(g, " ");

	err = vgpu_gr_init_ctx_state(g);
	if (err)
		return err;

	g->gr.ctx_vars.preempt_image_size =
			priv->constants.preempt_ctx_size;
	if (!g->gr.ctx_vars.preempt_image_size)
		return -EINVAL;

	return 0;
}
