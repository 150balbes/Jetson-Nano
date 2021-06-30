/*
 * GP106 GPU GR
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/dma.h>
#include <nvgpu/gk20a.h>

#include "gk20a/gr_gk20a.h"
#include "gm20b/gr_gm20b.h"
#include "gp10b/gr_gp10b.h"

#include <nvgpu/io.h>

#include "gr_gp106.h"

#include <nvgpu/hw/gp106/hw_gr_gp106.h>

bool gr_gp106_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case PASCAL_COMPUTE_A:
	case PASCAL_COMPUTE_B:
	case PASCAL_A:
	case PASCAL_B:
	case PASCAL_DMA_COPY_A:
	case PASCAL_DMA_COPY_B:
		valid = true;
		break;

	case MAXWELL_COMPUTE_B:
	case MAXWELL_B:
	case FERMI_TWOD_A:
	case KEPLER_DMA_COPY_A:
	case MAXWELL_DMA_COPY_A:
		valid = true;
		break;

	default:
		break;
	}
	nvgpu_log_info(g, "class=0x%x valid=%d", class_num, valid);
	return valid;
}

u32 gr_gp106_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

static void gr_gp106_set_go_idle_timeout(struct gk20a *g, u32 data)
{
	gk20a_writel(g, gr_fe_go_idle_timeout_r(), data);
}

int gr_gp106_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (class_num == PASCAL_COMPUTE_B) {
		switch (offset << 2) {
		case NVC0C0_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVC0C0_SET_RD_COALESCE:
			gr_gm20b_set_rd_coalesce(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == PASCAL_B) {
		switch (offset << 2) {
		case NVC097_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVC097_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVC097_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		case NVC097_SET_GO_IDLE_TIMEOUT:
			gr_gp106_set_go_idle_timeout(g, data);
			break;
		case NVC097_SET_RD_COALESCE:
			gr_gm20b_set_rd_coalesce(g, data);
			break;
		case NVC097_SET_BES_CROP_DEBUG3:
			g->ops.gr.set_bes_crop_debug3(g, data);
			break;
		case NVC097_SET_BES_CROP_DEBUG4:
			g->ops.gr.set_bes_crop_debug4(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

void gr_gp106_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size) {
		gr->attrib_cb_default_size = 0x800;
	}
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
	gr->attrib_cb_gfxp_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
	gr->attrib_cb_gfxp_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
}

int gr_gp106_set_ctxsw_preemption_mode(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode)
{
	int err = 0;

	if (class == PASCAL_B && g->gr.ctx_vars.force_preemption_gfxp) {
		graphics_preempt_mode = NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP;
	}

	if (class == PASCAL_COMPUTE_B &&
			g->gr.ctx_vars.force_preemption_cilp) {
		compute_preempt_mode = NVGPU_PREEMPTION_MODE_COMPUTE_CILP;
	}

	/* check for invalid combinations */
	if ((graphics_preempt_mode == 0) && (compute_preempt_mode == 0)) {
		return -EINVAL;
	}

	if ((graphics_preempt_mode == NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP) &&
		   (compute_preempt_mode == NVGPU_PREEMPTION_MODE_COMPUTE_CILP)) {
		return -EINVAL;
	}

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
		attrib_cb_size = ALIGN(attrib_cb_size, 128);

		nvgpu_log_info(g, "gfxp context spill_size=%d", spill_size);
		nvgpu_log_info(g, "gfxp context pagepool_size=%d", pagepool_size);
		nvgpu_log_info(g, "gfxp context attrib_cb_size=%d",
				attrib_cb_size);

		/* Only allocate buffers the first time through */
		if (!nvgpu_mem_is_valid(&gr_ctx->preempt_ctxsw_buffer)) {
			err = gr_gp10b_alloc_buffer(vm,
					g->gr.ctx_vars.preempt_image_size,
					&gr_ctx->preempt_ctxsw_buffer);
			if (err) {
				nvgpu_err(g, "cannot allocate preempt buffer");
				goto fail;
			}

			err = gr_gp10b_alloc_buffer(vm,
					spill_size,
					&gr_ctx->spill_ctxsw_buffer);
			if (err) {
				nvgpu_err(g, "cannot allocate spill buffer");
				goto fail_free_preempt;
			}

			err = gr_gp10b_alloc_buffer(vm,
					attrib_cb_size,
					&gr_ctx->betacb_ctxsw_buffer);
			if (err) {
				nvgpu_err(g, "cannot allocate beta buffer");
				goto fail_free_spill;
			}

			err = gr_gp10b_alloc_buffer(vm,
					pagepool_size,
					&gr_ctx->pagepool_ctxsw_buffer);
			if (err) {
				nvgpu_err(g, "cannot allocate page pool");
				goto fail_free_betacb;
			}
		}

		gr_ctx->graphics_preempt_mode = graphics_preempt_mode;
		break;
		}

	case NVGPU_PREEMPTION_MODE_GRAPHICS_WFI:
		gr_ctx->graphics_preempt_mode = graphics_preempt_mode;
		break;

	default:
		break;
	}

	if (class == PASCAL_COMPUTE_B) {
		switch (compute_preempt_mode) {
		case NVGPU_PREEMPTION_MODE_COMPUTE_WFI:
		case NVGPU_PREEMPTION_MODE_COMPUTE_CTA:
		case NVGPU_PREEMPTION_MODE_COMPUTE_CILP:
			gr_ctx->compute_preempt_mode = compute_preempt_mode;
			break;
		default:
			break;
		}
	}

	return 0;

fail_free_betacb:
	nvgpu_dma_unmap_free(vm, &gr_ctx->betacb_ctxsw_buffer);
fail_free_spill:
	nvgpu_dma_unmap_free(vm, &gr_ctx->spill_ctxsw_buffer);
fail_free_preempt:
	nvgpu_dma_unmap_free(vm, &gr_ctx->preempt_ctxsw_buffer);
fail:
	return err;
}
