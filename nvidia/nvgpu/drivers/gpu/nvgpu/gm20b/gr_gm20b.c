/*
 * GM20B GPC MMU
 *
 * Copyright (c) 2011-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/fuse.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include "gk20a/gr_gk20a.h"
#include "gk20a/regops_gk20a.h"

#include "gr_gm20b.h"
#include "pmu_gm20b.h"

#include <nvgpu/hw/gm20b/hw_gr_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fifo_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>
#include <nvgpu/hw/gm20b/hw_ctxsw_prog_gm20b.h>
#include <nvgpu/hw/gm20b/hw_perf_gm20b.h>

void gr_gm20b_init_gpc_mmu(struct gk20a *g)
{
	u32 temp;

	nvgpu_log_info(g, "initialize gpc mmu");

	temp = g->ops.fb.mmu_ctrl(g);
	temp &= gr_gpcs_pri_mmu_ctrl_vm_pg_size_m() |
		gr_gpcs_pri_mmu_ctrl_use_pdb_big_page_size_m() |
		gr_gpcs_pri_mmu_ctrl_use_full_comp_tag_line_m() |
		gr_gpcs_pri_mmu_ctrl_vol_fault_m() |
		gr_gpcs_pri_mmu_ctrl_comp_fault_m() |
		gr_gpcs_pri_mmu_ctrl_miss_gran_m() |
		gr_gpcs_pri_mmu_ctrl_cache_mode_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_aperture_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_vol_m() |
		gr_gpcs_pri_mmu_ctrl_mmu_disable_m();
	gk20a_writel(g, gr_gpcs_pri_mmu_ctrl_r(), temp);
	gk20a_writel(g, gr_gpcs_pri_mmu_pm_unit_mask_r(), 0);
	gk20a_writel(g, gr_gpcs_pri_mmu_pm_req_mask_r(), 0);

	gk20a_writel(g, gr_gpcs_pri_mmu_debug_ctrl_r(),
			g->ops.fb.mmu_debug_ctrl(g));
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_wr_r(),
			g->ops.fb.mmu_debug_wr(g));
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_rd_r(),
			g->ops.fb.mmu_debug_rd(g));

	gk20a_writel(g, gr_gpcs_mmu_num_active_ltcs_r(), g->ltc_count);
}

void gr_gm20b_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

void gr_gm20b_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size) {
		gr->attrib_cb_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	}
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
}

int gr_gm20b_calc_global_ctx_buffer_size(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int size;

	gr->attrib_cb_size = gr->attrib_cb_default_size
		+ (gr->attrib_cb_default_size >> 1);
	gr->alpha_cb_size = gr->alpha_cb_default_size
		+ (gr->alpha_cb_default_size >> 1);

	size = gr->attrib_cb_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size += gr->alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	return size;
}

void gr_gm20b_commit_global_attrib_cb(struct gk20a *g,
				      struct nvgpu_gr_ctx *ch_ctx,
				      u64 addr, bool patch)
{
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_setup_attrib_cb_base_r(),
		gr_gpcs_setup_attrib_cb_base_addr_39_12_f(addr) |
		gr_gpcs_setup_attrib_cb_base_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pe_pin_cb_global_base_addr_r(),
		gr_gpcs_tpcs_pe_pin_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_pe_pin_cb_global_base_addr_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_r(),
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_valid_true_f(), patch);
}

void gr_gm20b_commit_global_bundle_cb(struct gk20a *g,
					    struct nvgpu_gr_ctx *ch_ctx,
					    u64 addr, u64 size, bool patch)
{
	u32 data;

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_bundle_cb_base_r(),
		gr_scc_bundle_cb_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_scc_bundle_cb_size_r(),
		gr_scc_bundle_cb_size_div_256b_f(size) |
		gr_scc_bundle_cb_size_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_bundle_cb_base_r(),
		gr_gpcs_swdx_bundle_cb_base_addr_39_8_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_bundle_cb_size_r(),
		gr_gpcs_swdx_bundle_cb_size_div_256b_f(size) |
		gr_gpcs_swdx_bundle_cb_size_valid_true_f(), patch);

	/* data for state_limit */
	data = (g->gr.bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v()) /
		gr_pd_ab_dist_cfg2_state_limit_scc_bundle_granularity_v();

	data = min_t(u32, data, g->gr.min_gpm_fifo_depth);

	nvgpu_log_info(g, "bundle cb token limit : %d, state limit : %d",
		   g->gr.bundle_cb_token_limit, data);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg2_r(),
		gr_pd_ab_dist_cfg2_token_limit_f(g->gr.bundle_cb_token_limit) |
		gr_pd_ab_dist_cfg2_state_limit_f(data), patch);

}

int gr_gm20b_commit_global_cb_manager(struct gk20a *g,
			struct channel_gk20a *c, bool patch)
{
	struct gr_gk20a *gr = &g->gr;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *ch_ctx;
	u32 attrib_offset_in_chunk = 0;
	u32 alpha_offset_in_chunk = 0;
	u32 pd_ab_max_output;
	u32 gpc_index, ppc_index;
	u32 cbm_cfg_size1, cbm_cfg_size2;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_STRIDE);
	u32 num_pes_per_gpc = nvgpu_get_litter_value(g,
			GPU_LIT_NUM_PES_PER_GPC);

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg) {
		return -EINVAL;
	}

	ch_ctx = &tsg->gr_ctx;

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_tga_constraintlogic_r(),
		gr_ds_tga_constraintlogic_beta_cbsize_f(gr->attrib_cb_default_size) |
		gr_ds_tga_constraintlogic_alpha_cbsize_f(gr->alpha_cb_default_size),
		patch);

	pd_ab_max_output = (gr->alpha_cb_default_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v()) /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f(), patch);

	alpha_offset_in_chunk = attrib_offset_in_chunk +
		gr->tpc_count * gr->attrib_cb_size;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		u32 temp = gpc_stride * gpc_index;
		u32 temp2 = num_pes_per_gpc * gpc_index;
		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
		     ppc_index++) {
			cbm_cfg_size1 = gr->attrib_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];
			cbm_cfg_size2 = gr->alpha_cb_default_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_size_r() + temp +
				ppc_in_gpc_stride * ppc_index,
				cbm_cfg_size1, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_beta_cb_offset_r() + temp +
				ppc_in_gpc_stride * ppc_index,
				attrib_offset_in_chunk, patch);

			attrib_offset_in_chunk += gr->attrib_cb_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_size_r() + temp +
				ppc_in_gpc_stride * ppc_index,
				cbm_cfg_size2, patch);

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpc0_ppc0_cbm_alpha_cb_offset_r() + temp +
				ppc_in_gpc_stride * ppc_index,
				alpha_offset_in_chunk, patch);

			alpha_offset_in_chunk += gr->alpha_cb_size *
				gr->pes_tpc_count[ppc_index][gpc_index];

			gr_gk20a_ctx_patch_write(g, ch_ctx,
				gr_gpcs_swdx_tc_beta_cb_size_r(ppc_index + temp2),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cbm_cfg_size1) |
				gr_gpcs_swdx_tc_beta_cb_size_div3_f(cbm_cfg_size1/3),
				patch);
		}
	}

	return 0;
}

void gr_gm20b_commit_global_pagepool(struct gk20a *g,
					    struct nvgpu_gr_ctx *ch_ctx,
					    u64 addr, u32 size, bool patch)
{
	gr_gk20a_commit_global_pagepool(g, ch_ctx, addr, size, patch);

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_swdx_rm_pagepool_r(),
		gr_gpcs_swdx_rm_pagepool_total_pages_f(size) |
		gr_gpcs_swdx_rm_pagepool_valid_true_f(), patch);

}

void gr_gm20b_set_rd_coalesce(struct gk20a *g, u32 data)
{
	u32 val;

	nvgpu_log_fn(g, " ");

	val = gk20a_readl(g, gr_gpcs_tpcs_tex_m_dbg2_r());
	val = set_field(val, gr_gpcs_tpcs_tex_m_dbg2_lg_rd_coalesce_en_m(),
			     gr_gpcs_tpcs_tex_m_dbg2_lg_rd_coalesce_en_f(data));
	gk20a_writel(g, gr_gpcs_tpcs_tex_m_dbg2_r(), val);

	nvgpu_log_fn(g, "done");
}

int gr_gm20b_handle_sw_method(struct gk20a *g, u32 addr,
					  u32 class_num, u32 offset, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (class_num == MAXWELL_COMPUTE_B) {
		switch (offset << 2) {
		case NVB1C0_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVB1C0_SET_RD_COALESCE:
			gr_gm20b_set_rd_coalesce(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == MAXWELL_B) {
		switch (offset << 2) {
		case NVB197_SET_SHADER_EXCEPTIONS:
			gk20a_gr_set_shader_exceptions(g, data);
			break;
		case NVB197_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVB197_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		case NVB197_SET_RD_COALESCE:
			gr_gm20b_set_rd_coalesce(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

void gr_gm20b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 pd_ab_max_output;
	u32 alpha_cb_size = data * 4;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_STRIDE);

	nvgpu_log_fn(g, " ");
	/* if (NO_ALPHA_BETA_TIMESLICE_SUPPORT_DEF)
		return; */

	if (alpha_cb_size > gr->alpha_cb_size) {
		alpha_cb_size = gr->alpha_cb_size;
	}

	gk20a_writel(g, gr_ds_tga_constraintlogic_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_r()) &
		 ~gr_ds_tga_constraintlogic_alpha_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_alpha_cbsize_f(alpha_cb_size));

	pd_ab_max_output = alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gk20a_writel(g, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f());

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = gpc_stride * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				ppc_in_gpc_stride * ppc_index);

			val = set_field(val, gr_gpc0_ppc0_cbm_alpha_cb_size_v_m(),
					gr_gpc0_ppc0_cbm_alpha_cb_size_v_f(alpha_cb_size *
						gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				ppc_in_gpc_stride * ppc_index, val);
		}
	}
}

void gr_gm20b_set_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 cb_size = data * 4;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 ppc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_PPC_IN_GPC_STRIDE);

	nvgpu_log_fn(g, " ");

	if (cb_size > gr->attrib_cb_size) {
		cb_size = gr->attrib_cb_size;
	}

	gk20a_writel(g, gr_ds_tga_constraintlogic_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_r()) &
		 ~gr_ds_tga_constraintlogic_beta_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_beta_cbsize_f(cb_size));

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = gpc_stride * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				ppc_in_gpc_stride * ppc_index);

			val = set_field(val,
				gr_gpc0_ppc0_cbm_beta_cb_size_v_m(),
				gr_gpc0_ppc0_cbm_beta_cb_size_v_f(cb_size *
					gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				ppc_in_gpc_stride * ppc_index, val);

			val = gk20a_readl(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index));

			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_v_m(),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(cb_size *
					gr->gpc_ppc_count[gpc_index]));
			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_div3_m(),
				gr_gpcs_swdx_tc_beta_cb_size_div3_f((cb_size *
					gr->gpc_ppc_count[gpc_index])/3));

			gk20a_writel(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index), val);
		}
	}
}

void gr_gm20b_set_hww_esr_report_mask(struct gk20a *g)
{
	/* setup sm warp esr report masks */
	gk20a_writel(g, gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_r(),
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_stack_error_report_f()	|
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_api_stack_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_ret_empty_stack_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_wrap_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_pc_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_pc_overflow_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_immc_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_reg_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_encoding_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_sph_instr_combo_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_reg_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_oor_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_misaligned_addr_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_addr_space_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_illegal_instr_param2_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_invalid_const_addr_ldc_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_mmu_fault_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_stack_overflow_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_geometry_sm_error_report_f() |
		gr_gpcs_tpcs_sm_hww_warp_esr_report_mask_divergent_report_f());

	/* setup sm global esr report mask */
	gk20a_writel(g, gr_gpcs_tpcs_sm_hww_global_esr_report_mask_r(),
		gr_gpcs_tpcs_sm_hww_global_esr_report_mask_sm_to_sm_fault_report_f() |
		gr_gpcs_tpcs_sm_hww_global_esr_report_mask_multiple_warp_errors_report_f());
}

bool gr_gm20b_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
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

	return valid;
}

bool gr_gm20b_is_valid_gfx_class(struct gk20a *g, u32 class_num)
{
	if (class_num == MAXWELL_B) {
		return true;
	} else {
		return false;
	}
}

bool gr_gm20b_is_valid_compute_class(struct gk20a *g, u32 class_num)
{
	if (class_num == MAXWELL_COMPUTE_B) {
		return true;
	} else {
		return false;
	}
}


/* Following are the blocks of registers that the ucode
 stores in the extended region.*/
/* ==  ctxsw_extended_sm_dsm_perf_counter_register_stride_v() ? */
static const u32 _num_sm_dsm_perf_regs;
/* ==  ctxsw_extended_sm_dsm_perf_counter_control_register_stride_v() ?*/
static const u32 _num_sm_dsm_perf_ctrl_regs = 2;
static u32 *_sm_dsm_perf_regs;
static u32 _sm_dsm_perf_ctrl_regs[2];

void gr_gm20b_init_sm_dsm_reg_info(void)
{
	if (_sm_dsm_perf_ctrl_regs[0] != 0) {
		return;
	}

	_sm_dsm_perf_ctrl_regs[0] =
			      gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control0_r();
	_sm_dsm_perf_ctrl_regs[1] =
			      gr_pri_gpc0_tpc0_sm_dsm_perf_counter_control5_r();
}

void gr_gm20b_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride)
{
	*num_sm_dsm_perf_regs = _num_sm_dsm_perf_regs;
	*sm_dsm_perf_regs = _sm_dsm_perf_regs;
	*perf_register_stride = 0;
}

void gr_gm20b_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride)
{
	*num_sm_dsm_perf_ctrl_regs = _num_sm_dsm_perf_ctrl_regs;
	*sm_dsm_perf_ctrl_regs = _sm_dsm_perf_ctrl_regs;

	*ctrl_register_stride =
	    ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v();
}

u32 gr_gm20b_get_gpc_mask(struct gk20a *g)
{
	u32 val;
	struct gr_gk20a *gr = &g->gr;

	/*
	 * For register NV_FUSE_STATUS_OPT_GPC a set bit with index i indicates
	 * corresponding GPC is floorswept
	 * But for s/w mask a set bit means GPC is enabled and it is disabled
	 * otherwise
	 * Hence toggle the bits of register value to get s/w mask
	 */
	val = g->ops.fuse.fuse_status_opt_gpc(g);

	return (~val) & (BIT32(gr->max_gpc_count) - 1U);
}

u32 gr_gm20b_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	u32 val;
	struct gr_gk20a *gr = &g->gr;

	/* Toggle the bits of NV_FUSE_STATUS_OPT_TPC_GPC */
	val = g->ops.fuse.fuse_status_opt_tpc_gpc(g, gpc_index);

	return (~val) & ((0x1 << gr->max_tpc_per_gpc_count) - 1);
}

void gr_gm20b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	nvgpu_tegra_fuse_write_bypass(g, 0x1);
	nvgpu_tegra_fuse_write_access_sw(g, 0x0);

	if (g->gr.gpc_tpc_mask[gpc_index] == 0x1) {
		nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(g, 0x0);
		nvgpu_tegra_fuse_write_opt_gpu_tpc1_disable(g, 0x1);
	} else if (g->gr.gpc_tpc_mask[gpc_index] == 0x2) {
		nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(g, 0x1);
		nvgpu_tegra_fuse_write_opt_gpu_tpc1_disable(g, 0x0);
	} else {
		nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(g, 0x0);
		nvgpu_tegra_fuse_write_opt_gpu_tpc1_disable(g, 0x0);
	}
}

void gr_gm20b_load_tpc_mask(struct gk20a *g)
{
	u32 pes_tpc_mask = 0, fuse_tpc_mask;
	u32 gpc, pes;
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_TPC_PER_GPC);

	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		for (pes = 0; pes < g->gr.pe_count_per_gpc; pes++) {
			pes_tpc_mask |= g->gr.pes_tpc_mask[pes][gpc] <<
					num_tpc_per_gpc * gpc;
		}
	}

	fuse_tpc_mask = g->ops.gr.get_gpc_tpc_mask(g, 0);
	if (g->tpc_fs_mask_user && g->tpc_fs_mask_user != fuse_tpc_mask &&
		fuse_tpc_mask == (0x1U << g->gr.max_tpc_count) - 1U) {
		u32 val = g->tpc_fs_mask_user;
		val &= (0x1U << g->gr.max_tpc_count) - 1U;
		/* skip tpc to disable the other tpc cause channel timeout */
		val = (0x1U << hweight32(val)) - 1U;
		gk20a_writel(g, gr_fe_tpc_fs_r(), val);
	} else {
		gk20a_writel(g, gr_fe_tpc_fs_r(), pes_tpc_mask);
	}
}

void gr_gm20b_program_sm_id_numbering(struct gk20a *g,
					     u32 gpc, u32 tpc, u32 smid)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 gpc_offset = gpc_stride * gpc;
	u32 tpc_offset = tpc_in_gpc_stride * tpc;

	gk20a_writel(g, gr_gpc0_tpc0_sm_cfg_r() + gpc_offset + tpc_offset,
			gr_gpc0_tpc0_sm_cfg_sm_id_f(smid));
	gk20a_writel(g, gr_gpc0_gpm_pd_sm_id_r(tpc) + gpc_offset,
			gr_gpc0_gpm_pd_sm_id_id_f(smid));
	gk20a_writel(g, gr_gpc0_tpc0_pe_cfg_smid_r() + gpc_offset + tpc_offset,
			gr_gpc0_tpc0_pe_cfg_smid_value_f(smid));
}

int gr_gm20b_load_smid_config(struct gk20a *g)
{
	u32 *tpc_sm_id;
	u32 i, j;
	u32 tpc_index, gpc_index;

	tpc_sm_id = nvgpu_kcalloc(g, gr_cwd_sm_id__size_1_v(), sizeof(u32));
	if (!tpc_sm_id) {
		return -ENOMEM;
	}

	/* Each NV_PGRAPH_PRI_CWD_GPC_TPC_ID can store 4 TPCs.*/
	for (i = 0; i <= ((g->gr.tpc_count-1) / 4); i++) {
		u32 reg = 0;
		u32 bit_stride = gr_cwd_gpc_tpc_id_gpc0_s() +
				 gr_cwd_gpc_tpc_id_tpc0_s();

		for (j = 0; j < 4; j++) {
			u32 sm_id = (i * 4) + j;
			u32 bits;

			if (sm_id >= g->gr.tpc_count) {
				break;
			}

			gpc_index = g->gr.sm_to_cluster[sm_id].gpc_index;
			tpc_index = g->gr.sm_to_cluster[sm_id].tpc_index;

			bits = gr_cwd_gpc_tpc_id_gpc0_f(gpc_index) |
			       gr_cwd_gpc_tpc_id_tpc0_f(tpc_index);
			reg |= bits << (j * bit_stride);

			tpc_sm_id[gpc_index] |= sm_id << tpc_index * bit_stride;
		}
		gk20a_writel(g, gr_cwd_gpc_tpc_id_r(i), reg);
	}

	for (i = 0; i < gr_cwd_sm_id__size_1_v(); i++) {
		gk20a_writel(g, gr_cwd_sm_id_r(i), tpc_sm_id[i]);
	}

	nvgpu_kfree(g, tpc_sm_id);

	return 0;
}

int gr_gm20b_init_fs_state(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	err = gr_gk20a_init_fs_state(g);
	if (err) {
		return err;
	}

	g->ops.gr.load_tpc_mask(g);

	gk20a_writel(g, gr_bes_zrop_settings_r(),
		     gr_bes_zrop_settings_num_active_ltcs_f(g->ltc_count));
	gk20a_writel(g, gr_bes_crop_settings_r(),
		     gr_bes_crop_settings_num_active_ltcs_f(g->ltc_count));

	gk20a_writel(g, gr_bes_crop_debug3_r(),
		     gk20a_readl(g, gr_be0_crop_debug3_r()) |
		     gr_bes_crop_debug3_comp_vdc_4to2_disable_m());

	g->ops.gr.load_smid_config(g);

	return err;
}

int gr_gm20b_load_ctxsw_ucode_segments(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset)
{
	gk20a_writel(g, reg_offset + gr_fecs_dmactl_r(),
			gr_fecs_dmactl_require_ctx_f(0));

	/* Copy falcon bootloader into dmem */
	gr_gk20a_load_ctxsw_ucode_header(g, addr_base, segments, reg_offset);
	gr_gk20a_load_ctxsw_ucode_boot(g, addr_base, segments, reg_offset);

	/* start the falcon immediately if PRIV security is disabled*/
	if (!nvgpu_is_enabled(g, NVGPU_SEC_PRIVSECURITY)) {
		gk20a_writel(g, reg_offset + gr_fecs_cpuctl_r(),
				gr_fecs_cpuctl_startcpu_f(0x01));
	}

	return 0;
}

static bool gr_gm20b_is_tpc_addr_shared(struct gk20a *g, u32 addr)
{
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 tpc_in_gpc_shared_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_SHARED_BASE);
	return (addr >= tpc_in_gpc_shared_base) &&
		(addr < (tpc_in_gpc_shared_base +
			 tpc_in_gpc_stride));
}

bool gr_gm20b_is_tpc_addr(struct gk20a *g, u32 addr)
{
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_TPC_PER_GPC);
	return ((addr >= tpc_in_gpc_base) &&
		(addr < tpc_in_gpc_base +
		 (num_tpc_per_gpc * tpc_in_gpc_stride)))
		|| gr_gm20b_is_tpc_addr_shared(g, addr);
}

u32 gr_gm20b_get_tpc_num(struct gk20a *g, u32 addr)
{
	u32 i, start;
	u32 num_tpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_TPC_PER_GPC);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	for (i = 0; i < num_tpcs; i++) {
		start = tpc_in_gpc_base + (i * tpc_in_gpc_stride);
		if ((addr >= start) &&
		    (addr < (start + tpc_in_gpc_stride))) {
			return i;
		}
	}
	return 0;
}

static void gr_gm20b_load_gpccs_with_bootloader(struct gk20a *g)
{
	struct gk20a_ctxsw_ucode_info *ucode_info = &g->ctxsw_ucode_info;
	u64 addr_base = ucode_info->surface_desc.gpu_va;

	gr_gk20a_load_falcon_bind_instblk(g);

	g->ops.gr.falcon_load_ucode(g, addr_base,
		&g->ctxsw_ucode_info.gpccs,
		gr_gpcs_gpccs_falcon_hwcfg_r() -
		gr_fecs_falcon_hwcfg_r());
}

int gr_gm20b_load_ctxsw_ucode(struct gk20a *g)
{
	u32 err;
	u32 reg_offset = gr_gpcs_gpccs_falcon_hwcfg_r() -
	  gr_fecs_falcon_hwcfg_r();
	u8 falcon_id_mask = 0;

	nvgpu_log_fn(g, " ");

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(7),
			gr_fecs_ctxsw_mailbox_value_f(0xc0de7777));
		gk20a_writel(g, gr_gpccs_ctxsw_mailbox_r(7),
			gr_gpccs_ctxsw_mailbox_value_f(0xc0de7777));
	}

	g->pmu_lsf_loaded_falcon_id = 0;
	if (nvgpu_is_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE)) {
		/* this must be recovery so bootstrap fecs and gpccs */
		if (!nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS)) {
			gr_gm20b_load_gpccs_with_bootloader(g);
			err = g->ops.pmu.load_lsfalcon_ucode(g,
					(1 << LSF_FALCON_ID_FECS));
		} else {
			/* bind WPR VA inst block */
			gr_gk20a_load_falcon_bind_instblk(g);
			err = g->ops.pmu.load_lsfalcon_ucode(g,
				(1 << LSF_FALCON_ID_FECS) |
				(1 << LSF_FALCON_ID_GPCCS));
		}
		if (err) {
			nvgpu_err(g, "Unable to recover GR falcon");
			return err;
		}

	} else {
		/* cold boot or rg exit */
		__nvgpu_set_enabled(g, NVGPU_PMU_FECS_BOOTSTRAP_DONE, true);
		if (!nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS)) {
			gr_gm20b_load_gpccs_with_bootloader(g);
		} else {
			/* bind WPR VA inst block */
			gr_gk20a_load_falcon_bind_instblk(g);
			if (g->ops.pmu.is_lazy_bootstrap(LSF_FALCON_ID_FECS)) {
				falcon_id_mask |= (1 << LSF_FALCON_ID_FECS);
			}
			if (g->ops.pmu.is_lazy_bootstrap(LSF_FALCON_ID_GPCCS)) {
				falcon_id_mask |= (1 << LSF_FALCON_ID_GPCCS);
			}

			err = g->ops.pmu.load_lsfalcon_ucode(g, falcon_id_mask);

			if (err) {
				nvgpu_err(g, "Unable to boot GPCCS");
				return err;
			}
		}
	}

	/*start gpccs */
	if (nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS)) {
		gk20a_writel(g, reg_offset +
			gr_fecs_cpuctl_alias_r(),
			gr_gpccs_cpuctl_startcpu_f(1));
	} else {
		gk20a_writel(g, gr_gpccs_dmactl_r(),
			gr_gpccs_dmactl_require_ctx_f(0));
		gk20a_writel(g, gr_gpccs_cpuctl_r(),
			gr_gpccs_cpuctl_startcpu_f(1));
	}
	/* start fecs */
	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(0), ~0x0);
	gk20a_writel(g, gr_fecs_ctxsw_mailbox_r(1), 0x1);
	gk20a_writel(g, gr_fecs_ctxsw_mailbox_clear_r(6), 0xffffffff);
	gk20a_writel(g, gr_fecs_cpuctl_alias_r(),
			gr_fecs_cpuctl_startcpu_f(1));
	nvgpu_log_fn(g, "done");

	return 0;
}

void gr_gm20b_detect_sm_arch(struct gk20a *g)
{
	u32 v = gk20a_readl(g, gr_gpc0_tpc0_sm_arch_r());

	g->params.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->params.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->params.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

u32 gr_gm20b_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

int gr_gm20b_alloc_gr_ctx(struct gk20a *g,
			  struct nvgpu_gr_ctx *gr_ctx, struct vm_gk20a *vm,
			  u32 class,
			  u32 flags)
{
	int err;

	nvgpu_log_fn(g, " ");

	err = gr_gk20a_alloc_gr_ctx(g, gr_ctx, vm, class, flags);
	if (err) {
		return err;
	}

	if (class == MAXWELL_COMPUTE_B) {
		gr_ctx->compute_preempt_mode = NVGPU_PREEMPTION_MODE_COMPUTE_CTA;
	}

	nvgpu_log_fn(g, "done");

	return 0;
}

void gr_gm20b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_gk20a *c,
		struct nvgpu_mem *mem)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	u32 cta_preempt_option =
		ctxsw_prog_main_image_preemption_options_control_cta_enabled_f();

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg) {
		return;
	}

	gr_ctx = &tsg->gr_ctx;
	if (gr_ctx->compute_preempt_mode == NVGPU_PREEMPTION_MODE_COMPUTE_CTA) {
		nvgpu_log_info(g, "CTA: %x", cta_preempt_option);
		nvgpu_mem_wr(g, mem,
				ctxsw_prog_main_image_preemption_options_o(),
				cta_preempt_option);
	}

	nvgpu_log_fn(g, "done");
}

int gr_gm20b_dump_gr_status_regs(struct gk20a *g,
			   struct gk20a_debug_output *o)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gr_engine_id;

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);

	gk20a_debug_output(o, "NV_PGRAPH_STATUS: 0x%x\n",
		gk20a_readl(g, gr_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_STATUS1: 0x%x\n",
		gk20a_readl(g, gr_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_STATUS2: 0x%x\n",
		gk20a_readl(g, gr_status_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ENGINE_STATUS: 0x%x\n",
		gk20a_readl(g, gr_engine_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_GRFIFO_STATUS : 0x%x\n",
		gk20a_readl(g, gr_gpfifo_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_GRFIFO_CONTROL : 0x%x\n",
		gk20a_readl(g, gr_gpfifo_ctl_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_STATUS : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_EXCEPTION  : 0x%x\n",
		gk20a_readl(g, gr_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_FECS_INTR  : 0x%x\n",
		gk20a_readl(g, gr_fecs_intr_r()));
	gk20a_debug_output(o, "NV_PFIFO_ENGINE_STATUS(GR) : 0x%x\n",
		gk20a_readl(g, fifo_engine_status_r(gr_engine_id)));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_activity_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_activity_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_ACTIVITY4: 0x%x\n",
		gk20a_readl(g, gr_activity_4_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_SKED_ACTIVITY: 0x%x\n",
		gk20a_readl(g, gr_pri_sked_activity_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_ACTIVITY3: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_activity3_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_activity_0_r()));
	if (gr->gpc_tpc_count && gr->gpc_tpc_count[0] == 2) {
		gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC1_TPCCS_TPC_ACTIVITY0: 0x%x\n",
			gk20a_readl(g, gr_pri_gpc0_tpc1_tpccs_tpc_activity_0_r()));
	}
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPCS_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpcs_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY1: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY2: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_GPCCS_GPC_ACTIVITY3: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_gpccs_gpc_activity_3_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPC0_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_tpc0_tpccs_tpc_activity_0_r()));
	if (gr->gpc_tpc_count && gr->gpc_tpc_count[0] == 2) {
		gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPC1_TPCCS_TPC_ACTIVITY0: 0x%x\n",
			gk20a_readl(g, gr_pri_gpcs_tpc1_tpccs_tpc_activity_0_r()));
	}
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPCS_TPCS_TPCCS_TPC_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_gpcs_tpcs_tpccs_tpc_activity_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE1_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_be1_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_BECS_BE_ACTIVITY0: 0x%x\n",
		gk20a_readl(g, gr_pri_bes_becs_be_activity0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_DS_MPIPE_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_ds_mpipe_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_ON_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_on_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_TIMEOUT : 0x%x\n",
		gk20a_readl(g, gr_fe_go_idle_timeout_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_CHECK : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_check_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_INFO : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_info_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TEX_M_TEX_SUBUNITS_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tex_m_tex_subunits_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_FS: 0x%x\n",
		gk20a_readl(g, gr_cwd_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_TPC_FS: 0x%x\n",
		gk20a_readl(g, gr_fe_tpc_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_GPC_TPC_ID(0): 0x%x\n",
		gk20a_readl(g, gr_cwd_gpc_tpc_id_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_SM_ID(0): 0x%x\n",
		gk20a_readl(g, gr_cwd_sm_id_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_STATUS_FE_0: 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_status_fe_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_STATUS_1: 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_STATUS_GPC_0: 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_status_gpc_0_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_STATUS_1: 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_status_1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CTXSW_IDLESTATE : 0x%x\n",
		gk20a_readl(g, gr_fecs_ctxsw_idlestate_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_CTXSW_IDLESTATE : 0x%x\n",
		gk20a_readl(g, gr_gpc0_gpccs_ctxsw_idlestate_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_CURRENT_CTX : 0x%x\n",
		gk20a_readl(g, gr_fecs_current_ctx_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_NEW_CTX : 0x%x\n",
		gk20a_readl(g, gr_fecs_new_ctx_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_CROP_STATUS1 : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_crop_status1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_CROP_STATUS1 : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_crop_status1_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_ZROP_STATUS : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_zrop_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_ZROP_STATUS2 : 0x%x\n",
		gk20a_readl(g, gr_pri_be0_zrop_status2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_ZROP_STATUS : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_zrop_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BES_ZROP_STATUS2 : 0x%x\n",
		gk20a_readl(g, gr_pri_bes_zrop_status2_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_BE0_BECS_BE_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_be0_becs_be_exception_en_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_GPCCS_GPC_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_gpccs_gpc_exception_en_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_EXCEPTION: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_exception_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TPCCS_TPC_EXCEPTION_EN: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tpccs_tpc_exception_en_r()));

	return 0;
}

int gr_gm20b_update_pc_sampling(struct channel_gk20a *c,
				       bool enable)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	struct nvgpu_mem *mem;
	u32 v;

	nvgpu_log_fn(c->g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;
	mem = &gr_ctx->mem;
	if (!nvgpu_mem_is_valid(mem) || c->vpr) {
		return -EINVAL;
	}


	v = nvgpu_mem_rd(c->g, mem, ctxsw_prog_main_image_pm_o());
	v &= ~ctxsw_prog_main_image_pm_pc_sampling_m();
	v |= ctxsw_prog_main_image_pm_pc_sampling_f(enable);
	nvgpu_mem_wr(c->g, mem, ctxsw_prog_main_image_pm_o(), v);

	nvgpu_log_fn(c->g, "done");

	return 0;
}

u32 gr_gm20b_get_fbp_en_mask(struct gk20a *g)
{
	u32 fbp_en_mask;
	u32 tmp, max_fbps_count;

	tmp = gk20a_readl(g, top_num_fbps_r());
	max_fbps_count = top_num_fbps_value_v(tmp);

	/*
	 * Read active fbp mask from fuse
	 * Note that 0:enable and 1:disable in value read from fuse so we've to
	 * flip the bits.
	 * Also set unused bits to zero
	 */
	fbp_en_mask = g->ops.fuse.fuse_status_opt_fbp(g);
	fbp_en_mask = ~fbp_en_mask;
	fbp_en_mask = fbp_en_mask & ((1 << max_fbps_count) - 1);

	return fbp_en_mask;
}

u32 gr_gm20b_get_max_ltc_per_fbp(struct gk20a *g)
{
	u32 ltc_per_fbp, reg;
	reg = gk20a_readl(g,  top_ltc_per_fbp_r());
	ltc_per_fbp = top_ltc_per_fbp_value_v(reg);
	return ltc_per_fbp;
}

u32 gr_gm20b_get_max_lts_per_ltc(struct gk20a *g)
{
	u32 lts_per_ltc, reg;
	reg = gk20a_readl(g,  top_slices_per_ltc_r());
	lts_per_ltc = top_slices_per_ltc_value_v(reg);
	return lts_per_ltc;
}

u32 *gr_gm20b_rop_l2_en_mask(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 i, tmp, max_fbps_count, max_ltc_per_fbp;
	unsigned long fbp_en_mask;
	u32 rop_l2_all_en;

	tmp = gk20a_readl(g, top_num_fbps_r());
	max_fbps_count = top_num_fbps_value_v(tmp);
	max_ltc_per_fbp = gr_gm20b_get_max_ltc_per_fbp(g);
	rop_l2_all_en = (1 << max_ltc_per_fbp) - 1;
	fbp_en_mask = gr_gm20b_get_fbp_en_mask(g);

	/* mask of Rop_L2 for each FBP */
	for_each_set_bit(i, &fbp_en_mask, max_fbps_count) {
		tmp = g->ops.fuse.fuse_status_opt_rop_l2_fbp(g, i);
		gr->fbp_rop_l2_en_mask[i] = rop_l2_all_en ^ tmp;
	}

	return gr->fbp_rop_l2_en_mask;
}

u32 gr_gm20b_get_max_fbps_count(struct gk20a *g)
{
	u32 tmp, max_fbps_count;
	tmp = gk20a_readl(g, top_num_fbps_r());
	max_fbps_count = top_num_fbps_value_v(tmp);
	return max_fbps_count;
}

void gr_gm20b_init_cyclestats(struct gk20a *g)
{
#if defined(CONFIG_GK20A_CYCLE_STATS)
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS, true);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_CYCLE_STATS_SNAPSHOT, true);
	g->gr.max_css_buffer_size = 0xffffffff;
#else
	(void)g;
#endif
}

void gr_gm20b_enable_cde_in_fecs(struct gk20a *g, struct nvgpu_mem *mem)
{
	u32 cde_v;

	cde_v = nvgpu_mem_rd(g, mem, ctxsw_prog_main_image_ctl_o());
	cde_v |=  ctxsw_prog_main_image_ctl_cde_enabled_f();
	nvgpu_mem_wr(g, mem, ctxsw_prog_main_image_ctl_o(), cde_v);
}

void gr_gm20b_bpt_reg_info(struct gk20a *g, struct nvgpu_warpstate *w_state)
{
	/* Check if we have at least one valid warp */
	/* get paused state on maxwell */
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm_id;
	u32  tpc_offset, gpc_offset, reg_offset;
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);

	/* for maxwell & kepler */
	u32 numSmPerTpc = 1;
	u32 numWarpPerTpc = g->params.sm_arch_warp_count * numSmPerTpc;

	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {
		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;

		tpc_offset = tpc_in_gpc_stride * tpc;
		gpc_offset = gpc_stride * gpc;
		reg_offset = tpc_offset + gpc_offset;

		/* 64 bit read */
		warps_valid = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_r() + reg_offset + 4) << 32;
		warps_valid |= gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_r() + reg_offset);

		/* 64 bit read */
		warps_paused = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_r() + reg_offset + 4) << 32;
		warps_paused |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_r() + reg_offset);

		/* 64 bit read */
		warps_trapped = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_r() + reg_offset + 4) << 32;
		warps_trapped |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_r() + reg_offset);

		w_state[sm_id].valid_warps[0] = warps_valid;
		w_state[sm_id].trapped_warps[0] = warps_trapped;
		w_state[sm_id].paused_warps[0] = warps_paused;


		if (numWarpPerTpc > 64) {
			/* 64 bit read */
			warps_valid = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_2_r() + reg_offset + 4) << 32;
			warps_valid |= gk20a_readl(g, gr_gpc0_tpc0_sm_warp_valid_mask_2_r() + reg_offset);

			/* 64 bit read */
			warps_paused = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_2_r() + reg_offset + 4) << 32;
			warps_paused |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_pause_mask_2_r() + reg_offset);

			/* 64 bit read */
			warps_trapped = (u64)gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_2_r() + reg_offset + 4) << 32;
			warps_trapped |= gk20a_readl(g, gr_gpc0_tpc0_sm_dbgr_bpt_trap_mask_2_r() + reg_offset);

			w_state[sm_id].valid_warps[1] = warps_valid;
			w_state[sm_id].trapped_warps[1] = warps_trapped;
			w_state[sm_id].paused_warps[1] = warps_paused;
		}
	}


	/* Only for debug purpose */
	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {
		nvgpu_log_fn(g, "w_state[%d].valid_warps[0]: %llx\n",
						sm_id, w_state[sm_id].valid_warps[0]);
		nvgpu_log_fn(g, "w_state[%d].valid_warps[1]: %llx\n",
						sm_id, w_state[sm_id].valid_warps[1]);

		nvgpu_log_fn(g, "w_state[%d].trapped_warps[0]: %llx\n",
							sm_id, w_state[sm_id].trapped_warps[0]);
		nvgpu_log_fn(g, "w_state[%d].trapped_warps[1]: %llx\n",
						sm_id, w_state[sm_id].trapped_warps[1]);

		nvgpu_log_fn(g, "w_state[%d].paused_warps[0]: %llx\n",
						sm_id, w_state[sm_id].paused_warps[0]);
		nvgpu_log_fn(g, "w_state[%d].paused_warps[1]: %llx\n",
						sm_id, w_state[sm_id].paused_warps[1]);
	}
}

void gr_gm20b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries)
{
	static u32 wl_addr_gm20b[] = {
		/* this list must be sorted (low to high) */
		0x404468, /* gr_pri_mme_max_instructions       */
		0x418300, /* gr_pri_gpcs_rasterarb_line_class  */
		0x418800, /* gr_pri_gpcs_setup_debug           */
		0x418e00, /* gr_pri_gpcs_swdx_config           */
		0x418e40, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e44, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e48, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e4c, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e50, /* gr_pri_gpcs_swdx_tc_bundle_ctrl   */
		0x418e58, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e5c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e60, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e64, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e68, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e6c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e70, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e74, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e78, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e7c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e80, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e84, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e88, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e8c, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e90, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x418e94, /* gr_pri_gpcs_swdx_tc_bundle_addr   */
		0x419864, /* gr_pri_gpcs_tpcs_pe_l2_evict_policy */
		0x419a04, /* gr_pri_gpcs_tpcs_tex_lod_dbg      */
		0x419a08, /* gr_pri_gpcs_tpcs_tex_samp_dbg     */
		0x419e10, /* gr_pri_gpcs_tpcs_sm_dbgr_control0 */
		0x419f78, /* gr_pri_gpcs_tpcs_sm_disp_ctrl     */
	};

	*whitelist = wl_addr_gm20b;
	*num_entries = ARRAY_SIZE(wl_addr_gm20b);
}

static void gm20b_gr_read_sm_error_state(struct gk20a *g,
			u32 offset,
			struct nvgpu_tsg_sm_error_state *sm_error_states)
{
	sm_error_states->hww_global_esr = gk20a_readl(g,
			gr_gpc0_tpc0_sm_hww_global_esr_r() + offset);
	sm_error_states->hww_warp_esr = gk20a_readl(g,
			gr_gpc0_tpc0_sm_hww_warp_esr_r() + offset);
	sm_error_states->hww_warp_esr_pc = (u64)(gk20a_readl(g,
			gr_gpc0_tpc0_sm_hww_warp_esr_pc_r() + offset));
	sm_error_states->hww_global_esr_report_mask = gk20a_readl(g,
		       gr_gpc0_tpc0_sm_hww_global_esr_report_mask_r() + offset);
	sm_error_states->hww_warp_esr_report_mask = gk20a_readl(g,
			gr_gpc0_tpc0_sm_hww_warp_esr_report_mask_r() + offset);

}

int gm20b_gr_record_sm_error_state(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
				struct channel_gk20a *fault_ch)
{
	int sm_id;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					       GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	struct nvgpu_tsg_sm_error_state *sm_error_states = NULL;
	struct tsg_gk20a *tsg = NULL;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	sm_id = gr_gpc0_tpc0_sm_cfg_sm_id_v(gk20a_readl(g,
			gr_gpc0_tpc0_sm_cfg_r() + offset));

	if (fault_ch != NULL) {
		tsg = tsg_gk20a_from_ch(fault_ch);
	}

	if (tsg == NULL) {
		nvgpu_err(g, "no valid tsg");
		goto record_fail;
	}

	sm_error_states = tsg->sm_error_states + sm_id;
	gm20b_gr_read_sm_error_state(g, offset, sm_error_states);

record_fail:
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return sm_id;
}

int gm20b_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id)
{
	u32 gpc, tpc, offset;
	u32 val;
	struct tsg_gk20a *tsg;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					       GPU_LIT_TPC_IN_GPC_STRIDE);
	int err = 0;

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	memset(&tsg->sm_error_states[sm_id], 0, sizeof(*tsg->sm_error_states));

	err = gr_gk20a_disable_ctxsw(g);
	if (err) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		goto fail;
	}

	if (gk20a_is_channel_ctx_resident(ch)) {
		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;

		offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;

		val = gk20a_readl(g, gr_gpc0_tpc0_sm_hww_global_esr_r() + offset);
		gk20a_writel(g, gr_gpc0_tpc0_sm_hww_global_esr_r() + offset,
				val);
		gk20a_writel(g, gr_gpc0_tpc0_sm_hww_warp_esr_r() + offset,
				0);
	}

	err = gr_gk20a_enable_ctxsw(g);

fail:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return err;
}

int gr_gm20b_get_preemption_mode_flags(struct gk20a *g,
		struct nvgpu_preemption_modes_rec *preemption_modes_rec)
{
	preemption_modes_rec->graphics_preemption_mode_flags =
			NVGPU_PREEMPTION_MODE_GRAPHICS_WFI;
	preemption_modes_rec->compute_preemption_mode_flags = (
			NVGPU_PREEMPTION_MODE_COMPUTE_WFI |
			NVGPU_PREEMPTION_MODE_COMPUTE_CTA);

	preemption_modes_rec->default_graphics_preempt_mode =
			NVGPU_PREEMPTION_MODE_GRAPHICS_WFI;
	preemption_modes_rec->default_compute_preempt_mode =
			NVGPU_PREEMPTION_MODE_COMPUTE_CTA;

	return 0;
}

void gm20b_gr_clear_sm_hww(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
			u32 global_esr)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	gk20a_writel(g, gr_gpc0_tpc0_sm_hww_global_esr_r() + offset,
			global_esr);

	/* clear the warp hww */
	gk20a_writel(g, gr_gpc0_tpc0_sm_hww_warp_esr_r() + offset, 0);
}

/*
 * Disable both surface and LG coalesce.
 */
void gm20a_gr_disable_rd_coalesce(struct gk20a *g)
{
	u32 dbg2_reg;

	dbg2_reg = gk20a_readl(g, gr_gpcs_tpcs_tex_m_dbg2_r());
	dbg2_reg = set_field(dbg2_reg,
			     gr_gpcs_tpcs_tex_m_dbg2_lg_rd_coalesce_en_m(),
			     gr_gpcs_tpcs_tex_m_dbg2_lg_rd_coalesce_en_f(0));
	dbg2_reg = set_field(dbg2_reg,
			     gr_gpcs_tpcs_tex_m_dbg2_su_rd_coalesce_en_m(),
			     gr_gpcs_tpcs_tex_m_dbg2_su_rd_coalesce_en_f(0));

	gk20a_writel(g, gr_gpcs_tpcs_tex_m_dbg2_r(), dbg2_reg);
}

u32 gr_gm20b_get_pmm_per_chiplet_offset(void)
{
	return (perf_pmmsys_extent_v() - perf_pmmsys_base_v() + 1);
}

int gm20b_gr_set_mmu_debug_mode(struct gk20a *g,
		struct channel_gk20a *ch, bool enable)
{
	struct nvgpu_dbg_reg_op ctx_ops = {
		.op = REGOP(WRITE_32),
		.type = REGOP(TYPE_GR_CTX),
		.offset = gr_gpcs_pri_mmu_debug_ctrl_r(),
		.value_lo = enable ?
			gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_f() :
			gr_gpcs_pri_mmu_debug_ctrl_debug_disabled_f(),
	};
	int err;
	struct tsg_gk20a *tsg = tsg_gk20a_from_ch(ch);

	if (tsg == NULL) {
		return enable ? -EINVAL : 0;
	}

	err = gr_gk20a_exec_ctx_ops(ch, &ctx_ops, 1, 1, 0, NULL);
	if (err != 0) {
		nvgpu_err(g, "update MMU debug mode failed");
	}
	return err;
}

void gm20b_gr_set_debug_mode(struct gk20a *g, bool enable)
{
	u32 reg_val, gpc_debug_ctrl;

	if (enable) {
		gpc_debug_ctrl = gr_gpcs_pri_mmu_debug_ctrl_debug_enabled_f();
	} else {
		gpc_debug_ctrl = gr_gpcs_pri_mmu_debug_ctrl_debug_disabled_f();
	}

	reg_val = gk20a_readl(g, gr_gpcs_pri_mmu_debug_ctrl_r());
	reg_val = set_field(reg_val,
			gr_gpcs_pri_mmu_debug_ctrl_debug_m(), gpc_debug_ctrl);
	gk20a_writel(g, gr_gpcs_pri_mmu_debug_ctrl_r(), reg_val);
}
