/*
 * GV11b GPU GR
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

#include <nvgpu/timers.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/debug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/fuse.h>
#include <nvgpu/bug.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/soc.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/bitops.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include "gk20a/gr_gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "gk20a/gr_pri_gk20a.h"

#include "gm20b/gr_gm20b.h"

#include "gp10b/gr_gp10b.h"

#include "gv11b/gr_gv11b.h"
#include "gv11b/mm_gv11b.h"
#include "gv11b/subctx_gv11b.h"
#include "gv11b/gv11b.h"
#include "gv11b/gr_pri_gv11b.h"

#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_proj_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_perf_gv11b.h>

#define GFXP_WFI_TIMEOUT_COUNT_IN_USEC_DEFAULT 100

/* ecc scrubbing will done in 1 pri read cycle,but for safety used 10 retries */
#define ECC_SCRUBBING_TIMEOUT_MAX 1000
#define ECC_SCRUBBING_TIMEOUT_DEFAULT 10

/*
 * Each gpc can have maximum 32 tpcs, so each tpc index need
 * 5 bits. Each map register(32bits) can hold 6 tpcs info.
 */
#define GR_TPCS_INFO_FOR_MAPREGISTER 6

bool gr_gv11b_is_valid_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_COMPUTE_A:
	case VOLTA_A:
	case VOLTA_DMA_COPY_A:
		valid = true;
		break;

	case MAXWELL_COMPUTE_B:
	case MAXWELL_B:
	case FERMI_TWOD_A:
	case KEPLER_DMA_COPY_A:
	case MAXWELL_DMA_COPY_A:
	case PASCAL_COMPUTE_A:
	case PASCAL_A:
	case PASCAL_DMA_COPY_A:
		valid = true;
		break;

	default:
		break;
	}
	nvgpu_log_info(g, "class=0x%x valid=%d", class_num, valid);
	return valid;
}

bool gr_gv11b_is_valid_gfx_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_A:
	case PASCAL_A:
	case MAXWELL_B:
		valid = true;
		break;

	default:
		break;
	}
	return valid;
}

void gr_gv11b_powergate_tpc(struct gk20a *g)
{
	u32 tpc_pg_status = g->ops.fuse.fuse_status_opt_tpc_gpc(g, 0);

	if (tpc_pg_status == g->tpc_pg_mask) {
		return;
	}

	g->ops.fuse.fuse_ctrl_opt_tpc_gpc(g, 0, g->tpc_pg_mask);

	do {
		tpc_pg_status = g->ops.fuse.fuse_status_opt_tpc_gpc(g, 0);
	} while (tpc_pg_status != g->tpc_pg_mask);

	return;
}

bool gr_gv11b_is_valid_compute_class(struct gk20a *g, u32 class_num)
{
	bool valid = false;

	switch (class_num) {
	case VOLTA_COMPUTE_A:
	case PASCAL_COMPUTE_A:
	case MAXWELL_COMPUTE_B:
		valid = true;
		break;

	default:
		break;
	}
	return valid;
}

u32 gv11b_gr_sm_offset(struct gk20a *g, u32 sm)
{

	u32 sm_pri_stride = nvgpu_get_litter_value(g, GPU_LIT_SM_PRI_STRIDE);
	u32 sm_offset = sm_pri_stride * sm;

	return sm_offset;
}

static int gr_gv11b_handle_l1_tag_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 l1_tag_ecc_status, l1_tag_ecc_corrected_err_status = 0;
	u32 l1_tag_ecc_uncorrected_err_status = 0;
	u32 l1_tag_corrected_err_count_delta = 0;
	u32 l1_tag_uncorrected_err_count_delta = 0;
	bool is_l1_tag_ecc_corrected_total_err_overflow = 0;
	bool is_l1_tag_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L1 tag ECC errors. */
	l1_tag_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_r() + offset);
	l1_tag_ecc_corrected_err_status = l1_tag_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_el1_1_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_pixrpf_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_miss_fifo_m());
	l1_tag_ecc_uncorrected_err_status = l1_tag_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_el1_1_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_pixrpf_m() |
		 gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_miss_fifo_m());

	if ((l1_tag_ecc_corrected_err_status == 0) && (l1_tag_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	l1_tag_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_r() +
				offset));
	l1_tag_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_r() +
				offset));
	is_l1_tag_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_corrected_err_total_counter_overflow_v(l1_tag_ecc_status);
	is_l1_tag_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_uncorrected_err_total_counter_overflow_v(l1_tag_ecc_status);

	if ((l1_tag_corrected_err_count_delta > 0) || is_l1_tag_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L1 tag! err_mask [%08x] is_overf [%d]",
			l1_tag_ecc_corrected_err_status, is_l1_tag_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_l1_tag_ecc_corrected_total_err_overflow) {
			l1_tag_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.sm_l1_tag_ecc_corrected_err_count[gpc][tpc].counter +=
							l1_tag_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((l1_tag_uncorrected_err_count_delta > 0) || is_l1_tag_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L1 tag! err_mask [%08x] is_overf [%d]",
			l1_tag_ecc_uncorrected_err_status, is_l1_tag_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_l1_tag_ecc_uncorrected_total_err_overflow) {
			l1_tag_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.sm_l1_tag_ecc_uncorrected_err_count[gpc][tpc].counter +=
							l1_tag_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_l1_tag_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_lrf_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 lrf_ecc_status, lrf_ecc_corrected_err_status = 0;
	u32 lrf_ecc_uncorrected_err_status = 0;
	u32 lrf_corrected_err_count_delta = 0;
	u32 lrf_uncorrected_err_count_delta = 0;
	bool is_lrf_ecc_corrected_total_err_overflow = 0;
	bool is_lrf_ecc_uncorrected_total_err_overflow = 0;

	/* Check for LRF ECC errors. */
	lrf_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset);
	lrf_ecc_corrected_err_status = lrf_ecc_status &
		(gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp0_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp1_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp2_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp3_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp4_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp5_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp6_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_qrfdp7_m());
	lrf_ecc_uncorrected_err_status = lrf_ecc_status &
		(gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp0_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp1_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp2_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp3_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp4_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp5_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp6_m() |
		 gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_qrfdp7_m());

	if ((lrf_ecc_corrected_err_status == 0) && (lrf_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	lrf_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_r() +
				offset));
	lrf_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_r() +
				offset));
	is_lrf_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_corrected_err_total_counter_overflow_v(lrf_ecc_status);
	is_lrf_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_lrf_ecc_status_uncorrected_err_total_counter_overflow_v(lrf_ecc_status);

	if ((lrf_corrected_err_count_delta > 0) || is_lrf_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM LRF! err_mask [%08x] is_overf [%d]",
			lrf_ecc_corrected_err_status, is_lrf_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_lrf_ecc_corrected_total_err_overflow) {
			lrf_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.sm_lrf_ecc_single_err_count[gpc][tpc].counter +=
							lrf_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((lrf_uncorrected_err_count_delta > 0) || is_lrf_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM LRF! err_mask [%08x] is_overf [%d]",
			lrf_ecc_uncorrected_err_status, is_lrf_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_lrf_ecc_uncorrected_total_err_overflow) {
			lrf_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.sm_lrf_ecc_double_err_count[gpc][tpc].counter +=
							lrf_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_lrf_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_lrf_ecc_status_reset_task_f());

	return 0;

}

void gr_gv11b_enable_hww_exceptions(struct gk20a *g)
{
	/* enable exceptions */

	gk20a_writel(g, gr_fe_hww_esr_r(),
		     gr_fe_hww_esr_en_enable_f() |
		     gr_fe_hww_esr_reset_active_f());
	gk20a_writel(g, gr_memfmt_hww_esr_r(),
		     gr_memfmt_hww_esr_en_enable_f() |
		     gr_memfmt_hww_esr_reset_active_f());
	gk20a_writel(g, gr_pd_hww_esr_r(),
		     gr_pd_hww_esr_en_enable_f() |
		     gr_pd_hww_esr_reset_active_f());
	gk20a_writel(g, gr_scc_hww_esr_r(),
		     gr_scc_hww_esr_en_enable_f() |
		     gr_scc_hww_esr_reset_active_f());
	gk20a_writel(g, gr_ds_hww_esr_r(),
		     gr_ds_hww_esr_en_enabled_f() |
		     gr_ds_hww_esr_reset_task_f());
	gk20a_writel(g, gr_ssync_hww_esr_r(),
		     gr_ssync_hww_esr_en_enable_f() |
		     gr_ssync_hww_esr_reset_active_f());
	gk20a_writel(g, gr_mme_hww_esr_r(),
		     gr_mme_hww_esr_en_enable_f() |
		     gr_mme_hww_esr_reset_active_f());

	/* For now leave POR values */
	nvgpu_log(g, gpu_dbg_info, "gr_sked_hww_esr_en_r 0x%08x",
			gk20a_readl(g, gr_sked_hww_esr_en_r()));
}

void gr_gv11b_fecs_host_int_enable(struct gk20a *g)
{
	gk20a_writel(g, gr_fecs_host_int_enable_r(),
		     gr_fecs_host_int_enable_ctxsw_intr1_enable_f() |
		     gr_fecs_host_int_enable_fault_during_ctxsw_enable_f() |
		     gr_fecs_host_int_enable_umimp_firmware_method_enable_f() |
		     gr_fecs_host_int_enable_umimp_illegal_method_enable_f() |
		     gr_fecs_host_int_enable_watchdog_enable_f() |
		     gr_fecs_host_int_enable_flush_when_busy_enable_f() |
		     gr_fecs_host_int_enable_ecc_corrected_enable_f() |
		     gr_fecs_host_int_enable_ecc_uncorrected_enable_f());
}

void gr_gv11b_enable_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 reg_val;

	/*
	 * clear exceptions :
	 * other than SM : hww_esr are reset in *enable_hww_excetpions*
	 * SM            : cleared in *set_hww_esr_report_mask*
	 */

	/* enable exceptions */
	gk20a_writel(g, gr_exception2_en_r(), 0x0); /* BE not enabled */
	gk20a_writel(g, gr_exception1_en_r(), (1 << gr->gpc_count) - 1);

	reg_val = gr_exception_en_fe_enabled_f() |
			gr_exception_en_memfmt_enabled_f() |
			gr_exception_en_pd_enabled_f() |
			gr_exception_en_scc_enabled_f() |
			gr_exception_en_ds_enabled_f() |
			gr_exception_en_ssync_enabled_f() |
			gr_exception_en_mme_enabled_f() |
			gr_exception_en_sked_enabled_f() |
			gr_exception_en_gpc_enabled_f();

	nvgpu_log(g, gpu_dbg_info, "gr_exception_en 0x%08x", reg_val);

	gk20a_writel(g, gr_exception_en_r(), reg_val);

}

static int gr_gv11b_handle_cbu_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 cbu_ecc_status, cbu_ecc_corrected_err_status = 0;
	u32 cbu_ecc_uncorrected_err_status = 0;
	u32 cbu_corrected_err_count_delta = 0;
	u32 cbu_uncorrected_err_count_delta = 0;
	bool is_cbu_ecc_corrected_total_err_overflow = 0;
	bool is_cbu_ecc_uncorrected_total_err_overflow = 0;

	/* Check for CBU ECC errors. */
	cbu_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_r() + offset);
	cbu_ecc_corrected_err_status = cbu_ecc_status &
		(gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_warp_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_warp_sm1_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_barrier_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_barrier_sm1_m());
	cbu_ecc_uncorrected_err_status = cbu_ecc_status &
		(gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_warp_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_warp_sm1_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_barrier_sm0_m() |
		 gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_barrier_sm1_m());

	if ((cbu_ecc_corrected_err_status == 0) && (cbu_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	cbu_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_r() +
				offset));
	cbu_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_r() +
				offset));
	is_cbu_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_corrected_err_total_counter_overflow_v(cbu_ecc_status);
	is_cbu_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_cbu_ecc_status_uncorrected_err_total_counter_overflow_v(cbu_ecc_status);

	if ((cbu_corrected_err_count_delta > 0) || is_cbu_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM CBU! err_mask [%08x] is_overf [%d]",
			cbu_ecc_corrected_err_status, is_cbu_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_cbu_ecc_corrected_total_err_overflow) {
			cbu_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.sm_cbu_ecc_corrected_err_count[gpc][tpc].counter +=
							cbu_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((cbu_uncorrected_err_count_delta > 0) || is_cbu_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM CBU! err_mask [%08x] is_overf [%d]",
			cbu_ecc_uncorrected_err_status, is_cbu_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_cbu_ecc_uncorrected_total_err_overflow) {
			cbu_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.sm_cbu_ecc_uncorrected_err_count[gpc][tpc].counter +=
							cbu_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_cbu_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_cbu_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_l1_data_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 l1_data_ecc_status, l1_data_ecc_corrected_err_status = 0;
	u32 l1_data_ecc_uncorrected_err_status = 0;
	u32 l1_data_corrected_err_count_delta = 0;
	u32 l1_data_uncorrected_err_count_delta = 0;
	bool is_l1_data_ecc_corrected_total_err_overflow = 0;
	bool is_l1_data_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L1 data ECC errors. */
	l1_data_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_r() + offset);
	l1_data_ecc_corrected_err_status = l1_data_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_el1_1_m());
	l1_data_ecc_uncorrected_err_status = l1_data_ecc_status &
		(gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_el1_0_m() |
		 gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_el1_1_m());

	if ((l1_data_ecc_corrected_err_status == 0) && (l1_data_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	l1_data_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_r() +
				offset));
	l1_data_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_r() +
				offset));
	is_l1_data_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_corrected_err_total_counter_overflow_v(l1_data_ecc_status);
	is_l1_data_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_uncorrected_err_total_counter_overflow_v(l1_data_ecc_status);

	if ((l1_data_corrected_err_count_delta > 0) || is_l1_data_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L1 data! err_mask [%08x] is_overf [%d]",
			l1_data_ecc_corrected_err_status, is_l1_data_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_l1_data_ecc_corrected_total_err_overflow) {
			l1_data_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.sm_l1_data_ecc_corrected_err_count[gpc][tpc].counter +=
							l1_data_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((l1_data_uncorrected_err_count_delta > 0) || is_l1_data_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L1 data! err_mask [%08x] is_overf [%d]",
			l1_data_ecc_uncorrected_err_status, is_l1_data_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_l1_data_ecc_uncorrected_total_err_overflow) {
			l1_data_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.sm_l1_data_ecc_uncorrected_err_count[gpc][tpc].counter +=
							l1_data_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_l1_data_ecc_status_reset_task_f());

	return 0;

}

static int gr_gv11b_handle_icache_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 offset = gpc_stride * gpc + tpc_in_gpc_stride * tpc;
	u32 icache_ecc_status, icache_ecc_corrected_err_status = 0;
	u32 icache_ecc_uncorrected_err_status = 0;
	u32 icache_corrected_err_count_delta = 0;
	u32 icache_uncorrected_err_count_delta = 0;
	bool is_icache_ecc_corrected_total_err_overflow = 0;
	bool is_icache_ecc_uncorrected_total_err_overflow = 0;

	/* Check for L0 && L1 icache ECC errors. */
	icache_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_r() + offset);
	icache_ecc_corrected_err_status = icache_ecc_status &
		(gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l0_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l0_predecode_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l1_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_l1_predecode_m());
	icache_ecc_uncorrected_err_status = icache_ecc_status &
		(gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l0_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l0_predecode_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l1_data_m() |
		 gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_l1_predecode_m());

	if ((icache_ecc_corrected_err_status == 0) && (icache_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	icache_corrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_r() +
				offset));
	icache_uncorrected_err_count_delta =
		gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_r() +
				offset));
	is_icache_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_corrected_err_total_counter_overflow_v(icache_ecc_status);
	is_icache_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_tpc0_sm_icache_ecc_status_uncorrected_err_total_counter_overflow_v(icache_ecc_status);

	if ((icache_corrected_err_count_delta > 0) || is_icache_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in SM L0 && L1 icache! err_mask [%08x] is_overf [%d]",
			icache_ecc_corrected_err_status, is_icache_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_icache_ecc_corrected_total_err_overflow) {
			icache_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.sm_icache_ecc_corrected_err_count[gpc][tpc].counter +=
							icache_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_icache_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((icache_uncorrected_err_count_delta > 0) || is_icache_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in SM L0 && L1 icache! err_mask [%08x] is_overf [%d]",
			icache_ecc_uncorrected_err_status, is_icache_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_icache_ecc_uncorrected_total_err_overflow) {
			icache_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.sm_icache_ecc_uncorrected_err_count[gpc][tpc].counter +=
							icache_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_tpc0_sm_icache_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_tpc0_sm_icache_ecc_status_r() + offset,
			gr_pri_gpc0_tpc0_sm_icache_ecc_status_reset_task_f());

	return 0;

}

int gr_gv11b_handle_tpc_sm_ecc_exception(struct gk20a *g,
		u32 gpc, u32 tpc,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr)
{
	int ret = 0;

	/* Check for L1 tag ECC errors. */
	gr_gv11b_handle_l1_tag_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for LRF ECC errors. */
	gr_gv11b_handle_lrf_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for CBU ECC errors. */
	gr_gv11b_handle_cbu_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for L1 data ECC errors. */
	gr_gv11b_handle_l1_data_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	/* Check for L0 && L1 icache ECC errors. */
	gr_gv11b_handle_icache_exception(g, gpc, tpc, post_event, fault_ch, hww_global_esr);

	return ret;
}

int gr_gv11b_handle_gcc_exception(struct gk20a *g, u32 gpc, u32 tpc,
			bool *post_event, struct channel_gk20a *fault_ch,
			u32 *hww_global_esr)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 gcc_l15_ecc_status, gcc_l15_ecc_corrected_err_status = 0;
	u32 gcc_l15_ecc_uncorrected_err_status = 0;
	u32 gcc_l15_corrected_err_count_delta = 0;
	u32 gcc_l15_uncorrected_err_count_delta = 0;
	bool is_gcc_l15_ecc_corrected_total_err_overflow = 0;
	bool is_gcc_l15_ecc_uncorrected_total_err_overflow = 0;

	/* Check for gcc l15 ECC errors. */
	gcc_l15_ecc_status = gk20a_readl(g,
		gr_pri_gpc0_gcc_l15_ecc_status_r() + offset);
	gcc_l15_ecc_corrected_err_status = gcc_l15_ecc_status &
		(gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_bank0_m() |
		 gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_bank1_m());
	gcc_l15_ecc_uncorrected_err_status = gcc_l15_ecc_status &
		(gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_bank0_m() |
		 gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_bank1_m());

	if ((gcc_l15_ecc_corrected_err_status == 0) && (gcc_l15_ecc_uncorrected_err_status == 0)) {
		return 0;
	}

	gcc_l15_corrected_err_count_delta =
		gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_r() +
				offset));
	gcc_l15_uncorrected_err_count_delta =
		gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_total_v(
			gk20a_readl(g,
				gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_r() +
				offset));
	is_gcc_l15_ecc_corrected_total_err_overflow =
		gr_pri_gpc0_gcc_l15_ecc_status_corrected_err_total_counter_overflow_v(gcc_l15_ecc_status);
	is_gcc_l15_ecc_uncorrected_total_err_overflow =
		gr_pri_gpc0_gcc_l15_ecc_status_uncorrected_err_total_counter_overflow_v(gcc_l15_ecc_status);

	if ((gcc_l15_corrected_err_count_delta > 0) || is_gcc_l15_ecc_corrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"corrected error (SBE) detected in GCC L1.5! err_mask [%08x] is_overf [%d]",
			gcc_l15_ecc_corrected_err_status, is_gcc_l15_ecc_corrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_gcc_l15_ecc_corrected_total_err_overflow) {
			gcc_l15_corrected_err_count_delta +=
				BIT32(gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_total_s());
		}
		g->ecc.gr.gcc_l15_ecc_corrected_err_count[gpc].counter +=
							gcc_l15_corrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_gcc_l15_ecc_corrected_err_count_r() + offset,
			0);
	}
	if ((gcc_l15_uncorrected_err_count_delta > 0) || is_gcc_l15_ecc_uncorrected_total_err_overflow) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_intr,
			"Uncorrected error (DBE) detected in GCC L1.5! err_mask [%08x] is_overf [%d]",
			gcc_l15_ecc_uncorrected_err_status, is_gcc_l15_ecc_uncorrected_total_err_overflow);

		/* HW uses 16-bits counter */
		if (is_gcc_l15_ecc_uncorrected_total_err_overflow) {
			gcc_l15_uncorrected_err_count_delta +=
				BIT32(gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_total_s());
		}
		g->ecc.gr.gcc_l15_ecc_uncorrected_err_count[gpc].counter +=
							gcc_l15_uncorrected_err_count_delta;
		gk20a_writel(g,
			gr_pri_gpc0_gcc_l15_ecc_uncorrected_err_count_r() + offset,
			0);
	}

	gk20a_writel(g, gr_pri_gpc0_gcc_l15_ecc_status_r() + offset,
			gr_pri_gpc0_gcc_l15_ecc_status_reset_task_f());

	return 0;
}

static int gr_gv11b_handle_gpcmmu_ecc_exception(struct gk20a *g, u32 gpc,
								u32 exception)
{
	int ret = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;
	int hww_esr;

	hww_esr = gk20a_readl(g, gr_gpc0_mmu_gpcmmu_global_esr_r() + offset);

	if (!(hww_esr & (gr_gpc0_mmu_gpcmmu_global_esr_ecc_corrected_m() |
			gr_gpc0_mmu_gpcmmu_global_esr_ecc_uncorrected_m()))) {
		return ret;
	}

	ecc_status = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_status_r() + offset);
	ecc_addr = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_address_r() + offset);
	corrected_cnt = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_r() + offset);
	uncorrected_cnt = gk20a_readl(g,
		gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_r() + offset);

	corrected_delta = gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_total_counter_overflow_m();


	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow) {
		gk20a_writel(g,
			gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_r() +
			offset, 0);
	}
	if ((uncorrected_delta > 0) || uncorrected_overflow) {
		gk20a_writel(g,
			gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_r() +
			offset, 0);
	}

	gk20a_writel(g, gr_gpc0_mmu_l1tlb_ecc_status_r() + offset,
				gr_gpc0_mmu_l1tlb_ecc_status_reset_task_f());

	/* Handle overflow */
	if (corrected_overflow) {
		corrected_delta += (0x1UL << gr_gpc0_mmu_l1tlb_ecc_corrected_err_count_total_s());
	}
	if (uncorrected_overflow) {
		uncorrected_delta += (0x1UL << gr_gpc0_mmu_l1tlb_ecc_uncorrected_err_count_total_s());
	}


	g->ecc.gr.mmu_l1tlb_ecc_corrected_err_count[gpc].counter +=
							corrected_delta;
	g->ecc.gr.mmu_l1tlb_ecc_uncorrected_err_count[gpc].counter +=
							uncorrected_delta;
	nvgpu_log(g, gpu_dbg_intr,
			"mmu l1tlb gpc:%d ecc interrupt intr: 0x%x", gpc, hww_esr);

	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_l1tlb_sa_data_m()) {
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc sa data error");
	}
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_l1tlb_sa_data_m()) {
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc sa data error");
	}
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_corrected_err_l1tlb_fa_data_m()) {
		nvgpu_log(g, gpu_dbg_intr, "corrected ecc fa data error");
	}
	if (ecc_status & gr_gpc0_mmu_l1tlb_ecc_status_uncorrected_err_l1tlb_fa_data_m()) {
		nvgpu_log(g, gpu_dbg_intr, "uncorrected ecc fa data error");
	}
	if (corrected_overflow || uncorrected_overflow) {
		nvgpu_info(g, "mmu l1tlb ecc counter overflow!");
	}

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error address: 0x%x", ecc_addr);
	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.gr.mmu_l1tlb_ecc_corrected_err_count[gpc].counter,
		g->ecc.gr.mmu_l1tlb_ecc_uncorrected_err_count[gpc].counter);

	return ret;
}

static int gr_gv11b_handle_gpccs_ecc_exception(struct gk20a *g, u32 gpc,
								u32 exception)
{
	int ret = 0;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 offset = gpc_stride * gpc;
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;
	int hww_esr;

	hww_esr = gk20a_readl(g, gr_gpc0_gpccs_hww_esr_r() + offset);

	if (!(hww_esr & (gr_gpc0_gpccs_hww_esr_ecc_uncorrected_m() |
			gr_gpc0_gpccs_hww_esr_ecc_corrected_m()))) {
		return ret;
	}

	ecc_status = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_status_r() + offset);
	ecc_addr = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_address_r() + offset);
	corrected_cnt = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_corrected_err_count_r() + offset);
	uncorrected_cnt = gk20a_readl(g,
		gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_r() + offset);

	corrected_delta = gr_gpc0_gpccs_falcon_ecc_corrected_err_count_total_v(
							corrected_cnt);
	uncorrected_delta = gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);
	corrected_overflow = ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_corrected_err_total_counter_overflow_m();

	uncorrected_overflow = ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_total_counter_overflow_m();


	/* clear the interrupt */
	if ((corrected_delta > 0) || corrected_overflow) {
		gk20a_writel(g,
			gr_gpc0_gpccs_falcon_ecc_corrected_err_count_r() +
			offset, 0);
	}
	if ((uncorrected_delta > 0) || uncorrected_overflow) {
		gk20a_writel(g,
			gr_gpc0_gpccs_falcon_ecc_uncorrected_err_count_r() +
			offset, 0);
	}

	gk20a_writel(g, gr_gpc0_gpccs_falcon_ecc_status_r() + offset,
				gr_gpc0_gpccs_falcon_ecc_status_reset_task_f());

	g->ecc.gr.gpccs_ecc_corrected_err_count[gpc].counter +=
							corrected_delta;
	g->ecc.gr.gpccs_ecc_uncorrected_err_count[gpc].counter +=
							uncorrected_delta;
	nvgpu_log(g, gpu_dbg_intr,
			"gppcs gpc:%d ecc interrupt intr: 0x%x", gpc, hww_esr);

	if (ecc_status & gr_gpc0_gpccs_falcon_ecc_status_corrected_err_imem_m()) {
		nvgpu_log(g, gpu_dbg_intr, "imem ecc error corrected");
	}
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_imem_m()) {
		nvgpu_log(g, gpu_dbg_intr, "imem ecc error uncorrected");
	}
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_corrected_err_dmem_m()) {
		nvgpu_log(g, gpu_dbg_intr, "dmem ecc error corrected");
	}
	if (ecc_status &
		gr_gpc0_gpccs_falcon_ecc_status_uncorrected_err_dmem_m()) {
		nvgpu_log(g, gpu_dbg_intr, "dmem ecc error uncorrected");
	}
	if (corrected_overflow || uncorrected_overflow) {
		nvgpu_info(g, "gpccs ecc counter overflow!");
	}

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error row address: 0x%x",
		gr_gpc0_gpccs_falcon_ecc_address_row_address_v(ecc_addr));

	nvgpu_log(g, gpu_dbg_intr,
		"ecc error count corrected: %d, uncorrected %d",
		g->ecc.gr.gpccs_ecc_corrected_err_count[gpc].counter,
		g->ecc.gr.gpccs_ecc_uncorrected_err_count[gpc].counter);

	return ret;
}

int gr_gv11b_handle_gpc_gpcmmu_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception)
{
	if (gpc_exception & gr_gpc0_gpccs_gpc_exception_gpcmmu_m()) {
		return gr_gv11b_handle_gpcmmu_ecc_exception(g, gpc,
								gpc_exception);
	}
	return 0;
}

int gr_gv11b_handle_gpc_gpccs_exception(struct gk20a *g, u32 gpc,
							u32 gpc_exception)
{
	if (gpc_exception & gr_gpc0_gpccs_gpc_exception_gpccs_m()) {
		return gr_gv11b_handle_gpccs_ecc_exception(g, gpc,
								gpc_exception);
	}

	return 0;
}

void gr_gv11b_enable_gpc_exceptions(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 tpc_mask;

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(),
			gr_gpcs_tpcs_tpccs_tpc_exception_en_sm_enabled_f() |
			gr_gpcs_tpcs_tpccs_tpc_exception_en_mpc_enabled_f());

	tpc_mask =
		gr_gpcs_gpccs_gpc_exception_en_tpc_f(
			(1 << gr->max_tpc_per_gpc_count) - 1);

	gk20a_writel(g, gr_gpcs_gpccs_gpc_exception_en_r(),
		(tpc_mask | gr_gpcs_gpccs_gpc_exception_en_gcc_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpccs_f(1) |
			    gr_gpcs_gpccs_gpc_exception_en_gpcmmu_f(1)));
}

int gr_gv11b_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
		bool *post_event)
{
	return 0;
}

int gr_gv11b_zbc_s_query_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	u32 index = query_params->index_size;

	if (index >= GK20A_ZBC_TABLE_SIZE) {
		nvgpu_err(g, "invalid zbc stencil table index");
		return -EINVAL;
	}

	nvgpu_speculation_barrier();
	query_params->depth = gr->zbc_s_tbl[index].stencil;
	query_params->format = gr->zbc_s_tbl[index].format;
	query_params->ref_cnt = gr->zbc_s_tbl[index].ref_cnt;

	return 0;
}

bool gr_gv11b_add_zbc_type_s(struct gk20a *g, struct gr_gk20a *gr,
		     struct zbc_entry *zbc_val, int *ret_val)
{
	struct zbc_s_table *s_tbl;
	u32 i;
	bool added = false;

	*ret_val = -ENOMEM;

	/* search existing tables */
	for (i = 0; i < gr->max_used_s_index; i++) {

		s_tbl = &gr->zbc_s_tbl[i];

		if (s_tbl->ref_cnt &&
		    s_tbl->stencil == zbc_val->depth &&
		    s_tbl->format == zbc_val->format) {
			added = true;
			s_tbl->ref_cnt++;
			*ret_val = 0;
			break;
		}
	}
	/* add new table */
	if (!added &&
	    gr->max_used_s_index < GK20A_ZBC_TABLE_SIZE) {

		s_tbl = &gr->zbc_s_tbl[gr->max_used_s_index];
		WARN_ON(s_tbl->ref_cnt != 0);

		*ret_val = g->ops.gr.add_zbc_s(g, gr,
				zbc_val, gr->max_used_s_index);

		if (!(*ret_val)) {
			gr->max_used_s_index++;
		}
	}
	return added;
}

int gr_gv11b_add_zbc_stencil(struct gk20a *g, struct gr_gk20a *gr,
				struct zbc_entry *stencil_val, u32 index)
{
	u32 zbc_s;

	/* update l2 table */
	g->ops.ltc.set_zbc_s_entry(g, stencil_val, index);

	/* update local copy */
	gr->zbc_s_tbl[index].stencil = stencil_val->depth;
	gr->zbc_s_tbl[index].format = stencil_val->format;
	gr->zbc_s_tbl[index].ref_cnt++;

	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_s_r(index), stencil_val->depth);
	zbc_s = gk20a_readl(g, gr_gpcs_swdx_dss_zbc_s_01_to_04_format_r() +
				 (index & ~3));
	zbc_s &= ~(0x7f << (index % 4) * 7);
	zbc_s |= stencil_val->format << (index % 4) * 7;
	gk20a_writel(g, gr_gpcs_swdx_dss_zbc_s_01_to_04_format_r() +
				 (index & ~3), zbc_s);

	return 0;
}

int gr_gv11b_load_stencil_default_tbl(struct gk20a *g,
		 struct gr_gk20a *gr)
{
	struct zbc_entry zbc_val;
	int err;

	/* load default stencil table */
	zbc_val.type = GV11B_ZBC_TYPE_STENCIL;

	zbc_val.depth = 0x0;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto fail;
	}
	zbc_val.depth = 0x1;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto fail;
	}

	zbc_val.depth = 0xff;
	zbc_val.format = ZBC_STENCIL_CLEAR_FMT_U8;
	err = gr_gk20a_add_zbc(g, gr, &zbc_val);
	if (err != 0) {
		goto fail;
	}

	gr->max_default_s_index = 3;

	return 0;

fail:
	nvgpu_err(g, "fail to load default zbc stencil table");
	return err;
}

int gr_gv11b_load_stencil_tbl(struct gk20a *g, struct gr_gk20a *gr)
{
	int ret;
	u32 i;

	for (i = 0; i < gr->max_used_s_index; i++) {
		struct zbc_s_table *s_tbl = &gr->zbc_s_tbl[i];
		struct zbc_entry zbc_val;

		zbc_val.type = GV11B_ZBC_TYPE_STENCIL;
		zbc_val.depth = s_tbl->stencil;
		zbc_val.format = s_tbl->format;

		ret = g->ops.gr.add_zbc_s(g, gr, &zbc_val, i);
		if (ret) {
			return ret;
		}
	}
	return 0;
}

u32 gr_gv11b_pagepool_default_size(struct gk20a *g)
{
	return gr_scc_pagepool_total_pages_hwmax_value_v();
}

int gr_gv11b_calc_global_ctx_buffer_size(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int size;

	gr->attrib_cb_size = gr->attrib_cb_default_size;
	gr->alpha_cb_size = gr->alpha_cb_default_size;

	gr->attrib_cb_size = min(gr->attrib_cb_size,
		 gr_gpc0_ppc0_cbm_beta_cb_size_v_f(~0) / g->gr.tpc_count);
	gr->alpha_cb_size = min(gr->alpha_cb_size,
		 gr_gpc0_ppc0_cbm_alpha_cb_size_v_f(~0) / g->gr.tpc_count);

	size = gr->attrib_cb_size *
		gr_gpc0_ppc0_cbm_beta_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size += gr->alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() *
		gr->max_tpc_count;

	size = ALIGN(size, 128);

	return size;
}

void gr_gv11b_set_go_idle_timeout(struct gk20a *g, u32 data)
{
	gk20a_writel(g, gr_fe_go_idle_timeout_r(), data);
}

void gr_gv11b_set_coalesce_buffer_size(struct gk20a *g, u32 data)
{
	u32 val;

	nvgpu_log_fn(g, " ");

	val = gk20a_readl(g, gr_gpcs_tc_debug0_r());
	val = set_field(val, gr_gpcs_tc_debug0_limit_coalesce_buffer_size_m(),
			     gr_gpcs_tc_debug0_limit_coalesce_buffer_size_f(data));
	gk20a_writel(g, gr_gpcs_tc_debug0_r(), val);

	nvgpu_log_fn(g, "done");
}

void gr_gv11b_set_tex_in_dbg(struct gk20a *g, u32 data)
{
	u32 val;
	bool flag;

	nvgpu_log_fn(g, " ");

	val = gk20a_readl(g, gr_gpcs_tpcs_tex_in_dbg_r());
	flag = (data & NVC397_SET_TEX_IN_DBG_TSL1_RVCH_INVALIDATE) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_tex_in_dbg_tsl1_rvch_invalidate_m(),
			gr_gpcs_tpcs_tex_in_dbg_tsl1_rvch_invalidate_f(flag));
	gk20a_writel(g, gr_gpcs_tpcs_tex_in_dbg_r(), val);

	val = gk20a_readl(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r());
	flag = (data &
		NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_LD) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_ld_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_ld_f(flag));
	flag = (data &
		NVC397_SET_TEX_IN_DBG_SM_L1TAG_CTRL_CACHE_SURFACE_ST) ? 1 : 0;
	val = set_field(val, gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_st_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_cache_surface_st_f(flag));
	gk20a_writel(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r(), val);
}

void gr_gv11b_set_skedcheck(struct gk20a *g, u32 data)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, gr_sked_hww_esr_en_r());

	if ((data & NVC397_SET_SKEDCHECK_18_MASK) ==
			NVC397_SET_SKEDCHECK_18_DISABLE) {
		reg_val = set_field(reg_val,
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_m(),
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_disabled_f()
			);
	} else if ((data & NVC397_SET_SKEDCHECK_18_MASK) ==
			NVC397_SET_SKEDCHECK_18_ENABLE) {
		reg_val = set_field(reg_val,
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_m(),
			gr_sked_hww_esr_en_skedcheck18_l1_config_too_small_enabled_f()
			);
	}
	nvgpu_log_info(g, "sked_hww_esr_en = 0x%x", reg_val);
	gk20a_writel(g, gr_sked_hww_esr_en_r(), reg_val);

}

void gv11b_gr_set_shader_exceptions(struct gk20a *g, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (data == NVA297_SET_SHADER_EXCEPTIONS_ENABLE_FALSE) {
		gk20a_writel(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r(),
				 0);
		gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r(),
				 0);
	} else {
		g->ops.gr.set_hww_esr_report_mask(g);
	}
}

void gr_gv11b_set_shader_cut_collector(struct gk20a *g, u32 data)
{
	u32 val;

	nvgpu_log_fn(g, "gr_gv11b_set_shader_cut_collector");

	val = gk20a_readl(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r());
	if (data & NVC397_SET_SHADER_CUT_COLLECTOR_STATE_ENABLE) {
		val = set_field(val,
			gr_gpcs_tpcs_sm_l1tag_ctrl_always_cut_collector_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_always_cut_collector_enable_f());
	} else {
		val = set_field(val,
			gr_gpcs_tpcs_sm_l1tag_ctrl_always_cut_collector_m(),
			gr_gpcs_tpcs_sm_l1tag_ctrl_always_cut_collector_disable_f());
	}
	gk20a_writel(g, gr_gpcs_tpcs_sm_l1tag_ctrl_r(), val);
}


int gr_gv11b_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data)
{
	nvgpu_log_fn(g, " ");

	if (class_num == VOLTA_COMPUTE_A) {
		switch (offset << 2) {
		case NVC0C0_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC3C0_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		case NVC3C0_SET_SHADER_CUT_COLLECTOR:
			gr_gv11b_set_shader_cut_collector(g, data);
			break;
		default:
			goto fail;
		}
	}

	if (class_num == VOLTA_A) {
		switch (offset << 2) {
		case NVC397_SET_SHADER_EXCEPTIONS:
			gv11b_gr_set_shader_exceptions(g, data);
			break;
		case NVC397_SET_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_circular_buffer_size(g, data);
			break;
		case NVC397_SET_ALPHA_CIRCULAR_BUFFER_SIZE:
			g->ops.gr.set_alpha_circular_buffer_size(g, data);
			break;
		case NVC397_SET_GO_IDLE_TIMEOUT:
			gr_gv11b_set_go_idle_timeout(g, data);
			break;
		case NVC097_SET_COALESCE_BUFFER_SIZE:
			gr_gv11b_set_coalesce_buffer_size(g, data);
			break;
		case NVC397_SET_TEX_IN_DBG:
			gr_gv11b_set_tex_in_dbg(g, data);
			break;
		case NVC397_SET_SKEDCHECK:
			gr_gv11b_set_skedcheck(g, data);
			break;
		case NVC397_SET_BES_CROP_DEBUG3:
			g->ops.gr.set_bes_crop_debug3(g, data);
			break;
		case NVC397_SET_BES_CROP_DEBUG4:
			g->ops.gr.set_bes_crop_debug4(g, data);
			break;
		case NVC397_SET_SHADER_CUT_COLLECTOR:
			gr_gv11b_set_shader_cut_collector(g, data);
			break;
		default:
			goto fail;
		}
	}
	return 0;

fail:
	return -EINVAL;
}

void gr_gv11b_bundle_cb_defaults(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	gr->bundle_cb_default_size =
		gr_scc_bundle_cb_size_div_256b__prod_v();
	gr->min_gpm_fifo_depth =
		gr_pd_ab_dist_cfg2_state_limit_min_gpm_fifo_depths_v();
	gr->bundle_cb_token_limit =
		gr_pd_ab_dist_cfg2_token_limit_init_v();
}

void gr_gv11b_cb_size_default(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;

	if (!gr->attrib_cb_default_size) {
		gr->attrib_cb_default_size =
			gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v();
	}
	gr->alpha_cb_default_size =
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_default_v();
	gr->attrib_cb_gfxp_default_size =
		gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
	gr->attrib_cb_gfxp_size =
		gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v();
}

void gr_gv11b_set_alpha_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 pd_ab_max_output;
	u32 alpha_cb_size = data * 4;

	nvgpu_log_fn(g, " ");

	if (alpha_cb_size > gr->alpha_cb_size) {
		alpha_cb_size = gr->alpha_cb_size;
	}

	gk20a_writel(g, gr_ds_tga_constraintlogic_alpha_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_alpha_r()) &
		 ~gr_ds_tga_constraintlogic_alpha_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_alpha_cbsize_f(alpha_cb_size));

	pd_ab_max_output = alpha_cb_size *
		gr_gpc0_ppc0_cbm_alpha_cb_size_v_granularity_v() /
		gr_pd_ab_dist_cfg1_max_output_granularity_v();

	gk20a_writel(g, gr_pd_ab_dist_cfg1_r(),
		gr_pd_ab_dist_cfg1_max_output_f(pd_ab_max_output) |
		gr_pd_ab_dist_cfg1_max_batches_init_f());

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = proj_gpc_stride_v() * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index);

			val = set_field(val, gr_gpc0_ppc0_cbm_alpha_cb_size_v_m(),
					gr_gpc0_ppc0_cbm_alpha_cb_size_v_f(alpha_cb_size *
						gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_alpha_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index, val);
		}
	}
}

void gr_gv11b_set_circular_buffer_size(struct gk20a *g, u32 data)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index, ppc_index, stride, val;
	u32 cb_size_steady = data * 4, cb_size;

	nvgpu_log_fn(g, " ");

	if (cb_size_steady > gr->attrib_cb_size) {
		cb_size_steady = gr->attrib_cb_size;
	}
	if (gk20a_readl(g, gr_gpc0_ppc0_cbm_beta_cb_size_r()) !=
		gk20a_readl(g,
			gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r())) {
		cb_size = cb_size_steady +
			(gr_gpc0_ppc0_cbm_beta_cb_size_v_gfxp_v() -
			 gr_gpc0_ppc0_cbm_beta_cb_size_v_default_v());
	} else {
		cb_size = cb_size_steady;
	}

	gk20a_writel(g, gr_ds_tga_constraintlogic_beta_r(),
		(gk20a_readl(g, gr_ds_tga_constraintlogic_beta_r()) &
		 ~gr_ds_tga_constraintlogic_beta_cbsize_f(~0)) |
		 gr_ds_tga_constraintlogic_beta_cbsize_f(cb_size_steady));

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		stride = proj_gpc_stride_v() * gpc_index;

		for (ppc_index = 0; ppc_index < gr->gpc_ppc_count[gpc_index];
			ppc_index++) {

			val = gk20a_readl(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index);

			val = set_field(val,
				gr_gpc0_ppc0_cbm_beta_cb_size_v_m(),
				gr_gpc0_ppc0_cbm_beta_cb_size_v_f(cb_size *
					gr->pes_tpc_count[ppc_index][gpc_index]));

			gk20a_writel(g, gr_gpc0_ppc0_cbm_beta_cb_size_r() +
				stride +
				proj_ppc_in_gpc_stride_v() * ppc_index, val);

			gk20a_writel(g, proj_ppc_in_gpc_stride_v() * ppc_index +
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_r() +
				stride,
				gr_gpc0_ppc0_cbm_beta_steady_state_cb_size_v_f(
					cb_size_steady));

			val = gk20a_readl(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index));

			val = set_field(val,
				gr_gpcs_swdx_tc_beta_cb_size_v_m(),
				gr_gpcs_swdx_tc_beta_cb_size_v_f(
					cb_size_steady *
					gr->gpc_ppc_count[gpc_index]));

			gk20a_writel(g, gr_gpcs_swdx_tc_beta_cb_size_r(
						ppc_index + gpc_index), val);
		}
	}
}

int gr_gv11b_alloc_buffer(struct vm_gk20a *vm, size_t size,
			struct nvgpu_mem *mem)
{
	int err;
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_log_fn(g, " ");

	err = nvgpu_dma_alloc_sys(vm->mm->g, size, mem);
	if (err) {
		return err;
	}

	mem->gpu_va = nvgpu_gmmu_map(vm,
				mem,
				size,
				NVGPU_VM_MAP_CACHEABLE,
				gk20a_mem_flag_none,
				false,
				mem->aperture);

	if (!mem->gpu_va) {
		err = -ENOMEM;
		goto fail_free;
	}

	return 0;

fail_free:
	nvgpu_dma_free(vm->mm->g, mem);
	return err;
}

int gr_gv11b_set_ctxsw_preemption_mode(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode)
{
	int err = 0;

	if (g->ops.gr.is_valid_gfx_class(g, class) &&
				g->gr.ctx_vars.force_preemption_gfxp) {
		graphics_preempt_mode = NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP;
	}

	if (g->ops.gr.is_valid_compute_class(g, class) &&
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

	/* Do not allow lower preemption modes than current ones */
	if (graphics_preempt_mode &&
	   (graphics_preempt_mode < gr_ctx->graphics_preempt_mode)) {
		return -EINVAL;
	}

	if (compute_preempt_mode &&
	   (compute_preempt_mode < gr_ctx->compute_preempt_mode)) {
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

	if (g->ops.gr.is_valid_compute_class(g, class) ||
			g->ops.gr.is_valid_gfx_class(g, class)) {
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

void gr_gv11b_update_ctxsw_preemption_mode(struct gk20a *g,
		struct channel_gk20a *c,
		struct nvgpu_mem *mem)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	u32 gfxp_preempt_option =
		ctxsw_prog_main_image_graphics_preemption_options_control_gfxp_f();
	u32 cilp_preempt_option =
		ctxsw_prog_main_image_compute_preemption_options_control_cilp_f();
	u32 cta_preempt_option =
		ctxsw_prog_main_image_compute_preemption_options_control_cta_f();
	int err;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg) {
		return;
	}

	gr_ctx = &tsg->gr_ctx;

	if (gr_ctx->graphics_preempt_mode ==
					NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP) {
		nvgpu_log_info(g, "GfxP: %x", gfxp_preempt_option);
		nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_graphics_preemption_options_o(),
			gfxp_preempt_option);
	}

	if (gr_ctx->compute_preempt_mode ==
					NVGPU_PREEMPTION_MODE_COMPUTE_CILP) {
		nvgpu_log_info(g, "CILP: %x", cilp_preempt_option);
		nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_compute_preemption_options_o(),
			cilp_preempt_option);
	}

	if (gr_ctx->compute_preempt_mode ==
					NVGPU_PREEMPTION_MODE_COMPUTE_CTA) {
		nvgpu_log_info(g, "CTA: %x", cta_preempt_option);
		nvgpu_mem_wr(g, mem,
			ctxsw_prog_main_image_compute_preemption_options_o(),
			cta_preempt_option);
	}

	if (gr_ctx->preempt_ctxsw_buffer.gpu_va) {
		u32 addr;
		u32 size;
		u32 cbes_reserve;

		if (g->ops.gr.set_preemption_buffer_va) {
			if (ctxheader->gpu_va) {
				g->ops.gr.set_preemption_buffer_va(g, ctxheader,
				gr_ctx->preempt_ctxsw_buffer.gpu_va);
			} else {
				g->ops.gr.set_preemption_buffer_va(g, mem,
				gr_ctx->preempt_ctxsw_buffer.gpu_va);
			}
		}

		err = gr_gk20a_ctx_patch_write_begin(g, gr_ctx, true);
		if (err) {
			nvgpu_err(g, "can't map patch context");
			goto out;
		}

		addr = (u64_lo32(gr_ctx->betacb_ctxsw_buffer.gpu_va) >>
			gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()) |
			(u64_hi32(gr_ctx->betacb_ctxsw_buffer.gpu_va) <<
			 (32 - gr_gpcs_setup_attrib_cb_base_addr_39_12_align_bits_v()));

		nvgpu_log_info(g, "attrib cb addr : 0x%016x", addr);
		g->ops.gr.commit_global_attrib_cb(g, gr_ctx, addr, true);

		addr = (u64_lo32(gr_ctx->pagepool_ctxsw_buffer.gpu_va) >>
			gr_scc_pagepool_base_addr_39_8_align_bits_v()) |
			(u64_hi32(gr_ctx->pagepool_ctxsw_buffer.gpu_va) <<
			 (32 - gr_scc_pagepool_base_addr_39_8_align_bits_v()));
		size = gr_ctx->pagepool_ctxsw_buffer.size;

		if (size == g->ops.gr.pagepool_default_size(g)) {
			size = gr_scc_pagepool_total_pages_hwmax_v();
		}

		g->ops.gr.commit_global_pagepool(g, gr_ctx, addr, size, true);

		addr = (u64_lo32(gr_ctx->spill_ctxsw_buffer.gpu_va) >>
			gr_gpc0_swdx_rm_spill_buffer_addr_39_8_align_bits_v()) |
			(u64_hi32(gr_ctx->spill_ctxsw_buffer.gpu_va) <<
			 (32 - gr_gpc0_swdx_rm_spill_buffer_addr_39_8_align_bits_v()));
		size = gr_ctx->spill_ctxsw_buffer.size /
			gr_gpc0_swdx_rm_spill_buffer_size_256b_byte_granularity_v();

		gr_gk20a_ctx_patch_write(g, gr_ctx,
				gr_gpc0_swdx_rm_spill_buffer_addr_r(),
				gr_gpc0_swdx_rm_spill_buffer_addr_39_8_f(addr),
				true);
		gr_gk20a_ctx_patch_write(g, gr_ctx,
				gr_gpc0_swdx_rm_spill_buffer_size_r(),
				gr_gpc0_swdx_rm_spill_buffer_size_256b_f(size),
				true);

		cbes_reserve = gr_gpcs_swdx_beta_cb_ctrl_cbes_reserve_gfxp_v();
		gr_gk20a_ctx_patch_write(g, gr_ctx,
				gr_gpcs_swdx_beta_cb_ctrl_r(),
				gr_gpcs_swdx_beta_cb_ctrl_cbes_reserve_f(
					cbes_reserve),
				true);
		gr_gk20a_ctx_patch_write(g, gr_ctx,
				gr_gpcs_ppcs_cbm_beta_cb_ctrl_r(),
				gr_gpcs_ppcs_cbm_beta_cb_ctrl_cbes_reserve_f(
					cbes_reserve),
				true);

		gr_gk20a_ctx_patch_write(g, gr_ctx,
				gr_fe_gfxp_wfi_timeout_r(),
				g->gr.gfxp_wfi_timeout_count,
				true);

		gr_gk20a_ctx_patch_write_end(g, gr_ctx, true);
	}

out:
	nvgpu_log_fn(g, "done");
}
static void gr_gv11b_dump_gr_per_sm_regs(struct gk20a *g,
			struct gk20a_debug_output *o,
			u32 gpc, u32 tpc, u32 sm, u32 offset)
{

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_WARP_ESR: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_WARP_ESR_REPORT_MASK: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_GLOBAL_ESR: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_HWW_GLOBAL_ESR_REPORT_MASK: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_DBGR_CONTROL0: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset));

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPC%d_TPC%d_SM%d_DBGR_STATUS0: 0x%x\n",
		gpc, tpc, sm, gk20a_readl(g,
		gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset));
}

static int gr_gv11b_dump_gr_sm_regs(struct gk20a *g,
			   struct gk20a_debug_output *o)
{
	u32 gpc, tpc, sm, sm_per_tpc;
	u32 gpc_offset, tpc_offset, offset;

	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_GLOBAL_ESR_REPORT_MASK: 0x%x\n",
		gk20a_readl(g,
		gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_WARP_ESR_REPORT_MASK: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_HWW_GLOBAL_ESR: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_hww_global_esr_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_CONTROL0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_control0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_STATUS0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_status0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_BPT_PAUSE_MASK_0: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_0_r()));
	gk20a_debug_output(o,
		"NV_PGRAPH_PRI_GPCS_TPCS_SMS_DBGR_BPT_PAUSE_MASK_1: 0x%x\n",
		gk20a_readl(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_1_r()));

	sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		gpc_offset = gk20a_gr_gpc_offset(g, gpc);

		for (tpc = 0; tpc < g->gr.gpc_tpc_count[gpc]; tpc++) {
			tpc_offset = gk20a_gr_tpc_offset(g, tpc);

			for (sm = 0; sm < sm_per_tpc; sm++) {
				offset = gpc_offset + tpc_offset +
					gv11b_gr_sm_offset(g, sm);

				gr_gv11b_dump_gr_per_sm_regs(g, o,
					gpc, tpc, sm, offset);
			}
		}
	}

	return 0;
}

int gr_gv11b_dump_gr_status_regs(struct gk20a *g,
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
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_TIMEOUT : 0x%x\n",
		gk20a_readl(g, gr_fe_go_idle_timeout_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_GO_IDLE_INFO : 0x%x\n",
		gk20a_readl(g, gr_pri_fe_go_idle_info_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_GPC0_TPC0_TEX_M_TEX_SUBUNITS_STATUS: 0x%x\n",
		gk20a_readl(g, gr_pri_gpc0_tpc0_tex_m_tex_subunits_status_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_FS: 0x%x\n",
		gk20a_readl(g, gr_cwd_fs_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FE_TPC_FS(0): 0x%x\n",
		gk20a_readl(g, gr_fe_tpc_fs_r(0)));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_CWD_GPC_TPC_ID: 0x%x\n",
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
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_ENABLE : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_enable_r()));
	gk20a_debug_output(o, "NV_PGRAPH_PRI_FECS_HOST_INT_STATUS : 0x%x\n",
		gk20a_readl(g, gr_fecs_host_int_status_r()));
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

	gr_gv11b_dump_gr_sm_regs(g, o);

	return 0;
}

static bool gr_activity_empty_or_preempted(u32 val)
{
	while (val) {
		u32 v = val & 7;
		if (v != gr_activity_4_gpc0_empty_v() &&
		    v != gr_activity_4_gpc0_preempted_v()) {
			return false;
		}
		val >>= 3;
	}

	return true;
}

int gr_gv11b_wait_empty(struct gk20a *g, unsigned long duration_ms,
		       u32 expect_delay)
{
	u32 delay = expect_delay;
	bool ctxsw_active;
	bool gr_busy;
	u32 gr_status;
	u32 activity0, activity1, activity2, activity4;
	struct nvgpu_timeout timeout;

	nvgpu_log_fn(g, " ");

	nvgpu_timeout_init(g, &timeout, duration_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		/* fmodel: host gets fifo_engine_status(gr) from gr
		   only when gr_status is read */
		gr_status = gk20a_readl(g, gr_status_r());

		ctxsw_active = gr_status & 1<<7;

		activity0 = gk20a_readl(g, gr_activity_0_r());
		activity1 = gk20a_readl(g, gr_activity_1_r());
		activity2 = gk20a_readl(g, gr_activity_2_r());
		activity4 = gk20a_readl(g, gr_activity_4_r());

		gr_busy = !(gr_activity_empty_or_preempted(activity0) &&
			    gr_activity_empty_or_preempted(activity1) &&
			    activity2 == 0 &&
			    gr_activity_empty_or_preempted(activity4));

		if (!gr_busy && !ctxsw_active) {
			nvgpu_log_fn(g, "done");
			return 0;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);

	} while (!nvgpu_timeout_expired(&timeout));

	nvgpu_err(g,
		"timeout, ctxsw busy : %d, gr busy : %d, %08x, %08x, %08x, %08x",
		ctxsw_active, gr_busy, activity0, activity1, activity2, activity4);

	return -EAGAIN;
}

void gr_gv11b_commit_global_attrib_cb(struct gk20a *g,
					     struct nvgpu_gr_ctx *gr_ctx,
					     u64 addr, bool patch)
{
	int attrBufferSize;

	if (gr_ctx->preempt_ctxsw_buffer.gpu_va) {
		attrBufferSize = gr_ctx->betacb_ctxsw_buffer.size;
	} else {
		attrBufferSize = g->ops.gr.calc_global_ctx_buffer_size(g);
	}

	attrBufferSize /= gr_gpcs_tpcs_tex_rm_cb_1_size_div_128b_granularity_f();

	gr_gm20b_commit_global_attrib_cb(g, gr_ctx, addr, patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_r(),
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_v_f(addr) |
		gr_gpcs_tpcs_mpc_vtg_cb_global_base_addr_valid_true_f(), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_tex_rm_cb_0_r(),
		gr_gpcs_tpcs_tex_rm_cb_0_base_addr_43_12_f(addr), patch);

	gr_gk20a_ctx_patch_write(g, gr_ctx, gr_gpcs_tpcs_tex_rm_cb_1_r(),
		gr_gpcs_tpcs_tex_rm_cb_1_size_div_128b_f(attrBufferSize) |
		gr_gpcs_tpcs_tex_rm_cb_1_valid_true_f(), patch);
}

void gr_gv11b_set_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	u32 fuse_val;

	if (!g->gr.gpc_tpc_mask[gpc_index]) {
		return;
	}

	/*
	 * For s/w value g->gr.gpc_tpc_mask[gpc_index], bit value 1 indicates
	 * corresponding TPC is enabled. But for h/w fuse register, bit value 1
	 * indicates corresponding TPC is disabled.
	 * So we need to flip the bits and ensure we don't write to bits greater
	 * than TPC count
	 */
	fuse_val = g->gr.gpc_tpc_mask[gpc_index];
	fuse_val = ~fuse_val;
	fuse_val = fuse_val & 0xf; /* tpc0_disable fuse is only 4-bit wide */

	nvgpu_tegra_fuse_write_bypass(g, 0x1);
	nvgpu_tegra_fuse_write_access_sw(g, 0x0);

	nvgpu_tegra_fuse_write_opt_gpu_tpc0_disable(g, fuse_val);
}

void gr_gv11b_get_access_map(struct gk20a *g,
				   u32 **whitelist, int *num_entries)
{
	static u32 wl_addr_gv11b[] = {
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
		0x419e84, /* gr_pri_gpcs_tpcs_sms_dbgr_control0 */
		0x419ba4, /* gr_pri_gpcs_tpcs_sm_disp_ctrl     */
	};

	*whitelist = wl_addr_gv11b;
	*num_entries = ARRAY_SIZE(wl_addr_gv11b);
}

static int gr_gv11b_handle_warp_esr_error_mmu_nack(struct gk20a *g,
	u32 gpc, u32 tpc, u32 sm,
	u32 warp_esr_error,
	struct channel_gk20a *fault_ch)
{
	u32 offset;
	int err = 0;

	fault_ch = gk20a_channel_get(fault_ch);
	if (fault_ch) {
		if (!fault_ch->mmu_nack_handled) {
			/* recovery is not done for the channel implying mmu
			 * nack interrupt is serviced before mmu fault. Force
			 * recovery by returning an error. Also indicate we
			 * should skip a second recovery.
			 */
			fault_ch->mmu_nack_handled = true;
			err = -EFAULT;
		}
	}
	/* else mmu fault is serviced first and channel is closed */

	/* do not release reference to ch as we do not want userspace to close
	 * this channel on recovery. Otherwise mmu fault handler will enter
	 * recovery path even if channel is invalid. We want to explicitly check
	 * for teardown value in mmu fault handler.
	 */
	if (!err) {
		gk20a_channel_put(fault_ch);
	}

	/* clear interrupt */
	offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);
	nvgpu_writel(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset, 0);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"ESR %s(0x%x)",
			"MMU NACK ERROR",
			warp_esr_error);
	return err;
}

static bool gr_gv11b_check_warp_esr_error(struct gk20a *g, u32 warp_esr_error)
{
	u32 index = 0U;
	bool esr_err = false;

	struct warp_esr_error_table_s {
		u32 error_value;
		const char *error_name;
	};

	struct warp_esr_error_table_s warp_esr_error_table[] = {
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_stack_error_f(),
				"STACK ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_api_stack_error_f(),
				"API STACK ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_pc_wrap_f(),
				"PC WRAP ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_misaligned_pc_f(),
				"MISALIGNED PC ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_pc_overflow_f(),
				"PC OVERFLOW ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_misaligned_reg_f(),
				"MISALIGNED REG ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_illegal_instr_encoding_f(),
				"ILLEGAL INSTRUCTION ENCODING ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_illegal_instr_param_f(),
				"ILLEGAL INSTRUCTION PARAM ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_oor_reg_f(),
				"OOR REG ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_oor_addr_f(),
				"OOR ADDR ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_misaligned_addr_f(),
				"MISALIGNED ADDR ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_invalid_addr_space_f(),
				"INVALID ADDR SPACE ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_invalid_const_addr_ldc_f(),
				"INVALID ADDR LDC ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_stack_overflow_f(),
				"STACK OVERFLOW ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_mmu_fault_f(),
				"MMU FAULT ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_tex_format_f(),
				"TEX FORMAT ERROR"},
		{ gr_gpc0_tpc0_sm0_hww_warp_esr_error_tex_layout_f(),
				"TEX LAYOUT ERROR"},
	};

	for (index = 0; index < ARRAY_SIZE(warp_esr_error_table); index++) {
		if (warp_esr_error_table[index].error_value == warp_esr_error) {
			esr_err = true;
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"WARP_ESR %s(0x%x)",
				warp_esr_error_table[index].error_name,
				esr_err);
			break;
		}
	}

	return esr_err;
}

static int gr_gv11b_handle_all_warp_esr_errors(struct gk20a *g,
						u32 gpc, u32 tpc, u32 sm,
						u32 warp_esr_error,
						struct channel_gk20a *fault_ch)
{
	struct tsg_gk20a *tsg;
	struct channel_gk20a *ch_tsg;
	u32 offset = 0U;
	bool is_esr_error = false;

	/*
	 * Check for an esr error
	 */
	is_esr_error = gr_gv11b_check_warp_esr_error(g, warp_esr_error);
	if (!is_esr_error) {
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"No ESR error, Skip RC recovery and Trigeer CILP");
		return 0;
	}

	if (fault_ch) {
		tsg = &g->fifo.tsg[fault_ch->tsgid];

		/*
		 * Check SET_EXCEPTION_TYPE_MASK is being set.
		 * If set, skip the recovery and trigger CILP
		 * If not set, trigger the recovery.
		 */
		if ((tsg->sm_exception_mask_type &
			NVGPU_SM_EXCEPTION_TYPE_MASK_FATAL) ==
				NVGPU_SM_EXCEPTION_TYPE_MASK_FATAL) {
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"SM Exception Type Mask set %d,"
				"skip recovery",
				tsg->sm_exception_mask_type);
			return 0;
		}

		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		nvgpu_list_for_each_entry(ch_tsg, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch_tsg)) {
				g->ops.fifo.set_error_notifier(ch_tsg,
						 NVGPU_ERR_NOTIFIER_GR_EXCEPTION);
				gk20a_channel_put(ch_tsg);
			}
		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	}

	/* clear interrupt */
	offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);
	nvgpu_writel(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset, 0);

	/* return error so that recovery is triggered by gk20a_gr_isr() */
	return -EFAULT;
}

/* @brief pre-process work on the SM exceptions to determine if we clear them or not.
 *
 * On Pascal, if we are in CILP preemtion mode, preempt the channel and handle errors with special processing
 */
int gr_gv11b_pre_process_sm_exception(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm, u32 global_esr, u32 warp_esr,
		bool sm_debugger_attached, struct channel_gk20a *fault_ch,
		bool *early_exit, bool *ignore_debugger)
{
	int ret;
	bool cilp_enabled = false;
	u32 global_mask = 0, dbgr_control0, global_esr_copy;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);
	u32 warp_esr_error = gr_gpc0_tpc0_sm0_hww_warp_esr_error_v(warp_esr);
	struct tsg_gk20a *tsg;

	*early_exit = false;
	*ignore_debugger = false;

	/*
	 * We don't need to trigger CILP in case of MMU_NACK
	 * So just handle MMU_NACK and return
	 */
	if (warp_esr_error == gr_gpc0_tpc0_sm0_hww_warp_esr_error_mmu_nack_f()) {
		return gr_gv11b_handle_warp_esr_error_mmu_nack(g, gpc, tpc, sm,
				warp_esr_error, fault_ch);
	}

	/*
	 * Proceed to trigger CILP preemption if the return value
	 * from this function is zero, else proceed to recovery
	 */
	ret = gr_gv11b_handle_all_warp_esr_errors(g, gpc, tpc, sm,
				warp_esr_error, fault_ch);
	if (ret) {
		return ret;
	}

	if (fault_ch) {
		tsg = tsg_gk20a_from_ch(fault_ch);
		if (!tsg) {
			return -EINVAL;
		}

		cilp_enabled = (tsg->gr_ctx.compute_preempt_mode ==
			NVGPU_PREEMPTION_MODE_COMPUTE_CILP);
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"SM Exception received on gpc %d tpc %d sm %d = 0x%08x",
			gpc, tpc, sm, global_esr);

	if (cilp_enabled && sm_debugger_attached) {
		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f()) {
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f());
		}

		if (global_esr & gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f()) {
			gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
					gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f());
		}

		global_mask = gr_gpc0_tpc0_sm0_hww_global_esr_multiple_warp_errors_pending_f() |
			gr_gpc0_tpc0_sm0_hww_global_esr_bpt_pause_pending_f();

		if (warp_esr != 0 || (global_esr & global_mask) != 0) {
			*ignore_debugger = true;

			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: starting wait for LOCKED_DOWN on "
				"gpc %d tpc %d sm %d",
				gpc, tpc, sm);

			if (gk20a_dbg_gpu_broadcast_stop_trigger(fault_ch)) {
				nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: Broadcasting STOP_TRIGGER from "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);
				g->ops.gr.suspend_all_sms(g,
						global_mask, false);

				gk20a_dbg_gpu_clear_broadcast_stop_trigger(fault_ch);
			} else {
				nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: STOP_TRIGGER from "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);
				g->ops.gr.suspend_single_sm(g,
					gpc, tpc, sm, global_mask, true);
			}

			/* reset the HWW errors after locking down */
			global_esr_copy = g->ops.gr.get_sm_hww_global_esr(g,
							gpc, tpc, sm);
			g->ops.gr.clear_sm_hww(g,
					gpc, tpc, sm, global_esr_copy);
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: HWWs cleared for "
					"gpc %d tpc %d sm %d",
					gpc, tpc, sm);

			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "CILP: Setting CILP preempt pending\n");
			ret = gr_gp10b_set_cilp_preempt_pending(g, fault_ch);
			if (ret) {
				nvgpu_err(g, "CILP: error while setting CILP preempt pending!");
				return ret;
			}

			dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
			if (dbgr_control0 & gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_enable_f()) {
				nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
					"CILP: clearing SINGLE_STEP_MODE "
					"before resume for gpc %d tpc %d sm %d",
						gpc, tpc, sm);
				dbgr_control0 = set_field(dbgr_control0,
						gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_m(),
						gr_gpc0_tpc0_sm0_dbgr_control0_single_step_mode_disable_f());
				gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset, dbgr_control0);
			}

			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: resume for gpc %d tpc %d sm %d",
					gpc, tpc, sm);
			g->ops.gr.resume_single_sm(g, gpc, tpc, sm);

			*ignore_debugger = true;
			nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
				"CILP: All done on gpc %d, tpc %d sm %d",
				gpc, tpc, sm);
		}

		*early_exit = true;
	}
	return 0;
}

static void gr_gv11b_handle_fecs_ecc_error(struct gk20a *g, u32 intr)
{
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;

	if (intr & (gr_fecs_host_int_status_ecc_uncorrected_m() |
		    gr_fecs_host_int_status_ecc_corrected_m())) {
		ecc_status = gk20a_readl(g, gr_fecs_falcon_ecc_status_r());
		ecc_addr = gk20a_readl(g,
			gr_fecs_falcon_ecc_address_r());
		corrected_cnt = gk20a_readl(g,
			gr_fecs_falcon_ecc_corrected_err_count_r());
		uncorrected_cnt = gk20a_readl(g,
			gr_fecs_falcon_ecc_uncorrected_err_count_r());

		corrected_delta =
			gr_fecs_falcon_ecc_corrected_err_count_total_v(
							corrected_cnt);
		uncorrected_delta =
			gr_fecs_falcon_ecc_uncorrected_err_count_total_v(
							uncorrected_cnt);

		corrected_overflow = ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_total_counter_overflow_m();
		uncorrected_overflow = ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_total_counter_overflow_m();

		/* clear the interrupt */
		if ((corrected_delta > 0) || corrected_overflow) {
			gk20a_writel(g,
				gr_fecs_falcon_ecc_corrected_err_count_r(), 0);
		}
		if ((uncorrected_delta > 0) || uncorrected_overflow) {
			gk20a_writel(g,
				gr_fecs_falcon_ecc_uncorrected_err_count_r(),
				0);
		}


		/* clear the interrupt */
		gk20a_writel(g, gr_fecs_falcon_ecc_uncorrected_err_count_r(),
				0);
		gk20a_writel(g, gr_fecs_falcon_ecc_corrected_err_count_r(), 0);

		/* clear the interrupt */
		gk20a_writel(g, gr_fecs_falcon_ecc_status_r(),
				gr_fecs_falcon_ecc_status_reset_task_f());

		g->ecc.gr.fecs_ecc_corrected_err_count[0].counter +=
							corrected_delta;
		g->ecc.gr.fecs_ecc_uncorrected_err_count[0].counter +=
							uncorrected_delta;

		nvgpu_log(g, gpu_dbg_intr,
			"fecs ecc interrupt intr: 0x%x", intr);

		if (ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_imem_m()) {
			nvgpu_log(g, gpu_dbg_intr, "imem ecc error corrected");
		}
		if (ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_imem_m()) {
			nvgpu_log(g, gpu_dbg_intr,
						"imem ecc error uncorrected");
		}
		if (ecc_status &
			gr_fecs_falcon_ecc_status_corrected_err_dmem_m()) {
			nvgpu_log(g, gpu_dbg_intr, "dmem ecc error corrected");
		}
		if (ecc_status &
			gr_fecs_falcon_ecc_status_uncorrected_err_dmem_m()) {
			nvgpu_log(g, gpu_dbg_intr,
						"dmem ecc error uncorrected");
		}
		if (corrected_overflow || uncorrected_overflow) {
			nvgpu_info(g, "fecs ecc counter overflow!");
		}

		nvgpu_log(g, gpu_dbg_intr,
			"ecc error row address: 0x%x",
			gr_fecs_falcon_ecc_address_row_address_v(ecc_addr));

		nvgpu_log(g, gpu_dbg_intr,
			"ecc error count corrected: %d, uncorrected %d",
			g->ecc.gr.fecs_ecc_corrected_err_count[0].counter,
			g->ecc.gr.fecs_ecc_uncorrected_err_count[0].counter);
	}
}

int gr_gv11b_handle_fecs_error(struct gk20a *g,
				struct channel_gk20a *__ch,
				struct gr_gk20a_isr_data *isr_data)
{
	u32 gr_fecs_intr = gk20a_readl(g, gr_fecs_host_int_status_r());
	int ret;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, " ");

	/* Handle ECC errors */
	gr_gv11b_handle_fecs_ecc_error(g, gr_fecs_intr);

	ret = gr_gp10b_handle_fecs_error(g, __ch, isr_data);

	return ret;
}

int gr_gv11b_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 map;
	u32 i, j;
	u32 mapreg_num, base, offset, mapregs;
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_TPC_PER_GPC);
	u32 num_tpcs = num_gpcs * num_tpc_per_gpc;

	nvgpu_log_fn(g, " ");

	if (!gr->map_tiles) {
		return -1;
	}

	gk20a_writel(g, gr_crstr_map_table_cfg_r(),
		gr_crstr_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_crstr_map_table_cfg_num_entries_f(gr->tpc_count));
	/*
	 * 6 tpc can be stored in one map register.
	 * But number of tpcs are not always multiple of six,
	 * so adding additional check for valid number of
	 * tpcs before programming map register.
	 */
	mapregs = DIV_ROUND_UP(num_tpcs, GR_TPCS_INFO_FOR_MAPREGISTER);

	for (mapreg_num = 0, base = 0; mapreg_num < mapregs; mapreg_num++,
				base = base + GR_TPCS_INFO_FOR_MAPREGISTER) {
		map = 0;
		for (offset = 0;
			(offset < GR_TPCS_INFO_FOR_MAPREGISTER && num_tpcs > 0);
			offset++, num_tpcs--) {
			switch (offset) {
			case 0:
				map =  map | gr_crstr_gpc_map_tile0_f(
						gr->map_tiles[base + offset]);
				break;
			case 1:
				map =  map | gr_crstr_gpc_map_tile1_f(
						gr->map_tiles[base + offset]);
				break;
			case 2:
				map =  map | gr_crstr_gpc_map_tile2_f(
						gr->map_tiles[base + offset]);
				break;
			case 3:
				map =  map | gr_crstr_gpc_map_tile3_f(
						gr->map_tiles[base + offset]);
				break;
			case 4:
				map =  map | gr_crstr_gpc_map_tile4_f(
						gr->map_tiles[base + offset]);
				break;
			case 5:
				map =  map | gr_crstr_gpc_map_tile5_f(
						gr->map_tiles[base + offset]);
				break;
			default:
				nvgpu_err(g, "incorrect rop mapping %x", offset);
				break;
			}
		}

		gk20a_writel(g, gr_crstr_gpc_map_r(mapreg_num), map);
		gk20a_writel(g, gr_ppcs_wwdx_map_gpc_map_r(mapreg_num), map);
		gk20a_writel(g, gr_rstr2d_gpc_map_r(mapreg_num), map);
	}

	gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg_r(),
		gr_ppcs_wwdx_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_ppcs_wwdx_map_table_cfg_num_entries_f(gr->tpc_count));

	for (i = 0, j = 1; i < gr_ppcs_wwdx_map_table_cfg_coeff__size_1_v();
					i++, j = j + 4) {
		gk20a_writel(g, gr_ppcs_wwdx_map_table_cfg_coeff_r(i),
			gr_ppcs_wwdx_map_table_cfg_coeff_0_mod_value_f(
					((1 << j) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_1_mod_value_f(
				((1 << (j + 1)) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_2_mod_value_f(
				((1 << (j + 2)) % gr->tpc_count)) |
			gr_ppcs_wwdx_map_table_cfg_coeff_3_mod_value_f(
				((1 << (j + 3)) % gr->tpc_count)));
	}

	gk20a_writel(g, gr_rstr2d_map_table_cfg_r(),
		gr_rstr2d_map_table_cfg_row_offset_f(gr->map_row_offset) |
		gr_rstr2d_map_table_cfg_num_entries_f(gr->tpc_count));

	return 0;
}

static int gv11b_write_bundle_veid_state(struct gk20a *g, u32 index)
{
	struct av_list_gk20a *sw_veid_bundle_init =
			&g->gr.ctx_vars.sw_veid_bundle_init;
	u32 j;
	u32  num_subctx, err = 0;

	num_subctx = g->fifo.max_subctx_count;

	for (j = 0; j < num_subctx; j++) {
		nvgpu_log_fn(g, "write bundle_address_r for subctx: %d", j);
		gk20a_writel(g, gr_pipe_bundle_address_r(),
			sw_veid_bundle_init->l[index].addr |
			gr_pipe_bundle_address_veid_f(j));

		err = gr_gk20a_wait_fe_idle(g, gk20a_get_gr_idle_timeout(g),
					    GR_IDLE_CHECK_DEFAULT);
	}
	return err;
}

int gr_gv11b_init_sw_veid_bundle(struct gk20a *g)
{
	struct av_list_gk20a *sw_veid_bundle_init =
			&g->gr.ctx_vars.sw_veid_bundle_init;
	u32 i;
	u32 last_bundle_data = 0;
	u32 err = 0;

	for (i = 0; i < sw_veid_bundle_init->count; i++) {
		nvgpu_log_fn(g, "veid bundle count: %d", i);

		if (i == 0 || last_bundle_data !=
				sw_veid_bundle_init->l[i].value) {
			gk20a_writel(g, gr_pipe_bundle_data_r(),
				sw_veid_bundle_init->l[i].value);
			last_bundle_data = sw_veid_bundle_init->l[i].value;
			nvgpu_log_fn(g, "last_bundle_data : 0x%08x",
						last_bundle_data);
		}

		if (gr_pipe_bundle_address_value_v(
			sw_veid_bundle_init->l[i].addr) == GR_GO_IDLE_BUNDLE) {
			nvgpu_log_fn(g, "go idle bundle");
				gk20a_writel(g, gr_pipe_bundle_address_r(),
					sw_veid_bundle_init->l[i].addr);
				err |= gr_gk20a_wait_idle(g,
						gk20a_get_gr_idle_timeout(g),
						GR_IDLE_CHECK_DEFAULT);
		} else {
			err = gv11b_write_bundle_veid_state(g, i);
		}

		if (err) {
			nvgpu_err(g, "failed to init sw veid bundle");
			break;
		}
	}
	return err;
}

void gr_gv11b_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles)
{
	u32 val, i, j;

	nvgpu_log_fn(g, " ");

	for (i = 0, j = 0; i < (zcull_num_entries / 8); i++, j += 8) {
		val =
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_0_f(
						zcull_map_tiles[j+0]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_1_f(
						zcull_map_tiles[j+1]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_2_f(
						zcull_map_tiles[j+2]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_3_f(
						zcull_map_tiles[j+3]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_4_f(
						zcull_map_tiles[j+4]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_5_f(
						zcull_map_tiles[j+5]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_6_f(
						zcull_map_tiles[j+6]) |
		gr_gpcs_zcull_sm_in_gpc_number_map_tile_7_f(
						zcull_map_tiles[j+7]);

		gk20a_writel(g, gr_gpcs_zcull_sm_in_gpc_number_map_r(i), val);
	}
}

void gr_gv11b_detect_sm_arch(struct gk20a *g)
{
	u32 v = gk20a_readl(g, gr_gpc0_tpc0_sm_arch_r());

	g->params.sm_arch_spa_version =
		gr_gpc0_tpc0_sm_arch_spa_version_v(v);
	g->params.sm_arch_sm_version =
		gr_gpc0_tpc0_sm_arch_sm_version_v(v);
	g->params.sm_arch_warp_count =
		gr_gpc0_tpc0_sm_arch_warp_count_v(v);
}

u32 gr_gv11b_get_nonpes_aware_tpc(struct gk20a *g, u32 gpc, u32 tpc)
{
	u32 tpc_new = 0;
	u32 temp;
	u32 pes;
	struct gr_gk20a *gr = &g->gr;

	for (pes = 0; pes < gr->gpc_ppc_count[gpc]; pes++) {
		if (gr->pes_tpc_mask[pes][gpc] & BIT(tpc)) {
			break;
		}
		tpc_new += gr->pes_tpc_count[pes][gpc];
	}
	temp = (BIT(tpc) - 1) & gr->pes_tpc_mask[pes][gpc];
	temp = hweight32(temp);
	tpc_new += temp;

	nvgpu_log_info(g, "tpc: %d -> new tpc: %d", tpc, tpc_new);
	return tpc_new;
}

void gr_gv11b_program_sm_id_numbering(struct gk20a *g,
					u32 gpc, u32 tpc, u32 smid)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 gpc_offset = gpc_stride * gpc;
	u32 global_tpc_index = g->gr.sm_to_cluster[smid].global_tpc_index;
	u32 tpc_offset;

	tpc = g->ops.gr.get_nonpes_aware_tpc(g, gpc, tpc);
	tpc_offset = tpc_in_gpc_stride * tpc;

	gk20a_writel(g, gr_gpc0_tpc0_sm_cfg_r() + gpc_offset + tpc_offset,
			gr_gpc0_tpc0_sm_cfg_tpc_id_f(global_tpc_index));
	gk20a_writel(g, gr_gpc0_gpm_pd_sm_id_r(tpc) + gpc_offset,
			gr_gpc0_gpm_pd_sm_id_id_f(global_tpc_index));
	gk20a_writel(g, gr_gpc0_tpc0_pe_cfg_smid_r() + gpc_offset + tpc_offset,
			gr_gpc0_tpc0_pe_cfg_smid_value_f(global_tpc_index));
}

int gr_gv11b_load_smid_config(struct gk20a *g)
{
	u32 *tpc_sm_id;
	u32 i, j;
	u32 tpc_index, gpc_index, tpc_id;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	int num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);

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
			u32 sm_id;
			u32 bits;

			tpc_id = (i << 2) + j;
			sm_id = tpc_id * sm_per_tpc;

			if (sm_id >= g->gr.no_of_sm) {
				break;
			}

			gpc_index = g->gr.sm_to_cluster[sm_id].gpc_index;
			tpc_index = g->gr.sm_to_cluster[sm_id].tpc_index;

			bits = gr_cwd_gpc_tpc_id_gpc0_f(gpc_index) |
				gr_cwd_gpc_tpc_id_tpc0_f(tpc_index);
			reg |= bits << (j * bit_stride);

			tpc_sm_id[gpc_index + (num_gpcs * ((tpc_index & 4)
				 >> 2))] |= tpc_id << tpc_index * bit_stride;
		}
		gk20a_writel(g, gr_cwd_gpc_tpc_id_r(i), reg);
	}

	for (i = 0; i < gr_cwd_sm_id__size_1_v(); i++) {
		gk20a_writel(g, gr_cwd_sm_id_r(i), tpc_sm_id[i]);
        }
	nvgpu_kfree(g, tpc_sm_id);

	return 0;
}

int gr_gv11b_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	u32 addr_lo;
	u32 addr_hi;
	struct nvgpu_mem *ctxheader;
	int err;
	struct gk20a *g = c->g;

	nvgpu_log_fn(g, " ");

	err = gv11b_alloc_subctx_header(c);
	if (err) {
		return err;
	}

	err = gv11b_update_subctx_header(c, gpu_va);
	if (err) {
		return err;
	}

	ctxheader = &c->ctx_header;
	addr_lo = u64_lo32(ctxheader->gpu_va) >> ram_in_base_shift_v();
	addr_hi = u64_hi32(ctxheader->gpu_va);

	/* point this address to engine_wfi_ptr */
	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_engine_wfi_target_w(),
		ram_in_engine_cs_wfi_v() |
		ram_in_engine_wfi_mode_f(ram_in_engine_wfi_mode_virtual_v()) |
		ram_in_engine_wfi_ptr_lo_f(addr_lo));

	nvgpu_mem_wr32(c->g, &c->inst_block, ram_in_engine_wfi_ptr_hi_w(),
		ram_in_engine_wfi_ptr_hi_f(addr_hi));

	return 0;
}



int gr_gv11b_commit_global_timeslice(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_gr_ctx *ch_ctx = NULL;
	u32 pd_ab_dist_cfg0;
	u32 ds_debug;
	u32 mpc_vtg_debug;
	u32 pe_vaf;
	u32 pe_vsc_vpc;

	nvgpu_log_fn(g, " ");

	pd_ab_dist_cfg0 = gk20a_readl(g, gr_pd_ab_dist_cfg0_r());
	ds_debug = gk20a_readl(g, gr_ds_debug_r());
	mpc_vtg_debug = gk20a_readl(g, gr_gpcs_tpcs_mpc_vtg_debug_r());

	pe_vaf = gk20a_readl(g, gr_gpcs_tpcs_pe_vaf_r());
	pe_vsc_vpc = gk20a_readl(g, gr_gpcs_tpcs_pes_vsc_vpc_r());

	pe_vaf = gr_gpcs_tpcs_pe_vaf_fast_mode_switch_true_f() | pe_vaf;
	pe_vsc_vpc = gr_gpcs_tpcs_pes_vsc_vpc_fast_mode_switch_true_f() |
								pe_vsc_vpc;
	pd_ab_dist_cfg0 = gr_pd_ab_dist_cfg0_timeslice_enable_en_f() |
							pd_ab_dist_cfg0;
	ds_debug = gr_ds_debug_timeslice_mode_enable_f() | ds_debug;
	mpc_vtg_debug = gr_gpcs_tpcs_mpc_vtg_debug_timeslice_mode_enabled_f() |
							mpc_vtg_debug;

	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pe_vaf_r(), pe_vaf,
									false);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_pes_vsc_vpc_r(),
							pe_vsc_vpc, false);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_pd_ab_dist_cfg0_r(),
							pd_ab_dist_cfg0, false);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_ds_debug_r(), ds_debug, false);
	gr_gk20a_ctx_patch_write(g, ch_ctx, gr_gpcs_tpcs_mpc_vtg_debug_r(),
							mpc_vtg_debug, false);

	return 0;
}

void gr_gv11b_write_zcull_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va_lo, va_hi;

	gpu_va = gpu_va >> 8;
	va_lo = u64_lo32(gpu_va);
	va_hi = u64_hi32(gpu_va);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_zcull_ptr_o(), va_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_zcull_ptr_hi_o(), va_hi);
}


void gr_gv11b_write_pm_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 va_lo, va_hi;

	gpu_va = gpu_va >> 8;
	va_lo = u64_lo32(gpu_va);
	va_hi = u64_hi32(gpu_va);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_ptr_o(), va_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_pm_ptr_hi_o(), va_hi);
}

void gr_gv11b_load_tpc_mask(struct gk20a *g)
{
	u32 pes_tpc_mask = 0, fuse_tpc_mask;
	u32 gpc, pes, val;
	u32 num_tpc_per_gpc = nvgpu_get_litter_value(g,
				GPU_LIT_NUM_TPC_PER_GPC);

	/* gv11b has 1 GPC and 4 TPC/GPC, so mask will not overflow u32 */
	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		for (pes = 0; pes < g->gr.pe_count_per_gpc; pes++) {
			pes_tpc_mask |= g->gr.pes_tpc_mask[pes][gpc] <<
				num_tpc_per_gpc * gpc;
		}
	}

	nvgpu_log_info(g, "pes_tpc_mask %u\n", pes_tpc_mask);
	fuse_tpc_mask = g->ops.gr.get_gpc_tpc_mask(g, gpc);
	if (g->tpc_fs_mask_user &&
			g->tpc_fs_mask_user != fuse_tpc_mask &&
			fuse_tpc_mask == (0x1U << g->gr.max_tpc_count) - 1U) {
		val = g->tpc_fs_mask_user;
		val &= (0x1U << g->gr.max_tpc_count) - 1U;
		val = (0x1U << hweight32(val)) - 1U;
		gk20a_writel(g, gr_fe_tpc_fs_r(0), val);
	} else {
		gk20a_writel(g, gr_fe_tpc_fs_r(0), pes_tpc_mask);
	}

}

void gr_gv11b_set_preemption_buffer_va(struct gk20a *g,
			struct nvgpu_mem *mem, u64 gpu_va)
{
	u32 addr_lo, addr_hi;

	/* gpu va still needs to be 8 bit aligned */
	gpu_va = gpu_va >> 8;

	addr_lo = u64_lo32(gpu_va);
	addr_hi = u64_hi32(gpu_va);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_o(), addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_hi_o(), addr_hi);

	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_veid0_o(), addr_lo);
	nvgpu_mem_wr(g, mem,
		ctxsw_prog_main_image_full_preemption_ptr_veid0_hi_o(),
		addr_hi);

}

int gr_gv11b_init_fs_state(struct gk20a *g)
{
	u32 data;
	int err;
	u32 ver = g->params.gpu_arch + g->params.gpu_impl;

	nvgpu_log_fn(g, " ");

	data = gk20a_readl(g, gr_gpcs_tpcs_sm_texio_control_r());
	data = set_field(data, gr_gpcs_tpcs_sm_texio_control_oor_addr_check_mode_m(),
			gr_gpcs_tpcs_sm_texio_control_oor_addr_check_mode_arm_63_48_match_f());
	gk20a_writel(g, gr_gpcs_tpcs_sm_texio_control_r(), data);

	if (ver == NVGPU_GPUID_GV11B && nvgpu_is_soc_t194_a01(g))
	{
		/* Disable CBM alpha and beta invalidations for l2 for t194 A01 */
		data = gk20a_readl(g, gr_gpcs_ppcs_cbm_debug_r());
		data = set_field(data,
			gr_gpcs_ppcs_cbm_debug_invalidate_alpha_m(),
			gr_gpcs_ppcs_cbm_debug_invalidate_alpha_disable_f());
		data = set_field(data,
			gr_gpcs_ppcs_cbm_debug_invalidate_beta_m(),
			gr_gpcs_ppcs_cbm_debug_invalidate_beta_disable_f());
		gk20a_writel(g, gr_gpcs_ppcs_cbm_debug_r(), data);

		/* Disable SCC pagepool invalidates  for t194 A01 */
		data = gk20a_readl(g, gr_scc_debug_r());
		data = set_field(data,
			gr_scc_debug_pagepool_invalidates_m(),
			gr_scc_debug_pagepool_invalidates_disable_f());
		gk20a_writel(g, gr_scc_debug_r(), data);

		/* Disable SWDX spill buffer invalidates */
		data = gk20a_readl(g, gr_gpcs_swdx_spill_unit_r());
		data = set_field(data,
			gr_gpcs_swdx_spill_unit_spill_buffer_cache_mgmt_mode_m(),
			gr_gpcs_swdx_spill_unit_spill_buffer_cache_mgmt_mode_disabled_f());
		gk20a_writel(g, gr_gpcs_swdx_spill_unit_r(), data);
	}

	data = gk20a_readl(g, gr_gpcs_tpcs_sm_disp_ctrl_r());
	data = set_field(data, gr_gpcs_tpcs_sm_disp_ctrl_re_suppress_m(),
			 gr_gpcs_tpcs_sm_disp_ctrl_re_suppress_disable_f());
	gk20a_writel(g, gr_gpcs_tpcs_sm_disp_ctrl_r(), data);

	if (g->gr.fecs_feature_override_ecc_val != 0) {
		gk20a_writel(g,
			gr_fecs_feature_override_ecc_r(),
			g->gr.fecs_feature_override_ecc_val);
	}

	err = gr_gk20a_init_fs_state(g);
	if (err) {
		return err;
	}

	g->ops.gr.load_tpc_mask(g);

	gk20a_writel(g, gr_bes_zrop_settings_r(),
		     gr_bes_zrop_settings_num_active_ltcs_f(g->ltc_count));
	gk20a_writel(g, gr_bes_crop_settings_r(),
		     gr_bes_crop_settings_num_active_ltcs_f(g->ltc_count));

	g->ops.gr.load_smid_config(g);

	return err;
}

void gv11b_gr_get_esr_sm_sel(struct gk20a *g, u32 gpc, u32 tpc,
				u32 *esr_sm_sel)
{
	u32 reg_val;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);

	reg_val = gk20a_readl(g, gr_gpc0_tpc0_sm_tpc_esr_sm_sel_r() + offset);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"sm tpc esr sm sel reg val: 0x%x", reg_val);
	*esr_sm_sel = 0;
	if (gr_gpc0_tpc0_sm_tpc_esr_sm_sel_sm0_error_v(reg_val)) {
		*esr_sm_sel = 1;
	}
	if (gr_gpc0_tpc0_sm_tpc_esr_sm_sel_sm1_error_v(reg_val)) {
		*esr_sm_sel |= 1 << 1;
	}
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"esr_sm_sel bitmask: 0x%x", *esr_sm_sel);
}

int gv11b_gr_sm_trigger_suspend(struct gk20a *g)
{
	u32 dbgr_control0;

	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0.
	 */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"stop trigger enable: broadcast dbgr_control0: 0x%x ",
			dbgr_control0);

	return 0;
}

void gv11b_gr_bpt_reg_info(struct gk20a *g, struct nvgpu_warpstate *w_state)
{
	/* Check if we have at least one valid warp
	 * get paused state on maxwell
	 */
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm, sm_id;
	u32 offset;
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;

	for (sm_id = 0; sm_id < gr->no_of_sm; sm_id++) {
		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

		/* 64 bit read */
		warps_valid = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_warp_valid_mask_1_r() +
				offset) << 32;
		warps_valid |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_warp_valid_mask_0_r() +
				offset);

		/* 64 bit read */
		warps_paused = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_1_r() +
				offset) << 32;
		warps_paused |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_0_r() +
				offset);

		/* 64 bit read */
		warps_trapped = (u64)gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_1_r() +
				offset) << 32;
		warps_trapped |= gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_0_r() +
				offset);

		w_state[sm_id].valid_warps[0] = warps_valid;
		w_state[sm_id].trapped_warps[0] = warps_trapped;
		w_state[sm_id].paused_warps[0] = warps_paused;
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

int gv11b_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable)
{
	struct nvgpu_dbg_reg_op *ops;
	unsigned int i = 0, sm_id;
	int err;

	ops = nvgpu_kcalloc(g, g->gr.no_of_sm, sizeof(*ops));
	if (!ops) {
		return -ENOMEM;
	}
	for (sm_id = 0; sm_id < g->gr.no_of_sm; sm_id++) {
		u32 gpc, tpc, sm;
		u32 reg_offset, reg_mask, reg_val;

		if (!(sms & (1 << sm_id))) {
			continue;
		}

		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		if (g->ops.gr.get_nonpes_aware_tpc != NULL) {
			tpc = g->ops.gr.get_nonpes_aware_tpc(g,
					g->gr.sm_to_cluster[sm_id].gpc_index,
					g->gr.sm_to_cluster[sm_id].tpc_index);
		} else {
			tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		}
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		reg_offset = gk20a_gr_gpc_offset(g, gpc) +
				gk20a_gr_tpc_offset(g, tpc) +
				gv11b_gr_sm_offset(g, sm);

		ops[i].op = REGOP(WRITE_32);
		ops[i].type = REGOP(TYPE_GR_CTX);
		ops[i].offset = gr_gpc0_tpc0_sm0_dbgr_control0_r() + reg_offset;

		reg_mask = 0;
		reg_val = 0;
		if (enable) {
			nvgpu_log(g, gpu_dbg_gpu_dbg,
				"SM:%d debuggger mode ON", sm);
			reg_mask |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_m();
			reg_val |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_on_f();
		} else {
			nvgpu_log(g, gpu_dbg_gpu_dbg,
				"SM:%d debuggger mode Off", sm);
			reg_mask |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_m();
			reg_val |=
			 gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_off_f();
		}

		ops[i].and_n_mask_lo = reg_mask;
		ops[i].value_lo = reg_val;
		i++;
	}

	err = gr_gk20a_exec_ctx_ops(ch, ops, i, i, 0, NULL);
	if (err) {
		nvgpu_err(g, "Failed to access register\n");
	}
	nvgpu_kfree(g, ops);
	return err;
}

static void gv11b_gr_read_sm_error_state(struct gk20a *g,
			u32 offset,
			struct nvgpu_tsg_sm_error_state *sm_error_states)
{
	sm_error_states->hww_global_esr = nvgpu_readl(g,
		gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);

	sm_error_states->hww_warp_esr = nvgpu_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset);

	sm_error_states->hww_warp_esr_pc = hi32_lo32_to_u64((nvgpu_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_pc_hi_r() + offset)),
		(nvgpu_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_pc_r() + offset)));

	sm_error_states->hww_global_esr_report_mask = nvgpu_readl(g,
	       gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_r() + offset);

	sm_error_states->hww_warp_esr_report_mask = nvgpu_readl(g,
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_r() + offset);
}

int gv11b_gr_record_sm_error_state(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
				struct channel_gk20a *fault_ch)
{
	int sm_id;
	u32 offset, sm_per_tpc, tpc_id;
	u32 gpc_offset, gpc_tpc_offset;
	struct nvgpu_tsg_sm_error_state *sm_error_states = NULL;
	struct tsg_gk20a *tsg = NULL;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	gpc_offset = gk20a_gr_gpc_offset(g, gpc);
	gpc_tpc_offset = gpc_offset + gk20a_gr_tpc_offset(g, tpc);

	tpc_id = gk20a_readl(g, gr_gpc0_gpm_pd_sm_id_r(tpc) + gpc_offset);
	sm_id = tpc_id * sm_per_tpc + sm;

	offset = gpc_tpc_offset + gv11b_gr_sm_offset(g, sm);

	if (fault_ch != NULL) {
		tsg = tsg_gk20a_from_ch(fault_ch);
	}

	if (tsg == NULL) {
		nvgpu_err(g, "no valid tsg");
		goto record_fail;
	}

	sm_error_states = tsg->sm_error_states + sm_id;
	gv11b_gr_read_sm_error_state(g, offset, sm_error_states);

record_fail:
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return sm_id;
}

void gv11b_gr_set_hww_esr_report_mask(struct gk20a *g)
{

	/* clear hww */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_r(), 0xffffffff);
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_r(), 0xffffffff);

	/* setup sm warp esr report masks */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_warp_esr_report_mask_r(),
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_stack_error_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_api_stack_error_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_pc_wrap_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_pc_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_pc_overflow_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_reg_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_illegal_instr_encoding_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_illegal_instr_param_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_oor_reg_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_oor_addr_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_misaligned_addr_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_invalid_addr_space_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_invalid_const_addr_ldc_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_stack_overflow_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_mmu_fault_report_f() |
		gr_gpc0_tpc0_sm0_hww_warp_esr_report_mask_mmu_nack_report_f());

	/* setup sm global esr report mask. vat_alarm_report is not enabled */
	gk20a_writel(g, gr_gpcs_tpcs_sms_hww_global_esr_report_mask_r(),
		gr_gpc0_tpc0_sm0_hww_global_esr_report_mask_multiple_warp_errors_report_f());
}

bool gv11b_gr_sm_debugger_attached(struct gk20a *g)
{
	u32 debugger_mode;
	u32 dbgr_control0 = gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());

	/* check if sm debugger is attached.
	 * assumption: all SMs will have debug mode enabled/disabled
	 * uniformly.
	 */
	debugger_mode =
		gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_v(dbgr_control0);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"SM Debugger Mode: %d", debugger_mode);
	if (debugger_mode ==
			gr_gpc0_tpc0_sm0_dbgr_control0_debugger_mode_on_v()) {
		return true;
	}

	return false;
}

void gv11b_gr_suspend_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	int err;
	u32 dbgr_control0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
		"suspending gpc:%d, tpc:%d, sm%d", gpc, tpc, sm);

	/* assert stop trigger. */
	dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset,
			dbgr_control0);

	err = g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm,
			global_esr_mask, check_errors);
	if (err) {
		nvgpu_err(g,
			"SuspendSm failed");
		return;
	}
}

void gv11b_gr_suspend_all_sms(struct gk20a *g,
		u32 global_esr_mask, bool check_errors)
{
	struct gr_gk20a *gr = &g->gr;
	u32 gpc, tpc, sm;
	int err;
	u32 dbgr_control0;
	u32 sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);

	/* if an SM debugger isn't attached, skip suspend */
	if (!g->ops.gr.sm_debugger_attached(g)) {
		nvgpu_err(g,
			"SM debugger not attached, skipping suspend!");
		return;
	}

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "suspending all sms");

	/* assert stop trigger. uniformity assumption: all SMs will have
	 * the same state in dbg_control0.
	 */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();

	/* broadcast write */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		for (tpc = 0; tpc < gr_gk20a_get_tpc_count(gr, gpc); tpc++) {
			for (sm = 0; sm < sm_per_tpc; sm++) {
				err = g->ops.gr.wait_for_sm_lock_down(g,
					gpc, tpc, sm,
					global_esr_mask, check_errors);
				if (err) {
					nvgpu_err(g,
						"SuspendAllSms failed");
					return;
				}
			}
		}
	}
}

void gv11b_gr_resume_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm)
{
	u32 dbgr_control0, dbgr_status0;
	u32 offset;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"resuming gpc:%d, tpc:%d, sm%d", gpc, tpc, sm);
	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before stop trigger disable: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

	/*De-assert stop trigger */
	dbgr_control0 = set_field(dbgr_control0,
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_m(),
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_disable_f());
	gk20a_writel(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() +
				offset, dbgr_control0);

	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
	/* Run trigger */
	dbgr_control0 |=
		gr_gpc0_tpc0_sm0_dbgr_control0_run_trigger_task_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() +
		offset, dbgr_control0);

	dbgr_control0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_status0 = gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);
	/* run trigger is not sticky bit. SM clears it immediately */
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"after run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

}

void gv11b_gr_resume_all_sms(struct gk20a *g)
{
	u32 dbgr_control0, dbgr_status0;
	/*
	 * The following requires some clarification. Despite the fact that both
	 * RUN_TRIGGER and STOP_TRIGGER have the word "TRIGGER" in their
	 *  names, only one is actually a trigger, and that is the STOP_TRIGGER.
	 * Merely writing a 1(_TASK) to the RUN_TRIGGER is not sufficient to
	 * resume the gpu - the _STOP_TRIGGER must explicitly be set to 0
	 * (_DISABLE) as well.

	* Advice from the arch group:  Disable the stop trigger first, as a
	* separate operation, in order to ensure that the trigger has taken
	* effect, before enabling the run trigger.
	*/

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "resuming all sms");

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before stop trigger disable: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);

	dbgr_control0 = set_field(dbgr_control0,
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_m(),
			gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_disable_f());
	/* Write to broadcast registers */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"before run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
	/* Run trigger */
	dbgr_control0 |=
		gr_gpc0_tpc0_sm0_dbgr_control0_run_trigger_task_f();
	/* Write to broadcast registers */
	gk20a_writel(g,
		gr_gpcs_tpcs_sms_dbgr_control0_r(), dbgr_control0);

	/* Read from unicast registers */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r());
	dbgr_status0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_status0_r());
	/* run trigger is not sticky bit. SM clears it immediately */
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"after run trigger: "
			"dbgr_control0 = 0x%x dbgr_status0: 0x%x",
			dbgr_control0, dbgr_status0);
}

int gv11b_gr_resume_from_pause(struct gk20a *g)
{
	int err = 0;
	u32 reg_val;

	/* Clear the pause mask to tell the GPU we want to resume everyone */
	gk20a_writel(g, gr_gpcs_tpcs_sms_dbgr_bpt_pause_mask_0_r(), 0);

	/* explicitly re-enable forwarding of SM interrupts upon any resume */
	reg_val = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_en_r());
	reg_val |= gr_gpc0_tpc0_tpccs_tpc_exception_en_sm_enabled_f();

	gk20a_writel(g, gr_gpcs_tpcs_tpccs_tpc_exception_en_r(), reg_val);

	g->ops.gr.resume_all_sms(g);

	return err;
}

u32 gv11b_gr_get_sm_hww_warp_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

	u32 hww_warp_esr = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset);
	return hww_warp_esr;
}

u32 gv11b_gr_get_sm_hww_global_esr(struct gk20a *g,
			u32 gpc, u32 tpc, u32 sm)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			 gk20a_gr_tpc_offset(g, tpc) +
			 gv11b_gr_sm_offset(g, sm);

	u32 hww_global_esr = gk20a_readl(g,
				 gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);

	return hww_global_esr;
}

u32 gv11b_gr_get_sm_no_lock_down_hww_global_esr_mask(struct gk20a *g)
{
	/*
	 * These three interrupts don't require locking down the SM. They can
	 * be handled by usermode clients as they aren't fatal. Additionally,
	 * usermode clients may wish to allow some warps to execute while others
	 * are at breakpoints, as opposed to fatal errors where all warps should
	 * halt.
	 */
	u32 global_esr_mask =
		gr_gpc0_tpc0_sm0_hww_global_esr_bpt_int_pending_f()   |
		gr_gpc0_tpc0_sm0_hww_global_esr_bpt_pause_pending_f() |
		gr_gpc0_tpc0_sm0_hww_global_esr_single_step_complete_pending_f();

	return global_esr_mask;
}

static void gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(struct gk20a *g,
					u32 offset, bool timeout)
{
	u64 warps_valid = 0, warps_paused = 0, warps_trapped = 0;
	u32 dbgr_control0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	u32 dbgr_status0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);
	/* 64 bit read */
	warps_valid =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_warp_valid_mask_1_r() +
					offset) << 32;
	warps_valid |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_warp_valid_mask_0_r() + offset);

	/* 64 bit read */
	warps_paused =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_1_r() +
					offset) << 32;
	warps_paused |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_bpt_pause_mask_0_r() + offset);

	/* 64 bit read */
	warps_trapped =
		(u64)gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_1_r() +
					offset) << 32;
	warps_trapped |= gk20a_readl(g,
			gr_gpc0_tpc0_sm0_dbgr_bpt_trap_mask_0_r() + offset);
	if (timeout) {
		nvgpu_err(g,
		"STATUS0=0x%x CONTROL0=0x%x VALID_MASK=0x%llx "
		"PAUSE_MASK=0x%llx TRAP_MASK=0x%llx\n",
		dbgr_status0, dbgr_control0, warps_valid,
		warps_paused, warps_trapped);
	} else {
		nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"STATUS0=0x%x CONTROL0=0x%x VALID_MASK=0x%llx "
		"PAUSE_MASK=0x%llx TRAP_MASK=0x%llx\n",
		dbgr_status0, dbgr_control0, warps_valid,
		warps_paused, warps_trapped);
	}
}

int gv11b_gr_wait_for_sm_lock_down(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors)
{
	bool locked_down;
	bool no_error_pending;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	bool mmu_debug_mode_enabled = g->ops.fb.is_debug_mode_enabled(g);
	u32 dbgr_status0 = 0;
	u32 warp_esr, global_esr;
	struct nvgpu_timeout timeout;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) +
			gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
		"GPC%d TPC%d: locking down SM%d", gpc, tpc, sm);

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	/* wait for the sm to lock down */
	do {
		global_esr = g->ops.gr.get_sm_hww_global_esr(g, gpc, tpc, sm);
		dbgr_status0 = gk20a_readl(g,
				gr_gpc0_tpc0_sm0_dbgr_status0_r() + offset);

		warp_esr = g->ops.gr.get_sm_hww_warp_esr(g, gpc, tpc, sm);

		locked_down =
		    (gr_gpc0_tpc0_sm0_dbgr_status0_locked_down_v(dbgr_status0) ==
		     gr_gpc0_tpc0_sm0_dbgr_status0_locked_down_true_v());
		no_error_pending =
			check_errors &&
			(gr_gpc0_tpc0_sm0_hww_warp_esr_error_v(warp_esr) ==
			 gr_gpc0_tpc0_sm0_hww_warp_esr_error_none_v()) &&
			((global_esr & global_esr_mask) == 0);

		if (locked_down) {
		/*
		 * if SM reports locked down, it means that SM is idle and
		 * trapped and also that one of the these conditions are true
		 * 1) sm is nonempty and all valid warps are paused
		 * 2) sm is empty and held in trapped state due to stop trigger
		 * 3) sm is nonempty and some warps are not paused, but are
		 *    instead held at RTT due to an "active" stop trigger
		 * Check for Paused warp mask != Valid
		 * warp mask after SM reports it is locked down in order to
		 * distinguish case 1 from case 3.  When case 3 is detected,
		 * it implies a misprogrammed trap handler code, as all warps
		 * in the handler must promise to BPT.PAUSE instead of RTT
		 * whenever SR64 read in trap mode indicates stop trigger
		 * is asserted.
		 */
			gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(g,
						offset, false);
		}

		if (locked_down || no_error_pending) {
			nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
				"GPC%d TPC%d: locked down SM%d", gpc, tpc, sm);
			return 0;
		}

		if (mmu_debug_mode_enabled &&
		    g->ops.fb.handle_replayable_fault != NULL) {
			g->ops.fb.handle_replayable_fault(g);
		} else {
			/* if an mmu fault is pending and mmu debug mode is not
			 * enabled, the sm will never lock down.
			 */
			if (g->ops.mm.mmu_fault_pending(g)) {
				nvgpu_err(g,
					"GPC%d TPC%d: mmu fault pending,"
					" SM%d will never lock down!",
					gpc, tpc, sm);
				return -EFAULT;
			}
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	nvgpu_err(g, "GPC%d TPC%d: timed out while trying to "
			"lock down SM%d", gpc, tpc, sm);
	gv11b_gr_sm_dump_warp_bpt_pause_trap_mask_regs(g, offset, true);

	return -ETIMEDOUT;
}

int gv11b_gr_lock_down_sm(struct gk20a *g,
			 u32 gpc, u32 tpc, u32 sm, u32 global_esr_mask,
			 bool check_errors)
{
	u32 dbgr_control0;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d SM%d: assert stop trigger", gpc, tpc, sm);

	/* assert stop trigger */
	dbgr_control0 =
		gk20a_readl(g, gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset);
	dbgr_control0 |= gr_gpc0_tpc0_sm0_dbgr_control0_stop_trigger_enable_f();
	gk20a_writel(g,
		gr_gpc0_tpc0_sm0_dbgr_control0_r() + offset, dbgr_control0);

	return g->ops.gr.wait_for_sm_lock_down(g, gpc, tpc, sm, global_esr_mask,
			check_errors);
}

void gv11b_gr_clear_sm_hww(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
				u32 global_esr)
{
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc) +
			gv11b_gr_sm_offset(g, sm);

	gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
			global_esr);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"Cleared HWW global esr, current reg val: 0x%x",
			gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() +
						offset));

	gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset, 0);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"Cleared HWW warp esr, current reg val: 0x%x",
			gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_warp_esr_r() +
						offset));
}

int gr_gv11b_handle_tpc_mpc_exception(struct gk20a *g,
		u32 gpc, u32 tpc, bool *post_event)
{
	u32 esr;
	u32 offset = gk20a_gr_gpc_offset(g, gpc) + gk20a_gr_tpc_offset(g, tpc);
	u32 tpc_exception = gk20a_readl(g, gr_gpc0_tpc0_tpccs_tpc_exception_r()
			+ offset);

	if (!(tpc_exception & gr_gpc0_tpc0_tpccs_tpc_exception_mpc_m())) {
		return 0;
	}

	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"GPC%d TPC%d MPC exception", gpc, tpc);

	esr = gk20a_readl(g, gr_gpc0_tpc0_mpc_hww_esr_r() + offset);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg, "mpc hww esr 0x%08x", esr);

	esr = gk20a_readl(g, gr_gpc0_tpc0_mpc_hww_esr_info_r() + offset);
	nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
			"mpc hww esr info: veid 0x%08x",
			gr_gpc0_tpc0_mpc_hww_esr_info_veid_v(esr));

	gk20a_writel(g, gr_gpc0_tpc0_mpc_hww_esr_r() + offset,
		     gr_gpc0_tpc0_mpc_hww_esr_reset_trigger_f());

	return 0;
}

static const u32 _num_ovr_perf_regs = 20;
static u32 _ovr_perf_regs[20] = { 0, };

void gv11b_gr_init_ovr_sm_dsm_perf(void)
{
	if (_ovr_perf_regs[0] != 0) {
		return;
	}

	_ovr_perf_regs[0]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control_sel0_r();
	_ovr_perf_regs[1]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control_sel1_r();
	_ovr_perf_regs[2]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control0_r();
	_ovr_perf_regs[3]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control1_r();
	_ovr_perf_regs[4]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control2_r();
	_ovr_perf_regs[5]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control3_r();
	_ovr_perf_regs[6]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control4_r();
	_ovr_perf_regs[7]  = gr_egpc0_etpc0_sm_dsm_perf_counter_control5_r();
	_ovr_perf_regs[8]  = gr_egpc0_etpc0_sm_dsm_perf_counter0_control_r();
	_ovr_perf_regs[9]  = gr_egpc0_etpc0_sm_dsm_perf_counter1_control_r();
	_ovr_perf_regs[10] = gr_egpc0_etpc0_sm_dsm_perf_counter2_control_r();
	_ovr_perf_regs[11] = gr_egpc0_etpc0_sm_dsm_perf_counter3_control_r();
	_ovr_perf_regs[12] = gr_egpc0_etpc0_sm_dsm_perf_counter4_control_r();
	_ovr_perf_regs[13] = gr_egpc0_etpc0_sm_dsm_perf_counter5_control_r();
	_ovr_perf_regs[14] = gr_egpc0_etpc0_sm_dsm_perf_counter6_control_r();
	_ovr_perf_regs[15] = gr_egpc0_etpc0_sm_dsm_perf_counter7_control_r();

	_ovr_perf_regs[16] = gr_egpc0_etpc0_sm0_dsm_perf_counter4_r();
	_ovr_perf_regs[17] = gr_egpc0_etpc0_sm0_dsm_perf_counter5_r();
	_ovr_perf_regs[18] = gr_egpc0_etpc0_sm0_dsm_perf_counter6_r();
	_ovr_perf_regs[19] = gr_egpc0_etpc0_sm0_dsm_perf_counter7_r();
}

/* Following are the blocks of registers that the ucode
 * stores in the extended region.
 */
/* ==  ctxsw_extended_sm_dsm_perf_counter_register_stride_v() ? */
static const u32 _num_sm_dsm_perf_regs;
/* ==  ctxsw_extended_sm_dsm_perf_counter_control_register_stride_v() ?*/
static const u32 _num_sm_dsm_perf_ctrl_regs = 2;
static u32 *_sm_dsm_perf_regs;
static u32 _sm_dsm_perf_ctrl_regs[2];

void gv11b_gr_init_sm_dsm_reg_info(void)
{
	if (_sm_dsm_perf_ctrl_regs[0] != 0) {
		return;
	}

	_sm_dsm_perf_ctrl_regs[0] =
			      gr_egpc0_etpc0_sm_dsm_perf_counter_control0_r();
	_sm_dsm_perf_ctrl_regs[1] =
			      gr_egpc0_etpc0_sm_dsm_perf_counter_control5_r();
}

void gv11b_gr_get_sm_dsm_perf_regs(struct gk20a *g,
					  u32 *num_sm_dsm_perf_regs,
					  u32 **sm_dsm_perf_regs,
					  u32 *perf_register_stride)
{
	*num_sm_dsm_perf_regs = _num_sm_dsm_perf_regs;
	*sm_dsm_perf_regs = _sm_dsm_perf_regs;
	*perf_register_stride =
		ctxsw_prog_extended_sm_dsm_perf_counter_register_stride_v();
}

void gv11b_gr_get_sm_dsm_perf_ctrl_regs(struct gk20a *g,
					       u32 *num_sm_dsm_perf_ctrl_regs,
					       u32 **sm_dsm_perf_ctrl_regs,
					       u32 *ctrl_register_stride)
{
	*num_sm_dsm_perf_ctrl_regs = _num_sm_dsm_perf_ctrl_regs;
	*sm_dsm_perf_ctrl_regs = _sm_dsm_perf_ctrl_regs;
	*ctrl_register_stride =
		ctxsw_prog_extended_sm_dsm_perf_counter_control_register_stride_v();
}

void gv11b_gr_get_ovr_perf_regs(struct gk20a *g, u32 *num_ovr_perf_regs,
					       u32 **ovr_perf_regs)
{
	*num_ovr_perf_regs = _num_ovr_perf_regs;
	*ovr_perf_regs = _ovr_perf_regs;
}

void gv11b_gr_access_smpc_reg(struct gk20a *g, u32 quad, u32 offset)
{
	u32 reg_val;
	u32 quad_ctrl;
	u32 half_ctrl;
	u32 tpc, gpc;
	u32 gpc_tpc_addr;
	u32 gpc_tpc_stride;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "offset=0x%x", offset);

	gpc = pri_get_gpc_num(g, offset);
	gpc_tpc_addr = pri_gpccs_addr_mask(offset);
	tpc = g->ops.gr.get_tpc_num(g, gpc_tpc_addr);

	quad_ctrl = quad & 0x1; /* first bit tells us quad */
	half_ctrl = (quad >> 1) & 0x1; /* second bit tells us half */

	gpc_tpc_stride = gpc * gpc_stride + tpc * tpc_in_gpc_stride;
	gpc_tpc_addr = gr_gpc0_tpc0_sm_halfctl_ctrl_r() + gpc_tpc_stride;

	/* read from unicast reg */
	reg_val = gk20a_readl(g, gpc_tpc_addr);
	reg_val = set_field(reg_val,
		gr_gpcs_tpcs_sm_halfctl_ctrl_sctl_read_quad_ctl_m(),
		gr_gpcs_tpcs_sm_halfctl_ctrl_sctl_read_quad_ctl_f(quad_ctrl));

	/* write to broadcast reg */
	gk20a_writel(g, gr_gpcs_tpcs_sm_halfctl_ctrl_r(), reg_val);

	gpc_tpc_addr = gr_gpc0_tpc0_sm_debug_sfe_control_r() + gpc_tpc_stride;
	reg_val = gk20a_readl(g, gpc_tpc_addr);
	reg_val = set_field(reg_val,
		gr_gpcs_tpcs_sm_debug_sfe_control_read_half_ctl_m(),
		gr_gpcs_tpcs_sm_debug_sfe_control_read_half_ctl_f(half_ctrl));

	/* write to broadcast reg */
	gk20a_writel(g, gr_gpcs_tpcs_sm_debug_sfe_control_r(), reg_val);
}

static bool pri_is_egpc_addr_shared(struct gk20a *g, u32 addr)
{
	u32 egpc_shared_base = EGPC_PRI_SHARED_BASE;
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	return (addr >= egpc_shared_base) &&
		(addr < egpc_shared_base + gpc_stride);
}

bool gv11b_gr_pri_is_egpc_addr(struct gk20a *g, u32 addr)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);

	return	((addr >= egpc_base) &&
		 (addr < egpc_base + num_gpcs * gpc_stride)) ||
		pri_is_egpc_addr_shared(g, addr);
}

static inline u32 pri_smpc_in_etpc_addr_mask(struct gk20a *g, u32 addr)
{
	u32 smpc_stride = nvgpu_get_litter_value(g,
				GPU_LIT_SMPC_PRI_STRIDE);

	return (addr & (smpc_stride - 1));
}

static u32 pri_smpc_ext_addr(struct gk20a *g, u32 sm_offset, u32 gpc_num,
			u32 tpc_num, u32 sm_num)
{
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 smpc_unique_base = nvgpu_get_litter_value(g,
				GPU_LIT_SMPC_PRI_UNIQUE_BASE);
	u32 smpc_stride = nvgpu_get_litter_value(g,
				GPU_LIT_SMPC_PRI_STRIDE);

	return (egpc_base + (gpc_num * gpc_stride) + tpc_in_gpc_base +
			(tpc_num * tpc_in_gpc_stride) +
			(sm_num * smpc_stride) +
			(smpc_unique_base + sm_offset));
}

static bool pri_is_smpc_addr_in_etpc_shared(struct gk20a *g, u32 addr)
{
	u32 smpc_shared_base = nvgpu_get_litter_value(g,
				GPU_LIT_SMPC_PRI_SHARED_BASE);
	u32 smpc_stride = nvgpu_get_litter_value(g,
				GPU_LIT_SMPC_PRI_STRIDE);

	return (addr >= smpc_shared_base) &&
		(addr < smpc_shared_base + smpc_stride);
}

bool gv11b_gr_pri_is_etpc_addr(struct gk20a *g, u32 addr)
{
	u32 egpc_addr = 0;

	if (g->ops.gr.is_egpc_addr(g, addr)) {
		egpc_addr = pri_gpccs_addr_mask(addr);
		if (g->ops.gr.is_tpc_addr(g, egpc_addr)) {
			return true;
		}
	}

	return false;
}

static u32 pri_get_egpc_num(struct gk20a *g, u32 addr)
{
	u32 i, start;
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 num_gpcs = nvgpu_get_litter_value(g, GPU_LIT_NUM_GPCS);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	for (i = 0; i < num_gpcs; i++) {
		start = egpc_base + (i * gpc_stride);
		if ((addr >= start) && (addr < (start + gpc_stride))) {
			return i;
		}
	}
	return 0;
}

static u32 pri_egpc_addr(struct gk20a *g, u32 addr, u32 gpc)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);

	return egpc_base + (gpc * gpc_stride) + addr;
}

static u32 pri_etpc_addr(struct gk20a *g, u32 addr, u32 gpc, u32 tpc)
{
	u32 egpc_base = g->ops.gr.get_egpc_base(g);
	u32 gpc_stride = nvgpu_get_litter_value(g, GPU_LIT_GPC_STRIDE);
	u32 tpc_in_gpc_base = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_BASE);
	u32 tpc_in_gpc_stride = nvgpu_get_litter_value(g,
					GPU_LIT_TPC_IN_GPC_STRIDE);

	return egpc_base + (gpc * gpc_stride) +
		tpc_in_gpc_base + (tpc * tpc_in_gpc_stride) +
		addr;
}

void gv11b_gr_get_egpc_etpc_num(struct gk20a *g, u32 addr,
			u32 *egpc_num, u32 *etpc_num)
{
	u32 egpc_addr = 0;

	*egpc_num = pri_get_egpc_num(g, addr);
	egpc_addr = pri_gpccs_addr_mask(addr);
	*etpc_num = g->ops.gr.get_tpc_num(g, egpc_addr);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"egpc_num = %d etpc_num = %d", *egpc_num, *etpc_num);
}

int gv11b_gr_decode_egpc_addr(struct gk20a *g, u32 addr,
	enum ctxsw_addr_type *addr_type, u32 *gpc_num, u32 *tpc_num,
	u32 *broadcast_flags)
{
	u32 gpc_addr;
	u32 tpc_addr;

	if (g->ops.gr.is_egpc_addr(g, addr)) {
		nvgpu_log_info(g, "addr=0x%x is egpc", addr);

		*addr_type = CTXSW_ADDR_TYPE_EGPC;
		gpc_addr = pri_gpccs_addr_mask(addr);
		if (pri_is_egpc_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_EGPC;
			*gpc_num = 0;
			nvgpu_log_info(g, "shared egpc");
		} else {
			*gpc_num = pri_get_egpc_num(g, addr);
			nvgpu_log_info(g, "gpc=0x%x", *gpc_num);
		}
		if (g->ops.gr.is_tpc_addr(g, gpc_addr)) {
			nvgpu_log_info(g, "addr=0x%x is etpc", addr);
			*addr_type = CTXSW_ADDR_TYPE_ETPC;
			if (pri_is_tpc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_ETPC;
				*tpc_num = 0;
				nvgpu_log_info(g, "shared etpc");
			} else {
				*tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
				nvgpu_log_info(g, "tpc=0x%x", *tpc_num);
			}
			tpc_addr = pri_tpccs_addr_mask(addr);
			if (pri_is_smpc_addr_in_etpc_shared(g, tpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_SMPC;
			}
		}

		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg,
			"addr_type = %d, broadcast_flags = %#08x",
			*addr_type, *broadcast_flags);
		return 0;
	}
	return -EINVAL;
}

static void gv11b_gr_update_priv_addr_table_smpc(struct gk20a *g, u32 gpc_num,
			u32 tpc_num, u32 addr,
			u32 *priv_addr_table, u32 *t)
{
	u32 sm_per_tpc, sm_num;

	nvgpu_log_info(g, "broadcast flags smpc");

	sm_per_tpc = nvgpu_get_litter_value(g, GPU_LIT_NUM_SM_PER_TPC);
	for (sm_num = 0; sm_num < sm_per_tpc; sm_num++) {
		priv_addr_table[*t] = pri_smpc_ext_addr(g,
				pri_smpc_in_etpc_addr_mask(g, addr),
				gpc_num, tpc_num, sm_num);
		nvgpu_log_info(g, "priv_addr_table[%d]:%#08x",
				*t, priv_addr_table[*t]);
		(*t)++;
	}
}

void gv11b_gr_egpc_etpc_priv_addr_table(struct gk20a *g, u32 addr,
	u32 gpc_num, u32 tpc_num, u32 broadcast_flags,
	u32 *priv_addr_table, u32 *t)
{
	u32 priv_addr, gpc_addr;

	nvgpu_log_info(g, "addr=0x%x", addr);

	/* The GPC/TPC unicast registers are included in the compressed PRI
	 * tables. Convert a GPC/TPC broadcast address to unicast addresses so
	 * that we can look up the offsets.
	 */
	if (broadcast_flags & PRI_BROADCAST_FLAGS_EGPC) {
		nvgpu_log_info(g, "broadcast flags egpc");
		for (gpc_num = 0; gpc_num < g->gr.gpc_count; gpc_num++) {

			if (broadcast_flags & PRI_BROADCAST_FLAGS_ETPC) {
				nvgpu_log_info(g, "broadcast flags etpc");
				for (tpc_num = 0;
				     tpc_num < g->gr.gpc_tpc_count[gpc_num];
				     tpc_num++) {
					if (broadcast_flags &
						PRI_BROADCAST_FLAGS_SMPC) {
							gv11b_gr_update_priv_addr_table_smpc(
								g, gpc_num, tpc_num, addr,
								priv_addr_table, t);
					} else {
						priv_addr_table[*t] =
							pri_etpc_addr(g,
							pri_tpccs_addr_mask(addr),
							gpc_num, tpc_num);
							nvgpu_log_info(g,
							"priv_addr_table[%d]:%#08x",
							*t, priv_addr_table[*t]);
						(*t)++;
					}
				}
			} else if (broadcast_flags & PRI_BROADCAST_FLAGS_SMPC) {
				gv11b_gr_update_priv_addr_table_smpc(
					g, gpc_num, tpc_num, addr,
					priv_addr_table, t);
			} else {
				priv_addr = pri_egpc_addr(g,
						pri_gpccs_addr_mask(addr),
						gpc_num);

				gpc_addr = pri_gpccs_addr_mask(priv_addr);
				tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
				if (tpc_num >= g->gr.gpc_tpc_count[gpc_num]) {
					continue;
				}

				priv_addr_table[*t] = priv_addr;
				nvgpu_log_info(g, "priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
				(*t)++;
			}
		}
	} else if (!(broadcast_flags & PRI_BROADCAST_FLAGS_EGPC)) {
		if (broadcast_flags & PRI_BROADCAST_FLAGS_ETPC) {
			nvgpu_log_info(g, "broadcast flags etpc but not egpc");
			for (tpc_num = 0;
			     tpc_num < g->gr.gpc_tpc_count[gpc_num];
			     tpc_num++) {
				if (broadcast_flags &
						PRI_BROADCAST_FLAGS_SMPC) {
					gv11b_gr_update_priv_addr_table_smpc(
						g, gpc_num, tpc_num, addr,
						priv_addr_table, t);
				} else {
					priv_addr_table[*t] =
					pri_etpc_addr(g,
					pri_tpccs_addr_mask(addr),
					     gpc_num, tpc_num);
					nvgpu_log_info(g,
					"priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
					(*t)++;
				}
			}
		} else if (broadcast_flags & PRI_BROADCAST_FLAGS_SMPC) {
			gv11b_gr_update_priv_addr_table_smpc(
				g, gpc_num, tpc_num, addr,
				priv_addr_table, t);
		} else {
			priv_addr_table[*t] = addr;
			nvgpu_log_info(g, "priv_addr_table[%d]:%#08x",
					*t, priv_addr_table[*t]);
			(*t)++;
		}
	}
}

u32 gv11b_gr_get_egpc_base(struct gk20a *g)
{
	return EGPC_PRI_BASE;
}

void gr_gv11b_init_gpc_mmu(struct gk20a *g)
{
	u32 temp;

	nvgpu_log_info(g, "initialize gpc mmu");

	temp = g->ops.fb.mmu_ctrl(g);
	temp &= gr_gpcs_pri_mmu_ctrl_vm_pg_size_m() |
		gr_gpcs_pri_mmu_ctrl_use_pdb_big_page_size_m() |
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
}

int gr_gv11b_init_preemption_state(struct gk20a *g)
{
	u32 debug_2;
	struct gr_gk20a *gr = &g->gr;
	u32 unit;

	nvgpu_log_fn(g, " ");

	if (gr->gfxp_wfi_timeout_unit == GFXP_WFI_TIMEOUT_UNIT_USEC) {
		unit = gr_debug_2_gfxp_wfi_timeout_unit_usec_f();
	} else {
		unit = gr_debug_2_gfxp_wfi_timeout_unit_sysclk_f();
	}

	debug_2 = gk20a_readl(g, gr_debug_2_r());
	debug_2 = set_field(debug_2,
		gr_debug_2_gfxp_wfi_timeout_unit_m(),
		unit);
	gk20a_writel(g, gr_debug_2_r(), debug_2);

	return 0;
}
void gr_gv11b_init_gfxp_wfi_timeout_count(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	gr->gfxp_wfi_timeout_unit = GFXP_WFI_TIMEOUT_UNIT_USEC;
	gr->gfxp_wfi_timeout_count = GFXP_WFI_TIMEOUT_COUNT_IN_USEC_DEFAULT;
}

unsigned long gr_gv11b_get_max_gfxp_wfi_timeout_count(struct gk20a *g)
{
	if (g->gr.gfxp_wfi_timeout_unit == GFXP_WFI_TIMEOUT_UNIT_USEC) {
		/* 100 msec in usec count */
		return (100UL * 1000UL);
	} else {
		/* 100 msec for 1GHz clock */
		return (100UL * 1000UL * 1000UL);
	}
}

static int gr_gv11b_ecc_scrub_is_done(struct gk20a *g,
			u32 scrub_reg, u32 scrub_mask, u32 scrub_done)
{
	struct nvgpu_timeout timeout;
	u32 val;
	u32 gpc, tpc;
	u32 gpc_offset, tpc_offset;

	nvgpu_timeout_init(g, &timeout,
		ECC_SCRUBBING_TIMEOUT_MAX /
		ECC_SCRUBBING_TIMEOUT_DEFAULT,
		NVGPU_TIMER_RETRY_TIMER);

	for (gpc = 0; gpc < g->gr.gpc_count; gpc++) {
		gpc_offset = gk20a_gr_gpc_offset(g, gpc);

		for (tpc = 0; tpc < g->gr.gpc_tpc_count[gpc]; tpc++) {
			tpc_offset = gk20a_gr_tpc_offset(g, tpc);

			do {
				val = gk20a_readl(g, gpc_offset + tpc_offset + scrub_reg);
				if ((val & scrub_mask) == scrub_done) {
					break;
				}

				if (nvgpu_timeout_expired(&timeout)) {
					return -ETIMEDOUT;
				}

				nvgpu_udelay(ECC_SCRUBBING_TIMEOUT_DEFAULT);
			} while (1);
		}
	}

	return 0;
}

static int gr_gv11b_ecc_scrub_sm_lrf(struct gk20a *g)
{
	u32 scrub_mask, scrub_done;

	if (!nvgpu_is_enabled(g, NVGPU_ECC_ENABLED_SM_LRF)) {
		nvgpu_log_info(g, "ECC SM LRF is disabled");
		return 0;
	}

	nvgpu_log_info(g, "gr_gv11b_ecc_scrub_sm_lrf");
	scrub_mask =
		(gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp0_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp1_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp2_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp3_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp4_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp5_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp6_task_f() |
		gr_pri_gpcs_tpcs_sm_lrf_ecc_control_scrub_qrfdp7_task_f());

	/* Issue scrub lrf regions with single write command */
	gk20a_writel(g, gr_pri_gpcs_tpcs_sm_lrf_ecc_control_r(), scrub_mask);

	scrub_done =
		(gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp0_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp1_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp2_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp3_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp4_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp5_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp6_init_f() |
		gr_pri_gpc0_tpc0_sm_lrf_ecc_control_scrub_qrfdp7_init_f());

	return gr_gv11b_ecc_scrub_is_done(g,
				gr_pri_gpc0_tpc0_sm_lrf_ecc_control_r(),
				scrub_mask, scrub_done);
}

static int gr_gv11b_ecc_scrub_sm_l1_data(struct gk20a *g)
{
	u32 scrub_mask, scrub_done;

	if (!nvgpu_is_enabled(g, NVGPU_ECC_ENABLED_SM_L1_DATA)) {
		nvgpu_log_info(g, "ECC L1DATA is disabled");
		return 0;
	}
	nvgpu_log_info(g, "gr_gv11b_ecc_scrub_sm_l1_data");
	scrub_mask =
		(gr_pri_gpcs_tpcs_sm_l1_data_ecc_control_scrub_el1_0_task_f() |
		gr_pri_gpcs_tpcs_sm_l1_data_ecc_control_scrub_el1_1_task_f());

	gk20a_writel(g, gr_pri_gpcs_tpcs_sm_l1_data_ecc_control_r(),
				scrub_mask);

	scrub_done =
		(gr_pri_gpc0_tpc0_sm_l1_data_ecc_control_scrub_el1_0_init_f() |
		gr_pri_gpc0_tpc0_sm_l1_data_ecc_control_scrub_el1_1_init_f());
	return gr_gv11b_ecc_scrub_is_done(g,
				gr_pri_gpc0_tpc0_sm_l1_data_ecc_control_r(),
				scrub_mask, scrub_done);
}

static int gr_gv11b_ecc_scrub_sm_l1_tag(struct gk20a *g)
{
	u32 scrub_mask, scrub_done;

	if (!nvgpu_is_enabled(g, NVGPU_ECC_ENABLED_SM_L1_TAG)) {
		nvgpu_log_info(g, "ECC L1TAG is disabled");
		return 0;
	}
	nvgpu_log_info(g, "gr_gv11b_ecc_scrub_sm_l1_tag");
	scrub_mask =
		(gr_pri_gpcs_tpcs_sm_l1_tag_ecc_control_scrub_el1_0_task_f() |
		gr_pri_gpcs_tpcs_sm_l1_tag_ecc_control_scrub_el1_1_task_f() |
		gr_pri_gpcs_tpcs_sm_l1_tag_ecc_control_scrub_pixprf_task_f() |
		gr_pri_gpcs_tpcs_sm_l1_tag_ecc_control_scrub_miss_fifo_task_f());
	gk20a_writel(g, gr_pri_gpcs_tpcs_sm_l1_tag_ecc_control_r(), scrub_mask);

	scrub_done =
		(gr_pri_gpc0_tpc0_sm_l1_tag_ecc_control_scrub_el1_0_init_f() |
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_control_scrub_el1_1_init_f() |
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_control_scrub_pixprf_init_f() |
		gr_pri_gpc0_tpc0_sm_l1_tag_ecc_control_scrub_miss_fifo_init_f());
	return gr_gv11b_ecc_scrub_is_done(g,
				gr_pri_gpc0_tpc0_sm_l1_tag_ecc_control_r(),
				scrub_mask, scrub_done);
}

static int gr_gv11b_ecc_scrub_sm_cbu(struct gk20a *g)
{
	u32 scrub_mask, scrub_done;

	if (!nvgpu_is_enabled(g, NVGPU_ECC_ENABLED_SM_CBU)) {
		nvgpu_log_info(g, "ECC CBU is disabled");
		return 0;
	}
	nvgpu_log_info(g, "gr_gv11b_ecc_scrub_sm_cbu");
	scrub_mask =
		(gr_pri_gpcs_tpcs_sm_cbu_ecc_control_scrub_warp_sm0_task_f() |
		gr_pri_gpcs_tpcs_sm_cbu_ecc_control_scrub_warp_sm1_task_f() |
		gr_pri_gpcs_tpcs_sm_cbu_ecc_control_scrub_barrier_sm0_task_f() |
		gr_pri_gpcs_tpcs_sm_cbu_ecc_control_scrub_barrier_sm1_task_f());
	gk20a_writel(g, gr_pri_gpcs_tpcs_sm_cbu_ecc_control_r(), scrub_mask);

	scrub_done =
		(gr_pri_gpc0_tpc0_sm_cbu_ecc_control_scrub_warp_sm0_init_f() |
		gr_pri_gpc0_tpc0_sm_cbu_ecc_control_scrub_warp_sm1_init_f() |
		gr_pri_gpc0_tpc0_sm_cbu_ecc_control_scrub_barrier_sm0_init_f() |
		gr_pri_gpc0_tpc0_sm_cbu_ecc_control_scrub_barrier_sm1_init_f());
	return gr_gv11b_ecc_scrub_is_done(g,
				gr_pri_gpc0_tpc0_sm_cbu_ecc_control_r(),
				scrub_mask, scrub_done);
}

static int gr_gv11b_ecc_scrub_sm_icahe(struct gk20a *g)
{
	u32 scrub_mask, scrub_done;

	if (!nvgpu_is_enabled(g, NVGPU_ECC_ENABLED_SM_ICACHE)) {
		nvgpu_log_info(g, "ECC ICAHE is disabled");
		return 0;
	}
	nvgpu_log_info(g, "gr_gv11b_ecc_scrub_sm_icahe");
	scrub_mask =
		(gr_pri_gpcs_tpcs_sm_icache_ecc_control_scrub_l0_data_task_f() |
		gr_pri_gpcs_tpcs_sm_icache_ecc_control_scrub_l0_predecode_task_f() |
		gr_pri_gpcs_tpcs_sm_icache_ecc_control_scrub_l1_data_task_f() |
		gr_pri_gpcs_tpcs_sm_icache_ecc_control_scrub_l1_predecode_task_f());
	gk20a_writel(g, gr_pri_gpcs_tpcs_sm_icache_ecc_control_r(), scrub_mask);

	scrub_done =
		(gr_pri_gpc0_tpc0_sm_icache_ecc_control_scrub_l0_data_init_f() |
		gr_pri_gpc0_tpc0_sm_icache_ecc_control_scrub_l0_predecode_init_f() |
		gr_pri_gpc0_tpc0_sm_icache_ecc_control_scrub_l1_data_init_f() |
		gr_pri_gpc0_tpc0_sm_icache_ecc_control_scrub_l1_predecode_init_f());
	return gr_gv11b_ecc_scrub_is_done(g,
				gr_pri_gpc0_tpc0_sm_icache_ecc_control_r(),
				scrub_mask, scrub_done);
}

static void gr_gv11b_detect_ecc_enabled_units(struct gk20a *g)
{
	bool opt_ecc_en = g->ops.fuse.is_opt_ecc_enable(g);
	bool opt_feature_fuses_override_disable =
		g->ops.fuse.is_opt_feature_override_disable(g);
	u32 fecs_feature_override_ecc =
			gk20a_readl(g,
				gr_fecs_feature_override_ecc_r());

	if (opt_feature_fuses_override_disable) {
		if (opt_ecc_en) {
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_LRF, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			__nvgpu_set_enabled(g, NVGPU_ECC_ENABLED_LTC, true);
			__nvgpu_set_enabled(g, NVGPU_ECC_ENABLED_SM_CBU, true);
		}
	} else {
		/* SM LRF */
		if (gr_fecs_feature_override_ecc_sm_lrf_override_v(
				fecs_feature_override_ecc) == 1U) {
			if (gr_fecs_feature_override_ecc_sm_lrf_v(
				fecs_feature_override_ecc) == 1U) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_SM_LRF, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_SM_LRF, true);
			}
		}
		/* SM L1 DATA*/
		if (gr_fecs_feature_override_ecc_sm_l1_data_override_v(
				fecs_feature_override_ecc) == 1U) {
			if (gr_fecs_feature_override_ecc_sm_l1_data_v(
				fecs_feature_override_ecc) == 1U) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_DATA, true);
			}
		}
		/* SM L1 TAG*/
		if (gr_fecs_feature_override_ecc_sm_l1_tag_override_v(
				fecs_feature_override_ecc) == 1U) {
			if (gr_fecs_feature_override_ecc_sm_l1_tag_v(
				fecs_feature_override_ecc) == 1U) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_L1_TAG, true);
			}
		}
		/* SM ICACHE*/
		if ((gr_fecs_feature_override_ecc_1_sm_l0_icache_override_v(
				fecs_feature_override_ecc) == 1U) &&
			(gr_fecs_feature_override_ecc_1_sm_l1_icache_override_v(
				fecs_feature_override_ecc) == 1U)) {
			if ((gr_fecs_feature_override_ecc_1_sm_l0_icache_v(
					fecs_feature_override_ecc) == 1U) &&
				(gr_fecs_feature_override_ecc_1_sm_l1_icache_v(
					fecs_feature_override_ecc) == 1U)) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_ICACHE, true);
			}
		}
		/* LTC */
		if (gr_fecs_feature_override_ecc_ltc_override_v(
				fecs_feature_override_ecc) == 1U) {
			if (gr_fecs_feature_override_ecc_ltc_v(
				fecs_feature_override_ecc) == 1U) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_LTC, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
						NVGPU_ECC_ENABLED_LTC, true);
			}
		}
		/* SM CBU */
		if (gr_fecs_feature_override_ecc_sm_cbu_override_v(
				fecs_feature_override_ecc) == 1U) {
			if (gr_fecs_feature_override_ecc_sm_cbu_v(
				fecs_feature_override_ecc) == 1U) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_CBU, true);
			}
		} else {
			if (opt_ecc_en) {
				__nvgpu_set_enabled(g,
					NVGPU_ECC_ENABLED_SM_CBU, true);
			}
		}
	}
}

void gr_gv11b_ecc_init_scrub_reg(struct gk20a *g)
{
	nvgpu_log_fn(g, "ecc srub start ");

	gr_gv11b_detect_ecc_enabled_units(g);

	if (gr_gv11b_ecc_scrub_sm_lrf(g)) {
		nvgpu_warn(g, "ECC SCRUB SM LRF Failed");
	}
	if (gr_gv11b_ecc_scrub_sm_l1_data(g)) {
		nvgpu_warn(g, "ECC SCRUB SM L1 DATA Failed");
	}
	if (gr_gv11b_ecc_scrub_sm_l1_tag(g)) {
		nvgpu_warn(g, "ECC SCRUB SM L1 TAG Failed");
	}
	if (gr_gv11b_ecc_scrub_sm_cbu(g)) {
		nvgpu_warn(g, "ECC SCRUB SM CBU Failed");
	}
	if (gr_gv11b_ecc_scrub_sm_icahe(g)) {
		nvgpu_warn(g, "ECC SCRUB SM ICACHE Failed");
	}

}

u32 gr_gv11b_get_gpcs_swdx_dss_zbc_c_format_reg(struct gk20a *g)
{
	return gr_gpcs_swdx_dss_zbc_c_01_to_04_format_r();
}

u32 gr_gv11b_get_gpcs_swdx_dss_zbc_z_format_reg(struct gk20a *g)
{
	return gr_gpcs_swdx_dss_zbc_z_01_to_04_format_r();
}

int gr_gv11b_handle_ssync_hww(struct gk20a *g)
{
	u32 ssync = gk20a_readl(g, gr_ssync_hww_esr_r());

	nvgpu_err(g, "ssync exception: esr 0x%08x", ssync);
	gk20a_writel(g, gr_ssync_hww_esr_r(),
			 gr_ssync_hww_esr_reset_active_f());
	return -EFAULT;
}

/*
 * This function will decode a priv address and return the partition
 * type and numbers
 */
int gr_gv11b_decode_priv_addr(struct gk20a *g, u32 addr,
	enum ctxsw_addr_type *addr_type,
	u32 *gpc_num, u32 *tpc_num, u32 *ppc_num, u32 *be_num,
	u32 *broadcast_flags)
{
	u32 gpc_addr;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	/* setup defaults */
	*addr_type = CTXSW_ADDR_TYPE_SYS;
	*broadcast_flags = PRI_BROADCAST_FLAGS_NONE;
	*gpc_num = 0;
	*tpc_num = 0;
	*ppc_num = 0;
	*be_num  = 0;

	if (pri_is_gpc_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_GPC;
		gpc_addr = pri_gpccs_addr_mask(addr);
		if (pri_is_gpc_addr_shared(g, addr)) {
			*addr_type = CTXSW_ADDR_TYPE_GPC;
			*broadcast_flags |= PRI_BROADCAST_FLAGS_GPC;
		} else {
			*gpc_num = pri_get_gpc_num(g, addr);
		}

		if (pri_is_ppc_addr(g, gpc_addr)) {
			*addr_type = CTXSW_ADDR_TYPE_PPC;
			if (pri_is_ppc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_PPC;
				return 0;
			}
		}
		if (g->ops.gr.is_tpc_addr(g, gpc_addr)) {
			*addr_type = CTXSW_ADDR_TYPE_TPC;
			if (pri_is_tpc_addr_shared(g, gpc_addr)) {
				*broadcast_flags |= PRI_BROADCAST_FLAGS_TPC;
				return 0;
			}
			*tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
		}
		return 0;
	} else if (pri_is_be_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_BE;
		if (pri_is_be_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_BE;
			return 0;
		}
		*be_num = pri_get_be_num(g, addr);
		return 0;
	} else if (g->ops.ltc.pri_is_ltc_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_LTCS;
		if (g->ops.ltc.is_ltcs_ltss_addr(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_LTCS;
		} else if (g->ops.ltc.is_ltcn_ltss_addr(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_LTSS;
		}
		return 0;
	} else if (pri_is_fbpa_addr(g, addr)) {
		*addr_type = CTXSW_ADDR_TYPE_FBPA;
		if (pri_is_fbpa_addr_shared(g, addr)) {
			*broadcast_flags |= PRI_BROADCAST_FLAGS_FBPA;
			return 0;
		}
		return 0;
	} else if (g->ops.gr.is_egpc_addr && g->ops.gr.is_egpc_addr(g, addr)) {
		return g->ops.gr.decode_egpc_addr(g,
				addr, addr_type, gpc_num,
				tpc_num, broadcast_flags);
	} else if (PRI_PMMGS_BASE_ADDR_MASK(addr) ==
			NV_PERF_PMMGPC_GPCGS_GPCTPCA) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_GPCGS_GPCTPCA |
				     PRI_BROADCAST_FLAGS_PMMGPC);
		*addr_type = CTXSW_ADDR_TYPE_GPC;
		return 0;
	} else if (PRI_PMMGS_BASE_ADDR_MASK(addr) ==
			NV_PERF_PMMGPC_GPCGS_GPCTPCB) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_GPCGS_GPCTPCB |
				     PRI_BROADCAST_FLAGS_PMMGPC);
		*addr_type = CTXSW_ADDR_TYPE_GPC;
		return 0;
	} else if (PRI_PMMGS_BASE_ADDR_MASK(addr) == NV_PERF_PMMFBP_FBPGS_LTC) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_FBPGS_LTC |
				     PRI_BROADCAST_FLAGS_PMMFBP);
		*addr_type = CTXSW_ADDR_TYPE_LTCS;
		return 0;
	} else if (PRI_PMMGS_BASE_ADDR_MASK(addr) == NV_PERF_PMMFBP_FBPGS_ROP) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_FBPGS_ROP |
				     PRI_BROADCAST_FLAGS_PMMFBP);
		*addr_type = CTXSW_ADDR_TYPE_ROP;
		return 0;
	} else if (PRI_PMMS_BASE_ADDR_MASK(addr) == NV_PERF_PMMGPC_GPCS) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_GPCS |
				     PRI_BROADCAST_FLAGS_PMMGPC);
		*addr_type = CTXSW_ADDR_TYPE_GPC;
		return 0;
	} else if (PRI_PMMS_BASE_ADDR_MASK(addr) == NV_PERF_PMMFBP_FBPS) {
		*broadcast_flags |= (PRI_BROADCAST_FLAGS_PMM_FBPS |
				     PRI_BROADCAST_FLAGS_PMMFBP);
		*addr_type = CTXSW_ADDR_TYPE_FBP;
		return 0;
	}

	*addr_type = CTXSW_ADDR_TYPE_SYS;
	return 0;
}

u32 gr_gv11b_get_pmm_per_chiplet_offset(void)
{
	return (perf_pmmsys_extent_v() - perf_pmmsys_base_v() + 1);
}

static u32 gr_gv11b_pri_pmmgpc_addr(struct gk20a *g, u32 gpc_num,
	u32 domain_idx, u32 offset)
{
	return perf_pmmgpc_base_v() +
		(gpc_num * g->ops.gr.get_pmm_per_chiplet_offset()) +
		(domain_idx * perf_pmmgpc_perdomain_offset_v()) +
		offset;
}

static void gr_gv11b_split_pmm_fbp_broadcast_address(struct gk20a *g,
	u32 offset, u32 *priv_addr_table, u32 *t,
	u32 domain_start, u32 num_domains)
{
	u32 domain_idx = 0;
	u32 fbp_num = 0;
	u32 base = 0;

	for (fbp_num = 0; fbp_num < g->gr.num_fbps; fbp_num++) {
		base = perf_pmmfbp_base_v() +
			(fbp_num * g->ops.gr.get_pmm_per_chiplet_offset());

		for (domain_idx = domain_start;
		     domain_idx < (domain_start + num_domains);
		     domain_idx++) {
			priv_addr_table[(*t)++] = base +
				(domain_idx * perf_pmmgpc_perdomain_offset_v())
				+ offset;
		}
	}
}


int gr_gv11b_create_priv_addr_table(struct gk20a *g,
					   u32 addr,
					   u32 *priv_addr_table,
					   u32 *num_registers)
{
	enum ctxsw_addr_type addr_type;
	u32 gpc_num, tpc_num, ppc_num, be_num;
	u32 priv_addr, gpc_addr;
	u32 broadcast_flags;
	u32 t;
	int err;

	t = 0;
	*num_registers = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, "addr=0x%x", addr);

	err = g->ops.gr.decode_priv_addr(g, addr, &addr_type,
					&gpc_num, &tpc_num, &ppc_num, &be_num,
					&broadcast_flags);
	nvgpu_log(g, gpu_dbg_gpu_dbg, "addr_type = %d", addr_type);
	if (err) {
		return err;
	}

	if ((addr_type == CTXSW_ADDR_TYPE_SYS) ||
	    (addr_type == CTXSW_ADDR_TYPE_BE)) {
		/*
		 * The BE broadcast registers are included in the compressed PRI
		 * table. Convert a BE unicast address to a broadcast address
		 * so that we can look up the offset
		 */
		if ((addr_type == CTXSW_ADDR_TYPE_BE) &&
		    !(broadcast_flags & PRI_BROADCAST_FLAGS_BE)) {
			priv_addr_table[t++] = pri_be_shared_addr(g, addr);
		} else {
			priv_addr_table[t++] = addr;
		}

		*num_registers = t;
		return 0;
	}

	/*
	 * The GPC/TPC unicast registers are included in the compressed PRI
	 * tables. Convert a GPC/TPC broadcast address to unicast addresses so
	 * that we can look up the offsets
	 */
	if (broadcast_flags & PRI_BROADCAST_FLAGS_GPC) {
		for (gpc_num = 0; gpc_num < g->gr.gpc_count; gpc_num++) {

			if (broadcast_flags & PRI_BROADCAST_FLAGS_TPC) {
				for (tpc_num = 0;
				     tpc_num < g->gr.gpc_tpc_count[gpc_num];
				     tpc_num++) {
					priv_addr_table[t++] =
						pri_tpc_addr(g,
						    pri_tpccs_addr_mask(addr),
						    gpc_num, tpc_num);
				}
			}

			else if (broadcast_flags & PRI_BROADCAST_FLAGS_PPC) {
				err = gr_gk20a_split_ppc_broadcast_addr(g,
					addr, gpc_num, priv_addr_table, &t);
				if (err) {
					return err;
				}
			} else {
				priv_addr = pri_gpc_addr(g,
						pri_gpccs_addr_mask(addr),
						gpc_num);

				gpc_addr = pri_gpccs_addr_mask(priv_addr);
				tpc_num = g->ops.gr.get_tpc_num(g, gpc_addr);
				if (tpc_num >= g->gr.gpc_tpc_count[gpc_num]) {
					continue;
				}

				priv_addr_table[t++] = priv_addr;
			}
		}
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_PMMGPC) {
		u32 pmm_domain_start = 0;
		u32 domain_idx = 0;
		u32 num_domains = 0;
		u32 offset = 0;

		if (broadcast_flags & PRI_BROADCAST_FLAGS_PMM_GPCGS_GPCTPCA) {
			pmm_domain_start = nvgpu_get_litter_value(g,
				GPU_LIT_PERFMON_PMMGPCTPCA_DOMAIN_START);
			num_domains = nvgpu_get_litter_value(g,
				GPU_LIT_PERFMON_PMMGPCTPC_DOMAIN_COUNT);
			offset = PRI_PMMGS_OFFSET_MASK(addr);
		} else if (broadcast_flags &
				PRI_BROADCAST_FLAGS_PMM_GPCGS_GPCTPCB) {
			pmm_domain_start = nvgpu_get_litter_value(g,
				GPU_LIT_PERFMON_PMMGPCTPCB_DOMAIN_START);
			num_domains = nvgpu_get_litter_value(g,
				GPU_LIT_PERFMON_PMMGPCTPC_DOMAIN_COUNT);
			offset = PRI_PMMGS_OFFSET_MASK(addr);
		} else if (broadcast_flags & PRI_BROADCAST_FLAGS_PMM_GPCS) {
			pmm_domain_start = (addr -
			     (NV_PERF_PMMGPC_GPCS + PRI_PMMS_ADDR_MASK(addr)))/
			     perf_pmmgpc_perdomain_offset_v();
			num_domains = 1;
			offset = PRI_PMMS_ADDR_MASK(addr);
		} else {
			return -EINVAL;
		}

		for (gpc_num = 0; gpc_num < g->gr.gpc_count; gpc_num++) {
			for (domain_idx = pmm_domain_start;
			     domain_idx < (pmm_domain_start + num_domains);
			     domain_idx++) {
				priv_addr_table[t++] =
					gr_gv11b_pri_pmmgpc_addr(g, gpc_num,
					domain_idx, offset);
			}
		}
	} else if (((addr_type == CTXSW_ADDR_TYPE_EGPC) ||
			(addr_type == CTXSW_ADDR_TYPE_ETPC)) &&
				g->ops.gr.egpc_etpc_priv_addr_table) {
		nvgpu_log(g, gpu_dbg_gpu_dbg, "addr_type : EGPC/ETPC");
		g->ops.gr.egpc_etpc_priv_addr_table(g, addr, gpc_num, tpc_num,
				broadcast_flags, priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_LTSS) {
		g->ops.ltc.split_lts_broadcast_addr(g, addr,
							priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_LTCS) {
		g->ops.ltc.split_ltc_broadcast_addr(g, addr,
							priv_addr_table, &t);
	} else if (broadcast_flags & PRI_BROADCAST_FLAGS_FBPA) {
		g->ops.gr.split_fbpa_broadcast_addr(g, addr,
				nvgpu_get_litter_value(g, GPU_LIT_NUM_FBPAS),
				priv_addr_table, &t);
	} else if ((addr_type == CTXSW_ADDR_TYPE_LTCS) &&
		   (broadcast_flags & PRI_BROADCAST_FLAGS_PMM_FBPGS_LTC)) {
		gr_gv11b_split_pmm_fbp_broadcast_address(g,
			PRI_PMMGS_OFFSET_MASK(addr),
			priv_addr_table, &t,
			nvgpu_get_litter_value(g, GPU_LIT_PERFMON_PMMFBP_LTC_DOMAIN_START),
			nvgpu_get_litter_value(g, GPU_LIT_PERFMON_PMMFBP_LTC_DOMAIN_COUNT));
	} else if ((addr_type == CTXSW_ADDR_TYPE_ROP) &&
		   (broadcast_flags & PRI_BROADCAST_FLAGS_PMM_FBPGS_ROP)) {
		gr_gv11b_split_pmm_fbp_broadcast_address(g,
			PRI_PMMGS_OFFSET_MASK(addr),
			priv_addr_table, &t,
			nvgpu_get_litter_value(g, GPU_LIT_PERFMON_PMMFBP_ROP_DOMAIN_START),
			nvgpu_get_litter_value(g, GPU_LIT_PERFMON_PMMFBP_ROP_DOMAIN_COUNT));
	} else if ((addr_type == CTXSW_ADDR_TYPE_FBP) &&
		   (broadcast_flags & PRI_BROADCAST_FLAGS_PMM_FBPS)) {
		u32 domain_start;

		domain_start = (addr -
			(NV_PERF_PMMFBP_FBPS + PRI_PMMS_ADDR_MASK(addr)))/
			perf_pmmgpc_perdomain_offset_v();
		gr_gv11b_split_pmm_fbp_broadcast_address(g,
			PRI_PMMS_ADDR_MASK(addr),
			priv_addr_table, &t,
			domain_start, 1);
	} else if (!(broadcast_flags & PRI_BROADCAST_FLAGS_GPC)) {
		if (broadcast_flags & PRI_BROADCAST_FLAGS_TPC) {
			for (tpc_num = 0;
			     tpc_num < g->gr.gpc_tpc_count[gpc_num];
			     tpc_num++) {
				priv_addr_table[t++] =
					pri_tpc_addr(g,
						pri_tpccs_addr_mask(addr),
						gpc_num, tpc_num);
			}
		} else if (broadcast_flags & PRI_BROADCAST_FLAGS_PPC) {
			err = gr_gk20a_split_ppc_broadcast_addr(g,
					addr, gpc_num, priv_addr_table, &t);
		} else {
			priv_addr_table[t++] = addr;
		}
	}

	*num_registers = t;
	return 0;
}

int gv11b_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id)
{
	u32 gpc, tpc, sm, offset;
	u32 val;
	struct tsg_gk20a *tsg;

	int err = 0;

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	(void)memset(&tsg->sm_error_states[sm_id], 0, sizeof(*tsg->sm_error_states));

	err = gr_gk20a_disable_ctxsw(g);
	if (err != 0) {
		nvgpu_err(g, "unable to stop gr ctxsw");
		goto fail;
	}

	if (gk20a_is_channel_ctx_resident(ch)) {
		gpc = g->gr.sm_to_cluster[sm_id].gpc_index;
		if (g->ops.gr.get_nonpes_aware_tpc != NULL) {
			tpc = g->ops.gr.get_nonpes_aware_tpc(g,
					g->gr.sm_to_cluster[sm_id].gpc_index,
					g->gr.sm_to_cluster[sm_id].tpc_index);
		} else {
			tpc = g->gr.sm_to_cluster[sm_id].tpc_index;
		}
		sm = g->gr.sm_to_cluster[sm_id].sm_index;

		offset = gk20a_gr_gpc_offset(g, gpc) +
				gk20a_gr_tpc_offset(g, tpc) +
				gv11b_gr_sm_offset(g, sm);

		val = gk20a_readl(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset);
		gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_global_esr_r() + offset,
				val);
		gk20a_writel(g, gr_gpc0_tpc0_sm0_hww_warp_esr_r() + offset,
				0);
	}

	err = gr_gk20a_enable_ctxsw(g);

fail:
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	return err;
}
