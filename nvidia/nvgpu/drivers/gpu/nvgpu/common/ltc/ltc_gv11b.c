/*
 * GV11B LTC
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "ltc_gp10b.h"
#include "ltc_gv11b.h"

#include <nvgpu/hw/gv11b/hw_ltc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>

#include <nvgpu/utils.h>

/*
 * Sets the ZBC stencil for the passed index.
 */
void gv11b_ltc_set_zbc_stencil_entry(struct gk20a *g,
					  struct zbc_entry *stencil_val,
					  u32 index)
{
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	nvgpu_writel_check(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	nvgpu_writel_check(g,
			   ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_r(),
			   stencil_val->depth);
}

void gv11b_ltc_init_fs_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 ltc_intr;
	u32 reg;

	nvgpu_log_info(g, "initialize gv11b l2");

	g->max_ltc_count = gk20a_readl(g, top_num_ltcs_r());
	g->ltc_count = g->ops.priv_ring.enum_ltc(g);
	nvgpu_log_info(g, "%u ltcs out of %u", g->ltc_count, g->max_ltc_count);

	reg = gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	gr->slices_per_ltc = ltc_ltcs_ltss_cbc_param_slices_per_ltc_v(reg);;
	gr->cacheline_size =
		512U << ltc_ltcs_ltss_cbc_param_cache_line_size_v(reg);

	/* Disable LTC interrupts */
	reg = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	reg &= ~ltc_ltcs_ltss_intr_en_evicted_cb_m();
	reg &= ~ltc_ltcs_ltss_intr_en_illegal_compstat_access_m();
	nvgpu_writel_check(g, ltc_ltcs_ltss_intr_r(), reg);

	if (g->ops.ltc.intr_en_illegal_compstat) {
		g->ops.ltc.intr_en_illegal_compstat(g,
					g->ltc_intr_en_illegal_compstat);
	}

	/* Enable ECC interrupts */
	ltc_intr = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	ltc_intr |= ltc_ltcs_ltss_intr_en_ecc_sec_error_enabled_f() |
		ltc_ltcs_ltss_intr_en_ecc_ded_error_enabled_f();
	nvgpu_writel_check(g, ltc_ltcs_ltss_intr_r(),
				ltc_intr);
}

void gv11b_ltc_intr_en_illegal_compstat(struct gk20a *g, bool enable)
{
	u32 val;

	/* disble/enble illegal_compstat interrupt */
	val = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	if (enable) {
		val = set_field(val,
			ltc_ltcs_ltss_intr_en_illegal_compstat_m(),
			ltc_ltcs_ltss_intr_en_illegal_compstat_enabled_f());
	} else {
		val = set_field(val,
			ltc_ltcs_ltss_intr_en_illegal_compstat_m(),
			ltc_ltcs_ltss_intr_en_illegal_compstat_disabled_f());
        }
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(), val);
}

void gv11b_ltc_lts_isr(struct gk20a *g,
		unsigned int ltc, unsigned int slice)
{
	u32 offset;
	u32 ltc_intr3;
	u32 ecc_status, ecc_addr, corrected_cnt, uncorrected_cnt;
	u32 corrected_delta, uncorrected_delta;
	u32 corrected_overflow, uncorrected_overflow;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	offset = ltc_stride * ltc + lts_stride * slice;
	ltc_intr3 = gk20a_readl(g, ltc_ltc0_lts0_intr3_r() +
				offset);

	/* Detect and handle ECC PARITY errors */
	if (ltc_intr3 &
		(ltc_ltcs_ltss_intr3_ecc_uncorrected_m() |
		 ltc_ltcs_ltss_intr3_ecc_corrected_m())) {

		ecc_status = gk20a_readl(g,
			ltc_ltc0_lts0_l2_cache_ecc_status_r() +
			offset);
		ecc_addr = gk20a_readl(g,
			ltc_ltc0_lts0_l2_cache_ecc_address_r() +
			offset);
		corrected_cnt = gk20a_readl(g,
			ltc_ltc0_lts0_l2_cache_ecc_corrected_err_count_r() + offset);
		uncorrected_cnt = gk20a_readl(g,
			ltc_ltc0_lts0_l2_cache_ecc_uncorrected_err_count_r() + offset);

		corrected_delta =
			ltc_ltc0_lts0_l2_cache_ecc_corrected_err_count_total_v(corrected_cnt);
		uncorrected_delta =
			ltc_ltc0_lts0_l2_cache_ecc_uncorrected_err_count_total_v(uncorrected_cnt);
		corrected_overflow = ecc_status &
			ltc_ltc0_lts0_l2_cache_ecc_status_corrected_err_total_counter_overflow_m();

		uncorrected_overflow = ecc_status &
			ltc_ltc0_lts0_l2_cache_ecc_status_uncorrected_err_total_counter_overflow_m();

		/* clear the interrupt */
		if ((corrected_delta > 0U) || corrected_overflow) {
			nvgpu_writel_check(g,
				ltc_ltc0_lts0_l2_cache_ecc_corrected_err_count_r() + offset, 0);
		}
		if ((uncorrected_delta > 0U) || uncorrected_overflow) {
			nvgpu_writel_check(g,
				ltc_ltc0_lts0_l2_cache_ecc_uncorrected_err_count_r() + offset, 0);
		}

		nvgpu_writel_check(g,
			ltc_ltc0_lts0_l2_cache_ecc_status_r() + offset,
			ltc_ltc0_lts0_l2_cache_ecc_status_reset_task_f());

		/* update counters per slice */
		if (corrected_overflow) {
			corrected_delta += (0x1U << ltc_ltc0_lts0_l2_cache_ecc_corrected_err_count_total_s());
		}
		if (uncorrected_overflow) {
			uncorrected_delta += (0x1U << ltc_ltc0_lts0_l2_cache_ecc_uncorrected_err_count_total_s());
		}

		g->ecc.ltc.ecc_sec_count[ltc][slice].counter += corrected_delta;
		g->ecc.ltc.ecc_ded_count[ltc][slice].counter += uncorrected_delta;
		nvgpu_log(g, gpu_dbg_intr,
			"ltc:%d lts: %d cache ecc interrupt intr: 0x%x", ltc, slice, ltc_intr3);

		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_corrected_err_rstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "rstg ecc error corrected");
		}
		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_uncorrected_err_rstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "rstg ecc error uncorrected");
		}
		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_corrected_err_tstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "tstg ecc error corrected");
		}
		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_uncorrected_err_tstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "tstg ecc error uncorrected");
		}
		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_corrected_err_dstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "dstg ecc error corrected");
		}
		if (ecc_status & ltc_ltc0_lts0_l2_cache_ecc_status_uncorrected_err_dstg_m()) {
			nvgpu_log(g, gpu_dbg_intr, "dstg ecc error uncorrected");
		}

		if (corrected_overflow || uncorrected_overflow) {
			nvgpu_info(g, "ecc counter overflow!");
		}

		nvgpu_log(g, gpu_dbg_intr,
			"ecc error address: 0x%x", ecc_addr);
	}

	gp10b_ltc_lts_isr(g, ltc, slice);
}

void gv11b_ltc_isr(struct gk20a *g)
{
	u32 mc_intr;
	unsigned int ltc, slice;

	mc_intr = gk20a_readl(g, mc_intr_ltc_r());
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		if ((mc_intr & 1U << ltc) == 0) {
			continue;
		}

		for (slice = 0; slice < g->gr.slices_per_ltc; slice++) {
			gv11b_ltc_lts_isr(g, ltc, slice);
		}
	}
}
