/*
 * GP10B L2
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <trace/events/gk20a.h>

#include <nvgpu/ltc.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/hw/gp10b/hw_mc_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ltc_gp10b.h>

#include "ltc_gm20b.h"
#include "ltc_gp10b.h"

int gp10b_determine_L2_size_bytes(struct gk20a *g)
{
	u32 tmp;
	int ret;

	nvgpu_log_fn(g, " ");

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_info_1_r());

	ret = g->ltc_count *
		ltc_ltc0_lts0_tstg_info_1_slice_size_in_kb_v(tmp)*1024 *
		ltc_ltc0_lts0_tstg_info_1_slices_per_l2_v(tmp);

	nvgpu_log(g, gpu_dbg_info, "L2 size: %d\n", ret);

	nvgpu_log_fn(g, "done");

	return ret;
}

int gp10b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	/* max memory size (MB) to cover */
	u32 max_size = gr->max_comptag_mem;
	/* one tag line covers 64KB */
	u32 max_comptag_lines = max_size << 4U;

	u32 hw_max_comptag_lines =
		ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_init_v();

	u32 cbc_param =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	u32 comptags_per_cacheline =
		ltc_ltcs_ltss_cbc_param_comptags_per_cache_line_v(cbc_param);
	u32 cbc_param2 =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param2_r());
	u32 gobs_per_comptagline_per_slice =
		ltc_ltcs_ltss_cbc_param2_gobs_per_comptagline_per_slice_v(cbc_param2);

	u32 compbit_backing_size;

	int err;

	nvgpu_log_fn(g, " ");

	if (max_comptag_lines == 0U) {
		return 0;
	}

	/* Already initialized */
	if (gr->max_comptag_lines) {
		return 0;
	}

	if (max_comptag_lines > hw_max_comptag_lines) {
		max_comptag_lines = hw_max_comptag_lines;
	}

	compbit_backing_size =
		roundup(max_comptag_lines * gobs_per_comptagline_per_slice,
			gr->cacheline_size);
	compbit_backing_size = roundup(
		compbit_backing_size * gr->slices_per_ltc * g->ltc_count,
		g->ops.fb.compressible_page_size(g));

	/* aligned to 2KB * ltc_count */
	compbit_backing_size +=
		g->ltc_count << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	/* must be a multiple of 64KB */
	compbit_backing_size = roundup(compbit_backing_size, 64*1024);

	nvgpu_log_info(g, "compbit backing store size : %d",
		compbit_backing_size);
	nvgpu_log_info(g, "max comptag lines : %d",
		max_comptag_lines);
	nvgpu_log_info(g, "gobs_per_comptagline_per_slice: %d",
		gobs_per_comptagline_per_slice);

	err = nvgpu_ltc_alloc_cbc(g, compbit_backing_size, false);
	if (err) {
		return err;
	}

	err = gk20a_comptag_allocator_init(g, &gr->comp_tags, max_comptag_lines);
	if (err) {
		return err;
	}

	gr->max_comptag_lines = max_comptag_lines;
	gr->comptags_per_cacheline = comptags_per_cacheline;
	gr->gobs_per_comptagline_per_slice = gobs_per_comptagline_per_slice;
	gr->compbit_backing_size = compbit_backing_size;

	return 0;
}

int gp10b_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
		       u32 min, u32 max)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_timeout timeout;
	int err = 0;
	u32 ltc, slice, ctrl1, val, hw_op = 0U;
	u32 slices_per_ltc = ltc_ltcs_ltss_cbc_param_slices_per_ltc_v(
				gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r()));
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);
	const u32 max_lines = 16384U;

	nvgpu_log_fn(g, " ");

	trace_gk20a_ltc_cbc_ctrl_start(g->name, op, min, max);

	if (gr->compbit_store.mem.size == 0U) {
		return 0;
	}

	while (1) {
		const u32 iter_max = min(min + max_lines - 1, max);
		bool full_cache_op = true;

		nvgpu_mutex_acquire(&g->mm.l2_op_lock);

		nvgpu_log_info(g, "clearing CBC lines %u..%u", min, iter_max);

		if (op == gk20a_cbc_op_clear) {
			nvgpu_writel_check(
				g, ltc_ltcs_ltss_cbc_ctrl2_r(),
				ltc_ltcs_ltss_cbc_ctrl2_clear_lower_bound_f(
					min));

			nvgpu_writel_check(
				g, ltc_ltcs_ltss_cbc_ctrl3_r(),
				ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_f(
					iter_max));

			hw_op = ltc_ltcs_ltss_cbc_ctrl1_clear_active_f();
			full_cache_op = false;
		} else if (op == gk20a_cbc_op_clean) {
			/* this is full-cache op */
			hw_op = ltc_ltcs_ltss_cbc_ctrl1_clean_active_f();
		} else if (op == gk20a_cbc_op_invalidate) {
			/* this is full-cache op */
			hw_op = ltc_ltcs_ltss_cbc_ctrl1_invalidate_active_f();
		} else {
			nvgpu_err(g, "Unknown op: %u", (unsigned)op);
			err = -EINVAL;
			goto out;
		}
		gk20a_writel(g, ltc_ltcs_ltss_cbc_ctrl1_r(),
			     gk20a_readl(g,
					 ltc_ltcs_ltss_cbc_ctrl1_r()) | hw_op);

		for (ltc = 0; ltc < g->ltc_count; ltc++) {
			for (slice = 0; slice < slices_per_ltc; slice++) {

				ctrl1 = ltc_ltc0_lts0_cbc_ctrl1_r() +
					ltc * ltc_stride + slice * lts_stride;

				nvgpu_timeout_init(g, &timeout, 2000,
						   NVGPU_TIMER_RETRY_TIMER);
				do {
					val = gk20a_readl(g, ctrl1);
					if (!(val & hw_op)) {
						break;
					}
					nvgpu_udelay(5);
				} while (!nvgpu_timeout_expired(&timeout));

				if (nvgpu_timeout_peek_expired(&timeout)) {
					nvgpu_err(g, "comp tag clear timeout");
					err = -EBUSY;
					goto out;
				}
			}
		}

		/* are we done? */
		if (full_cache_op || iter_max == max) {
			break;
		}

		/* note: iter_max is inclusive upper bound */
		min = iter_max + 1;

		/* give a chance for higher-priority threads to progress */
		nvgpu_mutex_release(&g->mm.l2_op_lock);
	}
out:
	trace_gk20a_ltc_cbc_ctrl_done(g->name);
	nvgpu_mutex_release(&g->mm.l2_op_lock);
	return err;
}

void gp10b_ltc_lts_isr(struct gk20a *g,
		unsigned int ltc, unsigned int slice)
{
	u32 offset;
	u32 ltc_intr;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	offset = ltc_stride * ltc + lts_stride * slice;
	ltc_intr = gk20a_readl(g, ltc_ltc0_lts0_intr_r() + offset);

	/* Detect and handle ECC errors */
	if (ltc_intr &
		ltc_ltcs_ltss_intr_ecc_sec_error_pending_f()) {
		u32 ecc_stats_reg_val;

		nvgpu_err(g,
			"Single bit error detected in GPU L2!");

		ecc_stats_reg_val =
			gk20a_readl(g,
				ltc_ltc0_lts0_dstg_ecc_report_r() + offset);
		g->ecc.ltc.ecc_sec_count[ltc][slice].counter +=
			ltc_ltc0_lts0_dstg_ecc_report_sec_count_v(ecc_stats_reg_val);
		ecc_stats_reg_val &=
			~(ltc_ltc0_lts0_dstg_ecc_report_sec_count_m());
		nvgpu_writel_check(g,
			ltc_ltc0_lts0_dstg_ecc_report_r() + offset,
			ecc_stats_reg_val);
		g->ops.mm.l2_flush(g, true);
	}
	if (ltc_intr &
		ltc_ltcs_ltss_intr_ecc_ded_error_pending_f()) {
		u32 ecc_stats_reg_val;

		nvgpu_err(g,
			"Double bit error detected in GPU L2!");

		ecc_stats_reg_val =
			gk20a_readl(g,
				ltc_ltc0_lts0_dstg_ecc_report_r() + offset);
		g->ecc.ltc.ecc_ded_count[ltc][slice].counter +=
			ltc_ltc0_lts0_dstg_ecc_report_ded_count_v(ecc_stats_reg_val);
		ecc_stats_reg_val &=
			~(ltc_ltc0_lts0_dstg_ecc_report_ded_count_m());
		nvgpu_writel_check(g,
			ltc_ltc0_lts0_dstg_ecc_report_r() + offset,
			ecc_stats_reg_val);
	}

	nvgpu_err(g, "ltc%d, slice %d: %08x",
		  ltc, slice, ltc_intr);
	nvgpu_writel_check(g, ltc_ltc0_lts0_intr_r() +
		ltc_stride * ltc + lts_stride * slice,
		ltc_intr);
}

void gp10b_ltc_isr(struct gk20a *g)
{
	u32 mc_intr;
	unsigned int ltc, slice;

	mc_intr = gk20a_readl(g, mc_intr_ltc_r());
	nvgpu_err(g, "mc_ltc_intr: %08x", mc_intr);
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		if ((mc_intr & 1U << ltc) == 0) {
			continue;
		}
		for (slice = 0; slice < g->gr.slices_per_ltc; slice++) {
			gp10b_ltc_lts_isr(g, ltc, slice);
		}
	}
}

void gp10b_ltc_init_fs_state(struct gk20a *g)
{
	u32 ltc_intr;

	gm20b_ltc_init_fs_state(g);

	gk20a_writel(g, ltc_ltca_g_axi_pctrl_r(),
			ltc_ltca_g_axi_pctrl_user_sid_f(g->ltc_streamid));

	/* Enable ECC interrupts */
	ltc_intr = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	ltc_intr |= ltc_ltcs_ltss_intr_en_ecc_sec_error_enabled_f() |
			ltc_ltcs_ltss_intr_en_ecc_ded_error_enabled_f();
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(),
			ltc_intr);
}

void gp10b_ltc_set_enabled(struct gk20a *g, bool enabled)
{
	u32 reg_f = ltc_ltcs_ltss_tstg_set_mgmt_2_l2_bypass_mode_enabled_f();
	u32 reg = gk20a_readl(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r());

	if (enabled) {
		/* bypass disabled (normal caching ops) */
		reg &= ~reg_f;
	} else {
		/* bypass enabled (no caching) */
		reg |= reg_f;
	}

	nvgpu_writel_check(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r(), reg);
}
