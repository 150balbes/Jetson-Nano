/*
 * GM20B L2
 *
 * Copyright (c) 2014-2018 NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/timers.h>
#include <nvgpu/enabled.h>
#include <nvgpu/bug.h>
#include <nvgpu/ltc.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/hw/gm20b/hw_mc_gm20b.h>
#include <nvgpu/hw/gm20b/hw_ltc_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>
#include <nvgpu/hw/gm20b/hw_pri_ringmaster_gm20b.h>

#include "ltc_gm20b.h"

int gm20b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr)
{
	/* max memory size (MB) to cover */
	u32 max_size = gr->max_comptag_mem;
	/* one tag line covers 128KB */
	u32 max_comptag_lines = max_size << 3U;

	u32 hw_max_comptag_lines =
		ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_init_v();

	u32 cbc_param =
		gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	u32 comptags_per_cacheline =
		ltc_ltcs_ltss_cbc_param_comptags_per_cache_line_v(cbc_param);

	u32 compbit_backing_size;

	int err;

	nvgpu_log_fn(g, " ");

	if (max_comptag_lines == 0U) {
		return 0;
	}

	if (max_comptag_lines > hw_max_comptag_lines) {
		max_comptag_lines = hw_max_comptag_lines;
	}

	compbit_backing_size =
		DIV_ROUND_UP(max_comptag_lines, comptags_per_cacheline) *
		gr->cacheline_size * gr->slices_per_ltc * g->ltc_count;

	/* aligned to 2KB * ltc_count */
	compbit_backing_size +=
		g->ltc_count << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	/* must be a multiple of 64KB */
	compbit_backing_size = roundup(compbit_backing_size, 64*1024);

	max_comptag_lines =
		(compbit_backing_size * comptags_per_cacheline) /
		(gr->cacheline_size * gr->slices_per_ltc * g->ltc_count);

	if (max_comptag_lines > hw_max_comptag_lines) {
		max_comptag_lines = hw_max_comptag_lines;
	}

	nvgpu_log_info(g, "compbit backing store size : %d",
		compbit_backing_size);
	nvgpu_log_info(g, "max comptag lines : %d",
		max_comptag_lines);

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
	gr->compbit_backing_size = compbit_backing_size;

	return 0;
}

int gm20b_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
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

	if (gr->compbit_store.mem.size == 0) {
		return 0;
	}

	while (1) {
		const u32 iter_max = min(min + max_lines - 1, max);
		bool full_cache_op = true;

		nvgpu_mutex_acquire(&g->mm.l2_op_lock);

		nvgpu_log_info(g, "clearing CBC lines %u..%u", min, iter_max);

		if (op == gk20a_cbc_op_clear) {
			gk20a_writel(
				g, ltc_ltcs_ltss_cbc_ctrl2_r(),
				ltc_ltcs_ltss_cbc_ctrl2_clear_lower_bound_f(
					min));
			gk20a_writel(
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

void gm20b_ltc_init_fs_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	u32 reg;

	nvgpu_log_info(g, "initialize gm20b l2");

	g->max_ltc_count = gk20a_readl(g, top_num_ltcs_r());
	g->ltc_count = gk20a_readl(g, pri_ringmaster_enum_ltc_r());
	nvgpu_log_info(g, "%d ltcs out of %d", g->ltc_count, g->max_ltc_count);

	reg = gk20a_readl(g, ltc_ltcs_ltss_cbc_param_r());
	gr->slices_per_ltc = ltc_ltcs_ltss_cbc_param_slices_per_ltc_v(reg);;
	gr->cacheline_size =
		512U << ltc_ltcs_ltss_cbc_param_cache_line_size_v(reg);

	gk20a_writel(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r(),
	g->ltc_count);
	gk20a_writel(g, ltc_ltcs_misc_ltc_num_active_ltcs_r(),
	g->ltc_count);

	gk20a_writel(g, ltc_ltcs_ltss_dstg_cfg0_r(),
		     gk20a_readl(g, ltc_ltc0_lts0_dstg_cfg0_r()) |
		     ltc_ltcs_ltss_dstg_cfg0_vdc_4to2_disable_m());

	/* Disable LTC interrupts */
	reg = gk20a_readl(g, ltc_ltcs_ltss_intr_r());
	reg &= ~ltc_ltcs_ltss_intr_en_evicted_cb_m();
	reg &= ~ltc_ltcs_ltss_intr_en_illegal_compstat_access_m();
	reg &= ~ltc_ltcs_ltss_intr_en_illegal_compstat_m();
	gk20a_writel(g, ltc_ltcs_ltss_intr_r(), reg);
}

void gm20b_ltc_isr(struct gk20a *g)
{
	u32 mc_intr, ltc_intr;
	unsigned int ltc, slice;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	mc_intr = gk20a_readl(g, mc_intr_ltc_r());
	nvgpu_err(g, "mc_ltc_intr: %08x", mc_intr);
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		if ((mc_intr & 1U << ltc) == 0) {
			continue;
		}
		for (slice = 0; slice < g->gr.slices_per_ltc; slice++) {
			ltc_intr = gk20a_readl(g, ltc_ltc0_lts0_intr_r() +
					   ltc_stride * ltc +
					   lts_stride * slice);
			nvgpu_err(g, "ltc%d, slice %d: %08x",
				  ltc, slice, ltc_intr);
			gk20a_writel(g, ltc_ltc0_lts0_intr_r() +
					   ltc_stride * ltc +
					   lts_stride * slice,
				     ltc_intr);
		}
	}
}

u32 gm20b_ltc_cbc_fix_config(struct gk20a *g, int base)
{
	u32 val = gk20a_readl(g, ltc_ltcs_ltss_cbc_num_active_ltcs_r());
	if (val == 2U) {
		return base * 2;
	} else if (val != 1) {
		nvgpu_err(g, "Invalid number of active ltcs: %08x", val);
	}

	return base;
}

/*
 * Performs a full flush of the L2 cache.
 */
void gm20b_flush_ltc(struct gk20a *g)
{
	struct nvgpu_timeout timeout;
	unsigned int ltc;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);

	/* Clean... */
	nvgpu_writel_check(g, ltc_ltcs_ltss_tstg_cmgmt1_r(),
		ltc_ltcs_ltss_tstg_cmgmt1_clean_pending_f() |
		ltc_ltcs_ltss_tstg_cmgmt1_max_cycles_between_cleans_3_f() |
		ltc_ltcs_ltss_tstg_cmgmt1_clean_wait_for_fb_to_pull_true_f() |
		ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_last_class_true_f() |
		ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_normal_class_true_f() |
		ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_first_class_true_f());

	/* Wait on each LTC individually. */
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		u32 op_pending;

		/*
		 * Use 5ms - this should be sufficient time to flush the cache.
		 * On tegra, rough EMC BW available for old tegra chips (newer
		 * chips are strictly faster) can be estimated as follows:
		 *
		 * Lowest reasonable EMC clock speed will be around 102MHz on
		 * t124 for display enabled boards and generally fixed to max
		 * for non-display boards (since they are generally plugged in).
		 *
		 * Thus, the available BW is 64b * 2 * 102MHz = 1.3GB/s. Of that
		 * BW the GPU will likely get about half (display and overhead/
		 * utilization inefficiency eating the rest) so 650MB/s at
		 * worst. Assuming at most 1MB of GPU L2 cache (less for most
		 * chips) worst case is we take 1MB/650MB/s = 1.5ms.
		 *
		 * So 5ms timeout here should be more than sufficient.
		 */
		nvgpu_timeout_init(g, &timeout, 5, NVGPU_TIMER_CPU_TIMER);

		do {
			int cmgmt1 = ltc_ltc0_ltss_tstg_cmgmt1_r() +
				     ltc * ltc_stride;
			op_pending = gk20a_readl(g, cmgmt1);
		} while ((op_pending &
			  ltc_ltc0_ltss_tstg_cmgmt1_clean_pending_f()) &&
			 !nvgpu_timeout_expired_msg(&timeout,
						    "L2 flush timeout!"));
	}

	/* And invalidate. */
	nvgpu_writel_check(g, ltc_ltcs_ltss_tstg_cmgmt0_r(),
	     ltc_ltcs_ltss_tstg_cmgmt0_invalidate_pending_f() |
	     ltc_ltcs_ltss_tstg_cmgmt0_max_cycles_between_invalidates_3_f() |
	     ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_last_class_true_f() |
	     ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_normal_class_true_f() |
	     ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_first_class_true_f());

	/* Wait on each LTC individually. */
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		u32 op_pending;

		/* Again, 5ms. */
		nvgpu_timeout_init(g, &timeout, 5, NVGPU_TIMER_CPU_TIMER);

		do {
			int cmgmt0 = ltc_ltc0_ltss_tstg_cmgmt0_r() +
				     ltc * ltc_stride;
			op_pending = gk20a_readl(g, cmgmt0);
		} while ((op_pending &
			  ltc_ltc0_ltss_tstg_cmgmt0_invalidate_pending_f()) &&
			 !nvgpu_timeout_expired_msg(&timeout,
						    "L2 flush timeout!"));
	}
}

int gm20b_determine_L2_size_bytes(struct gk20a *g)
{
	u32 lts_per_ltc;
	u32 ways;
	u32 sets;
	u32 bytes_per_line;
	u32 active_ltcs;
	u32 cache_size;

	u32 tmp;
	u32 active_sets_value;

	tmp = gk20a_readl(g, ltc_ltc0_lts0_tstg_cfg1_r());
	ways = hweight32(ltc_ltc0_lts0_tstg_cfg1_active_ways_v(tmp));

	active_sets_value = ltc_ltc0_lts0_tstg_cfg1_active_sets_v(tmp);
	if (active_sets_value == ltc_ltc0_lts0_tstg_cfg1_active_sets_all_v()) {
		sets = 64U;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_half_v()) {
		sets = 32U;
	} else if (active_sets_value ==
		 ltc_ltc0_lts0_tstg_cfg1_active_sets_quarter_v()) {
		sets = 16U;
	} else {
		nvgpu_err(g, "Unknown constant %u for active sets",
		       (unsigned)active_sets_value);
		sets = 0U;
	}

	active_ltcs = g->gr.num_fbps;

	/* chip-specific values */
	lts_per_ltc = 2U;
	bytes_per_line = 128U;
	cache_size = active_ltcs * lts_per_ltc * ways * sets * bytes_per_line;

	return cache_size;
}

/*
 * Sets the ZBC color for the passed index.
 */
void gm20b_ltc_set_zbc_color_entry(struct gk20a *g,
					  struct zbc_entry *color_val,
					  u32 index)
{
	u32 i;
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	nvgpu_writel_check(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	for (i = 0;
	     i < ltc_ltcs_ltss_dstg_zbc_color_clear_value__size_1_v(); i++) {
		nvgpu_writel_check(g,
			ltc_ltcs_ltss_dstg_zbc_color_clear_value_r(i),
			color_val->color_l2[i]);
	}
}

/*
 * Sets the ZBC depth for the passed index.
 */
void gm20b_ltc_set_zbc_depth_entry(struct gk20a *g,
					  struct zbc_entry *depth_val,
					  u32 index)
{
	u32 real_index = index + GK20A_STARTOF_ZBC_TABLE;

	nvgpu_writel_check(g, ltc_ltcs_ltss_dstg_zbc_index_r(),
		     ltc_ltcs_ltss_dstg_zbc_index_address_f(real_index));

	nvgpu_writel_check(g,
			ltc_ltcs_ltss_dstg_zbc_depth_clear_value_r(),
			depth_val->depth);
}

void gm20b_ltc_init_cbc(struct gk20a *g, struct gr_gk20a *gr)
{
	u32 max_size = gr->max_comptag_mem;
	u32 max_comptag_lines = max_size << 3U;

	u32 compbit_base_post_divide;
	u64 compbit_base_post_multiply64;
	u64 compbit_store_iova;
	u64 compbit_base_post_divide64;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		compbit_store_iova = nvgpu_mem_get_phys_addr(g,
							&gr->compbit_store.mem);
	} else {
		compbit_store_iova = nvgpu_mem_get_addr(g,
							&gr->compbit_store.mem);
	}

	compbit_base_post_divide64 = compbit_store_iova >>
		ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	do_div(compbit_base_post_divide64, g->ltc_count);
	compbit_base_post_divide = u64_lo32(compbit_base_post_divide64);

	compbit_base_post_multiply64 = ((u64)compbit_base_post_divide *
		g->ltc_count) << ltc_ltcs_ltss_cbc_base_alignment_shift_v();

	if (compbit_base_post_multiply64 < compbit_store_iova) {
		compbit_base_post_divide++;
	}

	/* Bug 1477079 indicates sw adjustment on the posted divided base. */
	if (g->ops.ltc.cbc_fix_config) {
		compbit_base_post_divide =
			g->ops.ltc.cbc_fix_config(g, compbit_base_post_divide);
	}

	gk20a_writel(g, ltc_ltcs_ltss_cbc_base_r(),
		compbit_base_post_divide);

	nvgpu_log(g, gpu_dbg_info | gpu_dbg_map_v | gpu_dbg_pte,
		   "compbit base.pa: 0x%x,%08x cbc_base:0x%08x\n",
		   (u32)(compbit_store_iova >> 32),
		   (u32)(compbit_store_iova & 0xffffffff),
		   compbit_base_post_divide);

	gr->compbit_store.base_hw = compbit_base_post_divide;

	g->ops.ltc.cbc_ctrl(g, gk20a_cbc_op_invalidate,
			    0, max_comptag_lines - 1);

}

void gm20b_ltc_set_enabled(struct gk20a *g, bool enabled)
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

	gk20a_writel(g, ltc_ltcs_ltss_tstg_set_mgmt_2_r(), reg);
}

/*
 * LTC pri addressing
 */
bool gm20b_ltc_pri_is_ltc_addr(struct gk20a *g, u32 addr)
{
	return ((addr >= ltc_pltcg_base_v()) && (addr < ltc_pltcg_extent_v()));
}

bool gm20b_ltc_is_ltcs_ltss_addr(struct gk20a *g, u32 addr)
{
	u32 ltc_shared_base = ltc_ltcs_ltss_v();
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	return (addr >= ltc_shared_base) &&
		(addr < (ltc_shared_base + lts_stride));
}

bool gm20b_ltc_is_ltcn_ltss_addr(struct gk20a *g, u32 addr)
{
	u32 lts_shared_base = ltc_ltc0_ltss_v();
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);
	u32 addr_mask = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE) - 1;
	u32 base_offset = lts_shared_base & addr_mask;
	u32 end_offset = base_offset + lts_stride;

	return (!gm20b_ltc_is_ltcs_ltss_addr(g, addr)) &&
		((addr & addr_mask) >= base_offset) &&
		((addr & addr_mask) < end_offset);
}

static void gm20b_ltc_update_ltc_lts_addr(struct gk20a *g, u32 addr, u32 ltc_num,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index)
{
	u32 num_ltc_slices = g->ops.gr.get_max_lts_per_ltc(g);
	u32 index = *priv_addr_table_index;
	u32 lts_num;
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);
	u32 lts_stride = nvgpu_get_litter_value(g, GPU_LIT_LTS_STRIDE);

	for (lts_num = 0; lts_num < num_ltc_slices; lts_num++) {
		priv_addr_table[index++] = ltc_ltc0_lts0_v() +
						ltc_num * ltc_stride +
						lts_num * lts_stride +
						(addr & (lts_stride - 1));
	}

	*priv_addr_table_index = index;
}

void gm20b_ltc_split_lts_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index)
{
	u32 num_ltc = g->ltc_count;
	u32 i, start, ltc_num = 0;
	u32 pltcg_base = ltc_pltcg_base_v();
	u32 ltc_stride = nvgpu_get_litter_value(g, GPU_LIT_LTC_STRIDE);

	for (i = 0; i < num_ltc; i++) {
		start = pltcg_base + i * ltc_stride;
		if ((addr >= start) && (addr < (start + ltc_stride))) {
			ltc_num = i;
			break;
		}
	}
	gm20b_ltc_update_ltc_lts_addr(g, addr, ltc_num, priv_addr_table,
				priv_addr_table_index);
}

void gm20b_ltc_split_ltc_broadcast_addr(struct gk20a *g, u32 addr,
					u32 *priv_addr_table,
					u32 *priv_addr_table_index)
{
	u32 num_ltc = g->ltc_count;
	u32 ltc_num;

	for (ltc_num = 0; ltc_num < num_ltc; ltc_num++) {
		gm20b_ltc_update_ltc_lts_addr(g, addr, ltc_num,
					priv_addr_table, priv_addr_table_index);
	}
}
