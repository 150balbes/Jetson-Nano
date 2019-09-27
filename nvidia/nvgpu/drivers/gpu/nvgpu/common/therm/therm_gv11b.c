/*
 * GV11B Therm
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/soc.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include "therm_gv11b.h"

#include <nvgpu/hw/gv11b/hw_therm_gv11b.h>

int gv11b_init_therm_setup_hw(struct gk20a *g)
{
	u32 v;

	nvgpu_log_fn(g, " ");

	/* program NV_THERM registers */
	gk20a_writel(g, therm_use_a_r(), therm_use_a_ext_therm_0_enable_f() |
					therm_use_a_ext_therm_1_enable_f()  |
					therm_use_a_ext_therm_2_enable_f());
	gk20a_writel(g, therm_evt_ext_therm_0_r(),
		therm_evt_ext_therm_0_slow_factor_f(0x2));
	gk20a_writel(g, therm_evt_ext_therm_1_r(),
		therm_evt_ext_therm_1_slow_factor_f(0x6));
	gk20a_writel(g, therm_evt_ext_therm_2_r(),
		therm_evt_ext_therm_2_slow_factor_f(0xe));

	gk20a_writel(g, therm_grad_stepping_table_r(0),
		therm_grad_stepping_table_slowdown_factor0_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by1_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by1p5_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by2_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by4_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by8_f()));

	gk20a_writel(g, therm_grad_stepping_table_r(1),
		therm_grad_stepping_table_slowdown_factor0_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by16_f()) |
		therm_grad_stepping_table_slowdown_factor1_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by32_f()) |
		therm_grad_stepping_table_slowdown_factor2_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by32_f()) |
		therm_grad_stepping_table_slowdown_factor3_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by32_f()) |
		therm_grad_stepping_table_slowdown_factor4_f(
		therm_grad_stepping_table_slowdown_factor0_fpdiv_by32_f()));

	v = gk20a_readl(g, therm_clk_timing_r(0));
	v |= therm_clk_timing_grad_slowdown_enabled_f();
	gk20a_writel(g, therm_clk_timing_r(0), v);

	v = gk20a_readl(g, therm_config2_r());
	v |= therm_config2_grad_enable_f(1);
	v |= therm_config2_slowdown_factor_extended_f(1);
	v = set_field(v, therm_config2_grad_step_duration_m(),
			therm_config2_grad_step_duration_f(0));
	gk20a_writel(g, therm_config2_r(), v);

	gk20a_writel(g, therm_grad_stepping1_r(),
			therm_grad_stepping1_pdiv_duration_f(0xbf4));

	v = gk20a_readl(g, therm_grad_stepping0_r());
	v |= therm_grad_stepping0_feature_enable_f();
	gk20a_writel(g, therm_grad_stepping0_r(), v);

	/* disable idle clock slowdown */
	v = therm_clk_slowdown_2_idle_condition_a_select_f(0) |
		therm_clk_slowdown_2_idle_condition_a_type_never_f() |
		therm_clk_slowdown_2_idle_condition_b_type_never_f();
	gk20a_writel(g, therm_clk_slowdown_2_r(0), v);

	return 0;
}

void gv11b_therm_init_elcg_mode(struct gk20a *g, u32 mode, u32 engine)
{
	u32 gate_ctrl;

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG))
		return;

	gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(engine));

	switch (mode) {
	case ELCG_RUN:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_run_f());
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_idle_holdoff_m(),
				therm_gate_ctrl_idle_holdoff_on_f());
		break;
	case ELCG_STOP:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_stop_f());
		break;
	case ELCG_AUTO:
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_clk_m(),
				therm_gate_ctrl_eng_clk_auto_f());
		break;
	default:
		nvgpu_err(g, "invalid elcg mode %d", mode);
	}

	gk20a_writel(g, therm_gate_ctrl_r(engine), gate_ctrl);
}

int gv11b_elcg_init_idle_filters(struct gk20a *g)
{
	u32 gate_ctrl, idle_filter;
	u32 engine_id;
	u32 active_engine_id = 0;
	struct fifo_gk20a *f = &g->fifo;

	if (nvgpu_platform_is_simulation(g)) {
		return 0;
	}

	nvgpu_log_info(g, "init clock/power gate reg");

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];

		gate_ctrl = gk20a_readl(g, therm_gate_ctrl_r(active_engine_id));
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_exp_m(),
			therm_gate_ctrl_eng_idle_filt_exp__prod_f());
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_idle_filt_mant_m(),
			therm_gate_ctrl_eng_idle_filt_mant__prod_f());
		gate_ctrl = set_field(gate_ctrl,
			therm_gate_ctrl_eng_delay_before_m(),
			therm_gate_ctrl_eng_delay_before__prod_f());
		gate_ctrl = set_field(gate_ctrl,
				therm_gate_ctrl_eng_delay_after_m(),
				therm_gate_ctrl_eng_delay_after__prod_f());
		gk20a_writel(g, therm_gate_ctrl_r(active_engine_id), gate_ctrl);
	}

	idle_filter = gk20a_readl(g, therm_fecs_idle_filter_r());
	idle_filter = set_field(idle_filter,
			therm_fecs_idle_filter_value_m(),
			therm_fecs_idle_filter_value__prod_f());
	gk20a_writel(g, therm_fecs_idle_filter_r(), idle_filter);

	idle_filter = gk20a_readl(g, therm_hubmmu_idle_filter_r());
	idle_filter = set_field(idle_filter,
			therm_hubmmu_idle_filter_value_m(),
			therm_hubmmu_idle_filter_value__prod_f());
	gk20a_writel(g, therm_hubmmu_idle_filter_r(), idle_filter);

	return 0;
}
