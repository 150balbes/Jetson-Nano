/*
 * GM20B Graphics
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
#ifndef NVGPU_GM20B_CLK_GM20B_H
#define NVGPU_GM20B_CLK_GM20B_H

#include <nvgpu/lock.h>

struct gk20a;
struct clk_gk20a;

struct nvgpu_clk_pll_debug_data {
	u32 trim_sys_sel_vco_reg;
	u32 trim_sys_sel_vco_val;

	u32 trim_sys_gpc2clk_out_reg;
	u32 trim_sys_gpc2clk_out_val;

	u32 trim_sys_bypassctrl_reg;
	u32 trim_sys_bypassctrl_val;

	u32 trim_sys_gpcpll_cfg_reg;
	u32 trim_sys_gpcpll_dvfs2_reg;
	u32 trim_bcast_gpcpll_dvfs2_reg;

	u32 trim_sys_gpcpll_cfg_val;
	bool trim_sys_gpcpll_cfg_enabled;
	bool trim_sys_gpcpll_cfg_locked;
	bool trim_sys_gpcpll_cfg_sync_on;

	u32 trim_sys_gpcpll_coeff_val;
	u32 trim_sys_gpcpll_coeff_mdiv;
	u32 trim_sys_gpcpll_coeff_ndiv;
	u32 trim_sys_gpcpll_coeff_pldiv;

	u32 trim_sys_gpcpll_dvfs0_val;
	u32 trim_sys_gpcpll_dvfs0_dfs_coeff;
	u32 trim_sys_gpcpll_dvfs0_dfs_det_max;
	u32 trim_sys_gpcpll_dvfs0_dfs_dc_offset;
};

int gm20b_init_clk_setup_sw(struct gk20a *g);

int gm20b_clk_prepare(struct clk_gk20a *clk);
void gm20b_clk_unprepare(struct clk_gk20a *clk);
int gm20b_clk_is_prepared(struct clk_gk20a *clk);
unsigned long gm20b_recalc_rate(struct clk_gk20a *clk, unsigned long parent_rate);
int gm20b_gpcclk_set_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long parent_rate);
long gm20b_round_rate(struct clk_gk20a *clk, unsigned long rate,
		unsigned long *parent_rate);
struct pll_parms *gm20b_get_gpc_pll_parms(void);

int gm20b_clk_pll_reg_write(struct gk20a *g, u32 reg, u32 val);
int gm20b_init_clk_support(struct gk20a *g);
int gm20b_suspend_clk_support(struct gk20a *g);
int gm20b_clk_get_voltage(struct clk_gk20a *clk, u64 *val);
int gm20b_clk_get_gpcclk_clock_counter(struct clk_gk20a *clk, u64 *val);
int gm20b_clk_get_pll_debug_data(struct gk20a *g,
			struct nvgpu_clk_pll_debug_data *d);

/* 1:1 match between post divider settings and divisor value */
static inline u32 nvgpu_pl_to_div(u32 pl)
{
	return pl;
}

static inline u32 nvgpu_div_to_pl(u32 div)
{
	return div;
}

#endif /* NVGPU_GM20B_CLK_GM20B_H */
