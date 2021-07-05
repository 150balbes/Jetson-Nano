/*
 * Copyright (c) 2011 - 2019, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef CLK_GK20A_H
#define CLK_GK20A_H

#include <nvgpu/lock.h>

#if defined(CONFIG_COMMON_CLK)
#include <linux/clk-provider.h>
#endif

#define GPUFREQ_TABLE_END     ~(u32)1
enum {
	/* only one PLL for gk20a */
	GK20A_GPC_PLL = 0,
	/* 2 PLL revisions for gm20b */
	GM20B_GPC_PLL_B1,
	GM20B_GPC_PLL_C1,
};

enum gpc_pll_mode {
	GPC_PLL_MODE_F = 0,	/* fixed frequency mode a.k.a legacy mode */
	GPC_PLL_MODE_DVFS,	/* DVFS mode a.k.a NA mode */
};

struct na_dvfs {
	u32 n_int;
	u32 sdm_din;
	int dfs_coeff;
	int dfs_det_max;
	int dfs_ext_cal;
	int uv_cal;
	int mv;
};

struct pll {
	u32 id;
	u32 clk_in;	/* KHz */
	u32 M;
	u32 N;
	u32 PL;
	u32 freq;	/* KHz */
	bool enabled;
	enum gpc_pll_mode mode;
	struct na_dvfs dvfs;
};

struct pll_parms {
	u32 min_freq, max_freq;	/* KHz */
	u32 min_vco, max_vco;	/* KHz */
	u32 min_u,   max_u;	/* KHz */
	u32 min_M,   max_M;
	u32 min_N,   max_N;
	u32 min_PL,  max_PL;
	/* NA mode parameters*/
	int coeff_slope, coeff_offs; /* coeff = slope * V + offs */
	int uvdet_slope, uvdet_offs; /* uV = slope * det + offs */
	u32 vco_ctrl;
	/*
	 * Timing parameters in us. Lock timeout is applied to locking in fixed
	 * frequency mode and to dynamic ramp in any mode; does not affect lock
	 * latency, since lock/ramp done status bit is polled. NA mode lock and
	 * and IDDQ exit delays set the time of the respective opertaions with
	 * no status polling.
	 */
	u32 lock_timeout;
	u32 na_lock_delay;
	u32 iddq_exit_delay;
	/* NA mode DFS control */
	u32 dfs_ctrl;
};

struct namemap_cfg;

struct clk_gk20a {
	struct gk20a *g;
#if defined(CONFIG_COMMON_CLK)
	struct clk *tegra_clk;
	struct clk *tegra_clk_parent;
	struct clk_hw hw;
#endif
	struct pll gpc_pll;
	struct pll gpc_pll_last;
	struct nvgpu_mutex clk_mutex;
	struct namemap_cfg *clk_namemap;
	u32 namemap_num;
	u32 *namemap_xlat_table;
	bool sw_ready;
	bool clk_hw_on;
	bool debugfs_set;
	int pll_poweron_uv;
	unsigned long dvfs_safe_max_freq;
};

#if defined(CONFIG_COMMON_CLK)
#define to_clk_gk20a(_hw) container_of(_hw, struct clk_gk20a, hw)
#endif

struct gpu_ops;

#define KHZ 1000
#define MHZ 1000000

static inline unsigned long rate_gpc2clk_to_gpu(unsigned long rate)
{
	/* convert the kHz gpc2clk frequency to Hz gpcpll frequency */
	return (rate * KHZ) / 2;
}
static inline unsigned long rate_gpu_to_gpc2clk(unsigned long rate)
{
	/* convert the Hz gpcpll frequency to kHz gpc2clk frequency */
	return (rate * 2) / KHZ;
}

#endif /* CLK_GK20A_H */
