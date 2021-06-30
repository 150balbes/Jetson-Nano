/*
 * general p state infrastructure
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
#ifndef NVGPU_PSTATE_H
#define NVGPU_PSTATE_H

#include "clk/clk.h"

#define CTRL_PERF_PSTATE_TYPE_3X	0x3

#define CTRL_PERF_PSTATE_P0		0
#define CTRL_PERF_PSTATE_P5		5
#define CTRL_PERF_PSTATE_P8		8

#define CLK_SET_INFO_MAX_SIZE		(32)

struct gk20a;

struct clk_set_info {
	enum nv_pmu_clk_clkwhich clkwhich;
	u32 nominal_mhz;
	u16 min_mhz;
	u16 max_mhz;
};

struct clk_set_info_list {
	u32 num_info;
	struct clk_set_info clksetinfo[CLK_SET_INFO_MAX_SIZE];
};

struct pstate {
	struct boardobj super;
	u32 num;
	u8 lpwr_entry_idx;
	struct clk_set_info_list clklist;
};

struct pstates {
	struct boardobjgrp_e32 super;
	u32  num_levels;
	struct nvgpu_cond pstate_notifier_wq;
	u32 is_pstate_switch_on;
	struct nvgpu_mutex pstate_mutex; /* protect is_pstate_switch_on */
};

int gk20a_init_pstate_support(struct gk20a *g);
void gk20a_deinit_pstate_support(struct gk20a *g);
int gk20a_init_pstate_pmu_support(struct gk20a *g);

struct clk_set_info *pstate_get_clk_set_info(struct gk20a *g, u32 pstate_num,
		enum nv_pmu_clk_clkwhich clkwhich);
struct pstate *pstate_find(struct gk20a *g, u32 num);

#endif /* NVGPU_PSTATE_H */
