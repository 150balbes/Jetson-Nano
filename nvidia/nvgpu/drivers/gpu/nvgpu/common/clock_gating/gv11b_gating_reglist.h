/*
 * Copyright (c) 2016-2020, NVIDIA Corporation. All rights reserved.
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

#ifndef GV11B_GATING_REGLIST_H
#define GV11B_GATING_REGLIST_H

#include <nvgpu/types.h>

struct gk20a;

void gv11b_slcg_bus_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_ce2_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_chiplet_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_ctxsw_firmware_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_fb_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_fifo_load_gating_prod(struct gk20a *g,
	bool prod);

void gr_gv11b_slcg_gr_load_gating_prod(struct gk20a *g,
	bool prod);

void ltc_gv11b_slcg_ltc_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_perf_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_priring_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_pwr_csb_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_pmu_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_therm_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_xbar_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_hshub_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_slcg_acb_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_bus_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_ce_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_ctxsw_firmware_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_fb_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_fifo_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_gr_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_ltc_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_pwr_csb_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_pmu_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_xbar_load_gating_prod(struct gk20a *g,
	bool prod);

void gv11b_blcg_hshub_load_gating_prod(struct gk20a *g,
	bool prod);

void gr_gv11b_pg_gr_load_gating_prod(struct gk20a *g,
	bool prod);
#endif /* GV11B_GATING_REGLIST_H */
