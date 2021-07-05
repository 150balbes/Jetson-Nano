/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
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


#ifndef NVGPU_POWER_FEATURES_CG_H
#define NVGPU_POWER_FEATURES_CG_H

#include <nvgpu/types.h>

struct gk20a;
struct fifo_gk20a;

void nvgpu_cg_init_gr_load_gating_prod(struct gk20a *g);
void nvgpu_cg_elcg_enable(struct gk20a *g);
void nvgpu_cg_elcg_disable(struct gk20a *g);
void nvgpu_cg_elcg_enable_no_wait(struct gk20a *g);
void nvgpu_cg_elcg_disable_no_wait(struct gk20a *g);
void nvgpu_cg_elcg_set_elcg_enabled(struct gk20a *g, bool enable);

void nvgpu_cg_blcg_mode_enable(struct gk20a *g);
void nvgpu_cg_blcg_mode_disable(struct gk20a *g);
void nvgpu_cg_blcg_fb_ltc_load_enable(struct gk20a *g);
void nvgpu_cg_blcg_fifo_load_enable(struct gk20a *g);
void nvgpu_cg_blcg_pmu_load_enable(struct gk20a *g);
void nvgpu_cg_blcg_ce_load_enable(struct gk20a *g);
void nvgpu_cg_blcg_gr_load_enable(struct gk20a *g);
void nvgpu_cg_blcg_set_blcg_enabled(struct gk20a *g, bool enable);

void nvgpu_cg_slcg_gr_perf_ltc_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_gr_perf_ltc_load_disable(struct gk20a *g);
void nvgpu_cg_slcg_fb_ltc_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_priring_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_fifo_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_pmu_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_ce2_load_enable(struct gk20a *g);
void nvgpu_cg_slcg_set_slcg_enabled(struct gk20a *g, bool enable);

#endif /*NVGPU_POWER_FEATURES_CG_H*/
