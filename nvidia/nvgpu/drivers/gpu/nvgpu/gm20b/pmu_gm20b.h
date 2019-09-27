/*
 * GM20B PMU
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

#ifndef NVGPU_GM20B_PMU_GM20B_H
#define NVGPU_GM20B_PMU_GM20B_H

struct gk20a;

int gm20b_load_falcon_ucode(struct gk20a *g, u32 falconidmask);
int gm20b_pmu_setup_elpg(struct gk20a *g);
void pmu_dump_security_fuses_gm20b(struct gk20a *g);
void gm20b_pmu_load_lsf(struct gk20a *g, u32 falcon_id, u32 flags);
int gm20b_pmu_init_acr(struct gk20a *g);
void gm20b_write_dmatrfbase(struct gk20a *g, u32 addr);
bool gm20b_pmu_is_debug_mode_en(struct gk20a *g);
int gm20b_ns_pmu_setup_hw_and_bootstrap(struct gk20a *g);
void gm20b_pmu_setup_apertures(struct gk20a *g);
void gm20b_update_lspmu_cmdline_args(struct gk20a *g);
int gm20b_pmu_setup_hw_and_bl_bootstrap(struct gk20a *g,
	struct hs_acr *acr_desc,
	struct nvgpu_falcon_bl_info *bl_info);
void gm20b_secured_pmu_start(struct gk20a *g);
#endif /*NVGPU_GM20B_PMU_GM20B_H*/
