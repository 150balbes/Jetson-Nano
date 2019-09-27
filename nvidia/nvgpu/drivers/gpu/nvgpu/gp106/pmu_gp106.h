/*
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

#ifndef NVGPU_PMU_GP106_H
#define NVGPU_PMU_GP106_H

#define gp106_dbg_pmu(g, fmt, arg...) \
	nvgpu_log(g, gpu_dbg_pmu, fmt, ##arg)

struct gk20a;

bool gp106_is_pmu_supported(struct gk20a *g);
u32 gp106_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);
u32 gp106_pmu_pg_engines_list(struct gk20a *g);
int gp106_pg_param_init(struct gk20a *g, u32 pg_engine_id);
bool gp106_pmu_is_lpwr_feature_supported(struct gk20a *g, u32 feature_id);
bool gp106_is_lazy_bootstrap(u32 falcon_id);
bool gp106_is_priv_load(u32 falcon_id);
int gp106_load_falcon_ucode(struct gk20a *g, u32 falconidmask);

void gp106_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
bool gp106_pmu_is_engine_in_reset(struct gk20a *g);
int gp106_pmu_engine_reset(struct gk20a *g, bool do_reset);
void gp106_update_lspmu_cmdline_args(struct gk20a *g);
void gp106_pmu_setup_apertures(struct gk20a *g);

#endif /* NVGPU_PMU_GP106_H */
