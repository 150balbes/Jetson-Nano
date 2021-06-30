/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>
#include <nvgpu/power_features/cg.h>
#include <nvgpu/power_features/pg.h>
#include <nvgpu/power_features/power_features.h>

int nvgpu_cg_pg_disable(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	gk20a_gr_wait_initialized(g);

	/* disable elpg before clock gating */
	err = nvgpu_pg_elpg_disable(g);
	if (err != 0) {
		nvgpu_err(g, "failed to set disable elpg");
	}
	nvgpu_cg_slcg_gr_perf_ltc_load_disable(g);

	nvgpu_cg_blcg_mode_disable(g);

	nvgpu_cg_elcg_disable(g);

	return err;
}

int nvgpu_cg_pg_enable(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	gk20a_gr_wait_initialized(g);

	nvgpu_cg_elcg_enable(g);

	nvgpu_cg_blcg_mode_enable(g);

	nvgpu_cg_slcg_gr_perf_ltc_load_enable(g);

	err = nvgpu_pg_elpg_enable(g);
	if (err != 0) {
		nvgpu_err(g, "failed to set enable elpg");
	}

	return err;
}
