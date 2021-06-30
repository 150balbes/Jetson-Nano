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
#include <nvgpu/pmu.h>
#include <nvgpu/power_features/pg.h>

bool nvgpu_pg_elpg_is_enabled(struct gk20a *g)
{
	bool elpg_enabled;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	elpg_enabled = g->elpg_enabled;
	nvgpu_mutex_release(&g->cg_pg_lock);
	return elpg_enabled;
}

int nvgpu_pg_elpg_enable(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (!g->can_elpg) {
		return 0;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elpg_enabled) {
		err = nvgpu_pmu_pg_global_enable(g, true);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
	return err;
}

int nvgpu_pg_elpg_disable(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (!g->can_elpg) {
		return 0;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elpg_enabled) {
		err = nvgpu_pmu_pg_global_enable(g, false);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
	return err;
}

int nvgpu_pg_elpg_set_elpg_enabled(struct gk20a *g, bool enable)
{
	int err = 0;
	bool change_mode = false;

	nvgpu_log_fn(g, " ");

	if (!g->can_elpg) {
		return 0;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (enable) {
		if (!g->elpg_enabled) {
			change_mode = true;
			g->elpg_enabled = true;
		}
	} else {
		if (g->elpg_enabled) {
			change_mode = true;
			g->elpg_enabled = false;
		}
	}
	if (!change_mode) {
		goto done;
	}

	err = nvgpu_pmu_pg_global_enable(g, enable);
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
	return err;
}
