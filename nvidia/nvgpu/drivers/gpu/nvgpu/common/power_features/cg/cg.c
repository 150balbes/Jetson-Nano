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

#include <nvgpu/gk20a.h>
#include <nvgpu/enabled.h>
#include <nvgpu/power_features/cg.h>

static void nvgpu_cg_set_mode(struct gk20a *g, int cgmode, int mode_config)
{
	u32 engine_idx;
	u32 active_engine_id = 0;
	struct fifo_engine_info_gk20a *engine_info = NULL;
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_log_fn(g, " ");

	for (engine_idx = 0; engine_idx < f->num_engines; ++engine_idx) {
		active_engine_id = f->active_engines_list[engine_idx];
		engine_info = &f->engine_info[active_engine_id];

		/* gr_engine supports both BLCG and ELCG */
		if ((cgmode == BLCG_MODE) && (engine_info->engine_enum ==
						ENGINE_GR_GK20A)) {
			g->ops.therm.init_blcg_mode(g, (u32)mode_config,
						active_engine_id);
			break;
		} else if (cgmode == ELCG_MODE) {
			g->ops.therm.init_elcg_mode(g, (u32)mode_config,
						active_engine_id);
		} else {
			nvgpu_err(g, "invalid cg mode %d, config %d for "
							"act_eng_id %d",
					cgmode, mode_config, active_engine_id);
		}
	}
}

void nvgpu_cg_elcg_enable_no_wait(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG)) {
		return;
	}

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elcg_enabled) {
		nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_AUTO);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_elcg_disable_no_wait(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG)) {
		return;
	}

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elcg_enabled) {
		nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_RUN);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_elcg_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elcg_enabled) {
		nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_AUTO);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_elcg_disable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->elcg_enabled) {
		nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_RUN);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);

}

void nvgpu_cg_blcg_mode_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->blcg_enabled) {
		nvgpu_cg_set_mode(g, BLCG_MODE, BLCG_AUTO);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);

}

void nvgpu_cg_blcg_mode_disable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (g->blcg_enabled) {
		nvgpu_cg_set_mode(g, BLCG_MODE, BLCG_RUN);
	}
	nvgpu_mutex_release(&g->cg_pg_lock);


}

void nvgpu_cg_blcg_fb_ltc_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->blcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.blcg_fb_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_fb_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.blcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ltc_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_blcg_fifo_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->blcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.blcg_fifo_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_fifo_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_blcg_pmu_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->blcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.blcg_pmu_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_pmu_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_blcg_ce_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->blcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.blcg_ce_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ce_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_blcg_gr_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->blcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.blcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_gr_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_fb_ltc_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_fb_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_fb_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_priring_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_priring_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_priring_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_gr_perf_ltc_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_perf_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_perf_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_gr_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_gr_perf_ltc_load_disable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_gr_load_gating_prod(g, false);
	}
	if (g->ops.clock_gating.slcg_perf_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_perf_load_gating_prod(g, false);
	}
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g, false);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_fifo_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_fifo_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_fifo_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_pmu_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_pmu_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_pmu_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_ce2_load_enable(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}
	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (!g->slcg_enabled) {
		goto done;
	}
	if (g->ops.clock_gating.slcg_ce2_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g, true);
	}
done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_init_gr_load_gating_prod(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&g->cg_pg_lock);

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		goto check_can_blcg;
	}
	if (!g->slcg_enabled) {
		goto check_can_blcg;
	}

	if (g->ops.clock_gating.slcg_bus_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_bus_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_chiplet_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_chiplet_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_gr_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod(g,
				true);
	}
	if (g->ops.clock_gating.slcg_perf_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_perf_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_xbar_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_xbar_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.slcg_hshub_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_hshub_load_gating_prod(g, true);
	}

check_can_blcg:
	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		goto pg_gr_load;
	}
	if (!g->blcg_enabled) {
		goto pg_gr_load;
	}
	if (g->ops.clock_gating.blcg_bus_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_bus_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.blcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_gr_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod(g,
				true);
	}
	if (g->ops.clock_gating.blcg_xbar_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_xbar_load_gating_prod(g, true);
	}
	if (g->ops.clock_gating.blcg_hshub_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_hshub_load_gating_prod(g, true);
	}
pg_gr_load:
	if (g->ops.clock_gating.pg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.pg_gr_load_gating_prod(g, true);
	}

	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_elcg_set_elcg_enabled(struct gk20a *g, bool enable)
{
	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_ELCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_release(&g->cg_pg_lock);
	if (enable) {
		if (!g->elcg_enabled) {
			g->elcg_enabled = true;
			nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_AUTO);
		}
	} else {
		if (g->elcg_enabled) {
			g->elcg_enabled = false;
			nvgpu_cg_set_mode(g, ELCG_MODE, ELCG_RUN);
		}
	}
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_blcg_set_blcg_enabled(struct gk20a *g, bool enable)
{
	bool load = false;

	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_BLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (enable) {
		if (!g->blcg_enabled) {
			load = true;
			g->blcg_enabled = true;
		}
	} else {
		if (g->blcg_enabled) {
			load = true;
			g->blcg_enabled = false;
		}
	}
	if (!load ) {
		goto done;
	}

	if (g->ops.clock_gating.blcg_bus_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_bus_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_ce_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ce_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ctxsw_firmware_load_gating_prod(g,
				enable);
	}
	if (g->ops.clock_gating.blcg_fb_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_fb_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_fifo_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_fifo_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_gr_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_ltc_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_pmu_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_pmu_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_xbar_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_xbar_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.blcg_hshub_load_gating_prod != NULL) {
		g->ops.clock_gating.blcg_hshub_load_gating_prod(g, enable);
	}

done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}

void nvgpu_cg_slcg_set_slcg_enabled(struct gk20a *g, bool enable)
{
	bool load = false;

	nvgpu_log_fn(g, " ");

	if (!nvgpu_is_enabled(g, NVGPU_GPU_CAN_SLCG)) {
		return;
	}

	gk20a_gr_wait_initialized(g);

	nvgpu_mutex_acquire(&g->cg_pg_lock);
	if (enable) {
		if (!g->slcg_enabled) {
			load = true;
			g->slcg_enabled = true;
		}
	} else {
		if (g->slcg_enabled) {
			load = true;
			g->slcg_enabled = false;
		}
	}
	if (!load ) {
		goto done;
	}

	if (g->ops.clock_gating.slcg_bus_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_bus_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_ce2_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ce2_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_chiplet_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_chiplet_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod !=
								NULL) {
		g->ops.clock_gating.slcg_ctxsw_firmware_load_gating_prod(g,
				enable);
	}
	if (g->ops.clock_gating.slcg_fb_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_fb_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_fifo_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_fifo_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_gr_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_gr_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_ltc_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_ltc_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_perf_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_perf_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_priring_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_priring_load_gating_prod(g,
				enable);
	}
	if (g->ops.clock_gating.slcg_pmu_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_pmu_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_xbar_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_xbar_load_gating_prod(g, enable);
	}
	if (g->ops.clock_gating.slcg_hshub_load_gating_prod != NULL) {
		g->ops.clock_gating.slcg_hshub_load_gating_prod(g, enable);
	}

done:
	nvgpu_mutex_release(&g->cg_pg_lock);
}
