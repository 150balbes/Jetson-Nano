/*
 * Tegra GK20A GPU Debugger/Profiler Driver
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/vm.h>
#include <nvgpu/atomic.h>
#include <nvgpu/mm.h>
#include <nvgpu/bug.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>
#include <nvgpu/power_features/power_features.h>

#include "gk20a.h"
#include "gr_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "regops_gk20a.h"

#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_perf_gk20a.h>

static void gk20a_perfbuf_reset_streaming(struct gk20a *g)
{
	u32 engine_status;
	u32 num_unread_bytes;

	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_PERFMON));

	engine_status = gk20a_readl(g, perf_pmasys_enginestatus_r());
	WARN_ON(0u ==
		(engine_status & perf_pmasys_enginestatus_rbufempty_empty_f()));

	gk20a_writel(g, perf_pmasys_control_r(),
		perf_pmasys_control_membuf_clear_status_doit_f());

	num_unread_bytes = gk20a_readl(g, perf_pmasys_mem_bytes_r());
	if (num_unread_bytes != 0u) {
		gk20a_writel(g, perf_pmasys_mem_bump_r(), num_unread_bytes);
	}
}

/*
 * API to get first channel from the list of all channels
 * bound to the debug session
 */
struct channel_gk20a *
nvgpu_dbg_gpu_get_session_channel(struct dbg_session_gk20a *dbg_s)
{
	struct dbg_session_channel_data *ch_data;
	struct channel_gk20a *ch;
	struct gk20a *g = dbg_s->g;

	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);
	if (nvgpu_list_empty(&dbg_s->ch_list)) {
		nvgpu_mutex_release(&dbg_s->ch_list_lock);
		return NULL;
	}

	ch_data = nvgpu_list_first_entry(&dbg_s->ch_list,
				   dbg_session_channel_data,
				   ch_entry);
	ch = g->fifo.channel + ch_data->chid;

	nvgpu_mutex_release(&dbg_s->ch_list_lock);

	return ch;
}

void gk20a_dbg_gpu_post_events(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;
	struct gk20a *g = ch->g;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	nvgpu_list_for_each_entry(session_data, &ch->dbg_s_list,
				dbg_session_data, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->dbg_events.events_enabled) {
			nvgpu_log(g, gpu_dbg_gpu_dbg, "posting event on session id %d",
					dbg_s->id);
			nvgpu_log(g, gpu_dbg_gpu_dbg, "%d events pending",
					dbg_s->dbg_events.num_pending_events);

			dbg_s->dbg_events.num_pending_events++;

			nvgpu_dbg_session_post_event(dbg_s);
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);
}

bool gk20a_dbg_gpu_broadcast_stop_trigger(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;
	bool broadcast = false;
	struct gk20a *g = ch->g;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, " ");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	nvgpu_list_for_each_entry(session_data, &ch->dbg_s_list,
				dbg_session_data, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->broadcast_stop_trigger) {
			nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn | gpu_dbg_intr,
					"stop trigger broadcast enabled");
			broadcast = true;
			break;
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);

	return broadcast;
}

int gk20a_dbg_gpu_clear_broadcast_stop_trigger(struct channel_gk20a *ch)
{
	struct dbg_session_data *session_data;
	struct dbg_session_gk20a *dbg_s;
	struct gk20a *g = ch->g;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg | gpu_dbg_intr, " ");

	/* guard against the session list being modified */
	nvgpu_mutex_acquire(&ch->dbg_s_lock);

	nvgpu_list_for_each_entry(session_data, &ch->dbg_s_list,
				dbg_session_data, dbg_s_entry) {
		dbg_s = session_data->dbg_s;
		if (dbg_s->broadcast_stop_trigger) {
			nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn | gpu_dbg_intr,
					"stop trigger broadcast disabled");
			dbg_s->broadcast_stop_trigger = false;
		}
	}

	nvgpu_mutex_release(&ch->dbg_s_lock);

	return 0;
}

u32 nvgpu_set_powergate_locked(struct dbg_session_gk20a *dbg_s,
				bool mode)
{
	u32 err = 0U;
	struct gk20a *g = dbg_s->g;

	if (dbg_s->is_pg_disabled != mode) {
		if (mode == false) {
			g->dbg_powergating_disabled_refcount--;
		}

		/*
		 * Allow powergate disable or enable only if
		 * the global pg disabled refcount is zero
		 */
		if (g->dbg_powergating_disabled_refcount == 0) {
			err = g->ops.dbg_session_ops.dbg_set_powergate(dbg_s,
									mode);
		}

		if (mode) {
			g->dbg_powergating_disabled_refcount++;
		}

		dbg_s->is_pg_disabled = mode;
	}

	return err;
}

int dbg_set_powergate(struct dbg_session_gk20a *dbg_s, bool disable_powergate)
{
	int err = 0;
	struct gk20a *g = dbg_s->g;

	 /* This function must be called with g->dbg_sessions_lock held */

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %s",
		   g->name, disable_powergate ? "disable" : "enable");

	/*
	 * Powergate mode here refers to railgate+powergate+clockgate
	 * so in case slcg/blcg/elcg are disabled and railgating is enabled,
	 * disable railgating and then set is_pg_disabled = true
	 * Similarly re-enable railgating and not other features if they are not
	 * enabled when powermode=MODE_ENABLE
	 */
	if (disable_powergate) {
		/* save off current powergate, clk state.
		 * set gpu module's can_powergate = 0.
		 * set gpu module's clk to max.
		 * while *a* debug session is active there will be no power or
		 * clocking state changes allowed from mainline code (but they
		 * should be saved).
		 */

		nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn,
						"module busy");
		err = gk20a_busy(g);
		if (err) {
			return err;
		}

		err = nvgpu_cg_pg_disable(g);

		if (err == 0) {
			dbg_s->is_pg_disabled = true;
			nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn,
					"pg disabled");
		}
	} else {
		/* restore (can) powergate, clk state */
		/* release pending exceptions to fault/be handled as usual */
		/*TBD: ordering of these? */

		err = nvgpu_cg_pg_enable(g);

		nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn, "module idle");
		gk20a_idle(g);

		if (err == 0) {
			dbg_s->is_pg_disabled = false;
			nvgpu_log(g, gpu_dbg_gpu_dbg | gpu_dbg_fn,
					"pg enabled");
		}
	}

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_gpu_dbg, "%s powergate mode = %s done",
		   g->name, disable_powergate ? "disable" : "enable");
	return err;
}

bool nvgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	if (g->profiler_reservation_count == 0) {
		g->global_profiler_reservation_held = true;
		g->profiler_reservation_count = 1;
		dbg_s->has_profiler_reservation = true;
		prof_obj->has_reservation = true;
		return true;
	}
	return false;
}

bool nvgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	/* Assumes that we've already checked that no global reservation
	 * is in effect.
	 */
	g->profiler_reservation_count++;
	dbg_s->has_profiler_reservation = true;
	prof_obj->has_reservation = true;
	return true;
}

void nvgpu_release_profiler_reservation(struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	g->profiler_reservation_count--;
	if (g->profiler_reservation_count < 0) {
		nvgpu_err(g, "Negative reservation count!");
	}
	dbg_s->has_profiler_reservation = false;
	prof_obj->has_reservation = false;
	if (prof_obj->ch == NULL) {
		g->global_profiler_reservation_held = false;
	}
}

int gk20a_perfbuf_enable_locked(struct gk20a *g, u64 offset, u32 size)
{
	struct mm_gk20a *mm = &g->mm;
	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;
	int err;

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to poweron");
		return err;
	}

	err = g->ops.mm.alloc_inst_block(g, &mm->perfbuf.inst_block);
	if (err) {
		return err;
	}

	g->ops.mm.init_inst_block(&mm->perfbuf.inst_block, mm->perfbuf.vm, 0);

	gk20a_perfbuf_reset_streaming(g);

	virt_addr_lo = u64_lo32(offset);
	virt_addr_hi = u64_hi32(offset);

	/* address and size are aligned to 32 bytes, the lowest bits read back
	 * as zeros */
	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), size);

	/* this field is aligned to 4K */
	inst_pa_page = nvgpu_inst_block_addr(g,	&mm->perfbuf.inst_block) >> 12;

	/* A write to MEM_BLOCK triggers the block bind operation. MEM_BLOCK
	 * should be written last */
	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(inst_pa_page) |
		        nvgpu_aperture_mask(g, &mm->perfbuf.inst_block,
				perf_pmasys_mem_block_target_sys_ncoh_f(),
				perf_pmasys_mem_block_target_sys_coh_f(),
				perf_pmasys_mem_block_target_lfb_f()) |
		        perf_pmasys_mem_block_valid_true_f());

	gk20a_idle(g);
	return 0;
}

/* must be called with dbg_sessions_lock held */
int gk20a_perfbuf_disable_locked(struct gk20a *g)
{
	int err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to poweron");
		return err;
	}

	gk20a_perfbuf_reset_streaming(g);

	gk20a_writel(g, perf_pmasys_outbase_r(), 0);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
			perf_pmasys_outbaseupper_ptr_f(0));
	gk20a_writel(g, perf_pmasys_outsize_r(), 0);

	gk20a_writel(g, perf_pmasys_mem_block_r(),
			perf_pmasys_mem_block_base_f(0) |
			perf_pmasys_mem_block_valid_false_f() |
			perf_pmasys_mem_block_target_f(0));

	gk20a_idle(g);

	return 0;
}
