/*
 * GK20A Graphics FIFO (gr host)
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <trace/events/gk20a.h>

#include <nvgpu/mm.h>
#include <nvgpu/dma.h>
#include <nvgpu/timers.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/enabled.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/soc.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>
#include <nvgpu/debug.h>
#include <nvgpu/nvhost.h>
#include <nvgpu/barrier.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>
#include <nvgpu/power_features/power_features.h>
#include <nvgpu/power_features/cg.h>

#include "gk20a.h"
#include "mm_gk20a.h"

#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

#define FECS_METHOD_WFI_RESTORE 0x80000
#define FECS_MAILBOX_0_ACK_RESTORE 0x4


static u32 gk20a_fifo_engines_on_id(struct gk20a *g, u32 id, bool is_tsg);

static const char *const pbdma_intr_fault_type_desc[] = {
	"MEMREQ timeout", "MEMACK_TIMEOUT", "MEMACK_EXTRA acks",
	"MEMDAT_TIMEOUT", "MEMDAT_EXTRA acks", "MEMFLUSH noack",
	"MEMOP noack", "LBCONNECT noack", "NONE - was LBREQ",
	"LBACK_TIMEOUT", "LBACK_EXTRA acks", "LBDAT_TIMEOUT",
	"LBDAT_EXTRA acks", "GPFIFO won't fit", "GPPTR invalid",
	"GPENTRY invalid", "GPCRC mismatch", "PBPTR get>put",
	"PBENTRY invld", "PBCRC mismatch", "NONE - was XBARC",
	"METHOD invld", "METHODCRC mismat", "DEVICE sw method",
	"[ENGINE]", "SEMAPHORE invlid", "ACQUIRE timeout",
	"PRI forbidden", "ILLEGAL SYNCPT", "[NO_CTXSW_SEG]",
	"PBSEG badsplit", "SIGNATURE bad"
};

u32 gk20a_fifo_get_engine_ids(struct gk20a *g,
		u32 engine_id[], u32 engine_id_sz,
		u32 engine_enum)
{
	struct fifo_gk20a *f = NULL;
	u32 instance_cnt = 0;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	struct fifo_engine_info_gk20a *info = NULL;

	if (g && engine_id_sz && (engine_enum < ENGINE_INVAL_GK20A)) {
		f = &g->fifo;
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			active_engine_id = f->active_engines_list[engine_id_idx];
			info = &f->engine_info[active_engine_id];

			if (info->engine_enum == engine_enum) {
				if (instance_cnt < engine_id_sz) {
					engine_id[instance_cnt] = active_engine_id;
					++instance_cnt;
				} else {
					nvgpu_log_info(g, "warning engine_id table sz is small %d",
							engine_id_sz);
				}
			}
		}
	}
	return instance_cnt;
}

struct fifo_engine_info_gk20a *gk20a_fifo_get_engine_info(struct gk20a *g, u32 engine_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *info = NULL;

	if (!g) {
		return info;
	}

	f = &g->fifo;

	if (engine_id < f->max_engines) {
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			if (engine_id == f->active_engines_list[engine_id_idx]) {
				info = &f->engine_info[engine_id];
				break;
			}
		}
	}

	if (!info) {
		nvgpu_err(g, "engine_id is not in active list/invalid %d", engine_id);
	}

	return info;
}

bool gk20a_fifo_is_valid_engine_id(struct gk20a *g, u32 engine_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	bool valid = false;

	if (!g) {
		return valid;
	}

	f = &g->fifo;

	if (engine_id < f->max_engines) {
		for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
			if (engine_id == f->active_engines_list[engine_id_idx]) {
				valid = true;
				break;
			}
		}
	}

	if (!valid) {
		nvgpu_err(g, "engine_id is not in active list/invalid %d", engine_id);
	}

	return valid;
}

u32 gk20a_fifo_get_gr_engine_id(struct gk20a *g)
{
	u32 gr_engine_cnt = 0;
	u32 gr_engine_id = FIFO_INVAL_ENGINE_ID;

	/* Consider 1st available GR engine */
	gr_engine_cnt = gk20a_fifo_get_engine_ids(g, &gr_engine_id,
			1, ENGINE_GR_GK20A);

	if (!gr_engine_cnt) {
		nvgpu_err(g, "No GR engine available on this device!");
	}

	return gr_engine_id;
}

u32 gk20a_fifo_get_all_ce_engine_reset_mask(struct gk20a *g)
{
	u32 reset_mask = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *engine_info;
	u32 active_engine_id = 0;

	if (!g) {
		return reset_mask;
	}

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = &f->engine_info[active_engine_id];
		engine_enum = engine_info->engine_enum;

		if ((engine_enum == ENGINE_GRCE_GK20A) ||
			(engine_enum == ENGINE_ASYNC_CE_GK20A)) {
				reset_mask |= engine_info->reset_mask;
		}
	}

	return reset_mask;
}

u32 gk20a_fifo_get_fast_ce_runlist_id(struct gk20a *g)
{
	u32 ce_runlist_id = gk20a_fifo_get_gr_runlist_id(g);
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	struct fifo_engine_info_gk20a *engine_info;
	u32 active_engine_id = 0;

	if (!g) {
		return ce_runlist_id;
	}

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = &f->engine_info[active_engine_id];
		engine_enum = engine_info->engine_enum;

		/* selecet last available ASYNC_CE if available */
		if (engine_enum == ENGINE_ASYNC_CE_GK20A) {
			ce_runlist_id = engine_info->runlist_id;
		}
	}

	return ce_runlist_id;
}

u32 gk20a_fifo_get_gr_runlist_id(struct gk20a *g)
{
	u32 gr_engine_cnt = 0;
	u32 gr_engine_id = FIFO_INVAL_ENGINE_ID;
	struct fifo_engine_info_gk20a *engine_info;
	u32 gr_runlist_id = ~0;

	/* Consider 1st available GR engine */
	gr_engine_cnt = gk20a_fifo_get_engine_ids(g, &gr_engine_id,
			1, ENGINE_GR_GK20A);

	if (!gr_engine_cnt) {
		nvgpu_err(g,
			"No GR engine available on this device!");
		goto end;
	}

	engine_info = gk20a_fifo_get_engine_info(g, gr_engine_id);

	if (engine_info) {
		gr_runlist_id = engine_info->runlist_id;
	} else {
		nvgpu_err(g,
			"gr_engine_id is not in active list/invalid %d", gr_engine_id);
	}

end:
	return gr_runlist_id;
}

bool gk20a_fifo_is_valid_runlist_id(struct gk20a *g, u32 runlist_id)
{
	struct fifo_gk20a *f = NULL;
	u32 engine_id_idx;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;

	if (!g) {
		return false;
	}

	f = &g->fifo;

	for (engine_id_idx = 0; engine_id_idx < f->num_engines; ++engine_id_idx) {
		active_engine_id = f->active_engines_list[engine_id_idx];
		engine_info = gk20a_fifo_get_engine_info(g, active_engine_id);
		if (engine_info && (engine_info->runlist_id == runlist_id)) {
			return true;
		}
	}

	return false;
}

/*
 * Link engine IDs to MMU IDs and vice versa.
 */

static inline u32 gk20a_engine_id_to_mmu_id(struct gk20a *g, u32 engine_id)
{
	u32 fault_id = FIFO_INVAL_ENGINE_ID;
	struct fifo_engine_info_gk20a *engine_info;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		fault_id = engine_info->fault_id;
	} else {
		nvgpu_err(g, "engine_id is not in active list/invalid %d", engine_id);
	}
	return fault_id;
}

static inline u32 gk20a_mmu_id_to_engine_id(struct gk20a *g, u32 fault_id)
{
	u32 engine_id;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_gk20a *f = &g->fifo;

	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if (engine_info->fault_id == fault_id) {
			break;
		}
		active_engine_id = FIFO_INVAL_ENGINE_ID;
	}
	return active_engine_id;
}

int gk20a_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id)
{
	int ret = ENGINE_INVAL_GK20A;

	nvgpu_log_info(g, "engine type %d", engine_type);
	if (engine_type == top_device_info_type_enum_graphics_v()) {
		ret = ENGINE_GR_GK20A;
	} else if ((engine_type >= top_device_info_type_enum_copy0_v()) &&
		(engine_type <= top_device_info_type_enum_copy2_v())) {
		/* Lets consider all the CE engine have separate runlist at this point
		 * We can identify the ENGINE_GRCE_GK20A type CE using runlist_id
		 * comparsion logic with GR runlist_id in init_engine_info() */
			ret = ENGINE_ASYNC_CE_GK20A;
		/* inst_id starts from CE0 to CE2 */
		if (inst_id) {
			*inst_id = (engine_type - top_device_info_type_enum_copy0_v());
		}
	}

	return ret;
}

int gk20a_fifo_init_engine_info(struct fifo_gk20a *f)
{
	struct gk20a *g = f->g;
	u32 i;
	u32 max_info_entries = top_device_info__size_1_v();
	u32 engine_enum = ENGINE_INVAL_GK20A;
	u32 engine_id = FIFO_INVAL_ENGINE_ID;
	u32 runlist_id = ~0;
	u32 pbdma_id = ~0;
	u32 intr_id = ~0;
	u32 reset_id = ~0;
	u32 inst_id  = 0;
	u32 pri_base = 0;
	u32 fault_id = 0;
	u32 gr_runlist_id = ~0;
	bool found_pbdma_for_runlist = false;

	nvgpu_log_fn(g, " ");

	f->num_engines = 0;

	for (i = 0; i < max_info_entries; i++) {
		u32 table_entry = gk20a_readl(f->g, top_device_info_r(i));
		u32 entry = top_device_info_entry_v(table_entry);
		u32 runlist_bit;

		if (entry == top_device_info_entry_enum_v()) {
			if (top_device_info_engine_v(table_entry)) {
				engine_id =
					top_device_info_engine_enum_v(table_entry);
				nvgpu_log_info(g, "info: engine_id %d",
					top_device_info_engine_enum_v(table_entry));
			}


			if (top_device_info_runlist_v(table_entry)) {
				runlist_id =
					top_device_info_runlist_enum_v(table_entry);
				nvgpu_log_info(g, "gr info: runlist_id %d", runlist_id);

				runlist_bit = BIT(runlist_id);

				found_pbdma_for_runlist = false;
				for (pbdma_id = 0; pbdma_id < f->num_pbdma;
								pbdma_id++) {
					if (f->pbdma_map[pbdma_id] &
								runlist_bit) {
						nvgpu_log_info(g,
						"gr info: pbdma_map[%d]=%d",
							pbdma_id,
							f->pbdma_map[pbdma_id]);
						found_pbdma_for_runlist = true;
						break;
					}
				}

				if (!found_pbdma_for_runlist) {
					nvgpu_err(g, "busted pbdma map");
					return -EINVAL;
				}
			}

			if (top_device_info_intr_v(table_entry)) {
				intr_id =
					top_device_info_intr_enum_v(table_entry);
				nvgpu_log_info(g, "gr info: intr_id %d", intr_id);
			}

			if (top_device_info_reset_v(table_entry)) {
				reset_id =
					top_device_info_reset_enum_v(table_entry);
				nvgpu_log_info(g, "gr info: reset_id %d",
						reset_id);
			}
		} else if (entry == top_device_info_entry_engine_type_v()) {
			u32 engine_type =
				top_device_info_type_enum_v(table_entry);
			engine_enum =
				g->ops.fifo.engine_enum_from_type(g,
						engine_type, &inst_id);
		} else if (entry == top_device_info_entry_data_v()) {
			/* gk20a doesn't support device_info_data packet parsing */
			if (g->ops.fifo.device_info_data_parse) {
				g->ops.fifo.device_info_data_parse(g,
					table_entry, &inst_id, &pri_base,
					&fault_id);
			}
		}

		if (!top_device_info_chain_v(table_entry)) {
			if (engine_enum < ENGINE_INVAL_GK20A) {
				struct fifo_engine_info_gk20a *info =
					&g->fifo.engine_info[engine_id];

				info->intr_mask |= BIT(intr_id);
				info->reset_mask |= BIT(reset_id);
				info->runlist_id = runlist_id;
				info->pbdma_id = pbdma_id;
				info->inst_id  = inst_id;
				info->pri_base = pri_base;

				if (engine_enum == ENGINE_GR_GK20A) {
					gr_runlist_id = runlist_id;
				}

				/* GR and GR_COPY shares same runlist_id */
				if ((engine_enum == ENGINE_ASYNC_CE_GK20A) &&
					(gr_runlist_id == runlist_id)) {
						engine_enum = ENGINE_GRCE_GK20A;
				}

				info->engine_enum = engine_enum;

				if (!fault_id && (engine_enum == ENGINE_GRCE_GK20A)) {
					fault_id = 0x1b;
				}
				info->fault_id = fault_id;

				/* engine_id starts from 0 to NV_HOST_NUM_ENGINES */
				f->active_engines_list[f->num_engines] = engine_id;

				++f->num_engines;

				engine_enum = ENGINE_INVAL_GK20A;
			}
		}
	}

	return 0;
}

u32 gk20a_fifo_act_eng_interrupt_mask(struct gk20a *g, u32 act_eng_id)
{
	struct fifo_engine_info_gk20a *engine_info = NULL;

	engine_info = gk20a_fifo_get_engine_info(g, act_eng_id);
	if (engine_info) {
		return engine_info->intr_mask;
	}

	return 0;
}

u32 gk20a_fifo_engine_interrupt_mask(struct gk20a *g)
{
	u32 eng_intr_mask = 0;
	unsigned int i;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 intr_mask;
		active_engine_id = g->fifo.active_engines_list[i];
		intr_mask = g->fifo.engine_info[active_engine_id].intr_mask;
		engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
		if (((engine_enum == ENGINE_GRCE_GK20A) ||
			(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
			(!g->ops.ce2.isr_stall || !g->ops.ce2.isr_nonstall)) {
				continue;
		}

		eng_intr_mask |= intr_mask;
	}

	return eng_intr_mask;
}

void gk20a_fifo_delete_runlist(struct fifo_gk20a *f)
{
	u32 i;
	u32 runlist_id;
	struct fifo_runlist_info_gk20a *runlist;
	struct gk20a *g = NULL;

	if (!f || !f->runlist_info) {
		return;
	}

	g = f->g;

	for (runlist_id = 0; runlist_id < f->max_runlists; runlist_id++) {
		runlist = &f->runlist_info[runlist_id];
		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
			nvgpu_dma_free(g, &runlist->mem[i]);
		}

		nvgpu_kfree(g, runlist->active_channels);
		runlist->active_channels = NULL;

		nvgpu_kfree(g, runlist->active_tsgs);
		runlist->active_tsgs = NULL;

		nvgpu_mutex_destroy(&runlist->runlist_lock);

	}
	memset(f->runlist_info, 0, (sizeof(struct fifo_runlist_info_gk20a) *
		f->max_runlists));

	nvgpu_kfree(g, f->runlist_info);
	f->runlist_info = NULL;
	f->max_runlists = 0;
}

static void gk20a_remove_fifo_support(struct fifo_gk20a *f)
{
	struct gk20a *g = f->g;
	unsigned int i = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_channel_worker_deinit(g);
	/*
	 * Make sure all channels are closed before deleting them.
	 */
	for (; i < f->num_channels; i++) {
		struct channel_gk20a *c = f->channel + i;
		struct tsg_gk20a *tsg = f->tsg + i;

		/*
		 * Could race but worst that happens is we get an error message
		 * from gk20a_free_channel() complaining about multiple closes.
		 */
		if (c->referenceable) {
			__gk20a_channel_kill(c);
		}

		nvgpu_mutex_destroy(&tsg->event_id_list_lock);

		nvgpu_mutex_destroy(&c->ioctl_lock);
		nvgpu_mutex_destroy(&c->joblist.cleanup_lock);
		nvgpu_mutex_destroy(&c->joblist.pre_alloc.read_lock);
		nvgpu_mutex_destroy(&c->sync_lock);
#if defined(CONFIG_GK20A_CYCLE_STATS)
		nvgpu_mutex_destroy(&c->cyclestate.cyclestate_buffer_mutex);
		nvgpu_mutex_destroy(&c->cs_client_mutex);
#endif
		nvgpu_mutex_destroy(&c->dbg_s_lock);

	}

	nvgpu_vfree(g, f->channel);
	nvgpu_vfree(g, f->tsg);
	if (g->ops.mm.is_bar1_supported(g)) {
		nvgpu_dma_unmap_free(g->mm.bar1.vm, &f->userd);
	} else {
		nvgpu_dma_free(g, &f->userd);
	}

	gk20a_fifo_delete_runlist(f);

	nvgpu_kfree(g, f->pbdma_map);
	f->pbdma_map = NULL;
	nvgpu_kfree(g, f->engine_info);
	f->engine_info = NULL;
	nvgpu_kfree(g, f->active_engines_list);
	f->active_engines_list = NULL;
}

/* reads info from hardware and fills in pbmda exception info record */
static inline void get_exception_pbdma_info(
	struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_pbdma_exception_info_gk20a *e =
		&eng_info->pbdma_exception_info;

	u32 pbdma_status_r = e->status_r = gk20a_readl(g,
		   fifo_pbdma_status_r(eng_info->pbdma_id));
	e->id = fifo_pbdma_status_id_v(pbdma_status_r); /* vs. id_hw_v()? */
	e->id_is_chid = fifo_pbdma_status_id_type_v(pbdma_status_r) ==
		fifo_pbdma_status_id_type_chid_v();
	e->chan_status_v  = fifo_pbdma_status_chan_status_v(pbdma_status_r);
	e->next_id_is_chid =
		fifo_pbdma_status_next_id_type_v(pbdma_status_r) ==
		fifo_pbdma_status_next_id_type_chid_v();
	e->next_id = fifo_pbdma_status_next_id_v(pbdma_status_r);
	e->chsw_in_progress =
		fifo_pbdma_status_chsw_v(pbdma_status_r) ==
		fifo_pbdma_status_chsw_in_progress_v();
}

static void fifo_pbdma_exception_status(struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_pbdma_exception_info_gk20a *e;
	get_exception_pbdma_info(g, eng_info);
	e = &eng_info->pbdma_exception_info;

	nvgpu_log_fn(g, "pbdma_id %d, "
		      "id_type %s, id %d, chan_status %d, "
		      "next_id_type %s, next_id %d, "
		      "chsw_in_progress %d",
		      eng_info->pbdma_id,
		      e->id_is_chid ? "chid" : "tsgid", e->id, e->chan_status_v,
		      e->next_id_is_chid ? "chid" : "tsgid", e->next_id,
		      e->chsw_in_progress);
}

/* reads info from hardware and fills in pbmda exception info record */
static inline void get_exception_engine_info(
	struct gk20a *g,
	struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_engine_exception_info_gk20a *e =
		&eng_info->engine_exception_info;
	u32 engine_status_r = e->status_r =
		gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	e->id = fifo_engine_status_id_v(engine_status_r); /* vs. id_hw_v()? */
	e->id_is_chid = fifo_engine_status_id_type_v(engine_status_r) ==
		fifo_engine_status_id_type_chid_v();
	e->ctx_status_v = fifo_engine_status_ctx_status_v(engine_status_r);
	e->faulted =
		fifo_engine_status_faulted_v(engine_status_r) ==
		fifo_engine_status_faulted_true_v();
	e->idle =
		fifo_engine_status_engine_v(engine_status_r) ==
		fifo_engine_status_engine_idle_v();
	e->ctxsw_in_progress =
		fifo_engine_status_ctxsw_v(engine_status_r) ==
		fifo_engine_status_ctxsw_in_progress_v();
}

static void fifo_engine_exception_status(struct gk20a *g,
			       struct fifo_engine_info_gk20a *eng_info)
{
	struct fifo_engine_exception_info_gk20a *e;
	get_exception_engine_info(g, eng_info);
	e = &eng_info->engine_exception_info;

	nvgpu_log_fn(g, "engine_id %d, id_type %s, id %d, ctx_status %d, "
		      "faulted %d, idle %d, ctxsw_in_progress %d, ",
		      eng_info->engine_id, e->id_is_chid ? "chid" : "tsgid",
		      e->id, e->ctx_status_v,
		      e->faulted, e->idle,  e->ctxsw_in_progress);
}

static int init_runlist(struct gk20a *g, struct fifo_gk20a *f)
{
	struct fifo_runlist_info_gk20a *runlist;
	struct fifo_engine_info_gk20a *engine_info;
	unsigned int runlist_id;
	u32 i;
	size_t runlist_size;
	u32 active_engine_id, pbdma_id, engine_id;
	int flags = nvgpu_is_enabled(g, NVGPU_MM_USE_PHYSICAL_SG) ?
		NVGPU_DMA_FORCE_CONTIGUOUS : 0;
	int err = 0;

	nvgpu_log_fn(g, " ");

	f->max_runlists = g->ops.fifo.eng_runlist_base_size();
	f->runlist_info = nvgpu_kzalloc(g,
					sizeof(struct fifo_runlist_info_gk20a) *
					f->max_runlists);
	if (!f->runlist_info) {
		goto clean_up_runlist;
	}

	memset(f->runlist_info, 0, (sizeof(struct fifo_runlist_info_gk20a) *
		f->max_runlists));

	for (runlist_id = 0; runlist_id < f->max_runlists; runlist_id++) {
		runlist = &f->runlist_info[runlist_id];

		runlist->active_channels =
			nvgpu_kzalloc(g, DIV_ROUND_UP(f->num_channels,
						      BITS_PER_BYTE));
		if (!runlist->active_channels) {
			goto clean_up_runlist;
		}

		runlist->active_tsgs =
			nvgpu_kzalloc(g, DIV_ROUND_UP(f->num_channels,
						      BITS_PER_BYTE));
		if (!runlist->active_tsgs) {
			goto clean_up_runlist;
		}

		runlist_size  = f->runlist_entry_size * f->num_runlist_entries;
		nvgpu_log(g, gpu_dbg_info,
				"runlist_entries %d runlist size %zu",
				f->num_runlist_entries, runlist_size);

		for (i = 0; i < MAX_RUNLIST_BUFFERS; i++) {
			err = nvgpu_dma_alloc_flags_sys(g, flags,
							    runlist_size,
							    &runlist->mem[i]);
			if (err) {
				nvgpu_err(g, "memory allocation failed");
				goto clean_up_runlist;
			}
		}

		err = nvgpu_mutex_init(&runlist->runlist_lock);
		if (err != 0) {
			nvgpu_err(g,
				"Error in runlist_lock mutex initialization");
			goto clean_up_runlist;
		}

		/* None of buffers is pinned if this value doesn't change.
		    Otherwise, one of them (cur_buffer) must have been pinned. */
		runlist->cur_buffer = MAX_RUNLIST_BUFFERS;

		for (pbdma_id = 0; pbdma_id < f->num_pbdma; pbdma_id++) {
			if (f->pbdma_map[pbdma_id] & BIT(runlist_id)) {
				runlist->pbdma_bitmask |= BIT(pbdma_id);
			}
		}
		nvgpu_log(g, gpu_dbg_info, "runlist %d : pbdma bitmask 0x%x",
				 runlist_id, runlist->pbdma_bitmask);

		for (engine_id = 0; engine_id < f->num_engines; ++engine_id) {
			active_engine_id = f->active_engines_list[engine_id];
			engine_info = &f->engine_info[active_engine_id];

			if (engine_info && engine_info->runlist_id == runlist_id) {
				runlist->eng_bitmask |= BIT(active_engine_id);
			}
		}
		nvgpu_log(g, gpu_dbg_info, "runlist %d : act eng bitmask 0x%x",
				 runlist_id, runlist->eng_bitmask);
	}

	nvgpu_log_fn(g, "done");
	return 0;

clean_up_runlist:
	gk20a_fifo_delete_runlist(f);
	nvgpu_log_fn(g, "fail");
	return err;
}

u32 gk20a_fifo_intr_0_error_mask(struct gk20a *g)
{
	u32 intr_0_error_mask =
		fifo_intr_0_bind_error_pending_f() |
		fifo_intr_0_sched_error_pending_f() |
		fifo_intr_0_chsw_error_pending_f() |
		fifo_intr_0_fb_flush_timeout_pending_f() |
		fifo_intr_0_dropped_mmu_fault_pending_f() |
		fifo_intr_0_mmu_fault_pending_f() |
		fifo_intr_0_lb_error_pending_f() |
		fifo_intr_0_pio_error_pending_f();

	return intr_0_error_mask;
}

static u32 gk20a_fifo_intr_0_en_mask(struct gk20a *g)
{
	u32 intr_0_en_mask;

	intr_0_en_mask = g->ops.fifo.intr_0_error_mask(g);

	intr_0_en_mask |= fifo_intr_0_runlist_event_pending_f() |
				 fifo_intr_0_pbdma_intr_pending_f();

	return intr_0_en_mask;
}

int gk20a_init_fifo_reset_enable_hw(struct gk20a *g)
{
	u32 intr_stall;
	u32 mask;
	u32 timeout;
	unsigned int i;
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	nvgpu_log_fn(g, " ");

	/* enable pmc pfifo */
	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_FIFO));

	nvgpu_cg_slcg_fifo_load_enable(g);

	nvgpu_cg_blcg_fifo_load_enable(g);

	timeout = gk20a_readl(g, fifo_fb_timeout_r());
	timeout = set_field(timeout, fifo_fb_timeout_period_m(),
			fifo_fb_timeout_period_max_f());
	nvgpu_log_info(g, "fifo_fb_timeout reg val = 0x%08x", timeout);
	gk20a_writel(g, fifo_fb_timeout_r(), timeout);

	/* write pbdma timeout value */
	for (i = 0; i < host_num_pbdma; i++) {
		timeout = gk20a_readl(g, pbdma_timeout_r(i));
		timeout = set_field(timeout, pbdma_timeout_period_m(),
				    pbdma_timeout_period_max_f());
		nvgpu_log_info(g, "pbdma_timeout reg val = 0x%08x", timeout);
		gk20a_writel(g, pbdma_timeout_r(i), timeout);
	}
	if (g->ops.fifo.apply_pb_timeout) {
		g->ops.fifo.apply_pb_timeout(g);
	}

	if (g->ops.fifo.apply_ctxsw_timeout_intr) {
		g->ops.fifo.apply_ctxsw_timeout_intr(g);
	} else {
		timeout = g->fifo_eng_timeout_us;
		timeout = scale_ptimer(timeout,
			ptimer_scalingfactor10x(g->ptimer_src_freq));
		timeout |= fifo_eng_timeout_detection_enabled_f();
		gk20a_writel(g, fifo_eng_timeout_r(), timeout);
	}

	/* clear and enable pbdma interrupt */
	for (i = 0; i < host_num_pbdma; i++) {
		gk20a_writel(g, pbdma_intr_0_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_1_r(i), 0xFFFFFFFF);

		intr_stall = gk20a_readl(g, pbdma_intr_stall_r(i));
		intr_stall &= ~pbdma_intr_stall_lbreq_enabled_f();
		gk20a_writel(g, pbdma_intr_stall_r(i), intr_stall);
		nvgpu_log_info(g, "pbdma id:%u, intr_en_0 0x%08x", i, intr_stall);
		gk20a_writel(g, pbdma_intr_en_0_r(i), intr_stall);
		intr_stall = gk20a_readl(g, pbdma_intr_stall_1_r(i));
		/*
		 * For bug 2082123
		 * Mask the unused HCE_RE_ILLEGAL_OP bit from the interrupt.
		 */
		intr_stall &= ~pbdma_intr_stall_1_hce_illegal_op_enabled_f();
		nvgpu_log_info(g, "pbdma id:%u, intr_en_1 0x%08x", i, intr_stall);
		gk20a_writel(g, pbdma_intr_en_1_r(i), intr_stall);
	}

	/* reset runlist interrupts */
	gk20a_writel(g, fifo_intr_runlist_r(), ~0);

	/* clear and enable pfifo interrupt */
	gk20a_writel(g, fifo_intr_0_r(), 0xFFFFFFFF);
	mask = gk20a_fifo_intr_0_en_mask(g);
	nvgpu_log_info(g, "fifo_intr_en_0 0x%08x", mask);
	gk20a_writel(g, fifo_intr_en_0_r(), mask);
	nvgpu_log_info(g, "fifo_intr_en_1 = 0x80000000");
	gk20a_writel(g, fifo_intr_en_1_r(), 0x80000000);

	nvgpu_log_fn(g, "done");

	return 0;
}

int gk20a_init_fifo_setup_sw_common(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned int chid, i;
	int err = 0;

	nvgpu_log_fn(g, " ");

	f->g = g;

	err = nvgpu_mutex_init(&f->intr.isr.mutex);
	if (err) {
		nvgpu_err(g, "failed to init isr.mutex");
		return err;
	}

	err = nvgpu_mutex_init(&f->engines_reset_mutex);
	if (err) {
		nvgpu_err(g, "failed to init engines_reset_mutex");
		return err;
	}

	g->ops.fifo.init_pbdma_intr_descs(f); /* just filling in data/tables */

	f->num_channels = g->ops.fifo.get_num_fifos(g);
	f->runlist_entry_size =  g->ops.fifo.runlist_entry_size();
	f->num_runlist_entries = fifo_eng_runlist_length_max_v();
	f->num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);
	f->max_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	f->userd_entry_size = 1 << ram_userd_base_shift_v();

	f->channel = nvgpu_vzalloc(g, f->num_channels * sizeof(*f->channel));
	f->tsg = nvgpu_vzalloc(g, f->num_channels * sizeof(*f->tsg));
	f->pbdma_map = nvgpu_kzalloc(g, f->num_pbdma * sizeof(*f->pbdma_map));
	f->engine_info = nvgpu_kzalloc(g, f->max_engines *
				sizeof(*f->engine_info));
	f->active_engines_list = nvgpu_kzalloc(g, f->max_engines * sizeof(u32));

	if (!(f->channel && f->tsg && f->pbdma_map && f->engine_info &&
		f->active_engines_list)) {
		err = -ENOMEM;
		goto clean_up;
	}
	memset(f->active_engines_list, 0xff, (f->max_engines * sizeof(u32)));

	/* pbdma map needs to be in place before calling engine info init */
	for (i = 0; i < f->num_pbdma; ++i) {
		f->pbdma_map[i] = gk20a_readl(g, fifo_pbdma_map_r(i));
	}

	g->ops.fifo.init_engine_info(f);

	err = init_runlist(g, f);
	if (err) {
		nvgpu_err(g, "failed to init runlist");
		goto clean_up;
	}

	nvgpu_init_list_node(&f->free_chs);

	err = nvgpu_mutex_init(&f->free_chs_mutex);
	if (err) {
		nvgpu_err(g, "failed to init free_chs_mutex");
		goto clean_up;
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		gk20a_init_channel_support(g, chid);
		gk20a_init_tsg_support(g, chid);
	}

	err = nvgpu_mutex_init(&f->tsg_inuse_mutex);
	if (err) {
		nvgpu_err(g, "failed to init tsg_inuse_mutex");
		goto clean_up;
	}

	f->remove_support = gk20a_remove_fifo_support;

	f->deferred_reset_pending = false;

	err = nvgpu_mutex_init(&f->deferred_reset_mutex);
	if (err) {
		nvgpu_err(g, "failed to init deferred_reset_mutex");
		goto clean_up;
	}

	nvgpu_log_fn(g, "done");
	return 0;

clean_up:
	nvgpu_err(g, "fail");

	nvgpu_vfree(g, f->channel);
	f->channel = NULL;
	nvgpu_vfree(g, f->tsg);
	f->tsg = NULL;
	nvgpu_kfree(g, f->pbdma_map);
	f->pbdma_map = NULL;
	nvgpu_kfree(g, f->engine_info);
	f->engine_info = NULL;
	nvgpu_kfree(g, f->active_engines_list);
	f->active_engines_list = NULL;

	return err;
}

int gk20a_init_fifo_setup_sw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned int chid;
	u64 userd_base;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (f->sw_ready) {
		nvgpu_log_fn(g, "skip init");
		return 0;
	}

	err = gk20a_init_fifo_setup_sw_common(g);
	if (err) {
		nvgpu_err(g, "fail: err: %d", err);
		return err;
	}

	if (g->ops.mm.is_bar1_supported(g)) {
		err = nvgpu_dma_alloc_map_sys(g->mm.bar1.vm,
				   f->userd_entry_size * f->num_channels,
				   &f->userd);
	} else {
		err = nvgpu_dma_alloc_sys(g, f->userd_entry_size *
				f->num_channels, &f->userd);
	}
	if (err) {
		nvgpu_err(g, "userd memory allocation failed");
		goto clean_up;
	}
	nvgpu_log(g, gpu_dbg_map, "userd gpu va = 0x%llx", f->userd.gpu_va);

	userd_base = nvgpu_mem_get_addr(g, &f->userd);
	for (chid = 0; chid < f->num_channels; chid++) {
		f->channel[chid].userd_iova = userd_base +
			chid * f->userd_entry_size;
		f->channel[chid].userd_gpu_va =
			f->userd.gpu_va + chid * f->userd_entry_size;
	}

	err = nvgpu_channel_worker_init(g);
	if (err) {
		goto clean_up;
	}

	f->sw_ready = true;

	nvgpu_log_fn(g, "done");
	return 0;

clean_up:
	nvgpu_log_fn(g, "fail");
	if (nvgpu_mem_is_valid(&f->userd)) {
		if (g->ops.mm.is_bar1_supported(g)) {
			nvgpu_dma_unmap_free(g->mm.bar1.vm, &f->userd);
		} else {
			nvgpu_dma_free(g, &f->userd);
		}
	}

	return err;
}

void gk20a_fifo_handle_runlist_event(struct gk20a *g)
{
	u32 runlist_event = gk20a_readl(g, fifo_intr_runlist_r());

	nvgpu_log(g, gpu_dbg_intr, "runlist event %08x",
		  runlist_event);

	gk20a_writel(g, fifo_intr_runlist_r(), runlist_event);
}

int gk20a_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_log_fn(g, " ");

	/* test write, read through bar1 @ userd region before
	 * turning on the snooping */
	{
		struct fifo_gk20a *f = &g->fifo;
		u32 v, v1 = 0x33, v2 = 0x55;

		u32 bar1_vaddr = f->userd.gpu_va;
		volatile u32 *cpu_vaddr = f->userd.cpu_va;

		nvgpu_log_info(g, "test bar1 @ vaddr 0x%x",
			   bar1_vaddr);

		v = gk20a_bar1_readl(g, bar1_vaddr);

		*cpu_vaddr = v1;
		nvgpu_mb();

		if (v1 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvgpu_err(g, "bar1 broken @ gk20a: CPU wrote 0x%x, \
				GPU read 0x%x", *cpu_vaddr, gk20a_bar1_readl(g, bar1_vaddr));
			return -EINVAL;
		}

		gk20a_bar1_writel(g, bar1_vaddr, v2);

		if (v2 != gk20a_bar1_readl(g, bar1_vaddr)) {
			nvgpu_err(g, "bar1 broken @ gk20a: GPU wrote 0x%x, \
				CPU read 0x%x", gk20a_bar1_readl(g, bar1_vaddr), *cpu_vaddr);
			return -EINVAL;
		}

		/* is it visible to the cpu? */
		if (*cpu_vaddr != v2) {
			nvgpu_err(g,
				"cpu didn't see bar1 write @ %p!",
				cpu_vaddr);
		}

		/* put it back */
		gk20a_bar1_writel(g, bar1_vaddr, v);
	}

	/*XXX all manner of flushes and caching worries, etc */

	/* set the base for the userd region now */
	gk20a_writel(g, fifo_bar1_base_r(),
			fifo_bar1_base_ptr_f(f->userd.gpu_va >> 12) |
			fifo_bar1_base_valid_true_f());

	nvgpu_log_fn(g, "done");

	return 0;
}

int gk20a_init_fifo_support(struct gk20a *g)
{
	u32 err;

	err = g->ops.fifo.setup_sw(g);
	if (err) {
		return err;
	}

	if (g->ops.fifo.init_fifo_setup_hw) {
		err = g->ops.fifo.init_fifo_setup_hw(g);
	}
	if (err) {
		return err;
	}

	return err;
}

/* return with a reference to the channel, caller must put it back */
struct channel_gk20a *
gk20a_refch_from_inst_ptr(struct gk20a *g, u64 inst_ptr)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned int ci;
	if (unlikely(!f->channel)) {
		return NULL;
	}
	for (ci = 0; ci < f->num_channels; ci++) {
		struct channel_gk20a *ch;
		u64 ch_inst_ptr;

		ch = gk20a_channel_from_id(g, ci);
		/* only alive channels are searched */
		if (!ch) {
			continue;
		}

		ch_inst_ptr = nvgpu_inst_block_addr(g, &ch->inst_block);
		if (inst_ptr == ch_inst_ptr) {
			return ch;
		}

		gk20a_channel_put(ch);
	}
	return NULL;
}

/* fault info/descriptions.
 * tbd: move to setup
 *  */
static const char * const gk20a_fault_type_descs[] = {
	 "pde", /*fifo_intr_mmu_fault_info_type_pde_v() == 0 */
	 "pde size",
	 "pte",
	 "va limit viol",
	 "unbound inst",
	 "priv viol",
	 "ro viol",
	 "wo viol",
	 "pitch mask",
	 "work creation",
	 "bad aperture",
	 "compression failure",
	 "bad kind",
	 "region viol",
	 "dual ptes",
	 "poisoned",
};
/* engine descriptions */
static const char * const engine_subid_descs[] = {
	"gpc",
	"hub",
};

static const char * const gk20a_hub_client_descs[] = {
	"vip", "ce0", "ce1", "dniso", "fe", "fecs", "host", "host cpu",
	"host cpu nb", "iso", "mmu", "mspdec", "msppp", "msvld",
	"niso", "p2p", "pd", "perf", "pmu", "raster twod", "scc",
	"scc nb", "sec", "ssync", "gr copy", "xv", "mmu nb",
	"msenc", "d falcon", "sked", "a falcon", "n/a",
};

static const char * const gk20a_gpc_client_descs[] = {
	"l1 0", "t1 0", "pe 0",
	"l1 1", "t1 1", "pe 1",
	"l1 2", "t1 2", "pe 2",
	"l1 3", "t1 3", "pe 3",
	"rast", "gcc", "gpccs",
	"prop 0", "prop 1", "prop 2", "prop 3",
	"l1 4", "t1 4", "pe 4",
	"l1 5", "t1 5", "pe 5",
	"l1 6", "t1 6", "pe 6",
	"l1 7", "t1 7", "pe 7",
};

static const char * const does_not_exist[] = {
	"does not exist"
};

/* fill in mmu fault desc */
void gk20a_fifo_get_mmu_fault_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->fault_type >= ARRAY_SIZE(gk20a_fault_type_descs)) {
		WARN_ON(mmfault->fault_type >=
				ARRAY_SIZE(gk20a_fault_type_descs));
	} else {
		mmfault->fault_type_desc =
			 gk20a_fault_type_descs[mmfault->fault_type];
	}
}

/* fill in mmu fault client description */
void gk20a_fifo_get_mmu_fault_client_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gk20a_hub_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gk20a_hub_client_descs));
	} else {
		mmfault->client_id_desc =
			 gk20a_hub_client_descs[mmfault->client_id];
	}
}

/* fill in mmu fault gpc description */
void gk20a_fifo_get_mmu_fault_gpc_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gk20a_gpc_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gk20a_gpc_client_descs));
	} else {
		mmfault->client_id_desc =
			 gk20a_gpc_client_descs[mmfault->client_id];
	}
}

static void get_exception_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
	struct mmu_fault_info *mmfault)
{
	g->ops.fifo.get_mmu_fault_info(g, mmu_fault_id, mmfault);

	/* parse info */
	mmfault->fault_type_desc =  does_not_exist[0];
	if (g->ops.fifo.get_mmu_fault_desc) {
		g->ops.fifo.get_mmu_fault_desc(mmfault);
	}

	if (mmfault->client_type >= ARRAY_SIZE(engine_subid_descs)) {
		WARN_ON(mmfault->client_type >= ARRAY_SIZE(engine_subid_descs));
		mmfault->client_type_desc = does_not_exist[0];
	} else {
		mmfault->client_type_desc =
				 engine_subid_descs[mmfault->client_type];
	}

	mmfault->client_id_desc = does_not_exist[0];
	if ((mmfault->client_type ==
		fifo_intr_mmu_fault_info_engine_subid_hub_v())
		&& g->ops.fifo.get_mmu_fault_client_desc) {
		g->ops.fifo.get_mmu_fault_client_desc(mmfault);
	} else if ((mmfault->client_type ==
			fifo_intr_mmu_fault_info_engine_subid_gpc_v())
			&& g->ops.fifo.get_mmu_fault_gpc_desc) {
		g->ops.fifo.get_mmu_fault_gpc_desc(mmfault);
	}
}

/* reads info from hardware and fills in mmu fault info record */
void gk20a_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
	struct mmu_fault_info *mmfault)
{
	u32 fault_info;
	u32 addr_lo, addr_hi;

	nvgpu_log_fn(g, "mmu_fault_id %d", mmu_fault_id);

	memset(mmfault, 0, sizeof(*mmfault));

	fault_info = gk20a_readl(g,
		fifo_intr_mmu_fault_info_r(mmu_fault_id));
	mmfault->fault_type =
		fifo_intr_mmu_fault_info_type_v(fault_info);
	mmfault->access_type =
		fifo_intr_mmu_fault_info_write_v(fault_info);
	mmfault->client_type =
		fifo_intr_mmu_fault_info_engine_subid_v(fault_info);
	mmfault->client_id =
		fifo_intr_mmu_fault_info_client_v(fault_info);

	addr_lo = gk20a_readl(g, fifo_intr_mmu_fault_lo_r(mmu_fault_id));
	addr_hi = gk20a_readl(g, fifo_intr_mmu_fault_hi_r(mmu_fault_id));
	mmfault->fault_addr = hi32_lo32_to_u64(addr_hi, addr_lo);
	/* note:ignoring aperture on gk20a... */
	mmfault->inst_ptr = fifo_intr_mmu_fault_inst_ptr_v(
		 gk20a_readl(g, fifo_intr_mmu_fault_inst_r(mmu_fault_id)));
	/* note: inst_ptr is a 40b phys addr.  */
	mmfault->inst_ptr <<= fifo_intr_mmu_fault_inst_ptr_align_shift_v();
}

void gk20a_fifo_reset_engine(struct gk20a *g, u32 engine_id)
{
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_engine_info_gk20a *engine_info;

	nvgpu_log_fn(g, " ");

	if (!g) {
		return;
	}

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		engine_enum = engine_info->engine_enum;
	}

	if (engine_enum == ENGINE_INVAL_GK20A) {
		nvgpu_err(g, "unsupported engine_id %d", engine_id);
	}

	if (engine_enum == ENGINE_GR_GK20A) {
		if (g->support_pmu) {
			if (nvgpu_pg_elpg_disable(g) != 0 ) {
				nvgpu_err(g, "failed to set disable elpg");
			}
		}

#ifdef CONFIG_GK20A_CTXSW_TRACE
		/*
		 * Resetting engine will alter read/write index. Need to flush
		 * circular buffer before re-enabling FECS.
		 */
		if (g->ops.fecs_trace.reset)
			g->ops.fecs_trace.reset(g);
#endif
		if (!nvgpu_platform_is_simulation(g)) {
			/*HALT_PIPELINE method, halt GR engine*/
			if (gr_gk20a_halt_pipe(g)) {
				nvgpu_err(g, "failed to HALT gr pipe");
			}
			/*
			 * resetting engine using mc_enable_r() is not
			 * enough, we do full init sequence
			 */
			nvgpu_log(g, gpu_dbg_info, "resetting gr engine");
			gk20a_gr_reset(g);
		} else {
			nvgpu_log(g, gpu_dbg_info,
				"HALT gr pipe not supported and "
				"gr cannot be reset without halting gr pipe");
		}
		if (g->support_pmu) {
			if (nvgpu_pg_elpg_enable(g) != 0 ) {
				nvgpu_err(g, "failed to set enable elpg");
			}
		}
	}
	if ((engine_enum == ENGINE_GRCE_GK20A) ||
		(engine_enum == ENGINE_ASYNC_CE_GK20A)) {
			g->ops.mc.reset(g, engine_info->reset_mask);
	}
}

static void gk20a_fifo_handle_chsw_fault(struct gk20a *g)
{
	u32 intr;

	intr = gk20a_readl(g, fifo_intr_chsw_error_r());
	nvgpu_err(g, "chsw: %08x", intr);
	gk20a_fecs_dump_falcon_stats(g);
	gk20a_writel(g, fifo_intr_chsw_error_r(), intr);
}

static void gk20a_fifo_handle_dropped_mmu_fault(struct gk20a *g)
{
	u32 fault_id = gk20a_readl(g, fifo_intr_mmu_fault_id_r());
	nvgpu_err(g, "dropped mmu fault (0x%08x)", fault_id);
}

bool gk20a_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid)
{
	return (engine_subid == fifo_intr_mmu_fault_info_engine_subid_gpc_v());
}

bool gk20a_fifo_should_defer_engine_reset(struct gk20a *g, u32 engine_id,
		u32 engine_subid, bool fake_fault)
{
	u32 engine_enum = ENGINE_INVAL_GK20A;
	struct fifo_engine_info_gk20a *engine_info;

	if (!g) {
		return false;
	}

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		engine_enum = engine_info->engine_enum;
	}

	if (engine_enum == ENGINE_INVAL_GK20A) {
		return false;
	}

	/* channel recovery is only deferred if an sm debugger
	   is attached and has MMU debug mode is enabled */
	if (!g->ops.gr.sm_debugger_attached(g) ||
	    !g->ops.fb.is_debug_mode_enabled(g)) {
		return false;
	}

	/* if this fault is fake (due to RC recovery), don't defer recovery */
	if (fake_fault) {
		return false;
	}

	if (engine_enum != ENGINE_GR_GK20A) {
		return false;
	}

	return g->ops.fifo.is_fault_engine_subid_gpc(g, engine_subid);
}

/* caller must hold a channel reference */
static bool gk20a_fifo_ch_timeout_debug_dump_state(struct gk20a *g,
		struct channel_gk20a *refch)
{
	bool verbose = false;
	if (!refch) {
		return verbose;
	}

	if (nvgpu_is_error_notifier_set(refch,
			NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT)) {
		verbose = refch->timeout_debug_dump;
	}

	return verbose;
}

/* caller must hold a channel reference */
static void gk20a_fifo_set_has_timedout_and_wake_up_wqs(struct gk20a *g,
		struct channel_gk20a *refch)
{
	if (refch) {
		/* mark channel as faulted */
		gk20a_channel_set_timedout(refch);

		/* unblock pending waits */
		nvgpu_cond_broadcast_interruptible(&refch->semaphore_wq);
		nvgpu_cond_broadcast_interruptible(&refch->notifier_wq);
	}
}

/* caller must hold a channel reference */
bool gk20a_fifo_error_ch(struct gk20a *g,
		struct channel_gk20a *refch)
{
	bool verbose;

	verbose = gk20a_fifo_ch_timeout_debug_dump_state(g, refch);
	gk20a_fifo_set_has_timedout_and_wake_up_wqs(g, refch);

	return verbose;
}

bool gk20a_fifo_error_tsg(struct gk20a *g,
		struct tsg_gk20a *tsg)
{
	struct channel_gk20a *ch = NULL;
	bool verbose = false;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		if (gk20a_channel_get(ch)) {
			if (gk20a_fifo_error_ch(g, ch)) {
				verbose = true;
			}
			gk20a_channel_put(ch);
		}
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	return verbose;

}
/* caller must hold a channel reference */
void gk20a_fifo_set_ctx_mmu_error_ch(struct gk20a *g,
		struct channel_gk20a *refch)
{
	nvgpu_err(g,
		"channel %d generated a mmu fault", refch->chid);
	g->ops.fifo.set_error_notifier(refch,
				NVGPU_ERR_NOTIFIER_FIFO_ERROR_MMU_ERR_FLT);
}

void gk20a_fifo_set_ctx_mmu_error_tsg(struct gk20a *g,
		struct tsg_gk20a *tsg)
{
	struct channel_gk20a *ch = NULL;

	nvgpu_err(g,
		"TSG %d generated a mmu fault", tsg->tsgid);

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		if (gk20a_channel_get(ch)) {
			gk20a_fifo_set_ctx_mmu_error_ch(g, ch);
			gk20a_channel_put(ch);
		}
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

}

void gk20a_fifo_abort_tsg(struct gk20a *g, struct tsg_gk20a *tsg, bool preempt)
{
	struct channel_gk20a *ch = NULL;

	nvgpu_log_fn(g, " ");

	g->ops.fifo.disable_tsg(tsg);

	if (preempt) {
		g->ops.fifo.preempt_tsg(g, tsg);
	}

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		if (gk20a_channel_get(ch)) {
			gk20a_channel_set_timedout(ch);
			if (ch->g->ops.fifo.ch_abort_clean_up) {
				ch->g->ops.fifo.ch_abort_clean_up(ch);
			}
			gk20a_channel_put(ch);
		}
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);
}

int gk20a_fifo_deferred_reset(struct gk20a *g, struct channel_gk20a *ch)
{
	unsigned long engine_id, engines = 0U;
	struct tsg_gk20a *tsg;
	bool deferred_reset_pending;
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	nvgpu_mutex_acquire(&f->deferred_reset_mutex);
	deferred_reset_pending = g->fifo.deferred_reset_pending;
	nvgpu_mutex_release(&f->deferred_reset_mutex);

	if (!deferred_reset_pending) {
		nvgpu_mutex_release(&g->dbg_sessions_lock);
		return 0;
	}

	gr_gk20a_disable_ctxsw(g);

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg != NULL) {
		engines = gk20a_fifo_engines_on_id(g, tsg->tsgid, true);
	} else {
		nvgpu_err(g, "chid: %d is not bound to tsg", ch->chid);
	}

	if (engines == 0U) {
		goto clean_up;
	}

	/*
	 * If deferred reset is set for an engine, and channel is running
	 * on that engine, reset it
	 */
	for_each_set_bit(engine_id, &g->fifo.deferred_fault_engines, 32) {
		if (BIT(engine_id) & engines) {
			gk20a_fifo_reset_engine(g, engine_id);
		}
	}

	nvgpu_mutex_acquire(&f->deferred_reset_mutex);
	g->fifo.deferred_fault_engines = 0;
	g->fifo.deferred_reset_pending = false;
	nvgpu_mutex_release(&f->deferred_reset_mutex);

clean_up:
	gr_gk20a_enable_ctxsw(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return 0;
}

static bool gk20a_fifo_handle_mmu_fault_locked(
	struct gk20a *g,
	u32 mmu_fault_engines, /* queried from HW if 0 */
	u32 hw_id, /* queried from HW if ~(u32)0 OR mmu_fault_engines == 0*/
	bool id_is_tsg)
{
	bool fake_fault;
	unsigned long fault_id;
	unsigned long engine_mmu_fault_id;
	bool verbose = true;
	u32 grfifo_ctl;

	bool deferred_reset_pending = false;
	struct fifo_gk20a *f = &g->fifo;

	nvgpu_log_fn(g, " ");

	/* Disable power management */
	if (g->support_pmu) {
		if (nvgpu_cg_pg_disable(g) != 0) {
			nvgpu_warn(g, "fail to disable power mgmt");
		}
	}

	/* Disable fifo access */
	grfifo_ctl = gk20a_readl(g, gr_gpfifo_ctl_r());
	grfifo_ctl &= ~gr_gpfifo_ctl_semaphore_access_f(1);
	grfifo_ctl &= ~gr_gpfifo_ctl_access_f(1);

	gk20a_writel(g, gr_gpfifo_ctl_r(),
		grfifo_ctl | gr_gpfifo_ctl_access_f(0) |
		gr_gpfifo_ctl_semaphore_access_f(0));

	if (mmu_fault_engines) {
		fault_id = mmu_fault_engines;
		fake_fault = true;
	} else {
		fault_id = gk20a_readl(g, fifo_intr_mmu_fault_id_r());
		fake_fault = false;
		gk20a_debug_dump(g);
	}

	nvgpu_mutex_acquire(&f->deferred_reset_mutex);
	g->fifo.deferred_reset_pending = false;
	nvgpu_mutex_release(&f->deferred_reset_mutex);

	/* go through all faulted engines */
	for_each_set_bit(engine_mmu_fault_id, &fault_id, 32) {
		/* bits in fifo_intr_mmu_fault_id_r do not correspond 1:1 to
		 * engines. Convert engine_mmu_id to engine_id */
		u32 engine_id = gk20a_mmu_id_to_engine_id(g,
					engine_mmu_fault_id);
		struct mmu_fault_info mmfault_info;
		struct channel_gk20a *ch = NULL;
		struct tsg_gk20a *tsg = NULL;
		struct channel_gk20a *refch = NULL;
		/* read and parse engine status */
		u32 status = gk20a_readl(g, fifo_engine_status_r(engine_id));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);
		bool ctxsw = (ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_switch_v()
				|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_save_v()
				|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v());

		get_exception_mmu_fault_info(g, engine_mmu_fault_id,
						 &mmfault_info);
		trace_gk20a_mmu_fault(mmfault_info.fault_addr,
				      mmfault_info.fault_type,
				      mmfault_info.access_type,
				      mmfault_info.inst_ptr,
				      engine_id,
				      mmfault_info.client_type_desc,
				      mmfault_info.client_id_desc,
				      mmfault_info.fault_type_desc);
		nvgpu_err(g, "%s mmu fault on engine %d, "
			   "engine subid %d (%s), client %d (%s), "
			   "addr 0x%llx, type %d (%s), access_type 0x%08x,"
			   "inst_ptr 0x%llx",
			   fake_fault ? "fake" : "",
			   engine_id,
			   mmfault_info.client_type,
			   mmfault_info.client_type_desc,
			   mmfault_info.client_id, mmfault_info.client_id_desc,
			   mmfault_info.fault_addr,
			   mmfault_info.fault_type,
			   mmfault_info.fault_type_desc,
			   mmfault_info.access_type, mmfault_info.inst_ptr);

		if (ctxsw) {
			gk20a_fecs_dump_falcon_stats(g);
			nvgpu_err(g, "gr_status_r : 0x%x",
					gk20a_readl(g, gr_status_r()));
		}

		/* get the channel/TSG */
		if (fake_fault) {
			/* use next_id if context load is failing */
			u32 id, type;

			if (hw_id == ~(u32)0) {
				id = (ctx_status ==
				      fifo_engine_status_ctx_status_ctxsw_load_v()) ?
					fifo_engine_status_next_id_v(status) :
					fifo_engine_status_id_v(status);
				type = (ctx_status ==
					fifo_engine_status_ctx_status_ctxsw_load_v()) ?
					fifo_engine_status_next_id_type_v(status) :
					fifo_engine_status_id_type_v(status);
			} else {
				id = hw_id;
				type = id_is_tsg ?
					fifo_engine_status_id_type_tsgid_v() :
					fifo_engine_status_id_type_chid_v();
			}

			if (type == fifo_engine_status_id_type_tsgid_v()) {
				tsg = &g->fifo.tsg[id];
			} else if (type == fifo_engine_status_id_type_chid_v()) {
				ch = &g->fifo.channel[id];
				refch = gk20a_channel_get(ch);
				if (refch != NULL) {
					tsg = tsg_gk20a_from_ch(refch);
				}
			}
		} else {
			/* read channel based on instruction pointer */
			ch = gk20a_refch_from_inst_ptr(g,
					mmfault_info.inst_ptr);
			refch = ch;
			if (refch != NULL) {
				tsg = tsg_gk20a_from_ch(refch);
			}
		}

		/* check if engine reset should be deferred */
		if (engine_id != FIFO_INVAL_ENGINE_ID) {
			bool defer = gk20a_fifo_should_defer_engine_reset(g,
					engine_id, mmfault_info.client_type,
					fake_fault);
			if ((ch || tsg) && defer) {
				g->fifo.deferred_fault_engines |= BIT(engine_id);

				/* handled during channel free */
				nvgpu_mutex_acquire(&f->deferred_reset_mutex);
				g->fifo.deferred_reset_pending = true;
				nvgpu_mutex_release(&f->deferred_reset_mutex);

				deferred_reset_pending = true;

				nvgpu_log(g, gpu_dbg_intr | gpu_dbg_gpu_dbg,
					   "sm debugger attached,"
					   " deferring channel recovery to channel free");
			} else {
				gk20a_fifo_reset_engine(g, engine_id);
			}
		}

#ifdef CONFIG_GK20A_CTXSW_TRACE
		if (tsg) {
			gk20a_ctxsw_trace_tsg_reset(g, tsg);
		}
#endif
		/*
		 * Disable the channel/TSG from hw and increment syncpoints.
		 */
		if (tsg) {
			if (deferred_reset_pending) {
				gk20a_disable_tsg(tsg);
			} else {
				if (!fake_fault) {
					gk20a_fifo_set_ctx_mmu_error_tsg(g,
									 tsg);
				}
				verbose = gk20a_fifo_error_tsg(g, tsg);
				gk20a_fifo_abort_tsg(g, tsg, false);
			}

			/* put back the ref taken early above */
			if (refch) {
				gk20a_channel_put(ch);
			}
		} else if (refch != NULL) {
			nvgpu_err(g, "mmu error in unbound channel %d",
					  ch->chid);
			gk20a_channel_put(ch);
		} else if (mmfault_info.inst_ptr ==
				nvgpu_inst_block_addr(g, &g->mm.bar1.inst_block)) {
			nvgpu_err(g, "mmu fault from bar1");
		} else if (mmfault_info.inst_ptr ==
				nvgpu_inst_block_addr(g, &g->mm.pmu.inst_block)) {
			nvgpu_err(g, "mmu fault from pmu");
		} else {
			nvgpu_err(g, "couldn't locate channel for mmu fault");
		}
	}

	/* clear interrupt */
	gk20a_writel(g, fifo_intr_mmu_fault_id_r(), fault_id);

	/* resume scheduler */
	gk20a_writel(g, fifo_error_sched_disable_r(),
		     gk20a_readl(g, fifo_error_sched_disable_r()));

	/* Re-enable fifo access */
	gk20a_writel(g, gr_gpfifo_ctl_r(),
		     gr_gpfifo_ctl_access_enabled_f() |
		     gr_gpfifo_ctl_semaphore_access_enabled_f());

	/* It is safe to enable ELPG again. */
	if (g->support_pmu) {
		if (nvgpu_cg_pg_enable(g) != 0) {
			nvgpu_warn(g, "fail to enable power mgmt");
		}
	}

	return verbose;
}

static bool gk20a_fifo_handle_mmu_fault(
	struct gk20a *g,
	u32 mmu_fault_engines, /* queried from HW if 0 */
	u32 hw_id, /* queried from HW if ~(u32)0 OR mmu_fault_engines == 0*/
	bool id_is_tsg)
{
	u32 rlid;
	bool verbose;

	nvgpu_log_fn(g, " ");

	nvgpu_log_info(g, "acquire engines_reset_mutex");
	nvgpu_mutex_acquire(&g->fifo.engines_reset_mutex);

	nvgpu_log_info(g, "acquire runlist_lock for all runlists");
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		nvgpu_mutex_acquire(&g->fifo.runlist_info[rlid].runlist_lock);
	}

	verbose = gk20a_fifo_handle_mmu_fault_locked(g, mmu_fault_engines,
			hw_id, id_is_tsg);

	nvgpu_log_info(g, "release runlist_lock for all runlists");
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		nvgpu_mutex_release(&g->fifo.runlist_info[rlid].runlist_lock);
	}

	nvgpu_log_info(g, "release engines_reset_mutex");
	nvgpu_mutex_release(&g->fifo.engines_reset_mutex);

	return verbose;
}

static void gk20a_fifo_get_faulty_id_type(struct gk20a *g, int engine_id,
					  u32 *id, u32 *type)
{
	u32 status = gk20a_readl(g, fifo_engine_status_r(engine_id));
	u32 ctx_status = fifo_engine_status_ctx_status_v(status);

	/* use next_id if context load is failing */
	*id = (ctx_status ==
		fifo_engine_status_ctx_status_ctxsw_load_v()) ?
		fifo_engine_status_next_id_v(status) :
		fifo_engine_status_id_v(status);

	*type = (ctx_status ==
		fifo_engine_status_ctx_status_ctxsw_load_v()) ?
		fifo_engine_status_next_id_type_v(status) :
		fifo_engine_status_id_type_v(status);
}

static u32 gk20a_fifo_engines_on_id(struct gk20a *g, u32 id, bool is_tsg)
{
	unsigned int i;
	u32 engines = 0;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		u32 status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		u32 ctx_status =
			fifo_engine_status_ctx_status_v(status);
		u32 ctx_id = (ctx_status ==
			fifo_engine_status_ctx_status_ctxsw_load_v()) ?
			fifo_engine_status_next_id_v(status) :
			fifo_engine_status_id_v(status);
		u32 type = (ctx_status ==
			fifo_engine_status_ctx_status_ctxsw_load_v()) ?
			fifo_engine_status_next_id_type_v(status) :
			fifo_engine_status_id_type_v(status);
		bool busy = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();
		if (busy && ctx_id == id) {
			if ((is_tsg && type ==
					fifo_engine_status_id_type_tsgid_v()) ||
				    (!is_tsg && type ==
					fifo_engine_status_id_type_chid_v())) {
				engines |= BIT(active_engine_id);
			}
		}
	}

	return engines;
}

void gk20a_fifo_recover_ch(struct gk20a *g, struct channel_gk20a *ch,
	bool verbose, u32 rc_type)
{
	u32 engines;

	/* stop context switching to prevent engine assignments from
	   changing until channel is recovered */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	gr_gk20a_disable_ctxsw(g);

	engines = gk20a_fifo_engines_on_id(g, ch->chid, false);

	if (engines) {
		gk20a_fifo_recover(g, engines, ch->chid, false, true, verbose,
					rc_type);
	} else {
		gk20a_channel_abort(ch, false);

		if (gk20a_fifo_error_ch(g, ch)) {
			gk20a_debug_dump(g);
		}
	}

	gr_gk20a_enable_ctxsw(g);
	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

void gk20a_fifo_recover_tsg(struct gk20a *g, struct tsg_gk20a *tsg,
			 bool verbose, u32 rc_type)
{
	u32 engines = 0U;
	int err;

	/* stop context switching to prevent engine assignments from
	   changing until TSG is recovered */
	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	/* disable tsg so that it does not get scheduled again */
	g->ops.fifo.disable_tsg(tsg);

	/*
	 * On hitting engine reset, h/w drops the ctxsw_status to INVALID in
	 * fifo_engine_status register. Also while the engine is held in reset
	 * h/w passes busy/idle straight through. fifo_engine_status registers
	 * are correct in that there is no context switch outstanding
	 * as the CTXSW is aborted when reset is asserted.
	*/
	nvgpu_log_info(g, "acquire engines_reset_mutex");
	nvgpu_mutex_acquire(&g->fifo.engines_reset_mutex);

	/*
	 * stop context switching to prevent engine assignments from
	 * changing until engine status is checked to make sure tsg
	 * being recovered is not loaded on the engines
	 */
	err = gr_gk20a_disable_ctxsw(g);

	if (err != 0) {
		/* if failed to disable ctxsw, just abort tsg */
		nvgpu_err(g, "failed to disable ctxsw");
	} else {
		/* recover engines if tsg is loaded on the engines */
		engines = gk20a_fifo_engines_on_id(g, tsg->tsgid, true);

		/*
		 * it is ok to enable ctxsw before tsg is recovered. If engines
		 * is 0, no engine recovery is needed and if it is  non zero,
		 * gk20a_fifo_recover will call get_engines_mask_on_id again.
		 * By that time if tsg is not on the engine, engine need not
		 * be reset.
		 */
		err = gr_gk20a_enable_ctxsw(g);
		if (err != 0) {
			nvgpu_err(g, "failed to enable ctxsw");
		}
	}

	nvgpu_log_info(g, "release engines_reset_mutex");
	nvgpu_mutex_release(&g->fifo.engines_reset_mutex);

	if (engines) {
		gk20a_fifo_recover(g, engines, tsg->tsgid, true, true, verbose,
					rc_type);
	} else {
		if (gk20a_fifo_error_tsg(g, tsg) && verbose) {
			gk20a_debug_dump(g);
		}

		gk20a_fifo_abort_tsg(g, tsg, false);
	}

	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

void gk20a_fifo_teardown_mask_intr(struct gk20a *g)
{
	u32 val;

	val = gk20a_readl(g, fifo_intr_en_0_r());
	val &= ~(fifo_intr_en_0_sched_error_m() |
		fifo_intr_en_0_mmu_fault_m());
	gk20a_writel(g, fifo_intr_en_0_r(), val);
	gk20a_writel(g, fifo_intr_0_r(), fifo_intr_0_sched_error_reset_f());
}

void gk20a_fifo_teardown_unmask_intr(struct gk20a *g)
{
	u32 val;

	val = gk20a_readl(g, fifo_intr_en_0_r());
	val |= fifo_intr_en_0_mmu_fault_f(1) | fifo_intr_en_0_sched_error_f(1);
	gk20a_writel(g, fifo_intr_en_0_r(), val);

}

void gk20a_fifo_teardown_ch_tsg(struct gk20a *g, u32 __engine_ids,
			u32 hw_id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault)
{
	unsigned long engine_id, i;
	unsigned long _engine_ids = __engine_ids;
	unsigned long engine_ids = 0;
	u32 mmu_fault_engines = 0;
	u32 ref_type;
	u32 ref_id;
	u32 ref_id_is_tsg = false;
	bool id_is_known = (id_type != ID_TYPE_UNKNOWN) ? true : false;
	bool id_is_tsg = (id_type == ID_TYPE_TSG) ? true : false;
	u32 rlid;

	nvgpu_log_info(g, "acquire engines_reset_mutex");
	nvgpu_mutex_acquire(&g->fifo.engines_reset_mutex);

	nvgpu_log_info(g, "acquire runlist_lock for all runlists");
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		nvgpu_mutex_acquire(&g->fifo.runlist_info[rlid].runlist_lock);
	}

	if (id_is_known) {
		engine_ids = gk20a_fifo_engines_on_id(g, hw_id, id_is_tsg);
		ref_id = hw_id;
		ref_type = id_is_tsg ?
			fifo_engine_status_id_type_tsgid_v() :
			fifo_engine_status_id_type_chid_v();
		ref_id_is_tsg = id_is_tsg;
		/* atleast one engine will get passed during sched err*/
		engine_ids |= __engine_ids;
		for_each_set_bit(engine_id, &engine_ids, 32) {
			u32 mmu_id = gk20a_engine_id_to_mmu_id(g, engine_id);

			if (mmu_id != FIFO_INVAL_ENGINE_ID) {
				mmu_fault_engines |= BIT(mmu_id);
			}
		}
	} else {
		/* store faulted engines in advance */
		for_each_set_bit(engine_id, &_engine_ids, 32) {
			gk20a_fifo_get_faulty_id_type(g, engine_id, &ref_id,
						      &ref_type);
			if (ref_type == fifo_engine_status_id_type_tsgid_v()) {
				ref_id_is_tsg = true;
			} else {
				ref_id_is_tsg = false;
			}
			/* Reset *all* engines that use the
			 * same channel as faulty engine */
			for (i = 0; i < g->fifo.num_engines; i++) {
				u32 active_engine_id = g->fifo.active_engines_list[i];
				u32 type;
				u32 id;

				gk20a_fifo_get_faulty_id_type(g, active_engine_id, &id, &type);
				if (ref_type == type && ref_id == id) {
					u32 mmu_id = gk20a_engine_id_to_mmu_id(g, active_engine_id);

					engine_ids |= BIT(active_engine_id);
					if (mmu_id != FIFO_INVAL_ENGINE_ID) {
						mmu_fault_engines |= BIT(mmu_id);
					}
				}
			}
		}
	}

	if (mmu_fault_engines) {
		g->ops.fifo.teardown_mask_intr(g);
		g->ops.fifo.trigger_mmu_fault(g, engine_ids);
		gk20a_fifo_handle_mmu_fault_locked(g, mmu_fault_engines, ref_id,
				ref_id_is_tsg);

		g->ops.fifo.teardown_unmask_intr(g);
	}

	nvgpu_log_info(g, "release runlist_lock for all runlists");
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		nvgpu_mutex_release(&g->fifo.runlist_info[rlid].runlist_lock);
	}

	nvgpu_log_info(g, "release engines_reset_mutex");
	nvgpu_mutex_release(&g->fifo.engines_reset_mutex);
}

void gk20a_fifo_recover(struct gk20a *g, u32 __engine_ids,
			u32 hw_id, bool id_is_tsg,
			bool id_is_known, bool verbose, int rc_type)
{
	unsigned int id_type;

	if (verbose) {
		gk20a_debug_dump(g);
	}

	if (g->ops.ltc.flush) {
		g->ops.ltc.flush(g);
	}

	if (id_is_known) {
		id_type = id_is_tsg ? ID_TYPE_TSG : ID_TYPE_CHANNEL;
	} else {
		id_type = ID_TYPE_UNKNOWN;
	}

	g->ops.fifo.teardown_ch_tsg(g, __engine_ids, hw_id, id_type,
					 rc_type, NULL);
}

/* force reset channel and tsg */
int gk20a_fifo_force_reset_ch(struct channel_gk20a *ch,
				u32 err_code, bool verbose)
{
	struct channel_gk20a *ch_tsg = NULL;
	struct gk20a *g = ch->g;

	struct tsg_gk20a *tsg = tsg_gk20a_from_ch(ch);

	if (tsg != NULL) {
		nvgpu_rwsem_down_read(&tsg->ch_list_lock);

		nvgpu_list_for_each_entry(ch_tsg, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch_tsg)) {
				g->ops.fifo.set_error_notifier(ch_tsg,
								err_code);
				gk20a_channel_put(ch_tsg);
			}
		}

		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
		gk20a_fifo_recover_tsg(g, tsg, verbose,
				RC_TYPE_FORCE_RESET);
	} else {
		nvgpu_err(g, "chid: %d is not bound to tsg", ch->chid);
	}

	return 0;
}

int gk20a_fifo_tsg_unbind_channel_verify_status(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;

	if (gk20a_fifo_channel_status_is_next(g, ch->chid)) {
		nvgpu_err(g, "Channel %d to be removed from TSG %d has NEXT set!",
			ch->chid, ch->tsgid);
		return -EINVAL;
	}

	if (g->ops.fifo.tsg_verify_status_ctx_reload) {
		g->ops.fifo.tsg_verify_status_ctx_reload(ch);
	}

	if (g->ops.fifo.tsg_verify_status_faulted) {
		g->ops.fifo.tsg_verify_status_faulted(ch);
	}

	return 0;
}

int gk20a_fifo_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
        struct tsg_gk20a *tsg = tsg_gk20a_from_ch(ch);
	int err;
	bool tsg_timedout = false;

	if (tsg == NULL) {
		nvgpu_err(g, "chid: %d is not bound to tsg", ch->chid);
		return 0;
	}

	/* If one channel in TSG times out, we disable all channels */
	nvgpu_rwsem_down_write(&tsg->ch_list_lock);
	tsg_timedout = gk20a_channel_check_timedout(ch);
	nvgpu_rwsem_up_write(&tsg->ch_list_lock);

	/* Disable TSG and examine status before unbinding channel */
	g->ops.fifo.disable_tsg(tsg);

	err = g->ops.fifo.preempt_tsg(g, tsg);
	if (err != 0) {
		goto fail_enable_tsg;
	}

	if (g->ops.fifo.tsg_verify_channel_status && !tsg_timedout) {
		err = g->ops.fifo.tsg_verify_channel_status(ch);
		if (err) {
			goto fail_enable_tsg;
		}
	}

	/* Channel should be seen as TSG channel while updating runlist */
	err = channel_gk20a_update_runlist(ch, false);
	if (err) {
		goto fail_enable_tsg;
	}

	/* Remove channel from TSG and re-enable rest of the channels */
	nvgpu_rwsem_down_write(&tsg->ch_list_lock);
	nvgpu_list_del(&ch->ch_entry);
	ch->tsgid = NVGPU_INVALID_TSG_ID;

	/* another thread could have re-enabled the channel because it was
	 * still on the list at that time, so make sure it's truly disabled
	 */
	g->ops.fifo.disable_channel(ch);
	nvgpu_rwsem_up_write(&tsg->ch_list_lock);

	/*
	 * Don't re-enable all channels if TSG has timed out already
	 *
	 * Note that we can skip disabling and preempting TSG too in case of
	 * time out, but we keep that to ensure TSG is kicked out
	 */
	if (!tsg_timedout) {
		g->ops.fifo.enable_tsg(tsg);
	}

	if (ch->g->ops.fifo.ch_abort_clean_up) {
		ch->g->ops.fifo.ch_abort_clean_up(ch);
	}

	return 0;

fail_enable_tsg:
	if (!tsg_timedout) {
		g->ops.fifo.enable_tsg(tsg);
	}
	return err;
}

u32 gk20a_fifo_get_failing_engine_data(struct gk20a *g,
			int *__id, bool *__is_tsg)
{
	u32 engine_id;
	int id = -1;
	bool is_tsg = false;
	u32 mailbox2;
	u32 active_engine_id = FIFO_INVAL_ENGINE_ID;

	for (engine_id = 0; engine_id < g->fifo.num_engines; engine_id++) {
		u32 status;
		u32 ctx_status;
		bool failing_engine;

		active_engine_id = g->fifo.active_engines_list[engine_id];
		status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		ctx_status = fifo_engine_status_ctx_status_v(status);

		/* we are interested in busy engines */
		failing_engine = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();

		/* ..that are doing context switch */
		failing_engine = failing_engine &&
			(ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_switch_v()
			|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_save_v()
			|| ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v());

		if (!failing_engine) {
		    active_engine_id = FIFO_INVAL_ENGINE_ID;
			continue;
		}

		if (ctx_status ==
				fifo_engine_status_ctx_status_ctxsw_load_v()) {
			id = fifo_engine_status_next_id_v(status);
			is_tsg = fifo_engine_status_next_id_type_v(status) !=
				fifo_engine_status_next_id_type_chid_v();
		} else if (ctx_status ==
			       fifo_engine_status_ctx_status_ctxsw_switch_v()) {
			mailbox2 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(2));
			if (mailbox2 & FECS_METHOD_WFI_RESTORE) {
				id = fifo_engine_status_next_id_v(status);
				is_tsg = fifo_engine_status_next_id_type_v(status) !=
					fifo_engine_status_next_id_type_chid_v();
			} else {
				id = fifo_engine_status_id_v(status);
				is_tsg = fifo_engine_status_id_type_v(status) !=
					fifo_engine_status_id_type_chid_v();
			}
		} else {
			id = fifo_engine_status_id_v(status);
			is_tsg = fifo_engine_status_id_type_v(status) !=
				fifo_engine_status_id_type_chid_v();
		}
		break;
	}

	*__id = id;
	*__is_tsg = is_tsg;

	return active_engine_id;
}

bool gk20a_fifo_check_ch_ctxsw_timeout(struct channel_gk20a *ch,
		bool *verbose, u32 *ms)
{
	bool recover = false;
	bool progress = false;
	struct gk20a *g = ch->g;

	if (gk20a_channel_get(ch)) {
		recover = gk20a_channel_update_and_check_timeout(ch,
				g->fifo_eng_timeout_us / 1000,
				&progress);
		*verbose = ch->timeout_debug_dump;
		*ms = ch->timeout_accumulated_ms;
		if (recover) {
			g->ops.fifo.set_error_notifier(ch,
					NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT);
		}

		gk20a_channel_put(ch);
	}
	return recover;
}

bool gk20a_fifo_check_tsg_ctxsw_timeout(struct tsg_gk20a *tsg,
		bool *verbose, u32 *ms)
{
	struct channel_gk20a *ch;
	bool recover = false;
	bool progress = false;
	struct gk20a *g = tsg->g;

	*verbose = false;
	*ms = g->fifo_eng_timeout_us / 1000;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);

	/* check if there was some progress on any of the TSG channels.
	 * fifo recovery is needed if at least one channel reached the
	 * maximum timeout without progress (update in gpfifo pointers).
	 */
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		if (gk20a_channel_get(ch)) {
			recover = gk20a_channel_update_and_check_timeout(ch,
					*ms, &progress);
			if (progress || recover) {
				break;
			}
			gk20a_channel_put(ch);
		}
	}

	if (recover) {
		/*
		 * if one channel is presumed dead (no progress for too long),
		 * then fifo recovery is needed. we can't really figure out
		 * which channel caused the problem, so set timeout error
		 * notifier for all channels.
		 */
		nvgpu_log_info(g, "timeout on tsg=%d ch=%d",
				tsg->tsgid, ch->chid);
		*ms = ch->timeout_accumulated_ms;
		gk20a_channel_put(ch);
		nvgpu_list_for_each_entry(ch, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch)) {
				ch->g->ops.fifo.set_error_notifier(ch,
					NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT);
				if (ch->timeout_debug_dump) {
					*verbose = true;
				}
				gk20a_channel_put(ch);
			}
		}
	} else if (progress) {
		/*
		 * if at least one channel in the TSG made some progress, reset
		 * accumulated timeout for all channels in the TSG. In
		 * particular, this resets timeout for channels that already
		 * completed their work
		 */
		nvgpu_log_info(g, "progress on tsg=%d ch=%d",
				tsg->tsgid, ch->chid);
		gk20a_channel_put(ch);
		*ms = g->fifo_eng_timeout_us / 1000;
		nvgpu_list_for_each_entry(ch, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch)) {
				ch->timeout_accumulated_ms = *ms;
				gk20a_channel_put(ch);
			}
		}
	}

	/* if we could not detect progress on any of the channel, but none
	 * of them has reached the timeout, there is nothing more to do:
	 * timeout_accumulated_ms has been updated for all of them.
	 */
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	return recover;
}

bool gk20a_fifo_handle_sched_error(struct gk20a *g)
{
	u32 sched_error;
	u32 engine_id;
	int id = -1;
	bool is_tsg = false;
	bool ret = false;

	/* read the scheduler error register */
	sched_error = gk20a_readl(g, fifo_intr_sched_error_r());

	engine_id = gk20a_fifo_get_failing_engine_data(g, &id, &is_tsg);
	/*
	 * Could not find the engine
	 * Possible Causes:
	 * a)
	 * On hitting engine reset, h/w drops the ctxsw_status to INVALID in
	 * fifo_engine_status register. Also while the engine is held in reset
	 * h/w passes busy/idle straight through. fifo_engine_status registers
	 * are correct in that there is no context switch outstanding
	 * as the CTXSW is aborted when reset is asserted.
	 * This is just a side effect of how gv100 and earlier versions of
	 * ctxsw_timeout behave.
	 * With gv11b and later, h/w snaps the context at the point of error
	 * so that s/w can see the tsg_id which caused the HW timeout.
	 * b)
	 * If engines are not busy and ctxsw state is valid then intr occurred
	 * in the past and if the ctxsw state has moved on to VALID from LOAD
	 * or SAVE, it means that whatever timed out eventually finished
	 * anyways. The problem with this is that s/w cannot conclude which
	 * context caused the problem as maybe more switches occurred before
	 * intr is handled.
	 */
	if (engine_id == FIFO_INVAL_ENGINE_ID) {
		nvgpu_info(g, "fifo sched error: 0x%08x, failed to find engine "
				"that is busy doing ctxsw. "
				"May be ctxsw already happened", sched_error);
		ret = false;
		goto err;
	}

	/* could not find the engine - should never happen */
	if (!gk20a_fifo_is_valid_engine_id(g, engine_id)) {
		nvgpu_err(g, "fifo sched error : 0x%08x, failed to find engine",
			sched_error);
		ret = false;
		goto err;
	}

	if (fifo_intr_sched_error_code_f(sched_error) ==
			fifo_intr_sched_error_code_ctxsw_timeout_v()) {
		struct fifo_gk20a *f = &g->fifo;
		u32 ms = 0;
		bool verbose = false;

		if (is_tsg) {
			ret = g->ops.fifo.check_tsg_ctxsw_timeout(
					&f->tsg[id], &verbose, &ms);
		} else {
			ret = g->ops.fifo.check_ch_ctxsw_timeout(
					&f->channel[id], &verbose, &ms);
		}

		if (ret) {
			nvgpu_err(g,
				"fifo sched ctxsw timeout error: "
				"engine=%u, %s=%d, ms=%u",
				engine_id, is_tsg ? "tsg" : "ch", id, ms);
			/*
			 * Cancel all channels' timeout since SCHED error might
			 * trigger multiple watchdogs at a time
			 */
			gk20a_channel_timeout_restart_all_channels(g);
			gk20a_fifo_recover(g, BIT(engine_id), id,
					is_tsg, true, verbose,
					RC_TYPE_CTXSW_TIMEOUT);
		} else {
			nvgpu_log_info(g,
				"fifo is waiting for ctx switch for %d ms, "
				"%s=%d", ms, is_tsg ? "tsg" : "ch", id);
		}
	} else {
		nvgpu_err(g,
			"fifo sched error : 0x%08x, engine=%u, %s=%d",
			sched_error, engine_id, is_tsg ? "tsg" : "ch", id);
	}

err:
	return ret;
}

static u32 fifo_error_isr(struct gk20a *g, u32 fifo_intr)
{
	bool print_channel_reset_log = false;
	u32 handled = 0;

	nvgpu_log_fn(g, "fifo_intr=0x%08x", fifo_intr);

	if (fifo_intr & fifo_intr_0_pio_error_pending_f()) {
		/* pio mode is unused.  this shouldn't happen, ever. */
		/* should we clear it or just leave it pending? */
		nvgpu_err(g, "fifo pio error!");
		BUG_ON(1);
	}

	if (fifo_intr & fifo_intr_0_bind_error_pending_f()) {
		u32 bind_error = gk20a_readl(g, fifo_intr_bind_error_r());
		nvgpu_err(g, "fifo bind error: 0x%08x", bind_error);
		print_channel_reset_log = true;
		handled |= fifo_intr_0_bind_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_sched_error_pending_f()) {
		print_channel_reset_log = g->ops.fifo.handle_sched_error(g);
		handled |= fifo_intr_0_sched_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_chsw_error_pending_f()) {
		gk20a_fifo_handle_chsw_fault(g);
		handled |= fifo_intr_0_chsw_error_pending_f();
	}

	if (fifo_intr & fifo_intr_0_mmu_fault_pending_f()) {
		if (gk20a_fifo_handle_mmu_fault(g, 0, ~(u32)0, false)) {
			print_channel_reset_log = true;
		}
		handled |= fifo_intr_0_mmu_fault_pending_f();
	}

	if (fifo_intr & fifo_intr_0_dropped_mmu_fault_pending_f()) {
		gk20a_fifo_handle_dropped_mmu_fault(g);
		handled |= fifo_intr_0_dropped_mmu_fault_pending_f();
	}

	print_channel_reset_log = !g->fifo.deferred_reset_pending
			&& print_channel_reset_log;

	if (print_channel_reset_log) {
		unsigned int engine_id;
		nvgpu_err(g,
			   "channel reset initiated from %s; intr=0x%08x",
			   __func__, fifo_intr);
		for (engine_id = 0;
		     engine_id < g->fifo.num_engines;
		     engine_id++) {
				u32 active_engine_id = g->fifo.active_engines_list[engine_id];
				u32 engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
				nvgpu_log_fn(g, "enum:%d -> engine_id:%d", engine_enum,
					active_engine_id);
				fifo_pbdma_exception_status(g,
						&g->fifo.engine_info[active_engine_id]);
				fifo_engine_exception_status(g,
						&g->fifo.engine_info[active_engine_id]);
		}
	}

	return handled;
}

static inline void gk20a_fifo_reset_pbdma_header(struct gk20a *g, int pbdma_id)
{
	gk20a_writel(g, pbdma_pb_header_r(pbdma_id),
			pbdma_pb_header_first_true_f() |
			pbdma_pb_header_type_non_inc_f());
}

void gk20a_fifo_reset_pbdma_method(struct gk20a *g, int pbdma_id,
						int pbdma_method_index)
{
	u32 pbdma_method_stride;
	u32 pbdma_method_reg;

	pbdma_method_stride = pbdma_method1_r(pbdma_id) -
				pbdma_method0_r(pbdma_id);

	pbdma_method_reg = pbdma_method0_r(pbdma_id) +
		(pbdma_method_index * pbdma_method_stride);

	gk20a_writel(g, pbdma_method_reg,
			pbdma_method0_valid_true_f() |
			pbdma_method0_first_true_f() |
			pbdma_method0_addr_f(
			     pbdma_udma_nop_r() >> 2));
}

static bool gk20a_fifo_is_sw_method_subch(struct gk20a *g, int pbdma_id,
						int pbdma_method_index)
{
	u32 pbdma_method_stride;
	u32 pbdma_method_reg, pbdma_method_subch;

	pbdma_method_stride = pbdma_method1_r(pbdma_id) -
				pbdma_method0_r(pbdma_id);

	pbdma_method_reg = pbdma_method0_r(pbdma_id) +
			(pbdma_method_index * pbdma_method_stride);

	pbdma_method_subch = pbdma_method0_subch_v(
			gk20a_readl(g, pbdma_method_reg));

	if (pbdma_method_subch == 5 ||
	    pbdma_method_subch == 6 ||
	    pbdma_method_subch == 7) {
		return true;
	}

	return false;
}

unsigned int gk20a_fifo_handle_pbdma_intr_0(struct gk20a *g, u32 pbdma_id,
			u32 pbdma_intr_0, u32 *handled, u32 *error_notifier)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned int rc_type = RC_TYPE_NO_RC;
	int i;
	unsigned long pbdma_intr_err;
	u32 bit;

	if ((f->intr.pbdma.device_fatal_0 |
	     f->intr.pbdma.channel_fatal_0 |
	     f->intr.pbdma.restartable_0) & pbdma_intr_0) {

		pbdma_intr_err = (unsigned long)pbdma_intr_0;
		for_each_set_bit(bit, &pbdma_intr_err, 32) {
			nvgpu_err(g, "PBDMA intr %s Error",
				pbdma_intr_fault_type_desc[bit]);
		}

		nvgpu_err(g,
			"pbdma_intr_0(%d):0x%08x PBH: %08x "
			"SHADOW: %08x gp shadow0: %08x gp shadow1: %08x"
			"M0: %08x %08x %08x %08x ",
			pbdma_id, pbdma_intr_0,
			gk20a_readl(g, pbdma_pb_header_r(pbdma_id)),
			gk20a_readl(g, pbdma_hdr_shadow_r(pbdma_id)),
			gk20a_readl(g, pbdma_gp_shadow_0_r(pbdma_id)),
			gk20a_readl(g, pbdma_gp_shadow_1_r(pbdma_id)),
			gk20a_readl(g, pbdma_method0_r(pbdma_id)),
			gk20a_readl(g, pbdma_method1_r(pbdma_id)),
			gk20a_readl(g, pbdma_method2_r(pbdma_id)),
			gk20a_readl(g, pbdma_method3_r(pbdma_id))
			);

		rc_type = RC_TYPE_PBDMA_FAULT;
		*handled |= ((f->intr.pbdma.device_fatal_0 |
			     f->intr.pbdma.channel_fatal_0 |
			     f->intr.pbdma.restartable_0) &
			    pbdma_intr_0);
	}

	if (pbdma_intr_0 & pbdma_intr_0_acquire_pending_f()) {
		u32 val = gk20a_readl(g, pbdma_acquire_r(pbdma_id));

		val &= ~pbdma_acquire_timeout_en_enable_f();
		gk20a_writel(g, pbdma_acquire_r(pbdma_id), val);
		if (nvgpu_is_timeouts_enabled(g)) {
			rc_type = RC_TYPE_PBDMA_FAULT;
			nvgpu_err(g,
				"semaphore acquire timeout!");
			*error_notifier = NVGPU_ERR_NOTIFIER_GR_SEMAPHORE_TIMEOUT;
		}
		*handled |= pbdma_intr_0_acquire_pending_f();
	}

	if (pbdma_intr_0 & pbdma_intr_0_pbentry_pending_f()) {
		gk20a_fifo_reset_pbdma_header(g, pbdma_id);
		gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	if (pbdma_intr_0 & pbdma_intr_0_method_pending_f()) {
		gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	if (pbdma_intr_0 & pbdma_intr_0_pbcrc_pending_f()) {
		*error_notifier =
			NVGPU_ERR_NOTIFIER_PBDMA_PUSHBUFFER_CRC_MISMATCH;
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	if (pbdma_intr_0 & pbdma_intr_0_device_pending_f()) {
		gk20a_fifo_reset_pbdma_header(g, pbdma_id);

		for (i = 0; i < 4; i++) {
			if (gk20a_fifo_is_sw_method_subch(g,
					pbdma_id, i)) {
				gk20a_fifo_reset_pbdma_method(g,
						pbdma_id, i);
			}
		}
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	return rc_type;
}

unsigned int gk20a_fifo_handle_pbdma_intr_1(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_1,
			u32 *handled, u32 *error_notifier)
{
	unsigned int rc_type = RC_TYPE_PBDMA_FAULT;

	/*
	 * all of the interrupts in _intr_1 are "host copy engine"
	 * related, which is not supported. For now just make them
	 * channel fatal.
	 */
	nvgpu_err(g, "hce err: pbdma_intr_1(%d):0x%08x",
		pbdma_id, pbdma_intr_1);
	*handled |= pbdma_intr_1;

	return rc_type;
}

static void gk20a_fifo_pbdma_fault_rc(struct gk20a *g,
			struct fifo_gk20a *f, u32 pbdma_id,
			u32 error_notifier)
{
	u32 status;
	u32 id;

	nvgpu_log(g, gpu_dbg_info, "pbdma id %d error notifier %d",
			pbdma_id, error_notifier);
	status = gk20a_readl(g, fifo_pbdma_status_r(pbdma_id));
	/* Remove channel from runlist */
	id = fifo_pbdma_status_id_v(status);
	if (fifo_pbdma_status_id_type_v(status)
			== fifo_pbdma_status_id_type_chid_v()) {
		struct channel_gk20a *ch = gk20a_channel_from_id(g, id);

		if (ch != NULL) {
			g->ops.fifo.set_error_notifier(ch, error_notifier);
			gk20a_fifo_recover_ch(g, ch, true, RC_TYPE_PBDMA_FAULT);
			gk20a_channel_put(ch);
		}
	} else if (fifo_pbdma_status_id_type_v(status)
			== fifo_pbdma_status_id_type_tsgid_v()) {
		struct tsg_gk20a *tsg = &f->tsg[id];
		struct channel_gk20a *ch = NULL;

		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		nvgpu_list_for_each_entry(ch, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (gk20a_channel_get(ch)) {
				g->ops.fifo.set_error_notifier(ch,
					error_notifier);
				gk20a_channel_put(ch);
			}
		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
		gk20a_fifo_recover_tsg(g, tsg, true, RC_TYPE_PBDMA_FAULT);
	}
}

u32 gk20a_fifo_handle_pbdma_intr(struct gk20a *g, struct fifo_gk20a *f,
			u32 pbdma_id, unsigned int rc)
{
	u32 pbdma_intr_0 = gk20a_readl(g, pbdma_intr_0_r(pbdma_id));
	u32 pbdma_intr_1 = gk20a_readl(g, pbdma_intr_1_r(pbdma_id));

	u32 handled = 0;
	u32 error_notifier = NVGPU_ERR_NOTIFIER_PBDMA_ERROR;
	unsigned int rc_type = RC_TYPE_NO_RC;

	if (pbdma_intr_0) {
		nvgpu_log(g, gpu_dbg_info | gpu_dbg_intr,
			"pbdma id %d intr_0 0x%08x pending",
			pbdma_id, pbdma_intr_0);

		if (g->ops.fifo.handle_pbdma_intr_0(g, pbdma_id, pbdma_intr_0,
			&handled, &error_notifier) != RC_TYPE_NO_RC) {
			rc_type = RC_TYPE_PBDMA_FAULT;
		}
		gk20a_writel(g, pbdma_intr_0_r(pbdma_id), pbdma_intr_0);
	}

	if (pbdma_intr_1) {
		nvgpu_log(g, gpu_dbg_info | gpu_dbg_intr,
			"pbdma id %d intr_1 0x%08x pending",
			pbdma_id, pbdma_intr_1);

		if (g->ops.fifo.handle_pbdma_intr_1(g, pbdma_id, pbdma_intr_1,
			&handled, &error_notifier) != RC_TYPE_NO_RC) {
			rc_type = RC_TYPE_PBDMA_FAULT;
		}
		gk20a_writel(g, pbdma_intr_1_r(pbdma_id), pbdma_intr_1);
	}

	if (rc == RC_YES && rc_type == RC_TYPE_PBDMA_FAULT) {
		gk20a_fifo_pbdma_fault_rc(g, f, pbdma_id, error_notifier);
	}

	return handled;
}

static u32 fifo_pbdma_isr(struct gk20a *g, u32 fifo_intr)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 clear_intr = 0, i;
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);
	u32 pbdma_pending = gk20a_readl(g, fifo_intr_pbdma_id_r());

	for (i = 0; i < host_num_pbdma; i++) {
		if (fifo_intr_pbdma_id_status_v(pbdma_pending, i)) {
			nvgpu_log(g, gpu_dbg_intr, "pbdma id %d intr pending", i);
			clear_intr |=
				gk20a_fifo_handle_pbdma_intr(g, f, i, RC_YES);
		}
	}
	return fifo_intr_0_pbdma_intr_pending_f();
}

void gk20a_fifo_isr(struct gk20a *g)
{
	u32 error_intr_mask;
	u32 clear_intr = 0;
	u32 fifo_intr = gk20a_readl(g, fifo_intr_0_r());

	error_intr_mask = g->ops.fifo.intr_0_error_mask(g);

	if (g->fifo.sw_ready) {
		/* note we're not actually in an "isr", but rather
		 * in a threaded interrupt context... */
		nvgpu_mutex_acquire(&g->fifo.intr.isr.mutex);

		nvgpu_log(g, gpu_dbg_intr, "fifo isr %08x\n", fifo_intr);

		/* handle runlist update */
		if (fifo_intr & fifo_intr_0_runlist_event_pending_f()) {
			gk20a_fifo_handle_runlist_event(g);
			clear_intr |= fifo_intr_0_runlist_event_pending_f();
		}
		if (fifo_intr & fifo_intr_0_pbdma_intr_pending_f()) {
			clear_intr |= fifo_pbdma_isr(g, fifo_intr);
		}

		if (g->ops.fifo.handle_ctxsw_timeout) {
			g->ops.fifo.handle_ctxsw_timeout(g, fifo_intr);
		}

		if (unlikely((fifo_intr & error_intr_mask) != 0U)) {
			clear_intr |= fifo_error_isr(g, fifo_intr);
		}

		nvgpu_mutex_release(&g->fifo.intr.isr.mutex);
	}
	gk20a_writel(g, fifo_intr_0_r(), clear_intr);

	return;
}

u32 gk20a_fifo_nonstall_isr(struct gk20a *g)
{
	u32 fifo_intr = gk20a_readl(g, fifo_intr_0_r());
	u32 clear_intr = 0;

	nvgpu_log(g, gpu_dbg_intr, "fifo nonstall isr %08x\n", fifo_intr);

	if (fifo_intr & fifo_intr_0_channel_intr_pending_f()) {
		clear_intr = fifo_intr_0_channel_intr_pending_f();
	}

	gk20a_writel(g, fifo_intr_0_r(), clear_intr);

	return GK20A_NONSTALL_OPS_WAKEUP_SEMAPHORE;
}

void gk20a_fifo_issue_preempt(struct gk20a *g, u32 id, bool is_tsg)
{
	if (is_tsg) {
		gk20a_writel(g, fifo_preempt_r(),
			fifo_preempt_id_f(id) |
			fifo_preempt_type_tsg_f());
	} else {
		gk20a_writel(g, fifo_preempt_r(),
			fifo_preempt_chid_f(id) |
			fifo_preempt_type_channel_f());
	}
}

static u32 gk20a_fifo_get_preempt_timeout(struct gk20a *g)
{
	/* Use fifo_eng_timeout converted to ms for preempt
	 * polling. gr_idle_timeout i.e 3000 ms is and not appropriate
	 * for polling preempt done as context switch timeout gets
	 * triggered every 100 ms and context switch recovery
	 * happens every 3000 ms */

	return g->fifo_eng_timeout_us / 1000;
}

int gk20a_fifo_is_preempt_pending(struct gk20a *g, u32 id,
		unsigned int id_type)
{
	struct nvgpu_timeout timeout;
	u32 delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -EBUSY;

	nvgpu_timeout_init(g, &timeout, gk20a_fifo_get_preempt_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);
	do {
		if (!(gk20a_readl(g, fifo_preempt_r()) &
				fifo_preempt_pending_true_f())) {
			ret = 0;
			break;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret) {
		nvgpu_err(g, "preempt timeout: id: %u id_type: %d ",
			id, id_type);
	}
	return ret;
}

void gk20a_fifo_preempt_timeout_rc_tsg(struct gk20a *g, struct tsg_gk20a *tsg)
{
	struct channel_gk20a *ch = NULL;

	nvgpu_err(g, "preempt TSG %d timeout", tsg->tsgid);

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list,
			channel_gk20a, ch_entry) {
		if (!gk20a_channel_get(ch)) {
			continue;
		}
		g->ops.fifo.set_error_notifier(ch,
			NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT);
		gk20a_channel_put(ch);
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	gk20a_fifo_recover_tsg(g, tsg, true, RC_TYPE_PREEMPT_TIMEOUT);
}

void gk20a_fifo_preempt_timeout_rc(struct gk20a *g, struct channel_gk20a *ch)
{
	nvgpu_err(g, "preempt channel %d timeout", ch->chid);

	g->ops.fifo.set_error_notifier(ch,
				NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT);
	gk20a_fifo_recover_ch(g, ch, true,
					RC_TYPE_PREEMPT_TIMEOUT);
}

int __locked_fifo_preempt(struct gk20a *g, u32 id, bool is_tsg)
{
	int ret;
	unsigned int id_type;

	nvgpu_log_fn(g, "id: %d is_tsg: %d", id, is_tsg);

	/* issue preempt */
	gk20a_fifo_issue_preempt(g, id, is_tsg);

	id_type = is_tsg ? ID_TYPE_TSG : ID_TYPE_CHANNEL;

	/* wait for preempt */
	ret = g->ops.fifo.is_preempt_pending(g, id, id_type);

	return ret;
}

int gk20a_fifo_preempt_channel(struct gk20a *g, struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 i;

	nvgpu_log_fn(g, "chid: %d", ch->chid);

	/* we have no idea which runlist we are using. lock all */
	for (i = 0; i < g->fifo.max_runlists; i++) {
		nvgpu_mutex_acquire(&f->runlist_info[i].runlist_lock);
	}

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, ch->chid, false);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}

	for (i = 0; i < g->fifo.max_runlists; i++) {
		nvgpu_mutex_release(&f->runlist_info[i].runlist_lock);
	}

	if (ret) {
		if (nvgpu_platform_is_silicon(g)) {
			nvgpu_err(g, "preempt timed out for chid: %u, "
			"ctxsw timeout will trigger recovery if needed",
			ch->chid);
		} else {
			gk20a_fifo_preempt_timeout_rc(g, ch);
		}
	}

	return ret;
}

int gk20a_fifo_preempt_tsg(struct gk20a *g, struct tsg_gk20a *tsg)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 i;

	nvgpu_log_fn(g, "tsgid: %d", tsg->tsgid);

	/* we have no idea which runlist we are using. lock all */
	for (i = 0; i < g->fifo.max_runlists; i++) {
		nvgpu_mutex_acquire(&f->runlist_info[i].runlist_lock);
	}

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, tsg->tsgid, true);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}

	for (i = 0; i < g->fifo.max_runlists; i++) {
		nvgpu_mutex_release(&f->runlist_info[i].runlist_lock);
	}

	if (ret) {
		if (nvgpu_platform_is_silicon(g)) {
			nvgpu_err(g, "preempt timed out for tsgid: %u, "
			"ctxsw timeout will trigger recovery if needed",
			tsg->tsgid);
		} else {
			gk20a_fifo_preempt_timeout_rc_tsg(g, tsg);
		}
	}

	return ret;
}

int gk20a_fifo_preempt(struct gk20a *g, struct channel_gk20a *ch)
{
	int err;
	struct tsg_gk20a *tsg = tsg_gk20a_from_ch(ch);

	if (tsg != NULL) {
		err = g->ops.fifo.preempt_tsg(ch->g, tsg);
	} else {
		err = g->ops.fifo.preempt_channel(ch->g, ch);
	}

	return err;
}

static void gk20a_fifo_sched_disable_rw(struct gk20a *g, u32 runlists_mask,
					 u32 runlist_state)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, fifo_sched_disable_r());

	if (runlist_state == RUNLIST_DISABLED) {
		reg_val |= runlists_mask;
	} else {
		reg_val &= (~runlists_mask);
	}

	gk20a_writel(g, fifo_sched_disable_r(), reg_val);

}

void gk20a_fifo_set_runlist_state(struct gk20a *g, u32 runlists_mask,
		u32 runlist_state)
{
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;

	nvgpu_log(g, gpu_dbg_info, "runlist mask = 0x%08x state = 0x%08x",
			runlists_mask, runlist_state);

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	gk20a_fifo_sched_disable_rw(g, runlists_mask, runlist_state);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}
}

void gk20a_fifo_enable_tsg_sched(struct gk20a *g, struct tsg_gk20a *tsg)
{
	gk20a_fifo_set_runlist_state(g, fifo_sched_disable_runlist_m(
					tsg->runlist_id), RUNLIST_ENABLED);

}

void gk20a_fifo_disable_tsg_sched(struct gk20a *g, struct tsg_gk20a *tsg)
{
	gk20a_fifo_set_runlist_state(g, fifo_sched_disable_runlist_m(
					tsg->runlist_id), RUNLIST_DISABLED);
}

int gk20a_fifo_enable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info)
{
	nvgpu_log(g, gpu_dbg_info, "start");

	gk20a_fifo_set_runlist_state(g, fifo_sched_disable_runlist_m(
				eng_info->runlist_id), RUNLIST_ENABLED);
	return 0;
}

int gk20a_fifo_enable_all_engine_activity(struct gk20a *g)
{
	unsigned int i;
	int err = 0, ret = 0;

	for (i = 0; i < g->fifo.num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		err = gk20a_fifo_enable_engine_activity(g,
				&g->fifo.engine_info[active_engine_id]);
		if (err) {
			nvgpu_err(g,
				"failed to enable engine %d activity", active_engine_id);
			ret = err;
		}
	}

	return ret;
}

int gk20a_fifo_disable_engine_activity(struct gk20a *g,
				struct fifo_engine_info_gk20a *eng_info,
				bool wait_for_idle)
{
	u32 gr_stat, pbdma_stat, chan_stat, eng_stat, ctx_stat;
	u32 pbdma_chid = FIFO_INVAL_CHANNEL_ID;
	u32 engine_chid = FIFO_INVAL_CHANNEL_ID;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	int mutex_ret;
	struct channel_gk20a *ch = NULL;
	int err = 0;

	nvgpu_log_fn(g, " ");

	gr_stat =
		gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	if (fifo_engine_status_engine_v(gr_stat) ==
	    fifo_engine_status_engine_busy_v() && !wait_for_idle) {
		return -EBUSY;
	}

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	gk20a_fifo_set_runlist_state(g, fifo_sched_disable_runlist_m(
				eng_info->runlist_id), RUNLIST_DISABLED);

	/* chid from pbdma status */
	pbdma_stat = gk20a_readl(g, fifo_pbdma_status_r(eng_info->pbdma_id));
	chan_stat  = fifo_pbdma_status_chan_status_v(pbdma_stat);
	if (chan_stat == fifo_pbdma_status_chan_status_valid_v() ||
	    chan_stat == fifo_pbdma_status_chan_status_chsw_save_v()) {
		pbdma_chid = fifo_pbdma_status_id_v(pbdma_stat);
	} else if (chan_stat == fifo_pbdma_status_chan_status_chsw_load_v() ||
		 chan_stat == fifo_pbdma_status_chan_status_chsw_switch_v()) {
		pbdma_chid = fifo_pbdma_status_next_id_v(pbdma_stat);
	}

	if (pbdma_chid != FIFO_INVAL_CHANNEL_ID) {
		ch = gk20a_channel_from_id(g, pbdma_chid);
		if (ch != NULL) {
			err = g->ops.fifo.preempt_channel(g, ch);
			gk20a_channel_put(ch);
		}
		if (err != 0) {
			goto clean_up;
		}
	}

	/* chid from engine status */
	eng_stat = gk20a_readl(g, fifo_engine_status_r(eng_info->engine_id));
	ctx_stat  = fifo_engine_status_ctx_status_v(eng_stat);
	if (ctx_stat == fifo_engine_status_ctx_status_valid_v() ||
	    ctx_stat == fifo_engine_status_ctx_status_ctxsw_save_v()) {
		engine_chid = fifo_engine_status_id_v(eng_stat);
	} else if (ctx_stat == fifo_engine_status_ctx_status_ctxsw_load_v() ||
		 ctx_stat == fifo_engine_status_ctx_status_ctxsw_switch_v()) {
		engine_chid = fifo_engine_status_next_id_v(eng_stat);
	}

	if (engine_chid != FIFO_INVAL_ENGINE_ID && engine_chid != pbdma_chid) {
		ch = gk20a_channel_from_id(g, engine_chid);
		if (ch != NULL) {
			err = g->ops.fifo.preempt_channel(g, ch);
			gk20a_channel_put(ch);
		}
		if (err != 0) {
			goto clean_up;
		}
	}

clean_up:
	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}

	if (err) {
		nvgpu_log_fn(g, "failed");
		if (gk20a_fifo_enable_engine_activity(g, eng_info)) {
			nvgpu_err(g,
				"failed to enable gr engine activity");
		}
	} else {
		nvgpu_log_fn(g, "done");
	}
	return err;
}

int gk20a_fifo_disable_all_engine_activity(struct gk20a *g,
					   bool wait_for_idle)
{
	unsigned int i;
	int err = 0, ret = 0;
	u32 active_engine_id;

	for (i = 0; i < g->fifo.num_engines; i++) {
		active_engine_id = g->fifo.active_engines_list[i];
		err = gk20a_fifo_disable_engine_activity(g,
				&g->fifo.engine_info[active_engine_id],
				wait_for_idle);
		if (err) {
			nvgpu_err(g, "failed to disable engine %d activity",
				active_engine_id);
			ret = err;
			break;
		}
	}

	if (err) {
		while (i-- != 0) {
			active_engine_id = g->fifo.active_engines_list[i];
			err = gk20a_fifo_enable_engine_activity(g,
					&g->fifo.engine_info[active_engine_id]);
			if (err) {
				nvgpu_err(g,
					"failed to re-enable engine %d activity",
					active_engine_id);
			}
		}
	}

	return ret;
}

static void gk20a_fifo_runlist_reset_engines(struct gk20a *g, u32 runlist_id)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 engines = 0;
	unsigned int i;

	for (i = 0; i < f->num_engines; i++) {
		u32 active_engine_id = g->fifo.active_engines_list[i];
		u32 status = gk20a_readl(g, fifo_engine_status_r(active_engine_id));
		bool engine_busy = fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v();

		if (engine_busy &&
		    (f->engine_info[active_engine_id].runlist_id == runlist_id)) {
			engines |= BIT(active_engine_id);
		}
	}

	if (engines) {
		gk20a_fifo_recover(g, engines, ~(u32)0, false, false, true,
				RC_TYPE_RUNLIST_UPDATE_TIMEOUT);
	}
}

int gk20a_fifo_runlist_wait_pending(struct gk20a *g, u32 runlist_id)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -ETIMEDOUT;

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	do {
		if ((gk20a_readl(g, fifo_eng_runlist_r(runlist_id)) &
				fifo_eng_runlist_pending_true_f()) == 0) {
			ret = 0;
			break;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret) {
		nvgpu_err(g, "runlist wait timeout: runlist id: %u",
			runlist_id);
	}

	return ret;
}

void gk20a_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
{

	u32 runlist_entry_0 = ram_rl_entry_id_f(tsg->tsgid) |
			ram_rl_entry_type_tsg_f() |
			ram_rl_entry_tsg_length_f(tsg->num_active_channels);

	if (tsg->timeslice_timeout) {
		runlist_entry_0 |=
			ram_rl_entry_timeslice_scale_f(tsg->timeslice_scale) |
			ram_rl_entry_timeslice_timeout_f(tsg->timeslice_timeout);
	} else {
		runlist_entry_0 |=
			ram_rl_entry_timeslice_scale_f(
				NVGPU_FIFO_DEFAULT_TIMESLICE_SCALE) |
			ram_rl_entry_timeslice_timeout_f(
				NVGPU_FIFO_DEFAULT_TIMESLICE_TIMEOUT);
	}

	runlist[0] = runlist_entry_0;
	runlist[1] = 0;

}

u32 gk20a_fifo_default_timeslice_us(struct gk20a *g)
{
	return (((u64)(NVGPU_FIFO_DEFAULT_TIMESLICE_TIMEOUT <<
				NVGPU_FIFO_DEFAULT_TIMESLICE_SCALE) *
			(u64)g->ptimer_src_freq) /
			(u64)PTIMER_REF_FREQ_HZ);
}

void gk20a_get_ch_runlist_entry(struct channel_gk20a *ch, u32 *runlist)
{
	runlist[0] = ram_rl_entry_chid_f(ch->chid);
	runlist[1] = 0;
}

/* recursively construct a runlist with interleaved bare channels and TSGs */
u32 *gk20a_runlist_construct_locked(struct fifo_gk20a *f,
				struct fifo_runlist_info_gk20a *runlist,
				u32 cur_level,
				u32 *runlist_entry,
				bool interleave_enabled,
				bool prev_empty,
				u32 *entries_left)
{
	bool last_level = cur_level == NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_HIGH;
	struct channel_gk20a *ch;
	bool skip_next = false;
	u32 tsgid, count = 0;
	u32 runlist_entry_words = f->runlist_entry_size / sizeof(u32);
	struct gk20a *g = f->g;

	nvgpu_log_fn(g, " ");

	/* for each TSG, T, on this level, insert all higher-level channels
	   and TSGs before inserting T. */
	for_each_set_bit(tsgid, runlist->active_tsgs, f->num_channels) {
		struct tsg_gk20a *tsg = &f->tsg[tsgid];

		if (tsg->interleave_level != cur_level) {
			continue;
		}

		if (!last_level && !skip_next) {
			runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							false,
							entries_left);
			if (!interleave_enabled) {
				skip_next = true;
			}
		}

		if (*entries_left == 0U) {
			return NULL;
		}

		/* add TSG entry */
		nvgpu_log_info(g, "add TSG %d to runlist", tsg->tsgid);
		f->g->ops.fifo.get_tsg_runlist_entry(tsg, runlist_entry);
		nvgpu_log_info(g, "tsg runlist count %d runlist [0] %x [1] %x\n",
				count, runlist_entry[0], runlist_entry[1]);
		runlist_entry += runlist_entry_words;
		count++;
		(*entries_left)--;

		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		/* add runnable channels bound to this TSG */
		nvgpu_list_for_each_entry(ch, &tsg->ch_list,
				channel_gk20a, ch_entry) {
			if (!test_bit((int)ch->chid,
				      runlist->active_channels)) {
				continue;
			}

			if (*entries_left == 0U) {
				nvgpu_rwsem_up_read(&tsg->ch_list_lock);
				return NULL;
			}

			nvgpu_log_info(g, "add channel %d to runlist",
				ch->chid);
			f->g->ops.fifo.get_ch_runlist_entry(ch, runlist_entry);
			nvgpu_log_info(g,
				"run list count %d runlist [0] %x [1] %x\n",
				count, runlist_entry[0], runlist_entry[1]);
			count++;
			runlist_entry += runlist_entry_words;
			(*entries_left)--;
		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	}

	/* append entries from higher level if this level is empty */
	if (!count && !last_level) {
		runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							true,
							entries_left);
	}

	/*
	 * if previous and this level have entries, append
	 * entries from higher level.
	 *
	 * ex. dropping from MEDIUM to LOW, need to insert HIGH
	 */
	if (interleave_enabled && count && !prev_empty && !last_level) {
		runlist_entry = gk20a_runlist_construct_locked(f,
							runlist,
							cur_level + 1,
							runlist_entry,
							interleave_enabled,
							false,
							entries_left);
	}
	return runlist_entry;
}

int gk20a_fifo_set_runlist_interleave(struct gk20a *g,
				u32 id,
				u32 runlist_id,
				u32 new_level)
{
	nvgpu_log_fn(g, " ");

	g->fifo.tsg[id].interleave_level = new_level;

	return 0;
}

int gk20a_fifo_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct gk20a *g = tsg->g;

	if (timeslice < g->min_timeslice_us ||
		timeslice > g->max_timeslice_us) {
		return -EINVAL;
	}

	gk20a_channel_get_timescale_from_timeslice(g, timeslice,
			&tsg->timeslice_timeout, &tsg->timeslice_scale);

	tsg->timeslice_us = timeslice;

	return g->ops.fifo.update_runlist(g, tsg->runlist_id, ~0, true, true);
}

void gk20a_fifo_runlist_hw_submit(struct gk20a *g, u32 runlist_id,
	u32 count, u32 buffer_index)
{
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u64 runlist_iova;

	runlist = &g->fifo.runlist_info[runlist_id];
	runlist_iova = nvgpu_mem_get_addr(g, &runlist->mem[buffer_index]);

	if (count != 0) {
		gk20a_writel(g, fifo_runlist_base_r(),
			fifo_runlist_base_ptr_f(u64_lo32(runlist_iova >> 12)) |
			nvgpu_aperture_mask(g, &runlist->mem[buffer_index],
				fifo_runlist_base_target_sys_mem_ncoh_f(),
				fifo_runlist_base_target_sys_mem_coh_f(),
				fifo_runlist_base_target_vid_mem_f()));
	}

	gk20a_writel(g, fifo_runlist_r(),
		fifo_runlist_engine_f(runlist_id) |
		fifo_eng_runlist_length_f(count));
}

int gk20a_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					    u32 chid, bool add,
					    bool wait_for_finish)
{
	int ret = 0;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 *runlist_entry_base = NULL;
	u64 runlist_iova;
	u32 new_buf;
	struct channel_gk20a *ch = NULL;
	struct tsg_gk20a *tsg = NULL;
	u32 runlist_entry_words = f->runlist_entry_size / sizeof(u32);

	runlist = &f->runlist_info[runlist_id];

	/* valid channel, add/remove it from active list.
	   Otherwise, keep active list untouched for suspend/resume. */
	if (chid != FIFO_INVAL_CHANNEL_ID) {
		ch = &f->channel[chid];
		tsg = tsg_gk20a_from_ch(ch);

		if (add) {
			if (test_and_set_bit(chid,
				runlist->active_channels) == 1) {
				return 0;
			}
			if (tsg && ++tsg->num_active_channels) {
				set_bit((int)f->channel[chid].tsgid,
					runlist->active_tsgs);
			}
		} else {
			if (test_and_clear_bit(chid,
				runlist->active_channels) == 0) {
				return 0;
			}
			if (tsg && --tsg->num_active_channels == 0) {
				clear_bit((int)f->channel[chid].tsgid,
					runlist->active_tsgs);
			}
		}
	}

	new_buf = !runlist->cur_buffer;

	runlist_iova = nvgpu_mem_get_addr(g, &runlist->mem[new_buf]);

	nvgpu_log_info(g, "runlist_id : %d, switch to new buffer 0x%16llx",
		runlist_id, (u64)runlist_iova);

	if (!runlist_iova) {
		ret = -EINVAL;
		goto clean_up;
	}

	runlist_entry_base = runlist->mem[new_buf].cpu_va;
	if (!runlist_entry_base) {
		ret = -ENOMEM;
		goto clean_up;
	}

	if (chid != FIFO_INVAL_CHANNEL_ID || /* add/remove a valid channel */
	    add /* resume to add all channels back */) {
		u32 max_entries = f->num_runlist_entries;
		u32 *runlist_end;

		runlist_end = gk20a_runlist_construct_locked(f,
						runlist,
						0,
						runlist_entry_base,
						g->runlist_interleave,
						true,
						&max_entries);
		if (!runlist_end) {
			ret = -E2BIG;
			goto clean_up;
		}
		runlist->count = (runlist_end - runlist_entry_base) /
			runlist_entry_words;
		WARN_ON(runlist->count > f->num_runlist_entries);
	} else {
		/* suspend to remove all channels */
		runlist->count = 0;
	}

	g->ops.fifo.runlist_hw_submit(g, runlist_id, runlist->count, new_buf);

	if (wait_for_finish) {
		ret = g->ops.fifo.runlist_wait_pending(g, runlist_id);

		if (ret == -ETIMEDOUT) {
			nvgpu_err(g, "runlist %d update timeout", runlist_id);
			/* trigger runlist update timeout recovery */
			return ret;

		} else if (ret == -EINTR) {
			nvgpu_err(g, "runlist update interrupted");
		}
	}

	runlist->cur_buffer = new_buf;

clean_up:
	return ret;
}

int gk20a_fifo_update_runlist_ids(struct gk20a *g, u32 runlist_ids, u32 chid,
				bool add, bool wait_for_finish)
{
	u32 ret = -EINVAL;
	u32 runlist_id = 0;
	u32 errcode;
	unsigned long ulong_runlist_ids = (unsigned long)runlist_ids;

	if (!g) {
		goto end;
	}

	ret = 0;
	for_each_set_bit(runlist_id, &ulong_runlist_ids, 32) {
		/* Capture the last failure error code */
		errcode = g->ops.fifo.update_runlist(g, runlist_id, chid, add, wait_for_finish);
		if (errcode) {
			nvgpu_err(g,
				"failed to update_runlist %d %d", runlist_id, errcode);
			ret = errcode;
		}
	}
end:
	return ret;
}

/* trigger host preempt of GR pending load ctx if that ctx is not for ch */
static int __locked_fifo_reschedule_preempt_next(struct channel_gk20a *ch,
		bool wait_preempt)
{
	struct gk20a *g = ch->g;
	struct fifo_runlist_info_gk20a *runlist =
		&g->fifo.runlist_info[ch->runlist_id];
	int ret = 0;
	u32 gr_eng_id = 0;
	u32 engstat = 0, ctxstat = 0, fecsstat0 = 0, fecsstat1 = 0;
	u32 preempt_id;
	u32 preempt_type = 0;

	if (1 != gk20a_fifo_get_engine_ids(
		g, &gr_eng_id, 1, ENGINE_GR_GK20A)) {
		return ret;
	}
	if (!(runlist->eng_bitmask & (1 << gr_eng_id))) {
		return ret;
	}

	if (wait_preempt && gk20a_readl(g, fifo_preempt_r()) &
		fifo_preempt_pending_true_f()) {
		return ret;
	}

	fecsstat0 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
	engstat = gk20a_readl(g, fifo_engine_status_r(gr_eng_id));
	ctxstat = fifo_engine_status_ctx_status_v(engstat);
	if (ctxstat == fifo_engine_status_ctx_status_ctxsw_switch_v()) {
		/* host switching to next context, preempt that if needed */
		preempt_id = fifo_engine_status_next_id_v(engstat);
		preempt_type = fifo_engine_status_next_id_type_v(engstat);
	} else {
		return ret;
	}
	if (preempt_id == ch->tsgid && preempt_type) {
		return ret;
	}
	fecsstat1 = gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0));
	if (fecsstat0 != FECS_MAILBOX_0_ACK_RESTORE ||
		fecsstat1 != FECS_MAILBOX_0_ACK_RESTORE) {
		/* preempt useless if FECS acked save and started restore */
		return ret;
	}

	gk20a_fifo_issue_preempt(g, preempt_id, preempt_type);
#ifdef TRACEPOINTS_ENABLED
	trace_gk20a_reschedule_preempt_next(ch->chid, fecsstat0, engstat,
		fecsstat1, gk20a_readl(g, gr_fecs_ctxsw_mailbox_r(0)),
		gk20a_readl(g, fifo_preempt_r()));
#endif
	if (wait_preempt) {
		g->ops.fifo.is_preempt_pending(g, preempt_id, preempt_type);
	}
#ifdef TRACEPOINTS_ENABLED
	trace_gk20a_reschedule_preempted_next(ch->chid);
#endif
	return ret;
}

int gk20a_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next)
{
	return nvgpu_fifo_reschedule_runlist(ch, preempt_next, true);
}

/* trigger host to expire current timeslice and reschedule runlist from front */
int nvgpu_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next,
		bool wait_preempt)
{
	struct gk20a *g = ch->g;
	struct fifo_runlist_info_gk20a *runlist;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	int ret = 0;

	runlist = &g->fifo.runlist_info[ch->runlist_id];
	if (!nvgpu_mutex_tryacquire(&runlist->runlist_lock)) {
		return -EBUSY;
	}

	mutex_ret = nvgpu_pmu_mutex_acquire(
		&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	g->ops.fifo.runlist_hw_submit(
		g, ch->runlist_id, runlist->count, runlist->cur_buffer);

	if (preempt_next) {
		__locked_fifo_reschedule_preempt_next(ch, wait_preempt);
	}

	gk20a_fifo_runlist_wait_pending(g, ch->runlist_id);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(
			&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}
	nvgpu_mutex_release(&runlist->runlist_lock);

	return ret;
}

/* add/remove a channel from runlist
   special cases below: runlist->active_channels will NOT be changed.
   (chid == ~0 && !add) means remove all active channels from runlist.
   (chid == ~0 &&  add) means restore all active channels on runlist. */
int gk20a_fifo_update_runlist(struct gk20a *g, u32 runlist_id, u32 chid,
			      bool add, bool wait_for_finish)
{
	struct fifo_runlist_info_gk20a *runlist = NULL;
	struct fifo_gk20a *f = &g->fifo;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret;
	int ret = 0;

	nvgpu_log_fn(g, " ");

	runlist = &f->runlist_info[runlist_id];

	nvgpu_mutex_acquire(&runlist->runlist_lock);

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = gk20a_fifo_update_runlist_locked(g, runlist_id, chid, add,
					       wait_for_finish);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}

	nvgpu_mutex_release(&runlist->runlist_lock);

	if (ret == -ETIMEDOUT) {
		gk20a_fifo_runlist_reset_engines(g, runlist_id);
	}

	return ret;
}

int gk20a_fifo_suspend(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	/* stop bar1 snooping */
	if (g->ops.mm.is_bar1_supported(g)) {
		gk20a_writel(g, fifo_bar1_base_r(),
			fifo_bar1_base_valid_false_f());
	}

	/* disable fifo intr */
	gk20a_writel(g, fifo_intr_en_0_r(), 0);
	gk20a_writel(g, fifo_intr_en_1_r(), 0);

	nvgpu_log_fn(g, "done");
	return 0;
}

bool gk20a_fifo_mmu_fault_pending(struct gk20a *g)
{
	if (gk20a_readl(g, fifo_intr_0_r()) &
			fifo_intr_0_mmu_fault_pending_f()) {
		return true;
	} else {
		return false;
	}
}

bool gk20a_fifo_is_engine_busy(struct gk20a *g)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		if (fifo_engine_status_engine_v(status) ==
			fifo_engine_status_engine_busy_v()) {
			return true;
		}
	}
	return false;
}

int gk20a_fifo_wait_engine_idle(struct gk20a *g)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	int ret = -ETIMEDOUT;
	u32 i, host_num_engines;

	nvgpu_log_fn(g, " ");

	host_num_engines =
		 nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	for (i = 0; i < host_num_engines; i++) {
		do {
			u32 status = gk20a_readl(g, fifo_engine_status_r(i));
			if (!fifo_engine_status_engine_v(status)) {
				ret = 0;
				break;
			}

			nvgpu_usleep_range(delay, delay * 2);
			delay = min_t(unsigned long,
					delay << 1, GR_IDLE_CHECK_MAX);
		} while (!nvgpu_timeout_expired(&timeout));

		if (ret) {
			nvgpu_log_info(g, "cannot idle engine %u", i);
			break;
		}
	}

	nvgpu_log_fn(g, "done");

	return ret;
}

u32 gk20a_fifo_get_pbdma_signature(struct gk20a *g)
{
	return pbdma_signature_hw_valid_f() | pbdma_signature_sw_zero_f();
}

static const char * const ccsr_chan_status_str[] = {
	"idle",
	"pending",
	"pending_ctx_reload",
	"pending_acquire",
	"pending_acq_ctx_reload",
	"on_pbdma",
	"on_pbdma_and_eng",
	"on_eng",
	"on_eng_pending_acquire",
	"on_eng_pending",
	"on_pbdma_ctx_reload",
	"on_pbdma_and_eng_ctx_reload",
	"on_eng_ctx_reload",
	"on_eng_pending_ctx_reload",
	"on_eng_pending_acq_ctx_reload",
};

static const char * const pbdma_chan_eng_ctx_status_str[] = {
	"invalid",
	"valid",
	"NA",
	"NA",
	"NA",
	"load",
	"save",
	"switch",
};

static const char * const not_found_str[] = {
	"NOT FOUND"
};

const char *gk20a_decode_ccsr_chan_status(u32 index)
{
	if (index >= ARRAY_SIZE(ccsr_chan_status_str)) {
		return not_found_str[0];
	} else {
		return ccsr_chan_status_str[index];
	}
}

const char *gk20a_decode_pbdma_chan_eng_ctx_status(u32 index)
{
	if (index >= ARRAY_SIZE(pbdma_chan_eng_ctx_status_str)) {
		return not_found_str[0];
	} else {
		return pbdma_chan_eng_ctx_status_str[index];
	}
}

bool gk20a_fifo_channel_status_is_next(struct gk20a *g, u32 chid)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));

	return ccsr_channel_next_v(channel) == ccsr_channel_next_true_v();
}

bool gk20a_fifo_channel_status_is_ctx_reload(struct gk20a *g, u32 chid)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));
	u32 status = ccsr_channel_status_v(channel);

	return (status == ccsr_channel_status_pending_ctx_reload_v() ||
		status == ccsr_channel_status_pending_acq_ctx_reload_v() ||
		status == ccsr_channel_status_on_pbdma_ctx_reload_v() ||
		status == ccsr_channel_status_on_pbdma_and_eng_ctx_reload_v() ||
		status == ccsr_channel_status_on_eng_ctx_reload_v() ||
		status == ccsr_channel_status_on_eng_pending_ctx_reload_v() ||
		status == ccsr_channel_status_on_eng_pending_acq_ctx_reload_v());
}

void gk20a_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));
	u32 status = ccsr_channel_status_v(channel);
	u32 syncpointa, syncpointb;
	u32 *inst_mem;
	struct channel_gk20a *c = g->fifo.channel + chid;
	struct nvgpu_semaphore_int *hw_sema = NULL;

	if (c->hw_sema) {
		hw_sema = c->hw_sema;
	}

	if (!ch_state) {
		return;
	}

	inst_mem = &ch_state->inst_block[0];

	syncpointa = inst_mem[ram_fc_syncpointa_w()];
	syncpointb = inst_mem[ram_fc_syncpointb_w()];

	gk20a_debug_output(o, "%d-%s, pid %d, refs %d%s: ", chid,
			g->name,
			ch_state->pid,
			ch_state->refs,
			ch_state->deterministic ? ", deterministic" : "");
	gk20a_debug_output(o, "channel status: %s in use %s %s\n",
			ccsr_channel_enable_v(channel) ? "" : "not",
			gk20a_decode_ccsr_chan_status(status),
			ccsr_channel_busy_v(channel) ? "busy" : "not busy");
	gk20a_debug_output(o, "RAMFC : TOP: %016llx PUT: %016llx GET: %016llx "
			"FETCH: %016llx\nHEADER: %08x COUNT: %08x\n"
			"SYNCPOINT %08x %08x SEMAPHORE %08x %08x %08x %08x\n",
		(u64)inst_mem[ram_fc_pb_top_level_get_w()] +
		((u64)inst_mem[ram_fc_pb_top_level_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_put_w()] +
		((u64)inst_mem[ram_fc_pb_put_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_get_w()] +
		((u64)inst_mem[ram_fc_pb_get_hi_w()] << 32ULL),
		(u64)inst_mem[ram_fc_pb_fetch_w()] +
		((u64)inst_mem[ram_fc_pb_fetch_hi_w()] << 32ULL),
		inst_mem[ram_fc_pb_header_w()],
		inst_mem[ram_fc_pb_count_w()],
		syncpointa,
		syncpointb,
		inst_mem[ram_fc_semaphorea_w()],
		inst_mem[ram_fc_semaphoreb_w()],
		inst_mem[ram_fc_semaphorec_w()],
		inst_mem[ram_fc_semaphored_w()]);
	if (hw_sema) {
		gk20a_debug_output(o, "SEMA STATE: value: 0x%08x "
				   "next_val: 0x%08x addr: 0x%010llx\n",
				   __nvgpu_semaphore_read(hw_sema),
				   nvgpu_atomic_read(&hw_sema->next_value),
				   nvgpu_hw_sema_addr(hw_sema));
	}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	if ((pbdma_syncpointb_op_v(syncpointb) == pbdma_syncpointb_op_wait_v())
		&& (pbdma_syncpointb_wait_switch_v(syncpointb) ==
			pbdma_syncpointb_wait_switch_en_v()))
		gk20a_debug_output(o, "%s on syncpt %u (%s) val %u\n",
			(status == 3 || status == 8) ? "Waiting" : "Waited",
			pbdma_syncpointb_syncpt_index_v(syncpointb),
			nvgpu_nvhost_syncpt_get_name(g->nvhost_dev,
				pbdma_syncpointb_syncpt_index_v(syncpointb)),
			pbdma_syncpointa_payload_v(syncpointa));
#endif

	gk20a_debug_output(o, "\n");
}

void gk20a_debug_dump_all_channel_status_ramfc(struct gk20a *g,
		 struct gk20a_debug_output *o)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	struct ch_state **ch_state;

	ch_state = nvgpu_kzalloc(g, sizeof(*ch_state) * f->num_channels);
	if (!ch_state) {
		gk20a_debug_output(o, "cannot alloc memory for channels\n");
		return;
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = gk20a_channel_from_id(g, chid);
		if (ch != NULL) {
			ch_state[chid] =
				nvgpu_kmalloc(g, sizeof(struct ch_state) +
					ram_in_alloc_size_v());
			/* ref taken stays to below loop with
			 * successful allocs */
			if (!ch_state[chid]) {
				gk20a_channel_put(ch);
			}
		}
	}

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];
		if (!ch_state[chid]) {
			continue;
		}

		ch_state[chid]->pid = ch->pid;
		ch_state[chid]->refs = nvgpu_atomic_read(&ch->ref_count);
		ch_state[chid]->deterministic = ch->deterministic;
		nvgpu_mem_rd_n(g, &ch->inst_block, 0,
				&ch_state[chid]->inst_block[0],
				ram_in_alloc_size_v());
		gk20a_channel_put(ch);
	}
	for (chid = 0; chid < f->num_channels; chid++) {
		if (ch_state[chid]) {
			g->ops.fifo.dump_channel_status_ramfc(g, o, chid,
						 ch_state[chid]);
			nvgpu_kfree(g, ch_state[chid]);
		}
	}
	nvgpu_kfree(g, ch_state);
}

void gk20a_dump_pbdma_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_pbdma;

	host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	for (i = 0; i < host_num_pbdma; i++) {
		u32 status = gk20a_readl(g, fifo_pbdma_status_r(i));
		u32 chan_status = fifo_pbdma_status_chan_status_v(status);

		gk20a_debug_output(o, "%s pbdma %d: ", g->name, i);
		gk20a_debug_output(o,
				"id: %d (%s), next_id: %d (%s) chan status: %s\n",
				fifo_pbdma_status_id_v(status),
				fifo_pbdma_status_id_type_v(status) ?
					"tsg" : "channel",
				fifo_pbdma_status_next_id_v(status),
				fifo_pbdma_status_next_id_type_v(status) ?
					"tsg" : "channel",
			gk20a_decode_pbdma_chan_eng_ctx_status(chan_status));
		gk20a_debug_output(o, "PBDMA_PUT: %016llx PBDMA_GET: %016llx "
				"GP_PUT: %08x GP_GET: %08x "
				"FETCH: %08x HEADER: %08x\n"
				"HDR: %08x SHADOW0: %08x SHADOW1: %08x",
			(u64)gk20a_readl(g, pbdma_put_r(i)) +
			((u64)gk20a_readl(g, pbdma_put_hi_r(i)) << 32ULL),
			(u64)gk20a_readl(g, pbdma_get_r(i)) +
			((u64)gk20a_readl(g, pbdma_get_hi_r(i)) << 32ULL),
			gk20a_readl(g, pbdma_gp_put_r(i)),
			gk20a_readl(g, pbdma_gp_get_r(i)),
			gk20a_readl(g, pbdma_gp_fetch_r(i)),
			gk20a_readl(g, pbdma_pb_header_r(i)),
			gk20a_readl(g, pbdma_hdr_shadow_r(i)),
			gk20a_readl(g, pbdma_gp_shadow_0_r(i)),
			gk20a_readl(g, pbdma_gp_shadow_1_r(i)));
	}
	gk20a_debug_output(o, "\n");
}

void gk20a_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o)
{
	u32 i, host_num_engines;

	host_num_engines = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_ENGINES);

	for (i = 0; i < host_num_engines; i++) {
		u32 status = gk20a_readl(g, fifo_engine_status_r(i));
		u32 ctx_status = fifo_engine_status_ctx_status_v(status);

		gk20a_debug_output(o, "%s eng %d: ", g->name, i);
		gk20a_debug_output(o,
			"id: %d (%s), next_id: %d (%s), ctx status: %s ",
			fifo_engine_status_id_v(status),
			fifo_engine_status_id_type_v(status) ?
				"tsg" : "channel",
			fifo_engine_status_next_id_v(status),
			fifo_engine_status_next_id_type_v(status) ?
				"tsg" : "channel",
			gk20a_decode_pbdma_chan_eng_ctx_status(ctx_status));

		if (fifo_engine_status_faulted_v(status)) {
			gk20a_debug_output(o, "faulted ");
		}
		if (fifo_engine_status_engine_v(status)) {
			gk20a_debug_output(o, "busy ");
		}
		gk20a_debug_output(o, "\n");
	}
	gk20a_debug_output(o, "\n");
}

void gk20a_fifo_enable_channel(struct channel_gk20a *ch)
{
	gk20a_writel(ch->g, ccsr_channel_r(ch->chid),
		gk20a_readl(ch->g, ccsr_channel_r(ch->chid)) |
		ccsr_channel_enable_set_true_f());
}

void gk20a_fifo_disable_channel(struct channel_gk20a *ch)
{
	gk20a_writel(ch->g, ccsr_channel_r(ch->chid),
		gk20a_readl(ch->g,
			ccsr_channel_r(ch->chid)) |
			ccsr_channel_enable_clr_true_f());
}

void gk20a_fifo_channel_unbind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;

	nvgpu_log_fn(g, " ");

	if (nvgpu_atomic_cmpxchg(&ch_gk20a->bound, true, false)) {
		gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->chid),
			ccsr_channel_inst_ptr_f(0) |
			ccsr_channel_inst_bind_false_f());
	}
}

static int gk20a_fifo_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	struct gk20a *g = c->g;

	nvgpu_log_fn(g, " ");

	addr_lo = u64_lo32(c->userd_iova >> ram_userd_base_shift_v());
	addr_hi = u64_hi32(c->userd_iova);

	nvgpu_log_info(g, "channel %d : set ramfc userd 0x%16llx",
		c->chid, (u64)c->userd_iova);

	nvgpu_mem_wr32(g, &c->inst_block,
		       ram_in_ramfc_w() + ram_fc_userd_w(),
		       nvgpu_aperture_mask(g, &g->fifo.userd,
					   pbdma_userd_target_sys_mem_ncoh_f(),
					   pbdma_userd_target_sys_mem_coh_f(),
					   pbdma_userd_target_vid_mem_f()) |
		       pbdma_userd_addr_f(addr_lo));

	nvgpu_mem_wr32(g, &c->inst_block,
		       ram_in_ramfc_w() + ram_fc_userd_hi_w(),
		       pbdma_userd_hi_addr_f(addr_hi));

	return 0;
}

int gk20a_fifo_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long timeout,
			u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem = &c->inst_block;

	nvgpu_log_fn(g, " ");

	nvgpu_memset(g, mem, 0, 0, ram_fc_size_val_v());

	nvgpu_mem_wr32(g, mem, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	nvgpu_mem_wr32(g, mem, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	nvgpu_mem_wr32(g, mem, ram_fc_signature_w(),
		 c->g->ops.fifo.get_pbdma_signature(c->g));

	nvgpu_mem_wr32(g, mem, ram_fc_formats_w(),
		pbdma_formats_gp_fermi0_f() |
		pbdma_formats_pb_fermi1_f() |
		pbdma_formats_mp_fermi0_f());

	nvgpu_mem_wr32(g, mem, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	nvgpu_mem_wr32(g, mem, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(1) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	nvgpu_mem_wr32(g, mem, ram_fc_target_w(), pbdma_target_engine_sw_f());

	nvgpu_mem_wr32(g, mem, ram_fc_acquire_w(),
		g->ops.fifo.pbdma_acquire_val(timeout));

	nvgpu_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		fifo_runlist_timeslice_timeout_128_f() |
		fifo_runlist_timeslice_timescale_3_f() |
		fifo_runlist_timeslice_enable_true_f());

	nvgpu_mem_wr32(g, mem, ram_fc_pb_timeslice_w(),
		fifo_pb_timeslice_timeout_16_f() |
		fifo_pb_timeslice_timescale_0_f() |
		fifo_pb_timeslice_enable_true_f());

	nvgpu_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->chid));

	if (c->is_privileged_channel) {
		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	return gk20a_fifo_commit_userd(c);
}

void gk20a_fifo_setup_ramfc_for_privileged_channel(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem = &c->inst_block;

	nvgpu_log_info(g, "channel %d : set ramfc privileged_channel", c->chid);

	/* Enable HCE priv mode for phys mode transfer */
	nvgpu_mem_wr32(g, mem, ram_fc_hce_ctrl_w(),
		pbdma_hce_ctrl_hce_priv_mode_yes_f());
}

int gk20a_fifo_setup_userd(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem;
	u32 offset;

	nvgpu_log_fn(g, " ");

	if (nvgpu_mem_is_valid(&c->usermode_userd)) {
		mem = &c->usermode_userd;
		offset = 0;
	} else {
		mem = &g->fifo.userd;
		offset = c->chid * g->fifo.userd_entry_size / sizeof(u32);
	}

	nvgpu_mem_wr32(g, mem, offset + ram_userd_put_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_get_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_ref_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_put_hi_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_ref_threshold_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_gp_top_level_get_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_gp_top_level_get_hi_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_get_hi_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_gp_get_w(), 0);
	nvgpu_mem_wr32(g, mem, offset + ram_userd_gp_put_w(), 0);

	return 0;
}

int gk20a_fifo_alloc_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	int err;

	nvgpu_log_fn(g, " ");

	err = g->ops.mm.alloc_inst_block(g, &ch->inst_block);
	if (err) {
		return err;
	}

	nvgpu_log_info(g, "channel %d inst block physical addr: 0x%16llx",
		ch->chid, nvgpu_inst_block_addr(g, &ch->inst_block));

	nvgpu_log_fn(g, "done");
	return 0;
}

void gk20a_fifo_free_inst(struct gk20a *g, struct channel_gk20a *ch)
{
	nvgpu_free_inst_block(g, &ch->inst_block);
}

u32 gk20a_fifo_userd_gp_get(struct gk20a *g, struct channel_gk20a *c)
{
	return gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_gp_get_w());
}

u64 gk20a_fifo_userd_pb_get(struct gk20a *g, struct channel_gk20a *c)
{
	u32 lo = gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_get_w());
	u32 hi = gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_get_hi_w());

	return ((u64)hi << 32) | lo;
}

void gk20a_fifo_userd_gp_put(struct gk20a *g, struct channel_gk20a *c)
{
	gk20a_bar1_writel(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_gp_put_w(),
		c->gpfifo.put);
}

u32 gk20a_fifo_pbdma_acquire_val(u64 timeout)
{
	u32 val, exp, man;
	unsigned int val_len;

	val = pbdma_acquire_retry_man_2_f() |
		pbdma_acquire_retry_exp_2_f();

	if (!timeout) {
		return val;
	}

	timeout *= 80UL;
	do_div(timeout, 100); /* set acquire timeout to 80% of channel wdt */
	timeout *= 1000000UL; /* ms -> ns */
	do_div(timeout, 1024); /* in unit of 1024ns */
	val_len = fls(timeout >> 32) + 32;
	if (val_len == 32) {
		val_len = fls(timeout);
	}
	if (val_len > 16U + pbdma_acquire_timeout_exp_max_v()) { /* man: 16bits */
		exp = pbdma_acquire_timeout_exp_max_v();
		man = pbdma_acquire_timeout_man_max_v();
	} else if (val_len > 16) {
		exp = val_len - 16;
		man = timeout >> exp;
	} else {
		exp = 0;
		man = timeout;
	}

	val |= pbdma_acquire_timeout_exp_f(exp) |
		pbdma_acquire_timeout_man_f(man) |
		pbdma_acquire_timeout_en_enable_f();

	return val;
}

const char *gk20a_fifo_interleave_level_name(u32 interleave_level)
{
	switch (interleave_level) {
	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_LOW:
		return "LOW";

	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
		return "MEDIUM";

	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		return "HIGH";

	default:
		return "?";
	}
}

u32 gk20a_fifo_get_sema_wait_cmd_size(void)
{
	return 8;
}

u32 gk20a_fifo_get_sema_incr_cmd_size(void)
{
	return 10;
}

void gk20a_fifo_add_sema_cmd(struct gk20a *g,
	struct nvgpu_semaphore *s, u64 sema_va,
	struct priv_cmd_entry *cmd,
	u32 off, bool acquire, bool wfi)
{
	nvgpu_log_fn(g, " ");

	/* semaphore_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010004);
	/* offset_upper */
	nvgpu_mem_wr32(g, cmd->mem, off++, (sema_va >> 32) & 0xff);
	/* semaphore_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010005);
	/* offset */
	nvgpu_mem_wr32(g, cmd->mem, off++, sema_va & 0xffffffff);

	if (acquire) {
		/* semaphore_c */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
		/* payload */
		nvgpu_mem_wr32(g, cmd->mem, off++,
			       nvgpu_semaphore_get_value(s));
		/* semaphore_d */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);
		/* operation: acq_geq, switch_en */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x4 | (0x1 << 12));
	} else {
		/* semaphore_c */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010006);
		/* payload */
		nvgpu_mem_wr32(g, cmd->mem, off++,
			       nvgpu_semaphore_get_value(s));
		/* semaphore_d */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010007);
		/* operation: release, wfi */
		nvgpu_mem_wr32(g, cmd->mem, off++,
				0x2 | ((wfi ? 0x0 : 0x1) << 20));
		/* non_stall_int */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010008);
		/* ignored */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0);
	}
}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
void gk20a_fifo_add_syncpt_wait_cmd(struct gk20a *g,
		struct priv_cmd_entry *cmd, u32 off,
		u32 id, u32 thresh, u64 gpu_va)
{
	nvgpu_log_fn(g, " ");

	off = cmd->off + off;
	/* syncpoint_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001C);
	/* payload */
	nvgpu_mem_wr32(g, cmd->mem, off++, thresh);
	/* syncpoint_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001D);
	/* syncpt_id, switch_en, wait */
	nvgpu_mem_wr32(g, cmd->mem, off++, (id << 8) | 0x10);
}

u32 gk20a_fifo_get_syncpt_wait_cmd_size(void)
{
	return 4;
}

u32 gk20a_fifo_get_syncpt_incr_per_release(void)
{
	return 2;
}

void gk20a_fifo_add_syncpt_incr_cmd(struct gk20a *g,
		bool wfi_cmd, struct priv_cmd_entry *cmd,
		u32 id, u64 gpu_va)
{
	u32 off = cmd->off;

	nvgpu_log_fn(g, " ");
	if (wfi_cmd) {
		/* wfi */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001E);
		/* handle, ignored */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x00000000);
	}
	/* syncpoint_a */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001C);
	/* payload, ignored */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);
	/* syncpoint_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001D);
	/* syncpt_id, incr */
	nvgpu_mem_wr32(g, cmd->mem, off++, (id << 8) | 0x1);
	/* syncpoint_b */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001D);
	/* syncpt_id, incr */
	nvgpu_mem_wr32(g, cmd->mem, off++, (id << 8) | 0x1);

}

u32 gk20a_fifo_get_syncpt_incr_cmd_size(bool wfi_cmd)
{
	if (wfi_cmd)
		return 8;
	else
		return 6;
}

void gk20a_fifo_free_syncpt_buf(struct channel_gk20a *c,
				struct nvgpu_mem *syncpt_buf)
{

}

int gk20a_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf)
{
	return 0;
}
#endif
