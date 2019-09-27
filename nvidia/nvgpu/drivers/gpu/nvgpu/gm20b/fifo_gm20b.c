/*
 * GM20B Fifo
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

#include <nvgpu/timers.h>
#include <nvgpu/log.h>
#include <nvgpu/atomic.h>
#include <nvgpu/barrier.h>
#include <nvgpu/mm.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/bug.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include "gk20a/fifo_gk20a.h"

#include "fifo_gm20b.h"

#include <nvgpu/hw/gm20b/hw_ccsr_gm20b.h>
#include <nvgpu/hw/gm20b/hw_ram_gm20b.h>
#include <nvgpu/hw/gm20b/hw_fifo_gm20b.h>
#include <nvgpu/hw/gm20b/hw_top_gm20b.h>
#include <nvgpu/hw/gm20b/hw_pbdma_gm20b.h>

void channel_gm20b_bind(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;

	u32 inst_ptr = nvgpu_inst_block_addr(g, &c->inst_block)
		>> ram_in_base_shift_v();

	nvgpu_log_info(g, "bind channel %d inst ptr 0x%08x",
		c->chid, inst_ptr);


	gk20a_writel(g, ccsr_channel_inst_r(c->chid),
		     ccsr_channel_inst_ptr_f(inst_ptr) |
		     nvgpu_aperture_mask(g, &c->inst_block,
				ccsr_channel_inst_target_sys_mem_ncoh_f(),
				ccsr_channel_inst_target_sys_mem_coh_f(),
				ccsr_channel_inst_target_vid_mem_f()) |
		     ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(c->chid),
		(gk20a_readl(g, ccsr_channel_r(c->chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
	nvgpu_smp_wmb();
	nvgpu_atomic_set(&c->bound, true);
}

static inline u32 gm20b_engine_id_to_mmu_id(struct gk20a *g, u32 engine_id)
{
	u32 fault_id = ~0;
	struct fifo_engine_info_gk20a *engine_info;

	engine_info = gk20a_fifo_get_engine_info(g, engine_id);

	if (engine_info) {
		fault_id = engine_info->fault_id;
	} else {
		nvgpu_err(g, "engine_id is not in active list/invalid %d", engine_id);
	}
	return fault_id;
}

void gm20b_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids)
{
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;
	unsigned long engine_id;
	int ret = -EBUSY;
	struct nvgpu_timeout timeout;

	/* trigger faults for all bad engines */
	for_each_set_bit(engine_id, &engine_ids, 32) {
		if (!gk20a_fifo_is_valid_engine_id(g, engine_id)) {
			nvgpu_err(g, "faulting unknown engine %ld", engine_id);
		} else {
			u32 mmu_id = gm20b_engine_id_to_mmu_id(g,
								engine_id);
			if (mmu_id != (u32)~0) {
				gk20a_writel(g, fifo_trigger_mmu_fault_r(mmu_id),
					     fifo_trigger_mmu_fault_enable_f(1));
			}
		}
	}

	nvgpu_timeout_init(g, &timeout, gk20a_get_gr_idle_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	/* Wait for MMU fault to trigger */
	do {
		if (gk20a_readl(g, fifo_intr_0_r()) &
				fifo_intr_0_mmu_fault_pending_f()) {
			ret = 0;
			break;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret) {
		nvgpu_err(g, "mmu fault timeout");
	}

	/* release mmu fault trigger */
	for_each_set_bit(engine_id, &engine_ids, 32) {
		gk20a_writel(g, fifo_trigger_mmu_fault_r(engine_id), 0);
	}
}

u32 gm20b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

void gm20b_device_info_data_parse(struct gk20a *g,
						u32 table_entry, u32 *inst_id,
						u32 *pri_base, u32 *fault_id)
{
	if (top_device_info_data_type_v(table_entry) ==
	    top_device_info_data_type_enum2_v()) {
		if (pri_base) {
			*pri_base =
				(top_device_info_data_pri_base_v(table_entry)
				<< top_device_info_data_pri_base_align_v());
		}
		if (fault_id && (top_device_info_data_fault_id_v(table_entry) ==
			top_device_info_data_fault_id_valid_v())) {
			*fault_id =
			    top_device_info_data_fault_id_enum_v(table_entry);
		}
	} else {
		nvgpu_err(g, "unknown device_info_data %d",
				top_device_info_data_type_v(table_entry));
	}
}

void gm20b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
{
	/*
	 * These are all errors which indicate something really wrong
	 * going on in the device.
	 */
	f->intr.pbdma.device_fatal_0 =
		pbdma_intr_0_memreq_pending_f() |
		pbdma_intr_0_memack_timeout_pending_f() |
		pbdma_intr_0_memack_extra_pending_f() |
		pbdma_intr_0_memdat_timeout_pending_f() |
		pbdma_intr_0_memdat_extra_pending_f() |
		pbdma_intr_0_memflush_pending_f() |
		pbdma_intr_0_memop_pending_f() |
		pbdma_intr_0_lbconnect_pending_f() |
		pbdma_intr_0_lback_timeout_pending_f() |
		pbdma_intr_0_lback_extra_pending_f() |
		pbdma_intr_0_lbdat_timeout_pending_f() |
		pbdma_intr_0_lbdat_extra_pending_f() |
		pbdma_intr_0_pri_pending_f();

	/*
	 * These are data parsing, framing errors or others which can be
	 * recovered from with intervention... or just resetting the
	 * channel
	 */
	f->intr.pbdma.channel_fatal_0 =
		pbdma_intr_0_gpfifo_pending_f() |
		pbdma_intr_0_gpptr_pending_f() |
		pbdma_intr_0_gpentry_pending_f() |
		pbdma_intr_0_gpcrc_pending_f() |
		pbdma_intr_0_pbptr_pending_f() |
		pbdma_intr_0_pbentry_pending_f() |
		pbdma_intr_0_pbcrc_pending_f() |
		pbdma_intr_0_method_pending_f() |
		pbdma_intr_0_methodcrc_pending_f() |
		pbdma_intr_0_pbseg_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

static void gm20b_fifo_set_ctx_reload(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	u32 channel = gk20a_readl(g, ccsr_channel_r(ch->chid));

	gk20a_writel(g, ccsr_channel_r(ch->chid),
		channel | ccsr_channel_force_ctx_reload_true_f());
}

void gm20b_fifo_tsg_verify_status_ctx_reload(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg = &g->fifo.tsg[ch->tsgid];
	struct channel_gk20a *temp_ch;

	/* If CTX_RELOAD is set on a channel, move it to some other channel */
	if (gk20a_fifo_channel_status_is_ctx_reload(ch->g, ch->chid)) {
		nvgpu_rwsem_down_read(&tsg->ch_list_lock);
		nvgpu_list_for_each_entry(temp_ch, &tsg->ch_list, channel_gk20a, ch_entry) {
			if (temp_ch->chid != ch->chid) {
				gm20b_fifo_set_ctx_reload(temp_ch);
				break;
			}
		}
		nvgpu_rwsem_up_read(&tsg->ch_list_lock);
	}
}

static const char * const gm20b_gpc_client_descs[] = {
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
	"l1 9", "t1 9", "pe 9",
	"l1 10", "t1 10", "pe 10",
	"l1 11", "t1 11", "pe 11",
	"unknown", "unknown", "unknown", "unknown",
	"tpccs 0", "tpccs 1", "tpccs 2",
	"tpccs 3", "tpccs 4", "tpccs 5",
	"tpccs 6", "tpccs 7", "tpccs 8",
	"tpccs 9", "tpccs 10", "tpccs 11",
};

void gm20b_fifo_get_mmu_fault_gpc_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gm20b_gpc_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gm20b_gpc_client_descs));
	} else {
		mmfault->client_id_desc =
			 gm20b_gpc_client_descs[mmfault->client_id];
	}
}
