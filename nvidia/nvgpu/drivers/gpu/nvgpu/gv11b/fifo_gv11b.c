/*
 * GV11B fifo
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bug.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/timers.h>
#include <nvgpu/log.h>
#include <nvgpu/dma.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/soc.h>
#include <nvgpu/debug.h>
#include <nvgpu/nvhost.h>
#include <nvgpu/barrier.h>
#include <nvgpu/mm.h>
#include <nvgpu/log2.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/io_usermode.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/unit.h>
#include <nvgpu/power_features/cg.h>
#include <nvgpu/power_features/power_features.h>

#include "gk20a/fifo_gk20a.h"

#include "gp10b/fifo_gp10b.h"

#include <nvgpu/hw/gv11b/hw_pbdma_gv11b.h>
#include <nvgpu/hw/gv11b/hw_fifo_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ccsr_gv11b.h>
#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>
#include <nvgpu/hw/gv11b/hw_top_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gmmu_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>

#include "fifo_gv11b.h"
#include "subctx_gv11b.h"
#include "gr_gv11b.h"

void gv11b_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist)
{
	struct gk20a *g = tsg->g;
	u32 runlist_entry_0 = ram_rl_entry_type_tsg_v();

	if (tsg->timeslice_timeout) {
		runlist_entry_0 |=
		ram_rl_entry_tsg_timeslice_scale_f(tsg->timeslice_scale) |
		ram_rl_entry_tsg_timeslice_timeout_f(tsg->timeslice_timeout);
	} else {
		runlist_entry_0 |=
			ram_rl_entry_tsg_timeslice_scale_f(
				ram_rl_entry_tsg_timeslice_scale_3_v()) |
			ram_rl_entry_tsg_timeslice_timeout_f(
				ram_rl_entry_tsg_timeslice_timeout_128_v());
	}

	runlist[0] = runlist_entry_0;
	runlist[1] = ram_rl_entry_tsg_length_f(tsg->num_active_channels);
	runlist[2] = ram_rl_entry_tsg_tsgid_f(tsg->tsgid);
	runlist[3] = 0;

	nvgpu_log_info(g, "gv11b tsg runlist [0] %x [1]  %x [2] %x [3] %x\n",
		runlist[0], runlist[1], runlist[2], runlist[3]);

}

void gv11b_get_ch_runlist_entry(struct channel_gk20a *c, u32 *runlist)
{
	struct gk20a *g = c->g;
	u32 addr_lo, addr_hi;
	u32 runlist_entry;

	/* Time being use 0 pbdma sequencer */
	runlist_entry = ram_rl_entry_type_channel_v() |
			ram_rl_entry_chan_runqueue_selector_f(
						c->runqueue_sel) |
			ram_rl_entry_chan_userd_target_f(
				nvgpu_aperture_mask(g, &g->fifo.userd,
					ram_rl_entry_chan_userd_target_sys_mem_ncoh_v(),
					ram_rl_entry_chan_userd_target_sys_mem_coh_v(),
					ram_rl_entry_chan_userd_target_vid_mem_v())) |
			ram_rl_entry_chan_inst_target_f(
				nvgpu_aperture_mask(g, &c->inst_block,
					ram_rl_entry_chan_inst_target_sys_mem_ncoh_v(),
					ram_rl_entry_chan_inst_target_sys_mem_coh_v(),
					ram_rl_entry_chan_inst_target_vid_mem_v()));

	addr_lo = u64_lo32(c->userd_iova) >>
			ram_rl_entry_chan_userd_ptr_align_shift_v();
	addr_hi = u64_hi32(c->userd_iova);
	runlist[0] = runlist_entry | ram_rl_entry_chan_userd_ptr_lo_f(addr_lo);
	runlist[1] = ram_rl_entry_chan_userd_ptr_hi_f(addr_hi);

	addr_lo = u64_lo32(nvgpu_inst_block_addr(g, &c->inst_block)) >>
			ram_rl_entry_chan_inst_ptr_align_shift_v();
	addr_hi = u64_hi32(nvgpu_inst_block_addr(g, &c->inst_block));

	runlist[2] = ram_rl_entry_chan_inst_ptr_lo_f(addr_lo) |
				ram_rl_entry_chid_f(c->chid);
	runlist[3] = ram_rl_entry_chan_inst_ptr_hi_f(addr_hi);

	nvgpu_log_info(g, "gv11b channel runlist [0] %x [1]  %x [2] %x [3] %x\n",
			runlist[0], runlist[1], runlist[2], runlist[3]);
}

void gv11b_userd_writeback_config(struct gk20a *g)
{
	gk20a_writel(g, fifo_userd_writeback_r(), fifo_userd_writeback_timer_f(
				fifo_userd_writeback_timer_100us_v()));


}

int channel_gv11b_setup_ramfc(struct channel_gk20a *c,
		u64 gpfifo_base, u32 gpfifo_entries,
		unsigned long acquire_timeout, u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *mem = &c->inst_block;
	u32 data;
	bool replayable = false;

	nvgpu_log_fn(g, " ");

	nvgpu_memset(g, mem, 0, 0, ram_fc_size_val_v());

	if ((flags & NVGPU_SETUP_BIND_FLAGS_REPLAYABLE_FAULTS_ENABLE) != 0) {
		replayable = true;
	}
	gv11b_init_subcontext_pdb(c->vm, mem, replayable);

        nvgpu_mem_wr32(g, mem, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	nvgpu_mem_wr32(g, mem, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	nvgpu_mem_wr32(g, mem, ram_fc_signature_w(),
		c->g->ops.fifo.get_pbdma_signature(c->g));

	nvgpu_mem_wr32(g, mem, ram_fc_pb_header_w(),
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	nvgpu_mem_wr32(g, mem, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(PBDMA_SUBDEVICE_ID) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	nvgpu_mem_wr32(g, mem, ram_fc_target_w(),
		pbdma_target_eng_ctx_valid_true_f() |
		pbdma_target_ce_ctx_valid_true_f() |
		pbdma_target_engine_sw_f());

	nvgpu_mem_wr32(g, mem, ram_fc_acquire_w(),
		g->ops.fifo.pbdma_acquire_val(acquire_timeout));

	nvgpu_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());


	nvgpu_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->chid));

	nvgpu_mem_wr32(g, mem, ram_fc_set_channel_info_w(),
			pbdma_set_channel_info_veid_f(c->subctx_id));

	nvgpu_mem_wr32(g, mem, ram_in_engine_wfi_veid_w(),
			ram_in_engine_wfi_veid_f(c->subctx_id));

	gv11b_fifo_init_ramfc_eng_method_buffer(g, c, mem);

	if (c->is_privileged_channel) {
		/* Set privilege level for channel */
		nvgpu_mem_wr32(g, mem, ram_fc_config_w(),
			pbdma_config_auth_level_privileged_f());

		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	/* Enable userd writeback */
	data = nvgpu_mem_rd32(g, mem, ram_fc_config_w());
	data = data | pbdma_config_userd_writeback_enable_f();
	nvgpu_mem_wr32(g, mem, ram_fc_config_w(),data);

	gv11b_userd_writeback_config(g);

	return channel_gp10b_commit_userd(c);
}

u64 gv11b_fifo_usermode_base(struct gk20a *g)
{
	return usermode_cfg0_r();
}

void gv11b_ring_channel_doorbell(struct channel_gk20a *c)
{
	struct gk20a *g = c->g;
	struct fifo_gk20a *f = &g->fifo;
	u32 hw_chid = f->channel_base + c->chid;

	nvgpu_log_info(g, "channel ring door bell %d\n", c->chid);

	nvgpu_usermode_writel(c->g, usermode_notify_channel_pending_r(),
		usermode_notify_channel_pending_id_f(hw_chid));
}

u32 gv11b_userd_gp_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));

	return nvgpu_mem_rd32(g, userd_mem,
			offset + ram_userd_gp_get_w());
}

u64 gv11b_userd_pb_get(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));
	u32 lo = nvgpu_mem_rd32(g, userd_mem, offset + ram_userd_get_w());
	u32 hi = nvgpu_mem_rd32(g, userd_mem, offset + ram_userd_get_hi_w());

	return ((u64)hi << 32) | lo;
}

void gv11b_userd_gp_put(struct gk20a *g, struct channel_gk20a *c)
{
	struct nvgpu_mem *userd_mem = &g->fifo.userd;
	u32 offset = c->chid * (g->fifo.userd_entry_size / sizeof(u32));

	nvgpu_mem_wr32(g, userd_mem, offset + ram_userd_gp_put_w(),
							c->gpfifo.put);
	/* Commit everything to GPU. */
	nvgpu_mb();

	g->ops.fifo.ring_channel_doorbell(c);
}

void channel_gv11b_unbind(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;

	nvgpu_log_fn(g, " ");

	if (nvgpu_atomic_cmpxchg(&ch->bound, true, false)) {
		gk20a_writel(g, ccsr_channel_inst_r(ch->chid),
			ccsr_channel_inst_ptr_f(0) |
			ccsr_channel_inst_bind_false_f());

		gk20a_writel(g, ccsr_channel_r(ch->chid),
			ccsr_channel_enable_clr_true_f() |
			ccsr_channel_pbdma_faulted_reset_f() |
			ccsr_channel_eng_faulted_reset_f());
	}
}

u32 gv11b_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

bool gv11b_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid)
{
	return (engine_subid == gmmu_fault_client_type_gpc_v());
}

void gv11b_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 chid,
				     struct ch_state *ch_state)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));
	u32 status = ccsr_channel_status_v(channel);
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

	gk20a_debug_output(o, "%d-%s, pid %d, refs: %d: ", chid,
			g->name,
			ch_state->pid,
			ch_state->refs);
	gk20a_debug_output(o, "channel status: %s in use %s %s\n",
			ccsr_channel_enable_v(channel) ? "" : "not",
			gk20a_decode_ccsr_chan_status(status),
			ccsr_channel_busy_v(channel) ? "busy" : "not busy");
	gk20a_debug_output(o, "RAMFC : TOP: %016llx PUT: %016llx GET: %016llx "
			"FETCH: %016llx\nHEADER: %08x COUNT: %08x\n"
			"SEMAPHORE: addr hi: %08x addr lo: %08x\n"
			"payload %08x execute %08x\n",
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
		inst_mem[ram_fc_sem_addr_hi_w()],
		inst_mem[ram_fc_sem_addr_lo_w()],
		inst_mem[ram_fc_sem_payload_lo_w()],
		inst_mem[ram_fc_sem_execute_w()]);
	if (hw_sema) {
		gk20a_debug_output(o, "SEMA STATE: value: 0x%08x next_val: 0x%08x addr: 0x%010llx\n",
				  __nvgpu_semaphore_read(hw_sema),
				  nvgpu_atomic_read(&hw_sema->next_value),
				  nvgpu_hw_sema_addr(hw_sema));
	}
	gk20a_debug_output(o, "\n");
}

void gv11b_dump_eng_status(struct gk20a *g,
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

		if (fifo_engine_status_eng_reload_v(status)) {
			gk20a_debug_output(o, "ctx_reload ");
		}
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

u32 gv11b_fifo_intr_0_error_mask(struct gk20a *g)
{
	u32 intr_0_error_mask =
		fifo_intr_0_bind_error_pending_f() |
		fifo_intr_0_sched_error_pending_f() |
		fifo_intr_0_chsw_error_pending_f() |
		fifo_intr_0_memop_timeout_pending_f() |
		fifo_intr_0_lb_error_pending_f();

	return intr_0_error_mask;
}

u32 gv11b_fifo_get_preempt_timeout(struct gk20a *g)
{
	/* using gr_idle_timeout for polling pdma/eng/runlist
	 * might kick in timeout handler in the cases where
	 * preempt is stuck. Use fifo_eng_timeout converted to ms
	 * for preempt polling */

	return g->fifo_eng_timeout_us / 1000 ;
}

static int gv11b_fifo_poll_pbdma_chan_status(struct gk20a *g, u32 id,
				 u32 pbdma_id)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT; /* in micro seconds */
	u32 pbdma_stat;
	u32 chan_stat;
	int ret = -EBUSY;
	unsigned int loop_count = 0;

	/* timeout in milli seconds */
	nvgpu_timeout_init(g, &timeout, g->ops.fifo.get_preempt_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	nvgpu_log(g, gpu_dbg_info, "wait preempt pbdma %d", pbdma_id);
	/* Verify that ch/tsg is no longer on the pbdma */
	do {
		if (!nvgpu_platform_is_silicon(g)) {
			if (loop_count >= MAX_PRE_SI_RETRIES) {
				nvgpu_err(g, "preempt pbdma retries: %u",
					loop_count);
				break;
			}
			loop_count++;
		}
		/*
		 * If the PBDMA has a stalling interrupt and receives a NACK,
		 * the PBDMA won't save out until the STALLING interrupt is
		 * cleared. Stalling interrupt need not be directly addressed,
		 * as simply clearing of the interrupt bit will be sufficient
		 * to allow the PBDMA to save out. If the stalling interrupt
		 * was due to a SW method or another deterministic failure,
		 * the PBDMA will assert it when the channel is reloaded
		 * or resumed. Note that the fault will still be
		 * reported to SW.
		 */

		gk20a_fifo_handle_pbdma_intr(g, &g->fifo, pbdma_id, RC_NO);

		pbdma_stat = gk20a_readl(g, fifo_pbdma_status_r(pbdma_id));
		chan_stat  = fifo_pbdma_status_chan_status_v(pbdma_stat);

		if (chan_stat ==
			 fifo_pbdma_status_chan_status_valid_v() ||
			chan_stat ==
			 fifo_pbdma_status_chan_status_chsw_save_v()) {

			if (id != fifo_pbdma_status_id_v(pbdma_stat)) {
				ret = 0;
				break;
			}

		} else if (chan_stat ==
			fifo_pbdma_status_chan_status_chsw_load_v()) {

			if (id != fifo_pbdma_status_next_id_v(pbdma_stat)) {
				ret = 0;
				break;
			}

		} else if (chan_stat ==
				fifo_pbdma_status_chan_status_chsw_switch_v()) {

			if ((id != fifo_pbdma_status_next_id_v(pbdma_stat)) &&
				 (id != fifo_pbdma_status_id_v(pbdma_stat))) {
				ret = 0;
				break;
			}
		} else {
			/* pbdma status is invalid i.e. it is not loaded */
			ret = 0;
			break;
		}

		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(unsigned long,
				delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret) {
		nvgpu_err(g, "preempt timeout pbdma: %u pbdma_stat: %u "
				"tsgid: %u", pbdma_id, pbdma_stat, id);
	}
	return ret;
}

static int gv11b_fifo_poll_eng_ctx_status(struct gk20a *g, u32 id,
			 u32 act_eng_id, u32 *reset_eng_bitmask)
{
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT; /* in micro seconds */
	u32 eng_stat;
	u32 ctx_stat;
	int ret = -EBUSY;
	unsigned int loop_count = 0;
	u32 eng_intr_pending;

	/* timeout in milli seconds */
	nvgpu_timeout_init(g, &timeout, g->ops.fifo.get_preempt_timeout(g),
			   NVGPU_TIMER_CPU_TIMER);

	nvgpu_log(g, gpu_dbg_info, "wait preempt act engine id: %u",
			act_eng_id);
	/* Check if ch/tsg has saved off the engine or if ctxsw is hung */
	do {
		if (!nvgpu_platform_is_silicon(g)) {
			if (loop_count >= MAX_PRE_SI_RETRIES) {
				nvgpu_err(g, "preempt eng retries: %u",
					loop_count);
				break;
			}
			loop_count++;
		}
		eng_stat = gk20a_readl(g, fifo_engine_status_r(act_eng_id));
		ctx_stat  = fifo_engine_status_ctx_status_v(eng_stat);

		if (g->ops.mc.is_stall_and_eng_intr_pending(g, act_eng_id,
					&eng_intr_pending)) {
		/* From h/w team
		 * Engine save can be blocked by eng  stalling interrupts.
		 * FIFO interrupts shouldn’t block an engine save from
		 * finishing, but could block FIFO from reporting preempt done.
		 * No immediate reason to reset the engine if FIFO interrupt is
		 * pending.
		 * The hub, priv_ring, and ltc interrupts could block context
		 * switch (or memory), but doesn’t necessarily have to.
		 * For Hub interrupts they just report access counters and page
		 * faults. Neither of these necessarily block context switch
		 * or preemption, but they could.
		 * For example a page fault for graphics would prevent graphics
		 * from saving out. An access counter interrupt is a
		 * notification and has no effect.
		 * SW should handle page faults though for preempt to complete.
		 * PRI interrupt (due to a failed PRI transaction) will result
		 * in ctxsw failure reported to HOST.
		 * LTC interrupts are generally ECC related and if so,
		 * certainly don’t block preemption/ctxsw but they could.
		 * Bus interrupts shouldn’t have anything to do with preemption
		 * state as they are part of the Host EXT pipe, though they may
		 * exhibit a symptom that indicates that GPU is in a bad state.
		 * To be completely fair, when an engine is preempting SW
		 * really should just handle other interrupts as they come in.
		 * It’s generally bad to just poll and wait on a preempt
		 * to complete since there are many things in the GPU which may
		 * cause a system to hang/stop responding.
		 */
			nvgpu_log(g, gpu_dbg_info | gpu_dbg_intr,
					"stall intr set, "
					"preemption might not finish");
		}
		if (ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_switch_v()) {
			/* Eng save hasn't started yet. Continue polling */
			if (eng_intr_pending) {
				/* if eng intr, stop polling */
				*reset_eng_bitmask |= BIT(act_eng_id);
				ret = 0;
				break;
			}

		} else if (ctx_stat ==
			 fifo_engine_status_ctx_status_valid_v() ||
				ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_save_v()) {

			if (id == fifo_engine_status_id_v(eng_stat)) {
				if (eng_intr_pending) {
					/* preemption will not finish */
					*reset_eng_bitmask |= BIT(act_eng_id);
					ret = 0;
					break;
				}
			} else {
				/* context is not running on the engine */
				ret = 0;
				break;
			}

		} else if (ctx_stat ==
			 fifo_engine_status_ctx_status_ctxsw_load_v()) {

			if (id == fifo_engine_status_next_id_v(eng_stat)) {
				if (eng_intr_pending) {
					/* preemption will not finish */
					*reset_eng_bitmask |= BIT(act_eng_id);
					ret = 0;
					break;
				}
			} else {
				/* context is not running on the engine */
				ret = 0;
				break;
			}

		} else {
			/* Preempt should be finished */
			ret = 0;
			break;
		}
		nvgpu_usleep_range(delay, delay * 2);
		delay = min_t(unsigned long,
				delay << 1, GR_IDLE_CHECK_MAX);
	} while (!nvgpu_timeout_expired(&timeout));

	if (ret) {
		/*
		* The reasons a preempt can fail are:
		* 1.Some other stalling interrupt is asserted preventing
		*   channel or context save.
		* 2.The memory system hangs.
		* 3.The engine hangs during CTXSW.
		*/
		nvgpu_err(g, "preempt timeout eng: %u ctx_stat: %u tsgid: %u",
			act_eng_id, ctx_stat, id);
		*reset_eng_bitmask |= BIT(act_eng_id);
	}

	return ret;
}

static void gv11b_reset_eng_faulted_ch(struct gk20a *g, u32 chid)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, ccsr_channel_r(chid));
	reg_val |= ccsr_channel_eng_faulted_reset_f();
	gk20a_writel(g, ccsr_channel_r(chid), reg_val);
}

static void gv11b_reset_eng_faulted_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		gv11b_reset_eng_faulted_ch(g, ch->chid);
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);
}

static void gv11b_reset_pbdma_faulted_ch(struct gk20a *g, u32 chid)
{
	u32 reg_val;

	reg_val = gk20a_readl(g, ccsr_channel_r(chid));
	reg_val |= ccsr_channel_pbdma_faulted_reset_f();
	gk20a_writel(g, ccsr_channel_r(chid), reg_val);
}

static void gv11b_reset_pbdma_faulted_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		gv11b_reset_pbdma_faulted_ch(g, ch->chid);
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);
}

void gv11b_fifo_reset_pbdma_and_eng_faulted(struct gk20a *g,
			struct tsg_gk20a *tsg,
			u32 faulted_pbdma, u32 faulted_engine)
{
	if (!tsg) {
		return;
	}

	nvgpu_log(g, gpu_dbg_intr, "reset faulted pbdma:0x%x eng:0x%x",
				faulted_pbdma, faulted_engine);

	if (faulted_pbdma != FIFO_INVAL_PBDMA_ID) {
		gv11b_reset_pbdma_faulted_tsg(tsg);
	}
	if (faulted_engine != FIFO_INVAL_ENGINE_ID) {
		gv11b_reset_eng_faulted_tsg(tsg);
	}
}

static u32 gv11b_fifo_get_runlists_mask(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault)
{
	u32 runlists_mask = 0;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	u32 rlid, pbdma_bitmask = 0;

	if (id_type != ID_TYPE_UNKNOWN) {
		if (id_type == ID_TYPE_TSG) {
			runlists_mask |= fifo_sched_disable_runlist_m(
						f->tsg[id].runlist_id);
		} else {
			runlists_mask |= fifo_sched_disable_runlist_m(
						f->channel[id].runlist_id);
		}
	}

	if (rc_type == RC_TYPE_MMU_FAULT && mmfault) {
		if (mmfault->faulted_pbdma != FIFO_INVAL_PBDMA_ID) {
			pbdma_bitmask = BIT(mmfault->faulted_pbdma);
		}

		for (rlid = 0; rlid < f->max_runlists; rlid++) {

			runlist = &f->runlist_info[rlid];

			if (runlist->eng_bitmask & act_eng_bitmask) {
				runlists_mask |=
				 fifo_sched_disable_runlist_m(rlid);
			}

			if (runlist->pbdma_bitmask & pbdma_bitmask) {
				runlists_mask |=
				 fifo_sched_disable_runlist_m(rlid);
			}
		}
	}

	if (id_type == ID_TYPE_UNKNOWN) {
		for (rlid = 0; rlid < f->max_runlists; rlid++) {
			if (act_eng_bitmask) {
				/* eng ids are known */
				runlist = &f->runlist_info[rlid];
				if (runlist->eng_bitmask & act_eng_bitmask) {
					runlists_mask |=
					fifo_sched_disable_runlist_m(rlid);
                                }
			} else {
				runlists_mask |=
					fifo_sched_disable_runlist_m(rlid);
			}
		}
	}
	nvgpu_log(g, gpu_dbg_info, "runlists_mask = 0x%08x", runlists_mask);
	return runlists_mask;
}

int gv11b_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next)
{
	/* gv11b allows multiple outstanding preempts,
	   so always preempt next for best reschedule effect */
	return nvgpu_fifo_reschedule_runlist(ch, true, false);
}

static void gv11b_fifo_issue_runlist_preempt(struct gk20a *g,
					 u32 runlists_mask)
{
	u32 reg_val;

	/* issue runlist preempt */
	reg_val = gk20a_readl(g, fifo_runlist_preempt_r());
	reg_val |= runlists_mask;
	gk20a_writel(g, fifo_runlist_preempt_r(), reg_val);
}

int gv11b_fifo_is_preempt_pending(struct gk20a *g, u32 id,
		 unsigned int id_type)
{
	struct fifo_gk20a *f = &g->fifo;
	unsigned long runlist_served_pbdmas;
	unsigned long runlist_served_engines;
	u32 pbdma_id;
	u32 act_eng_id;
	u32 runlist_id;
	int ret = 0;
	u32 tsgid;

	if (id_type == ID_TYPE_TSG) {
		runlist_id = f->tsg[id].runlist_id;
		tsgid = id;
	} else {
		runlist_id = f->channel[id].runlist_id;
		tsgid = f->channel[id].tsgid;
	}

	nvgpu_log_info(g, "Check preempt pending for tsgid = %u", tsgid);

	runlist_served_pbdmas = f->runlist_info[runlist_id].pbdma_bitmask;
	runlist_served_engines = f->runlist_info[runlist_id].eng_bitmask;

	for_each_set_bit(pbdma_id, &runlist_served_pbdmas, f->num_pbdma) {
		ret |= gv11b_fifo_poll_pbdma_chan_status(g, tsgid, pbdma_id);
	}

	f->runlist_info[runlist_id].reset_eng_bitmask = 0;

	for_each_set_bit(act_eng_id, &runlist_served_engines, f->max_engines) {
		ret |= gv11b_fifo_poll_eng_ctx_status(g, tsgid, act_eng_id,
				&f->runlist_info[runlist_id].reset_eng_bitmask);
	}
	return ret;
}

int gv11b_fifo_preempt_channel(struct gk20a *g, struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg = NULL;

	tsg = tsg_gk20a_from_ch(ch);

	if (tsg == NULL) {
		return 0;
	}

	nvgpu_log_info(g, "chid:%d tsgid:%d", ch->chid, tsg->tsgid);

	/* Preempt tsg. Channel preempt is NOOP */
	return g->ops.fifo.preempt_tsg(g, tsg);
}

/* TSG enable sequence applicable for Volta and onwards */
int gv11b_fifo_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;
	struct channel_gk20a *last_ch = NULL;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		g->ops.fifo.enable_channel(ch);
		last_ch = ch;
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	if (last_ch) {
		g->ops.fifo.ring_channel_doorbell(last_ch);
	}

	return 0;
}

int gv11b_fifo_preempt_tsg(struct gk20a *g, struct tsg_gk20a *tsg)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 ret = 0;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 runlist_id;

	nvgpu_log_fn(g, "tsgid: %d", tsg->tsgid);

	runlist_id = tsg->runlist_id;
	nvgpu_log_fn(g, "runlist_id: %d", runlist_id);
	if (runlist_id == FIFO_INVAL_RUNLIST_ID) {
		return 0;
	}

	nvgpu_mutex_acquire(&f->runlist_info[runlist_id].runlist_lock);

	/* WAR for Bug 2065990 */
	gk20a_fifo_disable_tsg_sched(g, tsg);

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	ret = __locked_fifo_preempt(g, tsg->tsgid, true);

	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}

	/* WAR for Bug 2065990 */
	gk20a_fifo_enable_tsg_sched(g, tsg);

	nvgpu_mutex_release(&f->runlist_info[runlist_id].runlist_lock);

	if (ret) {
		if (nvgpu_platform_is_silicon(g)) {
			nvgpu_err(g, "preempt timed out for tsgid: %u, "
			"ctxsw timeout will trigger recovery if needed", tsg->tsgid);
		} else {
			gk20a_fifo_preempt_timeout_rc_tsg(g, tsg);
		}
	}

	return ret;
}

static void gv11b_fifo_locked_preempt_runlists_rc(struct gk20a *g,
						u32 runlists_mask)
{
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	u32 rlid;

	/* runlist_lock are locked by teardown and sched are disabled too */
	nvgpu_log_fn(g, "preempt runlists_mask:0x%08x", runlists_mask);

	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	/* issue runlist preempt */
	gv11b_fifo_issue_runlist_preempt(g, runlists_mask);

	/*
	 * Preemption will never complete in RC due to some fatal condition.
	 * Do not poll for preemption to complete. Reset engines served by
	 * runlists.
	 */
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		if ((runlists_mask &
			fifo_runlist_preempt_runlist_m(rlid)) != 0U) {
			g->fifo.runlist_info[rlid].reset_eng_bitmask =
			g->fifo.runlist_info[rlid].eng_bitmask;
		}
	}

	if (mutex_ret == 0U) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}
}

static void gv11b_fifo_locked_abort_runlist_active_tsgs(struct gk20a *g,
			unsigned int rc_type,
			u32 runlists_mask)
{
	struct tsg_gk20a *tsg = NULL;
	u32 rlid, tsgid;
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 token = PMU_INVALID_MUTEX_OWNER_ID;
	u32 mutex_ret = 0;
	bool add = false, wait_for_finish = false;
	int err;

	nvgpu_err(g, "runlist id unknown, abort active tsgs in runlists");

	/* runlist_lock  are locked by teardown */
	mutex_ret = nvgpu_pmu_mutex_acquire(&g->pmu, PMU_MUTEX_ID_FIFO, &token);

	for (rlid = 0; rlid < g->fifo.max_runlists;
						 rlid++) {
		if (!(runlists_mask & BIT(rlid))) {
			continue;
		}
		nvgpu_log(g, gpu_dbg_info, "abort runlist id %d",
				rlid);
		runlist = &g->fifo.runlist_info[rlid];

		for_each_set_bit(tsgid, runlist->active_tsgs,
			g->fifo.num_channels) {
			nvgpu_log(g, gpu_dbg_info, "abort tsg id %d", tsgid);
			tsg = &g->fifo.tsg[tsgid];
			gk20a_disable_tsg(tsg);

			/* assume all pbdma and eng faulted are set */
			nvgpu_log(g, gpu_dbg_info, "reset pbdma and eng faulted");
			gv11b_reset_pbdma_faulted_tsg(tsg);
			gv11b_reset_eng_faulted_tsg(tsg);

#ifdef CONFIG_GK20A_CTXSW_TRACE
			gk20a_ctxsw_trace_tsg_reset(g, tsg);
#endif
			if (!g->fifo.deferred_reset_pending) {
				if (rc_type == RC_TYPE_MMU_FAULT) {
					gk20a_fifo_set_ctx_mmu_error_tsg(g, tsg);
					gk20a_fifo_error_tsg(g, tsg);
				}
			}

			/* (chid == ~0 && !add) remove all act ch from runlist*/
			err = gk20a_fifo_update_runlist_locked(g, rlid,
					FIFO_INVAL_CHANNEL_ID, add, wait_for_finish);
			if (err) {
				nvgpu_err(g, "runlist id %d is not cleaned up",
					rlid);
			}

			gk20a_fifo_abort_tsg(g, tsg, false);

			nvgpu_log(g, gpu_dbg_info, "aborted tsg id %d", tsgid);
		}
	}
	if (!mutex_ret) {
		nvgpu_pmu_mutex_release(&g->pmu, PMU_MUTEX_ID_FIFO, &token);
	}
}

void gv11b_fifo_teardown_mask_intr(struct gk20a *g)
{
	u32 val;

	/*
	 * ctxsw timeout error prevents recovery, and ctxsw error will retrigger
	 * every 100ms. Disable ctxsw timeout error to allow recovery.
	 */
	val = gk20a_readl(g, fifo_intr_en_0_r());
	val &= ~ fifo_intr_0_ctxsw_timeout_pending_f();
	gk20a_writel(g, fifo_intr_en_0_r(), val);
	gk20a_writel(g, fifo_intr_ctxsw_timeout_r(),
			gk20a_readl(g, fifo_intr_ctxsw_timeout_r()));

}

void gv11b_fifo_teardown_unmask_intr(struct gk20a *g)
{
	u32 val;

	/* enable ctxsw timeout interrupt */
	val = gk20a_readl(g, fifo_intr_en_0_r());
	val |= fifo_intr_0_ctxsw_timeout_pending_f();
	gk20a_writel(g, fifo_intr_en_0_r(), val);
}


void gv11b_fifo_teardown_ch_tsg(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault)
{
	struct tsg_gk20a *tsg = NULL;
	u32 runlists_mask, rlid, pbdma_id;
	struct fifo_runlist_info_gk20a *runlist = NULL;
	u32 engine_id, client_type = ~0;
	struct fifo_gk20a *f = &g->fifo;
	u32 runlist_id = FIFO_INVAL_RUNLIST_ID;
	u32 num_runlists = 0;
	unsigned long runlist_served_pbdmas;

	bool deferred_reset_pending = false;

	nvgpu_log_info(g, "acquire engines_reset_mutex");
	nvgpu_mutex_acquire(&g->fifo.engines_reset_mutex);

	nvgpu_log_fn(g, "acquire runlist_lock for all runlists");
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
		nvgpu_mutex_acquire(&f->runlist_info[rlid].
			runlist_lock);
	}

	g->ops.fifo.teardown_mask_intr(g);

	/* get runlist id and tsg */
	if (id_type == ID_TYPE_TSG) {
		if (id != FIFO_INVAL_TSG_ID) {
			tsg = &g->fifo.tsg[id];
			runlist_id = tsg->runlist_id;
			if (runlist_id != FIFO_INVAL_RUNLIST_ID) {
				num_runlists++;
			} else {
				nvgpu_log_fn(g, "tsg runlist id is invalid");
			}
		} else {
			nvgpu_log_fn(g, "id type is tsg but tsg id is inval");
		}
	} else {
		/*
		 * id type is unknown, get runlist_id if eng mask is such that
		 * it corresponds to single runlist id. If eng mask corresponds
		 * to multiple runlists, then abort all runlists
		 */
		for (rlid = 0; rlid < f->max_runlists; rlid++) {
			if (act_eng_bitmask) {
				/* eng ids are known */
				runlist = &f->runlist_info[rlid];
				if (runlist->eng_bitmask & act_eng_bitmask) {
					runlist_id = rlid;
					num_runlists++;
				}
			} else {
				break;
			}
		}
		if (num_runlists > 1) {
			/* abort all runlists */
			runlist_id = FIFO_INVAL_RUNLIST_ID;
		}
	}

	/* if runlist_id is valid and there is only single runlist to be
	 * aborted, release runlist lock that are not
	 * needed for this recovery
	 */
	if (runlist_id != FIFO_INVAL_RUNLIST_ID && num_runlists == 1) {
		for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
			if (rlid != runlist_id) {
				nvgpu_log_fn(g, "release runlist_lock for "
					"unused runlist id: %d", rlid);
				nvgpu_mutex_release(&f->runlist_info[rlid].
					runlist_lock);
			}
		}
	}

	nvgpu_log(g, gpu_dbg_info, "id = %d, id_type = %d, rc_type = %d, "
			"act_eng_bitmask = 0x%x, mmfault ptr = 0x%p",
			 id, id_type, rc_type, act_eng_bitmask, mmfault);

	runlists_mask =  gv11b_fifo_get_runlists_mask(g, act_eng_bitmask, id,
					 id_type, rc_type, mmfault);

	/* Disable runlist scheduler */
	gk20a_fifo_set_runlist_state(g, runlists_mask, RUNLIST_DISABLED);

	/* Disable power management */
	if (g->support_pmu) {
		if (nvgpu_cg_pg_disable(g) != 0) {
			nvgpu_warn(g, "fail to disable power mgmt");
		}
	}

	if (rc_type == RC_TYPE_MMU_FAULT) {
		gk20a_debug_dump(g);
		client_type = mmfault->client_type;
		gv11b_fifo_reset_pbdma_and_eng_faulted(g, tsg,
				mmfault->faulted_pbdma,
				mmfault->faulted_engine);
	}

	if (tsg) {
		gk20a_disable_tsg(tsg);
	}

	/*
	 * Even though TSG preempt timed out, the RC sequence would by design
	 * require s/w to issue another preempt.
	 * If recovery includes an ENGINE_RESET, to not have race conditions,
	 * use RUNLIST_PREEMPT to kick all work off, and cancel any context
	 * load which may be pending. This is also needed to make sure
	 * that all PBDMAs serving the engine are not loaded when engine is
	 * reset.
	 */
	gv11b_fifo_locked_preempt_runlists_rc(g, runlists_mask);
	/*
	 * For each PBDMA which serves the runlist, poll to verify the TSG is no
	 * longer on the PBDMA and the engine phase of the preempt has started.
	 */
	if (tsg != NULL) {
		rlid = f->tsg[id].runlist_id;
		runlist_served_pbdmas = f->runlist_info[rlid].pbdma_bitmask;
		for_each_set_bit(pbdma_id, &runlist_served_pbdmas,
			f->num_pbdma) {
			/*
			 * If pbdma preempt fails the only option is to reset
			 * GPU. Any sort of hang indicates the entire GPU’s
			 * memory system would be blocked.
			 */
			gv11b_fifo_poll_pbdma_chan_status(g, id, pbdma_id);
		}
	}

	nvgpu_mutex_acquire(&f->deferred_reset_mutex);
	g->fifo.deferred_reset_pending = false;
	nvgpu_mutex_release(&f->deferred_reset_mutex);

	/* check if engine reset should be deferred */
	for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {

		runlist = &g->fifo.runlist_info[rlid];
		if ((runlists_mask & BIT(rlid)) &&
					runlist->reset_eng_bitmask) {

			unsigned long __reset_eng_bitmask =
				 runlist->reset_eng_bitmask;

			for_each_set_bit(engine_id, &__reset_eng_bitmask,
							g->fifo.max_engines) {
				if (tsg &&
					 gk20a_fifo_should_defer_engine_reset(g,
					engine_id, client_type, false)) {

					g->fifo.deferred_fault_engines |=
							 BIT(engine_id);

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
		}
	}

#ifdef CONFIG_GK20A_CTXSW_TRACE
	if (tsg)
		gk20a_ctxsw_trace_tsg_reset(g, tsg);
#endif
	if (tsg) {
		if (deferred_reset_pending) {
			gk20a_disable_tsg(tsg);
		} else {
			if (rc_type == RC_TYPE_MMU_FAULT) {
				gk20a_fifo_set_ctx_mmu_error_tsg(g, tsg);
			}
			(void)gk20a_fifo_error_tsg(g, tsg);
			gk20a_fifo_abort_tsg(g, tsg, false);
		}
	} else {
		gv11b_fifo_locked_abort_runlist_active_tsgs(g, rc_type,
			runlists_mask);
	}

	gk20a_fifo_set_runlist_state(g, runlists_mask, RUNLIST_ENABLED);

	/* It is safe to enable ELPG again. */
	if (g->support_pmu) {
		if (nvgpu_cg_pg_enable(g) != 0) {
			nvgpu_warn(g, "fail to enable power mgmt");
		}
	}

	g->ops.fifo.teardown_unmask_intr(g);

	/* release runlist_lock */
	if (runlist_id != FIFO_INVAL_RUNLIST_ID) {
		nvgpu_log_fn(g, "release runlist_lock runlist_id = %d",
				runlist_id);
		nvgpu_mutex_release(&f->runlist_info[runlist_id].runlist_lock);
	} else {
		nvgpu_log_fn(g, "release runlist_lock for all runlists");
		for (rlid = 0; rlid < g->fifo.max_runlists; rlid++) {
			nvgpu_mutex_release(&f->runlist_info[rlid].
				runlist_lock);
		}
	}

	nvgpu_log_info(g, "release engines_reset_mutex");
	nvgpu_mutex_release(&g->fifo.engines_reset_mutex);
}

void gv11b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
{
	/*
	 * These are all errors which indicate something really wrong
	 * going on in the device
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
		pbdma_intr_0_clear_faulted_error_pending_f() |
		pbdma_intr_0_eng_reset_pending_f() |
		pbdma_intr_0_semaphore_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

static u32 gv11b_fifo_intr_0_en_mask(struct gk20a *g)
{
	u32 intr_0_en_mask;

	intr_0_en_mask = g->ops.fifo.intr_0_error_mask(g);

	intr_0_en_mask |= fifo_intr_0_pbdma_intr_pending_f() |
				 fifo_intr_0_ctxsw_timeout_pending_f();

	return intr_0_en_mask;
}

int gv11b_init_fifo_reset_enable_hw(struct gk20a *g)
{
	u32 intr_stall;
	u32 mask;
	u32 timeout;
	unsigned int i;
	u32 host_num_pbdma = nvgpu_get_litter_value(g, GPU_LIT_HOST_NUM_PBDMA);

	nvgpu_log_fn(g, " ");

	/* enable pmc pfifo */
	g->ops.mc.reset(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_FIFO));

	nvgpu_cg_slcg_ce2_load_enable(g);

	nvgpu_cg_slcg_fifo_load_enable(g);

	nvgpu_cg_blcg_fifo_load_enable(g);

	timeout = gk20a_readl(g, fifo_fb_timeout_r());
	nvgpu_log_info(g, "fifo_fb_timeout reg val = 0x%08x", timeout);
	if (!nvgpu_platform_is_silicon(g)) {
		timeout = set_field(timeout, fifo_fb_timeout_period_m(),
					fifo_fb_timeout_period_max_f());
		timeout = set_field(timeout, fifo_fb_timeout_detection_m(),
					fifo_fb_timeout_detection_disabled_f());
		nvgpu_log_info(g, "new fifo_fb_timeout reg val = 0x%08x",
					timeout);
		gk20a_writel(g, fifo_fb_timeout_r(), timeout);
	}

	for (i = 0; i < host_num_pbdma; i++) {
		timeout = gk20a_readl(g, pbdma_timeout_r(i));
		nvgpu_log_info(g, "pbdma_timeout reg val = 0x%08x",
						 timeout);
		if (!nvgpu_platform_is_silicon(g)) {
			timeout = set_field(timeout, pbdma_timeout_period_m(),
					pbdma_timeout_period_max_f());
			nvgpu_log_info(g, "new pbdma_timeout reg val = 0x%08x",
						 timeout);
			gk20a_writel(g, pbdma_timeout_r(i), timeout);
		}
	}

	/* clear and enable pbdma interrupt */
	for (i = 0; i < host_num_pbdma; i++) {
		gk20a_writel(g, pbdma_intr_0_r(i), 0xFFFFFFFF);
		gk20a_writel(g, pbdma_intr_1_r(i), 0xFFFFFFFF);

		intr_stall = gk20a_readl(g, pbdma_intr_stall_r(i));
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

	/* clear ctxsw timeout interrupts */
	gk20a_writel(g, fifo_intr_ctxsw_timeout_r(), ~0);

	if (nvgpu_platform_is_silicon(g)) {
		/* enable ctxsw timeout */
		timeout = g->fifo_eng_timeout_us;
		timeout = scale_ptimer(timeout,
			ptimer_scalingfactor10x(g->ptimer_src_freq));
		timeout |= fifo_eng_ctxsw_timeout_detection_enabled_f();
		gk20a_writel(g, fifo_eng_ctxsw_timeout_r(), timeout);
	} else {
		timeout = gk20a_readl(g, fifo_eng_ctxsw_timeout_r());
		nvgpu_log_info(g, "fifo_eng_ctxsw_timeout reg val = 0x%08x",
						 timeout);
		timeout = set_field(timeout, fifo_eng_ctxsw_timeout_period_m(),
				fifo_eng_ctxsw_timeout_period_max_f());
		timeout = set_field(timeout,
				fifo_eng_ctxsw_timeout_detection_m(),
				fifo_eng_ctxsw_timeout_detection_disabled_f());
		nvgpu_log_info(g, "new fifo_eng_ctxsw_timeout reg val = 0x%08x",
						 timeout);
		gk20a_writel(g, fifo_eng_ctxsw_timeout_r(), timeout);
	}

	/* clear runlist interrupts */
	gk20a_writel(g, fifo_intr_runlist_r(), ~0);

	/* clear and enable pfifo interrupt */
	gk20a_writel(g, fifo_intr_0_r(), 0xFFFFFFFF);
	mask = gv11b_fifo_intr_0_en_mask(g);
	nvgpu_log_info(g, "fifo_intr_en_0 0x%08x", mask);
	gk20a_writel(g, fifo_intr_en_0_r(), mask);
	nvgpu_log_info(g, "fifo_intr_en_1 = 0x80000000");
	gk20a_writel(g, fifo_intr_en_1_r(), 0x80000000);

	nvgpu_log_fn(g, "done");

	return 0;
}

static const char *const gv11b_sched_error_str[] = {
	"xxx-0",
	"xxx-1",
	"xxx-2",
	"xxx-3",
	"xxx-4",
	"engine_reset",
	"rl_ack_timeout",
	"rl_ack_extra",
	"rl_rdat_timeout",
	"rl_rdat_extra",
	"eng_ctxsw_timeout",
	"xxx-b",
	"rl_req_timeout",
	"new_runlist",
	"code_config_while_busy",
	"xxx-f",
	"xxx-0x10",
	"xxx-0x11",
	"xxx-0x12",
	"xxx-0x13",
	"xxx-0x14",
	"xxx-0x15",
	"xxx-0x16",
	"xxx-0x17",
	"xxx-0x18",
	"xxx-0x19",
	"xxx-0x1a",
	"xxx-0x1b",
	"xxx-0x1c",
	"xxx-0x1d",
	"xxx-0x1e",
	"xxx-0x1f",
	"bad_tsg",
};

bool gv11b_fifo_handle_sched_error(struct gk20a *g)
{
	u32 sched_error;

	sched_error = gk20a_readl(g, fifo_intr_sched_error_r());

	if (sched_error < ARRAY_SIZE(gv11b_sched_error_str)) {
		nvgpu_err(g, "fifo sched error :%s",
			gv11b_sched_error_str[sched_error]);
	} else {
		nvgpu_err(g, "fifo sched error code not supported");
	}

	if (sched_error == SCHED_ERROR_CODE_BAD_TSG ) {
		/* id is unknown, preempt all runlists and do recovery */
		gk20a_fifo_recover(g, 0, 0, false, false, false,
				RC_TYPE_SCHED_ERR);
	}

	return false;
}

static const char * const invalid_str = "invalid";

static const char *const ctxsw_timeout_status_desc[] = {
	"awaiting ack",
	"eng was reset",
	"ack received",
	"dropped timeout"
};

static u32 gv11b_fifo_ctxsw_timeout_info(struct gk20a *g, u32 active_eng_id,
						u32 *info_status)
{
	u32 tsgid = FIFO_INVAL_TSG_ID;
	u32 timeout_info;
	u32 ctx_status;

	timeout_info = gk20a_readl(g,
			 fifo_intr_ctxsw_timeout_info_r(active_eng_id));

	/*
	 * ctxsw_state and tsgid are snapped at the point of the timeout and
	 * will not change while the corresponding INTR_CTXSW_TIMEOUT_ENGINE bit
	 * is PENDING.
	 */
	ctx_status = fifo_intr_ctxsw_timeout_info_ctxsw_state_v(timeout_info);
	if (ctx_status ==
		fifo_intr_ctxsw_timeout_info_ctxsw_state_load_v()) {

		tsgid = fifo_intr_ctxsw_timeout_info_next_tsgid_v(timeout_info);

	} else if (ctx_status ==
		       fifo_intr_ctxsw_timeout_info_ctxsw_state_switch_v() ||
			ctx_status ==
			fifo_intr_ctxsw_timeout_info_ctxsw_state_save_v()) {

		tsgid = fifo_intr_ctxsw_timeout_info_prev_tsgid_v(timeout_info);
	}
	nvgpu_log_info(g, "ctxsw timeout info: tsgid = %d", tsgid);

	/*
	 * STATUS indicates whether the context request ack was eventually
	 * received and whether a subsequent request timed out.  This field is
	 * updated live while the corresponding INTR_CTXSW_TIMEOUT_ENGINE bit
	 * is PENDING. STATUS starts in AWAITING_ACK, and progresses to
	 * ACK_RECEIVED and finally ends with DROPPED_TIMEOUT.
	 *
	 * AWAITING_ACK - context request ack still not returned from engine.
	 * ENG_WAS_RESET - The engine was reset via a PRI write to NV_PMC_ENABLE
	 * or NV_PMC_ELPG_ENABLE prior to receiving the ack.  Host will not
	 * expect ctx ack to return, but if it is already in flight, STATUS will
	 * transition shortly to ACK_RECEIVED unless the interrupt is cleared
	 * first.  Once the engine is reset, additional context switches can
	 * occur; if one times out, STATUS will transition to DROPPED_TIMEOUT
	 * if the interrupt isn't cleared first.
	 * ACK_RECEIVED - The ack for the timed-out context request was
	 * received between the point of the timeout and this register being
	 * read.  Note this STATUS can be reported during the load stage of the
	 * same context switch that timed out if the timeout occurred during the
	 * save half of a context switch.  Additional context requests may have
	 * completed or may be outstanding, but no further context timeout has
	 * occurred.  This simplifies checking for spurious context switch
	 * timeouts.
	 * DROPPED_TIMEOUT - The originally timed-out context request acked,
	 * but a subsequent context request then timed out.
	 * Information about the subsequent timeout is not stored; in fact, that
	 * context request may also have already been acked by the time SW
	 * SW reads this register.  If not, there is a chance SW can get the
	 * dropped information by clearing the corresponding
	 * INTR_CTXSW_TIMEOUT_ENGINE bit and waiting for the timeout to occur
	 * again. Note, however, that if the engine does time out again,
	 * it may not be from the  original request that caused the
	 * DROPPED_TIMEOUT state, as that request may
	 * be acked in the interim.
	 */
	*info_status = fifo_intr_ctxsw_timeout_info_status_v(timeout_info);
	if (*info_status ==
		 fifo_intr_ctxsw_timeout_info_status_ack_received_v()) {

		nvgpu_log_info(g, "ctxsw timeout info : ack received");
		/* no need to recover */
		tsgid = FIFO_INVAL_TSG_ID;

	} else if (*info_status ==
		fifo_intr_ctxsw_timeout_info_status_dropped_timeout_v()) {

		nvgpu_log_info(g, "ctxsw timeout info : dropped timeout");
		/* no need to recover */
		tsgid = FIFO_INVAL_TSG_ID;

	}
	return tsgid;
}

bool gv11b_fifo_handle_ctxsw_timeout(struct gk20a *g, u32 fifo_intr)
{
	bool ret = false;
	u32 tsgid = FIFO_INVAL_TSG_ID;
	u32 engine_id, active_eng_id;
	u32 timeout_val, ctxsw_timeout_engines;
	u32 info_status;
	const char *info_status_str;


	if (!(fifo_intr & fifo_intr_0_ctxsw_timeout_pending_f())) {
		return ret;
	}

	/* get ctxsw timedout engines */
	ctxsw_timeout_engines = gk20a_readl(g, fifo_intr_ctxsw_timeout_r());
	if (ctxsw_timeout_engines == 0) {
		nvgpu_err(g, "no eng ctxsw timeout pending");
		return ret;
	}

	timeout_val = gk20a_readl(g, fifo_eng_ctxsw_timeout_r());
	timeout_val = fifo_eng_ctxsw_timeout_period_v(timeout_val);

	nvgpu_log_info(g, "eng ctxsw timeout period = 0x%x", timeout_val);

	for (engine_id = 0; engine_id < g->fifo.num_engines; engine_id++) {
		active_eng_id = g->fifo.active_engines_list[engine_id];

		if (ctxsw_timeout_engines &
			fifo_intr_ctxsw_timeout_engine_pending_f(
				active_eng_id)) {

			struct fifo_gk20a *f = &g->fifo;
			u32 ms = 0;
			bool verbose = false;

			tsgid = gv11b_fifo_ctxsw_timeout_info(g, active_eng_id,
						&info_status);

			if (tsgid == FIFO_INVAL_TSG_ID) {
				continue;
			}

			if (g->ops.fifo.check_tsg_ctxsw_timeout(
				&f->tsg[tsgid], &verbose, &ms)) {
				ret = true;

				info_status_str =  invalid_str;
				if (info_status <
					ARRAY_SIZE(ctxsw_timeout_status_desc)) {
					info_status_str =
					ctxsw_timeout_status_desc[info_status];
				}

				nvgpu_err(g, "ctxsw timeout error: "
				"active engine id =%u, %s=%d, info: %s ms=%u",
				active_eng_id, "tsg", tsgid, info_status_str,
				ms);

				/* Cancel all channels' timeout */
				gk20a_channel_timeout_restart_all_channels(g);
				gk20a_fifo_recover(g, BIT(active_eng_id), tsgid,
						true, true, verbose,
						RC_TYPE_CTXSW_TIMEOUT);
			} else {
				nvgpu_log_info(g,
					"fifo is waiting for ctx switch: "
					"for %d ms, %s=%d", ms, "tsg", tsgid);
			}
		}
	}
	/* clear interrupt */
	gk20a_writel(g, fifo_intr_ctxsw_timeout_r(), ctxsw_timeout_engines);
	return ret;
}

unsigned int gv11b_fifo_handle_pbdma_intr_0(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_0,
			u32 *handled, u32 *error_notifier)
{
	unsigned int rc_type = RC_TYPE_NO_RC;

	rc_type = gk20a_fifo_handle_pbdma_intr_0(g, pbdma_id,
			 pbdma_intr_0, handled, error_notifier);

	if (pbdma_intr_0 & pbdma_intr_0_clear_faulted_error_pending_f()) {
		nvgpu_log(g, gpu_dbg_intr, "clear faulted error on pbdma id %d",
				 pbdma_id);
		gk20a_fifo_reset_pbdma_method(g, pbdma_id, 0);
		*handled |= pbdma_intr_0_clear_faulted_error_pending_f();
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	if (pbdma_intr_0 & pbdma_intr_0_eng_reset_pending_f()) {
		nvgpu_log(g, gpu_dbg_intr, "eng reset intr on pbdma id %d",
				 pbdma_id);
		*handled |= pbdma_intr_0_eng_reset_pending_f();
		rc_type = RC_TYPE_PBDMA_FAULT;
	}

	return rc_type;
}

/*
 * Pbdma which encountered the ctxnotvalid interrupt will stall and
 * prevent the channel which was loaded at the time the interrupt fired
 * from being swapped out until the interrupt is cleared.
 * CTXNOTVALID pbdma interrupt indicates error conditions related
 * to the *_CTX_VALID fields for a channel.  The following
 * conditions trigger the interrupt:
 * * CTX_VALID bit for the targeted engine is FALSE
 * * At channel start/resume, all preemptible eng have CTX_VALID FALSE but:
 *       - CTX_RELOAD is set in CCSR_CHANNEL_STATUS,
 *       - PBDMA_TARGET_SHOULD_SEND_HOST_TSG_EVENT is TRUE, or
 *       - PBDMA_TARGET_NEEDS_HOST_TSG_EVENT is TRUE
 * The field is left NOT_PENDING and the interrupt is not raised if the PBDMA is
 * currently halted.  This allows SW to unblock the PBDMA and recover.
 * SW may read METHOD0, CHANNEL_STATUS and TARGET to determine whether the
 * interrupt was due to an engine method, CTX_RELOAD, SHOULD_SEND_HOST_TSG_EVENT
 * or NEEDS_HOST_TSG_EVENT.  If METHOD0 VALID is TRUE, lazy context creation
 * can be used or the TSG may be destroyed.
 * If METHOD0 VALID is FALSE, the error is likely a bug in SW, and the TSG
 * will have to be destroyed.
 */

unsigned int gv11b_fifo_handle_pbdma_intr_1(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_1,
			u32 *handled, u32 *error_notifier)
{
	unsigned int rc_type = RC_TYPE_PBDMA_FAULT;
	u32 pbdma_intr_1_current = gk20a_readl(g, pbdma_intr_1_r(pbdma_id));

	/* minimize race with the gpu clearing the pending interrupt */
	if (!(pbdma_intr_1_current &
			pbdma_intr_1_ctxnotvalid_pending_f())) {
		pbdma_intr_1 &= ~pbdma_intr_1_ctxnotvalid_pending_f();
	}

	if (pbdma_intr_1 == 0) {
		return RC_TYPE_NO_RC;
	}

	if (pbdma_intr_1 & pbdma_intr_1_ctxnotvalid_pending_f()) {
		nvgpu_log(g, gpu_dbg_intr, "ctxnotvalid intr on pbdma id %d",
				 pbdma_id);
		nvgpu_err(g, "pbdma_intr_1(%d)= 0x%08x ",
				pbdma_id, pbdma_intr_1);
		*handled |= pbdma_intr_1_ctxnotvalid_pending_f();
	} else{
		/*
		 * rest of the interrupts in _intr_1 are "host copy engine"
		 * related, which is not supported. For now just make them
		 * channel fatal.
		 */
		nvgpu_err(g, "hce err: pbdma_intr_1(%d):0x%08x",
			pbdma_id, pbdma_intr_1);
		*handled |= pbdma_intr_1;
	}

	return rc_type;
}

void gv11b_fifo_init_ramfc_eng_method_buffer(struct gk20a *g,
			struct channel_gk20a *ch, struct nvgpu_mem *mem)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_mem *method_buffer_per_runque;

	tsg = tsg_gk20a_from_ch(ch);
	if (tsg == NULL) {
		nvgpu_err(g, "channel is not part of tsg");
		return;
	}
	if (tsg->eng_method_buffers == NULL) {
		nvgpu_log_info(g, "eng method buffer NULL");
		return;
	}
	if (tsg->runlist_id == gk20a_fifo_get_fast_ce_runlist_id(g)) {
		method_buffer_per_runque =
			&tsg->eng_method_buffers[ASYNC_CE_RUNQUE];
	} else {
		method_buffer_per_runque =
			&tsg->eng_method_buffers[GR_RUNQUE];
	}

	nvgpu_mem_wr32(g, mem, ram_in_eng_method_buffer_addr_lo_w(),
			u64_lo32(method_buffer_per_runque->gpu_va));
	nvgpu_mem_wr32(g, mem, ram_in_eng_method_buffer_addr_hi_w(),
			u64_hi32(method_buffer_per_runque->gpu_va));

	nvgpu_log_info(g, "init ramfc with method buffer");
}

static unsigned int gv11b_fifo_get_eng_method_buffer_size(struct gk20a *g)
{
	unsigned int buffer_size;

	buffer_size =  ((9 + 1 + 3) * g->ops.ce2.get_num_pce(g)) + 2;
	buffer_size = (27 * 5 * buffer_size);
	buffer_size = roundup(buffer_size, PAGE_SIZE);
	nvgpu_log_info(g, "method buffer size in bytes %d", buffer_size);

	return buffer_size;
}

void gv11b_fifo_init_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;
	int err = 0;
	int i;
	unsigned int runque, method_buffer_size;
	unsigned int num_pbdma = g->fifo.num_pbdma;

	if (tsg->eng_method_buffers != NULL) {
		return;
	}

	method_buffer_size = gv11b_fifo_get_eng_method_buffer_size(g);
	if (method_buffer_size == 0) {
		nvgpu_info(g, "ce will hit MTHD_BUFFER_FAULT");
		return;
	}

	tsg->eng_method_buffers = nvgpu_kzalloc(g,
					num_pbdma * sizeof(struct nvgpu_mem));

	for (runque = 0; runque < num_pbdma; runque++) {
		err = nvgpu_dma_alloc_map_sys(vm, method_buffer_size,
					&tsg->eng_method_buffers[runque]);
		if (err) {
			break;
		}
	}
	if (err) {
		for (i = (runque - 1); i >= 0; i--) {
			nvgpu_dma_unmap_free(vm,
				 &tsg->eng_method_buffers[i]);
		}

		nvgpu_kfree(g, tsg->eng_method_buffers);
		tsg->eng_method_buffers = NULL;
		nvgpu_err(g, "could not alloc eng method buffers");
		return;
	}
	nvgpu_log_info(g, "eng method buffers allocated");

}

void gv11b_fifo_deinit_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg)
{
	struct vm_gk20a *vm = g->mm.bar2.vm;
	unsigned int runque;

	if (tsg->eng_method_buffers == NULL) {
		return;
	}

	for (runque = 0; runque < g->fifo.num_pbdma; runque++) {
		nvgpu_dma_unmap_free(vm, &tsg->eng_method_buffers[runque]);
	}

	nvgpu_kfree(g, tsg->eng_method_buffers);
	tsg->eng_method_buffers = NULL;

	nvgpu_log_info(g, "eng method buffers de-allocated");
}

u32 gv11b_fifo_get_sema_wait_cmd_size(void)
{
	return 10;
}

u32 gv11b_fifo_get_sema_incr_cmd_size(void)
{
	return 12;
}

void gv11b_fifo_add_sema_cmd(struct gk20a *g,
	struct nvgpu_semaphore *s, u64 sema_va,
	struct priv_cmd_entry *cmd,
	u32 off, bool acquire, bool wfi)
{
	nvgpu_log_fn(g, " ");

	/* sema_addr_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010017);
	nvgpu_mem_wr32(g, cmd->mem, off++, sema_va & 0xffffffff);

	/* sema_addr_hi */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010018);
	nvgpu_mem_wr32(g, cmd->mem, off++, (sema_va >> 32) & 0xff);

	/* payload_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010019);
	nvgpu_mem_wr32(g, cmd->mem, off++, nvgpu_semaphore_get_value(s));

	/* payload_hi : ignored */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001a);
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);

	if (acquire) {
		/* sema_execute : acq_strict_geq | switch_en | 32bit */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001b);
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x2 | (1 << 12));
	} else {
		/* sema_execute : release | wfi | 32bit */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001b);
		nvgpu_mem_wr32(g, cmd->mem, off++,
			0x1 | ((wfi ? 0x1 : 0x0) << 20));

		/* non_stall_int : payload is ignored */
		nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010008);
		nvgpu_mem_wr32(g, cmd->mem, off++, 0);
	}
}

#ifdef CONFIG_TEGRA_GK20A_NVHOST
static int set_syncpt_ro_map_gpu_va_locked(struct vm_gk20a *vm)
{
	struct gk20a *g = gk20a_from_vm(vm);

	if (vm->syncpt_ro_map_gpu_va)
		return 0;

	vm->syncpt_ro_map_gpu_va = nvgpu_gmmu_map(vm,
			&g->syncpt_mem, g->syncpt_unit_size,
			0, gk20a_mem_flag_read_only,
			false, APERTURE_SYSMEM);

	if (!vm->syncpt_ro_map_gpu_va) {
		nvgpu_err(g, "failed to ro map syncpt buffer");
		return -ENOMEM;
	}

	return 0;
}

int gv11b_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf)
{
	u32 nr_pages;
	int err = 0;
	struct gk20a *g = c->g;

	/*
	 * Add ro map for complete sync point shim range in vm
	 * All channels sharing same vm will share same ro mapping.
	 * Create rw map for current channel sync point
	 */
	nvgpu_mutex_acquire(&c->vm->syncpt_ro_map_lock);
	err = set_syncpt_ro_map_gpu_va_locked(c->vm);
	nvgpu_mutex_release(&c->vm->syncpt_ro_map_lock);
	if (err)
		return err;

	nr_pages = DIV_ROUND_UP(g->syncpt_size, PAGE_SIZE);
	__nvgpu_mem_create_from_phys(g, syncpt_buf,
		(g->syncpt_unit_base +
		nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(syncpt_id)),
		nr_pages);
	syncpt_buf->gpu_va = nvgpu_gmmu_map(c->vm, syncpt_buf,
			g->syncpt_size, 0, gk20a_mem_flag_none,
			false, APERTURE_SYSMEM);

	if (!syncpt_buf->gpu_va) {
		nvgpu_err(g, "failed to map syncpt buffer");
		nvgpu_dma_free(g, syncpt_buf);
		err = -ENOMEM;
	}
	return err;
}

void gv11b_fifo_free_syncpt_buf(struct channel_gk20a *c,
					struct nvgpu_mem *syncpt_buf)
{
	nvgpu_gmmu_unmap(c->vm, syncpt_buf, syncpt_buf->gpu_va);
	nvgpu_dma_free(c->g, syncpt_buf);
}

int gv11b_fifo_get_sync_ro_map(struct vm_gk20a *vm,
	u64 *base_gpuva, u32 *sync_size)
{
	struct gk20a *g = gk20a_from_vm(vm);
	int err;

	nvgpu_mutex_acquire(&vm->syncpt_ro_map_lock);
	err = set_syncpt_ro_map_gpu_va_locked(vm);
	nvgpu_mutex_release(&vm->syncpt_ro_map_lock);
	if (err)
		return err;

	*base_gpuva = vm->syncpt_ro_map_gpu_va;
	*sync_size = g->syncpt_size;

	return 0;
}

void gv11b_fifo_add_syncpt_wait_cmd(struct gk20a *g,
		struct priv_cmd_entry *cmd, u32 off,
		u32 id, u32 thresh, u64 gpu_va_base)
{
	u64 gpu_va = gpu_va_base +
		nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(id);

	nvgpu_log_fn(g, " ");

	off = cmd->off + off;

	/* sema_addr_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010017);
	nvgpu_mem_wr32(g, cmd->mem, off++, gpu_va & 0xffffffff);

	/* sema_addr_hi */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010018);
	nvgpu_mem_wr32(g, cmd->mem, off++, (gpu_va >> 32) & 0xff);

	/* payload_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010019);
	nvgpu_mem_wr32(g, cmd->mem, off++, thresh);

	/* payload_hi : ignored */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001a);
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);

	/* sema_execute : acq_strict_geq | switch_en | 32bit */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001b);
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2 | (1 << 12));
}

u32 gv11b_fifo_get_syncpt_wait_cmd_size(void)
{
	return 10;
}

u32 gv11b_fifo_get_syncpt_incr_per_release(void)
{
	return 1;
}

void gv11b_fifo_add_syncpt_incr_cmd(struct gk20a *g,
		bool wfi_cmd, struct priv_cmd_entry *cmd,
		u32 id, u64 gpu_va)
{
	u32 off = cmd->off;

	nvgpu_log_fn(g, " ");

	/* sema_addr_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010017);
	nvgpu_mem_wr32(g, cmd->mem, off++, gpu_va & 0xffffffff);

	/* sema_addr_hi */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010018);
	nvgpu_mem_wr32(g, cmd->mem, off++, (gpu_va >> 32) & 0xff);

	/* payload_lo */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x20010019);
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);

	/* payload_hi : ignored */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001a);
	nvgpu_mem_wr32(g, cmd->mem, off++, 0);

	/* sema_execute : release | wfi | 32bit */
	nvgpu_mem_wr32(g, cmd->mem, off++, 0x2001001b);
	nvgpu_mem_wr32(g, cmd->mem, off++,
		0x1 | ((wfi_cmd ? 0x1 : 0x0) << 20));
}

u32 gv11b_fifo_get_syncpt_incr_cmd_size(bool wfi_cmd)
{
	return 10;
}
#endif /* CONFIG_TEGRA_GK20A_NVHOST */

int gv11b_init_fifo_setup_hw(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;

	f->max_subctx_count = gr_pri_fe_chip_def_info_max_veid_count_init_v();
	return 0;
}

static u32 gv11b_mmu_fault_id_to_gr_veid(struct gk20a *g, u32 gr_eng_fault_id,
				 u32 mmu_fault_id)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 num_subctx;
	u32 veid = FIFO_INVAL_VEID;

	num_subctx = f->max_subctx_count;

	if (mmu_fault_id >= gr_eng_fault_id &&
			mmu_fault_id < (gr_eng_fault_id + num_subctx)) {
		veid = mmu_fault_id - gr_eng_fault_id;
	}

	return veid;
}

static u32 gv11b_mmu_fault_id_to_eng_id_and_veid(struct gk20a *g,
			 u32 mmu_fault_id, u32 *veid)
{
	u32 engine_id;
	u32 active_engine_id;
	struct fifo_engine_info_gk20a *engine_info;
	struct fifo_gk20a *f = &g->fifo;


	for (engine_id = 0; engine_id < f->num_engines; engine_id++) {
		active_engine_id = f->active_engines_list[engine_id];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if (active_engine_id == ENGINE_GR_GK20A) {
			/* get faulted subctx id */
			*veid = gv11b_mmu_fault_id_to_gr_veid(g,
					engine_info->fault_id, mmu_fault_id);
			if (*veid != FIFO_INVAL_VEID) {
				break;
                        }
		} else {
			if (engine_info->fault_id == mmu_fault_id) {
				break;
			}
		}

		active_engine_id = FIFO_INVAL_ENGINE_ID;
	}
	return active_engine_id;
}

static u32 gv11b_mmu_fault_id_to_pbdma_id(struct gk20a *g, u32 mmu_fault_id)
{
	u32 num_pbdma, reg_val, fault_id_pbdma0;

	reg_val = gk20a_readl(g, fifo_cfg0_r());
	num_pbdma = fifo_cfg0_num_pbdma_v(reg_val);
	fault_id_pbdma0 = fifo_cfg0_pbdma_fault_id_v(reg_val);

	if (mmu_fault_id >= fault_id_pbdma0 &&
			mmu_fault_id <= fault_id_pbdma0 + num_pbdma - 1) {
		return mmu_fault_id - fault_id_pbdma0;
	}

	return FIFO_INVAL_PBDMA_ID;
}

void gv11b_mmu_fault_id_to_eng_pbdma_id_and_veid(struct gk20a *g,
	u32 mmu_fault_id, u32 *active_engine_id, u32 *veid, u32 *pbdma_id)
{
	*active_engine_id = gv11b_mmu_fault_id_to_eng_id_and_veid(g,
				 mmu_fault_id, veid);

	if (*active_engine_id == FIFO_INVAL_ENGINE_ID) {
		*pbdma_id = gv11b_mmu_fault_id_to_pbdma_id(g, mmu_fault_id);
	} else {
		*pbdma_id = FIFO_INVAL_PBDMA_ID;
	}
}

static bool gk20a_fifo_channel_status_is_eng_faulted(struct gk20a *g, u32 chid)
{
	u32 channel = gk20a_readl(g, ccsr_channel_r(chid));

	return ccsr_channel_eng_faulted_v(channel) ==
		ccsr_channel_eng_faulted_true_v();
}

void gv11b_fifo_tsg_verify_status_faulted(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg = &g->fifo.tsg[ch->tsgid];

	/*
	 * If channel has FAULTED set, clear the CE method buffer
	 * if saved out channel is same as faulted channel
	 */
	if (!gk20a_fifo_channel_status_is_eng_faulted(g, ch->chid)) {
		return;
	}

	if (tsg->eng_method_buffers == NULL) {
		return;
	}

	/*
	 * CE method buffer format :
	 * DWord0 = method count
	 * DWord1 = channel id
	 *
	 * It is sufficient to write 0 to method count to invalidate
	 */
	if ((u32)ch->chid ==
	    nvgpu_mem_rd32(g, &tsg->eng_method_buffers[ASYNC_CE_RUNQUE], 1)) {
		nvgpu_mem_wr32(g, &tsg->eng_method_buffers[ASYNC_CE_RUNQUE], 0, 0);
	}
}
