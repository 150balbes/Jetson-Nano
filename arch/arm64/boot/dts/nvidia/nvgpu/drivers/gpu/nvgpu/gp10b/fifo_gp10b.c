/*
 * GP10B fifo
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

#include <nvgpu/dma.h>
#include <nvgpu/bug.h>
#include <nvgpu/log2.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/channel_sync.h>

#include "fifo_gp10b.h"

#include "gm20b/fifo_gm20b.h"

#include <nvgpu/hw/gp10b/hw_pbdma_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ccsr_gp10b.h>
#include <nvgpu/hw/gp10b/hw_fifo_gp10b.h>
#include <nvgpu/hw/gp10b/hw_ram_gp10b.h>
#include <nvgpu/hw/gp10b/hw_top_gp10b.h>

int channel_gp10b_commit_userd(struct channel_gk20a *c)
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

int channel_gp10b_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long acquire_timeout, u32 flags)
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
		g->ops.fifo.pbdma_acquire_val(acquire_timeout));

	nvgpu_mem_wr32(g, mem, ram_fc_runlist_timeslice_w(),
		pbdma_runlist_timeslice_timeout_128_f() |
		pbdma_runlist_timeslice_timescale_3_f() |
		pbdma_runlist_timeslice_enable_true_f());

	nvgpu_mem_wr32(g, mem, ram_fc_chid_w(), ram_fc_chid_id_f(c->chid));

	if (c->is_privileged_channel) {
		/* Set privilege level for channel */
		nvgpu_mem_wr32(g, mem, ram_fc_config_w(),
			pbdma_config_auth_level_privileged_f());

		gk20a_fifo_setup_ramfc_for_privileged_channel(c);
	}

	return channel_gp10b_commit_userd(c);
}

u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g)
{
	return g->ops.get_litter_value(g, GPU_LIT_GPFIFO_CLASS)
		| pbdma_signature_sw_zero_f();
}

int gp10b_fifo_resetup_ramfc(struct channel_gk20a *c)
{
	u32 new_syncpt = 0, old_syncpt;
	u32 v;
	struct gk20a *g = c->g;

	nvgpu_log_fn(g, " ");

	v = nvgpu_mem_rd32(c->g, &c->inst_block,
			ram_fc_allowed_syncpoints_w());
	old_syncpt = pbdma_allowed_syncpoints_0_index_v(v);
	if (c->sync) {
		new_syncpt = c->sync->syncpt_id(c->sync);
	}

	if (new_syncpt && new_syncpt != old_syncpt) {
		/* disable channel */
		gk20a_disable_channel_tsg(c->g, c);

		/* preempt the channel */
		WARN_ON(gk20a_fifo_preempt(c->g, c));

		v = pbdma_allowed_syncpoints_0_valid_f(1);

		nvgpu_log_info(g, "Channel %d, syncpt id %d\n",
				c->chid, new_syncpt);

		v |= pbdma_allowed_syncpoints_0_index_f(new_syncpt);

		nvgpu_mem_wr32(c->g, &c->inst_block,
				ram_fc_allowed_syncpoints_w(), v);
	}

	/* enable channel */
	gk20a_enable_channel_tsg(c->g, c);

	nvgpu_log_fn(g, "done");

	return 0;
}

int gp10b_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id)
{
	int ret = ENGINE_INVAL_GK20A;

	nvgpu_log_info(g, "engine type %d", engine_type);
	if (engine_type == top_device_info_type_enum_graphics_v()) {
		ret = ENGINE_GR_GK20A;
	} else if (engine_type == top_device_info_type_enum_lce_v()) {
		/* Default assumptions - all the CE engine have separate runlist */
		ret = ENGINE_ASYNC_CE_GK20A;
	}

	return ret;
}

void gp10b_device_info_data_parse(struct gk20a *g, u32 table_entry,
				u32 *inst_id, u32 *pri_base, u32 *fault_id)
{
	if (top_device_info_data_type_v(table_entry) ==
	    top_device_info_data_type_enum2_v()) {
		if (inst_id) {
			*inst_id = top_device_info_data_inst_id_v(table_entry);
		}
		if (pri_base) {
			*pri_base =
			    (top_device_info_data_pri_base_v(table_entry)
			    << top_device_info_data_pri_base_align_v());
			nvgpu_log_info(g, "device info: pri_base: %d", *pri_base);
		}
		if (fault_id && (top_device_info_data_fault_id_v(table_entry) ==
		    top_device_info_data_fault_id_valid_v())) {
			*fault_id =
				 g->ops.fifo.device_info_fault_id(table_entry);
			nvgpu_log_info(g, "device info: fault_id: %d", *fault_id);
		}
	} else {
		nvgpu_err(g, "unknown device_info_data %d",
			top_device_info_data_type_v(table_entry));
	}
}

void gp10b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f)
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
		pbdma_intr_0_syncpoint_illegal_pending_f() |
		pbdma_intr_0_signature_pending_f();

	/* Can be used for sw-methods, or represents a recoverable timeout. */
	f->intr.pbdma.restartable_0 =
		pbdma_intr_0_device_pending_f();
}

void gp10b_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
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
		fifo_intr_mmu_fault_info_access_type_v(fault_info);
	mmfault->client_type =
		fifo_intr_mmu_fault_info_client_type_v(fault_info);
	mmfault->client_id =
		fifo_intr_mmu_fault_info_client_v(fault_info);

	addr_lo = gk20a_readl(g, fifo_intr_mmu_fault_lo_r(mmu_fault_id));
	addr_hi = gk20a_readl(g, fifo_intr_mmu_fault_hi_r(mmu_fault_id));
	mmfault->fault_addr = hi32_lo32_to_u64(addr_hi, addr_lo);
	/* note:ignoring aperture */
	mmfault->inst_ptr = fifo_intr_mmu_fault_inst_ptr_v(
		 gk20a_readl(g, fifo_intr_mmu_fault_inst_r(mmu_fault_id)));
	/* note: inst_ptr is a 40b phys addr.  */
	mmfault->inst_ptr <<= fifo_intr_mmu_fault_inst_ptr_align_shift_v();
}
/* fault info/descriptions */
static const char * const gp10b_fault_type_descs[] = {
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
	 "atomic violation",
};

static const char * const gp10b_hub_client_descs[] = {
	"vip", "ce0", "ce1", "dniso", "fe", "fecs", "host", "host cpu",
	"host cpu nb", "iso", "mmu", "mspdec", "msppp", "msvld",
	"niso", "p2p", "pd", "perf", "pmu", "raster twod", "scc",
	"scc nb", "sec", "ssync", "gr copy", "xv", "mmu nb",
	"msenc", "d falcon", "sked", "a falcon", "n/a",
	"hsce0", "hsce1", "hsce2", "hsce3", "hsce4", "hsce5",
	"hsce6", "hsce7", "hsce8", "hsce9", "hshub",
	"ptp x0", "ptp x1", "ptp x2", "ptp x3", "ptp x4",
	"ptp x5", "ptp x6", "ptp x7", "vpr scrubber0", "vpr scrubber1",
};

/* fill in mmu fault desc */
void gp10b_fifo_get_mmu_fault_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->fault_type >= ARRAY_SIZE(gp10b_fault_type_descs)) {
		WARN_ON(mmfault->fault_type >=
				ARRAY_SIZE(gp10b_fault_type_descs));
	} else {
		mmfault->fault_type_desc =
			 gp10b_fault_type_descs[mmfault->fault_type];
	}
}

/* fill in mmu fault client description */
void gp10b_fifo_get_mmu_fault_client_desc(struct mmu_fault_info *mmfault)
{
	if (mmfault->client_id >= ARRAY_SIZE(gp10b_hub_client_descs)) {
		WARN_ON(mmfault->client_id >=
				ARRAY_SIZE(gp10b_hub_client_descs));
	} else {
		mmfault->client_id_desc =
			 gp10b_hub_client_descs[mmfault->client_id];
	}
}
