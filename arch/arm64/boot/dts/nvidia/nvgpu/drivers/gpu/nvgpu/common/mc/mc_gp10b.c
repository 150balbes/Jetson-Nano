/*
 * GP10B master
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

#include <nvgpu/gk20a.h>
#include <nvgpu/io.h>
#include <nvgpu/mc.h>

#include "mc_gp10b.h"

#include <nvgpu/atomic.h>
#include <nvgpu/unit.h>

#include <nvgpu/hw/gp10b/hw_mc_gp10b.h>

#define MAX_MC_INTR_REGS	2U

void mc_gp10b_intr_mask(struct gk20a *g)
{
	nvgpu_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING),
				0xffffffffU);

	nvgpu_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
				0xffffffffU);
}

void mc_gp10b_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING),
				0xffffffffU);
	g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING] =
				mc_intr_pfifo_pending_f() |
				 mc_intr_priv_ring_pending_f() |
				 mc_intr_pbus_pending_f() |
				 mc_intr_ltc_pending_f() |
				 mc_intr_replayable_fault_pending_f() |
				 eng_intr_mask;
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING]);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
				0xffffffffU);
	g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING] =
				mc_intr_pfifo_pending_f() |
				 eng_intr_mask;
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);
}

void mc_gp10b_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask)
{
	u32 intr_index = 0;
	u32 reg = 0;

	intr_index = (is_stalling ? NVGPU_MC_INTR_STALLING :
			NVGPU_MC_INTR_NONSTALLING);
	if (enable) {
		reg = mc_intr_en_set_r(intr_index);
		g->mc_intr_mask_restore[intr_index] |= mask;

	} else {
		reg = mc_intr_en_clear_r(intr_index);
		g->mc_intr_mask_restore[intr_index] &= ~mask;
	}

	gk20a_writel(g, reg, mask);
}

void mc_gp10b_isr_stall(struct gk20a *g)
{
	u32 mc_intr_0;

	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	mc_intr_0 = gk20a_readl(g, mc_intr_r(0));

	nvgpu_log(g, gpu_dbg_intr, "stall intr 0x%08x\n", mc_intr_0);

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines; engine_id_idx++) {
		active_engine_id = g->fifo.active_engines_list[engine_id_idx];

		if ((mc_intr_0 & g->fifo.engine_info[active_engine_id].intr_mask) != 0U) {
			engine_enum = g->fifo.engine_info[active_engine_id].engine_enum;
			/* GR Engine */
			if (engine_enum == ENGINE_GR_GK20A) {
				gr_gk20a_elpg_protected_call(g, gk20a_gr_isr(g));
			}

			/* CE Engine */
			if (((engine_enum == ENGINE_GRCE_GK20A) ||
				(engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
				(g->ops.ce2.isr_stall != NULL)) {
					g->ops.ce2.isr_stall(g,
					g->fifo.engine_info[active_engine_id].inst_id,
					g->fifo.engine_info[active_engine_id].pri_base);
			}
		}
	}
	if ((g->ops.mc.is_intr_hub_pending != NULL) &&
		 g->ops.mc.is_intr_hub_pending(g, mc_intr_0)) {
		g->ops.fb.hub_isr(g);
	}
	if ((mc_intr_0 & mc_intr_pfifo_pending_f()) != 0U) {
		gk20a_fifo_isr(g);
	}
	if ((mc_intr_0 & mc_intr_pmu_pending_f()) != 0U) {
		g->ops.pmu.pmu_isr(g);
	}
	if ((mc_intr_0 & mc_intr_priv_ring_pending_f()) != 0U) {
		g->ops.priv_ring.isr(g);
	}
	if ((mc_intr_0 & mc_intr_ltc_pending_f()) != 0U) {
		g->ops.ltc.isr(g);
	}
	if ((mc_intr_0 & mc_intr_pbus_pending_f()) != 0U) {
		g->ops.bus.isr(g);
	}
	if ((g->ops.mc.is_intr_nvlink_pending != NULL) &&
			g->ops.mc.is_intr_nvlink_pending(g, mc_intr_0)) {
		g->ops.nvlink.isr(g);
	}
	if (mc_intr_0 & mc_intr_pfb_pending_f() && g->ops.mc.fbpa_isr) {
		g->ops.mc.fbpa_isr(g);
	}

	nvgpu_log(g, gpu_dbg_intr, "stall intr done 0x%08x\n", mc_intr_0);

}

u32 mc_gp10b_intr_stall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_STALLING));
}

void mc_gp10b_intr_stall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING), 0xffffffffU);
}

void mc_gp10b_intr_stall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING]);
}

u32 mc_gp10b_intr_nonstall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_NONSTALLING));
}

void mc_gp10b_intr_nonstall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
		     0xffffffffU);
}

void mc_gp10b_intr_nonstall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);
}

bool mc_gp10b_is_intr1_pending(struct gk20a *g,
				      enum nvgpu_unit unit, u32 mc_intr_1)
{
	u32 mask = 0;
	bool is_pending;

	switch (unit) {
	case NVGPU_UNIT_FIFO:
		mask = mc_intr_pfifo_pending_f();
		break;
	default:
		break;
	}

	if (mask == 0U) {
		nvgpu_err(g, "unknown unit %d", unit);
		is_pending = false;
	} else {
		is_pending = ((mc_intr_1 & mask) != 0U) ? true : false;
	}

	return is_pending;
}

void mc_gp10b_log_pending_intrs(struct gk20a *g)
{
	u32 i, intr;

	for (i = 0; i < MAX_MC_INTR_REGS; i++) {
		intr = nvgpu_readl(g, mc_intr_r(i));
		if (intr == 0U) {
			continue;
		}
		nvgpu_info(g, "Pending intr%d=0x%08x", i, intr);
	}

}
