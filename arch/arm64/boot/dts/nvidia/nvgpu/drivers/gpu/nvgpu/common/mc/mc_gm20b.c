/*
 * GM20B Master Control
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/atomic.h>
#include <nvgpu/unit.h>
#include <nvgpu/io.h>
#include <nvgpu/mc.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/bug.h>

#include "mc_gm20b.h"

#include <nvgpu/hw/gm20b/hw_mc_gm20b.h>

void gm20b_mc_isr_stall(struct gk20a *g)
{
	u32 mc_intr_0;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	mc_intr_0 = g->ops.mc.intr_stall(g);

	nvgpu_log(g, gpu_dbg_intr, "stall intr %08x\n", mc_intr_0);

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
}

u32 gm20b_mc_isr_nonstall(struct gk20a *g)
{
	u32 ops = 0;
	u32 mc_intr_1;
	u32 engine_id_idx;
	u32 active_engine_id = 0;
	u32 engine_enum = ENGINE_INVAL_GK20A;

	mc_intr_1 = g->ops.mc.intr_nonstall(g);

	if (g->ops.mc.is_intr1_pending(g, NVGPU_UNIT_FIFO, mc_intr_1) != 0U) {
		ops |= gk20a_fifo_nonstall_isr(g);
	}

	for (engine_id_idx = 0; engine_id_idx < g->fifo.num_engines;
							engine_id_idx++) {
		struct fifo_engine_info_gk20a *engine_info;

		active_engine_id = g->fifo.active_engines_list[engine_id_idx];
		engine_info = &g->fifo.engine_info[active_engine_id];

		if ((mc_intr_1 & engine_info->intr_mask) != 0U) {
			engine_enum = engine_info->engine_enum;
			/* GR Engine */
			if (engine_enum == ENGINE_GR_GK20A) {
				ops |= gk20a_gr_nonstall_isr(g);
			}
			/* CE Engine */
			if (((engine_enum == ENGINE_GRCE_GK20A) ||
			     (engine_enum == ENGINE_ASYNC_CE_GK20A)) &&
			      (g->ops.ce2.isr_nonstall != NULL)) {
				ops |= g->ops.ce2.isr_nonstall(g,
					engine_info->inst_id,
					engine_info->pri_base);
			}
		}
	}

	return ops;
}

void gm20b_mc_intr_mask(struct gk20a *g)
{
	nvgpu_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_disabled_f());
	nvgpu_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_disabled_f());
}

void gm20b_mc_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_mask_1_r(),
		     mc_intr_pfifo_pending_f()
		     | eng_intr_mask);
	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_1_inta_hardware_f());

	gk20a_writel(g, mc_intr_mask_0_r(),
		     mc_intr_pfifo_pending_f()
		     | mc_intr_priv_ring_pending_f()
		     | mc_intr_ltc_pending_f()
		     | mc_intr_pbus_pending_f()
		     | eng_intr_mask);
	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());
}

void gm20b_mc_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask)
{
	u32 mask_reg = (is_stalling ? mc_intr_mask_0_r() :
					mc_intr_mask_1_r());

	if (enable) {
		gk20a_writel(g, mask_reg,
			gk20a_readl(g, mask_reg) |
			mask);
	} else {
		gk20a_writel(g, mask_reg,
			gk20a_readl(g, mask_reg) &
			~mask);
	}
}

void gm20b_mc_intr_stall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_disabled_f());

	/* flush previous write */
	(void) gk20a_readl(g, mc_intr_en_0_r());
}

void gm20b_mc_intr_stall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_0_r(),
		mc_intr_en_0_inta_hardware_f());

	/* flush previous write */
	(void) gk20a_readl(g, mc_intr_en_0_r());
}

void gm20b_mc_intr_nonstall_pause(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_0_inta_disabled_f());

	/* flush previous write */
	(void) gk20a_readl(g, mc_intr_en_1_r());
}

void gm20b_mc_intr_nonstall_resume(struct gk20a *g)
{
	gk20a_writel(g, mc_intr_en_1_r(),
		mc_intr_en_0_inta_hardware_f());

	/* flush previous write */
	(void) gk20a_readl(g, mc_intr_en_1_r());
}

u32 gm20b_mc_intr_stall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_STALLING));
}

u32 gm20b_mc_intr_nonstall(struct gk20a *g)
{
	return gk20a_readl(g, mc_intr_r(NVGPU_MC_INTR_NONSTALLING));
}

void gm20b_mc_disable(struct gk20a *g, u32 units)
{
	u32 pmc;

	nvgpu_log(g, gpu_dbg_info, "pmc disable: %08x\n", units);

	nvgpu_spinlock_acquire(&g->mc_enable_lock);
	pmc = gk20a_readl(g, mc_enable_r());
	pmc &= ~units;
	gk20a_writel(g, mc_enable_r(), pmc);
	nvgpu_spinlock_release(&g->mc_enable_lock);
}

void gm20b_mc_enable(struct gk20a *g, u32 units)
{
	u32 pmc;

	nvgpu_log(g, gpu_dbg_info, "pmc enable: %08x\n", units);

	nvgpu_spinlock_acquire(&g->mc_enable_lock);
	pmc = gk20a_readl(g, mc_enable_r());
	pmc |= units;
	gk20a_writel(g, mc_enable_r(), pmc);
	pmc = gk20a_readl(g, mc_enable_r());
	nvgpu_spinlock_release(&g->mc_enable_lock);

	nvgpu_udelay(20);
}

void gm20b_mc_reset(struct gk20a *g, u32 units)
{
	g->ops.mc.disable(g, units);
	if ((units & gk20a_fifo_get_all_ce_engine_reset_mask(g)) != 0U) {
		nvgpu_udelay(500);
	} else {
		nvgpu_udelay(20);
	}
	g->ops.mc.enable(g, units);
}

bool gm20b_mc_is_intr1_pending(struct gk20a *g,
			       enum nvgpu_unit unit, u32 mc_intr_1)
{
	u32 mask = 0U;
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

void gm20b_mc_log_pending_intrs(struct gk20a *g)
{
	u32 mc_intr_0;
	u32 mc_intr_1;

	mc_intr_0 = g->ops.mc.intr_stall(g);
	if (mc_intr_0 != 0U) {
		if ((mc_intr_0 & mc_intr_priv_ring_pending_f()) != 0U) {
			/* clear priv ring interrupts */
			g->ops.priv_ring.isr(g);
		}
		mc_intr_0 = g->ops.mc.intr_stall(g);
		if (mc_intr_0 != 0U) {
			nvgpu_info(g, "Pending stall intr0=0x%08x", mc_intr_0);
		}
	}

	mc_intr_1 = g->ops.mc.intr_nonstall(g);
	if (mc_intr_1 != 0U) {
		nvgpu_info(g, "Pending nonstall intr1=0x%08x", mc_intr_1);
	}
}

u32 gm20b_mc_reset_mask(struct gk20a *g, enum nvgpu_unit unit)
{
	u32 mask = 0;

	switch(unit) {
	case NVGPU_UNIT_FIFO:
		mask = mc_enable_pfifo_enabled_f();
		break;
	case NVGPU_UNIT_PERFMON:
		mask = mc_enable_perfmon_enabled_f();
		break;
	case NVGPU_UNIT_GRAPH:
		mask = mc_enable_pgraph_enabled_f();
		break;
	case NVGPU_UNIT_BLG:
		mask = mc_enable_blg_enabled_f();
		break;
	case NVGPU_UNIT_PWR:
		mask = mc_enable_pwr_enabled_f();
		break;
	default:
		nvgpu_err(g, "unknown reset unit %d", unit);
		BUG();
		break;
	}

	return mask;
}

bool gm20b_mc_is_enabled(struct gk20a *g, enum nvgpu_unit unit)
{
	u32 mask = g->ops.mc.reset_mask(g, unit);

	return (nvgpu_readl(g, mc_enable_r()) & mask) != 0U;
}

void gm20b_mc_fb_reset(struct gk20a *g)
{
	u32 val;

	nvgpu_log_info(g, "reset gk20a fb");

	val = gk20a_readl(g, mc_elpg_enable_r());
	val |= mc_elpg_enable_xbar_enabled_f()
		| mc_elpg_enable_pfb_enabled_f()
		| mc_elpg_enable_hub_enabled_f();
	gk20a_writel(g, mc_elpg_enable_r(), val);
}
