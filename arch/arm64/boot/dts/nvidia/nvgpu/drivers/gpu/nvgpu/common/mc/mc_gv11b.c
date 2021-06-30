/*
 * GV11B master
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>
#include <nvgpu/io.h>
#include <nvgpu/mc.h>
#include <nvgpu/gk20a.h>

#include "mc_gp10b.h"
#include "mc_gv11b.h"

#include <nvgpu/hw/gv11b/hw_mc_gv11b.h>

void mc_gv11b_intr_enable(struct gk20a *g)
{
	u32 eng_intr_mask = gk20a_fifo_engine_interrupt_mask(g);

	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_STALLING),
				0xffffffffU);
	gk20a_writel(g, mc_intr_en_clear_r(NVGPU_MC_INTR_NONSTALLING),
				0xffffffffU);

	g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING] =
				mc_intr_pfifo_pending_f() |
				mc_intr_hub_pending_f() |
				mc_intr_priv_ring_pending_f() |
				mc_intr_pbus_pending_f() |
				mc_intr_ltc_pending_f() |
				eng_intr_mask;

	g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING] =
				mc_intr_pfifo_pending_f()
			     | eng_intr_mask;

	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_STALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_STALLING]);

	gk20a_writel(g, mc_intr_en_set_r(NVGPU_MC_INTR_NONSTALLING),
			g->mc_intr_mask_restore[NVGPU_MC_INTR_NONSTALLING]);

}

bool gv11b_mc_is_intr_hub_pending(struct gk20a *g, u32 mc_intr_0)
{
	return (((mc_intr_0 & mc_intr_hub_pending_f()) != 0U) ? true : false);
}

bool gv11b_mc_is_stall_and_eng_intr_pending(struct gk20a *g, u32 act_eng_id,
			u32 *eng_intr_pending)
{
	u32 mc_intr_0 = gk20a_readl(g, mc_intr_r(0));
	u32 stall_intr, eng_intr_mask;

	eng_intr_mask = gk20a_fifo_act_eng_interrupt_mask(g, act_eng_id);
	*eng_intr_pending = mc_intr_0 & eng_intr_mask;

	stall_intr = mc_intr_pfifo_pending_f() |
			mc_intr_hub_pending_f() |
			mc_intr_priv_ring_pending_f() |
			mc_intr_pbus_pending_f() |
			mc_intr_ltc_pending_f();

	nvgpu_log(g, gpu_dbg_info | gpu_dbg_intr,
		"mc_intr_0 = 0x%08x, eng_intr = 0x%08x",
		mc_intr_0 & stall_intr, *eng_intr_pending);

	return (mc_intr_0 & (eng_intr_mask | stall_intr)) != 0U;
}
