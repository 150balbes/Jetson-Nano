/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/log.h>
#include <nvgpu/soc.h>
#include <nvgpu/mm.h>
#include <nvgpu/io.h>
#include <nvgpu/bug.h>
#include <nvgpu/gk20a.h>

#include "bus_gk20a.h"

#include <nvgpu/hw/gk20a/hw_bus_gk20a.h>

void gk20a_bus_init_hw(struct gk20a *g)
{
	u32 intr_en_mask = 0;

	if (nvgpu_platform_is_silicon(g) || nvgpu_platform_is_fpga(g)) {
		intr_en_mask = bus_intr_en_0_pri_squash_m() |
				bus_intr_en_0_pri_fecserr_m() |
				bus_intr_en_0_pri_timeout_m();
	}

	gk20a_writel(g, bus_intr_en_0_r(), intr_en_mask);
}

void gk20a_bus_isr(struct gk20a *g)
{
	u32 val;

	val = gk20a_readl(g, bus_intr_0_r());

	if (val & (bus_intr_0_pri_squash_m() |
			bus_intr_0_pri_fecserr_m() |
			bus_intr_0_pri_timeout_m())) {

		g->ops.ptimer.isr(g);
	} else {
		nvgpu_err(g, "Unhandled NV_PBUS_INTR_0: 0x%08x", val);
	}
	gk20a_writel(g, bus_intr_0_r(), val);
}

u32 gk20a_bus_set_bar0_window(struct gk20a *g, struct nvgpu_mem *mem,
		       struct nvgpu_sgt *sgt, struct nvgpu_sgl *sgl, u32 w)
{
	u64 bufbase = nvgpu_sgt_get_phys(g, sgt, sgl);
	u64 addr = bufbase + w * sizeof(u32);
	u32 hi = (u32)((addr & ~(u64)0xfffff)
		>> bus_bar0_window_target_bar0_window_base_shift_v());
	u32 lo = (u32)(addr & 0xfffff);
	u32 win = nvgpu_aperture_mask(g, mem,
			bus_bar0_window_target_sys_mem_noncoherent_f(),
			bus_bar0_window_target_sys_mem_coherent_f(),
			bus_bar0_window_target_vid_mem_f()) |
		bus_bar0_window_base_f(hi);

	nvgpu_log(g, gpu_dbg_mem,
			"0x%08x:%08x begin for %p,%p at [%llx,%llx] (sz %llx)",
			hi, lo, mem, sgl, bufbase,
			bufbase + nvgpu_sgt_get_phys(g, sgt, sgl),
			nvgpu_sgt_get_length(sgt, sgl));

	WARN_ON(!bufbase);

	if (g->mm.pramin_window != win) {
		gk20a_writel(g, bus_bar0_window_r(), win);
		(void) gk20a_readl(g, bus_bar0_window_r());
		g->mm.pramin_window = win;
	}

	return lo;
}
