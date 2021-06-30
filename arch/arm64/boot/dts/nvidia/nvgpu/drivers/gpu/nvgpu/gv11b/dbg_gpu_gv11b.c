/*
 * Tegra GV11B GPU Debugger/Profiler Driver
 *
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
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/unit.h>

#include "gv11b/dbg_gpu_gv11b.h"
#include <nvgpu/hw/gv11b/hw_perf_gv11b.h>
#include <nvgpu/bug.h>

static void gv11b_perfbuf_reset_streaming(struct gk20a *g)
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

int gv11b_perfbuf_enable_locked(struct gk20a *g, u64 offset, u32 size)
{
	struct mm_gk20a *mm = &g->mm;
	u32 virt_addr_lo;
	u32 virt_addr_hi;
	u32 inst_pa_page;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to poweron");
		return err;
	}

	err = gk20a_alloc_inst_block(g, &mm->perfbuf.inst_block);
	if (err) {
		return err;
	}

	g->ops.mm.init_inst_block(&mm->perfbuf.inst_block, mm->perfbuf.vm, 0);

	gv11b_perfbuf_reset_streaming(g);

	virt_addr_lo = u64_lo32(offset);
	virt_addr_hi = u64_hi32(offset);

	gk20a_writel(g, perf_pmasys_outbase_r(), virt_addr_lo);
	gk20a_writel(g, perf_pmasys_outbaseupper_r(),
		perf_pmasys_outbaseupper_ptr_f(virt_addr_hi));
	gk20a_writel(g, perf_pmasys_outsize_r(), size);

	/* this field is aligned to 4K */
	inst_pa_page = nvgpu_inst_block_addr(g, &mm->perfbuf.inst_block) >> 12;

	gk20a_writel(g, perf_pmasys_mem_block_r(),
		     perf_pmasys_mem_block_base_f(inst_pa_page) |
		     perf_pmasys_mem_block_valid_true_f() |
		     nvgpu_aperture_mask(g, &mm->perfbuf.inst_block,
				perf_pmasys_mem_block_target_sys_ncoh_f(),
				perf_pmasys_mem_block_target_sys_coh_f(),
				perf_pmasys_mem_block_target_lfb_f()));

	gk20a_idle(g);
	return 0;
}

/* must be called with dbg_sessions_lock held */
int gv11b_perfbuf_disable_locked(struct gk20a *g)
{
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to poweron");
		return err;
	}

	gv11b_perfbuf_reset_streaming(g);

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
