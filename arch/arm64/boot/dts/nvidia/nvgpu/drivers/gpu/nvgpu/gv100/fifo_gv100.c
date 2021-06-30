/*
 * GV100 fifo
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/ptimer.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "fifo_gv100.h"

#include <nvgpu/hw/gv100/hw_ccsr_gv100.h>
#include <nvgpu/hw/gk20a/hw_fifo_gk20a.h>

#define DEFAULT_FIFO_PREEMPT_TIMEOUT 0x3FFFFFUL

u32 gv100_fifo_get_num_fifos(struct gk20a *g)
{
	return ccsr_channel__size_1_v();
}

u32 gv100_fifo_get_preempt_timeout(struct gk20a *g)
{
	return g->fifo_eng_timeout_us / 1000 ;
}

void gv100_apply_ctxsw_timeout_intr(struct gk20a *g)
{
	u32 timeout;

	timeout = g->ch_wdt_timeout_ms*1000;
	timeout = scale_ptimer(timeout,
		ptimer_scalingfactor10x(g->ptimer_src_freq));
	timeout |= fifo_eng_timeout_detection_enabled_f();
	gk20a_writel(g, fifo_eng_timeout_r(), timeout);
}

void gv100_fifo_teardown_mask_intr(struct gk20a *g)
{
	u32 val;

	val = gk20a_readl(g, fifo_intr_en_0_r());
	val &= ~(fifo_intr_en_0_sched_error_m());
	gk20a_writel(g, fifo_intr_en_0_r(), val);
	gk20a_writel(g, fifo_intr_0_r(), fifo_intr_0_sched_error_reset_f());
}

void gv100_fifo_teardown_unmask_intr(struct gk20a *g)
{
	u32 val;

	val = gk20a_readl(g, fifo_intr_en_0_r());
	val |= fifo_intr_en_0_sched_error_f(1);
	gk20a_writel(g, fifo_intr_en_0_r(), val);
}
