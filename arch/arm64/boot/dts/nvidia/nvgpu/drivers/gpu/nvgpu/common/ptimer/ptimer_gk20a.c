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
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "ptimer_gk20a.h"

#include <nvgpu/hw/gk20a/hw_timer_gk20a.h>

void gk20a_ptimer_isr(struct gk20a *g)
{
	u32 save0, save1, fecs_errcode = 0;

	save0 = gk20a_readl(g, timer_pri_timeout_save_0_r());
	if (timer_pri_timeout_save_0_fecs_tgt_v(save0)) {
		/*
		 * write & addr fields in timeout_save0
		 * might not be reliable
		 */
		fecs_errcode = gk20a_readl(g,
				timer_pri_timeout_fecs_errcode_r());
	}

	save1 = gk20a_readl(g, timer_pri_timeout_save_1_r());
	nvgpu_err(g, "PRI timeout: ADR 0x%08x "
		"%s  DATA 0x%08x",
		timer_pri_timeout_save_0_addr_v(save0) << 2,
		(timer_pri_timeout_save_0_write_v(save0) != 0U) ?
		"WRITE" : "READ", save1);

	gk20a_writel(g, timer_pri_timeout_save_0_r(), 0);
	gk20a_writel(g, timer_pri_timeout_save_1_r(), 0);

	if (fecs_errcode) {
		nvgpu_err(g, "FECS_ERRCODE 0x%08x", fecs_errcode);
		if (g->ops.priv_ring.decode_error_code) {
			g->ops.priv_ring.decode_error_code(g,
						fecs_errcode);
		}
	}
}

int gk20a_read_ptimer(struct gk20a *g, u64 *value)
{
	const unsigned int max_iterations = 3;
	unsigned int i = 0;
	u32 gpu_timestamp_hi_prev = 0;

	if (value == NULL) {
		return -EINVAL;
	}

	/* Note. The GPU nanosecond timer consists of two 32-bit
	 * registers (high & low). To detect a possible low register
	 * wrap-around between the reads, we need to read the high
	 * register before and after low. The wraparound happens
	 * approximately once per 4 secs. */

	/* get initial gpu_timestamp_hi value */
	gpu_timestamp_hi_prev = gk20a_readl(g, timer_time_1_r());

	for (i = 0; i < max_iterations; ++i) {
		u32 gpu_timestamp_hi = 0;
		u32 gpu_timestamp_lo = 0;

		gpu_timestamp_lo = gk20a_readl(g, timer_time_0_r());
		gpu_timestamp_hi = gk20a_readl(g, timer_time_1_r());

		if (gpu_timestamp_hi == gpu_timestamp_hi_prev) {
			*value = (((u64)gpu_timestamp_hi) << 32) |
				gpu_timestamp_lo;
			return 0;
		}

		/* wrap-around detected, retry */
		gpu_timestamp_hi_prev = gpu_timestamp_hi;
	}

	/* too many iterations, bail out */
	nvgpu_err(g, "failed to read ptimer");
	return -EBUSY;
}
