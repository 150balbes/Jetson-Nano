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
#ifndef NVGPU_PTIMER_H
#define NVGPU_PTIMER_H

#include <nvgpu/types.h>

struct gk20a;

struct nvgpu_cpu_time_correlation_sample {
	u64 cpu_timestamp;
	u64 gpu_timestamp;
};

/* PTIMER_REF_FREQ_HZ corresponds to a period of 32 nanoseconds.
    32 ns is the resolution of ptimer. */
#define PTIMER_REF_FREQ_HZ                      31250000

static inline u32 ptimer_scalingfactor10x(u32 ptimer_src_freq)
{
	return (u32)(((u64)(PTIMER_REF_FREQ_HZ * 10)) / ptimer_src_freq);
}

static inline u32 scale_ptimer(u32 timeout , u32 scale10x)
{
	if (((timeout*10) % scale10x) >= (scale10x/2)) {
		return ((timeout * 10) / scale10x) + 1;
	} else {
		return (timeout * 10) / scale10x;
	}
}

int nvgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples);
#endif
