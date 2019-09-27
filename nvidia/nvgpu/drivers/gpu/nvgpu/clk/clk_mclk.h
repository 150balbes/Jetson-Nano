/*
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

#ifndef NVGPU_CLK_MCLK_H
#define NVGPU_CLK_MCLK_H

#include <nvgpu/lock.h>

#define GP106_MCLK_LOW_SPEED	0U
#define GP106_MCLK_MID_SPEED	1U
#define GP106_MCLK_HIGH_SPEED	2U
#define GP106_MCLK_NUM_SPEED	3U

enum gk20a_mclk_speed {
	gk20a_mclk_low_speed,
	gk20a_mclk_mid_speed,
	gk20a_mclk_high_speed,
};

struct clk_mclk_state {
	u32 speed;
	struct nvgpu_mutex mclk_lock;
	struct nvgpu_mutex data_lock;

	u16 p5_min;
	u16 p0_min;

	void *vreg_buf;
	bool init;

#ifdef CONFIG_DEBUG_FS
	s64 switch_max;
	s64 switch_min;
	u64 switch_num;
	s64 switch_avg;
	s64 switch_std;
	bool debugfs_set;
#endif
};

#endif /* NVGPU_CLK_MCLK_H */
