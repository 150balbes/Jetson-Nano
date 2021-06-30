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
#ifndef CLK_GP106_H
#define CLK_GP106_H

#include <nvgpu/lock.h>
#include <nvgpu/clk.h>

#define CLK_NAMEMAP_INDEX_GPC2CLK	0x00
#define CLK_NAMEMAP_INDEX_XBAR2CLK	0x02
#define CLK_NAMEMAP_INDEX_SYS2CLK	0x07	/* SYSPLL */
#define CLK_NAMEMAP_INDEX_DRAMCLK	0x20	/* DRAMPLL */

#define CLK_DEFAULT_CNTRL_SETTLE_RETRIES 10
#define CLK_DEFAULT_CNTRL_SETTLE_USECS   5

#define XTAL_CNTR_CLKS		27000	/* 1000usec at 27KHz XTAL */
#define XTAL_CNTR_DELAY		1000	/* we need acuracy up to the ms   */
#define XTAL_SCALE_TO_KHZ	1

u32 gp106_get_rate_cntr(struct gk20a *g, struct namemap_cfg *c);
int gp106_init_clk_support(struct gk20a *g);
u32 gp106_crystal_clk_hz(struct gk20a *g);
unsigned long gp106_clk_measure_freq(struct gk20a *g, u32 api_domain);
int gp106_suspend_clk_support(struct gk20a *g);
int gp106_clk_domain_get_f_points(
	struct gk20a *g,
	u32 clkapidomain,
	u32 *pfpointscount,
	u16 *pfreqpointsinmhz);

#endif /* CLK_GP106_H */
