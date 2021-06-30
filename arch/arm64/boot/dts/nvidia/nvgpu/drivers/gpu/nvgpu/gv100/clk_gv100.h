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
#ifndef CLK_GV100_H
#define CLK_GV100_H

#include <nvgpu/lock.h>
#include "gk20a/gk20a.h"

#define CLK_NAMEMAP_INDEX_GPCCLK	0x00
#define CLK_NAMEMAP_INDEX_XBARCLK	0x02
#define CLK_NAMEMAP_INDEX_SYSCLK	0x07	/* SYSPLL */
#define CLK_NAMEMAP_INDEX_DRAMCLK	0x20	/* DRAMPLL */

#define CLK_DEFAULT_CNTRL_SETTLE_RETRIES 10
#define CLK_DEFAULT_CNTRL_SETTLE_USECS   5
#define CLK_MAX_CNTRL_REGISTERS   2

#define XTAL_CNTR_CLKS		27000	/* 1000usec at 27KHz XTAL */
#define XTAL_CNTR_DELAY		10000	/* we need acuracy up to the 10ms   */
#define XTAL_SCALE_TO_KHZ	1
#define NUM_NAMEMAPS    (3U)
#define XTAL4X_KHZ 108000

u32 gv100_get_rate_cntr(struct gk20a *g, struct namemap_cfg *c);
struct namemap_cfg {
	u32 namemap;
	u32 is_enable;	/* Namemap enabled */
	u32 is_counter;	/* Using cntr */
	struct gk20a *g;
	struct {
		u32 reg_ctrl_addr;
		u32 reg_ctrl_idx;
		u32 reg_cntr_addr[CLK_MAX_CNTRL_REGISTERS];
	} cntr;
	u32 scale;
	char name[24];
};

int gv100_init_clk_support(struct gk20a *g);
u32 gv100_crystal_clk_hz(struct gk20a *g);
unsigned long gv100_clk_measure_freq(struct gk20a *g, u32 api_domain);
int gv100_suspend_clk_support(struct gk20a *g);

#endif /* CLK_GV100_H */
