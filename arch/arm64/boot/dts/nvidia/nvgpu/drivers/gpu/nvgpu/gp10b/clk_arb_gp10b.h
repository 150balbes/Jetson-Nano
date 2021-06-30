/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef CLK_ARB_GP10B_H
#define CLK_ARB_GP10B_H

struct nvgpu_clk_session;
struct nvgpu_clk_arb;

u32 gp10b_get_arbiter_clk_domains(struct gk20a *g);
int gp10b_get_arbiter_f_points(struct gk20a *g,u32 api_domain,
				u32 *num_points, u16 *freqs_in_mhz);
int gp10b_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz);
int gp10b_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u16 *default_mhz);
int gp10b_init_clk_arbiter(struct gk20a *g);
void gp10b_clk_arb_run_arbiter_cb(struct nvgpu_clk_arb *arb);
void gp10b_clk_arb_cleanup(struct nvgpu_clk_arb *arb);

#endif /* CLK_ARB_GP106_H */
