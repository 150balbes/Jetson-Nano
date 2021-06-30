/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef LTC_GP10B_H
#define LTC_GP10B_H
struct gk20a;
struct gpu_ops;

void gp10b_ltc_isr(struct gk20a *g);

int gp10b_determine_L2_size_bytes(struct gk20a *g);
int gp10b_ltc_init_comptags(struct gk20a *g, struct gr_gk20a *gr);
void gp10b_ltc_init_fs_state(struct gk20a *g);
int gp10b_ltc_cbc_ctrl(struct gk20a *g, enum gk20a_cbc_op op,
		       u32 min, u32 max);
void gp10b_ltc_set_enabled(struct gk20a *g, bool enabled);
void gp10b_ltc_lts_isr(struct gk20a *g,
		unsigned int ltc, unsigned int slice);
#endif
