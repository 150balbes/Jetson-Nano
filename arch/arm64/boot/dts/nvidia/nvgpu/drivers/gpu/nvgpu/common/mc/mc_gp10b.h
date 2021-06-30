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

#ifndef MC_GP10B_H
#define MC_GP10B_H

#include <nvgpu/types.h>

struct gk20a;
enum nvgpu_unit;

void mc_gp10b_intr_mask(struct gk20a *g);
void mc_gp10b_intr_enable(struct gk20a *g);
void mc_gp10b_intr_unit_config(struct gk20a *g, bool enable,
		bool is_stalling, u32 mask);
void mc_gp10b_isr_stall(struct gk20a *g);
bool mc_gp10b_is_intr1_pending(struct gk20a *g,
				      enum nvgpu_unit unit, u32 mc_intr_1);

void mc_gp10b_log_pending_intrs(struct gk20a *g);
u32 mc_gp10b_intr_stall(struct gk20a *g);
void mc_gp10b_intr_stall_pause(struct gk20a *g);
void mc_gp10b_intr_stall_resume(struct gk20a *g);
u32 mc_gp10b_intr_nonstall(struct gk20a *g);
void mc_gp10b_intr_nonstall_pause(struct gk20a *g);
void mc_gp10b_intr_nonstall_resume(struct gk20a *g);

#endif
