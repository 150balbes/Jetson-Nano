/*
 * GP106 GPU GR
 *
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

#ifndef NVGPU_GR_GP106_H
#define NVGPU_GR_GP106_H

struct gk20a;

enum {
	PASCAL_B                 = 0xC197,
	PASCAL_COMPUTE_B         = 0xC1C0,
};

bool gr_gp106_is_valid_class(struct gk20a *g, u32 class_num);
u32 gr_gp106_pagepool_default_size(struct gk20a *g);
int gr_gp106_handle_sw_method(struct gk20a *g, u32 addr,
				     u32 class_num, u32 offset, u32 data);
void gr_gp106_cb_size_default(struct gk20a *g);
int gr_gp106_set_ctxsw_preemption_mode(struct gk20a *g,
				struct nvgpu_gr_ctx *gr_ctx,
				struct vm_gk20a *vm, u32 class,
				u32 graphics_preempt_mode,
				u32 compute_preempt_mode);

#endif /* NVGPU_GR_GP106_H */
