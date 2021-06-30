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

#ifndef _DBG_VGPU_H_
#define _DBG_VGPU_H_

struct dbg_session_gk20a;
struct nvgpu_dbg_reg_op;
struct dbg_profiler_object_data;
struct gk20a;

int vgpu_exec_regops(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_reg_op *ops,
		      u64 num_ops,
		      bool *is_current_ctx);
int vgpu_dbg_set_powergate(struct dbg_session_gk20a *dbg_s, bool disable_powergate);
bool vgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
bool vgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);

void vgpu_release_profiler_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
int vgpu_perfbuffer_enable(struct gk20a *g, u64 offset, u32 size);
int vgpu_perfbuffer_disable(struct gk20a *g);
#endif
