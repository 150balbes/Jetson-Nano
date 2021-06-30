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

#ifndef __FECS_TRACE_VGPU_H
#define __FECS_TRACE_VGPU_H

#include <nvgpu/types.h>

struct gk20a;
struct vm_area_struct;
struct nvgpu_gpu_ctxsw_trace_filter;

void vgpu_fecs_trace_data_update(struct gk20a *g);
int vgpu_fecs_trace_init(struct gk20a *g);
int vgpu_fecs_trace_deinit(struct gk20a *g);
int vgpu_fecs_trace_enable(struct gk20a *g);
int vgpu_fecs_trace_disable(struct gk20a *g);
bool vgpu_fecs_trace_is_enabled(struct gk20a *g);
int vgpu_fecs_trace_poll(struct gk20a *g);
int vgpu_alloc_user_buffer(struct gk20a *g, void **buf, size_t *size);
int vgpu_free_user_buffer(struct gk20a *g);
int vgpu_mmap_user_buffer(struct gk20a *g, struct vm_area_struct *vma);
int vgpu_fecs_trace_max_entries(struct gk20a *g,
			struct nvgpu_gpu_ctxsw_trace_filter *filter);
int vgpu_fecs_trace_set_filter(struct gk20a *g,
			struct nvgpu_gpu_ctxsw_trace_filter *filter);

#endif /* __FECS_TRACE_VGPU_H */
