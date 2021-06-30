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

#ifndef _VGPU_FIFO_GV11B_H_
#define _VGPU_FIFO_GV11B_H_

struct gk20a;

int vgpu_gv11b_init_fifo_setup_hw(struct gk20a *g);
int vgpu_gv11b_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf);
void vgpu_gv11b_fifo_free_syncpt_buf(struct channel_gk20a *c,
					struct nvgpu_mem *syncpt_buf);
int vgpu_gv11b_fifo_get_sync_ro_map(struct vm_gk20a *vm,
	u64 *base_gpuva, u32 *sync_size);
#endif
