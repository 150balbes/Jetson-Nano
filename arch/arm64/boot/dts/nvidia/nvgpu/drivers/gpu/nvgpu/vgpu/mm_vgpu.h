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

#ifndef _MM_VGPU_H_
#define _MM_VGPU_H_

struct nvgpu_mem;
struct channel_gk20a;
struct vm_gk20a_mapping_batch;
struct gk20a_as_share;
struct vm_gk20a;
enum gk20a_mem_rw_flag;

void vgpu_locked_gmmu_unmap(struct vm_gk20a *vm,
				u64 vaddr,
				u64 size,
				u32 pgsz_idx,
				bool va_allocated,
				enum gk20a_mem_rw_flag rw_flag,
				bool sparse,
				struct vm_gk20a_mapping_batch *batch);
int vgpu_vm_bind_channel(struct vm_gk20a *vm,
				struct channel_gk20a *ch);
int vgpu_mm_fb_flush(struct gk20a *g);
void vgpu_mm_l2_invalidate(struct gk20a *g);
void vgpu_mm_l2_flush(struct gk20a *g, bool invalidate);
int vgpu_mm_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb);
void vgpu_mm_mmu_set_debug_mode(struct gk20a *g, bool enable);
#endif
