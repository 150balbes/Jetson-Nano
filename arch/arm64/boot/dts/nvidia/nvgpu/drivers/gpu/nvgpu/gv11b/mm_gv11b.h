/*
 * GV11B MM
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

#ifndef MM_GV11B_H
#define MM_GV11B_H

struct gk20a;
struct nvgpu_mem;
struct vm_gk20a;

bool gv11b_mm_is_bar1_supported(struct gk20a *g);
void gv11b_init_inst_block(struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm, u32 big_page_size);
bool gv11b_mm_mmu_fault_pending(struct gk20a *g);
int gv11b_init_mm_setup_hw(struct gk20a *g);
void gv11b_mm_l2_flush(struct gk20a *g, bool invalidate);
u64 gv11b_gpu_phys_addr(struct gk20a *g,
			struct nvgpu_gmmu_attrs *attrs, u64 phys);
void gv11b_mm_fault_info_mem_destroy(struct gk20a *g);
void gv11b_mm_mmu_fault_disable_hw(struct gk20a *g);

#endif
