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

#ifndef MM_GP10B_H
#define MM_GP10B_H

struct gk20a;
struct gk20a_mmu_level;
struct nvgpu_mem;
struct vm_gk20a;

u32 gp10b_mm_get_default_big_page_size(void);
u32 gp10b_mm_get_iommu_bit(struct gk20a *g);
int gp10b_init_bar2_vm(struct gk20a *g);
const struct gk20a_mmu_level *gp10b_mm_get_mmu_levels(struct gk20a *g,
	u32 big_page_size);
void gp10b_mm_init_pdb(struct gk20a *g, struct nvgpu_mem *inst_block,
		struct vm_gk20a *vm);
void gp10b_remove_bar2_vm(struct gk20a *g);

#endif
