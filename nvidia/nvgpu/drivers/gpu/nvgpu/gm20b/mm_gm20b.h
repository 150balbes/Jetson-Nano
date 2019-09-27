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

#ifndef NVGPU_GM20B_MM_GM20B_H
#define NVGPU_GM20B_MM_GM20B_H
struct gk20a;

#define PDE_ADDR_START(x, y)	((x) &  ~((0x1UL << (y)) - 1))
#define PDE_ADDR_END(x, y)	((x) | ((0x1UL << (y)) - 1))

void gm20b_mm_set_big_page_size(struct gk20a *g,
				struct nvgpu_mem *mem, int size);
u32 gm20b_mm_get_big_page_sizes(void);
u32 gm20b_mm_get_default_big_page_size(void);
bool gm20b_mm_support_sparse(struct gk20a *g);
bool gm20b_mm_is_bar1_supported(struct gk20a *g);
u64 gm20b_gpu_phys_addr(struct gk20a *g,
			struct nvgpu_gmmu_attrs *attrs, u64 phys);
u32 gm20b_get_kind_invalid(void);
u32 gm20b_get_kind_pitch(void);
#endif /* NVGPU_GM20B_MM_GM20B_H */
