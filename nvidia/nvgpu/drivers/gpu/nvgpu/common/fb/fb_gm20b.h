/*
 * GM20B FB
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_FB_GM20B
#define NVGPU_FB_GM20B

#include <nvgpu/types.h>

struct gk20a;
struct wpr_carveout_info;
struct nvgpu_mem;

void gm20b_fb_init_hw(struct gk20a *g);
int gm20b_fb_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb);
void fb_gm20b_init_fs_state(struct gk20a *g);
void gm20b_fb_set_mmu_page_size(struct gk20a *g);
bool gm20b_fb_set_use_full_comp_tag_line(struct gk20a *g);
u32 gm20b_fb_mmu_ctrl(struct gk20a *g);
u32 gm20b_fb_mmu_debug_ctrl(struct gk20a *g);
u32 gm20b_fb_mmu_debug_wr(struct gk20a *g);
u32 gm20b_fb_mmu_debug_rd(struct gk20a *g);
unsigned int gm20b_fb_compression_page_size(struct gk20a *g);
unsigned int gm20b_fb_compressible_page_size(struct gk20a *g);
u32 gm20b_fb_compression_align_mask(struct gk20a *g);
void gm20b_fb_dump_vpr_info(struct gk20a *g);
void gm20b_fb_dump_wpr_info(struct gk20a *g);
void gm20b_fb_read_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf);
int gm20b_fb_vpr_info_fetch(struct gk20a *g);
bool gm20b_fb_debug_mode_enabled(struct gk20a *g);
void gm20b_fb_set_debug_mode(struct gk20a *g, bool enable);
void gm20b_fb_set_mmu_debug_mode(struct gk20a *g, bool enable);

#endif
