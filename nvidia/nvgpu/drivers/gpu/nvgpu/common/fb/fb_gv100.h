/*
 * GV100 FB
 *
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_FB_GV100_H
#define NVGPU_FB_GV100_H

struct gk20a;

void gv100_fb_reset(struct gk20a *g);
void gv100_fb_enable_hub_intr(struct gk20a *g);
void gv100_fb_disable_hub_intr(struct gk20a *g);
int gv100_fb_memory_unlock(struct gk20a *g);
int gv100_fb_init_nvlink(struct gk20a *g);
int gv100_fb_enable_nvlink(struct gk20a *g);
size_t gv100_fb_get_vidmem_size(struct gk20a *g);
void gv100_fb_set_mmu_debug_mode(struct gk20a *g, bool enable);

#endif /* NVGPU_FB_GV100_H */
