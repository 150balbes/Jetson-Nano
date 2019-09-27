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
#ifndef NVGPU_IO_H
#define NVGPU_IO_H

#include <nvgpu/types.h>

/* Legacy defines - should be removed once everybody uses nvgpu_* */
#define gk20a_writel nvgpu_writel
#define gk20a_readl nvgpu_readl
#define gk20a_writel_check nvgpu_writel_check
#define gk20a_bar1_writel nvgpu_bar1_writel
#define gk20a_bar1_readl nvgpu_bar1_readl
#define gk20a_io_exists nvgpu_io_exists
#define gk20a_io_valid_reg nvgpu_io_valid_reg

struct gk20a;

void nvgpu_writel(struct gk20a *g, u32 r, u32 v);
void nvgpu_writel_relaxed(struct gk20a *g, u32 r, u32 v);
u32 nvgpu_readl(struct gk20a *g, u32 r);
u32 __nvgpu_readl(struct gk20a *g, u32 r);
void nvgpu_writel_check(struct gk20a *g, u32 r, u32 v);
void nvgpu_writel_loop(struct gk20a *g, u32 r, u32 v);
void nvgpu_bar1_writel(struct gk20a *g, u32 b, u32 v);
u32 nvgpu_bar1_readl(struct gk20a *g, u32 b);
bool nvgpu_io_exists(struct gk20a *g);
bool nvgpu_io_valid_reg(struct gk20a *g, u32 r);

#endif /* NVGPU_IO_H */
