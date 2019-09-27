/*
 * GP106 FUSE
 *
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

#ifndef NVGPU_FUSE_GP106_H
#define NVGPU_FUSE_GP106_H

struct gk20a;

int gp106_fuse_check_priv_security(struct gk20a *g);
u32 gp106_fuse_read_vin_cal_fuse_rev(struct gk20a *g);
u32 gp106_fuse_read_vin_cal_slope_intercept_fuse(struct gk20a *g,
					     u32 vin_id, u32 *slope,
					     u32 *intercept);
u32 gp106_fuse_read_vin_cal_gain_offset_fuse(struct gk20a *g,
					     u32 vin_id, s8 *gain,
					     s8 *offset);

#endif /* NVGPU_FUSE_GP106_H */
