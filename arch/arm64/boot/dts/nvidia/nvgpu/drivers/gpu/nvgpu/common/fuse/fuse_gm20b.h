/*
 * GM20B FUSE
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

#ifndef NVGPU_FUSE_GM20B_H
#define NVGPU_FUSE_GM20B_H

#define GCPLEX_CONFIG_VPR_AUTO_FETCH_DISABLE_MASK	((u32)(1 << 0))
#define GCPLEX_CONFIG_VPR_ENABLED_MASK			((u32)(1 << 1))
#define GCPLEX_CONFIG_WPR_ENABLED_MASK			((u32)(1 << 2))


struct gk20a;

int gm20b_fuse_check_priv_security(struct gk20a *g);
u32 gm20b_fuse_status_opt_fbio(struct gk20a *g);
u32 gm20b_fuse_status_opt_fbp(struct gk20a *g);
u32 gm20b_fuse_status_opt_rop_l2_fbp(struct gk20a *g, u32 fbp);
u32 gm20b_fuse_status_opt_gpc(struct gk20a *g);
u32 gm20b_fuse_status_opt_tpc_gpc(struct gk20a *g, u32 gpc);
void gm20b_fuse_ctrl_opt_tpc_gpc(struct gk20a *g, u32 gpc, u32 val);
u32 gm20b_fuse_opt_sec_debug_en(struct gk20a *g);
u32 gm20b_fuse_opt_priv_sec_en(struct gk20a *g);

#endif /* NVGPU_FUSE_GM20B_H */
