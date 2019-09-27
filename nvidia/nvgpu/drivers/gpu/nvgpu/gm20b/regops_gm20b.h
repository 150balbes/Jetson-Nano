/*
 *
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2018 NVIDIA CORPORATION. All rights reserved.
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
#ifndef NVGPU_GM20B_REGOPS_GM20B_H
#define NVGPU_GM20B_REGOPS_GM20B_H

struct dbg_session_gk20a;

const struct regop_offset_range *gm20b_get_global_whitelist_ranges(void);
u64 gm20b_get_global_whitelist_ranges_count(void);
const struct regop_offset_range *gm20b_get_context_whitelist_ranges(void);
u64 gm20b_get_context_whitelist_ranges_count(void);
const u32 *gm20b_get_runcontrol_whitelist(void);
u64 gm20b_get_runcontrol_whitelist_count(void);
const struct regop_offset_range *gm20b_get_runcontrol_whitelist_ranges(void);
u64 gm20b_get_runcontrol_whitelist_ranges_count(void);
const u32 *gm20b_get_qctl_whitelist(void);
u64 gm20b_get_qctl_whitelist_count(void);
const struct regop_offset_range *gm20b_get_qctl_whitelist_ranges(void);
u64 gm20b_get_qctl_whitelist_ranges_count(void);
int gm20b_apply_smpc_war(struct dbg_session_gk20a *dbg_s);

#endif /* NVGPU_GM20B_REGOPS_GM20B_H */
