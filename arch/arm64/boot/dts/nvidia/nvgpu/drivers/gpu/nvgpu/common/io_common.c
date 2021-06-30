/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <nvgpu/io.h>
#include <nvgpu/types.h>
#include <nvgpu/gk20a.h>

void nvgpu_writel_check(struct gk20a *g, u32 r, u32 v)
{
	u32 read_val = 0U;

	nvgpu_writel(g, r, v);
	read_val = nvgpu_readl(g, r);
	if (v != read_val) {
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x rd=0x%x wr=0x%x (mismatch)",
					r, read_val, v);
	}
}
