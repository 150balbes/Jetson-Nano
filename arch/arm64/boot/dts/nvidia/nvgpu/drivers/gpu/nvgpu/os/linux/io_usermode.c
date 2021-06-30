/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "os_linux.h"

#include <nvgpu/hw/gv11b/hw_usermode_gv11b.h>

void nvgpu_usermode_writel(struct gk20a *g, u32 r, u32 v)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	void __iomem *reg = l->usermode_regs + (r - usermode_cfg0_r());

	writel_relaxed(v, reg);
	nvgpu_log(g, gpu_dbg_reg, "usermode r=0x%x v=0x%x", r, v);
}
