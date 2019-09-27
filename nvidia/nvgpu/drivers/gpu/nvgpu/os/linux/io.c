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

void nvgpu_writel(struct gk20a *g, u32 r, u32 v)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (unlikely(!l->regs)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		writel_relaxed(v, l->regs + r);
		nvgpu_wmb();
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}
}

void nvgpu_writel_relaxed(struct gk20a *g, u32 r, u32 v)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (unlikely(!l->regs)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		writel_relaxed(v, l->regs + r);
	}
}

u32 nvgpu_readl(struct gk20a *g, u32 r)
{
	u32 v = __nvgpu_readl(g, r);

	if (v == 0xffffffff)
		__nvgpu_check_gpu_state(g);

	return v;
}

u32 __nvgpu_readl(struct gk20a *g, u32 r)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	u32 v = 0xffffffff;

	if (unlikely(!l->regs)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		v = readl(l->regs + r);
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}

	return v;
}

void nvgpu_writel_loop(struct gk20a *g, u32 r, u32 v)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (unlikely(!l->regs)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x (failed)", r, v);
	} else {
		nvgpu_wmb();
		do {
			writel_relaxed(v, l->regs + r);
		} while (readl(l->regs + r) != v);
		nvgpu_log(g, gpu_dbg_reg, "r=0x%x v=0x%x", r, v);
	}
}

void nvgpu_bar1_writel(struct gk20a *g, u32 b, u32 v)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (unlikely(!l->bar1)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "b=0x%x v=0x%x (failed)", b, v);
	} else {
		nvgpu_wmb();
		writel_relaxed(v, l->bar1 + b);
		nvgpu_log(g, gpu_dbg_reg, "b=0x%x v=0x%x", b, v);
	}
}

u32 nvgpu_bar1_readl(struct gk20a *g, u32 b)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	u32 v = 0xffffffff;

	if (unlikely(!l->bar1)) {
		__gk20a_warn_on_no_regs();
		nvgpu_log(g, gpu_dbg_reg, "b=0x%x v=0x%x (failed)", b, v);
	} else {
		v = readl(l->bar1 + b);
		nvgpu_log(g, gpu_dbg_reg, "b=0x%x v=0x%x", b, v);
	}

	return v;
}

bool nvgpu_io_exists(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	return l->regs != NULL;
}

bool nvgpu_io_valid_reg(struct gk20a *g, u32 r)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	return r < resource_size(l->regs);
}
