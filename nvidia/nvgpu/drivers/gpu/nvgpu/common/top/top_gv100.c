/*
 * GV100 TOP UNIT
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "top_gv100.h"

#include <nvgpu/hw/gv100/hw_top_gv100.h>

u32 gv100_top_get_nvhsclk_ctrl_e_clk_nvl(struct gk20a *g)
{
	u32 reg;

	reg = nvgpu_readl(g, top_nvhsclk_ctrl_r());
	return top_nvhsclk_ctrl_e_clk_nvl_v(reg);
}

void gv100_top_set_nvhsclk_ctrl_e_clk_nvl(struct gk20a *g, u32 val)
{
	u32 reg;

	reg = nvgpu_readl(g, top_nvhsclk_ctrl_r());
	reg = set_field(reg, top_nvhsclk_ctrl_e_clk_nvl_m(),
				top_nvhsclk_ctrl_e_clk_nvl_f(val));
	nvgpu_writel(g, top_nvhsclk_ctrl_r(), reg);
}

u32 gv100_top_get_nvhsclk_ctrl_swap_clk_nvl(struct gk20a *g)
{
	u32 reg;

	reg = nvgpu_readl(g, top_nvhsclk_ctrl_r());
	return top_nvhsclk_ctrl_swap_clk_nvl_v(reg);
}

void gv100_top_set_nvhsclk_ctrl_swap_clk_nvl(struct gk20a *g, u32 val)
{
	u32 reg;

	reg = nvgpu_readl(g, top_nvhsclk_ctrl_r());
	reg = set_field(reg, top_nvhsclk_ctrl_swap_clk_nvl_m(),
				top_nvhsclk_ctrl_swap_clk_nvl_f(val));
	nvgpu_writel(g, top_nvhsclk_ctrl_r(), reg);
}
