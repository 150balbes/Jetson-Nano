/*
 *
 * nvgpu sim support
 *
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SIM_LINUX_H__
#define __SIM_LINUX_H__

struct platform_device;

struct sim_nvgpu_linux {
	struct sim_nvgpu sim;
	struct resource *reg_mem;
	void __iomem *regs;
	void (*remove_support_linux)(struct gk20a *g);
};

void sim_writel(struct sim_nvgpu *sim, u32 r, u32 v);
u32 sim_readl(struct sim_nvgpu *sim, u32 r);

int nvgpu_init_sim_support_linux(struct gk20a *g,
		struct platform_device *dev);
void nvgpu_remove_sim_support_linux(struct gk20a *g);
#endif
