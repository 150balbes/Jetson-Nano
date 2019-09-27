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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>

#include <nvgpu/log.h>
#include <nvgpu/linux/vm.h>
#include <nvgpu/bitops.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/dma.h>
#include <nvgpu/soc.h>
#include <nvgpu/hw_sim.h>
#include <nvgpu/sim.h>
#include <nvgpu/gk20a.h>

#include "platform_gk20a.h"
#include "os_linux.h"
#include "module.h"

void sim_writel(struct sim_nvgpu *sim, u32 r, u32 v)
{
	struct sim_nvgpu_linux *sim_linux =
		container_of(sim, struct sim_nvgpu_linux, sim);

	writel(v, sim_linux->regs + r);
}

u32 sim_readl(struct sim_nvgpu *sim, u32 r)
{
	struct sim_nvgpu_linux *sim_linux =
		container_of(sim, struct sim_nvgpu_linux, sim);

	return readl(sim_linux->regs + r);
}

void nvgpu_remove_sim_support_linux(struct gk20a *g)
{
	struct sim_nvgpu_linux *sim_linux;

	if (!g->sim)
		return;

	sim_linux = container_of(g->sim, struct sim_nvgpu_linux, sim);
	if (sim_linux->regs) {
		sim_writel(g->sim, sim_config_r(), sim_config_mode_disabled_v());
		iounmap(sim_linux->regs);
		sim_linux->regs = NULL;
	}
	nvgpu_kfree(g, sim_linux);
	g->sim = NULL;
}

int nvgpu_init_sim_support_linux(struct gk20a *g,
		struct platform_device *dev)
{
	struct sim_nvgpu_linux *sim_linux;
	int err = -ENOMEM;

	if (!nvgpu_platform_is_simulation(g))
		return 0;

	sim_linux = nvgpu_kzalloc(g, sizeof(*sim_linux));
	if (!sim_linux)
		return err;
	g->sim = &sim_linux->sim;
	g->sim->g = g;
	sim_linux->regs = nvgpu_devm_ioremap_resource(dev,
						      GK20A_SIM_IORESOURCE_MEM,
						      &sim_linux->reg_mem);
	if (IS_ERR(sim_linux->regs)) {
		nvgpu_err(g, "failed to remap gk20a sim regs");
		err = PTR_ERR(sim_linux->regs);
		goto fail;
	}
	sim_linux->remove_support_linux = nvgpu_remove_sim_support_linux;
	return 0;

fail:
	nvgpu_remove_sim_support_linux(g);
	return err;
}
