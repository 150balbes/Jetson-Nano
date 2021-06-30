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
#include <nvgpu/hw_sim_pci.h>
#include <nvgpu/sim.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "os_linux.h"
#include "module.h"

static bool _nvgpu_pci_is_simulation(struct gk20a *g, u32 sim_base)
{
	u32 cfg;
	bool is_simulation = false;

	cfg = nvgpu_readl(g, sim_base + sim_config_r());
	if (sim_config_mode_v(cfg) == sim_config_mode_enabled_v())
		is_simulation = true;

	return is_simulation;
}

void nvgpu_remove_sim_support_linux_pci(struct gk20a *g)
{
	struct sim_nvgpu_linux *sim_linux;
	bool is_simulation;

	is_simulation = _nvgpu_pci_is_simulation(g, sim_r());

	if (!is_simulation) {
		return;
	}

	if (!g->sim) {
		nvgpu_warn(g, "sim_gk20a not allocated");
		return;
	}
	sim_linux = container_of(g->sim, struct sim_nvgpu_linux, sim);

	if (sim_linux->regs) {
		sim_writel(g->sim, sim_config_r(), sim_config_mode_disabled_v());
		sim_linux->regs = NULL;
	}
	nvgpu_kfree(g, sim_linux);
	g->sim = NULL;
}

int nvgpu_init_sim_support_linux_pci(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct sim_nvgpu_linux *sim_linux;
	int err = -ENOMEM;
	bool is_simulation;

	is_simulation = _nvgpu_pci_is_simulation(g, sim_r());
	__nvgpu_set_enabled(g, NVGPU_IS_FMODEL, is_simulation);

	if (!is_simulation)
		return 0;

	sim_linux = nvgpu_kzalloc(g, sizeof(*sim_linux));
	if (!sim_linux)
		return err;
	g->sim = &sim_linux->sim;
	g->sim->g = g;
	sim_linux->regs = l->regs + sim_r();
	sim_linux->remove_support_linux = nvgpu_remove_sim_support_linux_pci;

	return 0;
}
