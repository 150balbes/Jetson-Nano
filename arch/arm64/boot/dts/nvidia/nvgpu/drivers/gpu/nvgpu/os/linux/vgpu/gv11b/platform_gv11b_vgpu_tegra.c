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

#include <linux/platform_device.h>

#include <nvgpu/nvhost.h>
#include <nvgpu/gk20a.h>

#include "vgpu/clk_vgpu.h"
#include "os/linux/platform_gk20a.h"
#include "os/linux/os_linux.h"
#include "os/linux/vgpu/vgpu_linux.h"
#include "os/linux/vgpu/platform_vgpu_tegra.h"

static int gv11b_vgpu_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	struct resource *r;
	void __iomem *regs;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(platform->g);
	struct gk20a *g = platform->g;
	int ret;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "usermode");
	if (!r) {
		nvgpu_err(g, "failed to get usermode regs");
		return -ENXIO;
	}
	regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(regs)) {
		nvgpu_err(g, "failed to map usermode regs");
		return PTR_ERR(regs);
	}
	l->usermode_regs = regs;

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	ret = nvgpu_get_nvhost_dev(g);
	if (ret) {
		l->usermode_regs = NULL;
		return ret;
	}

	ret = nvgpu_nvhost_syncpt_unit_interface_get_aperture(g->nvhost_dev,
							&g->syncpt_unit_base,
							&g->syncpt_unit_size);
	if (ret) {
		nvgpu_err(g, "Failed to get syncpt interface");
		return -ENOSYS;
	}
	g->syncpt_size = nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(1);
	nvgpu_info(g, "syncpt_unit_base %llx syncpt_unit_size %zx size %x\n",
		g->syncpt_unit_base, g->syncpt_unit_size, g->syncpt_size);
#endif
	vgpu_init_clk_support(platform->g);

	return 0;
}

struct gk20a_platform gv11b_vgpu_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.can_railgate_init	= false,
	.can_elpg_init          = false,
	.enable_slcg            = false,
	.enable_blcg            = false,
	.enable_elcg            = false,
	.enable_elpg            = false,
	.enable_aelpg           = false,
	.can_slcg               = false,
	.can_blcg               = false,
	.can_elcg               = false,

	.ch_wdt_timeout_ms = 5000,

	.probe = gv11b_vgpu_probe,

	.clk_round_rate = vgpu_plat_clk_round_rate,
	.get_clk_freqs = vgpu_plat_clk_get_freqs,

	/* frequency scaling configuration */
	.devfreq_governor = "userspace",

	.virtual_dev = true,

	/* power management callbacks */
	.suspend = vgpu_tegra_suspend,
	.resume = vgpu_tegra_resume,
};
