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

#include <soc/tegra/chip-id.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_bpmp.h>
#ifdef CONFIG_TEGRA_HV_MANAGER
#include <soc/tegra/virt/syscalls.h>
#endif

#include <nvgpu/soc.h>
#include "os_linux.h"
#include "platform_gk20a.h"

bool nvgpu_platform_is_silicon(struct gk20a *g)
{
	return tegra_platform_is_silicon();
}

bool nvgpu_platform_is_simulation(struct gk20a *g)
{
	return tegra_platform_is_vdk();
}

bool nvgpu_platform_is_fpga(struct gk20a *g)
{
	return tegra_platform_is_fpga();
}

bool nvgpu_is_hypervisor_mode(struct gk20a *g)
{
	return is_tegra_hypervisor_mode();
}

bool nvgpu_is_bpmp_running(struct gk20a *g)
{
	return tegra_bpmp_running();
}

bool nvgpu_is_soc_t194_a01(struct gk20a *g)
{
	return ((tegra_get_chip_id() == TEGRA194 &&
			tegra_chip_get_revision() == TEGRA194_REVISION_A01) ?
		true : false);
}

#ifdef CONFIG_TEGRA_HV_MANAGER
/* When nvlink is enabled on dGPU, we need to use physical memory addresses.
 * There is no SMMU translation. However, the device initially enumerates as a
 * PCIe device. As such, when allocation memory for this PCIe device, the DMA
 * framework ends up allocating memory using SMMU (if enabled in device tree).
 * As a result, when we switch to nvlink, we need to use underlying physical
 * addresses, even if memory mappings exist in SMMU.
 * In addition, when stage-2 SMMU translation is enabled (for instance when HV
 * is enabled), the addresses we get from dma_alloc are IPAs. We need to
 * convert them to PA.
 */
static u64 nvgpu_tegra_hv_ipa_pa(struct gk20a *g, u64 ipa)
{
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct hyp_ipa_pa_info info;
	int err;
	u64 pa = 0ULL;

	err = hyp_read_ipa_pa_info(&info, platform->vmid, ipa);
	if (err < 0) {
		/* WAR for bug 2096877
		 * hyp_read_ipa_pa_info only looks up RAM mappings.
		 * assume one to one IPA:PA mapping for syncpt aperture
		 */
		u64 start = g->syncpt_unit_base;
		u64 end = g->syncpt_unit_base + g->syncpt_unit_size;
		if ((ipa >= start) && (ipa < end)) {
			pa = ipa;
			nvgpu_log(g, gpu_dbg_map_v,
				"ipa=%llx vmid=%d -> pa=%llx (SYNCPT)\n",
				ipa, platform->vmid, pa);
		} else {
			nvgpu_err(g, "ipa=%llx translation failed vmid=%u err=%d",
				ipa, platform->vmid, err);
		}
	} else {
		pa = info.base + info.offset;
		nvgpu_log(g, gpu_dbg_map_v,
				"ipa=%llx vmid=%d -> pa=%llx "
				"base=%llx offset=%llx size=%llx\n",
				ipa, platform->vmid, pa, info.base,
				info.offset, info.size);
	}
	return pa;
}
#endif

int nvgpu_init_soc_vars(struct gk20a *g)
{
#ifdef CONFIG_TEGRA_HV_MANAGER
	struct device *dev = dev_from_gk20a(g);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	int err;

	if (nvgpu_is_hypervisor_mode(g)) {
		err = hyp_read_gid(&platform->vmid);
		if (err) {
			nvgpu_err(g, "failed to read vmid");
			return err;
		}
		platform->phys_addr = nvgpu_tegra_hv_ipa_pa;
	}
#endif
	return 0;
}
