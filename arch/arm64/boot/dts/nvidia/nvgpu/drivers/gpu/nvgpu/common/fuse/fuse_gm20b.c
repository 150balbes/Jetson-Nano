/*
 * GM20B FUSE
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>
#include <nvgpu/fuse.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "fuse_gm20b.h"

#include <nvgpu/hw/gm20b/hw_fuse_gm20b.h>

int gm20b_fuse_check_priv_security(struct gk20a *g)
{
	u32 gcplex_config;

	if (nvgpu_is_enabled(g, NVGPU_IS_FMODEL)) {
		__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
		__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, false);
		nvgpu_log(g, gpu_dbg_info, "priv sec is enabled in fmodel");
		return 0;
	}

	if (nvgpu_tegra_fuse_read_gcplex_config_fuse(g, &gcplex_config)) {
		nvgpu_err(g, "err reading gcplex config fuse, check fuse clk");
		return -EINVAL;
	}

	__nvgpu_set_enabled(g, NVGPU_SEC_SECUREGPCCS, false);

	if (gk20a_readl(g, fuse_opt_priv_sec_en_r())) {
		/*
		 * all falcons have to boot in LS mode and this needs
		 * wpr_enabled set to 1 and vpr_auto_fetch_disable
		 * set to 0. In this case gmmu tries to pull wpr
		 * and vpr settings from tegra mc
		 */
		__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, true);
		if ((gcplex_config &
			 GCPLEX_CONFIG_WPR_ENABLED_MASK) &&
			!(gcplex_config &
				GCPLEX_CONFIG_VPR_AUTO_FETCH_DISABLE_MASK)) {
			if (gk20a_readl(g, fuse_opt_sec_debug_en_r())) {
				nvgpu_log(g, gpu_dbg_info,
						"gcplex_config = 0x%08x, "
						"secure mode: ACR debug",
						gcplex_config);
			} else {
				nvgpu_log(g, gpu_dbg_info,
						"gcplex_config = 0x%08x, "
						"secure mode: ACR non debug",
						gcplex_config);
			}
		} else {
			nvgpu_err(g, "gcplex_config = 0x%08x "
				"invalid wpr_enabled/vpr_auto_fetch_disable "
				"with priv_sec_en", gcplex_config);
			/* do not try to boot GPU */
			return -EINVAL;
		}
	} else {
		__nvgpu_set_enabled(g, NVGPU_SEC_PRIVSECURITY, false);
		nvgpu_log(g, gpu_dbg_info,
				"gcplex_config = 0x%08x, non secure mode",
				gcplex_config);
	}

	return 0;
}

u32 gm20b_fuse_status_opt_fbio(struct gk20a *g)
{
	return nvgpu_readl(g, fuse_status_opt_fbio_r());
}

u32 gm20b_fuse_status_opt_fbp(struct gk20a *g)
{
	return nvgpu_readl(g, fuse_status_opt_fbp_r());
}

u32 gm20b_fuse_status_opt_rop_l2_fbp(struct gk20a *g, u32 fbp)
{
	return nvgpu_readl(g, fuse_status_opt_rop_l2_fbp_r(fbp));
}

u32 gm20b_fuse_status_opt_gpc(struct gk20a *g)
{
	return nvgpu_readl(g, fuse_status_opt_gpc_r());
}

u32 gm20b_fuse_status_opt_tpc_gpc(struct gk20a *g, u32 gpc)
{
	return nvgpu_readl(g, fuse_status_opt_tpc_gpc_r(gpc));
}

void gm20b_fuse_ctrl_opt_tpc_gpc(struct gk20a *g, u32 gpc, u32 val)
{
	nvgpu_writel(g, fuse_ctrl_opt_tpc_gpc_r(gpc), val);
}

u32 gm20b_fuse_opt_sec_debug_en(struct gk20a *g)
{
	return nvgpu_readl(g, fuse_opt_sec_debug_en_r());
}

u32 gm20b_fuse_opt_priv_sec_en(struct gk20a *g)
{
	return gk20a_readl(g, fuse_opt_priv_sec_en_r());
}
