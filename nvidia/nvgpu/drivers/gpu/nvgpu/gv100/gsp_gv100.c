/*
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

#include <nvgpu/falcon.h>
#include <nvgpu/io.h>
#include <nvgpu/timers.h>

#include "gk20a/gk20a.h"
#include "gv100/gsp_gv100.h"

#include <nvgpu/hw/gv100/hw_pgsp_gv100.h>

int gv100_gsp_reset(struct gk20a *g)
{
	gk20a_writel(g, pgsp_falcon_engine_r(),
		pgsp_falcon_engine_reset_true_f());
	nvgpu_udelay(10);
	gk20a_writel(g, pgsp_falcon_engine_r(),
		pgsp_falcon_engine_reset_false_f());

	return 0;
}

static int gsp_flcn_bl_bootstrap(struct gk20a *g,
	struct nvgpu_falcon_bl_info *bl_info)
{
	struct mm_gk20a *mm = &g->mm;
	u32 data = 0;
	u32 status = 0;

	gk20a_writel(g, pgsp_falcon_itfen_r(),
		gk20a_readl(g, pgsp_falcon_itfen_r()) |
		pgsp_falcon_itfen_ctxen_enable_f());

	gk20a_writel(g, pgsp_falcon_nxtctx_r(),
		pgsp_falcon_nxtctx_ctxptr_f(
		nvgpu_inst_block_addr(g, &mm->pmu.inst_block) >> 12U) |
		pgsp_falcon_nxtctx_ctxvalid_f(1) |
		nvgpu_aperture_mask(g, &mm->pmu.inst_block,
			pgsp_falcon_nxtctx_ctxtgt_sys_ncoh_f(),
			pgsp_falcon_nxtctx_ctxtgt_sys_coh_f(),
			pgsp_falcon_nxtctx_ctxtgt_fb_f()));

	data = gk20a_readl(g, pgsp_falcon_debug1_r());
	data |= pgsp_falcon_debug1_ctxsw_mode_m();
	gk20a_writel(g, pgsp_falcon_debug1_r(), data);

	data = gk20a_readl(g, pgsp_falcon_engctl_r());
	data |= pgsp_falcon_engctl_switch_context_true_f();
	gk20a_writel(g, pgsp_falcon_engctl_r(), data);

	status = nvgpu_flcn_bl_bootstrap(&g->gsp_flcn, bl_info);

	return status;
}

int gv100_gsp_setup_hw_and_bl_bootstrap(struct gk20a *g,
	struct hs_acr *acr_desc,
	struct nvgpu_falcon_bl_info *bl_info)
{
	u32 data = 0;
	int err = 0;

	err = nvgpu_flcn_reset(&g->gsp_flcn);
	if (err != 0) {
		goto exit;
	}

	data = gk20a_readl(g, pgsp_fbif_ctl_r());
	data |= pgsp_fbif_ctl_allow_phys_no_ctx_allow_f();
	gk20a_writel(g, pgsp_fbif_ctl_r(), data);

	/* setup apertures - virtual */
	gk20a_writel(g, pgsp_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			pgsp_fbif_transcfg_mem_type_physical_f() |
			pgsp_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pgsp_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			pgsp_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, pgsp_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			pgsp_fbif_transcfg_mem_type_physical_f() |
			pgsp_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pgsp_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			pgsp_fbif_transcfg_mem_type_physical_f() |
			pgsp_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, pgsp_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			pgsp_fbif_transcfg_mem_type_physical_f() |
			pgsp_fbif_transcfg_target_noncoherent_sysmem_f());

	err = gsp_flcn_bl_bootstrap(g, bl_info);

exit:
	return err;
}
