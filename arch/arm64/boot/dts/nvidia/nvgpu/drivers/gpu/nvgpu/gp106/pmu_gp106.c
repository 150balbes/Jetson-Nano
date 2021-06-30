/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/pmu.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/gk20a.h>

#include "gk20a/pmu_gk20a.h"

#include "gm20b/acr_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gp10b/pmu_gp10b.h"
#include "gp106/pmu_gp106.h"
#include "gp106/acr_gp106.h"

#include "clk/clk_mclk.h"

#include "lpwr/lpwr.h"
#include "lpwr/rppg.h"

#include <nvgpu/hw/gp106/hw_psec_gp106.h>
#include <nvgpu/hw/gp106/hw_pwr_gp106.h>

bool gp106_is_pmu_supported(struct gk20a *g)
{
	return true;
}

bool gp106_pmu_is_engine_in_reset(struct gk20a *g)
{
	u32 reg_reset;
	bool status = false;

	reg_reset = gk20a_readl(g, pwr_falcon_engine_r());
	if (reg_reset == pwr_falcon_engine_reset_true_f()) {
		status = true;
	}

	return status;
}

int gp106_pmu_engine_reset(struct gk20a *g, bool do_reset)
{
	/*
	* From GP10X onwards, we are using PPWR_FALCON_ENGINE for reset. And as
	* it may come into same behavior, reading NV_PPWR_FALCON_ENGINE again
	* after Reset.
	*/
	if (do_reset) {
		gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_false_f());
		(void) gk20a_readl(g, pwr_falcon_engine_r());
	} else {
		gk20a_writel(g, pwr_falcon_engine_r(),
			pwr_falcon_engine_reset_true_f());
		(void) gk20a_readl(g, pwr_falcon_engine_r());
	}

	return 0;
}

u32 gp106_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id)
{
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		return NVGPU_PMU_GR_FEATURE_MASK_RPPG;
	}

	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
		return NVGPU_PMU_MS_FEATURE_MASK_ALL;
	}

	return 0;
}

u32 gp106_pmu_pg_engines_list(struct gk20a *g)
{
	return BIT(PMU_PG_ELPG_ENGINE_ID_GRAPHICS) |
			BIT(PMU_PG_ELPG_ENGINE_ID_MS);
}

static void pmu_handle_param_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	nvgpu_log_fn(g, " ");

	if (status != 0) {
		nvgpu_err(g, "PG PARAM cmd aborted");
		return;
	}

	gp106_dbg_pmu(g, "PG PARAM is acknowledged from PMU %x",
			msg->msg.pg.msg_type);
}

int gp106_pg_param_init(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;
	u32 status;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {

		status = init_rppg(g);
		if (status != 0) {
			nvgpu_err(g, "RPPG init Failed");
			return -1;
		}

		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_pg_cmd_gr_init_param);
		cmd.cmd.pg.gr_init_param.cmd_type =
				PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.gr_init_param.sub_cmd_id =
				PMU_PG_PARAM_CMD_GR_INIT_PARAM;
		cmd.cmd.pg.gr_init_param.featuremask =
				NVGPU_PMU_GR_FEATURE_MASK_RPPG;

		gp106_dbg_pmu(g, "cmd post GR PMU_PG_CMD_ID_PG_PARAM");
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_param_msg, pmu, &seq, ~0);
	} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
			sizeof(struct pmu_pg_cmd_ms_init_param);
		cmd.cmd.pg.ms_init_param.cmd_type =
			PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.ms_init_param.cmd_id =
			PMU_PG_PARAM_CMD_MS_INIT_PARAM;
		cmd.cmd.pg.ms_init_param.support_mask =
			NVGPU_PMU_MS_FEATURE_MASK_CLOCK_GATING |
			NVGPU_PMU_MS_FEATURE_MASK_SW_ASR |
			NVGPU_PMU_MS_FEATURE_MASK_RPPG |
			NVGPU_PMU_MS_FEATURE_MASK_FB_TRAINING;

		gp106_dbg_pmu(g, "cmd post MS PMU_PG_CMD_ID_PG_PARAM");
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_param_msg, pmu, &seq, ~0);
	}

	return 0;
}

void gp106_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_pg_stats_v2 stats;

	nvgpu_flcn_copy_from_dmem(pmu->flcn,
		pmu->stat_dmem_offset[pg_engine_id],
		(u8 *)&stats, sizeof(struct pmu_pg_stats_v2), 0);

	pg_stat_data->ingating_time = stats.total_sleep_time_us;
	pg_stat_data->ungating_time = stats.total_non_sleep_time_us;
	pg_stat_data->gating_cnt = stats.entry_count;
	pg_stat_data->avg_entry_latency_us = stats.entry_latency_avg_us;
	pg_stat_data->avg_exit_latency_us = stats.exit_latency_avg_us;
}

bool gp106_pmu_is_lpwr_feature_supported(struct gk20a *g, u32 feature_id)
{
	bool is_feature_supported = false;

	switch (feature_id) {
	case PMU_PG_LPWR_FEATURE_RPPG:
		is_feature_supported = nvgpu_lpwr_is_rppg_supported(g,
			nvgpu_clk_arb_get_current_pstate(g));
		break;
	case PMU_PG_LPWR_FEATURE_MSCG:
		is_feature_supported = nvgpu_lpwr_is_mscg_supported(g,
			nvgpu_clk_arb_get_current_pstate(g));
		break;
	default:
		is_feature_supported = false;
	}

	return is_feature_supported;
}

bool gp106_is_lazy_bootstrap(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = true;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

bool gp106_is_priv_load(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = true;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

static void gp106_pmu_load_multiple_falcons(struct gk20a *g, u32 falconidmask,
					 u32 flags)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	nvgpu_log_fn(g, " ");

	gp106_dbg_pmu(g, "wprinit status = %x\n", g->pmu_lsf_pmu_wpr_init_done);
	if (g->pmu_lsf_pmu_wpr_init_done) {
		/* send message to load FECS falcon */
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_ACR;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
		  sizeof(struct pmu_acr_cmd_bootstrap_multiple_falcons);
		cmd.cmd.acr.boot_falcons.cmd_type =
		  PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS;
		cmd.cmd.acr.boot_falcons.flags = flags;
		cmd.cmd.acr.boot_falcons.falconidmask =
				falconidmask;
		cmd.cmd.acr.boot_falcons.usevamask = 0;
		cmd.cmd.acr.boot_falcons.wprvirtualbase.lo = 0;
		cmd.cmd.acr.boot_falcons.wprvirtualbase.hi = 0;

		gp106_dbg_pmu(g, "PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS:%x\n",
				falconidmask);
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_fecs_boot_acr_msg, pmu, &seq, ~0);
	}

	nvgpu_log_fn(g, "done");
}

int gp106_load_falcon_ucode(struct gk20a *g, u32 falconidmask)
{
	u32 flags = PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES;

	/* GM20B PMU supports loading FECS and GPCCS only */
	if (falconidmask == 0) {
		return -EINVAL;
	}
	if (falconidmask & ~((1 << LSF_FALCON_ID_FECS) |
			(1 << LSF_FALCON_ID_GPCCS))) {
		return -EINVAL;
	}
	g->pmu_lsf_loaded_falcon_id = 0;
	/* check whether pmu is ready to bootstrap lsf if not wait for it */
	if (!g->pmu_lsf_pmu_wpr_init_done) {
		pmu_wait_message_cond(&g->pmu,
				gk20a_get_gr_idle_timeout(g),
				&g->pmu_lsf_pmu_wpr_init_done, 1);
		/* check again if it still not ready indicate an error */
		if (!g->pmu_lsf_pmu_wpr_init_done) {
			nvgpu_err(g, "PMU not ready to load LSF");
			return -ETIMEDOUT;
		}
	}
	/* load falcon(s) */
	gp106_pmu_load_multiple_falcons(g, falconidmask, flags);
	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&g->pmu_lsf_loaded_falcon_id, falconidmask);
	if (g->pmu_lsf_loaded_falcon_id != falconidmask) {
		return -ETIMEDOUT;
	}
	return 0;
}

void gp106_update_lspmu_cmdline_args(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;

	/*Copying pmu cmdline args*/
	g->ops.pmu_ver.set_pmu_cmdline_args_cpu_freq(pmu, 0);
	g->ops.pmu_ver.set_pmu_cmdline_args_secure_mode(pmu, 1);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_size(
		pmu, GK20A_PMU_TRACE_BUFSIZE);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_base(pmu);
	g->ops.pmu_ver.set_pmu_cmdline_args_trace_dma_idx(
		pmu, GK20A_PMU_DMAIDX_VIRT);
	if (g->ops.pmu_ver.config_pmu_cmdline_args_super_surface) {
		g->ops.pmu_ver.config_pmu_cmdline_args_super_surface(pmu);
	}

	nvgpu_flcn_copy_to_dmem(pmu->flcn, g->acr.pmu_args,
		(u8 *)(g->ops.pmu_ver.get_pmu_cmdline_args_ptr(pmu)),
		g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu), 0);

}

void gp106_pmu_setup_apertures(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;

	/* PMU TRANSCFG */
	/* setup apertures - virtual */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			pwr_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_local_fb_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());

	/* PMU Config */
	gk20a_writel(g, pwr_falcon_itfen_r(),
				gk20a_readl(g, pwr_falcon_itfen_r()) |
				pwr_falcon_itfen_ctxen_enable_f());
	gk20a_writel(g, pwr_pmu_new_instblk_r(),
		pwr_pmu_new_instblk_ptr_f(
			nvgpu_inst_block_addr(g, &mm->pmu.inst_block) >> 12) |
		pwr_pmu_new_instblk_valid_f(1) |
		nvgpu_aperture_mask(g, &mm->pmu.inst_block,
			pwr_pmu_new_instblk_target_sys_ncoh_f(),
			pwr_pmu_new_instblk_target_sys_coh_f(),
			pwr_pmu_new_instblk_target_fb_f()));
}
