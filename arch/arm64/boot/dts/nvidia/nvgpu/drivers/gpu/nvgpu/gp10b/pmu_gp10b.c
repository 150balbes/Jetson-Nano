/*
 * GP10B PMU
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/log.h>
#include <nvgpu/fuse.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "gk20a/pmu_gk20a.h"
#include "gm20b/acr_gm20b.h"
#include "gm20b/pmu_gm20b.h"

#include "pmu_gp10b.h"

#include <nvgpu/hw/gp10b/hw_pwr_gp10b.h>

#define gp10b_dbg_pmu(g, fmt, arg...) \
	nvgpu_log(g, gpu_dbg_pmu, fmt, ##arg)

/* PROD settings for ELPG sequencing registers*/
static struct pg_init_sequence_list _pginitseq_gp10b[] = {
		{0x0010ab10, 0x0000868B} ,
		{0x0010e118, 0x8590848F} ,
		{0x0010e000, 0} ,
		{0x0010e06c, 0x000000A3} ,
		{0x0010e06c, 0x000000A0} ,
		{0x0010e06c, 0x00000095} ,
		{0x0010e06c, 0x000000A6} ,
		{0x0010e06c, 0x0000008C} ,
		{0x0010e06c, 0x00000080} ,
		{0x0010e06c, 0x00000081} ,
		{0x0010e06c, 0x00000087} ,
		{0x0010e06c, 0x00000088} ,
		{0x0010e06c, 0x0000008D} ,
		{0x0010e06c, 0x00000082} ,
		{0x0010e06c, 0x00000083} ,
		{0x0010e06c, 0x00000089} ,
		{0x0010e06c, 0x0000008A} ,
		{0x0010e06c, 0x000000A2} ,
		{0x0010e06c, 0x00000097} ,
		{0x0010e06c, 0x00000092} ,
		{0x0010e06c, 0x00000099} ,
		{0x0010e06c, 0x0000009B} ,
		{0x0010e06c, 0x0000009D} ,
		{0x0010e06c, 0x0000009F} ,
		{0x0010e06c, 0x000000A1} ,
		{0x0010e06c, 0x00000096} ,
		{0x0010e06c, 0x00000091} ,
		{0x0010e06c, 0x00000098} ,
		{0x0010e06c, 0x0000009A} ,
		{0x0010e06c, 0x0000009C} ,
		{0x0010e06c, 0x0000009E} ,
		{0x0010ab14, 0x00000000} ,
		{0x0010e024, 0x00000000} ,
		{0x0010e028, 0x00000000} ,
		{0x0010e11c, 0x00000000} ,
		{0x0010ab1c, 0x140B0BFF} ,
		{0x0010e020, 0x0E2626FF} ,
		{0x0010e124, 0x251010FF} ,
		{0x0010ab20, 0x89abcdef} ,
		{0x0010ab24, 0x00000000} ,
		{0x0010e02c, 0x89abcdef} ,
		{0x0010e030, 0x00000000} ,
		{0x0010e128, 0x89abcdef} ,
		{0x0010e12c, 0x00000000} ,
		{0x0010ab28, 0x7FFFFFFF} ,
		{0x0010ab2c, 0x70000000} ,
		{0x0010e034, 0x7FFFFFFF} ,
		{0x0010e038, 0x70000000} ,
		{0x0010e130, 0x7FFFFFFF} ,
		{0x0010e134, 0x70000000} ,
		{0x0010ab30, 0x00000000} ,
		{0x0010ab34, 0x00000001} ,
		{0x00020004, 0x00000000} ,
		{0x0010e138, 0x00000000} ,
		{0x0010e040, 0x00000000} ,
		{0x0010e168, 0x00000000} ,
		{0x0010e114, 0x0000A5A4} ,
		{0x0010e110, 0x00000000} ,
		{0x0010e10c, 0x8590848F} ,
		{0x0010e05c, 0x00000000} ,
		{0x0010e044, 0x00000000} ,
		{0x0010a644, 0x0000868B} ,
		{0x0010a648, 0x00000000} ,
		{0x0010a64c, 0x00829493} ,
		{0x0010a650, 0x00000000} ,
		{0x0010e000, 0} ,
		{0x0010e068, 0x000000A3} ,
		{0x0010e068, 0x000000A0} ,
		{0x0010e068, 0x00000095} ,
		{0x0010e068, 0x000000A6} ,
		{0x0010e068, 0x0000008C} ,
		{0x0010e068, 0x00000080} ,
		{0x0010e068, 0x00000081} ,
		{0x0010e068, 0x00000087} ,
		{0x0010e068, 0x00000088} ,
		{0x0010e068, 0x0000008D} ,
		{0x0010e068, 0x00000082} ,
		{0x0010e068, 0x00000083} ,
		{0x0010e068, 0x00000089} ,
		{0x0010e068, 0x0000008A} ,
		{0x0010e068, 0x000000A2} ,
		{0x0010e068, 0x00000097} ,
		{0x0010e068, 0x00000092} ,
		{0x0010e068, 0x00000099} ,
		{0x0010e068, 0x0000009B} ,
		{0x0010e068, 0x0000009D} ,
		{0x0010e068, 0x0000009F} ,
		{0x0010e068, 0x000000A1} ,
		{0x0010e068, 0x00000096} ,
		{0x0010e068, 0x00000091} ,
		{0x0010e068, 0x00000098} ,
		{0x0010e068, 0x0000009A} ,
		{0x0010e068, 0x0000009C} ,
		{0x0010e068, 0x0000009E} ,
		{0x0010e000, 0} ,
		{0x0010e004, 0x0000008E},
};

static void gp10b_pmu_load_multiple_falcons(struct gk20a *g, u32 falconidmask,
					 u32 flags)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	nvgpu_log_fn(g, " ");

	gp10b_dbg_pmu(g, "wprinit status = %x\n", g->pmu_lsf_pmu_wpr_init_done);
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
		cmd.cmd.acr.boot_falcons.wprvirtualbase.lo = 0x0;
		cmd.cmd.acr.boot_falcons.wprvirtualbase.hi = 0x0;
		gp10b_dbg_pmu(g, "PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS:%x\n",
				falconidmask);
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_fecs_boot_acr_msg, pmu, &seq, ~0);
	}

	nvgpu_log_fn(g, "done");
	return;
}

int gp10b_load_falcon_ucode(struct gk20a *g, u32 falconidmask)
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
	gp10b_pmu_load_multiple_falcons(g, falconidmask, flags);
	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&g->pmu_lsf_loaded_falcon_id, falconidmask);
	if (g->pmu_lsf_loaded_falcon_id != falconidmask) {
		return -ETIMEDOUT;
	}
	return 0;
}

static void pmu_handle_gr_param_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	nvgpu_log_fn(g, " ");

	if (status != 0) {
		nvgpu_err(g, "GR PARAM cmd aborted");
		/* TBD: disable ELPG */
		return;
	}

	gp10b_dbg_pmu(g, "GR PARAM is acknowledged from PMU %x \n",
			msg->msg.pg.msg_type);

	return;
}

int gp10b_pg_gr_init(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;

	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		memset(&cmd, 0, sizeof(struct pmu_cmd));
		cmd.hdr.unit_id = PMU_UNIT_PG;
		cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_pg_cmd_gr_init_param_v2);
		cmd.cmd.pg.gr_init_param_v2.cmd_type =
				PMU_PG_CMD_ID_PG_PARAM;
		cmd.cmd.pg.gr_init_param_v2.sub_cmd_id =
				PMU_PG_PARAM_CMD_GR_INIT_PARAM;
		cmd.cmd.pg.gr_init_param_v2.featuremask =
				NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING;
		cmd.cmd.pg.gr_init_param_v2.ldiv_slowdown_factor =
				g->ldiv_slowdown_factor;

		gp10b_dbg_pmu(g, "cmd post PMU_PG_CMD_ID_PG_PARAM ");
		nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
				pmu_handle_gr_param_msg, pmu, &seq, ~0);

	} else {
		return -EINVAL;
	}

	return 0;
}

void gp10b_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_pg_stats_v1 stats;

	nvgpu_flcn_copy_from_dmem(pmu->flcn,
		pmu->stat_dmem_offset[pg_engine_id],
		(u8 *)&stats, sizeof(struct pmu_pg_stats_v1), 0);

	pg_stat_data->ingating_time = stats.total_sleep_timeus;
	pg_stat_data->ungating_time = stats.total_nonsleep_timeus;
	pg_stat_data->gating_cnt = stats.entry_count;
	pg_stat_data->avg_entry_latency_us = stats.entrylatency_avgus;
	pg_stat_data->avg_exit_latency_us = stats.exitlatency_avgus;
}

int gp10b_pmu_setup_elpg(struct gk20a *g)
{
	int ret = 0;
	u32 reg_writes;
	u32 index;

	nvgpu_log_fn(g, " ");

	if (g->can_elpg && g->elpg_enabled) {
		reg_writes = ((sizeof(_pginitseq_gp10b) /
				sizeof((_pginitseq_gp10b)[0])));
		/* Initialize registers with production values*/
		for (index = 0; index < reg_writes; index++) {
			gk20a_writel(g, _pginitseq_gp10b[index].regaddr,
				_pginitseq_gp10b[index].writeval);
		}
	}

	nvgpu_log_fn(g, "done");
	return ret;
}

void gp10b_write_dmatrfbase(struct gk20a *g, u32 addr)
{
	gk20a_writel(g, pwr_falcon_dmatrfbase_r(),
				addr);
	gk20a_writel(g, pwr_falcon_dmatrfbase1_r(),
				0x0);
}

bool gp10b_is_lazy_bootstrap(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

bool gp10b_is_priv_load(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = true;
		break;
	default:
		break;
	}

	return enable_status;
}

bool gp10b_is_pmu_supported(struct gk20a *g)
{
	return true;
}
