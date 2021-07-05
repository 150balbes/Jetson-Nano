/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/barrier.h>
#include <nvgpu/bug.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

/* state transition :
 * OFF => [OFF_ON_PENDING optional] => ON_PENDING => ON => OFF
 * ON => OFF is always synchronized
 */
/* elpg is off */
#define PMU_ELPG_STAT_OFF	0U
/* elpg is on */
#define PMU_ELPG_STAT_ON	1U
/* elpg is off, ALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_ON_PENDING	2U
/* elpg is on, DISALLOW cmd has been sent, wait for ack */
#define PMU_ELPG_STAT_OFF_PENDING	3U
/* elpg is off, caller has requested on, but ALLOW
 * cmd hasn't been sent due to ENABLE_ALLOW delay
 */
#define PMU_ELPG_STAT_OFF_ON_PENDING	4U

#define PMU_PGENG_GR_BUFFER_IDX_INIT	(0)
#define PMU_PGENG_GR_BUFFER_IDX_ZBC		(1)
#define PMU_PGENG_GR_BUFFER_IDX_FECS	(2)

static void pmu_handle_pg_elpg_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct nvgpu_pmu *pmu = param;
	struct pmu_pg_msg_elpg_msg *elpg_msg = &msg->msg.pg.elpg_msg;

	nvgpu_log_fn(g, " ");

	if (status != 0U) {
		nvgpu_err(g, "ELPG cmd aborted");
		/* TBD: disable ELPG */
		return;
	}

	switch (elpg_msg->msg) {
	case PMU_PG_ELPG_MSG_INIT_ACK:
		nvgpu_pmu_dbg(g, "INIT_PG is ack from PMU, eng - %d",
			elpg_msg->engine_id);
		break;
	case PMU_PG_ELPG_MSG_ALLOW_ACK:
		nvgpu_pmu_dbg(g, "ALLOW is ack from PMU, eng - %d",
			elpg_msg->engine_id);
		if (elpg_msg->engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
			pmu->mscg_transition_state = PMU_ELPG_STAT_ON;
		} else {
			pmu->elpg_stat = PMU_ELPG_STAT_ON;
		}
		break;
	case PMU_PG_ELPG_MSG_DISALLOW_ACK:
		nvgpu_pmu_dbg(g, "DISALLOW is ack from PMU, eng - %d",
			elpg_msg->engine_id);

		if (elpg_msg->engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
			pmu->mscg_transition_state = PMU_ELPG_STAT_OFF;
		} else {
			pmu->elpg_stat = PMU_ELPG_STAT_OFF;
		}

		if (pmu->pmu_state == PMU_STATE_ELPG_BOOTING) {
			if (g->ops.pmu.pmu_pg_engines_feature_list != NULL &&
				g->ops.pmu.pmu_pg_engines_feature_list(g,
					PMU_PG_ELPG_ENGINE_ID_GRAPHICS) !=
				NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING) {
				pmu->initialized = true;
				nvgpu_pmu_state_change(g, PMU_STATE_STARTED,
					true);
				WRITE_ONCE(pmu->mscg_stat, PMU_MSCG_DISABLED);
				/* make status visible */
				nvgpu_smp_mb();
			} else {
				nvgpu_pmu_state_change(g, PMU_STATE_ELPG_BOOTED,
					true);
			}
		}
		break;
	default:
		nvgpu_err(g,
			"unsupported ELPG message : 0x%04x", elpg_msg->msg);
	}
}

/* PG enable/disable */
int nvgpu_pmu_pg_global_enable(struct gk20a *g, u32 enable_pg)
{
	u32 status = 0;

	if (enable_pg == true) {
		if (g->ops.pmu.pmu_pg_engines_feature_list != NULL &&
			g->ops.pmu.pmu_pg_engines_feature_list(g,
				PMU_PG_ELPG_ENGINE_ID_GRAPHICS) !=
			NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING) {
			if (g->ops.pmu.pmu_lpwr_enable_pg) {
				status = g->ops.pmu.pmu_lpwr_enable_pg(g,
						true);
			}
		} else if (g->support_pmu && g->can_elpg) {
			status = nvgpu_pmu_enable_elpg(g);
		}
	} else if (enable_pg == false) {
		if (g->ops.pmu.pmu_pg_engines_feature_list != NULL &&
			g->ops.pmu.pmu_pg_engines_feature_list(g,
				PMU_PG_ELPG_ENGINE_ID_GRAPHICS) !=
			NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING) {
			if (g->ops.pmu.pmu_lpwr_disable_pg) {
				status = g->ops.pmu.pmu_lpwr_disable_pg(g,
						true);
			}
		} else if (g->support_pmu && g->can_elpg) {
			status = nvgpu_pmu_disable_elpg(g);
		}
	}

	return status;
}

static int pmu_enable_elpg_locked(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq, status;

	nvgpu_log_fn(g, " ");

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
		sizeof(struct pmu_pg_cmd_elpg_cmd);
	cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
	cmd.cmd.pg.elpg_cmd.engine_id = pg_engine_id;
	cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_ALLOW;

	/* no need to wait ack for ELPG enable but set
	* pending to sync with follow up ELPG disable
	*/
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		pmu->elpg_stat = PMU_ELPG_STAT_ON_PENDING;
	} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
		pmu->mscg_transition_state = PMU_ELPG_STAT_ON_PENDING;
	}

	nvgpu_pmu_dbg(g, "cmd post PMU_PG_ELPG_CMD_ALLOW");
	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL,
		PMU_COMMAND_QUEUE_HPQ, pmu_handle_pg_elpg_msg,
		pmu, &seq, ~0);
	WARN_ON(status != 0U);

	nvgpu_log_fn(g, "done");
	return 0;
}

int nvgpu_pmu_enable_elpg(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct gr_gk20a *gr = &g->gr;
	u32 pg_engine_id;
	u32 pg_engine_id_list = 0;

	int ret = 0;

	nvgpu_log_fn(g, " ");

	if (!g->support_pmu) {
		return ret;
	}

	nvgpu_mutex_acquire(&pmu->elpg_mutex);

	pmu->elpg_refcnt++;
	if (pmu->elpg_refcnt <= 0) {
		goto exit_unlock;
	}

	/* something is not right if we end up in following code path */
	if (unlikely(pmu->elpg_refcnt > 1)) {
		nvgpu_warn(g,
			"%s(): possible elpg refcnt mismatch. elpg refcnt=%d",
			__func__, pmu->elpg_refcnt);
		WARN_ON(true);
	}

	/* do NOT enable elpg until golden ctx is created,
	 * which is related with the ctx that ELPG save and restore.
	*/
	if (unlikely(!gr->ctx_vars.golden_image_initialized)) {
		goto exit_unlock;
	}

	/* return if ELPG is already on or on_pending or off_on_pending */
	if (pmu->elpg_stat != PMU_ELPG_STAT_OFF) {
		goto exit_unlock;
	}

	if (g->ops.pmu.pmu_pg_supported_engines_list) {
		pg_engine_id_list = g->ops.pmu.pmu_pg_supported_engines_list(g);
	}

	for (pg_engine_id = PMU_PG_ELPG_ENGINE_ID_GRAPHICS;
		pg_engine_id < PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE;
		pg_engine_id++) {

		if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS &&
			pmu->mscg_stat == PMU_MSCG_DISABLED) {
			continue;
		}

		if (BIT(pg_engine_id) & pg_engine_id_list) {
			ret = pmu_enable_elpg_locked(g, pg_engine_id);
		}
	}

exit_unlock:
	nvgpu_mutex_release(&pmu->elpg_mutex);
	nvgpu_log_fn(g, "done");
	return ret;
}

int nvgpu_pmu_disable_elpg(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;
	int ret = 0;
	u32 pg_engine_id;
	u32 pg_engine_id_list = 0;
	u32 *ptr = NULL;

	nvgpu_log_fn(g, " ");

	if (g->ops.pmu.pmu_pg_supported_engines_list) {
		pg_engine_id_list = g->ops.pmu.pmu_pg_supported_engines_list(g);
	}

	if (!g->support_pmu) {
		return ret;
	}

	nvgpu_mutex_acquire(&pmu->elpg_mutex);

	pmu->elpg_refcnt--;
	if (pmu->elpg_refcnt > 0) {
		nvgpu_warn(g,
			"%s(): possible elpg refcnt mismatch. elpg refcnt=%d",
			__func__, pmu->elpg_refcnt);
		WARN_ON(true);
		ret = 0;
		goto exit_unlock;
	}

	/* cancel off_on_pending and return */
	if (pmu->elpg_stat == PMU_ELPG_STAT_OFF_ON_PENDING) {
		pmu->elpg_stat = PMU_ELPG_STAT_OFF;
		ret = 0;
		goto exit_reschedule;
	}
	/* wait if on_pending */
	else if (pmu->elpg_stat == PMU_ELPG_STAT_ON_PENDING) {

		pmu_wait_message_cond(pmu, gk20a_get_gr_idle_timeout(g),
				      &pmu->elpg_stat, PMU_ELPG_STAT_ON);

		if (pmu->elpg_stat != PMU_ELPG_STAT_ON) {
			nvgpu_err(g, "ELPG_ALLOW_ACK failed, elpg_stat=%d",
				pmu->elpg_stat);
			nvgpu_pmu_dump_elpg_stats(pmu);
			nvgpu_pmu_dump_falcon_stats(pmu);
			ret = -EBUSY;
			goto exit_unlock;
		}
	}
	/* return if ELPG is already off */
	else if (pmu->elpg_stat != PMU_ELPG_STAT_ON) {
		ret = 0;
		goto exit_reschedule;
	}

	for (pg_engine_id = PMU_PG_ELPG_ENGINE_ID_GRAPHICS;
		pg_engine_id < PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE;
		pg_engine_id++) {

		if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS &&
			pmu->mscg_stat == PMU_MSCG_DISABLED) {
			continue;
		}

		if (BIT(pg_engine_id) & pg_engine_id_list) {
			memset(&cmd, 0, sizeof(struct pmu_cmd));
			cmd.hdr.unit_id = PMU_UNIT_PG;
			cmd.hdr.size = PMU_CMD_HDR_SIZE +
				sizeof(struct pmu_pg_cmd_elpg_cmd);
			cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
			cmd.cmd.pg.elpg_cmd.engine_id = pg_engine_id;
			cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_DISALLOW;

			if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
				pmu->elpg_stat = PMU_ELPG_STAT_OFF_PENDING;
			} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
				pmu->mscg_transition_state =
					PMU_ELPG_STAT_OFF_PENDING;
			}
			if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
				ptr = &pmu->elpg_stat;
			} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
				ptr = &pmu->mscg_transition_state;
			}

			nvgpu_pmu_dbg(g, "cmd post PMU_PG_ELPG_CMD_DISALLOW");
			nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL,
				PMU_COMMAND_QUEUE_HPQ, pmu_handle_pg_elpg_msg,
				pmu, &seq, ~0);

			pmu_wait_message_cond(pmu,
				gk20a_get_gr_idle_timeout(g),
				ptr, PMU_ELPG_STAT_OFF);
			if (*ptr != PMU_ELPG_STAT_OFF) {
				nvgpu_err(g, "ELPG_DISALLOW_ACK failed");
					nvgpu_pmu_dump_elpg_stats(pmu);
					nvgpu_pmu_dump_falcon_stats(pmu);
				ret = -EBUSY;
				goto exit_unlock;
			}
		}
	}

exit_reschedule:
exit_unlock:
	nvgpu_mutex_release(&pmu->elpg_mutex);
	nvgpu_log_fn(g, "done");
	return ret;
}

int nvgpu_pmu_reenable_elpg(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	int ret = 0;

	nvgpu_log_fn(g, " ");

	if (!g->support_pmu) {
		return ret;
	}

	/* If pmu enabled, re-enable by first disabling, then
	 * enabling
	 */
	if (pmu->elpg_refcnt != 0) {
		ret = nvgpu_pmu_disable_elpg(g);
		if (ret != 0) {
			nvgpu_err(g, "failed disabling elpg");
			goto exit;
		}
		ret = nvgpu_pmu_enable_elpg(g);
		if (ret != 0) {
			nvgpu_err(g, "failed enabling elpg");
			goto exit;
		}
	}
exit:
	return ret;
}

/* PG init */
static void pmu_handle_pg_stat_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct nvgpu_pmu *pmu = param;

	nvgpu_log_fn(g, " ");

	if (status != 0U) {
		nvgpu_err(g, "ELPG cmd aborted");
		/* TBD: disable ELPG */
		return;
	}

	switch (msg->msg.pg.stat.sub_msg_id) {
	case PMU_PG_STAT_MSG_RESP_DMEM_OFFSET:
		nvgpu_pmu_dbg(g, "ALLOC_DMEM_OFFSET is acknowledged from PMU");
		pmu->stat_dmem_offset[msg->msg.pg.stat.engine_id] =
			msg->msg.pg.stat.data;
		break;
	default:
		break;
	}
}

static int pmu_pg_init_send(struct gk20a *g, u32 pg_engine_id)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 seq;
	int err = 0;

	nvgpu_log_fn(g, " ");

	g->ops.pmu.pmu_pg_idle_counter_config(g, pg_engine_id);

	if (g->ops.pmu.pmu_pg_init_param) {
		g->ops.pmu.pmu_pg_init_param(g, pg_engine_id);
	}

	/* init ELPG */
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_elpg_cmd);
	cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
	cmd.cmd.pg.elpg_cmd.engine_id = pg_engine_id;
	cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_INIT;

	nvgpu_pmu_dbg(g, "cmd post PMU_PG_ELPG_CMD_INIT");
	err = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_pg_elpg_msg, pmu, &seq, ~0);
	if (err) {
		nvgpu_err(g, "PMU_PG_ELPG_CMD_INIT cmd failed\n");
	}

	/* alloc dmem for powergating state log */
	pmu->stat_dmem_offset[pg_engine_id] = 0;
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_stat);
	cmd.cmd.pg.stat.cmd_type = PMU_PG_CMD_ID_PG_STAT;
	cmd.cmd.pg.stat.engine_id = pg_engine_id;
	cmd.cmd.pg.stat.sub_cmd_id = PMU_PG_STAT_CMD_ALLOC_DMEM;
	cmd.cmd.pg.stat.data = 0;

	nvgpu_pmu_dbg(g, "cmd post PMU_PG_STAT_CMD_ALLOC_DMEM");
	err = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			pmu_handle_pg_stat_msg, pmu, &seq, ~0);
	if (err) {
		nvgpu_err(g, "PMU_PG_STAT_CMD_ALLOC_DMEM cmd failed\n");
	}

	/* disallow ELPG initially
	 * PMU ucode requires a disallow cmd before allow cmd
	*/
	/* set for wait_event PMU_ELPG_STAT_OFF */
	if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_GRAPHICS) {
		pmu->elpg_stat = PMU_ELPG_STAT_OFF;
	} else if (pg_engine_id == PMU_PG_ELPG_ENGINE_ID_MS) {
		pmu->mscg_transition_state = PMU_ELPG_STAT_OFF;
	}
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_pg_cmd_elpg_cmd);
	cmd.cmd.pg.elpg_cmd.cmd_type = PMU_PG_CMD_ID_ELPG_CMD;
	cmd.cmd.pg.elpg_cmd.engine_id = pg_engine_id;
	cmd.cmd.pg.elpg_cmd.cmd = PMU_PG_ELPG_CMD_DISALLOW;

	nvgpu_pmu_dbg(g, "cmd post PMU_PG_ELPG_CMD_DISALLOW");
	err = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
		pmu_handle_pg_elpg_msg, pmu, &seq, ~0);
	if (err) {
		nvgpu_err(g, "PMU_PG_ELPG_CMD_DISALLOW cmd failed\n");
	}

	if (g->ops.pmu.pmu_pg_set_sub_feature_mask) {
		g->ops.pmu.pmu_pg_set_sub_feature_mask(g, pg_engine_id);
	}

	return 0;
}

int nvgpu_pmu_init_powergating(struct gk20a *g)
{
	u32 pg_engine_id;
	u32 pg_engine_id_list = 0;
	struct nvgpu_pmu *pmu = &g->pmu;

	nvgpu_log_fn(g, " ");

	if (g->ops.pmu.pmu_pg_supported_engines_list) {
		pg_engine_id_list = g->ops.pmu.pmu_pg_supported_engines_list(g);
	}

	gk20a_gr_wait_initialized(g);

	for (pg_engine_id = PMU_PG_ELPG_ENGINE_ID_GRAPHICS;
		pg_engine_id < PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE;
			pg_engine_id++) {

		if (BIT(pg_engine_id) & pg_engine_id_list) {
			if (pmu != NULL &&
			    pmu->pmu_state == PMU_STATE_INIT_RECEIVED) {
				nvgpu_pmu_state_change(g,
					PMU_STATE_ELPG_BOOTING, false);
			}
			pmu_pg_init_send(g, pg_engine_id);
		}
	}

	if (g->ops.pmu.pmu_pg_param_post_init) {
		g->ops.pmu.pmu_pg_param_post_init(g);
	}

	return 0;
}

static void pmu_handle_pg_buf_config_msg(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct nvgpu_pmu *pmu = param;
	struct pmu_pg_msg_eng_buf_stat *eng_buf_stat =
		&msg->msg.pg.eng_buf_stat;

	nvgpu_log_fn(g, " ");

	nvgpu_pmu_dbg(g,
		"reply PMU_PG_CMD_ID_ENG_BUF_LOAD PMU_PGENG_GR_BUFFER_IDX_FECS");
	if (status != 0U) {
		nvgpu_err(g, "PGENG cmd aborted");
		/* TBD: disable ELPG */
		return;
	}

	pmu->buf_loaded = (eng_buf_stat->status == PMU_PG_MSG_ENG_BUF_LOADED);
	if ((!pmu->buf_loaded) &&
		(pmu->pmu_state == PMU_STATE_LOADING_PG_BUF)) {
		nvgpu_err(g, "failed to load PGENG buffer");
	} else {
		nvgpu_pmu_state_change(g, pmu->pmu_state, true);
	}
}

int nvgpu_pmu_init_bind_fecs(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 desc;
	int err = 0;
	u32 gr_engine_id;

	nvgpu_log_fn(g, " ");

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
			g->ops.pmu_ver.pg_cmd_eng_buf_load_size(&cmd.cmd.pg);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_cmd_type(&cmd.cmd.pg,
			PMU_PG_CMD_ID_ENG_BUF_LOAD);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_engine_id(&cmd.cmd.pg,
			gr_engine_id);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_buf_idx(&cmd.cmd.pg,
			PMU_PGENG_GR_BUFFER_IDX_FECS);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_buf_size(&cmd.cmd.pg,
			pmu->pg_buf.size);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_base(&cmd.cmd.pg,
			u64_lo32(pmu->pg_buf.gpu_va));
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_offset(&cmd.cmd.pg,
			(u8)(pmu->pg_buf.gpu_va & 0xFFU));
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_idx(&cmd.cmd.pg,
			PMU_DMAIDX_VIRT);

	pmu->buf_loaded = false;
	nvgpu_pmu_dbg(g, "cmd post PMU_PG_CMD_ID_ENG_BUF_LOAD PMU_PGENG_GR_BUFFER_IDX_FECS");
	nvgpu_pmu_state_change(g, PMU_STATE_LOADING_PG_BUF, false);
	err = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			pmu_handle_pg_buf_config_msg, pmu, &desc, ~0);
	if (err) {
		nvgpu_err(g, "cmd LOAD PMU_PGENG_GR_BUFFER_IDX_FECS failed\n");
	}

	return err;
}

void nvgpu_pmu_setup_hw_load_zbc(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_cmd cmd;
	u32 desc;
	u32 gr_engine_id;
	int err = 0;

	gr_engine_id = gk20a_fifo_get_gr_engine_id(g);

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE +
			g->ops.pmu_ver.pg_cmd_eng_buf_load_size(&cmd.cmd.pg);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_cmd_type(&cmd.cmd.pg,
			PMU_PG_CMD_ID_ENG_BUF_LOAD);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_engine_id(&cmd.cmd.pg,
			gr_engine_id);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_buf_idx(&cmd.cmd.pg,
			PMU_PGENG_GR_BUFFER_IDX_ZBC);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_buf_size(&cmd.cmd.pg,
			pmu->seq_buf.size);
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_base(&cmd.cmd.pg,
			u64_lo32(pmu->seq_buf.gpu_va));
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_offset(&cmd.cmd.pg,
			(u8)(pmu->seq_buf.gpu_va & 0xFFU));
	g->ops.pmu_ver.pg_cmd_eng_buf_load_set_dma_idx(&cmd.cmd.pg,
			PMU_DMAIDX_VIRT);

	pmu->buf_loaded = false;
	nvgpu_pmu_dbg(g, "cmd post PMU_PG_CMD_ID_ENG_BUF_LOAD PMU_PGENG_GR_BUFFER_IDX_ZBC");
	nvgpu_pmu_state_change(g, PMU_STATE_LOADING_ZBC, false);
	err = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			pmu_handle_pg_buf_config_msg, pmu, &desc, ~0);
	if (err) {
		nvgpu_err(g, "CMD LOAD PMU_PGENG_GR_BUFFER_IDX_ZBC failed\n");
	}
}

/* stats */
int nvgpu_pmu_get_pg_stats(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	u32 pg_engine_id_list = 0;

	if (!pmu->initialized) {
		pg_stat_data->ingating_time = 0;
		pg_stat_data->ungating_time = 0;
		pg_stat_data->gating_cnt = 0;
		return 0;
	}

	if (g->ops.pmu.pmu_pg_supported_engines_list) {
		pg_engine_id_list = g->ops.pmu.pmu_pg_supported_engines_list(g);
	}

	if (BIT(pg_engine_id) & pg_engine_id_list) {
		g->ops.pmu.pmu_elpg_statistics(g, pg_engine_id,
			pg_stat_data);
	}

	return 0;
}

/* AELPG */
static void ap_callback_init_and_enable_ctrl(
		struct gk20a *g, struct pmu_msg *msg,
		void *param, u32 seq_desc, u32 status)
{
	/* Define p_ap (i.e pointer to pmu_ap structure) */
	WARN_ON(msg == NULL);

	if (status == 0U) {
		switch (msg->msg.pg.ap_msg.cmn.msg_id) {
		case PMU_AP_MSG_ID_INIT_ACK:
			nvgpu_pmu_dbg(g, "reply PMU_AP_CMD_ID_INIT");
			break;

		default:
			nvgpu_pmu_dbg(g,
			"%s: Invalid Adaptive Power Message: %x\n",
			__func__, msg->msg.pg.ap_msg.cmn.msg_id);
			break;
		}
	}
}

/* Send an Adaptive Power (AP) related command to PMU */
int nvgpu_pmu_ap_send_command(struct gk20a *g,
			union pmu_ap_cmd *p_ap_cmd, bool b_block)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	/* FIXME: where is the PG structure defined?? */
	u32 status = 0;
	struct pmu_cmd cmd;
	u32 seq;
	pmu_callback p_callback = NULL;

	memset(&cmd, 0, sizeof(struct pmu_cmd));

	/* Copy common members */
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(union pmu_ap_cmd);

	cmd.cmd.pg.ap_cmd.cmn.cmd_type = PMU_PG_CMD_ID_AP;
	cmd.cmd.pg.ap_cmd.cmn.cmd_id = p_ap_cmd->cmn.cmd_id;

	/* Copy other members of command */
	switch (p_ap_cmd->cmn.cmd_id) {
	case PMU_AP_CMD_ID_INIT:
		nvgpu_pmu_dbg(g, "cmd post PMU_AP_CMD_ID_INIT");
		cmd.cmd.pg.ap_cmd.init.pg_sampling_period_us =
			p_ap_cmd->init.pg_sampling_period_us;
		break;

	case PMU_AP_CMD_ID_INIT_AND_ENABLE_CTRL:
		nvgpu_pmu_dbg(g, "cmd post PMU_AP_CMD_ID_INIT_AND_ENABLE_CTRL");
		cmd.cmd.pg.ap_cmd.init_and_enable_ctrl.ctrl_id =
		p_ap_cmd->init_and_enable_ctrl.ctrl_id;
		memcpy(
		(void *)&(cmd.cmd.pg.ap_cmd.init_and_enable_ctrl.params),
			(void *)&(p_ap_cmd->init_and_enable_ctrl.params),
			sizeof(struct pmu_ap_ctrl_init_params));

		p_callback = ap_callback_init_and_enable_ctrl;
		break;

	case PMU_AP_CMD_ID_ENABLE_CTRL:
		nvgpu_pmu_dbg(g, "cmd post PMU_AP_CMD_ID_ENABLE_CTRL");
		cmd.cmd.pg.ap_cmd.enable_ctrl.ctrl_id =
			p_ap_cmd->enable_ctrl.ctrl_id;
		break;

	case PMU_AP_CMD_ID_DISABLE_CTRL:
		nvgpu_pmu_dbg(g, "cmd post PMU_AP_CMD_ID_DISABLE_CTRL");
		cmd.cmd.pg.ap_cmd.disable_ctrl.ctrl_id =
			p_ap_cmd->disable_ctrl.ctrl_id;
		break;

	case PMU_AP_CMD_ID_KICK_CTRL:
		nvgpu_pmu_dbg(g, "cmd post PMU_AP_CMD_ID_KICK_CTRL");
		cmd.cmd.pg.ap_cmd.kick_ctrl.ctrl_id =
			p_ap_cmd->kick_ctrl.ctrl_id;
		cmd.cmd.pg.ap_cmd.kick_ctrl.skip_count =
			p_ap_cmd->kick_ctrl.skip_count;
		break;

	default:
		nvgpu_pmu_dbg(g, "%s: Invalid Adaptive Power command %d\n",
			__func__, p_ap_cmd->cmn.cmd_id);
		return 0x2f;
	}

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			p_callback, pmu, &seq, ~0);

	if (status) {
		nvgpu_pmu_dbg(g,
			"%s: Unable to submit Adaptive Power Command %d\n",
			__func__, p_ap_cmd->cmn.cmd_id);
		goto err_return;
	}

	/* TODO: Implement blocking calls (b_block) */

err_return:
	return status;
}

int nvgpu_aelpg_init(struct gk20a *g)
{
	int status = 0;

	/* Remove reliance on app_ctrl field. */
	union pmu_ap_cmd ap_cmd;

	/* TODO: Check for elpg being ready? */
	ap_cmd.init.cmd_id = PMU_AP_CMD_ID_INIT;
	ap_cmd.init.pg_sampling_period_us = g->pmu.aelpg_param[0];

	status = nvgpu_pmu_ap_send_command(g, &ap_cmd, false);
	return status;
}

int nvgpu_aelpg_init_and_enable(struct gk20a *g, u8 ctrl_id)
{
	int status = 0;
	union pmu_ap_cmd ap_cmd;

	/* TODO: Probably check if ELPG is ready? */
	ap_cmd.init_and_enable_ctrl.cmd_id = PMU_AP_CMD_ID_INIT_AND_ENABLE_CTRL;
	ap_cmd.init_and_enable_ctrl.ctrl_id = ctrl_id;
	ap_cmd.init_and_enable_ctrl.params.min_idle_filter_us =
			g->pmu.aelpg_param[1];
	ap_cmd.init_and_enable_ctrl.params.min_target_saving_us =
			g->pmu.aelpg_param[2];
	ap_cmd.init_and_enable_ctrl.params.power_break_even_us =
			g->pmu.aelpg_param[3];
	ap_cmd.init_and_enable_ctrl.params.cycles_per_sample_max =
			g->pmu.aelpg_param[4];

	switch (ctrl_id) {
	case PMU_AP_CTRL_ID_GRAPHICS:
		break;
	default:
		break;
	}

	status = nvgpu_pmu_ap_send_command(g, &ap_cmd, true);
	return status;
}
