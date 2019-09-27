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
#include <nvgpu/gk20a.h>

#include "gp106/bios_gp106.h"
#include "pstate/pstate.h"
#include "lpwr/rppg.h"

static void pmu_handle_rppg_init_msg(struct gk20a *g, struct pmu_msg *msg,
	void *param, u32 handle, u32 status)
{
	u32 *success = param;

	if (status == 0) {
		switch (msg->msg.pg.rppg_msg.cmn.msg_id) {
		case NV_PMU_RPPG_MSG_ID_INIT_CTRL_ACK:
			*success = 1;
			nvgpu_pmu_dbg(g, "RPPG is acknowledged from PMU %x",
				msg->msg.pg.msg_type);
			break;
		}
	}

	nvgpu_pmu_dbg(g, "RPPG is acknowledged from PMU %x",
				msg->msg.pg.msg_type);
}

static u32 rppg_send_cmd(struct gk20a *g, struct nv_pmu_rppg_cmd *prppg_cmd)
{
	struct pmu_cmd cmd;
	u32 seq;
	u32 status = 0;
	u32 success = 0;

	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = PMU_UNIT_PG;
	cmd.hdr.size   = PMU_CMD_HDR_SIZE +
			sizeof(struct nv_pmu_rppg_cmd);

	cmd.cmd.pg.rppg_cmd.cmn.cmd_type = PMU_PMU_PG_CMD_ID_RPPG;
	cmd.cmd.pg.rppg_cmd.cmn.cmd_id   = prppg_cmd->cmn.cmd_id;

	switch (prppg_cmd->cmn.cmd_id) {
	case NV_PMU_RPPG_CMD_ID_INIT:
		break;
	case NV_PMU_RPPG_CMD_ID_INIT_CTRL:
		cmd.cmd.pg.rppg_cmd.init_ctrl.ctrl_id =
			prppg_cmd->init_ctrl.ctrl_id;
		cmd.cmd.pg.rppg_cmd.init_ctrl.domain_id =
			prppg_cmd->init_ctrl.domain_id;
		break;
	case NV_PMU_RPPG_CMD_ID_STATS_RESET:
		cmd.cmd.pg.rppg_cmd.stats_reset.ctrl_id =
			prppg_cmd->stats_reset.ctrl_id;
		break;
	default:
		nvgpu_err(g, "Inivalid RPPG command %d",
			prppg_cmd->cmn.cmd_id);
		return -1;
	}

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_HPQ,
			pmu_handle_rppg_init_msg, &success, &seq, ~0);
	if (status) {
		nvgpu_err(g, "Unable to submit parameter command %d",
			prppg_cmd->cmn.cmd_id);
		goto exit;
	}

	if (prppg_cmd->cmn.cmd_id == NV_PMU_RPPG_CMD_ID_INIT_CTRL) {
		pmu_wait_message_cond(&g->pmu, gk20a_get_gr_idle_timeout(g),
			&success, 1);
		if (success == 0) {
			status = -EINVAL;
			nvgpu_err(g, "Ack for the parameter command %x",
				prppg_cmd->cmn.cmd_id);
		}
	}

exit:
	return status;
}

static u32 rppg_init(struct gk20a *g)
{
	struct nv_pmu_rppg_cmd rppg_cmd;

	rppg_cmd.init.cmd_id = NV_PMU_RPPG_CMD_ID_INIT;

	return rppg_send_cmd(g, &rppg_cmd);
}

static u32 rppg_ctrl_init(struct gk20a *g, u8 ctrl_id)
{
	struct nv_pmu_rppg_cmd rppg_cmd;

	rppg_cmd.init_ctrl.cmd_id  = NV_PMU_RPPG_CMD_ID_INIT_CTRL;
	rppg_cmd.init_ctrl.ctrl_id = ctrl_id;

	switch (ctrl_id) {
	case NV_PMU_RPPG_CTRL_ID_GR:
	case NV_PMU_RPPG_CTRL_ID_MS:
		rppg_cmd.init_ctrl.domain_id = NV_PMU_RPPG_DOMAIN_ID_GFX;
		break;
	}

	return rppg_send_cmd(g, &rppg_cmd);
}

u32 init_rppg(struct gk20a *g)
{
	u32 status;

	status = rppg_init(g);
	if (status != 0) {
		nvgpu_err(g,
			"Failed to initialize RPPG in PMU: 0x%08x", status);
		return status;
	}


	status = rppg_ctrl_init(g, NV_PMU_RPPG_CTRL_ID_GR);
	if (status != 0) {
		nvgpu_err(g,
			"Failed to initialize RPPG_CTRL: GR in PMU: 0x%08x",
			status);
		return status;
	}

	status = rppg_ctrl_init(g, NV_PMU_RPPG_CTRL_ID_MS);
	if (status != 0) {
		nvgpu_err(g,
			"Failed to initialize RPPG_CTRL: MS in PMU: 0x%08x",
			status);
		return status;
	}

	return status;
}
