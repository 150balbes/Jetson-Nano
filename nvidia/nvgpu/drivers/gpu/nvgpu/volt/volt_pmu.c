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
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/gk20a.h>

#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "gp106/bios_gp106.h"
#include "ctrl/ctrlvolt.h"
#include "ctrl/ctrlperf.h"

#include "volt.h"

#define RAIL_COUNT_GP 2
#define RAIL_COUNT_GV 1

struct volt_rpc_pmucmdhandler_params {
	struct nv_pmu_volt_rpc *prpc_call;
	u32 success;
};

static void volt_rpc_pmucmdhandler(struct gk20a *g, struct pmu_msg *msg,
				  void *param, u32 handle, u32 status)
{
	struct volt_rpc_pmucmdhandler_params *phandlerparams =
		(struct volt_rpc_pmucmdhandler_params *)param;

	nvgpu_log_info(g, " ");

	if (msg->msg.volt.msg_type != NV_PMU_VOLT_MSG_ID_RPC) {
		nvgpu_err(g, "unsupported msg for VOLT RPC %x",
			msg->msg.volt.msg_type);
		return;
	}

	if (phandlerparams->prpc_call->b_supported) {
		phandlerparams->success = 1;
	}
}


static u32 volt_pmu_rpc_execute(struct gk20a *g,
	struct nv_pmu_volt_rpc *prpc_call)
{
	struct pmu_cmd cmd;
	struct pmu_msg msg;
	struct pmu_payload payload;
	u32 status = 0;
	u32 seqdesc;
	struct volt_rpc_pmucmdhandler_params handler;

	memset(&payload, 0, sizeof(struct pmu_payload));
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	memset(&msg, 0, sizeof(struct pmu_msg));
	memset(&handler, 0, sizeof(struct volt_rpc_pmucmdhandler_params));

	cmd.hdr.unit_id = PMU_UNIT_VOLT;
	cmd.hdr.size = (u32)sizeof(struct nv_pmu_volt_cmd) +
					(u32)sizeof(struct pmu_hdr);
	cmd.cmd.volt.cmd_type = NV_PMU_VOLT_CMD_ID_RPC;
	msg.hdr.size = sizeof(struct pmu_msg);

	payload.in.buf = (u8 *)prpc_call;
	payload.in.size = (u32)sizeof(struct nv_pmu_volt_rpc);
	payload.in.fb_size = PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED;
	payload.in.offset = NV_PMU_VOLT_CMD_RPC_ALLOC_OFFSET;

	payload.out.buf = (u8 *)prpc_call;
	payload.out.size = (u32)sizeof(struct nv_pmu_volt_rpc);
	payload.out.fb_size = PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED;
	payload.out.offset = NV_PMU_VOLT_MSG_RPC_ALLOC_OFFSET;

	handler.prpc_call = prpc_call;
	handler.success = 0;

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload,
			PMU_COMMAND_QUEUE_LPQ,
			volt_rpc_pmucmdhandler, (void *)&handler,
			&seqdesc, ~0);
	if (status) {
		nvgpu_err(g, "unable to post volt RPC cmd %x",
			cmd.cmd.volt.cmd_type);
		goto volt_pmu_rpc_execute;
	}

	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&handler.success, 1);

	if (handler.success == 0U) {
		status = -EINVAL;
		nvgpu_err(g, "rpc call to volt failed");
	}

volt_pmu_rpc_execute:
	return status;
}

u32 nvgpu_volt_send_load_cmd_to_pmu_gp10x(struct gk20a *g)
{
	struct nv_pmu_volt_rpc rpc_call = { 0 };
	u32 status = 0;

	rpc_call.function = NV_PMU_VOLT_RPC_ID_LOAD;

	status =  volt_pmu_rpc_execute(g, &rpc_call);
	if (status) {
		nvgpu_err(g,
			"Error while executing LOAD RPC: status = 0x%08x.",
			status);
	}

	return status;
}

u32 nvgpu_volt_send_load_cmd_to_pmu_gv10x(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_volt_load rpc;
	u32 status = 0;

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_volt_load));
	PMU_RPC_EXECUTE(status, pmu, VOLT, LOAD, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x",
			status);
	}

	return status;
}

u32 nvgpu_volt_rail_get_voltage_gp10x(struct gk20a *g,
	u8 volt_domain, u32 *pvoltage_uv)
{
	struct nv_pmu_volt_rpc rpc_call = { 0 };
	u32 status  = 0;
	u8 rail_idx;

	rail_idx = volt_rail_volt_domain_convert_to_idx(g, volt_domain);
	if ((rail_idx == CTRL_VOLT_RAIL_INDEX_INVALID) ||
		(!VOLT_RAIL_INDEX_IS_VALID(&g->perf_pmu.volt, rail_idx))) {
		nvgpu_err(g,
			"failed: volt_domain = %d, voltage rail table = %d.",
			volt_domain, rail_idx);
		return -EINVAL;
	}

	/* Set RPC parameters. */
	rpc_call.function = NV_PMU_VOLT_RPC_ID_VOLT_RAIL_GET_VOLTAGE;
	rpc_call.params.volt_rail_get_voltage.rail_idx = rail_idx;

	/* Execute the voltage get request via PMU RPC. */
	status = volt_pmu_rpc_execute(g, &rpc_call);
	if (status) {
		nvgpu_err(g,
			"Error while executing volt_rail_get_voltage rpc");
		return status;
	}

	/* Copy out the current voltage. */
	*pvoltage_uv = rpc_call.params.volt_rail_get_voltage.voltage_uv;

	return status;
}

u32 nvgpu_volt_rail_get_voltage_gv10x(struct gk20a *g,
	u8 volt_domain, u32 *pvoltage_uv)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_volt_volt_rail_get_voltage rpc;
	u32 status  = 0;
	u8 rail_idx;

	rail_idx = volt_rail_volt_domain_convert_to_idx(g, volt_domain);
	if ((rail_idx == CTRL_VOLT_RAIL_INDEX_INVALID) ||
		(!VOLT_RAIL_INDEX_IS_VALID(&g->perf_pmu.volt, rail_idx))) {
		nvgpu_err(g,
			"failed: volt_domain = %d, voltage rail table = %d.",
			volt_domain, rail_idx);
		return -EINVAL;
	}

	memset(&rpc, 0,
		sizeof(struct nv_pmu_rpc_struct_volt_volt_rail_get_voltage));
	rpc.rail_idx = rail_idx;

	PMU_RPC_EXECUTE_CPB(status, pmu, VOLT, VOLT_RAIL_GET_VOLTAGE, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x",
			status);
	}

	*pvoltage_uv = rpc.voltage_uv;

	return status;
}

static u32 volt_policy_set_voltage(struct gk20a *g, u8 client_id,
		struct ctrl_perf_volt_rail_list *prail_list)
{
	struct nv_pmu_volt_rpc rpc_call = { 0 };
	struct obj_volt *pvolt = &g->perf_pmu.volt;
	u32 status = 0;
	u8 policy_idx = CTRL_VOLT_POLICY_INDEX_INVALID;
	u8 i = 0;

	/* Sanity check input rail list. */
	for (i = 0; i < prail_list->num_rails; i++) {
		if ((prail_list->rails[i].volt_domain ==
				CTRL_VOLT_DOMAIN_INVALID) ||
			(prail_list->rails[i].voltage_uv ==
				NV_PMU_VOLT_VALUE_0V_IN_UV)) {
			nvgpu_err(g, "Invalid voltage domain or target");
			nvgpu_err(g, " client_id = %d, listEntry = %d",
					client_id, i);
			nvgpu_err(g, " volt_domain = %d, voltage_uv = %d uV.",
				prail_list->rails[i].volt_domain,
				prail_list->rails[i].voltage_uv);
			status = -EINVAL;
			goto exit;
		}
	}

	/* Convert the client ID to index. */
	if (client_id == CTRL_VOLT_POLICY_CLIENT_PERF_CORE_VF_SEQ) {
		policy_idx =
			pvolt->volt_policy_metadata.perf_core_vf_seq_policy_idx;
	}
	else {
		status = -EINVAL;
		goto exit;
	}

	/* Set RPC parameters. */
	rpc_call.function = NV_PMU_VOLT_RPC_ID_VOLT_POLICY_SET_VOLTAGE;
	rpc_call.params.volt_policy_voltage_data.policy_idx = policy_idx;
	memcpy(&rpc_call.params.volt_policy_voltage_data.rail_list, prail_list,
		(sizeof(struct ctrl_perf_volt_rail_list)));

	/* Execute the voltage change request via PMU RPC. */
	status = volt_pmu_rpc_execute(g, &rpc_call);
	if (status) {
		nvgpu_err(g,
			"Error while executing VOLT_POLICY_SET_VOLTAGE RPC");
	}

exit:
	return status;
}

static u32 volt_set_voltage_gv10x_rpc(struct gk20a *g, u8 client_id,
		struct ctrl_volt_volt_rail_list_v1 *prail_list)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_volt_volt_set_voltage rpc;
	int status = 0;

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_volt_volt_set_voltage));
	rpc.client_id = 0x1;
	rpc.rail_list = *prail_list;

	PMU_RPC_EXECUTE_CPB(status, pmu, VOLT, VOLT_SET_VOLTAGE, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x",
			status);
	}

	return status;
}

u32 nvgpu_volt_set_voltage_gv10x(struct gk20a *g, u32 logic_voltage_uv,
		u32 sram_voltage_uv)
{
	int status = 0;
	struct ctrl_volt_volt_rail_list_v1 rail_list = { 0 };

	rail_list.num_rails = RAIL_COUNT_GV;
	rail_list.rails[0].rail_idx =
		volt_rail_volt_domain_convert_to_idx(g,
			CTRL_VOLT_DOMAIN_LOGIC);
	rail_list.rails[0].voltage_uv = logic_voltage_uv;
	rail_list.rails[0].voltage_min_noise_unaware_uv = logic_voltage_uv;

	status = volt_set_voltage_gv10x_rpc(g,
		CTRL_VOLT_POLICY_CLIENT_PERF_CORE_VF_SEQ, &rail_list);

	return status;
}

u32 nvgpu_volt_set_voltage_gp10x(struct gk20a *g, u32 logic_voltage_uv,
		u32 sram_voltage_uv)
{
	int status = 0;
	struct ctrl_perf_volt_rail_list rail_list = { 0 };

	rail_list.num_rails = RAIL_COUNT_GP;
	rail_list.rails[0].volt_domain = CTRL_VOLT_DOMAIN_LOGIC;
	rail_list.rails[0].voltage_uv = logic_voltage_uv;
	rail_list.rails[0].voltage_min_noise_unaware_uv = logic_voltage_uv;
	rail_list.rails[1].volt_domain = CTRL_VOLT_DOMAIN_SRAM;
	rail_list.rails[1].voltage_uv = sram_voltage_uv;
	rail_list.rails[1].voltage_min_noise_unaware_uv = sram_voltage_uv;

	status = volt_policy_set_voltage(g,
		CTRL_VOLT_POLICY_CLIENT_PERF_CORE_VF_SEQ, &rail_list);

	return status;
}

u32 volt_set_voltage(struct gk20a *g, u32 logic_voltage_uv, u32 sram_voltage_uv)
{
	return g->ops.pmu_ver.volt.volt_set_voltage(g,
		logic_voltage_uv, sram_voltage_uv);
}

u32 volt_get_voltage(struct gk20a *g, u32 volt_domain, u32 *voltage_uv)
{
	return g->ops.pmu_ver.volt.volt_get_voltage(g,
		volt_domain, voltage_uv);
}

static int volt_policy_set_noiseaware_vmin(struct gk20a *g,
		struct ctrl_volt_volt_rail_list *prail_list)
{
	struct nv_pmu_volt_rpc rpc_call = { 0 };
	u32 status = 0;

	/* Set RPC parameters. */
	rpc_call.function = NV_PMU_VOLT_RPC_ID_VOLT_RAIL_SET_NOISE_UNAWARE_VMIN;
	rpc_call.params.volt_rail_set_noise_unaware_vmin.num_rails =
			prail_list->num_rails;
	memcpy(&rpc_call.params.volt_rail_set_noise_unaware_vmin.rail_list,
		prail_list, (sizeof(struct ctrl_volt_volt_rail_list)));

	/* Execute the voltage change request via PMU RPC. */
	status = volt_pmu_rpc_execute(g, &rpc_call);
	if (status) {
		nvgpu_err(g,
			"Error while executing VOLT_POLICY_SET_VOLTAGE RPC");
		return -EINVAL;
	}

	return 0;
}

int volt_set_noiseaware_vmin(struct gk20a *g, u32 logic_voltage_uv,
	u32 sram_voltage_uv)
{
	int status = 0;
	struct ctrl_volt_volt_rail_list rail_list = { 0 };

	rail_list.num_rails = RAIL_COUNT_GP;
	rail_list.rails[0].rail_idx = 0;
	rail_list.rails[0].voltage_uv = logic_voltage_uv;
	rail_list.rails[1].rail_idx = 1;
	rail_list.rails[1].voltage_uv = sram_voltage_uv;

	status = volt_policy_set_noiseaware_vmin(g, &rail_list);

	return status;

}

