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

#include <nvgpu/kmem.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/pmu.h>
#include <nvgpu/gk20a.h>

#include "gp106/bios_gp106.h"

#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"

#include "pwrdev.h"
#include "pmgrpmu.h"

struct pmgr_pmucmdhandler_params {
	u32 success;
};

static void pmgr_pmucmdhandler(struct gk20a *g, struct pmu_msg *msg,
			void *param, u32 handle, u32 status)
{
	struct pmgr_pmucmdhandler_params *phandlerparams =
		(struct pmgr_pmucmdhandler_params *)param;

	if ((msg->msg.pmgr.msg_type != NV_PMU_PMGR_MSG_ID_SET_OBJECT) &&
		(msg->msg.pmgr.msg_type != NV_PMU_PMGR_MSG_ID_QUERY) &&
		(msg->msg.pmgr.msg_type != NV_PMU_PMGR_MSG_ID_LOAD)) {
		nvgpu_err(g, "unknow msg %x", msg->msg.pmgr.msg_type);
		return;
	}

	if (msg->msg.pmgr.msg_type == NV_PMU_PMGR_MSG_ID_SET_OBJECT) {
		if ((msg->msg.pmgr.set_object.b_success != 1) ||
			(msg->msg.pmgr.set_object.flcnstatus != 0U)) {
			nvgpu_err(g, "pmgr msg failed %x %x %x %x",
				msg->msg.pmgr.set_object.msg_type,
				msg->msg.pmgr.set_object.b_success,
				msg->msg.pmgr.set_object.flcnstatus,
				msg->msg.pmgr.set_object.object_type);
			return;
		}
	} else if (msg->msg.pmgr.msg_type == NV_PMU_PMGR_MSG_ID_QUERY) {
		if ((msg->msg.pmgr.query.b_success != 1) ||
			(msg->msg.pmgr.query.flcnstatus != 0U)) {
			nvgpu_err(g, "pmgr msg failed %x %x %x %x",
				msg->msg.pmgr.query.msg_type,
				msg->msg.pmgr.query.b_success,
				msg->msg.pmgr.query.flcnstatus,
				msg->msg.pmgr.query.cmd_type);
			return;
		}
	} else if (msg->msg.pmgr.msg_type == NV_PMU_PMGR_MSG_ID_LOAD) {
		if ((msg->msg.pmgr.query.b_success != 1) ||
			(msg->msg.pmgr.query.flcnstatus != 0U)) {
			nvgpu_err(g, "pmgr msg failed %x %x %x",
				msg->msg.pmgr.load.msg_type,
				msg->msg.pmgr.load.b_success,
				msg->msg.pmgr.load.flcnstatus);
			return;
		}
	}

	phandlerparams->success = 1;
}

static u32 pmgr_pmu_set_object(struct gk20a *g,
		u8 type,
		u16 dmem_size,
		u16 fb_size,
		void *pobj)
{
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	struct nv_pmu_pmgr_cmd_set_object *pcmd;
	u32 status;
	u32 seqdesc;
	struct pmgr_pmucmdhandler_params handlerparams;

	memset(&payload, 0, sizeof(struct pmu_payload));
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	memset(&handlerparams, 0, sizeof(struct pmgr_pmucmdhandler_params));

	cmd.hdr.unit_id = PMU_UNIT_PMGR;
	cmd.hdr.size = (u32)sizeof(struct nv_pmu_pmgr_cmd_set_object) +
			(u32)sizeof(struct pmu_hdr);;

	pcmd = &cmd.cmd.pmgr.set_object;
	pcmd->cmd_type = NV_PMU_PMGR_CMD_ID_SET_OBJECT;
	pcmd->object_type = type;

	payload.in.buf = pobj;
	payload.in.size = dmem_size;
	payload.in.fb_size = fb_size;
	payload.in.offset = NV_PMU_PMGR_SET_OBJECT_ALLOC_OFFSET;

	/* Setup the handler params to communicate back results.*/
	handlerparams.success = 0;

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload,
				PMU_COMMAND_QUEUE_LPQ,
				pmgr_pmucmdhandler,
				(void *)&handlerparams,
				&seqdesc, ~0);
	if (status) {
		nvgpu_err(g,
			"unable to post pmgr cmd for unit %x cmd id %x obj type %x",
			cmd.hdr.unit_id, pcmd->cmd_type, pcmd->object_type);
		goto exit;
	}

	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&handlerparams.success, 1);

	if (handlerparams.success == 0U) {
		nvgpu_err(g, "could not process cmd");
		status = -ETIMEDOUT;
		goto exit;
	}

exit:
	return status;
}

static u32 pmgr_send_i2c_device_topology_to_pmu(struct gk20a *g)
{
	struct nv_pmu_pmgr_i2c_device_desc_table i2c_desc_table;
	u32 idx = g->ina3221_dcb_index;
	u32 status = 0;

	/* INA3221 I2C device info */
	i2c_desc_table.dev_mask = (1UL << idx);

	/* INA3221 */
	i2c_desc_table.devices[idx].super.type = 0x4E;

	i2c_desc_table.devices[idx].dcb_index = idx;
	i2c_desc_table.devices[idx].i2c_address = g->ina3221_i2c_address;
	i2c_desc_table.devices[idx].i2c_flags = 0xC2F;
	i2c_desc_table.devices[idx].i2c_port = g->ina3221_i2c_port;

	/* Pass the table down the PMU as an object */
	status = pmgr_pmu_set_object(
				g,
				NV_PMU_PMGR_OBJECT_I2C_DEVICE_DESC_TABLE,
				(u16)sizeof(struct nv_pmu_pmgr_i2c_device_desc_table),
				PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED,
				&i2c_desc_table);

	if (status) {
		nvgpu_err(g, "pmgr_pmu_set_object failed %x",
			status);
	}

	return status;
}

static int pmgr_send_pwr_device_topology_to_pmu(struct gk20a *g)
{
	struct nv_pmu_pmgr_pwr_device_desc_table *pwr_desc_table;
	struct nv_pmu_pmgr_pwr_device_desc_table_header *ppwr_desc_header;
	int status = 0;

	/* Set the BA-device-independent HW information */
	pwr_desc_table = nvgpu_kzalloc(g, sizeof(*pwr_desc_table));
	if (!pwr_desc_table) {
		return -ENOMEM;
	}

	ppwr_desc_header = &(pwr_desc_table->hdr.data);
	ppwr_desc_header->ba_info.b_initialized_and_used = false;

	/* populate the table */
	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)&ppwr_desc_header->super,
			g->pmgr_pmu.pmgr_deviceobjs.super.super.objmask);

	status = boardobjgrp_pmudatainit_legacy(g,
			&g->pmgr_pmu.pmgr_deviceobjs.super.super,
			(struct nv_pmu_boardobjgrp_super *)pwr_desc_table);

	if (status) {
		nvgpu_err(g, "boardobjgrp_pmudatainit_legacy failed %x",
			status);
		goto exit;
	}

	/* Pass the table down the PMU as an object */
	status = pmgr_pmu_set_object(
				g,
				NV_PMU_PMGR_OBJECT_PWR_DEVICE_DESC_TABLE,
				(u16)sizeof(
				union nv_pmu_pmgr_pwr_device_dmem_size),
				(u16)sizeof(struct nv_pmu_pmgr_pwr_device_desc_table),
				pwr_desc_table);

	if (status) {
		nvgpu_err(g, "pmgr_pmu_set_object failed %x",
			status);
	}

exit:
	nvgpu_kfree(g, pwr_desc_table);
	return status;
}

static int pmgr_send_pwr_mointer_to_pmu(struct gk20a *g)
{
	struct nv_pmu_pmgr_pwr_monitor_pack *pwr_monitor_pack = NULL;
	struct nv_pmu_pmgr_pwr_channel_header *pwr_channel_hdr;
	struct nv_pmu_pmgr_pwr_chrelationship_header *pwr_chrelationship_header;
	u32 max_dmem_size;
	int status = 0;

	pwr_monitor_pack = nvgpu_kzalloc(g, sizeof(*pwr_monitor_pack));
	if (!pwr_monitor_pack) {
		return -ENOMEM;
	}

	/* Copy all the global settings from the RM copy */
	pwr_channel_hdr = &(pwr_monitor_pack->channels.hdr.data);
	*pwr_monitor_pack = g->pmgr_pmu.pmgr_monitorobjs.pmu_data;

	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)&pwr_channel_hdr->super,
			g->pmgr_pmu.pmgr_monitorobjs.pwr_channels.super.objmask);

	/* Copy in each channel */
	status = boardobjgrp_pmudatainit_legacy(g,
			&g->pmgr_pmu.pmgr_monitorobjs.pwr_channels.super,
			(struct nv_pmu_boardobjgrp_super *)&(pwr_monitor_pack->channels));

	if (status) {
		nvgpu_err(g, "boardobjgrp_pmudatainit_legacy failed %x",
			status);
		goto exit;
	}

	/* Copy in each channel relationship */
	pwr_chrelationship_header =  &(pwr_monitor_pack->ch_rels.hdr.data);

	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)&pwr_chrelationship_header->super,
			g->pmgr_pmu.pmgr_monitorobjs.pwr_ch_rels.super.objmask);

	pwr_channel_hdr->physical_channel_mask = g->pmgr_pmu.pmgr_monitorobjs.physical_channel_mask;
	pwr_channel_hdr->type = NV_PMU_PMGR_PWR_MONITOR_TYPE_NO_POLLING;

	status = boardobjgrp_pmudatainit_legacy(g,
		&g->pmgr_pmu.pmgr_monitorobjs.pwr_ch_rels.super,
		(struct nv_pmu_boardobjgrp_super *)&(pwr_monitor_pack->ch_rels));

	if (status) {
		nvgpu_err(g, "boardobjgrp_pmudatainit_legacy failed %x",
			status);
		goto exit;
	}

	/* Calculate the max Dmem buffer size */
	max_dmem_size = sizeof(union nv_pmu_pmgr_pwr_monitor_dmem_size);

	/* Pass the table down the PMU as an object */
	status = pmgr_pmu_set_object(
				g,
				NV_PMU_PMGR_OBJECT_PWR_MONITOR,
				(u16)max_dmem_size,
				(u16)sizeof(struct nv_pmu_pmgr_pwr_monitor_pack),
				pwr_monitor_pack);

	if (status) {
		nvgpu_err(g, "pmgr_pmu_set_object failed %x",
			status);
	}

exit:
	nvgpu_kfree(g, pwr_monitor_pack);
	return status;
}

static int pmgr_send_pwr_policy_to_pmu(struct gk20a *g)
{
	struct nv_pmu_pmgr_pwr_policy_pack *ppwrpack = NULL;
	struct pwr_policy *ppolicy = NULL;
	int status = 0;
	u8 indx;
	u32 max_dmem_size;

	ppwrpack = nvgpu_kzalloc(g, sizeof(struct nv_pmu_pmgr_pwr_policy_pack));
	if (!ppwrpack) {
		nvgpu_err(g, "pwr policy alloc failed %x",
			status);
		status = -ENOMEM;
		goto exit;
	}

	ppwrpack->policies.hdr.data.version = g->pmgr_pmu.pmgr_policyobjs.version;
	ppwrpack->policies.hdr.data.b_enabled = g->pmgr_pmu.pmgr_policyobjs.b_enabled;

	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)
			&ppwrpack->policies.hdr.data.super,
			g->pmgr_pmu.pmgr_policyobjs.pwr_policies.super.objmask);

	memset(&ppwrpack->policies.hdr.data.reserved_pmu_policy_mask,
			0,
			sizeof(ppwrpack->policies.hdr.data.reserved_pmu_policy_mask));

	ppwrpack->policies.hdr.data.base_sample_period =
			g->pmgr_pmu.pmgr_policyobjs.base_sample_period;
	ppwrpack->policies.hdr.data.min_client_sample_period =
			g->pmgr_pmu.pmgr_policyobjs.min_client_sample_period;
	ppwrpack->policies.hdr.data.low_sampling_mult =
			g->pmgr_pmu.pmgr_policyobjs.low_sampling_mult;

	memcpy(&ppwrpack->policies.hdr.data.global_ceiling,
			&g->pmgr_pmu.pmgr_policyobjs.global_ceiling,
			sizeof(struct nv_pmu_perf_domain_group_limits));

	memcpy(&ppwrpack->policies.hdr.data.semantic_policy_tbl,
			&g->pmgr_pmu.pmgr_policyobjs.policy_idxs,
			sizeof(g->pmgr_pmu.pmgr_policyobjs.policy_idxs));

	BOARDOBJGRP_FOR_EACH_INDEX_IN_MASK(32, indx,
			ppwrpack->policies.hdr.data.super.obj_mask.super.data[0]) {
		ppolicy = PMGR_GET_PWR_POLICY(g, indx);

		status = ((struct boardobj *)ppolicy)->pmudatainit(g, (struct boardobj *)ppolicy,
				(struct nv_pmu_boardobj *)&(ppwrpack->policies.policies[indx].data));
		if (status) {
			nvgpu_err(g, "pmudatainit failed %x indx %x",
				status, indx);
			status = -ENOMEM;
			goto exit;
		}
	}
	BOARDOBJGRP_FOR_EACH_INDEX_IN_MASK_END;

	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)
			&ppwrpack->policy_rels.hdr.data.super,
			g->pmgr_pmu.pmgr_policyobjs.pwr_policy_rels.super.objmask);

	boardobjgrpe32hdrset((struct nv_pmu_boardobjgrp *)
			&ppwrpack->violations.hdr.data.super,
			g->pmgr_pmu.pmgr_policyobjs.pwr_violations.super.objmask);

	max_dmem_size = sizeof(union nv_pmu_pmgr_pwr_policy_dmem_size);

	/* Pass the table down the PMU as an object */
	status = pmgr_pmu_set_object(
				g,
				NV_PMU_PMGR_OBJECT_PWR_POLICY,
				(u16)max_dmem_size,
				(u16)sizeof(struct nv_pmu_pmgr_pwr_policy_pack),
				ppwrpack);

	if (status) {
		nvgpu_err(g, "pmgr_pmu_set_object failed %x",
			status);
	}

exit:
	if (ppwrpack) {
		nvgpu_kfree(g, ppwrpack);
	}

	return status;
}

u32 pmgr_pmu_pwr_devices_query_blocking(
		struct gk20a *g,
		u32 pwr_dev_mask,
		struct nv_pmu_pmgr_pwr_devices_query_payload *ppayload)
{
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	struct nv_pmu_pmgr_cmd_pwr_devices_query *pcmd;
	u32 status;
	u32 seqdesc;
	struct pmgr_pmucmdhandler_params handlerparams;

	memset(&payload, 0, sizeof(struct pmu_payload));
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	memset(&handlerparams, 0, sizeof(struct pmgr_pmucmdhandler_params));

	cmd.hdr.unit_id = PMU_UNIT_PMGR;
	cmd.hdr.size = (u32)sizeof(struct nv_pmu_pmgr_cmd_pwr_devices_query) +
			(u32)sizeof(struct pmu_hdr);

	pcmd = &cmd.cmd.pmgr.pwr_dev_query;
	pcmd->cmd_type = NV_PMU_PMGR_CMD_ID_PWR_DEVICES_QUERY;
	pcmd->dev_mask = pwr_dev_mask;

	payload.out.buf = ppayload;
	payload.out.size = sizeof(struct nv_pmu_pmgr_pwr_devices_query_payload);
	payload.out.fb_size = PMU_CMD_SUBMIT_PAYLOAD_PARAMS_FB_SIZE_UNUSED;
	payload.out.offset = NV_PMU_PMGR_PWR_DEVICES_QUERY_ALLOC_OFFSET;

	/* Setup the handler params to communicate back results.*/
	handlerparams.success = 0;

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload,
				PMU_COMMAND_QUEUE_LPQ,
				pmgr_pmucmdhandler,
				(void *)&handlerparams,
				&seqdesc, ~0);
	if (status) {
		nvgpu_err(g,
			"unable to post pmgr query cmd for unit %x cmd id %x dev mask %x",
			cmd.hdr.unit_id, pcmd->cmd_type, pcmd->dev_mask);
		goto exit;
	}

	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&handlerparams.success, 1);

	if (handlerparams.success == 0U) {
		nvgpu_err(g, "could not process cmd");
		status = -ETIMEDOUT;
		goto exit;
	}

exit:
	return status;
}

static u32 pmgr_pmu_load_blocking(struct gk20a *g)
{
	struct pmu_cmd cmd = { {0} };
	struct nv_pmu_pmgr_cmd_load *pcmd;
	u32 status;
	u32 seqdesc;
	struct pmgr_pmucmdhandler_params handlerparams = {0};

	cmd.hdr.unit_id = PMU_UNIT_PMGR;
	cmd.hdr.size = (u32)sizeof(struct nv_pmu_pmgr_cmd_load) +
			(u32)sizeof(struct pmu_hdr);

	pcmd = &cmd.cmd.pmgr.load;
	pcmd->cmd_type = NV_PMU_PMGR_CMD_ID_LOAD;

	/* Setup the handler params to communicate back results.*/
	handlerparams.success = 0;

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL,
				PMU_COMMAND_QUEUE_LPQ,
				pmgr_pmucmdhandler,
				(void *)&handlerparams,
				&seqdesc, ~0);
	if (status) {
		nvgpu_err(g,
			"unable to post pmgr load cmd for unit %x cmd id %x",
			cmd.hdr.unit_id, pcmd->cmd_type);
		goto exit;
	}

	pmu_wait_message_cond(&g->pmu,
			gk20a_get_gr_idle_timeout(g),
			&handlerparams.success, 1);

	if (handlerparams.success == 0U) {
		nvgpu_err(g, "could not process cmd");
		status = -ETIMEDOUT;
		goto exit;
	}

exit:
	return status;
}

int pmgr_send_pmgr_tables_to_pmu(struct gk20a *g)
{
	int status = 0;

	status = pmgr_send_i2c_device_topology_to_pmu(g);

	if (status) {
		nvgpu_err(g,
			"pmgr_send_i2c_device_topology_to_pmu failed %x",
			status);
		goto exit;
	}

	if (!BOARDOBJGRP_IS_EMPTY(&g->pmgr_pmu.pmgr_deviceobjs.super.super)) {
		status = pmgr_send_pwr_device_topology_to_pmu(g);
		if (status) {
			nvgpu_err(g,
				"pmgr_send_pwr_device_topology_to_pmu failed %x",
				status);
			goto exit;
		}
	}

	if (!(BOARDOBJGRP_IS_EMPTY(
			&g->pmgr_pmu.pmgr_monitorobjs.pwr_channels.super)) ||
		!(BOARDOBJGRP_IS_EMPTY(
			&g->pmgr_pmu.pmgr_monitorobjs.pwr_ch_rels.super))) {
		status = pmgr_send_pwr_mointer_to_pmu(g);
		if (status) {
			nvgpu_err(g,
				"pmgr_send_pwr_mointer_to_pmu failed %x", status);
			goto exit;
		}
	}

	if (!(BOARDOBJGRP_IS_EMPTY(
			&g->pmgr_pmu.pmgr_policyobjs.pwr_policies.super)) ||
		!(BOARDOBJGRP_IS_EMPTY(
			&g->pmgr_pmu.pmgr_policyobjs.pwr_policy_rels.super)) ||
		!(BOARDOBJGRP_IS_EMPTY(
			&g->pmgr_pmu.pmgr_policyobjs.pwr_violations.super))) {
		status = pmgr_send_pwr_policy_to_pmu(g);
		if (status) {
			nvgpu_err(g,
				"pmgr_send_pwr_policy_to_pmu failed %x", status);
			goto exit;
		}
	}

		status = pmgr_pmu_load_blocking(g);
		if (status) {
			nvgpu_err(g,
				"pmgr_send_pwr_mointer_to_pmu failed %x", status);
			goto exit;
		}

exit:
	return status;
}
