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

#include <nvgpu/bios.h>
#include <nvgpu/gk20a.h>

#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "gp106/bios_gp106.h"
#include "ctrl/ctrlvolt.h"

#include "volt.h"

u8 volt_rail_volt_domain_convert_to_idx(struct gk20a *g, u8 volt_domain)
{
	switch (g->perf_pmu.volt.volt_rail_metadata.volt_domain_hal) {
	case CTRL_VOLT_DOMAIN_HAL_GP10X_SINGLE_RAIL:
		switch (volt_domain) {
		case CTRL_VOLT_DOMAIN_LOGIC:
			return 0;
		}
		break;
	case CTRL_VOLT_DOMAIN_HAL_GP10X_SPLIT_RAIL:
		switch (volt_domain) {
		case CTRL_VOLT_DOMAIN_LOGIC:
			return 0;
		case CTRL_VOLT_DOMAIN_SRAM:
			return 1;
		}
		break;
	}

	return CTRL_BOARDOBJ_IDX_INVALID;
}

u32 volt_rail_volt_dev_register(struct gk20a *g, struct voltage_rail
	*pvolt_rail, u8 volt_dev_idx, u8 operation_type)
{
	u32 status = 0;

	if (operation_type == CTRL_VOLT_DEVICE_OPERATION_TYPE_DEFAULT) {
		if (pvolt_rail->volt_dev_idx_default ==
				CTRL_BOARDOBJ_IDX_INVALID) {
			pvolt_rail->volt_dev_idx_default = volt_dev_idx;
		} else {
			status = -EINVAL;
			goto exit;
		}
	} else if (operation_type ==
		CTRL_VOLT_VOLT_DEVICE_OPERATION_TYPE_IPC_VMIN) {
		if (pvolt_rail->volt_dev_idx_ipc_vmin ==
			CTRL_BOARDOBJ_IDX_INVALID) {
			pvolt_rail->volt_dev_idx_ipc_vmin = volt_dev_idx;
			/*
			* Exit on purpose as we do not want to register
			* IPC_VMIN device against the rail to avoid
			* setting current voltage instead of
			* IPC Vmin voltage.
			*/
			goto exit;
		} else {
			status = -EINVAL;
			goto exit;
		}
	} else {
		goto exit;
	}

	status = boardobjgrpmask_bitset(&pvolt_rail->volt_dev_mask.super,
			volt_dev_idx);

exit:
	if (status) {
		nvgpu_err(g, "Failed to register VOLTAGE_DEVICE");
	}

	return status;
}

static u32 volt_rail_state_init(struct gk20a *g,
		struct voltage_rail *pvolt_rail)
{
	u32 status = 0;
	u32 i;

	pvolt_rail->volt_dev_idx_default = CTRL_BOARDOBJ_IDX_INVALID;

	for (i = 0; i < CTRL_VOLT_RAIL_VOLT_DELTA_MAX_ENTRIES; i++) {
		pvolt_rail->volt_delta_uv[i] = (int)NV_PMU_VOLT_VALUE_0V_IN_UV;
		g->perf_pmu.volt.volt_rail_metadata.ext_rel_delta_uv[i] =
			NV_PMU_VOLT_VALUE_0V_IN_UV;
	}

	pvolt_rail->volt_margin_limit_vfe_equ_mon_handle =
		NV_PMU_PERF_RPC_VFE_EQU_MONITOR_COUNT_MAX;
	pvolt_rail->rel_limit_vfe_equ_mon_handle =
		NV_PMU_PERF_RPC_VFE_EQU_MONITOR_COUNT_MAX;
	pvolt_rail->alt_rel_limit_vfe_equ_mon_handle =
		NV_PMU_PERF_RPC_VFE_EQU_MONITOR_COUNT_MAX;
	pvolt_rail->ov_limit_vfe_equ_mon_handle =
		NV_PMU_PERF_RPC_VFE_EQU_MONITOR_COUNT_MAX;

	status = boardobjgrpmask_e32_init(&pvolt_rail->volt_dev_mask, NULL);
	if (status) {
		nvgpu_err(g,
			"Failed to initialize BOARDOBJGRPMASK of VOLTAGE_DEVICEs");
	}

	return status;
}

static int volt_rail_init_pmudata_super(struct gk20a *g,
	struct boardobj *board_obj_ptr, struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct voltage_rail *prail;
	struct nv_pmu_volt_volt_rail_boardobj_set *rail_pmu_data;
	u32 i;

	nvgpu_log_info(g, " ");

	status = boardobj_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status) {
		return status;
	}

	prail = (struct voltage_rail *)board_obj_ptr;
	rail_pmu_data = (struct nv_pmu_volt_volt_rail_boardobj_set *)
		ppmudata;

	rail_pmu_data->rel_limit_vfe_equ_idx = prail->rel_limit_vfe_equ_idx;
	rail_pmu_data->alt_rel_limit_vfe_equ_idx =
			prail->alt_rel_limit_vfe_equ_idx;
	rail_pmu_data->ov_limit_vfe_equ_idx = prail->ov_limit_vfe_equ_idx;
	rail_pmu_data->vmin_limit_vfe_equ_idx = prail->vmin_limit_vfe_equ_idx;
	rail_pmu_data->volt_margin_limit_vfe_equ_idx =
			prail->volt_margin_limit_vfe_equ_idx;
	rail_pmu_data->pwr_equ_idx = prail->pwr_equ_idx;
	rail_pmu_data->volt_dev_idx_default = prail->volt_dev_idx_default;
	rail_pmu_data->volt_scale_exp_pwr_equ_idx =
			prail->volt_scale_exp_pwr_equ_idx;
	rail_pmu_data->volt_dev_idx_ipc_vmin = prail->volt_dev_idx_ipc_vmin;

	for (i = 0; i < CTRL_VOLT_RAIL_VOLT_DELTA_MAX_ENTRIES; i++) {
		rail_pmu_data->volt_delta_uv[i] = prail->volt_delta_uv[i] +
			(int)g->perf_pmu.volt.volt_rail_metadata.ext_rel_delta_uv[i];
	}

	status = boardobjgrpmask_export(&prail->volt_dev_mask.super,
				prail->volt_dev_mask.super.bitcount,
				&rail_pmu_data->volt_dev_mask.super);
	if (status) {
		nvgpu_err(g,
			"Failed to export BOARDOBJGRPMASK of VOLTAGE_DEVICEs");
	}

	nvgpu_log_info(g, "Done");

	return status;
}

static struct voltage_rail *construct_volt_rail(struct gk20a *g, void *pargs)
{
	struct boardobj *board_obj_ptr = NULL;
	struct voltage_rail *ptemp_rail = (struct voltage_rail *)pargs;
	struct voltage_rail  *board_obj_volt_rail_ptr = NULL;
	int status;

	nvgpu_log_info(g, " ");
	status = boardobj_construct_super(g, &board_obj_ptr,
		sizeof(struct voltage_rail), pargs);
	if (status) {
		return NULL;
	}

	board_obj_volt_rail_ptr = (struct voltage_rail *)board_obj_ptr;
	/* override super class interface */
	board_obj_ptr->pmudatainit = volt_rail_init_pmudata_super;

	board_obj_volt_rail_ptr->boot_voltage_uv =
			ptemp_rail->boot_voltage_uv;
	board_obj_volt_rail_ptr->rel_limit_vfe_equ_idx =
			ptemp_rail->rel_limit_vfe_equ_idx;
	board_obj_volt_rail_ptr->alt_rel_limit_vfe_equ_idx =
			ptemp_rail->alt_rel_limit_vfe_equ_idx;
	board_obj_volt_rail_ptr->ov_limit_vfe_equ_idx =
			ptemp_rail->ov_limit_vfe_equ_idx;
	board_obj_volt_rail_ptr->pwr_equ_idx =
			ptemp_rail->pwr_equ_idx;
	board_obj_volt_rail_ptr->boot_volt_vfe_equ_idx =
			ptemp_rail->boot_volt_vfe_equ_idx;
	board_obj_volt_rail_ptr->vmin_limit_vfe_equ_idx =
			ptemp_rail->vmin_limit_vfe_equ_idx;
	board_obj_volt_rail_ptr->volt_margin_limit_vfe_equ_idx =
			ptemp_rail->volt_margin_limit_vfe_equ_idx;
	board_obj_volt_rail_ptr->volt_scale_exp_pwr_equ_idx =
			ptemp_rail->volt_scale_exp_pwr_equ_idx;

	nvgpu_log_info(g, "Done");

	return (struct voltage_rail *)board_obj_ptr;
}

u8 volt_rail_vbios_volt_domain_convert_to_internal(struct gk20a *g,
	u8 vbios_volt_domain)
{
	switch (g->perf_pmu.volt.volt_rail_metadata.volt_domain_hal) {
	case CTRL_VOLT_DOMAIN_HAL_GP10X_SINGLE_RAIL:
		if (vbios_volt_domain == 0U) {
			return CTRL_VOLT_DOMAIN_LOGIC;
		}
		break;
	case CTRL_VOLT_DOMAIN_HAL_GP10X_SPLIT_RAIL:
		switch (vbios_volt_domain) {
		case 0:
			return CTRL_VOLT_DOMAIN_LOGIC;
		case 1:
			return CTRL_VOLT_DOMAIN_SRAM;
		}
		break;
	}

	return CTRL_VOLT_DOMAIN_INVALID;
}

int volt_rail_pmu_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;

	nvgpu_log_info(g, " ");

	pboardobjgrp = &g->perf_pmu.volt.volt_rail_metadata.volt_rails.super;

	if (!pboardobjgrp->bconstructed) {
		return -EINVAL;
	}

	status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);

	nvgpu_log_info(g, "Done");
	return status;
}

static int volt_get_volt_rail_table(struct gk20a *g,
		struct voltage_rail_metadata *pvolt_rail_metadata)
{
	int status = 0;
	u8 *volt_rail_table_ptr = NULL;
	struct voltage_rail *prail = NULL;
	struct vbios_voltage_rail_table_1x_header header = { 0 };
	struct vbios_voltage_rail_table_1x_entry entry = { 0 };
	u8 i;
	u8 volt_domain;
	u8 *entry_ptr;
	union rail_type {
		struct boardobj board_obj;
		struct voltage_rail volt_rail;
	} rail_type_data;

	volt_rail_table_ptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.perf_token, VOLTAGE_RAIL_TABLE);
	if (volt_rail_table_ptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	memcpy(&header, volt_rail_table_ptr,
			sizeof(struct vbios_voltage_rail_table_1x_header));

	pvolt_rail_metadata->volt_domain_hal = (u8)header.volt_domain_hal;

	for (i = 0; i < header.num_table_entries; i++) {
		entry_ptr = (volt_rail_table_ptr + header.header_size +
			(i * header.table_entry_size));

		memset(&rail_type_data, 0x0, sizeof(rail_type_data));

		memcpy(&entry, entry_ptr,
			sizeof(struct vbios_voltage_rail_table_1x_entry));

		volt_domain = volt_rail_vbios_volt_domain_convert_to_internal(g,
			i);
		if (volt_domain == CTRL_VOLT_DOMAIN_INVALID) {
			continue;
		}

		rail_type_data.board_obj.type = volt_domain;
		rail_type_data.volt_rail.boot_voltage_uv =
			(u32)entry.boot_voltage_uv;
		rail_type_data.volt_rail.rel_limit_vfe_equ_idx =
			(u8)entry.rel_limit_vfe_equ_idx;
		rail_type_data.volt_rail.alt_rel_limit_vfe_equ_idx =
			(u8)entry.alt_rel_limit_vfe_equidx;
		rail_type_data.volt_rail.ov_limit_vfe_equ_idx =
			(u8)entry.ov_limit_vfe_equ_idx;

		if (header.table_entry_size >=
			NV_VBIOS_VOLTAGE_RAIL_1X_ENTRY_SIZE_0C) {
			rail_type_data.volt_rail.volt_scale_exp_pwr_equ_idx =
				(u8)entry.volt_scale_exp_pwr_equ_idx;
		} else {
			rail_type_data.volt_rail.volt_scale_exp_pwr_equ_idx =
				CTRL_BOARDOBJ_IDX_INVALID;
		}

		if (header.table_entry_size >=
			NV_VBIOS_VOLTAGE_RAIL_1X_ENTRY_SIZE_0B) {
			rail_type_data.volt_rail.volt_margin_limit_vfe_equ_idx =
				(u8)entry.volt_margin_limit_vfe_equ_idx;
		} else {
			rail_type_data.volt_rail.volt_margin_limit_vfe_equ_idx =
				CTRL_BOARDOBJ_IDX_INVALID;
		}

		if (header.table_entry_size >=
			NV_VBIOS_VOLTAGE_RAIL_1X_ENTRY_SIZE_0A) {
			rail_type_data.volt_rail.vmin_limit_vfe_equ_idx =
				(u8)entry.vmin_limit_vfe_equ_idx;
		} else {
			rail_type_data.volt_rail.vmin_limit_vfe_equ_idx =
				CTRL_BOARDOBJ_IDX_INVALID;
		}

		if (header.table_entry_size >=
			NV_VBIOS_VOLTAGE_RAIL_1X_ENTRY_SIZE_09) {
			rail_type_data.volt_rail.boot_volt_vfe_equ_idx =
				(u8)entry.boot_volt_vfe_equ_idx;
		} else {
			rail_type_data.volt_rail.boot_volt_vfe_equ_idx =
				CTRL_BOARDOBJ_IDX_INVALID;
		}

		if (header.table_entry_size >=
			NV_VBIOS_VOLTAGE_RAIL_1X_ENTRY_SIZE_08) {
			rail_type_data.volt_rail.pwr_equ_idx =
				(u8)entry.pwr_equ_idx;
		} else {
			rail_type_data.volt_rail.pwr_equ_idx =
				CTRL_PMGR_PWR_EQUATION_INDEX_INVALID;
		}

		prail = construct_volt_rail(g, &rail_type_data);

		status = boardobjgrp_objinsert(
				&pvolt_rail_metadata->volt_rails.super,
				(struct boardobj *)prail, i);
	}

done:
	return status;
}

static int _volt_rail_devgrp_pmudata_instget(struct gk20a *g,
	struct nv_pmu_boardobjgrp *pmuboardobjgrp, struct nv_pmu_boardobj
	**ppboardobjpmudata, u8 idx)
{
	struct nv_pmu_volt_volt_rail_boardobj_grp_set *pgrp_set =
		(struct nv_pmu_volt_volt_rail_boardobj_grp_set *)
		pmuboardobjgrp;

	nvgpu_log_info(g, " ");

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (((u32)BIT(idx) &
		pgrp_set->hdr.data.super.obj_mask.super.data[0]) == 0U) {
		return -EINVAL;
	}

	*ppboardobjpmudata = (struct nv_pmu_boardobj *)
		&pgrp_set->objects[idx].data.board_obj;
	nvgpu_log_info(g, " Done");
	return 0;
}

static int _volt_rail_devgrp_pmustatus_instget(struct gk20a *g,
	void *pboardobjgrppmu, struct nv_pmu_boardobj_query
	**ppboardobjpmustatus, u8 idx)
{
	struct nv_pmu_volt_volt_rail_boardobj_grp_get_status *pgrp_get_status =
		(struct nv_pmu_volt_volt_rail_boardobj_grp_get_status *)
		pboardobjgrppmu;

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (((u32)BIT(idx) &
		pgrp_get_status->hdr.data.super.obj_mask.super.data[0]) == 0U) {
		return -EINVAL;
	}

	*ppboardobjpmustatus = (struct nv_pmu_boardobj_query *)
			&pgrp_get_status->objects[idx].data.board_obj;
	return 0;
}

int volt_rail_sw_setup(struct gk20a *g)
{
	int status = 0;
	struct boardobjgrp *pboardobjgrp = NULL;
	struct voltage_rail *pvolt_rail;
	u8 i;

	nvgpu_log_info(g, " ");

	status = boardobjgrpconstruct_e32(g,
			&g->perf_pmu.volt.volt_rail_metadata.volt_rails);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for volt rail, status - 0x%x",
			status);
		goto done;
	}

	pboardobjgrp = &g->perf_pmu.volt.volt_rail_metadata.volt_rails.super;

	pboardobjgrp->pmudatainstget  = _volt_rail_devgrp_pmudata_instget;
	pboardobjgrp->pmustatusinstget  = _volt_rail_devgrp_pmustatus_instget;

	g->perf_pmu.volt.volt_rail_metadata.pct_delta =
			NV_PMU_VOLT_VALUE_0V_IN_UV;

	/* Obtain Voltage Rail Table from VBIOS */
	status = volt_get_volt_rail_table(g, &g->perf_pmu.volt.
			volt_rail_metadata);
	if (status) {
		goto done;
	}

	/* Populate data for the VOLT_RAIL PMU interface */
	BOARDOBJGRP_PMU_CONSTRUCT(pboardobjgrp, VOLT, VOLT_RAIL);

	status = BOARDOBJGRP_PMU_CMD_GRP_SET_CONSTRUCT(g, pboardobjgrp,
			volt, VOLT, volt_rail, VOLT_RAIL);
	if (status) {
		nvgpu_err(g,
			"error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			status);
		goto done;
	}

	status = BOARDOBJGRP_PMU_CMD_GRP_GET_STATUS_CONSTRUCT(g,
		&g->perf_pmu.volt.volt_rail_metadata.volt_rails.super,
			volt, VOLT, volt_rail, VOLT_RAIL);
	if (status) {
		nvgpu_err(g,
			"error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			status);
		goto done;
	}

	/* update calibration to fuse */
	BOARDOBJGRP_FOR_EACH(&(g->perf_pmu.volt.volt_rail_metadata.
			       volt_rails.super),
			     struct voltage_rail *, pvolt_rail, i) {
		status = volt_rail_state_init(g, pvolt_rail);
		if (status) {
			nvgpu_err(g,
				"Failure while executing RAIL's state init railIdx = %d",
				i);
			goto done;
		}
	}

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}
