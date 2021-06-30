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
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

#include "thrmdev.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "gp106/bios_gp106.h"
#include "ctrl/ctrltherm.h"

static int _therm_device_pmudata_instget(struct gk20a *g,
			struct nv_pmu_boardobjgrp *pmuboardobjgrp,
			struct nv_pmu_boardobj **ppboardobjpmudata,
			u8 idx)
{
	struct nv_pmu_therm_therm_device_boardobj_grp_set *pgrp_set =
		(struct nv_pmu_therm_therm_device_boardobj_grp_set *)
		pmuboardobjgrp;

	nvgpu_log_info(g, " ");

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (((u32)BIT(idx) &
			pgrp_set->hdr.data.super.obj_mask.super.data[0]) == 0) {
		return -EINVAL;
	}

	*ppboardobjpmudata = (struct nv_pmu_boardobj *)
		&pgrp_set->objects[idx].data;

	nvgpu_log_info(g, " Done");

	return 0;
}

static int construct_therm_device(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	return boardobj_construct_super(g, ppboardobj, size, pargs);
}

static int construct_therm_device_gpu(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	return construct_therm_device(g, ppboardobj, size, pargs);
}

static int construct_therm_device_gpu_sci(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	return construct_therm_device(g, ppboardobj, size, pargs);
}


static int therm_device_pmu_data_init_gpu_gpc_tsosc(struct gk20a *g,
	struct boardobj *pboard_obj, struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct therm_device_gpu_gpc_tsosc *pdev = NULL;
	struct nv_pmu_therm_therm_device_gpu_gpc_tsosc_boardobj_set *pset;

	status = boardobj_pmudatainit_super(g, pboard_obj, ppmudata);
	if (status != 0) {
		goto exit;
	}

	pdev = (struct therm_device_gpu_gpc_tsosc *)(void *)pboard_obj;
	pset = (struct nv_pmu_therm_therm_device_gpu_gpc_tsosc_boardobj_set *)
		(void*) ppmudata;

	pset->gpc_tsosc_idx = pdev->gpc_tsosc_idx;

exit:
	return status;
}

static int construct_therm_device_gpu_tsosc(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	struct therm_device_gpu_gpc_tsosc *pdev = NULL;
	struct therm_device_gpu_gpc_tsosc *ptmp_dev =
		(struct therm_device_gpu_gpc_tsosc *)pargs;
	int status = 0;

	status = construct_therm_device(g, ppboardobj, size, pargs);
	if (status != 0) {
		return status;
	}

	pdev = (struct therm_device_gpu_gpc_tsosc *)(void *)*ppboardobj;

	pdev->super.super.pmudatainit =
		therm_device_pmu_data_init_gpu_gpc_tsosc;

	pdev->gpc_tsosc_idx = ptmp_dev->gpc_tsosc_idx;

	return status;
}

static int therm_device_pmu_data_init_hbm2_site(struct gk20a *g,
	struct boardobj *pboard_obj, struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct therm_device_hbm2_site *pdev = NULL;
	struct nv_pmu_therm_therm_device_hbm2_site_boardobj_set *pset;

	status = boardobj_pmudatainit_super(g, pboard_obj, ppmudata);
	if (status != 0) {
		goto exit;
	}

	pdev = (struct therm_device_hbm2_site *)(void *)pboard_obj;
	pset = (struct nv_pmu_therm_therm_device_hbm2_site_boardobj_set *)
		(void *)ppmudata;

	pset->site_idx = pdev->site_idx;

exit:
	return status;
}

static int construct_therm_device_hbm2_site(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	struct therm_device_hbm2_site *pdev = NULL;
	struct therm_device_hbm2_site *ptmp_dev =
		(struct therm_device_hbm2_site *)pargs;
	int status = 0;

	status = construct_therm_device(g, ppboardobj, size, pargs);
	if (status != 0) {
		return status;
	}

	pdev = (struct therm_device_hbm2_site *)(void *)*ppboardobj;

	pdev->super.super.pmudatainit =
		therm_device_pmu_data_init_hbm2_site;

	pdev->site_idx = ptmp_dev->site_idx;

	return status;
}

static int construct_therm_device_hbm2_combined(struct gk20a *g,
	struct boardobj **ppboardobj, u16 size, void *pargs)
{
	return construct_therm_device(g, ppboardobj, size, pargs);
}


static struct boardobj *therm_device_construct(struct gk20a *g,
	void *pargs)
{
	struct boardobj *board_obj_ptr = NULL;
	int status = 0;

	switch (BOARDOBJ_GET_TYPE(pargs)) {
	case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU:
		status = construct_therm_device_gpu(g, &board_obj_ptr,
			sizeof(struct therm_device), pargs);
		break;
	case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU_GPC_SCI:
		status = construct_therm_device_gpu_sci(g, &board_obj_ptr,
			sizeof(struct therm_device_gpu_sci), pargs);
		break;
	case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU_GPC_TSOSC:
		status = construct_therm_device_gpu_tsosc(g, &board_obj_ptr,
			sizeof(struct therm_device_gpu_gpc_tsosc), pargs);
		break;
	case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_HBM2_SITE:
		status = construct_therm_device_hbm2_site(g, &board_obj_ptr,
			sizeof(struct therm_device_hbm2_site), pargs);
		break;
	case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_HBM2_COMBINED:
		status = construct_therm_device_hbm2_combined(g, &board_obj_ptr,
			sizeof(struct therm_device_hbm2_combined), pargs);
		break;
	default:
		nvgpu_err(g,
			"unsupported therm_device class - 0x%x",
			BOARDOBJ_GET_TYPE(pargs));
		break;
	}

	if(status) {
		board_obj_ptr = NULL;
		nvgpu_err(g,
			"could not allocate memory for therm_device");
		if (board_obj_ptr != NULL) {
			nvgpu_kfree(g, board_obj_ptr);
		}
	}


	return board_obj_ptr;
}

static int devinit_get_therm_device_table(struct gk20a *g,
				struct therm_devices *pthermdeviceobjs)
{
	int status = 0;
	u8 *therm_device_table_ptr = NULL;
	u8 *curr_therm_device_table_ptr = NULL;
	struct boardobj *boardobj;
	struct therm_device_1x_header therm_device_table_header = { 0 };
	struct therm_device_1x_entry *therm_device_table_entry = NULL;
	u32 index;
	u32 obj_index = 0;
	u8 class_id = 0;
	union {
		struct boardobj boardobj;
		struct therm_device therm_device;
		struct therm_device_gpu_sci gpu_sci;
		struct therm_device_gpu_gpc_tsosc gpu_gpc_tsosc;
		struct therm_device_hbm2_site hbm2_site;
		struct therm_device_hbm2_combined hbm2_combined;
	} therm_device_data;

	nvgpu_log_info(g, " ");

	therm_device_table_ptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.perf_token, THERMAL_DEVICE_TABLE);
	if (therm_device_table_ptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	memcpy(&therm_device_table_header, therm_device_table_ptr,
		VBIOS_THERM_DEVICE_1X_HEADER_SIZE_04);

	if (therm_device_table_header.version !=
			VBIOS_THERM_DEVICE_VERSION_1X) {
		status = -EINVAL;
		goto done;
	}

	if (therm_device_table_header.header_size <
			VBIOS_THERM_DEVICE_1X_HEADER_SIZE_04) {
		status = -EINVAL;
		goto done;
	}

	curr_therm_device_table_ptr = (therm_device_table_ptr +
		VBIOS_THERM_DEVICE_1X_HEADER_SIZE_04);

	for (index = 0; index < therm_device_table_header.num_table_entries;
		index++) {
		therm_device_table_entry = (struct therm_device_1x_entry *)
			(curr_therm_device_table_ptr +
				(therm_device_table_header.table_entry_size * index));

		class_id = therm_device_table_entry->class_id;

		switch (class_id) {
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_INVALID:
			continue;
			break;
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU:
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU_GPC_SCI:
			break;
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_GPU_GPC_TSOSC:
			therm_device_data.gpu_gpc_tsosc.gpc_tsosc_idx =
				therm_device_table_entry->param0;
			break;
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_HBM2_SITE:
			therm_device_data.hbm2_site.site_idx =
				therm_device_table_entry->param0;
			break;
		case NV_VBIOS_THERM_DEVICE_1X_ENTRY_CLASS_HBM2_COMBINED:
			break;
		default:
			nvgpu_err(g,
				"Unknown thermal device class i - %x, class - %x",
				index, class_id);
			goto done;
		}

		therm_device_data.boardobj.type = class_id;
		boardobj = therm_device_construct(g, &therm_device_data);
		if (!boardobj) {
			nvgpu_err(g,
				"unable to create thermal device for %d type %d",
				index, therm_device_data.boardobj.type);
			status = -EINVAL;
			goto done;
		}

		status = boardobjgrp_objinsert(&pthermdeviceobjs->super.super,
				boardobj, obj_index);

		if (status) {
			nvgpu_err(g,
			"unable to insert thermal device boardobj for %d", index);
			status = -EINVAL;
			goto done;
		}

		++obj_index;
	}

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}

int therm_device_sw_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;
	struct therm_devices *pthermdeviceobjs;

	/* Construct the Super Class and override the Interfaces */
	status = boardobjgrpconstruct_e32(g,
			&g->therm_pmu.therm_deviceobjs.super);
	if (status) {
		nvgpu_err(g,
			  "error creating boardobjgrp for therm devices, status - 0x%x",
			  status);
		goto done;
	}

	pboardobjgrp = &g->therm_pmu.therm_deviceobjs.super.super;
	pthermdeviceobjs = &(g->therm_pmu.therm_deviceobjs);

	/* Override the Interfaces */
	pboardobjgrp->pmudatainstget = _therm_device_pmudata_instget;

	status = devinit_get_therm_device_table(g, pthermdeviceobjs);
	if (status) {
		goto done;
	}

	BOARDOBJGRP_PMU_CONSTRUCT(pboardobjgrp, THERM, THERM_DEVICE);

	status = BOARDOBJGRP_PMU_CMD_GRP_SET_CONSTRUCT(g, pboardobjgrp,
			therm, THERM, therm_device, THERM_DEVICE);
	if (status) {
		nvgpu_err(g,
			  "error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			  status);
		goto done;
	}

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}
