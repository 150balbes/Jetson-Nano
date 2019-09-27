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

#include "pmu_perf.h"
#include "vfe_var.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "ctrl/ctrlclk.h"
#include "ctrl/ctrlvolt.h"
#include "ctrl/ctrlperf.h"

static int devinit_get_vfe_var_table(struct gk20a *g,
				     struct vfe_vars *pvarobjs);
static int vfe_var_construct_single(struct gk20a *g,
				    struct boardobj **ppboardobj,
				    u16 size, void *pargs);

static int _vfe_vars_pmudatainit(struct gk20a *g,
				 struct boardobjgrp *pboardobjgrp,
				 struct nv_pmu_boardobjgrp_super *pboardobjgrppmu)
{
	struct nv_pmu_perf_vfe_var_boardobjgrp_set_header *pset =
		(struct nv_pmu_perf_vfe_var_boardobjgrp_set_header *)
		pboardobjgrppmu;
	struct vfe_vars *pvars = (struct vfe_vars *)pboardobjgrp;
	int status = 0;

	status = boardobjgrp_pmudatainit_e32(g, pboardobjgrp, pboardobjgrppmu);
	if (status) {
		nvgpu_err(g,
			"error updating pmu boardobjgrp for vfe var 0x%x",
			 status);
		goto done;
	}
	pset->polling_periodms = pvars->polling_periodms;

done:
	return status;
}

static int _vfe_vars_pmudata_instget(struct gk20a *g,
				     struct nv_pmu_boardobjgrp *pmuboardobjgrp,
				     struct nv_pmu_boardobj **ppboardobjpmudata,
				     u8 idx)
{
	struct nv_pmu_perf_vfe_var_boardobj_grp_set  *pgrp_set =
		(struct nv_pmu_perf_vfe_var_boardobj_grp_set *)
		pmuboardobjgrp;

	nvgpu_log_info(g, " ");

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (idx >= CTRL_BOARDOBJGRP_E32_MAX_OBJECTS) {
		return -EINVAL;
	}

	*ppboardobjpmudata = (struct nv_pmu_boardobj *)
		&pgrp_set->objects[idx].data.board_obj;

	nvgpu_log_info(g, " Done");
	return 0;
}

static int _vfe_vars_pmustatus_instget(struct gk20a *g, void *pboardobjgrppmu,
	struct nv_pmu_boardobj_query **ppboardobjpmustatus, u8 idx)
{
	struct nv_pmu_perf_vfe_var_boardobj_grp_get_status *pgrp_get_status =
		(struct nv_pmu_perf_vfe_var_boardobj_grp_get_status *)
		pboardobjgrppmu;

	if (((u32)BIT(idx) &
		pgrp_get_status->hdr.data.super.obj_mask.super.data[0]) == 0) {
		return -EINVAL;
	}

	*ppboardobjpmustatus = (struct nv_pmu_boardobj_query *)
			&pgrp_get_status->objects[idx].data.board_obj;
	return 0;
}


int vfe_var_sw_setup(struct gk20a *g)
{
	u32 status;
	struct boardobjgrp *pboardobjgrp = NULL;
	struct vfe_vars *pvfevarobjs;

	nvgpu_log_info(g, " ");

	status = boardobjgrpconstruct_e32(g, &g->perf_pmu.vfe_varobjs.super);
	if (status) {
		nvgpu_err(g,
			  "error creating boardobjgrp for clk domain, status - 0x%x",
			  status);
		goto done;
	}

	pboardobjgrp = &g->perf_pmu.vfe_varobjs.super.super;
	pvfevarobjs = &g->perf_pmu.vfe_varobjs;

	BOARDOBJGRP_PMU_CONSTRUCT(pboardobjgrp, PERF, VFE_VAR);

	status = BOARDOBJGRP_PMU_CMD_GRP_SET_CONSTRUCT(g, pboardobjgrp,
			perf, PERF, vfe_var, VFE_VAR);
	if (status) {
		nvgpu_err(g,
			  "error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			 status);
		goto done;
	}

	pboardobjgrp->pmudatainit  = _vfe_vars_pmudatainit;
	pboardobjgrp->pmudatainstget  = _vfe_vars_pmudata_instget;
	pboardobjgrp->pmustatusinstget  = _vfe_vars_pmustatus_instget;

	status = devinit_get_vfe_var_table(g, pvfevarobjs);
	if (status) {
		goto done;
	}

	status = BOARDOBJGRP_PMU_CMD_GRP_GET_STATUS_CONSTRUCT(g,
				&g->perf_pmu.vfe_varobjs.super.super,
				perf, PERF, vfe_var, VFE_VAR);
	if (status) {
		nvgpu_err(g,
		"error constructing PMU_BOARDOBJ_CMD_GRP_GET_STATUS interface - 0x%x",
			status);
		goto done;
	}

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}

int vfe_var_pmu_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;

	nvgpu_log_info(g, " ");

	pboardobjgrp = &g->perf_pmu.vfe_varobjs.super.super;

	if (!pboardobjgrp->bconstructed) {
		return -EINVAL;
	}

	status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);

	nvgpu_log_info(g, "Done");
	return status;
}

static u32 dev_init_get_vfield_info(struct gk20a *g,
	struct vfe_var_single_sensed_fuse *pvfevar)
{
	u8 *vfieldtableptr = NULL;
	u32 vfieldheadersize = VFIELD_HEADER_SIZE;
	u8 *vfieldregtableptr = NULL;
	u32 vfieldregheadersize = VFIELD_REG_HEADER_SIZE;
	u32 i;
	u32 oldindex = 0xFFFFFFFF;
	u32 currindex;
	struct vfield_reg_header vregheader;
	struct vfield_reg_entry vregentry;
	struct vfield_header vheader;
	struct vfield_entry ventry;
	struct ctrl_bios_vfield_register_segment *psegment = NULL;
	u8 *psegmentcount = NULL;
	u32 status = 0;

	vfieldregtableptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.virt_token, VP_FIELD_REGISTER);
	if (vfieldregtableptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	vfieldtableptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.virt_token, VP_FIELD_TABLE);
	if (vfieldtableptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	memcpy(&vregheader, vfieldregtableptr, VFIELD_REG_HEADER_SIZE);

	if (vregheader.version != VBIOS_VFIELD_REG_TABLE_VERSION_1_0) {
		nvgpu_err(g, "invalid vreg header version");
		goto done;
	}

	memcpy(&vheader, vfieldtableptr, VFIELD_HEADER_SIZE);

	if (vregheader.version != VBIOS_VFIELD_TABLE_VERSION_1_0) {
		nvgpu_err(g, "invalid vfield header version");
		goto done;
	}

	pvfevar->vfield_info.fuse.segment_count = 0;
	pvfevar->vfield_ver_info.fuse.segment_count = 0;
	for (i = 0; i < (u32)vheader.count; i++) {
		memcpy(&ventry, vfieldtableptr + vfieldheadersize +
			(i * vheader.entry_size),
			vheader.entry_size);

		currindex = VFIELD_BIT_REG(ventry);
		if (currindex != oldindex) {

			memcpy(&vregentry, vfieldregtableptr +
				vfieldregheadersize +
				(currindex * vregheader.entry_size),
				vregheader.entry_size);
			oldindex = currindex;
		}

		if (pvfevar->vfield_info.v_field_id == ventry.strap_id) {
			psegmentcount =
				&(pvfevar->vfield_info.fuse.segment_count);
			psegment =
				&(pvfevar->vfield_info.fuse.segments[*psegmentcount]);
			if (*psegmentcount > NV_PMU_VFE_VAR_SINGLE_SENSED_FUSE_SEGMENTS_MAX) {
				status = -EINVAL;
				goto done;
			}
		} else if (pvfevar->vfield_ver_info.v_field_id_ver == ventry.strap_id) {
			psegmentcount =
				&(pvfevar->vfield_ver_info.fuse.segment_count);
			psegment =
				&(pvfevar->vfield_ver_info.fuse.segments[*psegmentcount]);
			if (*psegmentcount > NV_PMU_VFE_VAR_SINGLE_SENSED_FUSE_SEGMENTS_MAX) {
				status = -EINVAL;
				goto done;
			}
		} else {
			continue;
		}

		switch (VFIELD_CODE((&vregentry))) {
		case NV_VFIELD_DESC_CODE_REG:
			psegment->type =
				NV_PMU_BIOS_VFIELD_DESC_CODE_REG;
			psegment->data.reg.addr = vregentry.reg;
			psegment->data.reg.super.high_bit = (u8)(VFIELD_BIT_STOP(ventry));
			psegment->data.reg.super.low_bit = (u8)(VFIELD_BIT_START(ventry));
			break;

		case NV_VFIELD_DESC_CODE_INDEX_REG:
			psegment->type =
				NV_PMU_BIOS_VFIELD_DESC_CODE_INDEX_REG;
			psegment->data.index_reg.addr = vregentry.reg;
			psegment->data.index_reg.index = vregentry.index;
			psegment->data.index_reg.reg_index = vregentry.reg_index;
			psegment->data.index_reg.super.high_bit = (u8)(VFIELD_BIT_STOP(ventry));
			psegment->data.index_reg.super.low_bit = (u8)(VFIELD_BIT_START(ventry));
			break;

		default:
			psegment->type =
				NV_PMU_BIOS_VFIELD_DESC_CODE_INVALID;
			status = -EINVAL;
			goto done;
		}

		if (VFIELD_SIZE((&vregentry)) != NV_VFIELD_DESC_SIZE_DWORD) {
			psegment->type =
				NV_PMU_BIOS_VFIELD_DESC_CODE_INVALID;
			return -EINVAL;
		}
		(*psegmentcount)++;
	}

done:
	return status;
}

static int _vfe_var_pmudatainit_super(struct gk20a *g,
				      struct boardobj *board_obj_ptr,
				      struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var *pvfe_var;
	struct nv_pmu_vfe_var *pset;

	nvgpu_log_info(g, " ");

	status = boardobj_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var = (struct vfe_var *)board_obj_ptr;
	pset = (struct nv_pmu_vfe_var *) ppmudata;

	pset->out_range_min = pvfe_var->out_range_min;
	pset->out_range_max = pvfe_var->out_range_max;
	status = boardobjgrpmask_export(&pvfe_var->mask_dependent_vars.super,
					pvfe_var->mask_dependent_vars.super.bitcount,
					&pset->mask_dependent_vars.super);
	status = boardobjgrpmask_export(&pvfe_var->mask_dependent_equs.super,
					pvfe_var->mask_dependent_equs.super.bitcount,
					&pset->mask_dependent_equs.super);
	return status;
}

static int vfe_var_construct_super(struct gk20a *g,
				   struct boardobj **ppboardobj,
				   u16 size, void *pargs)
{
	struct vfe_var *pvfevar;
	struct vfe_var *ptmpvar = (struct vfe_var *)pargs;
	int status = 0;

	nvgpu_log_info(g, " ");

	status = boardobj_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var *)*ppboardobj;

	pvfevar->super.pmudatainit =
			_vfe_var_pmudatainit_super;

	pvfevar->out_range_min = ptmpvar->out_range_min;
	pvfevar->out_range_max = ptmpvar->out_range_max;
	pvfevar->b_is_dynamic_valid = false;
	status = boardobjgrpmask_e32_init(&pvfevar->mask_dependent_vars, NULL);
	status = boardobjgrpmask_e255_init(&pvfevar->mask_dependent_equs, NULL);
	nvgpu_log_info(g, " ");

	return status;
}

static int _vfe_var_pmudatainit_derived(struct gk20a *g,
					struct boardobj *board_obj_ptr,
					struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_super(g, board_obj_ptr, ppmudata);

	return status;
}

static int vfe_var_construct_derived(struct gk20a *g,
				     struct boardobj **ppboardobj,
				     u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	int status = 0;
	struct vfe_var_derived *pvfevar;

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_DERIVED);
	status = vfe_var_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_derived *)*ppboardobj;

	pvfevar->super.super.pmudatainit =
			_vfe_var_pmudatainit_derived;

	return status;
}

static int _vfe_var_pmudatainit_derived_product(struct gk20a *g,
						struct boardobj *board_obj_ptr,
						struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var_derived_product *pvfe_var_derived_product;
	struct nv_pmu_vfe_var_derived_product *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_derived(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var_derived_product =
		(struct vfe_var_derived_product *)board_obj_ptr;
	pset = (struct nv_pmu_vfe_var_derived_product *)ppmudata;

	pset->var_idx0 = pvfe_var_derived_product->var_idx0;
	pset->var_idx1 = pvfe_var_derived_product->var_idx1;

	return status;
}

static int vfe_var_construct_derived_product(struct gk20a *g,
					     struct boardobj **ppboardobj,
					     u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_derived_product *pvfevar;
	struct vfe_var_derived_product *ptmpvar =
			(struct vfe_var_derived_product *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_DERIVED_PRODUCT) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_DERIVED_PRODUCT);
	status = vfe_var_construct_derived(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_derived_product *)*ppboardobj;

	pvfevar->super.super.super.pmudatainit =
			_vfe_var_pmudatainit_derived_product;

	pvfevar->var_idx0 = ptmpvar->var_idx0;
	pvfevar->var_idx1 = ptmpvar->var_idx1;


	return status;
}

static int _vfe_var_pmudatainit_derived_sum(struct gk20a *g,
					    struct boardobj *board_obj_ptr,
					    struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var_derived_sum *pvfe_var_derived_sum;
	struct nv_pmu_vfe_var_derived_sum *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_derived(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var_derived_sum = (struct vfe_var_derived_sum *)board_obj_ptr;
	pset = (struct nv_pmu_vfe_var_derived_sum *)ppmudata;

	pset->var_idx0 = pvfe_var_derived_sum->var_idx0;
	pset->var_idx1 = pvfe_var_derived_sum->var_idx1;

	return status;
}

static int vfe_var_construct_derived_sum(struct gk20a *g,
					 struct boardobj **ppboardobj,
					 u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_derived_sum *pvfevar;
	struct vfe_var_derived_sum *ptmpvar =
			(struct vfe_var_derived_sum *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_DERIVED_SUM) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_DERIVED_SUM);
	status = vfe_var_construct_derived(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_derived_sum *)*ppboardobj;

	pvfevar->super.super.super.pmudatainit =
			_vfe_var_pmudatainit_derived_sum;

	pvfevar->var_idx0 = ptmpvar->var_idx0;
	pvfevar->var_idx1 = ptmpvar->var_idx1;

	return status;
}

static int _vfe_var_pmudatainit_single(struct gk20a *g,
				       struct boardobj *board_obj_ptr,
				       struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var_single *pvfe_var_single;
	struct nv_pmu_vfe_var_single *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var_single = (struct vfe_var_single *)board_obj_ptr;
	pset = (struct nv_pmu_vfe_var_single *)
		ppmudata;

	pset->override_type = pvfe_var_single->override_type;
	pset->override_value = pvfe_var_single->override_value;

	return status;
}

static int _vfe_var_pmudatainit_single_frequency(struct gk20a *g,
						 struct boardobj *board_obj_ptr,
						 struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_single(g, board_obj_ptr, ppmudata);

	return status;
}

static u32 vfe_var_construct_single_frequency(struct gk20a *g,
					      struct boardobj **ppboardobj,
					      u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single_frequency *pvfevar;
	u32 status = 0;

	nvgpu_log_info(g, " ");

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_SINGLE_FREQUENCY) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE_FREQUENCY);
	status = vfe_var_construct_single(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single_frequency *)*ppboardobj;

	pvfevar->super.super.super.pmudatainit =
			_vfe_var_pmudatainit_single_frequency;

	pvfevar->super.super.b_is_dynamic = false;
	pvfevar->super.super.b_is_dynamic_valid = true;

	nvgpu_log_info(g, "Done");
	return status;
}

static int _vfe_var_pmudatainit_single_sensed(struct gk20a *g,
					      struct boardobj *board_obj_ptr,
					      struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_single(g, board_obj_ptr, ppmudata);

	return status;
}

static int _vfe_var_pmudatainit_single_sensed_fuse(struct gk20a *g,
						   struct boardobj *board_obj_ptr,
						   struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var_single_sensed_fuse *pvfe_var_single_sensed_fuse;
	struct nv_pmu_vfe_var_single_sensed_fuse *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_single_sensed(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var_single_sensed_fuse =
		(struct vfe_var_single_sensed_fuse *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_var_single_sensed_fuse *)
		ppmudata;

	memcpy(&pset->vfield_info, &pvfe_var_single_sensed_fuse->vfield_info,
		sizeof(struct ctrl_perf_vfe_var_single_sensed_fuse_vfield_info));

	memcpy(&pset->vfield_ver_info,
		&pvfe_var_single_sensed_fuse->vfield_ver_info,
		sizeof(struct ctrl_perf_vfe_var_single_sensed_fuse_ver_vfield_info));

	memcpy(&pset->override_info,
		&pvfe_var_single_sensed_fuse->override_info,
		sizeof(struct ctrl_perf_vfe_var_single_sensed_fuse_override_info));

	pset->b_fuse_value_signed = pvfe_var_single_sensed_fuse->b_fuse_value_signed;
	return status;
}

static u32 vfe_var_construct_single_sensed(struct gk20a *g,
					   struct boardobj **ppboardobj,
					   u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single_sensed *pvfevar;

	u32 status = 0;

	nvgpu_log_info(g, " ");

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED);
	status = vfe_var_construct_single(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single_sensed *)*ppboardobj;

	pvfevar->super.super.super.pmudatainit =
			_vfe_var_pmudatainit_single_sensed;

	nvgpu_log_info(g, "Done");

	return status;
}

static u32 vfe_var_construct_single_sensed_fuse(struct gk20a *g,
						struct boardobj **ppboardobj,
						u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single_sensed_fuse *pvfevar;
	struct vfe_var_single_sensed_fuse *ptmpvar =
			(struct vfe_var_single_sensed_fuse *)pargs;
	u32 status = 0;

	nvgpu_log_info(g, " ");

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_FUSE) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_FUSE);
	status = vfe_var_construct_single_sensed(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single_sensed_fuse *)*ppboardobj;

	pvfevar->super.super.super.super.pmudatainit =
			_vfe_var_pmudatainit_single_sensed_fuse;

	pvfevar->vfield_info.v_field_id = ptmpvar->vfield_info.v_field_id;
	pvfevar->vfield_info.fuse_val_default =
		ptmpvar->vfield_info.fuse_val_default;
	pvfevar->vfield_info.hw_correction_scale =
		ptmpvar->vfield_info.hw_correction_scale;
	pvfevar->vfield_info.hw_correction_offset =
		ptmpvar->vfield_info.hw_correction_offset;
	pvfevar->vfield_ver_info.v_field_id_ver =
		ptmpvar->vfield_ver_info.v_field_id_ver;
	pvfevar->vfield_ver_info.ver_expected =
		ptmpvar->vfield_ver_info.ver_expected;
	pvfevar->vfield_ver_info.b_use_default_on_ver_check_fail =
		ptmpvar->vfield_ver_info.b_use_default_on_ver_check_fail;
	pvfevar->b_version_check_done = false;
	pvfevar->b_fuse_value_signed =
		ptmpvar->b_fuse_value_signed;
	pvfevar->super.super.super.b_is_dynamic = false;
	pvfevar->super.super.super.b_is_dynamic_valid = true;

	dev_init_get_vfield_info(g, pvfevar);
	/*check whether fuse segment got initialized*/
	if (pvfevar->vfield_info.fuse.segment_count == 0) {
		nvgpu_err(g, "unable to get fuse reg info %x",
			pvfevar->vfield_info.v_field_id);
		status = -EINVAL;
		goto exit;
	}
	if (pvfevar->vfield_ver_info.fuse.segment_count == 0) {
		nvgpu_err(g, "unable to get fuse reg info %x",
			pvfevar->vfield_ver_info.v_field_id_ver);
		status = -EINVAL;
		goto exit;
	}
exit:
	if (status) {
		(*ppboardobj)->destruct(*ppboardobj);
	}

	return status;
}

static int _vfe_var_pmudatainit_single_sensed_temp(struct gk20a *g,
						   struct boardobj *board_obj_ptr,
						   struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_var_single_sensed_temp *pvfe_var_single_sensed_temp;
	struct nv_pmu_vfe_var_single_sensed_temp *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_single_sensed(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_var_single_sensed_temp =
		(struct vfe_var_single_sensed_temp *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_var_single_sensed_temp *)
		ppmudata;
	pset->therm_channel_index =
		 pvfe_var_single_sensed_temp->therm_channel_index;
	pset->temp_hysteresis_positive =
		 pvfe_var_single_sensed_temp->temp_hysteresis_positive;
	pset->temp_hysteresis_negative =
		 pvfe_var_single_sensed_temp->temp_hysteresis_negative;
	pset->temp_default =
		 pvfe_var_single_sensed_temp->temp_default;
	return status;
}

static u32 vfe_var_construct_single_sensed_temp(struct gk20a *g,
						struct boardobj **ppboardobj,
						u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single_sensed_temp *pvfevar;
	struct vfe_var_single_sensed_temp *ptmpvar =
			(struct vfe_var_single_sensed_temp *)pargs;
	u32 status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_TEMP) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_TEMP);
	status = vfe_var_construct_single_sensed(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single_sensed_temp *)*ppboardobj;

	pvfevar->super.super.super.super.pmudatainit =
			_vfe_var_pmudatainit_single_sensed_temp;

	pvfevar->therm_channel_index =
		 ptmpvar->therm_channel_index;
	pvfevar->temp_hysteresis_positive =
		 ptmpvar->temp_hysteresis_positive;
	pvfevar->temp_hysteresis_negative =
		 ptmpvar->temp_hysteresis_negative;
	pvfevar->temp_default =
		 ptmpvar->temp_default;
	pvfevar->super.super.super.b_is_dynamic = false;
	pvfevar->super.super.super.b_is_dynamic_valid = true;

	return status;
}

static int _vfe_var_pmudatainit_single_voltage(struct gk20a *g,
					       struct boardobj *board_obj_ptr,
					       struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;

	nvgpu_log_info(g, " ");

	status = _vfe_var_pmudatainit_single(g, board_obj_ptr, ppmudata);

	return status;
}

static int vfe_var_construct_single_voltage(struct gk20a *g,
					    struct boardobj **ppboardobj,
					    u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single_voltage *pvfevar;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_VAR_TYPE_SINGLE_VOLTAGE) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE_VOLTAGE);
	status = vfe_var_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single_voltage *)*ppboardobj;

	pvfevar->super.super.super.pmudatainit =
			_vfe_var_pmudatainit_single_voltage;

	pvfevar->super.super.b_is_dynamic = false;
	pvfevar->super.super.b_is_dynamic_valid = true;

	return status;
}

static struct vfe_var *construct_vfe_var(struct gk20a *g, void *pargs)
{
	struct boardobj *board_obj_ptr = NULL;
	int status;

	nvgpu_log_info(g, " ");
	switch (BOARDOBJ_GET_TYPE(pargs)) {
	case CTRL_PERF_VFE_VAR_TYPE_DERIVED_PRODUCT:
		status = vfe_var_construct_derived_product(g, &board_obj_ptr,
			sizeof(struct vfe_var_derived_product), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_DERIVED_SUM:
		status = vfe_var_construct_derived_sum(g, &board_obj_ptr,
			sizeof(struct vfe_var_derived_sum), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_SINGLE_FREQUENCY:
		status = vfe_var_construct_single_frequency(g, &board_obj_ptr,
			sizeof(struct vfe_var_single_frequency), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_FUSE:
		status = vfe_var_construct_single_sensed_fuse(g, &board_obj_ptr,
			sizeof(struct vfe_var_single_sensed_fuse), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_TEMP:
		status = vfe_var_construct_single_sensed_temp(g, &board_obj_ptr,
			sizeof(struct vfe_var_single_sensed_temp), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_SINGLE_VOLTAGE:
		status = vfe_var_construct_single_voltage(g, &board_obj_ptr,
			sizeof(struct vfe_var_single_voltage), pargs);
		break;

	case CTRL_PERF_VFE_VAR_TYPE_DERIVED:
	case CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED:
	case CTRL_PERF_VFE_VAR_TYPE_SINGLE:
	default:
		return NULL;
	}

	if (status) {
		return NULL;
	}

	nvgpu_log_info(g, "done");

	return (struct vfe_var *)board_obj_ptr;
}

static int devinit_get_vfe_var_table(struct gk20a *g,
				     struct vfe_vars *pvfevarobjs)
{
	int status = 0;
	u8 *vfevars_tbl_ptr = NULL;
	struct vbios_vfe_3x_header_struct vfevars_tbl_header = { 0 };
	struct vbios_vfe_3x_var_entry_struct var = { 0 };
	u8 *vfevars_tbl_entry_ptr = NULL;
	u8 *rd_offset_ptr = NULL;
	u32 index = 0;
	struct vfe_var *pvar;
	u8 var_type;
	u32 szfmt;
	union {
		struct boardobj board_obj;
		struct vfe_var super;
		struct vfe_var_derived_product derived_product;
		struct vfe_var_derived_sum derived_sum;
		struct vfe_var_single_sensed_fuse single_sensed_fuse;
		struct vfe_var_single_sensed_temp single_sensed_temp;
	} var_data;

	nvgpu_log_info(g, " ");

	vfevars_tbl_ptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.perf_token,
			CONTINUOUS_VIRTUAL_BINNING_TABLE);
	if (vfevars_tbl_ptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	memcpy(&vfevars_tbl_header, vfevars_tbl_ptr,
	       VBIOS_CLOCKS_TABLE_1X_HEADER_SIZE_07);
	if (vfevars_tbl_header.header_size !=
	    VBIOS_CLOCKS_TABLE_1X_HEADER_SIZE_07){
		status = -EINVAL;
		goto done;
	}

	if (vfevars_tbl_header.vfe_var_entry_size ==
			VBIOS_VFE_3X_VAR_ENTRY_SIZE_19) {
		szfmt = VBIOS_VFE_3X_VAR_ENTRY_SIZE_19;
	} else if (vfevars_tbl_header.vfe_var_entry_size ==
			VBIOS_VFE_3X_VAR_ENTRY_SIZE_11) {
		szfmt = VBIOS_VFE_3X_VAR_ENTRY_SIZE_11;
	} else {
		status = -EINVAL;
		goto done;
	}

	/* Read table entries*/
	vfevars_tbl_entry_ptr = vfevars_tbl_ptr +
		vfevars_tbl_header.header_size;
	for (index = 0;
	     index < vfevars_tbl_header.vfe_var_entry_count;
	     index++) {
		rd_offset_ptr = vfevars_tbl_entry_ptr +
				(index * vfevars_tbl_header.vfe_var_entry_size);
		memcpy(&var, rd_offset_ptr, szfmt);

		var_data.super.out_range_min = var.out_range_min;
		var_data.super.out_range_max = var.out_range_max;

		switch ((u8)var.type) {
		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_DISABLED:
			continue;
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_SINGLE_FREQUENCY:
			var_type = CTRL_PERF_VFE_VAR_TYPE_SINGLE_FREQUENCY;
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_SINGLE_VOLTAGE:
			var_type = CTRL_PERF_VFE_VAR_TYPE_SINGLE_VOLTAGE;
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_SINGLE_SENSED_TEMP:
			var_type = CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_TEMP;
			var_data.single_sensed_temp.temp_default = 0x9600;
			var_data.single_sensed_temp.therm_channel_index =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSTEMP_TH_CH_IDX);
			var_data.single_sensed_temp.temp_hysteresis_positive =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSTEMP_HYS_POS) << 5;
			var_data.single_sensed_temp.temp_hysteresis_negative =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSTEMP_HYS_NEG) << 5;
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_SINGLE_SENSED_FUSE:
			var_type = CTRL_PERF_VFE_VAR_TYPE_SINGLE_SENSED_FUSE;
			var_data.single_sensed_fuse.vfield_info.v_field_id =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_VFIELD_ID);
			var_data.single_sensed_fuse.vfield_ver_info.v_field_id_ver =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_VFIELD_ID_VER);
			var_data.single_sensed_fuse.vfield_ver_info.ver_expected =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_EXPECTED_VER);
			var_data.single_sensed_fuse.vfield_ver_info.b_use_default_on_ver_check_fail =
				(BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_USE_DEFAULT_ON_VER_CHECK_FAIL) &&
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_USE_DEFAULT_ON_VER_CHECK_FAIL_YES);
			var_data.single_sensed_fuse.b_fuse_value_signed =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_SSFUSE_VALUE_SIGNED_INTEGER);
			var_data.single_sensed_fuse.vfield_info.fuse_val_default =
				var.param1;
			if (szfmt >= VBIOS_VFE_3X_VAR_ENTRY_SIZE_19) {
				var_data.single_sensed_fuse.vfield_info.hw_correction_scale  =
					(int)var.param2;
				var_data.single_sensed_fuse.vfield_info.hw_correction_offset =
					var.param3;
			} else {
				var_data.single_sensed_fuse.vfield_info.hw_correction_scale =
					1 << 12;
				var_data.single_sensed_fuse.vfield_info.hw_correction_offset =
					0;
				if ((var_data.single_sensed_fuse.vfield_info.v_field_id ==
				     VFIELD_ID_STRAP_IDDQ) ||
				    (var_data.single_sensed_fuse.vfield_info.v_field_id ==
				     VFIELD_ID_STRAP_IDDQ_1)) {
					var_data.single_sensed_fuse.vfield_info.hw_correction_scale =
						50 << 12;
				}
			}
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_DERIVED_PRODUCT:
			var_type = CTRL_PERF_VFE_VAR_TYPE_DERIVED_PRODUCT;
			var_data.derived_product.var_idx0 =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_DPROD_VFE_VAR_IDX_0);
			var_data.derived_product.var_idx1 =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_DPROD_VFE_VAR_IDX_1);
			break;

		case VBIOS_VFE_3X_VAR_ENTRY_TYPE_DERIVED_SUM:
			var_type = CTRL_PERF_VFE_VAR_TYPE_DERIVED_SUM;
			var_data.derived_sum.var_idx0 =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_DSUM_VFE_VAR_IDX_0);
			var_data.derived_sum.var_idx1 =
				(u8)BIOS_GET_FIELD(var.param0,
					VBIOS_VFE_3X_VAR_ENTRY_PAR0_DSUM_VFE_VAR_IDX_1);
			break;
		default:
			status = -EINVAL;
			goto done;
		}
		var_data.board_obj.type = var_type;
		var_data.board_obj.type_mask = 0;

		pvar = construct_vfe_var(g, &var_data);
		if (pvar == NULL) {
			nvgpu_err(g,
				  "error constructing vfe_var boardobj %d",
				  index);
			status = -EINVAL;
			goto done;
		}

		status = boardobjgrp_objinsert(&pvfevarobjs->super.super,
					       (struct boardobj *)pvar, index);
		if (status) {
			nvgpu_err(g, "error adding vfe_var boardobj %d", index);
			status = -EINVAL;
			goto done;
		}
	}
	pvfevarobjs->polling_periodms = vfevars_tbl_header.polling_periodms;
done:
	nvgpu_log_info(g, "done status %x", status);
	return status;
}

static int vfe_var_construct_single(struct gk20a *g,
				    struct boardobj **ppboardobj,
				    u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_var_single *pvfevar;
	int status = 0;

	nvgpu_log_info(g, " ");

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_VAR_TYPE_SINGLE);
	status = vfe_var_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfevar = (struct vfe_var_single *)*ppboardobj;

	pvfevar->super.super.pmudatainit =
			_vfe_var_pmudatainit_single;

	pvfevar->override_type = CTRL_PERF_VFE_VAR_SINGLE_OVERRIDE_TYPE_NONE;
	pvfevar->override_value = 0;

	nvgpu_log_info(g, "Done");
	return status;
}
