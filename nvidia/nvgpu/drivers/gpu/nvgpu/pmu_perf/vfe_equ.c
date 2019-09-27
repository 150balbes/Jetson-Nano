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
#include "vfe_equ.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e255.h"
#include "ctrl/ctrlclk.h"
#include "ctrl/ctrlvolt.h"

static struct vfe_equ *construct_vfe_equ(struct gk20a *g, void *pargs);
static int devinit_get_vfe_equ_table(struct gk20a *g,
	struct vfe_equs *pequobjs);

static int _vfe_equs_pmudatainit(struct gk20a *g,
				 struct boardobjgrp *pboardobjgrp,
				 struct nv_pmu_boardobjgrp_super *pboardobjgrppmu)
{
	int status = 0;

	status = boardobjgrp_pmudatainit_e255(g, pboardobjgrp, pboardobjgrppmu);
	if (status) {
		nvgpu_err(g, "error updating pmu boardobjgrp for vfe equ 0x%x",
			  status);
		goto done;
	}

done:
	return status;
}

static int _vfe_equs_pmudata_instget(struct gk20a *g,
				     struct nv_pmu_boardobjgrp *pmuboardobjgrp,
				     struct nv_pmu_boardobj **ppboardobjpmudata,
				     u8 idx)
{
	struct nv_pmu_perf_vfe_equ_boardobj_grp_set  *pgrp_set =
		(struct nv_pmu_perf_vfe_equ_boardobj_grp_set *)pmuboardobjgrp;

	nvgpu_log_info(g, " ");

	/* check whether pmuboardobjgrp has a valid boardobj in index */
	if (idx >= CTRL_BOARDOBJGRP_E255_MAX_OBJECTS) {
		return -EINVAL;
	}

	*ppboardobjpmudata = (struct nv_pmu_boardobj *)
		&pgrp_set->objects[idx].data.board_obj;
	nvgpu_log_info(g, " Done");
	return 0;
}

int vfe_equ_sw_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;
	struct vfe_equs *pvfeequobjs;

	nvgpu_log_info(g, " ");

	status = boardobjgrpconstruct_e255(g, &g->perf_pmu.vfe_equobjs.super);
	if (status) {
		nvgpu_err(g,
			  "error creating boardobjgrp for clk domain, status - 0x%x",
			  status);
		goto done;
	}

	pboardobjgrp = &g->perf_pmu.vfe_equobjs.super.super;
	pvfeequobjs = &(g->perf_pmu.vfe_equobjs);

	BOARDOBJGRP_PMU_CONSTRUCT(pboardobjgrp, PERF, VFE_EQU);

	status = BOARDOBJGRP_PMU_CMD_GRP_SET_CONSTRUCT(g, pboardobjgrp,
			perf, PERF, vfe_equ, VFE_EQU);
	if (status) {
		nvgpu_err(g,
			"error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			status);
		goto done;
	}

	pboardobjgrp->pmudatainit  = _vfe_equs_pmudatainit;
	pboardobjgrp->pmudatainstget  = _vfe_equs_pmudata_instget;

	status = devinit_get_vfe_equ_table(g, pvfeequobjs);
	if (status) {
		goto done;
	}

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}

int vfe_equ_pmu_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;

	nvgpu_log_info(g, " ");

	pboardobjgrp = &g->perf_pmu.vfe_equobjs.super.super;

	if (!pboardobjgrp->bconstructed) {
		return -EINVAL;
	}

	status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);

	nvgpu_log_info(g, "Done");
	return status;
}

static int devinit_get_vfe_equ_table(struct gk20a *g,
				     struct vfe_equs *pvfeequobjs)
{
	int status = 0;
	u8 *vfeequs_tbl_ptr = NULL;
	struct vbios_vfe_3x_header_struct vfeequs_tbl_header = { 0 };
	struct vbios_vfe_3x_equ_entry_struct equ = { 0 };
	u8 *vfeequs_tbl_entry_ptr = NULL;
	u8 *rd_offset_ptr = NULL;
	u32 index = 0;
	struct vfe_equ *pequ;
	u8 equ_type = 0;
	u32 szfmt;
	union {
		struct boardobj board_obj;
		struct vfe_equ super;
		struct vfe_equ_compare compare;
		struct vfe_equ_minmax minmax;
		struct vfe_equ_quadratic quadratic;
	} equ_data;

	nvgpu_log_info(g, " ");

	vfeequs_tbl_ptr = (u8 *)nvgpu_bios_get_perf_table_ptrs(g,
			g->bios.perf_token,
			CONTINUOUS_VIRTUAL_BINNING_TABLE);

	if (vfeequs_tbl_ptr == NULL) {
		status = -EINVAL;
		goto done;
	}

	memcpy(&vfeequs_tbl_header, vfeequs_tbl_ptr,
			VBIOS_CLOCKS_TABLE_1X_HEADER_SIZE_07);
	if (vfeequs_tbl_header.header_size != VBIOS_CLOCKS_TABLE_1X_HEADER_SIZE_07) {
		status = -EINVAL;
		goto done;
	}

	if (vfeequs_tbl_header.vfe_equ_entry_size ==
			VBIOS_VFE_3X_EQU_ENTRY_SIZE_17) {
		szfmt = VBIOS_VFE_3X_EQU_ENTRY_SIZE_17;
	} else if (vfeequs_tbl_header.vfe_equ_entry_size ==
			VBIOS_VFE_3X_EQU_ENTRY_SIZE_18) {
		szfmt = VBIOS_VFE_3X_EQU_ENTRY_SIZE_18;
	} else {
		status = -EINVAL;
		goto done;
	}

	vfeequs_tbl_entry_ptr = vfeequs_tbl_ptr +
		vfeequs_tbl_header.header_size +
		(vfeequs_tbl_header.vfe_var_entry_count *
		 vfeequs_tbl_header.vfe_var_entry_size);

	for (index = 0;
	     index < vfeequs_tbl_header.vfe_equ_entry_count;
	     index++) {
		memset(&equ, 0, sizeof(struct vbios_vfe_3x_equ_entry_struct));

		rd_offset_ptr = vfeequs_tbl_entry_ptr +
			(index * vfeequs_tbl_header.vfe_equ_entry_size);

		memcpy(&equ, rd_offset_ptr, szfmt);

		equ_data.super.var_idx = (u8)equ.var_idx;
		equ_data.super.equ_idx_next =
			(equ.equ_idx_next == VBIOS_VFE_3X_EQU_ENTRY_IDX_INVALID) ?
			CTRL_BOARDOBJ_IDX_INVALID : (u8)equ.equ_idx_next;
		equ_data.super.out_range_min = equ.out_range_min;
		equ_data.super.out_range_max = equ.out_range_max;

		switch (BIOS_GET_FIELD(equ.param3, VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE)) {
		case VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE_UNITLESS:
			equ_data.super.output_type =
				CTRL_PERF_VFE_EQU_OUTPUT_TYPE_UNITLESS;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE_FREQ_MHZ:
			equ_data.super.output_type =
				CTRL_PERF_VFE_EQU_OUTPUT_TYPE_FREQ_MHZ;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE_VOLT_UV:
			equ_data.super.output_type =
				CTRL_PERF_VFE_EQU_OUTPUT_TYPE_VOLT_UV;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE_VF_GAIN:
			equ_data.super.output_type =
				CTRL_PERF_VFE_EQU_OUTPUT_TYPE_VF_GAIN;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_PAR3_OUTPUT_TYPE_VOLT_DELTA_UV:
			equ_data.super.output_type =
				CTRL_PERF_VFE_EQU_OUTPUT_TYPE_VOLT_DELTA_UV;
			break;

		default:
			nvgpu_err(g, "unrecognized output id @vfeequ index %d",
				  index);
			goto done;
		}

		switch ((u8)equ.type) {
		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_DISABLED:
		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_QUADRATIC_FXP:
		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_MINMAX_FXP:
			continue;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_QUADRATIC:
			equ_type = CTRL_PERF_VFE_EQU_TYPE_QUADRATIC;
			equ_data.quadratic.coeffs[0] = equ.param0;
			equ_data.quadratic.coeffs[1] = equ.param1;
			equ_data.quadratic.coeffs[2] = equ.param2;
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_MINMAX:
			equ_type = CTRL_PERF_VFE_EQU_TYPE_MINMAX;
			equ_data.minmax.b_max = BIOS_GET_FIELD(equ.param0,
				VBIOS_VFE_3X_EQU_ENTRY_PAR0_MINMAX_CRIT) &&
				VBIOS_VFE_3X_EQU_ENTRY_PAR0_MINMAX_CRIT_MAX;
			equ_data.minmax.equ_idx0 = (u8)BIOS_GET_FIELD(
				equ.param0,
				VBIOS_VFE_3X_EQU_ENTRY_PAR0_MINMAX_VFE_EQU_IDX_0);
			equ_data.minmax.equ_idx1 = (u8)BIOS_GET_FIELD(
				equ.param0,
				VBIOS_VFE_3X_EQU_ENTRY_PAR0_MINMAX_VFE_EQU_IDX_1);
			break;

		case VBIOS_VFE_3X_EQU_ENTRY_TYPE_COMPARE:
		{
			u8 cmp_func = (u8)BIOS_GET_FIELD(
				equ.param1,
				VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_FUNCTION);
			equ_type = CTRL_PERF_VFE_EQU_TYPE_COMPARE;

			switch (cmp_func) {
			case VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_FUNCTION_EQUAL:
				equ_data.compare.func_id =
					CTRL_PERF_VFE_EQU_COMPARE_FUNCTION_EQUAL;
				break;

			case VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_FUNCTION_GREATER_EQ:
				equ_data.compare.func_id =
					CTRL_PERF_VFE_EQU_COMPARE_FUNCTION_GREATER_EQ;
				break;
			case VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_FUNCTION_GREATER:
				equ_data.compare.func_id =
					CTRL_PERF_VFE_EQU_COMPARE_FUNCTION_GREATER;
				break;
			default:
				nvgpu_err(g,
					  "invalid vfe compare index %x type %x ",
					  index, cmp_func);
				status = -EINVAL;
				goto done;
			}
			equ_data.compare.equ_idx_true = (u8)BIOS_GET_FIELD(
				equ.param1,
				VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_VFE_EQU_IDX_TRUE);
			equ_data.compare.equ_idx_false = (u8)BIOS_GET_FIELD(
				equ.param1,
				VBIOS_VFE_3X_EQU_ENTRY_PAR1_COMPARE_VFE_EQU_IDX_FALSE);
			equ_data.compare.criteria = equ.param0;
			break;
		}
		default:
			status = -EINVAL;
			nvgpu_err(g, "Invalid equ[%d].type = 0x%x.",
				index, (u8)equ.type);
			goto done;
		}

		equ_data.board_obj.type = equ_type;
		pequ = construct_vfe_equ(g, (void *)&equ_data);

		if (pequ == NULL) {
			nvgpu_err(g,
			"error constructing vfe_equ boardobj %d", index);
			status = -EINVAL;
			goto done;
		}

		status = boardobjgrp_objinsert(&pvfeequobjs->super.super,
					       (struct boardobj *)pequ, index);
		if (status) {
			nvgpu_err(g, "error adding vfe_equ boardobj %d", index);
			status = -EINVAL;
			goto done;
		}
	}
done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}

static int _vfe_equ_pmudatainit_super(struct gk20a *g,
				      struct boardobj *board_obj_ptr,
				      struct nv_pmu_boardobj *ppmudata)
{
	u32 status = 0;
	struct vfe_equ *pvfe_equ;
	struct nv_pmu_vfe_equ *pset;

	nvgpu_log_info(g, " ");

	status = boardobj_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_equ = (struct vfe_equ *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_equ *)
		ppmudata;

	pset->var_idx      = pvfe_equ->var_idx;
	pset->equ_idx_next  = pvfe_equ->equ_idx_next;
	pset->output_type  = pvfe_equ->output_type;
	pset->out_range_min = pvfe_equ->out_range_min;
	pset->out_range_max = pvfe_equ->out_range_max;

	return status;
}

static int vfe_equ_construct_super(struct gk20a *g,
				   struct boardobj **ppboardobj,
				   u16 size, void *pargs)
{
	struct vfe_equ *pvfeequ;
	struct vfe_equ *ptmpequ = (struct vfe_equ *)pargs;
	int status = 0;

	status = boardobj_construct_super(g, ppboardobj,
		size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfeequ = (struct vfe_equ *)*ppboardobj;

	pvfeequ->super.pmudatainit =
			_vfe_equ_pmudatainit_super;

	pvfeequ->var_idx = ptmpequ->var_idx;
	pvfeequ->equ_idx_next = ptmpequ->equ_idx_next;
	pvfeequ->output_type = ptmpequ->output_type;
	pvfeequ->out_range_min = ptmpequ->out_range_min;
	pvfeequ->out_range_max = ptmpequ->out_range_max;

	return status;
}

static int _vfe_equ_pmudatainit_compare(struct gk20a *g,
					struct boardobj *board_obj_ptr,
					struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_equ_compare *pvfe_equ_compare;
	struct nv_pmu_vfe_equ_compare *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_equ_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_equ_compare = (struct vfe_equ_compare *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_equ_compare *) ppmudata;

	pset->func_id = pvfe_equ_compare->func_id;
	pset->equ_idx_true = pvfe_equ_compare->equ_idx_true;
	pset->equ_idx_false = pvfe_equ_compare->equ_idx_false;
	pset->criteria = pvfe_equ_compare->criteria;

	return status;
}


static int vfe_equ_construct_compare(struct gk20a *g,
				     struct boardobj **ppboardobj,
				     u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_equ_compare *pvfeequ;
	struct vfe_equ_compare *ptmpequ =
			(struct vfe_equ_compare *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_EQU_TYPE_COMPARE) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_EQU_TYPE_COMPARE);
	status = vfe_equ_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfeequ = (struct vfe_equ_compare *)*ppboardobj;

	pvfeequ->super.super.pmudatainit =
			_vfe_equ_pmudatainit_compare;

	pvfeequ->func_id = ptmpequ->func_id;
	pvfeequ->equ_idx_true = ptmpequ->equ_idx_true;
	pvfeequ->equ_idx_false = ptmpequ->equ_idx_false;
	pvfeequ->criteria = ptmpequ->criteria;


	return status;
}

static int _vfe_equ_pmudatainit_minmax(struct gk20a *g,
				       struct boardobj *board_obj_ptr,
				       struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_equ_minmax *pvfe_equ_minmax;
	struct nv_pmu_vfe_equ_minmax *pset;

	nvgpu_log_info(g, " ");

	status = _vfe_equ_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_equ_minmax = (struct vfe_equ_minmax *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_equ_minmax *)
		ppmudata;

	pset->b_max = pvfe_equ_minmax->b_max;
	pset->equ_idx0 = pvfe_equ_minmax->equ_idx0;
	pset->equ_idx1 = pvfe_equ_minmax->equ_idx1;

	return status;
}

static int vfe_equ_construct_minmax(struct gk20a *g,
				    struct boardobj **ppboardobj,
				    u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_equ_minmax *pvfeequ;
	struct vfe_equ_minmax *ptmpequ =
			(struct vfe_equ_minmax *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_EQU_TYPE_MINMAX) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_EQU_TYPE_MINMAX);
	status = vfe_equ_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfeequ = (struct vfe_equ_minmax *)*ppboardobj;

	pvfeequ->super.super.pmudatainit =
			_vfe_equ_pmudatainit_minmax;
	pvfeequ->b_max = ptmpequ->b_max;
	pvfeequ->equ_idx0 = ptmpequ->equ_idx0;
	pvfeequ->equ_idx1 = ptmpequ->equ_idx1;

	return status;
}

static int _vfe_equ_pmudatainit_quadratic(struct gk20a *g,
					  struct boardobj *board_obj_ptr,
					  struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct vfe_equ_quadratic *pvfe_equ_quadratic;
	struct nv_pmu_vfe_equ_quadratic *pset;
	u32 i;

	nvgpu_log_info(g, " ");

	status = _vfe_equ_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pvfe_equ_quadratic = (struct vfe_equ_quadratic *)board_obj_ptr;

	pset = (struct nv_pmu_vfe_equ_quadratic *) ppmudata;

	for (i = 0; i < CTRL_PERF_VFE_EQU_QUADRATIC_COEFF_COUNT; i++) {
		pset->coeffs[i] = pvfe_equ_quadratic->coeffs[i];
	}

	return status;
}

static int vfe_equ_construct_quadratic(struct gk20a *g,
				       struct boardobj **ppboardobj,
				       u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct vfe_equ_quadratic *pvfeequ;
	struct vfe_equ_quadratic *ptmpequ =
			(struct vfe_equ_quadratic *)pargs;
	int status = 0;
	u32 i;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_PERF_VFE_EQU_TYPE_QUADRATIC) {
		return -EINVAL;
	}

	ptmpobj->type_mask |= BIT(CTRL_PERF_VFE_EQU_TYPE_QUADRATIC);
	status = vfe_equ_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pvfeequ = (struct vfe_equ_quadratic *)*ppboardobj;

	pvfeequ->super.super.pmudatainit =
			_vfe_equ_pmudatainit_quadratic;

	for (i = 0; i < CTRL_PERF_VFE_EQU_QUADRATIC_COEFF_COUNT; i++) {
		pvfeequ->coeffs[i] = ptmpequ->coeffs[i];
	}

	return status;
}

static struct vfe_equ *construct_vfe_equ(struct gk20a *g, void *pargs)
{
	struct boardobj *board_obj_ptr = NULL;
	int status;

	nvgpu_log_info(g, " ");

	switch (BOARDOBJ_GET_TYPE(pargs)) {
	case CTRL_PERF_VFE_EQU_TYPE_COMPARE:
		status = vfe_equ_construct_compare(g, &board_obj_ptr,
			sizeof(struct vfe_equ_compare), pargs);
		break;

	case CTRL_PERF_VFE_EQU_TYPE_MINMAX:
		status = vfe_equ_construct_minmax(g, &board_obj_ptr,
			sizeof(struct vfe_equ_minmax), pargs);
		break;

	case CTRL_PERF_VFE_EQU_TYPE_QUADRATIC:
		status = vfe_equ_construct_quadratic(g, &board_obj_ptr,
			sizeof(struct vfe_equ_quadratic), pargs);
		break;

	default:
		return NULL;

	}

	if (status) {
		return NULL;
	}

	nvgpu_log_info(g, " Done");

	return (struct vfe_equ *)board_obj_ptr;
}
