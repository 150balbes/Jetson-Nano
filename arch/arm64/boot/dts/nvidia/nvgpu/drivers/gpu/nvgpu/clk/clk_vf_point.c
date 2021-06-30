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

#include <nvgpu/gk20a.h>

#include "clk.h"
#include "clk_vf_point.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"
#include "ctrl/ctrlclk.h"
#include "ctrl/ctrlvolt.h"

static int _clk_vf_point_pmudatainit_super(struct gk20a *g, struct boardobj
	*board_obj_ptr,	struct nv_pmu_boardobj *ppmudata);

static int _clk_vf_points_pmudatainit(struct gk20a *g,
				      struct boardobjgrp *pboardobjgrp,
				      struct nv_pmu_boardobjgrp_super *pboardobjgrppmu)
{
	u32 status = 0;

	status = boardobjgrp_pmudatainit_e32(g, pboardobjgrp, pboardobjgrppmu);
	if (status) {
		nvgpu_err(g,
			  "error updating pmu boardobjgrp for clk vfpoint 0x%x",
			  status);
		goto done;
	}

done:
	return status;
}

static int _clk_vf_points_pmudata_instget(struct gk20a *g,
					  struct nv_pmu_boardobjgrp *pmuboardobjgrp,
					  struct nv_pmu_boardobj **ppboardobjpmudata,
					  u8 idx)
{
	struct nv_pmu_clk_clk_vf_point_boardobj_grp_set  *pgrp_set =
		(struct nv_pmu_clk_clk_vf_point_boardobj_grp_set *)
		pmuboardobjgrp;

	nvgpu_log_info(g, " ");

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (idx >= CTRL_BOARDOBJGRP_E255_MAX_OBJECTS) {
		return -EINVAL;
	}

	*ppboardobjpmudata = (struct nv_pmu_boardobj *)
		&pgrp_set->objects[idx].data.board_obj;
	nvgpu_log_info(g, " Done");
	return 0;
}

static int _clk_vf_points_pmustatus_instget(struct gk20a *g,
					    void *pboardobjgrppmu,
					    struct nv_pmu_boardobj_query **ppboardobjpmustatus,
					    u8 idx)
{
	struct nv_pmu_clk_clk_vf_point_boardobj_grp_get_status *pgrp_get_status =
		(struct nv_pmu_clk_clk_vf_point_boardobj_grp_get_status *)
		pboardobjgrppmu;

	/*check whether pmuboardobjgrp has a valid boardobj in index*/
	if (idx >= CTRL_BOARDOBJGRP_E255_MAX_OBJECTS) {
		return -EINVAL;
	}

	*ppboardobjpmustatus = (struct nv_pmu_boardobj_query *)
			&pgrp_get_status->objects[idx].data.board_obj;
	return 0;
}

int clk_vf_point_sw_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;

	nvgpu_log_info(g, " ");

	status = boardobjgrpconstruct_e255(g, &g->clk_pmu.clk_vf_pointobjs.super);
	if (status) {
		nvgpu_err(g,
		"error creating boardobjgrp for clk vfpoint, status - 0x%x",
		status);
		goto done;
	}

	pboardobjgrp = &g->clk_pmu.clk_vf_pointobjs.super.super;

	BOARDOBJGRP_PMU_CONSTRUCT(pboardobjgrp, CLK, CLK_VF_POINT);

	status = BOARDOBJGRP_PMU_CMD_GRP_SET_CONSTRUCT(g, pboardobjgrp,
			clk, CLK, clk_vf_point, CLK_VF_POINT);
	if (status) {
		nvgpu_err(g,
			"error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			status);
		goto done;
	}

	status = BOARDOBJGRP_PMU_CMD_GRP_GET_STATUS_CONSTRUCT(g,
				&g->clk_pmu.clk_vf_pointobjs.super.super,
				clk, CLK, clk_vf_point, CLK_VF_POINT);
	if (status) {
		nvgpu_err(g,
			"error constructing PMU_BOARDOBJ_CMD_GRP_SET interface - 0x%x",
			status);
		goto done;
	}

	pboardobjgrp->pmudatainit = _clk_vf_points_pmudatainit;
	pboardobjgrp->pmudatainstget  = _clk_vf_points_pmudata_instget;
	pboardobjgrp->pmustatusinstget  = _clk_vf_points_pmustatus_instget;

done:
	nvgpu_log_info(g, " done status %x", status);
	return status;
}

int clk_vf_point_pmu_setup(struct gk20a *g)
{
	int status;
	struct boardobjgrp *pboardobjgrp = NULL;

	nvgpu_log_info(g, " ");

	pboardobjgrp = &g->clk_pmu.clk_vf_pointobjs.super.super;

	if (!pboardobjgrp->bconstructed) {
		return -EINVAL;
	}

	status = pboardobjgrp->pmuinithandle(g, pboardobjgrp);

	nvgpu_log_info(g, "Done");
	return status;
}

static int clk_vf_point_construct_super(struct gk20a *g,
					struct boardobj **ppboardobj,
					u16 size, void *pargs)
{
	struct clk_vf_point *pclkvfpoint;
	struct clk_vf_point *ptmpvfpoint =
			(struct clk_vf_point *)pargs;
	int status = 0;

	status = boardobj_construct_super(g, ppboardobj,
		size, pargs);
	if (status) {
		return -EINVAL;
	}

	pclkvfpoint = (struct clk_vf_point *)*ppboardobj;

	pclkvfpoint->super.pmudatainit =
			_clk_vf_point_pmudatainit_super;

	pclkvfpoint->vfe_equ_idx = ptmpvfpoint->vfe_equ_idx;
	pclkvfpoint->volt_rail_idx = ptmpvfpoint->volt_rail_idx;

	return status;
}

static int _clk_vf_point_pmudatainit_volt(struct gk20a *g,
					  struct boardobj *board_obj_ptr,
					  struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct clk_vf_point_volt *pclk_vf_point_volt;
	struct nv_pmu_clk_clk_vf_point_volt_boardobj_set *pset;

	nvgpu_log_info(g, " ");

	status = _clk_vf_point_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pclk_vf_point_volt =
		(struct clk_vf_point_volt *)board_obj_ptr;

	pset = (struct nv_pmu_clk_clk_vf_point_volt_boardobj_set *)
		ppmudata;

	pset->source_voltage_uv = pclk_vf_point_volt->source_voltage_uv;
	pset->freq_delta.data = pclk_vf_point_volt->freq_delta.data;
	pset->freq_delta.type = pclk_vf_point_volt->freq_delta.type;

	return status;
}

static int _clk_vf_point_pmudatainit_freq(struct gk20a *g,
					  struct boardobj *board_obj_ptr,
					  struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct clk_vf_point_freq *pclk_vf_point_freq;
	struct nv_pmu_clk_clk_vf_point_freq_boardobj_set *pset;

	nvgpu_log_info(g, " ");

	status = _clk_vf_point_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pclk_vf_point_freq =
		(struct clk_vf_point_freq *)board_obj_ptr;

	pset = (struct nv_pmu_clk_clk_vf_point_freq_boardobj_set *)
		ppmudata;

	pset->freq_mhz =
		clkvfpointfreqmhzget(g, &pclk_vf_point_freq->super);

	pset->volt_delta_uv = pclk_vf_point_freq->volt_delta_uv;

	return status;
}

static int clk_vf_point_construct_volt(struct gk20a *g,
				       struct boardobj **ppboardobj,
				       u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct clk_vf_point_volt *pclkvfpoint;
	struct clk_vf_point_volt *ptmpvfpoint =
			(struct clk_vf_point_volt *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_CLK_CLK_VF_POINT_TYPE_VOLT) {
		return -EINVAL;
	}

	ptmpobj->type_mask = BIT(CTRL_CLK_CLK_VF_POINT_TYPE_VOLT);
	status = clk_vf_point_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pclkvfpoint = (struct clk_vf_point_volt *)*ppboardobj;

	pclkvfpoint->super.super.pmudatainit =
			_clk_vf_point_pmudatainit_volt;

	pclkvfpoint->source_voltage_uv = ptmpvfpoint->source_voltage_uv;
	pclkvfpoint->freq_delta = ptmpvfpoint->freq_delta;

	return status;
}

static int clk_vf_point_construct_freq(struct gk20a *g,
				       struct boardobj **ppboardobj,
				       u16 size, void *pargs)
{
	struct boardobj *ptmpobj = (struct boardobj *)pargs;
	struct clk_vf_point_freq *pclkvfpoint;
	struct clk_vf_point_freq *ptmpvfpoint =
			(struct clk_vf_point_freq *)pargs;
	int status = 0;

	if (BOARDOBJ_GET_TYPE(pargs) != CTRL_CLK_CLK_VF_POINT_TYPE_FREQ) {
		return -EINVAL;
	}

	ptmpobj->type_mask = BIT(CTRL_CLK_CLK_VF_POINT_TYPE_FREQ);
	status = clk_vf_point_construct_super(g, ppboardobj, size, pargs);
	if (status) {
		return -EINVAL;
	}

	pclkvfpoint = (struct clk_vf_point_freq *)*ppboardobj;

	pclkvfpoint->super.super.pmudatainit =
			_clk_vf_point_pmudatainit_freq;

	clkvfpointfreqmhzset(g, &pclkvfpoint->super,
		clkvfpointfreqmhzget(g, &ptmpvfpoint->super));

	return status;
}

struct clk_vf_point *construct_clk_vf_point(struct gk20a *g, void *pargs)
{
	struct boardobj *board_obj_ptr = NULL;
	int status;

	nvgpu_log_info(g, " ");
	switch (BOARDOBJ_GET_TYPE(pargs)) {
	case CTRL_CLK_CLK_VF_POINT_TYPE_FREQ:
		status = clk_vf_point_construct_freq(g, &board_obj_ptr,
			sizeof(struct clk_vf_point_freq), pargs);
		break;

	case CTRL_CLK_CLK_VF_POINT_TYPE_VOLT:
		status = clk_vf_point_construct_volt(g, &board_obj_ptr,
			sizeof(struct clk_vf_point_volt), pargs);
		break;

	default:
		return NULL;
	}

	if (status) {
		return NULL;
	}

	nvgpu_log_info(g, " Done");

	return (struct clk_vf_point *)board_obj_ptr;
}

static int _clk_vf_point_pmudatainit_super(struct gk20a *g,
					   struct boardobj *board_obj_ptr,
					   struct nv_pmu_boardobj *ppmudata)
{
	int status = 0;
	struct clk_vf_point *pclk_vf_point;
	struct nv_pmu_clk_clk_vf_point_boardobj_set *pset;

	nvgpu_log_info(g, " ");

	status = boardobj_pmudatainit_super(g, board_obj_ptr, ppmudata);
	if (status != 0) {
		return status;
	}

	pclk_vf_point =
		(struct clk_vf_point *)board_obj_ptr;

	pset = (struct nv_pmu_clk_clk_vf_point_boardobj_set *)
		ppmudata;


	pset->vfe_equ_idx = pclk_vf_point->vfe_equ_idx;
	pset->volt_rail_idx = pclk_vf_point->volt_rail_idx;
	return status;
}


static int clk_vf_point_update(struct gk20a *g,
				struct boardobj *board_obj_ptr,
				struct nv_pmu_boardobj *ppmudata)
{
	struct clk_vf_point *pclk_vf_point;
	struct nv_pmu_clk_clk_vf_point_boardobj_get_status *pstatus;

	nvgpu_log_info(g, " ");


	pclk_vf_point =
		(struct clk_vf_point *)board_obj_ptr;

	pstatus = (struct nv_pmu_clk_clk_vf_point_boardobj_get_status *)
		ppmudata;

	if (pstatus->super.type != pclk_vf_point->super.type) {
		nvgpu_err(g,
			"pmu data and boardobj type not matching");
		return -EINVAL;
	}
	/* now copy VF pair */
	memcpy(&pclk_vf_point->pair, &pstatus->pair,
		sizeof(struct ctrl_clk_vf_pair));
	return 0;
}

/*get latest vf point data from PMU */
int clk_vf_point_cache(struct gk20a *g)
{

	struct clk_vf_points *pclk_vf_points;
	struct boardobjgrp *pboardobjgrp;
	struct boardobjgrpmask *pboardobjgrpmask;
	struct nv_pmu_boardobjgrp_super *pboardobjgrppmu;
	struct boardobj *pboardobj = NULL;
	struct nv_pmu_boardobj_query *pboardobjpmustatus = NULL;
	int status;
	u8 index;

	nvgpu_log_info(g, " ");
	pclk_vf_points = &g->clk_pmu.clk_vf_pointobjs;
	pboardobjgrp = &pclk_vf_points->super.super;
	pboardobjgrpmask = &pclk_vf_points->super.mask.super;

	status = pboardobjgrp->pmugetstatus(g, pboardobjgrp, pboardobjgrpmask);
	if (status) {
		nvgpu_err(g, "err getting boardobjs from pmu");
		return status;
	}
	pboardobjgrppmu = pboardobjgrp->pmu.getstatus.buf;

	BOARDOBJGRP_FOR_EACH(pboardobjgrp, struct boardobj*, pboardobj, index) {
		status = pboardobjgrp->pmustatusinstget(g,
				(struct nv_pmu_boardobjgrp *)pboardobjgrppmu,
				&pboardobjpmustatus, index);
		if (status) {
			nvgpu_err(g, "could not get status object instance");
			return status;
		}

		status = clk_vf_point_update(g, pboardobj,
			(struct nv_pmu_boardobj *)pboardobjpmustatus);
		if (status) {
			nvgpu_err(g, "invalid data from pmu at %d", index);
			return status;
		}
	}

	return 0;
}
