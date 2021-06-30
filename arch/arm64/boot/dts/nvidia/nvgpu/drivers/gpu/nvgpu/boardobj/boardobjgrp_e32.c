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

#include "boardobj.h"
#include "boardobjgrp.h"
#include "boardobjgrp_e32.h"
#include "ctrl/ctrlboardobj.h"
#include "boardobjgrpmask.h"


int  boardobjgrpconstruct_e32(struct gk20a *g,
			      struct boardobjgrp_e32 *pboardobjgrp_e32)
{
	int status;
	u8  objslots;

	nvgpu_log_info(g, " ");
	objslots = 32;

	status = boardobjgrpmask_e32_init(&pboardobjgrp_e32->mask, NULL);
	if (status) {
		goto boardobjgrpconstruct_e32_exit;
	}

	pboardobjgrp_e32->super.type      = CTRL_BOARDOBJGRP_TYPE_E32;
	pboardobjgrp_e32->super.ppobjects = pboardobjgrp_e32->objects;
	pboardobjgrp_e32->super.objslots  = objslots;
	pboardobjgrp_e32->super.mask     = &(pboardobjgrp_e32->mask.super);

	status = boardobjgrp_construct_super(g, &pboardobjgrp_e32->super);
	if (status) {
		goto boardobjgrpconstruct_e32_exit;
	}

	pboardobjgrp_e32->super.pmuhdrdatainit = boardobjgrp_pmuhdrdatainit_e32;

boardobjgrpconstruct_e32_exit:
	return status;
}

int boardobjgrp_pmuhdrdatainit_e32(struct gk20a *g,
		struct boardobjgrp *pboardobjgrp,
		struct nv_pmu_boardobjgrp_super *pboardobjgrppmu,
		struct boardobjgrpmask *mask)
{
	struct nv_pmu_boardobjgrp_e32 *pgrpe32 =
		(struct nv_pmu_boardobjgrp_e32 *)pboardobjgrppmu;
	int status;

	nvgpu_log_info(g, " ");

	if (pboardobjgrp == NULL) {
		return -EINVAL;
	}

	if (pboardobjgrppmu == NULL) {
		return -EINVAL;
	}
	status = boardobjgrpmask_export(mask,
				mask->bitcount,
				&pgrpe32->obj_mask.super);
	if (status) {
		nvgpu_err(g, "e32 init:failed export grpmask");
		return status;
	}

	return boardobjgrp_pmuhdrdatainit_super(g,
			pboardobjgrp, pboardobjgrppmu, mask);
}
