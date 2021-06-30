/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/falcon.h>
#include <nvgpu/pmu.h>
#include <nvgpu/gk20a.h>

#include "gk20a/flcn_gk20a.h"
#include "gp106/flcn_gp106.h"
#include "gv100/flcn_gv100.h"
#include "gv100/gsp_gv100.h"

#include <nvgpu/hw/gv100/hw_falcon_gv100.h>
#include <nvgpu/hw/gv100/hw_pgsp_gv100.h>

static void gv100_falcon_engine_dependency_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_engine_dependency_ops *flcn_eng_dep_ops =
			&flcn->flcn_engine_dep_ops;

	switch (flcn->flcn_id) {
	case FALCON_ID_GSPLITE:
		flcn_eng_dep_ops->reset_eng = gv100_gsp_reset;
	break;
	default:
		flcn_eng_dep_ops->reset_eng = NULL;
		break;
	}
}

static void gv100_falcon_ops(struct nvgpu_falcon *flcn)
{
	gk20a_falcon_ops(flcn);
	gv100_falcon_engine_dependency_ops(flcn);
}

int gv100_falcon_hal_sw_init(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	int err = 0;

	switch (flcn->flcn_id) {
	case FALCON_ID_MINION:
		flcn->flcn_base = g->nvlink.minion_base;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = true;
		break;
	case FALCON_ID_GSPLITE:
		flcn->flcn_base = pgsp_falcon_irqsset_r();
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
		break;
	default:
		flcn->is_falcon_supported = false;
		break;
	}

	if (flcn->is_falcon_supported) {
		err = nvgpu_mutex_init(&flcn->copy_lock);
		if (err != 0) {
			nvgpu_err(g, "Error in flcn.copy_lock mutex initialization");
		} else {
			gv100_falcon_ops(flcn);
		}
	} else {
		/*
		 * Forward call to previous chips HAL
		 * to fetch info for requested
		 * falcon as no changes between
		 * current & previous chips.
		 */
		err = gp106_falcon_hal_sw_init(flcn);
	}

	return err;
}
