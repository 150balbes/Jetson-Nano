/*
 * general perf structures & definitions
 *
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
#ifndef NVGPU_PERF_VFE_EQU_H
#define NVGPU_PERF_VFE_EQU_H

#include "boardobj/boardobjgrp.h"
#include "vfe_var.h"
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

int vfe_equ_sw_setup(struct gk20a *g);
int vfe_equ_pmu_setup(struct gk20a *g);

#define VFE_EQU_GET(_pperf, _idx)                                              \
	((struct vfe_equ *)BOARDOBJGRP_OBJ_GET_BY_IDX(                         \
		&((_pperf)->vfe.equs.super.super), (_idx)))

#define VFE_EQU_IDX_IS_VALID(_pperf, _idx)                                     \
	boardobjgrp_idxisvalid(&((_pperf)->vfe.equs.super.super), (_idx))

#define VFE_EQU_OUTPUT_TYPE_IS_VALID(_pperf, _idx, _outputtype)                \
	(VFE_EQU_IDX_IS_VALID((_pperf), (_idx)) &&                             \
	((_outputtype) != CTRL_PERF_VFE_EQU_OUTPUT_TYPE_UNITLESS) &&           \
	((VFE_EQU_GET((_pperf), (_idx))->outputtype == (_outputtype)) ||       \
	(VFE_EQU_GET((_pperf), (_idx))->outputtype ==                          \
	CTRL_PERF_VFE_EQU_OUTPUT_TYPE_UNITLESS)))

struct vfe_equ {
	struct boardobj super;
	u8 var_idx;
	u8 equ_idx_next;
	u8 output_type;
	u32 out_range_min;
	u32 out_range_max;

	bool b_is_dynamic_valid;
	bool b_is_dynamic;
};

struct vfe_equs {
	struct boardobjgrp_e255 super;
};

struct vfe_equ_compare {
	struct vfe_equ super;
	u8 func_id;
	u8 equ_idx_true;
	u8 equ_idx_false;
	u32 criteria;
};

struct vfe_equ_minmax {
	struct vfe_equ super;
	bool b_max;
	u8 equ_idx0;
	u8 equ_idx1;
};

struct vfe_equ_quadratic {
	struct vfe_equ super;
	u32   coeffs[CTRL_PERF_VFE_EQU_QUADRATIC_COEFF_COUNT];
};

#endif /* NVGPU_PERF_VFE_EQU_H */
