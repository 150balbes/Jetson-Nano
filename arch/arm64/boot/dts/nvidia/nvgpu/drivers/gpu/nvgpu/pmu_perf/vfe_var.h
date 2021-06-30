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

#ifndef NVGPU_PERF_VFE_VAR_H
#define NVGPU_PERF_VFE_VAR_H

#include "boardobj/boardobjgrp.h"
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

int vfe_var_sw_setup(struct gk20a *g);
int vfe_var_pmu_setup(struct gk20a *g);

#define VFE_VAR_GET(_pperf, _idx)                                              \
	((struct vfe_var)BOARDOBJGRP_OBJ_GET_BY_IDX(                           \
	&((_pperf)->vfe.vars.super.super), (_idx)))

#define VFE_VAR_IDX_IS_VALID(_pperf, _idx)                                     \
	boardobjgrp_idxisvalid(&((_pperf)->vfe.vars.super.super), (_idx))

struct vfe_var {
	struct boardobj super;
	u32 out_range_min;
	u32 out_range_max;
	struct boardobjgrpmask_e32 mask_dependent_vars;
	struct boardobjgrpmask_e255 mask_dependent_equs;
	bool b_is_dynamic_valid;
	bool b_is_dynamic;
};

struct vfe_vars {
	struct boardobjgrp_e32 super;
	u8 polling_periodms;
};

struct vfe_var_derived {
	struct vfe_var super;
};

struct vfe_var_derived_product {
	struct vfe_var_derived super;
	u8 var_idx0;
	u8 var_idx1;
};

struct vfe_var_derived_sum {
	struct vfe_var_derived super;
	u8 var_idx0;
	u8 var_idx1;
};

struct vfe_var_single {
	struct vfe_var super;
	u8 override_type;
	u32 override_value;
};

struct vfe_var_single_frequency {
	struct vfe_var_single  super;
};

struct vfe_var_single_voltage {
	struct vfe_var_single super;
};

struct vfe_var_single_sensed {
	struct vfe_var_single super;
};

struct vfe_var_single_sensed_fuse {
	struct vfe_var_single_sensed super;
	struct ctrl_perf_vfe_var_single_sensed_fuse_override_info	override_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_vfield_info vfield_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_ver_vfield_info vfield_ver_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_value fuse_val_default;
	bool b_fuse_value_signed;
	u32 fuse_value_integer;
	u32 fuse_value_hw_integer;
	u8 fuse_version;
	bool b_version_check_done;
};

struct vfe_var_single_sensed_temp {
	struct vfe_var_single_sensed super;
	u8 therm_channel_index;
	int temp_hysteresis_positive;
	int temp_hysteresis_negative;
	int temp_default;
};

#endif /* NVGPU_PERF_VFE_VAR_H */
