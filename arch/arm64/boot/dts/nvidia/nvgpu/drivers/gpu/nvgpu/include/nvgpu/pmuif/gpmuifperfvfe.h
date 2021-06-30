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
#ifndef NVGPU_PMUIF_GPMUIFPERFVFE_H
#define NVGPU_PMUIF_GPMUIFPERFVFE_H

#include "gpmuifbios.h"
#include "gpmuifboardobj.h"
#include "ctrl/ctrlperf.h"

#define CTRL_PERF_VFE_EQU_QUADRATIC_COEFF_COUNT                      0x03
#define NV_PMU_PERF_RPC_VFE_EQU_EVAL_VAR_COUNT_MAX                             2
#define NV_PMU_PERF_RPC_VFE_EQU_MONITOR_COUNT_MAX                             16

struct nv_pmu_perf_vfe_var_value {
	u8 var_type;
	u8 reserved[3];
	u32 var_value;
};

union nv_pmu_perf_vfe_equ_result {
	u32 freq_m_hz;
	u32 voltu_v;
	u32 vf_gain;
	int volt_deltau_v;
};

struct nv_pmu_perf_rpc_vfe_equ_eval {
	u8 equ_idx;
	u8 var_count;
	u8 output_type;
	struct nv_pmu_perf_vfe_var_value var_values[
		NV_PMU_PERF_RPC_VFE_EQU_EVAL_VAR_COUNT_MAX];
	union nv_pmu_perf_vfe_equ_result result;
};

struct nv_pmu_perf_rpc_vfe_load {
	bool b_load;
};

struct nv_pmu_perf_vfe_var_boardobjgrp_get_status_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_perf_vfe_var_get_status_super {
	struct nv_pmu_boardobj_query board_obj;
};

struct nv_pmu_perf_vfe_var_single_sensed_fuse_get_status {
	struct nv_pmu_perf_vfe_var_get_status_super super;
	struct ctrl_perf_vfe_var_single_sensed_fuse_value fuse_value_integer;
	struct ctrl_perf_vfe_var_single_sensed_fuse_value fuse_value_hw_integer;
	u8 fuse_version;
	bool b_version_check_failed;
};

union nv_pmu_perf_vfe_var_boardobj_get_status_union {
	struct nv_pmu_boardobj_query board_obj;
	struct nv_pmu_perf_vfe_var_get_status_super super;
	struct nv_pmu_perf_vfe_var_single_sensed_fuse_get_status fuse_status;
};

NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E32(perf, vfe_var);

struct nv_pmu_vfe_var {
	struct nv_pmu_boardobj super;
	u32 out_range_min;
	u32 out_range_max;
	struct ctrl_boardobjgrp_mask_e32 mask_dependent_vars;
	struct ctrl_boardobjgrp_mask_e255 mask_dependent_equs;
};

struct nv_pmu_vfe_var_derived {
	struct nv_pmu_vfe_var super;
};

struct nv_pmu_vfe_var_derived_product {
	struct nv_pmu_vfe_var_derived super;
	u8 var_idx0;
	u8 var_idx1;
};

struct nv_pmu_vfe_var_derived_sum {
	struct nv_pmu_vfe_var_derived super;
	u8 var_idx0;
	u8 var_idx1;
};

struct nv_pmu_vfe_var_single {
	struct nv_pmu_vfe_var super;
	u8 override_type;
	u32 override_value;
};

struct nv_pmu_vfe_var_single_frequency {
	struct nv_pmu_vfe_var_single super;
};

struct nv_pmu_vfe_var_single_sensed {
	struct nv_pmu_vfe_var_single super;
};

struct nv_pmu_vfe_var_single_sensed_fuse {
	struct nv_pmu_vfe_var_single_sensed super;
	struct ctrl_perf_vfe_var_single_sensed_fuse_override_info override_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_vfield_info vfield_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_ver_vfield_info vfield_ver_info;
	struct ctrl_perf_vfe_var_single_sensed_fuse_value fuse_val_default;
	bool b_fuse_value_signed;
};

struct nv_pmu_vfe_var_single_sensed_temp {
	struct nv_pmu_vfe_var_single_sensed super;
	u8 therm_channel_index;
	int temp_hysteresis_positive;
	int temp_hysteresis_negative;
	int temp_default;
};

struct nv_pmu_vfe_var_single_voltage {
	struct nv_pmu_vfe_var_single super;
};

struct nv_pmu_perf_vfe_var_boardobjgrp_set_header {
	struct nv_pmu_boardobjgrp_e32 super;
	u8 polling_periodms;
};

union nv_pmu_perf_vfe_var_boardobj_set_union {
	struct nv_pmu_boardobj board_obj;
	struct nv_pmu_vfe_var var;
	struct nv_pmu_vfe_var_derived var_derived;
	struct nv_pmu_vfe_var_derived_product var_derived_product;
	struct nv_pmu_vfe_var_derived_sum var_derived_sum;
	struct nv_pmu_vfe_var_single var_single;
	struct nv_pmu_vfe_var_single_frequency var_single_frequiency;
	struct nv_pmu_vfe_var_single_sensed var_single_sensed;
	struct nv_pmu_vfe_var_single_sensed_fuse var_single_sensed_fuse;
	struct nv_pmu_vfe_var_single_sensed_temp var_single_sensed_temp;
	struct nv_pmu_vfe_var_single_voltage var_single_voltage;
};

NV_PMU_BOARDOBJ_GRP_SET_MAKE_E32(perf, vfe_var);

struct nv_pmu_vfe_equ {
	struct nv_pmu_boardobj super;
	u8 var_idx;
	u8 equ_idx_next;
	u8 output_type;
	u32 out_range_min;
	u32 out_range_max;
};

struct nv_pmu_vfe_equ_compare {
	struct nv_pmu_vfe_equ super;
	u8 func_id;
	u8 equ_idx_true;
	u8 equ_idx_false;
	u32 criteria;
};

struct nv_pmu_vfe_equ_minmax {
	struct nv_pmu_vfe_equ super;
	bool b_max;
	u8 equ_idx0;
	u8 equ_idx1;
};

struct nv_pmu_vfe_equ_quadratic {
	struct nv_pmu_vfe_equ super;
	u32 coeffs[CTRL_PERF_VFE_EQU_QUADRATIC_COEFF_COUNT];
};

struct nv_pmu_perf_vfe_equ_boardobjgrp_set_header {
	struct nv_pmu_boardobjgrp_e255 super;
};

union nv_pmu_perf_vfe_equ_boardobj_set_union {
	struct nv_pmu_boardobj board_obj;
	struct nv_pmu_vfe_equ equ;
	struct nv_pmu_vfe_equ_compare equ_comapre;
	struct nv_pmu_vfe_equ_minmax equ_minmax;
	struct nv_pmu_vfe_equ_quadratic equ_quadratic;
};

NV_PMU_BOARDOBJ_GRP_SET_MAKE_E255(perf, vfe_equ);

#endif  /* NVGPU_PMUIF_GPMUIFPERFVFE_H*/
