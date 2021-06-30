/*
 * Control pmgr state infrastructure
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
#ifndef NVGPU_CTRLPMGR_H
#define NVGPU_CTRLPMGR_H

#include "ctrlboardobj.h"

/* valid power domain values */
#define CTRL_PMGR_PWR_DEVICES_MAX_DEVICES			32U
#define CTRL_PMGR_PWR_VIOLATION_MAX				0x06U

#define CTRL_PMGR_PWR_DEVICE_TYPE_INA3221			0x4EU

#define CTRL_PMGR_PWR_CHANNEL_INDEX_INVALID			0xFFU
#define CTRL_PMGR_PWR_CHANNEL_TYPE_SENSOR			0x08U

#define CTRL_PMGR_PWR_POLICY_TABLE_VERSION_3X			0x30U
#define CTRL_PMGR_PWR_POLICY_TYPE_HW_THRESHOLD			0x04U
#define CTRL_PMGR_PWR_POLICY_TYPE_SW_THRESHOLD			0x0CU

#define CTRL_PMGR_PWR_POLICY_MAX_LIMIT_INPUTS			0x8U
#define CTRL_PMGR_PWR_POLICY_IDX_NUM_INDEXES			0x08U
#define CTRL_PMGR_PWR_POLICY_INDEX_INVALID			0xFFU
#define CTRL_PMGR_PWR_POLICY_LIMIT_INPUT_CLIENT_IDX_RM		0xFEU
#define CTRL_PMGR_PWR_POLICY_LIMIT_MAX				(0xFFFFFFFFU)

struct ctrl_pmgr_pwr_device_info_rshunt {
	bool use_fxp8_8;
	u16 rshunt_value;
};

struct ctrl_pmgr_pwr_policy_info_integral {
	u8 past_sample_count;
	u8 next_sample_count;
	u16 ratio_limit_min;
	u16 ratio_limit_max;
};

enum ctrl_pmgr_pwr_policy_filter_type {
	CTRL_PMGR_PWR_POLICY_FILTER_TYPE_NONE = 0,
	CTRL_PMGR_PWR_POLICY_FILTER_TYPE_BLOCK,
	CTRL_PMGR_PWR_POLICY_FILTER_TYPE_MOVING_AVERAGE,
	CTRL_PMGR_PWR_POLICY_FILTER_TYPE_IIR
};

struct ctrl_pmgr_pwr_policy_filter_param_block {
	u32 block_size;
};

struct ctrl_pmgr_pwr_policy_filter_param_moving_average {
	u32 window_size;
};

struct ctrl_pmgr_pwr_policy_filter_param_iir {
	u32 divisor;
};

union ctrl_pmgr_pwr_policy_filter_param {
	struct ctrl_pmgr_pwr_policy_filter_param_block block;
	struct ctrl_pmgr_pwr_policy_filter_param_moving_average moving_avg;
	struct ctrl_pmgr_pwr_policy_filter_param_iir iir;
};

struct ctrl_pmgr_pwr_policy_limit_input {
	u8 pwr_policy_idx;
	u32 limit_value;
};

struct ctrl_pmgr_pwr_policy_limit_arbitration {
	bool b_arb_max;
	u8 num_inputs;
	u32 output;
	struct ctrl_pmgr_pwr_policy_limit_input
		inputs[CTRL_PMGR_PWR_POLICY_MAX_LIMIT_INPUTS];
};

#endif /* NVGPU_CTRLPMGR_H */
