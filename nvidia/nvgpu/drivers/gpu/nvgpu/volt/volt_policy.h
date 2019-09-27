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

#ifndef NVGPU_VOLT_POLICY_H
#define NVGPU_VOLT_POLICY_H

#define VOLT_POLICY_INDEX_IS_VALID(pvolt, policy_idx)	\
		(boardobjgrp_idxisvalid(	\
		&((pvolt)->volt_policy_metadata.volt_policies.super), \
		(policy_idx)))

/*!
 * extends boardobj providing attributes common to all voltage_policies.
 */
struct voltage_policy {
	struct boardobj super;
};

struct voltage_policy_metadata {
	u8 perf_core_vf_seq_policy_idx;
	struct boardobjgrp_e32 volt_policies;
};

/*!
 * extends voltage_policy providing attributes
 * common to all voltage_policy_split_rail.
 */
struct voltage_policy_split_rail {
	struct voltage_policy super;
	u8 rail_idx_master;
	u8 rail_idx_slave;
	u8 delta_min_vfe_equ_idx;
	u8 delta_max_vfe_equ_idx;
	s32 offset_delta_min_uv;
	s32 offset_delta_max_uv;
};

struct voltage_policy_split_rail_single_step {
	struct voltage_policy_split_rail super;
};

struct voltage_policy_split_rail_multi_step {
	struct voltage_policy_split_rail super;
	u16 inter_switch_delay_us;
};

struct voltage_policy_single_rail {
	struct voltage_policy  super;
	u8 rail_idx;
};

struct voltage_policy_single_rail_multi_step {
	struct voltage_policy_single_rail super;
	u16 inter_switch_delay_us;
	u32 ramp_up_step_size_uv;
	u32 ramp_down_step_size_uv;
};

int volt_policy_sw_setup(struct gk20a *g);
int volt_policy_pmu_setup(struct gk20a *g);
#endif /* NVGPU_VOLT_POLICY_H */
