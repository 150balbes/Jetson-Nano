/*
 * general power channel structures & definitions
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
#ifndef NVGPU_PMGR_PWRPOLICY_H
#define NVGPU_PMGR_PWRPOLICY_H

#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobj.h"
#include "ctrl/ctrlpmgr.h"

#define PWR_POLICY_EXT_POWER_STATE_ID_COUNT 0x4U

enum pwr_policy_limit_id {
	PWR_POLICY_LIMIT_ID_MIN    = 0x00000000,
	PWR_POLICY_LIMIT_ID_RATED,
	PWR_POLICY_LIMIT_ID_MAX,
	PWR_POLICY_LIMIT_ID_CURR,
	PWR_POLICY_LIMIT_ID_BATT,
};

struct pwr_policy {
	struct boardobj super;
	u8 ch_idx;
	u8 num_limit_inputs;
	u8 limit_unit;
	s32 limit_delta;
	u32 limit_min;
	u32 limit_rated;
	u32 limit_max;
	u32 limit_batt;
	struct ctrl_pmgr_pwr_policy_info_integral integral;
	struct ctrl_pmgr_pwr_policy_limit_arbitration limit_arb_min;
	struct ctrl_pmgr_pwr_policy_limit_arbitration limit_arb_rated;
	struct ctrl_pmgr_pwr_policy_limit_arbitration limit_arb_max;
	struct ctrl_pmgr_pwr_policy_limit_arbitration limit_arb_batt;
	struct ctrl_pmgr_pwr_policy_limit_arbitration limit_arb_curr;
	u8 sample_mult;
	enum ctrl_pmgr_pwr_policy_filter_type filter_type;
	union ctrl_pmgr_pwr_policy_filter_param filter_param;
};

struct pwr_policy_ext_limit {
	u8 policy_table_idx;
	u32 limit;
};

struct pwr_policy_batt_workitem {
	u32 power_state;
	bool b_full_deflection;
};

struct pwr_policy_client_workitem {
	u32 limit;
	bool b_pending;
};

struct pwr_policy_relationship {
	struct boardobj super;
	u8 policy_idx;
};

struct pmgr_pwr_policy {
	u8 version;
	bool b_enabled;
	struct nv_pmu_perf_domain_group_limits global_ceiling;
	u8 policy_idxs[CTRL_PMGR_PWR_POLICY_IDX_NUM_INDEXES];
	struct pwr_policy_ext_limit ext_limits[PWR_POLICY_EXT_POWER_STATE_ID_COUNT];
	s32 ext_power_state;
	u16 base_sample_period;
	u16 min_client_sample_period;
	u8 low_sampling_mult;
	struct boardobjgrp_e32 pwr_policies;
	struct boardobjgrp_e32 pwr_policy_rels;
	struct boardobjgrp_e32 pwr_violations;
	struct pwr_policy_client_workitem client_work_item;
};

struct pwr_policy_limit {
	struct pwr_policy super;
};

struct pwr_policy_hw_threshold {
	struct pwr_policy_limit super;
	u8 threshold_idx;
	u8 low_threshold_idx;
	bool b_use_low_threshold;
	u16 low_threshold_value;
};

struct pwr_policy_sw_threshold {
	struct pwr_policy_limit super;
	u8 threshold_idx;
	u8 low_threshold_idx;
	bool b_use_low_threshold;
	u16 low_threshold_value;
	u8 event_id;
};

union pwr_policy_data_union {
	struct boardobj boardobj;
	struct pwr_policy pwrpolicy;
	struct pwr_policy_hw_threshold hw_threshold;
	struct pwr_policy_sw_threshold sw_threshold;
} ;

#define PMGR_GET_PWR_POLICY(g, policy_idx)                                 \
	((struct pwr_policy *)BOARDOBJGRP_OBJ_GET_BY_IDX(                                 \
		&(g->pmgr_pmu.pmgr_policyobjs.pwr_policies.super), (policy_idx)))

#define PMGR_PWR_POLICY_INCREMENT_LIMIT_INPUT_COUNT(ppolicy)                 \
	((ppolicy)->num_limit_inputs++)

int pmgr_policy_sw_setup(struct gk20a *g);

#endif /* NVGPU_PMGR_PWRPOLICY_H */
