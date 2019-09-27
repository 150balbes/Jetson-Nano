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
#ifndef NVGPU_PMUIF_GPMUIFVOLT_H
#define NVGPU_PMUIF_GPMUIFVOLT_H

#include "gpmuifboardobj.h"
#include <nvgpu/flcnif_cmn.h>
#include "ctrl/ctrlvolt.h"

#define NV_PMU_VOLT_VALUE_0V_IN_UV	(0U)

/* ------------- VOLT_RAIL's GRP_SET defines and structures ------------- */

#define NV_PMU_VOLT_BOARDOBJGRP_CLASS_ID_VOLT_RAIL		0x00U
#define NV_PMU_VOLT_BOARDOBJGRP_CLASS_ID_VOLT_DEVICE		0x01U
#define NV_PMU_VOLT_BOARDOBJGRP_CLASS_ID_VOLT_POLICY		0x02U


struct nv_pmu_volt_volt_rail_boardobjgrp_set_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_volt_volt_rail_boardobj_set {

	struct nv_pmu_boardobj super;
	u8 rel_limit_vfe_equ_idx;
	u8 alt_rel_limit_vfe_equ_idx;
	u8 ov_limit_vfe_equ_idx;
	u8 vmin_limit_vfe_equ_idx;
	u8 volt_margin_limit_vfe_equ_idx;
	u8 pwr_equ_idx;
	u8 volt_dev_idx_default;
	u8 volt_dev_idx_ipc_vmin;
	u8 volt_scale_exp_pwr_equ_idx;
	struct ctrl_boardobjgrp_mask_e32 volt_dev_mask;
	s32 volt_delta_uv[CTRL_VOLT_RAIL_VOLT_DELTA_MAX_ENTRIES];
};

union nv_pmu_volt_volt_rail_boardobj_set_union {
	struct nv_pmu_boardobj board_obj;
	struct nv_pmu_volt_volt_rail_boardobj_set super;
};

NV_PMU_BOARDOBJ_GRP_SET_MAKE_E32(volt, volt_rail);

/* ------------ VOLT_DEVICE's GRP_SET defines and structures ------------ */

struct nv_pmu_volt_volt_device_boardobjgrp_set_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_volt_volt_device_boardobj_set {
	struct nv_pmu_boardobj super;
	u32 switch_delay_us;
	u32 voltage_min_uv;
	u32 voltage_max_uv;
	u32 volt_step_uv;
};

struct nv_pmu_volt_volt_device_vid_boardobj_set {
	struct nv_pmu_volt_volt_device_boardobj_set super;
	s32 voltage_base_uv;
	s32 voltage_offset_scale_uv;
	u8 gpio_pin[CTRL_VOLT_VOLT_DEV_VID_VSEL_MAX_ENTRIES];
	u8 vsel_mask;
};

struct nv_pmu_volt_volt_device_pwm_boardobj_set {
	struct nv_pmu_volt_volt_device_boardobj_set super;
	u32 raw_period;
	s32 voltage_base_uv;
	s32 voltage_offset_scale_uv;
	enum nv_pmu_pmgr_pwm_source pwm_source;
};

union nv_pmu_volt_volt_device_boardobj_set_union {
	struct nv_pmu_boardobj board_obj;
	struct nv_pmu_volt_volt_device_boardobj_set super;
	struct nv_pmu_volt_volt_device_vid_boardobj_set vid;
	struct nv_pmu_volt_volt_device_pwm_boardobj_set pwm;
};

NV_PMU_BOARDOBJ_GRP_SET_MAKE_E32(volt, volt_device);

/* ------------ VOLT_POLICY's GRP_SET defines and structures ------------ */
struct nv_pmu_volt_volt_policy_boardobjgrp_set_header {
	struct nv_pmu_boardobjgrp_e32 super;
	u8 perf_core_vf_seq_policy_idx;
};

struct nv_pmu_volt_volt_policy_boardobj_set {
	struct nv_pmu_boardobj super;
};
struct nv_pmu_volt_volt_policy_sr_boardobj_set {
	struct nv_pmu_volt_volt_policy_boardobj_set super;
	u8 rail_idx;
};

struct nv_pmu_volt_volt_policy_sr_multi_step_boardobj_set {
	struct nv_pmu_volt_volt_policy_sr_boardobj_set super;
	u16 inter_switch_delay_us;
	u32 ramp_up_step_size_uv;
	u32 ramp_down_step_size_uv;
};

struct nv_pmu_volt_volt_policy_splt_r_boardobj_set {
	struct nv_pmu_volt_volt_policy_boardobj_set super;
	u8 rail_idx_master;
	u8 rail_idx_slave;
	u8 delta_min_vfe_equ_idx;
	u8 delta_max_vfe_equ_idx;
	s32 offset_delta_min_uv;
	s32 offset_delta_max_uv;
};

struct nv_pmu_volt_volt_policy_srms_boardobj_set {
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_set super;
	u16 inter_switch_delayus;
};

/* sr - > single_rail */
struct nv_pmu_volt_volt_policy_srss_boardobj_set {
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_set super;
};

union nv_pmu_volt_volt_policy_boardobj_set_union {
	struct nv_pmu_boardobj board_obj;
	struct nv_pmu_volt_volt_policy_boardobj_set super;
	struct nv_pmu_volt_volt_policy_sr_boardobj_set single_rail;
	struct nv_pmu_volt_volt_policy_sr_multi_step_boardobj_set
		single_rail_ms;
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_set split_rail;
	struct nv_pmu_volt_volt_policy_srms_boardobj_set
			split_rail_m_s;
	struct nv_pmu_volt_volt_policy_srss_boardobj_set
			split_rail_s_s;
};

NV_PMU_BOARDOBJ_GRP_SET_MAKE_E32(volt, volt_policy);

/* ----------- VOLT_RAIL's GRP_GET_STATUS defines and structures ----------- */
struct nv_pmu_volt_volt_rail_boardobjgrp_get_status_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_volt_volt_rail_boardobj_get_status {
	struct nv_pmu_boardobj_query super;
	u32 curr_volt_defaultu_v;
	u32 rel_limitu_v;
	u32 alt_rel_limitu_v;
	u32 ov_limitu_v;
	u32 max_limitu_v;
	u32 vmin_limitu_v;
	s32 volt_margin_limitu_v;
	u32 rsvd;
};

union nv_pmu_volt_volt_rail_boardobj_get_status_union {
	struct nv_pmu_boardobj_query board_obj;
	struct nv_pmu_volt_volt_rail_boardobj_get_status super;
};

NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E32(volt, volt_rail);

/* ---------- VOLT_DEVICE's GRP_GET_STATUS defines and structures ---------- */
struct nv_pmu_volt_volt_device_boardobjgrp_get_status_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_volt_volt_device_boardobj_get_status {
	struct nv_pmu_boardobj_query super;
};

union nv_pmu_volt_volt_device_boardobj_get_status_union {
	struct nv_pmu_boardobj_query board_obj;
	struct nv_pmu_volt_volt_device_boardobj_get_status super;
};

NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E32(volt, volt_device);

/* ---------- VOLT_POLICY's GRP_GET_STATUS defines and structures ---------- */
struct nv_pmu_volt_volt_policy_boardobjgrp_get_status_header {
	struct nv_pmu_boardobjgrp_e32 super;
};

struct nv_pmu_volt_volt_policy_boardobj_get_status {
	struct nv_pmu_boardobj_query super;
	u32 offset_volt_requ_v;
	u32 offset_volt_curru_v;
};

struct nv_pmu_volt_volt_policy_sr_boardobj_get_status {
	struct nv_pmu_volt_volt_policy_boardobj_get_status super;
	u32 curr_voltu_v;
};

struct nv_pmu_volt_volt_policy_splt_r_boardobj_get_status {
	struct nv_pmu_volt_volt_policy_boardobj_get_status super;
	s32 delta_minu_v;
	s32 delta_maxu_v;
	s32 orig_delta_minu_v;
	s32 orig_delta_maxu_v;
	u32 curr_volt_masteru_v;
	u32 curr_volt_slaveu_v;
	bool b_violation;
};

/* srms -> split_rail_multi_step */
struct nv_pmu_volt_volt_policy_srms_boardobj_get_status {
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_get_status super;
};

/* srss -> split_rail_single_step */
struct nv_pmu_volt_volt_policy_srss_boardobj_get_status {
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_get_status super;
};

union nv_pmu_volt_volt_policy_boardobj_get_status_union {
	struct nv_pmu_boardobj_query board_obj;
	struct nv_pmu_volt_volt_policy_boardobj_get_status super;
	struct nv_pmu_volt_volt_policy_sr_boardobj_get_status single_rail;
	struct nv_pmu_volt_volt_policy_splt_r_boardobj_get_status split_rail;
	struct nv_pmu_volt_volt_policy_srms_boardobj_get_status
			split_rail_m_s;
	struct nv_pmu_volt_volt_policy_srss_boardobj_get_status
			split_rail_s_s;
};

NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E32(volt, volt_policy);

struct nv_pmu_volt_policy_voltage_data {
	u8 policy_idx;
	struct ctrl_perf_volt_rail_list
	rail_list;
};

struct nv_pmu_volt_rail_get_voltage {
	u8 rail_idx;
	u32 voltage_uv;
};

struct nv_pmu_volt_volt_rail_set_noise_unaware_vmin {
	u8 num_rails;
	struct ctrl_volt_volt_rail_list
	rail_list;
};

#define NV_PMU_VOLT_CMD_ID_BOARDOBJ_GRP_SET			(0x00000000U)
#define NV_PMU_VOLT_CMD_ID_RPC					(0x00000001U)
#define NV_PMU_VOLT_CMD_ID_BOARDOBJ_GRP_GET_STATUS		(0x00000002U)
#define NV_PMU_VOLT_RPC_ID_VOLT_RAIL_SET_NOISE_UNAWARE_VMIN	(0x00000004U)

/*!
* PMU VOLT RPC calls.
*/
#define NV_PMU_VOLT_RPC_ID_LOAD					(0x00000000U)
#define NV_PMU_VOLT_RPC_ID_VOLT_POLICY_SET_VOLTAGE		(0x00000002U)
#define NV_PMU_VOLT_RPC_ID_VOLT_RAIL_GET_VOLTAGE		(0x00000003U)

struct nv_pmu_volt_cmd_rpc {
	u8 cmd_type;
	u8 pad[3];
	struct nv_pmu_allocation request;
};

#define NV_PMU_VOLT_CMD_RPC_ALLOC_OFFSET	\
	offsetof(struct nv_pmu_volt_cmd_rpc, request)

struct nv_pmu_volt_cmd {
	union {
		u8 cmd_type;
		struct nv_pmu_boardobj_cmd_grp grp_set;
		struct nv_pmu_volt_cmd_rpc rpc;
		struct nv_pmu_boardobj_cmd_grp grp_get_status;
	};
};

struct nv_pmu_volt_rpc {
	u8 function;
	bool b_supported;
	bool b_success;
	flcn_status flcn_status;
	union {
		struct nv_pmu_volt_policy_voltage_data volt_policy_voltage_data;
		struct nv_pmu_volt_rail_get_voltage volt_rail_get_voltage;
		struct nv_pmu_volt_volt_rail_set_noise_unaware_vmin
			volt_rail_set_noise_unaware_vmin;
	} params;
};

/*!
* VOLT MSG ID definitions
*/
#define NV_PMU_VOLT_MSG_ID_BOARDOBJ_GRP_SET			(0x00000000U)
#define NV_PMU_VOLT_MSG_ID_RPC					(0x00000001U)
#define NV_PMU_VOLT_MSG_ID_BOARDOBJ_GRP_GET_STATUS		(0x00000002U)

/*!
* Message carrying the result of the VOLT RPC execution.
*/
struct nv_pmu_volt_msg_rpc {
	u8 msg_type;
	u8 rsvd[3];
	struct nv_pmu_allocation response;
};

#define NV_PMU_VOLT_MSG_RPC_ALLOC_OFFSET	\
		offsetof(struct nv_pmu_volt_msg_rpc, response)

struct nv_pmu_volt_msg {
	union {
		u8 msg_type;
		struct nv_pmu_boardobj_msg_grp grp_set;
		struct nv_pmu_volt_msg_rpc rpc;
		struct nv_pmu_boardobj_msg_grp grp_get_status;
	};
};

#define NV_PMU_VF_INJECT_MAX_VOLT_RAILS		(2U)

struct nv_pmu_volt_volt_rail_list {
	u8 num_rails;
	struct ctrl_perf_volt_rail_list_item
	rails[NV_PMU_VF_INJECT_MAX_VOLT_RAILS];
};

struct nv_pmu_volt_volt_rail_list_v1 {
	u8 num_rails;
	struct ctrl_volt_volt_rail_list_item_v1
	rails[NV_PMU_VF_INJECT_MAX_VOLT_RAILS];
};

/* VOLT RPC */
#define NV_PMU_RPC_ID_VOLT_BOARD_OBJ_GRP_CMD			0x00U
#define NV_PMU_RPC_ID_VOLT_VOLT_SET_VOLTAGE			0x01U
#define NV_PMU_RPC_ID_VOLT_LOAD					0x02U
#define NV_PMU_RPC_ID_VOLT_VOLT_RAIL_GET_VOLTAGE		0x03U
#define NV_PMU_RPC_ID_VOLT_VOLT_POLICY_SANITY_CHECK		0x04U
#define NV_PMU_RPC_ID_VOLT_TEST_EXECUTE				0x05U
#define NV_PMU_RPC_ID_VOLT__COUNT				0x06U

/*
 * Defines the structure that holds data
 * used to execute LOAD RPC.
 */
struct nv_pmu_rpc_struct_volt_load {
	/*[IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
	u32  scratch[1];
};

/*
 * Defines the structure that holds data
 * used to execute VOLT_SET_VOLTAGE RPC.
 */
struct nv_pmu_rpc_struct_volt_volt_set_voltage {
	/*[IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
	/*[IN] ID of the client that wants to set the voltage */
	u8 client_id;
	/*
	 * [IN] The list containing target voltage and
	 *  noise-unaware Vmin value for the VOLT_RAILs.
	 */
	struct ctrl_volt_volt_rail_list_v1 rail_list;
	u32  scratch[1];
};

/*
 * Defines the structure that holds data
 * used to execute VOLT_RAIL_GET_VOLTAGE RPC.
 */
struct nv_pmu_rpc_struct_volt_volt_rail_get_voltage {
	/*[IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
	/* [OUT] Current voltage in uv */
	u32 voltage_uv;
	/* [IN] Voltage Rail Table Index */
	u8 rail_idx;
	u32  scratch[1];
};

#endif  /* NVGPU_PMUIF_GPMUIFVOLT_H*/
