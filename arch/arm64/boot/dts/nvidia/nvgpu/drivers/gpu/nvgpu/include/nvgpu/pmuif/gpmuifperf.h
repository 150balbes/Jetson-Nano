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
#ifndef NVGPU_PMUIF_GPMUIFPERF_H
#define NVGPU_PMUIF_GPMUIFPERF_H

#include "gpmuifvolt.h"
#include "gpmuifperfvfe.h"

/*
* Enumeration of BOARDOBJGRP class IDs within OBJPERF.  Used as "classId"
* argument for communications between Kernel and PMU via the various generic
* BOARDOBJGRP interfaces.
*/
#define NV_PMU_PERF_BOARDOBJGRP_CLASS_ID_VFE_VAR                 0x00U
#define NV_PMU_PERF_BOARDOBJGRP_CLASS_ID_VFE_EQU                 0x01U

#define NV_PMU_PERF_CMD_ID_RPC                                   (0x00000002U)
#define NV_PMU_PERF_CMD_ID_BOARDOBJ_GRP_SET                      (0x00000003U)
#define NV_PMU_PERF_CMD_ID_BOARDOBJ_GRP_GET_STATUS               (0x00000004U)

/*!
 * RPC calls serviced by PERF unit.
 */
#define NV_PMU_RPC_ID_PERF_BOARD_OBJ_GRP_CMD                     0x00U
#define NV_PMU_RPC_ID_PERF_LOAD                                  0x01U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_INFO_GET                   0x02U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_INFO_SET                   0x03U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_SET_CONTROL                0x04U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_QUEUE_CHANGE               0x05U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_LOCK                       0x06U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_LOAD                       0x07U
#define NV_PMU_RPC_ID_PERF_CHANGE_SEQ_QUERY                      0x08U
#define NV_PMU_RPC_ID_PERF_PERF_LIMITS_INVALIDATE                0x09U
#define NV_PMU_RPC_ID_PERF_VFE_EQU_EVAL                          0x0AU
#define NV_PMU_RPC_ID_PERF_VFE_INVALIDATE                        0x0BU
#define NV_PMU_RPC_ID_PERF_VFE_EQU_MONITOR_SET                   0x0CU
#define NV_PMU_RPC_ID_PERF_VFE_EQU_MONITOR_GET                   0x0DU
#define NV_PMU_RPC_ID_PERF__COUNT                                0x0EU
/*
 * Defines the structure that holds data
 * used to execute LOAD RPC.
 */
struct nv_pmu_rpc_struct_perf_load {
	/*[IN/OUT] Must be first field in RPC structure */
    struct nv_pmu_rpc_header hdr;
	u32  scratch[1];
};

struct nv_pmu_perf_cmd_set_object {
	u8 cmd_type;
	u8 pad[2];
	u8 object_type;
	struct nv_pmu_allocation object;
};

#define NV_PMU_PERF_SET_OBJECT_ALLOC_OFFSET                            \
	(offsetof(struct nv_pmu_perf_cmd_set_object, object))

/* RPC IDs */
#define NV_PMU_PERF_RPC_ID_VFE_LOAD                              (0x00000001U)

/*!
* Command requesting execution of the perf RPC.
*/
struct nv_pmu_perf_cmd_rpc {
	u8 cmd_type;
	u8 pad[3];
	struct nv_pmu_allocation request;
};

#define NV_PMU_PERF_CMD_RPC_ALLOC_OFFSET       \
	offsetof(struct nv_pmu_perf_cmd_rpc, request)

/*!
* Simply a union of all specific PERF commands. Forms the general packet
* exchanged between the Kernel and PMU when sending and receiving PERF commands
* (respectively).
*/
struct nv_pmu_perf_cmd {
	union {
	u8 cmd_type;
	struct nv_pmu_perf_cmd_set_object set_object;
	struct nv_pmu_boardobj_cmd_grp grp_set;
	struct nv_pmu_boardobj_cmd_grp grp_get_status;
	};
};

/*!
* Defines the data structure used to invoke PMU perf RPCs. Same structure is
* used to return the result of the RPC execution.
*/
struct nv_pmu_perf_rpc {
	u8 function;
	bool b_supported;
	bool b_success;
	flcn_status flcn_status;
	union {
		struct nv_pmu_perf_rpc_vfe_equ_eval vfe_equ_eval;
		struct nv_pmu_perf_rpc_vfe_load vfe_load;
	} params;
};


/* PERF Message-type Definitions */
#define NV_PMU_PERF_MSG_ID_RPC                                   (0x00000003U)
#define NV_PMU_PERF_MSG_ID_BOARDOBJ_GRP_SET                      (0x00000004U)
#define NV_PMU_PERF_MSG_ID_BOARDOBJ_GRP_GET_STATUS               (0x00000006U)
#define NV_PMU_PERF_MSG_ID_VFE_CALLBACK                          (0x00000005U)

/*!
* Message carrying the result of the perf RPC execution.
*/
struct nv_pmu_perf_msg_rpc {
	u8 msg_type;
	u8 rsvd[3];
	struct nv_pmu_allocation response;
};

#define NV_PMU_PERF_MSG_RPC_ALLOC_OFFSET       \
	(offsetof(struct nv_pmu_perf_msg_rpc, response))

/*!
* Simply a union of all specific PERF messages. Forms the general packet
* exchanged between the Kernel and PMU when sending and receiving PERF messages
* (respectively).
*/
struct nv_pmu_perf_msg {
	union {
		u8 msg_type;
		struct nv_pmu_perf_msg_rpc rpc;
		struct nv_pmu_boardobj_msg_grp grp_set;
	};
};

#endif  /* NVGPU_PMUIF_GPMUIFPERF_H*/
