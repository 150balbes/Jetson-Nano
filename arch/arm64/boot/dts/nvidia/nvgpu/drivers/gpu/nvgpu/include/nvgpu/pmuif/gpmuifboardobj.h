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
#ifndef NVGPU_PMUIF_GPMUIFBOARDOBJ_H
#define NVGPU_PMUIF_GPMUIFBOARDOBJ_H

#include <nvgpu/flcnif_cmn.h>
#include "ctrl/ctrlboardobj.h"

/* board object group command id's. */
#define NV_PMU_BOARDOBJGRP_CMD_SET			0x00U
#define NV_PMU_BOARDOBJGRP_CMD_GET_STATUS		0x01U

#define NV_PMU_RPC_ID_CLK_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_FAN_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_PERF_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_PERF_CF_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_PMGR_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_THERM_BOARD_OBJ_GRP_CMD		0x00U
#define NV_PMU_RPC_ID_VOLT_BOARD_OBJ_GRP_CMD		0x00U

/*
 * Base structure describing a BOARDOBJ for communication between Kernel and
 * PMU.
 */
struct nv_pmu_boardobj {
	u8 type;
	u8 grp_idx;
};

/*
 * Base structure describing a BOARDOBJ for Query interface between Kernel and
 * PMU.
 */
struct nv_pmu_boardobj_query {
	u8 type;
	u8 grp_idx;
};

/*
 * Virtual base structure describing a BOARDOBJGRP interface between Kernel and
 * PMU.
 */
struct nv_pmu_boardobjgrp_super {
	u8 type;
	u8 class_id;
	u8 obj_slots;
	u8 flags;
};

struct nv_pmu_boardobjgrp {
	struct nv_pmu_boardobjgrp_super super;
	u32 obj_mask;
};

struct nv_pmu_boardobjgrp_e32 {
	struct nv_pmu_boardobjgrp_super super;
	struct ctrl_boardobjgrp_mask_e32 obj_mask;
};

struct nv_pmu_boardobjgrp_e255 {
	struct nv_pmu_boardobjgrp_super super;
	struct ctrl_boardobjgrp_mask_e255 obj_mask;
};

struct nv_pmu_boardobj_cmd_grp_payload {
	struct pmu_allocation_v3 dmem_buf;
	struct flcn_mem_desc_v0 fb;
	u8 hdr_size;
	u8 entry_size;
};

struct nv_pmu_boardobj_cmd_grp {
	u8 cmd_type;
	u8 pad[2];
	u8 class_id;
	struct nv_pmu_boardobj_cmd_grp_payload grp;
};

#define NV_PMU_BOARDOBJ_GRP_ALLOC_OFFSET                                       \
	(NV_OFFSETOF(NV_PMU_BOARDOBJ_CMD_GRP, grp))

struct nv_pmu_boardobj_cmd {
	union {
		u8 cmd_type;
		struct nv_pmu_boardobj_cmd_grp grp;
		struct nv_pmu_boardobj_cmd_grp grp_set;
		struct nv_pmu_boardobj_cmd_grp grp_get_status;
	};
};

struct nv_pmu_boardobj_msg_grp {
	u8 msg_type;
	bool b_success;
	flcn_status flcn_status;
	u8 class_id;
};

struct nv_pmu_boardobj_msg {
	union {
		u8 msg_type;
		struct nv_pmu_boardobj_msg_grp grp;
		struct nv_pmu_boardobj_msg_grp grp_set;
		struct nv_pmu_boardobj_msg_grp grp_get_status;
	};
};

/*
* Macro generating structures describing classes which implement
* NV_PMU_BOARDOBJGRP via the NV_PMU_BOARDBOBJ_CMD_GRP SET interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
* @param   _slots  Max number of elements this group can contain.
*/
#define NV_PMU_BOARDOBJ_GRP_SET_MAKE(_eng, _class, _slots)                     \
	NV_PMU_MAKE_ALIGNED_STRUCT(                                            \
	nv_pmu_##_eng##_##_class##_boardobjgrp_set_header, one_structure);     \
	NV_PMU_MAKE_ALIGNED_UNION(                                             \
	nv_pmu_##_eng##_##_class##_boardobj_set_union, one_union);             \
	struct nv_pmu_##_eng##_##_class##_boardobj_grp_set {                   \
	union nv_pmu_##_eng##_##_class##_boardobjgrp_set_header_aligned  hdr;        \
	union nv_pmu_##_eng##_##_class##_boardobj_set_union_aligned objects[(_slots)];\
	}

/*
* Macro generating structures describing classes which implement
* NV_PMU_BOARDOBJGRP_E32 via the NV_PMU_BOARDBOBJ_CMD_GRP SET interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
*/
#define NV_PMU_BOARDOBJ_GRP_SET_MAKE_E32(_eng, _class)                         \
	NV_PMU_BOARDOBJ_GRP_SET_MAKE(_eng, _class,                             \
	CTRL_BOARDOBJGRP_E32_MAX_OBJECTS)

/*
* Macro generating structures describing classes which implement
* NV_PMU_BOARDOBJGRP_E255 via the NV_PMU_BOARDBOBJ_CMD_GRP SET interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
*/
#define NV_PMU_BOARDOBJ_GRP_SET_MAKE_E255(_eng, _class)                        \
	NV_PMU_BOARDOBJ_GRP_SET_MAKE(_eng, _class,                             \
	CTRL_BOARDOBJGRP_E255_MAX_OBJECTS)

/*
* Macro generating structures for querying dynamic state for classes which
* implement NV_PMU_BOARDOBJGRP via the NV_PMU_BOARDOBJ_CMD_GRP GET_STATUS
* interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
* @param   _slots  Max number of elements this group can contain.
*/
#define NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE(_eng, _class, _slots)              \
	NV_PMU_MAKE_ALIGNED_STRUCT(                                            \
	nv_pmu_##_eng##_##_class##_boardobjgrp_get_status_header, struct);     \
	NV_PMU_MAKE_ALIGNED_UNION(                                             \
	nv_pmu_##_eng##_##_class##_boardobj_get_status_union, union);          \
	struct  nv_pmu_##_eng##_##_class##_boardobj_grp_get_status {           \
	union nv_pmu_##_eng##_##_class##_boardobjgrp_get_status_header_aligned \
	hdr;                                                                   \
	union nv_pmu_##_eng##_##_class##_boardobj_get_status_union_aligned     \
	objects[(_slots)];                                                     \
	}

/*
* Macro generating structures for querying dynamic state for classes which
* implement NV_PMU_BOARDOBJGRP_E32  via the NV_PMU_BOARDOBJ_CMD_GRP GET_STATUS
* interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
*/
#define NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E32(_eng, _class)                  \
	NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE(_eng, _class,                      \
	CTRL_BOARDOBJGRP_E32_MAX_OBJECTS)

/*
* Macro generating structures for querying dynamic state for classes which
* implement NV_PMU_BOARDOBJGRP_E255 via the NV_PMU_BOARDOBJ_CMD_GRP GET_STATUS
* interface.
*
* @para    _eng    Name of implementing engine in which this structure is
* found.
* @param   _class  Class ID of Objects within Board Object Group.
*/
#define NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE_E255(_eng, _class)                 \
	NV_PMU_BOARDOBJ_GRP_GET_STATUS_MAKE(_eng, _class,                      \
	CTRL_BOARDOBJGRP_E255_MAX_OBJECTS)

/* RPC */

/*
 * structure that holds data used to
 * execute BOARD_OBJ_GRP_CMD RPC.
 */
struct nv_pmu_rpc_struct_board_obj_grp_cmd
{
    /* [IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
    /* [IN] BOARDOBJGRP class IDs. */
    u8  class_id;
    /* [IN] Requested command ID (@ref NV_PMU_BOARDOBJGRP_CMD_***)*/
    u8  command_id;
    u32  scratch[1];
};

#endif /*  NVGPU_PMUIF_GPMUIFBOARDOBJ_H */
