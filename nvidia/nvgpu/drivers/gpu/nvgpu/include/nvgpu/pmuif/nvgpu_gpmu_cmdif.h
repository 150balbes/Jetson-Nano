/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_PMUIF_NVGPU_GPMU_CMDIF_H
#define NVGPU_PMUIF_NVGPU_GPMU_CMDIF_H

#include <nvgpu/flcnif_cmn.h>
#include "gpmuif_cmn.h"
#include "gpmuif_pmu.h"
#include "gpmuif_ap.h"
#include "gpmuif_pg.h"
#include "gpmuif_perfmon.h"
#include "gpmuif_acr.h"
#include "gpmuifboardobj.h"
#include "gpmuifclk.h"
#include "gpmuifperf.h"
#include "gpmuifperfvfe.h"
#include "gpmuifpmgr.h"
#include "gpmuifvolt.h"
#include "gpmuiftherm.h"
#include "gpmuifthermsensor.h"
#include "gpmuifseq.h"
#include "gpmu_super_surf_if.h"

/*
 * Command requesting execution of the RPC (Remote Procedure Call)
 */
struct nv_pmu_rpc_cmd {
	/* Must be set to @ref NV_PMU_RPC_CMD_ID */
	u8 cmd_type;
	/* RPC call flags (@see PMU_RPC_FLAGS) */
	u8 flags;
	/* Size of RPC structure allocated
	 *  within NV managed DMEM heap
	 */
	u16 rpc_dmem_size;
	/*
	 * DMEM pointer of RPC structure allocated
	 * within RM managed DMEM heap.
	 */
	u32 rpc_dmem_ptr;
};

#define NV_PMU_RPC_CMD_ID 0x80U

/* Message carrying the result of the RPC execution */
struct nv_pmu_rpc_msg {
	/* Must be set to @ref NV_PMU_RPC_MSG_ID */
	u8 msg_type;
	/* RPC call flags (@see PMU_RPC_FLAGS)*/
	u8 flags;
	/*
	 * Size of RPC structure allocated
	 *  within NV managed DMEM heap.
	 */
	u16 rpc_dmem_size;
	/*
	 * DMEM pointer of RPC structure allocated
	 * within NV managed DMEM heap.
	 */
	u32 rpc_dmem_ptr;
};

#define NV_PMU_RPC_MSG_ID 0x80U

struct pmu_cmd {
	struct pmu_hdr hdr;
	union {
		struct pmu_perfmon_cmd perfmon;
		struct pmu_pg_cmd pg;
		struct pmu_zbc_cmd zbc;
		struct pmu_acr_cmd acr;
		struct nv_pmu_boardobj_cmd boardobj;
		struct nv_pmu_perf_cmd perf;
		struct nv_pmu_volt_cmd volt;
		struct nv_pmu_clk_cmd clk;
		struct nv_pmu_pmgr_cmd pmgr;
		struct nv_pmu_therm_cmd therm;
		struct nv_pmu_rpc_cmd rpc;
	} cmd;
};

struct pmu_msg {
	struct pmu_hdr hdr;
	union {
		struct pmu_init_msg init;
		struct pmu_perfmon_msg perfmon;
		struct pmu_pg_msg pg;
		struct pmu_rc_msg rc;
		struct pmu_acr_msg acr;
		struct nv_pmu_boardobj_msg boardobj;
		struct nv_pmu_perf_msg perf;
		struct nv_pmu_volt_msg volt;
		struct nv_pmu_clk_msg clk;
		struct nv_pmu_pmgr_msg pmgr;
		struct nv_pmu_therm_msg therm;
		struct nv_pmu_rpc_msg rpc;
	} msg;
};

#define PMU_UNIT_REWIND			(0x00U)
#define PMU_UNIT_PG			(0x03U)
#define PMU_UNIT_INIT			(0x07U)
#define PMU_UNIT_ACR			(0x0AU)
#define PMU_UNIT_PERFMON_T18X		(0x11U)
#define PMU_UNIT_PERFMON		(0x12U)
#define PMU_UNIT_PERF			(0x13U)
#define PMU_UNIT_RC			(0x1FU)
#define PMU_UNIT_FECS_MEM_OVERRIDE	(0x1EU)
#define PMU_UNIT_CLK			(0x0DU)
#define PMU_UNIT_THERM			(0x14U)
#define PMU_UNIT_PMGR			(0x18U)
#define PMU_UNIT_VOLT			(0x0EU)

#define PMU_UNIT_END			(0x23U)
#define PMU_UNIT_INVALID		(0xFFU)

#define PMU_UNIT_TEST_START		(0xFEU)
#define PMU_UNIT_END_SIM		(0xFFU)
#define PMU_UNIT_TEST_END		(0xFFU)

#define PMU_UNIT_ID_IS_VALID(id)		\
		(((id) < PMU_UNIT_END) || ((id) >= PMU_UNIT_TEST_START))

#endif /* NVGPU_PMUIF_NVGPU_GPMU_CMDIF_H*/
