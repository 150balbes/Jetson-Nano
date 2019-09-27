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
#ifndef NVGPU_PMUIF_GPMUIFSEQ_H
#define NVGPU_PMUIF_GPMUIFSEQ_H

#include <nvgpu/flcnif_cmn.h>

#define PMU_UNIT_SEQ            (0x02)

/*!
* @file   gpmuifseq.h
* @brief  PMU Command/Message Interfaces - Sequencer
*/

/*!
* Defines the identifiers various high-level types of sequencer commands.
*
* _RUN_SCRIPT @ref NV_PMU_SEQ_CMD_RUN_SCRIPT
*/
enum {
	NV_PMU_SEQ_CMD_ID_RUN_SCRIPT = 0,
};

struct nv_pmu_seq_cmd_run_script {
	u8 cmd_type;
	u8 pad[3];
	struct pmu_allocation_v3 script_alloc;
	struct pmu_allocation_v3 reg_alloc;
};

#define NV_PMU_SEQ_CMD_ALLOC_OFFSET              4

#define NV_PMU_SEQ_MSG_ALLOC_OFFSET                                         \
	(NV_PMU_SEQ_CMD_ALLOC_OFFSET + NV_PMU_CMD_ALLOC_SIZE)

struct nv_pmu_seq_cmd {
	struct pmu_hdr hdr;
	union {
		u8 cmd_type;
		struct nv_pmu_seq_cmd_run_script run_script;
	};
};

enum {
	NV_PMU_SEQ_MSG_ID_RUN_SCRIPT = 0,
};

struct nv_pmu_seq_msg_run_script {
	u8 msg_type;
	u8 error_code;
	u16 error_pc;
	u32 timeout_stat;
};

struct nv_pmu_seq_msg {
	struct pmu_hdr hdr;
	union {
		u8 msg_type;
		struct nv_pmu_seq_msg_run_script run_script;
	};
};

#endif /* NVGPU_PMUIF_GPMUIFSEQ_H */
