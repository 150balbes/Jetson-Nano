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
#ifndef NVGPU_PMUIF_GPMUIF_CMN_H
#define NVGPU_PMUIF_GPMUIF_CMN_H

/*
 * Defines the logical queue IDs that must be used when submitting
 * commands to the PMU
 */
/* write by sw, read by pmu, protected by sw mutex lock */
#define PMU_COMMAND_QUEUE_HPQ		0U
/* write by sw, read by pmu, protected by sw mutex lock */
#define PMU_COMMAND_QUEUE_LPQ		1U
/* write by pmu, read by sw, accessed by interrupt handler, no lock */
#define PMU_MESSAGE_QUEUE		4U
#define PMU_QUEUE_COUNT			5U

#define PMU_IS_COMMAND_QUEUE(id)	\
		((id)  < PMU_MESSAGE_QUEUE)

#define PMU_IS_SW_COMMAND_QUEUE(id)	\
		(((id) == PMU_COMMAND_QUEUE_HPQ) || \
		 ((id) == PMU_COMMAND_QUEUE_LPQ))

#define  PMU_IS_MESSAGE_QUEUE(id)	\
		((id) == PMU_MESSAGE_QUEUE)

#define OFLAG_READ 	0U
#define OFLAG_WRITE	1U

#define QUEUE_SET		(true)
#define QUEUE_GET		(false)

#define QUEUE_ALIGNMENT		(4U)

/* An enumeration containing all valid logical mutex identifiers */
enum {
	PMU_MUTEX_ID_RSVD1 = 0,
	PMU_MUTEX_ID_GPUSER,
	PMU_MUTEX_ID_QUEUE_BIOS,
	PMU_MUTEX_ID_QUEUE_SMI,
	PMU_MUTEX_ID_GPMUTEX,
	PMU_MUTEX_ID_I2C,
	PMU_MUTEX_ID_RMLOCK,
	PMU_MUTEX_ID_MSGBOX,
	PMU_MUTEX_ID_FIFO,
	PMU_MUTEX_ID_PG,
	PMU_MUTEX_ID_GR,
	PMU_MUTEX_ID_CLK,
	PMU_MUTEX_ID_RSVD6,
	PMU_MUTEX_ID_RSVD7,
	PMU_MUTEX_ID_RSVD8,
	PMU_MUTEX_ID_RSVD9,
	PMU_MUTEX_ID_INVALID
};

#define PMU_MUTEX_ID_IS_VALID(id)	\
		((id) < PMU_MUTEX_ID_INVALID)

#define PMU_INVALID_MUTEX_OWNER_ID	(0)

/*
 * The PMU's frame-buffer interface block has several slots/indices
 * which can be bound to support DMA to various surfaces in memory
 */
enum {
	PMU_DMAIDX_UCODE         = 0,
	PMU_DMAIDX_VIRT          = 1,
	PMU_DMAIDX_PHYS_VID      = 2,
	PMU_DMAIDX_PHYS_SYS_COH  = 3,
	PMU_DMAIDX_PHYS_SYS_NCOH = 4,
	PMU_DMAIDX_RSVD          = 5,
	PMU_DMAIDX_PELPG         = 6,
	PMU_DMAIDX_END           = 7
};

/*
 * Falcon PMU DMA's minimum size in bytes.
 */
#define PMU_DMA_MIN_READ_SIZE_BYTES		16
#define PMU_DMA_MIN_WRITE_SIZE_BYTES	4

#define PMU_FB_COPY_RW_ALIGNMENT	\
	((PMU_DMA_MIN_READ_SIZE_BYTES > PMU_DMA_MIN_WRITE_SIZE_BYTES) ?	\
	PMU_DMA_MIN_READ_SIZE_BYTES : PMU_DMA_MIN_WRITE_SIZE_BYTES)

/*
 * Macros to make aligned versions of RM_PMU_XXX structures. PMU needs aligned
 * data structures to issue DMA read/write operations.
 */
#define NV_PMU_MAKE_ALIGNED_STRUCT(name, size)           \
union name##_aligned {		                         \
		struct name data;                        \
		u8 pad[ALIGN_UP(sizeof(struct name),     \
		(PMU_FB_COPY_RW_ALIGNMENT))];            \
}

#define NV_PMU_MAKE_ALIGNED_UNION(name, size)	         \
union name##_aligned {		                         \
		union name data;                         \
		u8 pad[ALIGN_UP(sizeof(union name),      \
		(PMU_FB_COPY_RW_ALIGNMENT))];            \
}

/* RPC (Remote Procedure Call) header structure */
#define NV_PMU_RPC_FLAGS_TYPE_SYNC 0x00000000

struct nv_pmu_rpc_header {
	/* Identifies the unit servicing requested RPC*/
	u8  unit_id;
	/* Identifies the requested RPC (within the unit)*/
	u8  function;
	/* RPC call flags (@see PMU_RPC_FLAGS) */
	u8  flags;
	/* Falcon's status code to describe failures*/
	u8  flcn_status;
	/* RPC's total exec. time (measured on nvgpu driver side)*/
	u32  exec_time_nv_ns;
	/* RPC's actual exec. time (measured on PMU side)*/
	u32  exec_time_pmu_ns;
};

#endif /* NVGPU_PMUIF_GPMUIF_CMN_H*/
