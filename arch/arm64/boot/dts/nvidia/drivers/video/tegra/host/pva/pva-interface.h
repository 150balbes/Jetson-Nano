/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _PVA_INTERFACE_H_
#define _PVA_INTERFACE_H_

#include "pva-bit.h"
#include "pva-errors.h"

/*
 * Register definition for PVA_SHRD_SMP_STA0
 *
 * This is used to communicate various bits of information between the
 * OS and the PVA.
 */

/*
 * Bits set by the OS and examined by the R5
 */
#define PVA_BOOT_INT		PVA_BIT(31)	/* OS wants an interrupt */
#define PVA_OS_PRINT		PVA_BIT(30)	/* OS will process print */
#define PVA_TEST_WAIT		PVA_BIT(29)	/* R5 wait to start tests */
#define PVA_TEST_RUN		PVA_BIT(28)	/* Start tests */
#define PVA_WAIT_DEBUG		PVA_BIT(24)	/* Spin-wait early in boot */
#define PVA_CG_DISABLE		PVA_BIT(20)	/* Disable PVA clock gating */
#define PVA_VMEM_RD_WAR_DISABLE	PVA_BIT(19)	/* Disable VMEM RD fail WAR */
#define PVA_VMEM_MBX_WAR_ENABLE	PVA_BIT(18)	/* WAR for Bug 2090939 enabled*/

/*
 * Bits set by the R5 and examined by the OS
 */
#define PVA_TESTS_STARTED	PVA_BIT(10)	/* PVA Tests started */
#define PVA_TESTS_PASSED	PVA_BIT(9)	/* PVA Tests passed */
#define PVA_TESTS_FAILED	PVA_BIT(8)	/* PVA Tests failed */
#define PVA_HALTED		PVA_BIT(2)	/* PVA uCode halted */
#define PVA_BOOT_DONE		PVA_BIT(1)	/* PVA is "ready" */
#define PVA_TEST_MODE		PVA_BIT(0)	/* PVA is in "test mode" */

/*
 * Symbolic definitions of the mailbox registers (rather than using 0-7)
 */
#define PVA_MBOX_COMMAND		0U
#define PVA_MBOX_ADDR			1U
#define PVA_MBOX_LENGTH			2U
#define PVA_MBOX_ARG			3U
#define PVA_MBOX_SIDE_CHANNEL_HOST_WR	4U
#define PVA_MBOX_AISR			5U
#define PVA_MBOX_SIDE_CHANNEL_HOST_RD	6U
#define PVA_MBOX_ISR			7U

 /*
  * Mailbox side channel bit definitions
  */
#define PVA_SIDE_CHANNEL_MBOX_BIT	0U
#define PVA_SIDE_CHANNEL_MBOX_BIT_MASK	(~(1U << PVA_SIDE_CHANNEL_MBOX_BIT))

/*
 * Code checking the version of the R5 uCode should check
 * the values returned from the R5_VERSION subcommand of
 * CMD_GET_STATUS to determine if the version currently
 * running on the PVA's R5 is compatible with what the
 * driver was compiled against.
 */
#define PVA_R5_VERSION				\
	PVA_MAKE_VERSION(0,			\
			 PVA_VERSION_MAJOR,	\
			 PVA_VERSION_MINOR,	\
			 PVA_VERSION_SUBMINOR)


/*
 * PVA interrupt status register contained in PVA_MBOX_ISR.
 */
#define PVA_INT_PENDING			PVA_BIT(31)
#define PVA_READY			PVA_BIT(30)
#define PVA_BUSY			PVA_BIT(29)
#define PVA_CMD_COMPLETE		PVA_BIT(28)
#define PVA_CMD_ERROR			PVA_BIT(27)
#define PVA_VALID_STATUS7		PVA_BIT(26)
#define PVA_VALID_STATUS6		PVA_BIT(25)
#define PVA_VALID_STATUS5		PVA_BIT(24)
#define PVA_VALID_STATUS4		PVA_BIT(23)
#define PVA_VALID_STATUS3		PVA_BIT(22)

/*
 * PVA interrupt status register contained in PVA_MBOX_AISR
 */
#define PVA_AISR_INT_PENDING		PVA_BIT(31)
#define PVA_AISR_TASK_COMPLETE		PVA_BIT(28)
#define PVA_AISR_TASK_ERROR		PVA_BIT(27)
#define PVA_AISR_THRESHOLD_EXCEEDED	PVA_BIT(26)
#define PVA_AISR_LOGGING_OVERFLOW	PVA_BIT(25)
#define PVA_AISR_PRINTF_OVERFLOW	PVA_BIT(24)
#define PVA_AISR_CRASH_LOG		PVA_BIT(23)
#define PVA_AISR_ABORT			PVA_BIT(0)

#define PVA_GET_ERROR_CODE(_s_)		PVA_EXTRACT(_s_, 15, 0, enum pva_errors)

/*
 * Commands that can be sent to the PVA through the PVA_SHRD_MBOX
 * interface.
 */
enum pva_cmds {
	CMD_NOOP = 0,
	CMD_GET_STATUS = 1,
	CMD_MANAGE_STATISTICS = 2,
	CMD_SET_REGION = 3,
	CMD_SET_LOGGING = 4,
	CMD_PVA_CONTROL = 5,
	CMD_VPU_CONTROL = 6,
	CMD_SET_QUEUE_ATTRIBUTES = 7,
	CMD_SUBMIT = 8,
	CMD_SUSPEND = 9,
	CMD_RESUME = 10,
	CMD_FLUSH = 11,
	CMD_SET_THRESHOLD = 12,
	CMD_CLEAR_THRESHOLD = 13,
	CMD_RESET = 14,
	CMD_GET_VPU_FUNC_TABLE = 15,
	CMD_SET_SYNC_POLLING = 16,
	CMD_SET_SCHEDULER = 17,
	CMD_SET_SCHED_ATTR = 18,
	CMD_NEXT = 19,		/* Must be last */
};

/*
 * CMD_GET_STATUS subcommands
 */
enum pva_status_cmds {
	R5_VERSION = 0,
	VPU_VERSION = 1,
	PVA_HW_CONFIG = 2,
	PVA_SW_CONFIG = 3,
	VPU_UTILIZATION = 4,
	QUEUE_DEPTH = 5,
	QUEUE_WAIT_TIME = 6,
	WAIT_TIMES = 7,
	RUN_TIMES = 8,
	IDLE_TIME = 9,
	RUNNING_TASKS = 10,
	PVA_UPTIME = 11,
	PVA_ERRORS = 12,
	LOGGING_INFO = 13,
	PRINTF_INFO = 14,
	CRASH_INFO = 15,
	THRESHOLD = 16,
	THRESHOLDS_EXCEEDED = 17,
	QUEUE_ATTRIBUTES = 18,
	COMPLETED_TASK = 19,
	VMEM_USAGE = 20,
	SYNC_POLL_RATE = 21,
	SCHEDULER_ATTRIBUTES = 22,
	GET_STATUS_NEXT = 23,
};

/*
 * Statistics IDs
 */
enum pva_stats_ids {
	PVA_VPU = 0,
	PVA_QUEUE = 1,
	PVA_R5 = 2,
	PVA_ALL = 255,
};

/*
 * Region ID's
 */
enum pva_region_ids {
	PVA_REGION_LOGGING = 1,
	PVA_REGION_PRINTF = 2,
	PVA_REGION_CRASH_DUMP = 3,
	PVA_REGION_STATUS = 4,
};

/*
 * Queue Attribute ID's
 */
enum pva_queue_attr_id {
	QUEUE_ATTR_PRIORITY		= 1,
	QUEUE_ATTR_VPU			= 2,
	QUEUE_ATTR_MISR_TO		= 3,
	QUEUE_ATTR_MAX			= 4,
};

#define PVA_QUEUE_DEFAULT_PRIORITY	2U
#define PVA_QUEUE_DEFAULT_VPU_MASK	3U

/*
 * Threshold ID's
 */
enum pva_threshold_id {
	THRESHOLD_IDLE_TIME = 0,
	THRESHOLD_UTILIZATION = 1,
	THRESHOLD_LOW_UTILIZATION = 2,
	THRESHOLD_ANY_QUEUE_DEPTH = 3,
	THRESHOLD_QUEUE_DEPTH = 4,
	THRESHOLD_ANY_WAIT_TIME = 5,
	THRESHOLD_WAIT_TIME = 6,
	THRESHOLD_RUN_TIME = 7,
	THRESHOLD_HW_ERRORS = 8,
	THRESHOLD_R5_CRIT_ERRORS = 9,
	THRESHOLD_R5_ERRORS = 10,
	THRESHOLD_VPU_CRIT_ERRORS = 11,
	THRESHOLD_VPU_ERRORS = 12,
	THRESHOLD_PVA_ERRORS = 13,
	THRESHOLD_LAST = 13,		/* must be updated as new ones added */
};

/*
 * Reset ID's
 */
enum pva_reset_id {
	RESET_VPU0 = 0,
	RESET_VPU1 = 1,
	RESET_DMA0 = 0x10,
	RESET_DMA1 = 0x11,
};

/*
 * Scheduler ID's
 */
enum pva_sched_id {
	PVA_SCHED_PRIORITY_RR = 0U,
	PVA_SCHED_FIXED = 1U,
	PVA_SCHED_DEADLINE = 2U,
	PVA_SCHED_LAST = 2U,		/* must be updated as new ones added */
	PVA_SCHED_CURRENT = 0xFFU,
};

/*
 * Scheduler Attribute ID's
 */
enum pva_sched_attr_id {
	PVA_SCHED_ATTR_CURRENT = 0x00U,
	PVA_SCHED_ACTIVE = 0x01U,
	PVA_SCHED_FIXED_BUFF0 = 0x10U,
	PVA_SCHED_FIXED_BUFF1 = 0x11U,
};

/*
 * Scheduler Attributes
 */
struct pva_sched_attr {
	uint32_t	attr_a;
	uint32_t	attr_b;
};

/*
 * Attributes for PVA_SCHED_ACTIVE
 */

#define PVA_SCHED_ATTR_ACTIVE_ENABLED		PVA_BIT(31)
#define PVA_SCHED_ATTR_ACTIVE_SCHED_ID(_x_)	\
			PVA_EXTRACT(_x_, 7, 0, enum pva_sched_id)

static inline uint32_t
pva_get_sched_attr_active_enabled(struct pva_sched_attr * const attrs)
{
	return attrs->attr_a & PVA_SCHED_ATTR_ACTIVE_ENABLED;
}

static inline enum pva_sched_id
pva_get_sched_attr_active_sched_id(struct pva_sched_attr * const attrs)
{
	return PVA_SCHED_ATTR_ACTIVE_SCHED_ID(attrs->attr_a);
}

/*
 * Attributes for PVA_SCHED_FIXED0 & PVA_SCHED_FIXED1
 */
#define PVA_SCHED_ATTR_FIXED_VALID		PVA_BIT(31)
#define PVA_SCHED_ATTR_FIXED_REPEAT		PVA_BIT(30)
#define PVA_SCHED_ATTR_FIXED_LOADED		PVA_BIT(29)
#define PVA_SCHED_ATTR_FIXED_NEXT		PVA_BIT(28)

static inline uint32_t
pva_get_sched_attr_fixed_valid(struct pva_sched_attr * const attrs)
{
	return attrs->attr_a & PVA_SCHED_ATTR_FIXED_VALID;
}

static inline uint32_t
pva_get_sched_attr_fixed_repeat(struct pva_sched_attr * const attrs)
{
	return attrs->attr_a & PVA_SCHED_ATTR_FIXED_REPEAT;
}

static inline uint32_t
pva_get_sched_attr_fixed_loaded(struct pva_sched_attr * const attrs)
{
	return attrs->attr_a & PVA_SCHED_ATTR_FIXED_LOADED;
}

static inline uint32_t
pva_get_sched_attr_fixed_next(struct pva_sched_attr * const attrs)
{
	return attrs->attr_a & PVA_SCHED_ATTR_FIXED_NEXT;
}

static inline uint32_t
pva_get_sched_attr_fixed_offset(struct pva_sched_attr *const attrs)
{
	return PVA_EXTRACT(attrs->attr_a, 23, 16, uint32_t);
}

static inline uint32_t
pva_get_sched_attr_fixed_size(struct pva_sched_attr * const attrs)
{
	return PVA_EXTRACT(attrs->attr_a, 15, 8, uint32_t);
}

static inline uint64_t
pva_get_sched_attr_fixed_addr(struct pva_sched_attr *const attrs)
{
	return PVA_PACK64(attrs->attr_b,
			  PVA_EXTRACT(attrs->attr_a, 7, 0, uint32_t));
}

static inline void
pva_set_sched_attr_fixed(struct pva_sched_attr * const attrs,
			 const uint32_t flags,
			 const uint32_t size,
			 const uint64_t addr)
{
	attrs->attr_a = (flags & ~(PVA_SCHED_ATTR_FIXED_LOADED
				   | PVA_SCHED_ATTR_FIXED_NEXT))
			| PVA_INSERT(size, 15, 8)
			| PVA_INSERT(PVA_HI32(addr), 7, 0);
	attrs->attr_b = PVA_LOW32(addr);
}

/*
 * Generic fields in a command sent to the PVA through the PVA_SHRD_MBOX
 * interface.
 */
#define PVA_CMD_GO			PVA_BIT(31)
#define PVA_CMD_INT_ON_ERR		PVA_BIT(30)
#define PVA_CMD_INT_ON_COMPLETE		PVA_BIT(29)
#define PVA_GET_SUBCOMMAND(_c_, _t_)	PVA_EXTRACT(_c_, 15, 8, _t_)
#define PVA_SET_SUBCOMMAND(_c_)		PVA_INSERT(_c_, 15, 8)
#define PVA_GET_COMMAND(_c_)		PVA_EXTRACT(_c_, 7, 0, enum pva_cmds)
#define PVA_SET_COMMAND(_c_)		PVA_INSERT(_c_, 7, 0)

/*
 * Generic fields in a command sent through the command FIFO interface.
 */
#define PVA_FIFO_GET_COMMAND(_c_)	PVA_EXTRACT64(_c_, 63, 56, enum pva_cmds)
#define PVA_FIFO_INT_ON_ERR		PVA_BIT64(1)
#define PVA_FIFO_INT_ON_COMPLETE	PVA_BIT64(0)

/*
 * Structure for managing commands through PVA_SHRD_MBOX*
 */
struct pva_cmd {
	uint32_t mbox[4];
};

/*
 * CMD_NOOP command
 */
#define PVA_CMD_FL_NOOP_ECHO		PVA_BIT(28)
#define PVA_CMD_FL_NOOP_ERROR		PVA_BIT(27)

static inline uint32_t
pva_cmd_noop(struct pva_cmd * const cmd,
	     const uint32_t echo_data,
	     const uint32_t status_reg,
	     const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags | PVA_SET_SUBCOMMAND(status_reg)
		       | PVA_SET_COMMAND(CMD_NOOP);
	cmd->mbox[1] = echo_data;

	return 2U;
}

/*
 * CMD_GET_STATUS
 * Not used directly.
 */
static inline uint32_t
pva_cmd_get_status(const enum pva_status_cmds subcommand,
		   const uint32_t flags)
{
	return PVA_CMD_GO | flags | PVA_SET_SUBCOMMAND(subcommand)
	       | PVA_SET_COMMAND(CMD_GET_STATUS);
}

/*
 * R5_VERSION get status command
 */
struct pva_status_R5_version {
	uint32_t	cur_version;
	uint32_t	oldest_version;
	uint32_t	change_id;
	uint32_t	build_date;
};

static inline uint32_t
pva_cmd_R5_version(struct pva_cmd * const cmd,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(R5_VERSION, flags);
	return 1U;
}

/*
 * VPU_VERSION get status command
 */
struct pva_status_VPU_version {
	uint32_t	cur_version;
	uint32_t	oldest_version;
	uint32_t	change_id;
	uint32_t	build_date;
};

static inline uint32_t
pva_cmd_VPU_version(struct pva_cmd * const cmd,
		    const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(VPU_VERSION, flags);
	return 1U;
}

/*
 * PVA_HW_CONFIG get status command
 */
struct pva_status_pva_hw_config {
	uint32_t	version;
	uint32_t	r5_sizes;
	uint32_t	vpu_sizes;
	uint32_t	dma_info;
};

static inline uint32_t
pva_cmd_pva_hw_config(struct pva_cmd * const cmd,
		      const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(PVA_HW_CONFIG, flags);
	return 1U;
}

static inline uint32_t
pva_get_hw_version(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->version, 31, 24, uint32_t);
}

static inline uint32_t
pva_get_vpu_isa_version(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->version, 23, 16, uint32_t);
}

static inline uint32_t
pva_get_num_vpus(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->version, 7, 4, uint32_t);
}

static inline uint32_t
pva_get_num_dma_engines(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->version, 3, 0, uint32_t);
}

static inline uint32_t
pva_get_r5_icache_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->r5_sizes, 31, 24, uint32_t);
}

static inline uint32_t
pva_get_r5_dcache_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->r5_sizes, 23, 16, uint32_t);
}

static inline uint32_t
pva_get_tcma_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->r5_sizes, 15, 8, uint32_t);
}

static inline uint32_t
pva_get_tcmb_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->r5_sizes, 7, 0, uint32_t);
}

static inline uint32_t
pva_get_vpu_icache_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->vpu_sizes, 31, 24, uint32_t);
}

static inline uint32_t
pva_get_vpu_dcache_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->vpu_sizes, 23, 16, uint32_t);
}

static inline uint32_t
pva_get_num_vrams(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->vpu_sizes, 15, 8, uint32_t);
}

static inline uint32_t
pva_get_vram_size(struct pva_status_pva_hw_config * const status)
{
	return PVA_EXTRACT(status->vpu_sizes, 7, 0, uint32_t);
}

/*
 * PVA_SW_CONFIG get status command
 */
struct pva_status_pva_sw_config {
	uint32_t	sw_status;
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
};

static inline uint32_t
pva_cmd_pva_sw_config(struct pva_cmd * const cmd,
		      const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(PVA_SW_CONFIG, flags);
	return 1U;
}

#define PVA_SW_IN_TCM			PVA_BIT(31)
#define PVA_SW_FPU_ENABLED		PVA_BIT(30)
#define PVA_SW_PROCESSING_ENABLED	PVA_BIT(28)
#define PVA_SW_LOGGING_SUPPORTED	PVA_BIT(27)
#define PVA_SW_PRINTF_SUPPORTED		PVA_BIT(26)
#define PVA_SW_CRASHDUMP_SUPPORTED	PVA_BIT(25)
#define PVA_SW_STATISTICS_SUPPORTED	PVA_BIT(24)
#define pva_get_sw_num_queues(_x_)	PVA_EXTRACT(_x_, 23, 16, uint32_t)
#define pva_get_sw_num_vpus(_x_)	PVA_EXTRACT(_x_, 7, 4, uint32_t)
#define pva_get_sw_num_dma_engines(_x_)	PVA_EXTRACT(_x_, 3, 0, uint32_t)

/*
 * VPU_UTILIZATION get status command
 */
struct pva_status_vpu_utilization {
	uint32_t	vpu0_utilization;
	uint32_t	vpu1_utilization;
	uint32_t	reserved1;
	uint32_t	reserved2;
};

static inline uint32_t
pva_cmd_vpu_utilization(struct pva_cmd * const cmd,
			const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(VPU_UTILIZATION, flags);
	return 1U;
}

/*
 * QUEUE_DEPTH get status command
 */
struct pva_status_queue_depth {
	uint32_t	cur_depth;
	uint32_t	avg_depth;
	uint32_t	min_depth;
	uint32_t	max_depth;
};

static inline uint32_t
pva_cmd_queue_depth(struct pva_cmd * const cmd,
		    const uint8_t queue,
		    const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(QUEUE_DEPTH, flags)
		       | PVA_INSERT(queue, 23, 16);
	return 1U;
}

/*
 * QUEUE_WAIT_TIMES get status command
 */
struct pva_status_queue_wait_times {
	uint32_t	reserved;
	uint32_t	avg_wait;
	uint32_t	min_wait;
	uint32_t	max_wait;
};

static inline uint32_t
pva_cmd_queue_wait_times(struct pva_cmd * const cmd,
			 const uint8_t queue,
			 const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(QUEUE_WAIT_TIME, flags)
		       | PVA_INSERT(queue, 23, 16);
	return 1U;
}

/*
 * WAIT_TIMES get status command
 */
struct pva_status_wait_times {
	uint32_t	reserved;
	uint32_t	avg_wait;
	uint32_t	min_wait;
	uint32_t	max_wait;
};

static inline uint32_t
pva_cmd_wait_times(struct pva_cmd * const cmd,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(WAIT_TIMES, flags);
	return 1U;
}

/*
 * RUN_TIMES get status command
 */
#define PVA_CMD_FL_RUN_TIME_TOTALS	PVA_BIT(28)

union pva_status_run_times {
	struct {
		uint32_t	num_tasks;
		uint32_t	avg_run;
		uint32_t	min_run;
		uint32_t	max_run;
	} stats;
	struct {
		uint32_t	num_tasks;
		uint32_t	reserved;
		uint32_t	total_lo;
		uint32_t	total_hi;
	} totals;
};

static inline uint32_t
pva_cmd_run_times(struct pva_cmd * const cmd,
		  const uint8_t vpu,
		  const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(RUN_TIMES, flags)
		       | PVA_INSERT(vpu, 23, 16);
	return 1U;
}

/*
 * IDLE_TIME get status command
 */
#define PVA_IDLE_TIME_TOTALS		PVA_BIT(28)

union pva_status_idle_time {
	struct {
		uint32_t	num_idles;
		uint32_t	avg_idle;
		uint32_t	min_idle;
		uint32_t	max_idle;
	} stats;
	struct {
		uint32_t	num_idles;
		uint32_t	reserved;
		uint32_t	total_lo;
		uint32_t	total_hi;
	} totals;
};

static inline uint32_t
pva_cmd_idle_times(struct pva_cmd * const cmd,
		   const uint8_t vpu,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(IDLE_TIME, flags)
		       | PVA_INSERT(vpu, 23, 16);
	return 1U;
}


/*
 * RUNNING_TASKS get status command
 */
struct pva_status_running_tasks {
	uint32_t	task_addr_lo;
	uint32_t	task_addr_hi;
};

static inline uint32_t
pva_cmd_running_tasks(struct pva_cmd * const cmd,
		      const uint8_t vpu,
		      const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(RUNNING_TASKS, flags)
		       | PVA_INSERT(vpu, 23, 16);
	return 1U;
}

#define PVA_RUNNING_TASK_VALID		PVA_BIT64(63)
#define pva_get_task_addr(_a_)		PVA_EXTRACT64(_a_, 39, 0, uint64_t)

/*
 * PVA_UPTIME get status command
 */
struct pva_status_pva_uptime {
	uint32_t	uptime_lo;
	uint32_t	uptime_hi;
};

static inline uint32_t
pva_cmd_pva_uptime(struct pva_cmd * const cmd,
		   const uint8_t vpu,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(PVA_UPTIME, flags)
		       | PVA_INSERT(vpu, 23, 16);
	return 1U;
}

/*
 * PVA_ERRORS get status command
 */
struct pva_status_pva_errors {
	uint32_t	num_fatal_errors;
	uint32_t	num_input_errors;
	uint32_t	num_sw_errors;
	uint32_t	num_hw_errors;
};

static inline uint32_t
pva_cmd_pva_errors(struct pva_cmd * const cmd,
		   const uint8_t vpu,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(PVA_ERRORS, flags)
		       | PVA_INSERT(vpu, 23, 16);
	return 1U;
}

/*
 * LOGGING_INFO get status command
 */
struct pva_status_logging_info {
	uint32_t	status;
	uint32_t	length;
	uint32_t	addr_lo;
	uint32_t	addr_hi;
};

static inline uint32_t
pva_cmd_logging_info(struct pva_cmd * const cmd,
		     const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(LOGGING_INFO, flags);
	return 1U;
}

#define PVA_LOG_PVA_ENABLED		PVA_BIT(31)
#define PVA_LOG_R5_ENABLED		PVA_BIT(30)
#define PVA_LOG_VPU_ENABLED		PVA_BIT(29)
#define PVA_LOG_PVA_ACTIVE		PVA_BIT(28)

#define pva_get_pva_log_level(_x_)	PVA_EXTRACT(_x_, 23, 16, uint32_t)
#define pva_get_r5_log_level(_x_)	PVA_EXTRACT(_x_, 15, 8, uint32_t)
#define pva_get_vpu_log_level(_x_)	PVA_EXTRACT(_x_, 7, 0, uint32_t)

#define pva_get_crit_level(_l_)		PVA_EXTRACT(_l_, 7, 6, uint32_t)
#define pva_get_error_level(_l_)	PVA_EXTRACT(_l_, 5, 4, uint32_t)
#define pva_get_info_level(_l_)		PVA_EXTRACT(_l_, 3, 2, uint32_t)
#define pva_get_debug_level(_l_)	PVA_EXTRACT(_l_, 1, 0, uint32_t)

/*
 * PRINTF_INFO get status command
 */
struct pva_status_printf_info {
	uint32_t	status;
	uint32_t	length;
	uint32_t	addr_lo;
	uint32_t	addr_hi;
};

static inline uint32_t
pva_cmd_printf_info(struct pva_cmd * const cmd,
		    const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(PRINTF_INFO, flags);
	return 1U;
}

#define PVA_PRINT_PVA_ENABLED		PVA_BIT(31)
#define PVA_PRINT_R5_ENABLED		PVA_BIT(30)
#define PVA_PRINT_VPU_ENABLED		PVA_BIT(29)
#define PVA_PRINT_PVA_ACTIVE		PVA_BIT(28)

#define pva_get_pva_print_level(_x_)	PVA_EXTRACT(_x_, 23, 16, uint32_t)
#define pva_get_r5_print_level(_x_)	PVA_EXTRACT(_x_, 15, 8, uint32_t)
#define pva_get_vpu_print_level(_x_)	PVA_EXTRACT(_x_, 7, 0, uint32_t)

/*
 * CRASH_INFO get status command
 */
struct pva_status_crash_info {
	uint32_t	status;
	uint32_t	length;
	uint32_t	addr_lo;
	uint32_t	addr_hi;
};

static inline uint32_t
pva_cmd_crash_info(struct pva_cmd * const cmd,
		   const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(CRASH_INFO, flags);
	return 1U;
}

#define PVA_CRASH_PVA_ENABLED		PVA_BIT(31)
#define PVA_CRASH_R5_ENABLED		PVA_BIT(30)
#define PVA_CRASH_VPU_ENABLED		PVA_BIT(29)
#define PVA_CRASH_R5_CRASHED_INT	PVA_BIT(27)
#define PVA_CRASH_VPU0_CRASHED_INT	PVA_BIT(26)
#define PVA_CRASH_VPU1_CRASHED_INT	PVA_BIT(25)
#define PVA_CRASH_R5_CRASHED		PVA_BIT(15)
#define PVA_CRASH_VPU0_CRASHED		PVA_BIT(14)
#define PVA_CRASH_VPU1_CRASHED		PVA_BIT(13)

/*
 * THRESHOLD get status command
 */
struct pva_status_threshold {
	uint32_t	value;
	uint32_t	count;
	uint32_t	cur_count;
	uint32_t	reserved;
};

static inline uint32_t
pva_cmd_threshold(struct pva_cmd * const cmd,
		  const enum pva_threshold_id threshold_id,
		  const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(THRESHOLD, flags)
		       | PVA_INSERT(threshold_id, 23, 16);
	return 1U;
}


/*
 * THRESHOLDS_EXCEEDED get status command
 */
struct pva_status_thresholds_exceeded {
	uint32_t	status;
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
};

static inline uint32_t
pva_cmd_thresholds_exceeded(struct pva_cmd * const cmd,
			    const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(THRESHOLDS_EXCEEDED, flags);
	return 1U;
}

#define PVA_IDLE_TIME_EXCEEDED		PVA_BIT(31)
#define PVA_UTILIZATION_EXCEEDED	PVA_BIT(30)
#define PVA_LOW_UTILIZATION		PVA_BIT(29)
#define PVA_QUEUE_DEPTH_EXCEEDED	PVA_BIT(8)
#define PVA_WAIT_TIME_EXCEEDED		PVA_BIT(7)
#define PVA_RUN_TIME_EXCEEDED		PVA_BIT(6)
#define PVA_HW_ERRORS_EXCEEDED		PVA_BIT(5)
#define PVA_R5_CRIT_ERRORS		PVA_BIT(4)
#define PVA_R5_ERRORS			PVA_BIT(3)
#define PVA_VPU_CRIT_ERRORS		PVA_BIT(2)
#define PVA_VPU_ERRORS			PVA_BIT(1)
#define PVA_TOTAL_ERRORS		PVA_BIT(0)


/*
 * QUEUE_ATTRIBUTES get status command
 */

struct pva_status_queue_attributes {
	uint32_t	attribute;
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
};

static inline uint32_t
pva_cmd_queue_attributes(struct pva_cmd * const cmd,
			 const uint8_t queue_id,
			 const enum pva_queue_attr_id attribute_id,
			 const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(QUEUE_ATTRIBUTES, flags)
		       | PVA_INSERT(queue_id, 23, 16);
	cmd->mbox[1] = (uint32_t)attribute_id;
	return 2U;
}

/*
 * COMPLETED_TASK get status command
 */
struct pva_status_completed_task {
	uint32_t	task_addr_lo;
	uint32_t	task_addr_hi;
	uint32_t	task_error;
	uint32_t	task_queue_vpu;
};

static inline uint32_t pva_cmd_completed_task(struct pva_cmd * const cmd,
					      const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(COMPLETED_TASK, flags);
	return 1U;
}

/*
 * VMEM_USAGE get status command
 */

struct pva_status_vmem_usage {
	uint32_t	offset;
	uint32_t	reserved1;
	uint32_t	reserved2;
	uint32_t	reserved3;
};

static inline uint32_t pva_cmd_vmem_usage(struct pva_cmd * const cmd,
					  const uint8_t vpu,
					  const uint8_t vmem_id,
					  const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(VMEM_USAGE, flags)
		       | PVA_INSERT(vpu, 23, 16);
	cmd->mbox[1] = vmem_id;
	return 2U;
}

/*
 * SYNC_POLL_RATE get status command
 */
struct pva_status_sync_poll_rate {
	uint32_t	cur_rate;
	uint32_t	min_rate;
	uint32_t	reserved1;
	uint32_t	reserved2;
};

static inline uint32_t
pva_cmd_sync_poll_rate(struct pva_cmd * const cmd,
		       const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(SYNC_POLL_RATE, flags);
	return 1U;
}

/*
 * SCHEDULER_ATTRIBUTES get status command
 */

struct pva_status_scheduler_attributes {
	uint32_t	attr_lo;
	uint32_t	attr_hi;
	uint32_t	reserved1;
	uint32_t	reserved2;
};

static inline uint32_t
pva_cmd_scheduler_attributes(struct pva_cmd * const cmd,
			     const enum pva_sched_id sched_id,
			     const enum pva_sched_attr_id attr_id,
			     const uint32_t flags)
{
	cmd->mbox[0] = pva_cmd_get_status(SCHEDULER_ATTRIBUTES, flags)
		       | PVA_INSERT(sched_id, 23, 16);
	cmd->mbox[1] = (uint32_t)attr_id;
	return 2U;
}

/*
 * CMD_MANAGE_STATISTICS
 */

static inline uint32_t
pva_cmd_manage_statistics(struct pva_cmd * const cmd,
			  const enum pva_stats_ids stat_id,
			  const uint32_t instance,
			  const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_MANAGE_STATISTICS)
		       | PVA_INSERT(stat_id, 15, 8)
		       | PVA_INSERT(instance, 23, 16);
	return 1U;
}

/*
 * CMD_SET_REGION
 */

struct pva_status_set_region {
	uint32_t	header;
	uint32_t	region_length;
	uint32_t	reserved1;
	uint32_t	reserved2;
};

static inline uint32_t
pva_cmd_set_region(struct pva_cmd * const cmd,
		   const enum pva_region_ids region,
		   const uint32_t length,
		   const uint64_t addr,
		   const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_REGION)
		       | PVA_INSERT(PVA_EXTRACT64(addr, 39, 40, uint32_t),
				    23, 16)
		       | PVA_INSERT(region, 15, 8);
	cmd->mbox[1] = PVA_LOW32(addr);
	cmd->mbox[2] = length;
	return 3U;
}

#define pva_get_region_header_len(_x_)		PVA_EXTRACT(31, 16, uint32_t)
#define pva_get_region_element_len(_x_)		PVA_EXTRACT(15, 0, uint32_t)

/*
 * CMD_SET_LOGGING
 */

#define PVA_CMD_FL_LOG_PVA_ENABLE	PVA_BIT(28)
#define PVA_CMD_FL_LOG_R5_ENABLE	PVA_BIT(27)
#define PVA_CMD_FL_LOG_VPU_ENABLE	PVA_BIT(26)
#define PVA_CMD_FL_LOG_NO_OVERFLOW	PVA_BIT(25)
#define PVA_CMD_FL_LOG_OVERFLOW_INT	PVA_BIT(24)
#define PVA_CMD_FL_PRT_PVA_ENABLE	PVA_BIT(23)
#define PVA_CMD_FL_PRT_R5_ENABLE	PVA_BIT(22)
#define PVA_CMD_FL_PRT_VPU_ENABLE	PVA_BIT(21)
#define PVA_CMD_FL_PRT_NO_OVERFLOW	PVA_BIT(20)
#define PVA_CMD_FL_PRT_OVERFLOW_INT	PVA_BIT(19)

static inline uint32_t
pva_cmd_set_logging_level(struct pva_cmd * const cmd,
			  const uint32_t log_level,
			  const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags | PVA_SET_COMMAND(CMD_SET_LOGGING);
	cmd->mbox[1] = log_level;
	return 2U;
}


/*
 * CMD_VPU_CONTROL
 */
#define PVA_CMD_FL_VPU_ONLINE		PVA_BIT(28)
#define PVA_CMD_FL_VPU_INT_ON_CRASH	PVA_BIT(27)
#define PVA_CMD_FL_VPU_DUMP_ON_CRASH	PVA_BIT(26)
#define PVA_CMD_FL_VPU_FAIL_ON_CRASH	PVA_BIT(25)
#define PVA_CMD_FL_VPU_RESUME_ON_CRASH	PVA_BIT(24)
#define PVA_CMD_FL_VPU_STOP_ON_CRASH	PVA_BIT(23)

static inline uint32_t
pva_cmd_vpu_control(struct pva_cmd * const cmd,
		    const uint8_t vpu,
		    const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_VPU_CONTROL)
		       | PVA_INSERT(vpu, 15, 8);
	return 1U;
}

/*
 * CMD_PVA_CONTROL
 */
#define PVA_CMD_FL_R5_TASK_CMPL_INT	PVA_BIT(28)
#define PVA_CMD_FL_R5_TASK_ERR_INT	PVA_BIT(27)
#define PVA_CMD_FL_R5_INT_ON_CRASH	PVA_BIT(26)
#define PVA_CMD_FL_R5_DUMP_ON_CRASH	PVA_BIT(25)

static inline uint32_t
pva_cmd_pva_control(struct pva_cmd * const cmd,
		    const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_PVA_CONTROL);
	return 1U;
}

/*
 * CMD_SET_QUEUE_ATTRIBUTES
 */

static inline uint32_t
pva_cmd_set_queue_attributes(struct pva_cmd * const cmd,
			     const uint8_t queue_id,
			     const enum pva_queue_attr_id attr_id,
			     const uint32_t attribute,
			     const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_QUEUE_ATTRIBUTES)
		       | PVA_INSERT(queue_id, 15, 8)
		       | PVA_INSERT(attr_id, 23, 16);
	cmd->mbox[1] = attribute;
	return 2U;
}

/*
 * CMD_SET_THRESHOLD
 */

static inline uint32_t
pva_cmd_set_threshold(struct pva_cmd * const cmd,
		      const uint32_t threshold_id,
		      const uint32_t threshold_count,
		      const uint32_t threshold_value,
		      const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_THRESHOLD)
		       | PVA_INSERT(threshold_id, 15, 8);
	cmd->mbox[1] = threshold_count;
	cmd->mbox[2] = threshold_value;
	return 3U;
}

/*
 * CMD_CLEAR_THRESHOLD
 */

#define PVA_CMD_FL_THRESHOLD_CLEAR	PVA_BIT(28)
#define PVA_CMD_FL_THRESHOLD_DISABLE	PVA_BIT(27)

static inline uint32_t
pva_cmd_clear_threshold(struct pva_cmd * const cmd,
			const enum pva_threshold_id threshold_id,
			const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_CLEAR_THRESHOLD)
		       | PVA_INSERT(threshold_id, 15, 0);
	return 1U;
}

/*
 * CMD_GET_VPU_FUNC_TABLE
 */

struct pva_status_get_vpu_funcs {
	uint32_t	length;
	uint32_t	num_entries;
};

struct vpu_func {
	uint32_t	next;
	uint32_t	cur_version;
	uint32_t	compat_version;
	uint32_t	id;
	uint16_t	name_len;
	uint8_t		name;		/* place holder */
};

static inline uint32_t
pva_cmd_get_vpu_func_table(struct pva_cmd * const cmd,
			   const uint32_t length,
			   const uint64_t addr,
			   const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_GET_VPU_FUNC_TABLE)
		       | PVA_INSERT(PVA_EXTRACT64(addr, 39, 40, uint32_t),
				    23, 16);
	cmd->mbox[1] = PVA_LOW32(addr);
	cmd->mbox[2] = length;
	return 3U;
}

/*
 * CMD_RESET
 */

static inline uint32_t
pva_cmd_reset(struct pva_cmd *const cmd,
	      const enum pva_reset_id object,
	      const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_RESET)
		       | PVA_INSERT(object, 15, 8);
	return 1U;
}

/*
 * CMD_SET_SYNC_POLLING
 */
static inline uint32_t
pva_cmd_set_sync_polling(struct pva_cmd * const cmd,
			 const uint32_t polling_rate,
			 const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_SYNC_POLLING);
	cmd->mbox[1] = polling_rate;
	return 2U;
}

/*
 * CMD_FIFO_SUBMIT
 */
static inline uint64_t
pva_fifo_submit(const uint8_t queue_id,
		const uint64_t addr,
		const uint64_t flags)
{
	return PVA_INSERT64(CMD_SUBMIT, 63, 56)
		| PVA_INSERT64(addr, 39, 0)
		| PVA_INSERT64(queue_id, 47, 40)
		| flags;
}


/*
 * CMD_SUBMIT
 */
static inline uint32_t
pva_cmd_submit(struct pva_cmd *const cmd,
	       const uint8_t queue_id,
	       const uint64_t addr,
	       const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SUBMIT)
		       | PVA_INSERT(queue_id, 15, 8)
		       | PVA_INSERT(PVA_EXTRACT64(addr, 39, 32, uint32_t),
				    23, 16);
	cmd->mbox[1] = PVA_LOW32(addr);
	return 2U;
}

/*
 * CMD_SUSPEND
 */
#define PVA_CMD_FL_SUSPEND_ABORT	PVA_BIT(28)
#define PVA_CMD_FL_SUSPEND_FLUSH	PVA_BIT(27)
#define PVA_CMD_FL_SUSPEND_NOTIFY	PVA_BIT(26)
#define PVA_CMD_FL_SUSPEND_WAIT		PVA_BIT(25)

static inline uint32_t
pva_cmd_suspend(struct pva_cmd * const cmd,
		const uint8_t queue_id,
		const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SUSPEND)
		       | PVA_INSERT(queue_id, 15, 8);
	return 1U;
}

/*
 * CMD_RESUME
 */
#define PVA_CMD_FL_RESUME_PREEMPT	PVA_BIT(28)
#define PVA_CMD_FL_RESUME_PREEMPT_ERROR	PVA_BIT(27)

static inline uint32_t
pva_cmd_resume(struct pva_cmd * const cmd,
	       const uint8_t queue_id,
	       const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_RESUME)
		       | PVA_INSERT(queue_id, 15, 8);
	return 1U;
}

/*
 * CMD_SET_SCHEDULER
 */

#define PVA_CMD_FL_SCHED_ENABLE		PVA_BIT(28)
#define PVA_CMD_FL_SCHED_RESET		PVA_BIT(27)

static inline uint32_t
pva_cmd_set_scheduler(struct pva_cmd * const cmd,
		      const enum pva_sched_id sched_id,
		      const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_SCHEDULER)
		       | PVA_INSERT(sched_id, 15, 8);
	return 1U;
}

/*
 * CMD_SET_SCHED_ATTR
 */
static inline uint32_t
pva_cmd_set_sched_attr(struct pva_cmd * const cmd,
		       const enum pva_sched_id sched_id,
		       const enum pva_sched_attr_id attr_id,
		       const struct pva_sched_attr *attrs,
		       const uint32_t flags)
{
	cmd->mbox[0] = PVA_CMD_GO | flags
		       | PVA_SET_COMMAND(CMD_SET_SCHED_ATTR)
		       | PVA_INSERT(sched_id, 15, 8)
		       | PVA_INSERT(attr_id, 23, 16);
	cmd->mbox[1] = attrs->attr_a;
	cmd->mbox[2] = attrs->attr_b;
	return 3U;
}

#endif
