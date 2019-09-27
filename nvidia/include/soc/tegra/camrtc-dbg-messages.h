/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_DBG_MESSAGES_H
#define INCLUDE_CAMRTC_DBG_MESSAGES_H

#include "camrtc-common.h"

/* All the enums and the fields inside the structs described in this header
 * file supports only uintX_t types, where X can be 8,16,32,64.
 */

/* Requests and responses
 */

/* Ping request. RTCPU returns the fw version in the data field and
 * 0 in the status field.

*/
#define CAMRTC_REQ_PING                 0x01U
/* PM sleep request */
#define CAMRTC_REQ_PM_SLEEP             0x02U
/* Test request */
#define CAMRTC_REQ_MODS_TEST            0x03U
/* Set log level */
#define CAMRTC_REQ_SET_LOGLEVEL         0x04U
#define CAMRTC_REQ_LOGLEVEL             CAMRTC_REQ_SET_LOGLEVEL
/* Get FreeRTOS state */
#define CAMRTC_REQ_RTOS_STATE           0x05U
/* Read memory */
#define CAMRTC_REQ_READ_MEMORY_32BIT    0x06U
#define CAMRTC_REQ_READ_MEMORY          0x07U
/* Performance counter */
#define CAMRTC_REQ_SET_PERF_COUNTERS    0x08U
#define CAMRTC_REQ_GET_PERF_COUNTERS    0x09U
#define CAMRTC_REQ_GET_LOGLEVEL         0x0AU
#define CAMRTC_REQ_RUN_TEST             0x0BU
#define CAMRTC_REQ_GET_TASK_STAT        0x0CU
#define CAMRTC_REQ_ENABLE_VI_STAT       0x0DU
#define CAMRTC_REQ_GET_VI_STAT          0x0EU
#define CAMRTC_REQ_GET_MEM_USAGE        0x0FU
#define CAMRTC_REQ_RUN_MEM_TEST         0x10U
#define CAMRTC_REQ_GET_IRQ_STAT		0x11U
#define CAMRTC_REQUEST_TYPE_MAX		0x12U

enum camrtc_response {
	CAMRTC_RESP_PONG = 1,
	CAMRTC_RESP_PM_SLEEP,
	CAMRTC_RESP_MODS_RESULT,
	CAMRTC_RESP_LOGLEVEL,
	CAMRTC_RESP_RTOS_STATE,

	CAMRTC_RESP_READ_MEMORY_32BIT,
	CAMRTC_RESP_READ_MEMORY,

	CAMRTC_RESP_SET_PERF_COUNTERS,
	CAMRTC_RESP_GET_PERF_COUNTERS,

	CAMRTC_RESPONSE_TYPE_MAX,
};

#define CAMRTC_STATUS_OK                0U
#define CAMRTC_STATUS_ERROR             1U /* Generic error */
#define CAMRTC_STATUS_REQ_UNKNOWN       2U /* Unknown req_type */
#define CAMRTC_STATUS_NOT_IMPLEMENTED   3U /* Request not implemented */
#define CAMRTC_STATUS_INVALID_PARAM     4U /* Invalid parameter */

enum {
	CAMRTC_DBG_FRAME_SIZE = 384U,
	CAMRTC_DBG_MAX_DATA = 376U,
	CAMRTC_DBG_READ_MEMORY_COUNT_MAX = 256U,
	CAMRTC_DBG_MAX_PERF_COUNTERS = 31U,
	CAMRTC_DBG_TASK_STAT_MAX = 16U,
	/** Limit for default CAMRTC_DBG_FRAME_SIZE */
	CAMRTC_DBG_IRQ_STAT_MAX = 11U,
};

/* This struct is used to query or set the wake timeout for the target.
 * Fields:
 * force_entry:	when set forces the target to sleep for a set time
 */
struct camrtc_pm_data {
	uint32_t force_entry;
} __packed;

/* This struct is used to send the loop count to perform the mods test
 * on the target.
 * Fields:
 * mods_loops:	number of times mods test should be run
 */
struct camrtc_mods_data {
	uint32_t mods_loops;
} __packed;

/* This struct is used to extract the firmware version of the RTCPU.
 * Fields:
 * data:	buffer to store the version string. Uses uint8_t
 */
struct camrtc_ping_data {
	uint64_t ts_req;		/* requestor timestamp */
	uint64_t ts_resp;		/* response timestamp */
	uint8_t data[64];		/* data */
} __packed;

struct camrtc_log_data {
	uint32_t level;
} __packed;

struct camrtc_rtos_state_data {
	uint8_t rtos_state[CAMRTC_DBG_MAX_DATA];	/* string data */
} __packed;

/* This structure is used to read 32 bit data from firmware address space.
 * Fields:
 *   addr: address to read from. should be 4 byte aligned.
 *   data: 32 bit value read from memory.
 */
struct camrtc_dbg_read_memory_32bit {
	uint32_t addr;
} __packed;

struct camrtc_dbg_read_memory_32bit_result {
	uint32_t data;
} __packed;

/* This structure is used to read memory in firmware address space.
 * Fields:
 *   addr: starting address. no alignment requirement
 *   count: number of bytes to read. limited to CAMRTC_DBG_READ_MEMORY_COUNT_MAX
 *   data: contents read from memory.
 */
struct camrtc_dbg_read_memory {
	uint32_t addr;
	uint32_t count;
} __packed;

struct camrtc_dbg_read_memory_result {
	uint8_t data[CAMRTC_DBG_READ_MEMORY_COUNT_MAX];
} __packed;

/* These structure is used to set event type that each performance counter
 * will monitor. This doesn't include fixed performance counter. If there
 * are 4 counters available, only 3 of them are configurable.
 * Fields:
 *   number: Number of performance counters to set.
 *     This excludes a fixed performance counter: cycle counter
 *   do_reset: Whether to reset counters
 *   cycle_counter_div64: Whether to enable cycle counter divider
 *   events: Event type to monitor
 */
struct camrtc_dbg_set_perf_counters {
	uint32_t number;
	uint32_t do_reset;
	uint32_t cycle_counter_div64;
	uint32_t events[CAMRTC_DBG_MAX_PERF_COUNTERS];
} __packed;

/* These structure is used to get performance counters.
 * Fields:
 *   number: Number of performance counters.
 *     This includes a fixed performance counter: cycle counter
 *   counters: Descriptors of event counters. First entry is for cycle counter.
 *     event: Event type that the value represents.
 *       For first entry, this field is don't care.
 *     value: Value of performance counter.
 */
struct camrtc_dbg_get_perf_counters_result {
	uint32_t number;
	struct {
		uint32_t event;
		uint32_t value;
	} counters[CAMRTC_DBG_MAX_PERF_COUNTERS];
} __packed;


#define CAMRTC_DBG_MAX_TEST_DATA (CAMRTC_DBG_MAX_DATA - 8U)

/* This structure is used pass textual input data to functional test
 * case and get back the test output, including verdict.
 *
 * Fields:
 *   timeout: maximum time test may run in nanoseconds
 *      data: textual data (e.g., test name, verdict)
 */
struct camrtc_dbg_run_test_data {
	uint64_t timeout;	/* Time in nanoseconds */
	char data[CAMRTC_DBG_MAX_TEST_DATA];
} __packed;

#define CAMRTC_DBG_NUM_MEM_TEST_MEM 8U

#define CAMRTC_DBG_MAX_MEM_TEST_DATA (CAMRTC_DBG_MAX_DATA - 12U - \
		(sizeof(struct camrtc_dbg_test_mem) * CAMRTC_DBG_NUM_MEM_TEST_MEM))

struct camrtc_dbg_test_mem {
	uint64_t size;
	uint64_t rtcpu_iova;
	uint64_t vi_iova;
	uint64_t isp_iova;
};

struct camrtc_dbg_streamids {
	uint8_t rtcpu;
	uint8_t vi;
	uint8_t isp;
	uint8_t _pad;
};

/* This structure is used pass memory areas and textual input data to
 * functional test case and get back the test output, including
 * verdict.
 *
 * Fields:
 *   timeout: maximum time test may run in nanoseconds
 *     mem[]: address and size of memory areas passed to the test
 *      data: textual data (e.g., test name, verdict)
 */
struct camrtc_dbg_run_mem_test_data {
	uint64_t timeout;	/* Time in nanoseconds */
	struct camrtc_dbg_test_mem mem[CAMRTC_DBG_NUM_MEM_TEST_MEM];
	struct camrtc_dbg_streamids streamids;
	char data[CAMRTC_DBG_MAX_MEM_TEST_DATA];
} __packed;

/* This structure is used get information on system tasks.
 * Fields:
 *   n_task: number of reported tasks
 *   total_count: total runtime
 *   task: array of reported tasks
 *     id: task name
 *     count: runtime allocated to task
 *     number: unique task number
 *     priority: priority of task when this structure was populated
 */
struct camrtc_dbg_task_stat {
	uint32_t n_task;
	uint32_t total_count;
	struct {
		uint32_t id[2];
		uint32_t count;
		uint32_t number;
		uint32_t priority;
	} task[CAMRTC_DBG_TASK_STAT_MAX];
} __packed;

/*
 * This structure is used get information on interrupts.
 *
 * Fields:
 *   n_active: number of active interrupts
 *   total_called: total number of interrupts handled
 *   total_runtime: total runtime
 *   n_irq: number of reported interrupts
 *   irqs: array of reported tasks
 *     irq_num: irq number
 *     num_called: times this interrupt has been handled
 *     runtime: runtime for this interrupt
 *     name: name of the interrupt (may not be NUL-terminated)
 */
struct camrtc_dbg_irq_stat {
	uint32_t n_active;
	uint32_t n_irq;
	uint64_t total_called;
	uint64_t total_runtime;
	struct {
		uint32_t irq_num;
		char name[12];
		uint64_t runtime;
		uint32_t max_runtime;
		uint32_t num_called;
	} irqs[CAMRTC_DBG_IRQ_STAT_MAX];
} __packed;

/* These structure is used to get VI message statistics.
 * Fields:
 *   enable: enable/disable collecting vi message statistics
 */
struct camrtc_dbg_enable_vi_stat {
	uint32_t enable;
} __packed;

/* These structure is used to get VI message statistics.
 * Fields:
 *   avg: running average of VI message latency.
 *   max: maximum VI message latency observed so far.
 */
struct camrtc_dbg_vi_stat {
	uint32_t avg;
	uint32_t max;
} __packed;

/* These structure is used to get memory usage.
 * Fields:
 *   text: code memory usage
 *   bss: global/static memory usage.
 *   data: global/static memory usage.
 *   heap: heap memory usage.
 *   stack: cpu stack memory usage.
 *   free: remaining free memory.
 */
struct camrtc_dbg_mem_usage {
	uint32_t text;
	uint32_t bss;
	uint32_t data;
	uint32_t heap;
	uint32_t stack;
	uint32_t free;
} __packed;

/* This struct encapsulates the type of the request and the respective
 * data associated with that request.
 * Fields:
 * req_type:	indicates the type of the request be it pm related,
 *		mods or ping.
 * data:	Union of structs of all the request types.
 */
struct camrtc_dbg_request {
	uint32_t req_type;
	uint32_t reserved;
	union {
		struct camrtc_pm_data	pm_data;
		struct camrtc_mods_data mods_data;
		struct camrtc_ping_data ping_data;
		struct camrtc_log_data	log_data;
		struct camrtc_dbg_read_memory_32bit rm_32bit_data;
		struct camrtc_dbg_read_memory rm_data;
		struct camrtc_dbg_set_perf_counters set_perf_data;
		struct camrtc_dbg_run_test_data run_test_data;
		struct camrtc_dbg_run_mem_test_data run_mem_test_data;
		struct camrtc_dbg_enable_vi_stat enable_vi_stat;
	} data;
} __packed;

/* This struct encapsulates the type of the response and the respective
 * data associated with that response.
 * Fields:
 * resp_type:	indicates the type of the response be it pm related,
 *		mods or ping.
 * status:	response in regard to the request i.e success/failure.
 *		In case of mods, this field is the result.
 * data:	Union of structs of all the request/response types.
 */
struct camrtc_dbg_response {
	uint32_t resp_type;
	uint32_t status;
	union {
		struct camrtc_pm_data pm_data;
		struct camrtc_ping_data ping_data;
		struct camrtc_log_data log_data;
		struct camrtc_rtos_state_data rtos_state_data;
		struct camrtc_dbg_read_memory_32bit_result rm_32bit_data;
		struct camrtc_dbg_read_memory_result rm_data;
		struct camrtc_dbg_get_perf_counters_result get_perf_data;
		struct camrtc_dbg_run_test_data run_test_data;
		struct camrtc_dbg_run_mem_test_data run_mem_test_data;
		struct camrtc_dbg_task_stat task_stat_data;
		struct camrtc_dbg_vi_stat vi_stat;
		struct camrtc_dbg_mem_usage mem_usage;
		struct camrtc_dbg_irq_stat irq_stat;
	} data;
} __packed;

#endif /* INCLUDE_CAMRTC_DBG_MESSAGES_H */
