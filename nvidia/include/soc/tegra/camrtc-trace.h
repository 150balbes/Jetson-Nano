/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef INCLUDE_CAMRTC_TRACE_H
#define INCLUDE_CAMRTC_TRACE_H

#include "camrtc-common.h"
#include "camrtc-channels.h"

#pragma GCC diagnostic error "-Wpadded"

/*
 * Trace memory consists of three part.
 *
 * 1. Trace memory header: This describes the layout of trace memory,
 * and latest activities.
 *
 * 2. Exception memory: This is an array of exception entries. Each
 * entry describes an exception occurred in the firmware.
 *
 * 3. Event memory: This is an array of event entries. This is implemented
 * as a ring buffer.
 *
 * The next index gets updated when new messages are committed to the
 * trace memory. The next index points to the entry to be written to
 * at next occurrence of the exception or event.
 *
 * Trace memory layout
 *
 * 0x00000 +-------------------------------+
 *         |      Trace Memory Header      |
 * 0x01000 +-------------------------------+
 *         |                               |
 *         |        Exception Memory       | <- exception_next_idx
 *         |                               |
 * 0x10000 +-------------------------------+
 *         |                               |
 *         |                               |
 *         |          Event Memory         |
 *         |                               | <- event_next_idx
 *         |                               |
 *         +-------------------------------+
 */

/* Offset of each memory */
#define CAMRTC_TRACE_NEXT_IDX_SIZE	U32_C(64)
#define CAMRTC_TRACE_EXCEPTION_OFFSET	U32_C(0x01000)
#define CAMRTC_TRACE_EVENT_OFFSET	U32_C(0x10000)

/* Size of each entry */
#define CAMRTC_TRACE_EXCEPTION_SIZE	U32_C(1024)
#define CAMRTC_TRACE_EVENT_SIZE		U32_C(64)

/* Depth of call stack */
#define CAMRTC_TRACE_CALLSTACK_MAX	U32_C(32)
#define CAMRTC_TRACE_CALLSTACK_MIN	U32_C(4)

/*
 * Trace memory header
 */

#define CAMRTC_TRACE_SIGNATURE_1	U32_C(0x5420564e)
#define CAMRTC_TRACE_SIGNATURE_2	U32_C(0x45434152)

#define CAMRTC_TRACE_ALIGN		__aligned(U32_C(64))

struct camrtc_trace_memory_header {
	/* layout: offset 0 */
	union {
		/*
		 * Temporary union to provide source compatiblity
		 * during the transition to new header format.
		 */
		struct camrtc_tlv tlv;
		uint32_t signature[4] __attribute__((deprecated));
	};
	uint32_t revision;
	uint32_t reserved1;
	uint32_t exception_offset;
	uint32_t exception_size;
	uint32_t exception_entries;
	uint32_t reserved2;
	uint32_t event_offset;
	uint32_t event_size;
	uint32_t event_entries;
	uint32_t reserved3;
	uint32_t reserved4[0xc8 / 4];

	/* pointer: offset 0x100 */
	uint32_t exception_next_idx;
	uint32_t event_next_idx;
	uint32_t reserved_ptrs[0x38 / 4];
} CAMRTC_TRACE_ALIGN;


/*
 * Exception entry
 */

enum camrtc_trace_armv7_exception_type {
	/* Reset = 0 */
	CAMRTC_ARMV7_EXCEPTION_UNDEFINED_INSTRUCTION = 1,
	/* SWI = 2 */
	CAMRTC_ARMV7_EXCEPTION_PREFETCH_ABORT = 3,
	CAMRTC_ARMV7_EXCEPTION_DATA_ABORT,
	CAMRTC_ARMV7_EXCEPTION_RSVD,	/* Should never happen */
	CAMRTC_ARMV7_EXCEPTION_IRQ,	/* Unhandled IRQ */
	CAMRTC_ARMV7_EXCEPTION_FIQ,	/* Unhandled FIQ */
};

struct camrtc_trace_callstack {
	uint32_t lr_stack_addr;		/* address in stack where lr is saved */
	uint32_t lr;			/* value of saved lr */
};

struct camrtc_trace_armv7_exception {
	uint32_t len;		/* length in byte including this */
	uint32_t type;		/* enum camrtc_trace_armv7_exception_type */
	union {
		uint32_t data[24];
		struct {
			uint32_t r0, r1, r2, r3;
			uint32_t r4, r5, r6, r7;
			uint32_t r8, r9, r10, r11;
			uint32_t r12, sp, lr, pc;
			uint32_t r8_prev, r9_prev, r10_prev, r11_prev, r12_prev;
			uint32_t sp_prev, lr_prev;
			uint32_t reserved;
		};
	} gpr;
	/* program status registers */
	uint32_t cpsr, spsr;
	/* data fault status/address register */
	uint32_t dfsr, dfar, adfsr;
	/* instruction fault status/address register */
	uint32_t ifsr, ifar, aifsr;
	struct camrtc_trace_callstack callstack[CAMRTC_TRACE_CALLSTACK_MAX];
};

/*
 * Each trace event shares the header.
 * The format of event data is determined by event type.
 */

#define CAMRTC_TRACE_EVENT_HEADER_SIZE		U32_C(16)
#define CAMRTC_TRACE_EVENT_PAYLOAD_SIZE		\
	(CAMRTC_TRACE_EVENT_SIZE - CAMRTC_TRACE_EVENT_HEADER_SIZE)

#define CAMRTC_EVENT_TYPE_OFFSET		U32_C(24)
#define CAMRTC_EVENT_TYPE_MASK			\
	(U32_C(0xff) << CAMRTC_EVENT_TYPE_OFFSET)
#define CAMRTC_EVENT_TYPE_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_TYPE_MASK) >> CAMRTC_EVENT_TYPE_OFFSET)

#define CAMRTC_EVENT_MODULE_OFFSET		U32_C(16)
#define CAMRTC_EVENT_MODULE_MASK		\
	(U32_C(0xff) << CAMRTC_EVENT_MODULE_OFFSET)
#define CAMRTC_EVENT_MODULE_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_MODULE_MASK) >> CAMRTC_EVENT_MODULE_OFFSET)

#define CAMRTC_EVENT_SUBID_OFFSET		U32_C(0)
#define CAMRTC_EVENT_SUBID_MASK			\
	(U32_C(0xffff) << CAMRTC_EVENT_SUBID_OFFSET)
#define CAMRTC_EVENT_SUBID_FROM_ID(id)		\
	(((id) & CAMRTC_EVENT_SUBID_MASK) >> CAMRTC_EVENT_SUBID_OFFSET)

#define CAMRTC_EVENT_MAKE_ID(type, module, subid) \
	(((type) << CAMRTC_EVENT_TYPE_OFFSET) | \
	((module) << CAMRTC_EVENT_MODULE_OFFSET) | (uint32_t)(subid))

struct camrtc_event_header {
	uint32_t len;		/* Size in bytes including this field */
	uint32_t id;		/* Event ID */
	uint64_t tstamp;	/* Timestamp from TKE TSC */
} __packed;

struct camrtc_event_struct {
	struct camrtc_event_header header;
	union {
		uint8_t data8[CAMRTC_TRACE_EVENT_PAYLOAD_SIZE];
		uint32_t data32[CAMRTC_TRACE_EVENT_PAYLOAD_SIZE / 4];
	} data;
} __packed;

// camrtc_event_type
#define CAMRTC_EVENT_TYPE_ARRAY			U32_C(0)
#define CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION	U32_C(1)
#define CAMRTC_EVENT_TYPE_PAD			U32_C(2)
#define CAMRTC_EVENT_TYPE_START			U32_C(3)
#define CAMRTC_EVENT_TYPE_STRING		U32_C(4)
#define CAMRTC_EVENT_TYPE_BULK			U32_C(5)

// camrtc_event_module
#define CAMRTC_EVENT_MODULE_UNKNOWN	U32_C(0)
#define CAMRTC_EVENT_MODULE_BASE	U32_C(1)
#define CAMRTC_EVENT_MODULE_RTOS	U32_C(2)
#define CAMRTC_EVENT_MODULE_HEARTBEAT	U32_C(3)
#define CAMRTC_EVENT_MODULE_DBG		U32_C(4)
#define CAMRTC_EVENT_MODULE_MODS	U32_C(5)
#define CAMRTC_EVENT_MODULE_VINOTIFY	U32_C(6)
#define CAMRTC_EVENT_MODULE_I2C		U32_C(7)
#define CAMRTC_EVENT_MODULE_VI		U32_C(8)
#define CAMRTC_EVENT_MODULE_ISP		U32_C(9)
#define CAMRTC_EVENT_MODULE_NVCSI	U32_C(10)

// camrtc_trace_event_type_ids
#define camrtc_trace_type_exception \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARMV7_EXCEPTION, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define camrtc_trace_type_pad \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_PAD, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define camrtc_trace_type_start \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_START, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))
#define camrtc_trace_type_string \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_STRING, \
		CAMRTC_EVENT_MODULE_BASE, U32_C(0))

// camrtc_trace_base_ids
#define camrtc_trace_base_id(_subid) \
	CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
		CAMRTC_EVENT_MODULE_BASE, (_subid))
#define camrtc_trace_base_target_init \
	camrtc_trace_base_id(1)
#define camrtc_trace_base_start_scheduler \
	camrtc_trace_base_id(2)

// camrtc_trace_event_rtos_ids
#define camrtc_trace_rtos_id(_subid) \
        CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
		CAMRTC_EVENT_MODULE_RTOS, (_subid))
#define camrtc_trace_rtos_task_switched_in \
	camrtc_trace_rtos_id(1)
#define camrtc_trace_rtos_increase_tick_count \
	camrtc_trace_rtos_id(2)
#define camrtc_trace_rtos_low_power_idle_begin \
	camrtc_trace_rtos_id(3)
#define camrtc_trace_rtos_low_power_idle_end \
	camrtc_trace_rtos_id(4)
#define camrtc_trace_rtos_task_switched_out \
	camrtc_trace_rtos_id(5)
#define camrtc_trace_rtos_task_priority_inherit \
	camrtc_trace_rtos_id(6)
#define camrtc_trace_rtos_task_priority_disinherit \
	camrtc_trace_rtos_id(7)
#define camrtc_trace_rtos_blocking_on_queue_receive \
	camrtc_trace_rtos_id(8)
#define camrtc_trace_rtos_blocking_on_queue_send \
	camrtc_trace_rtos_id(9)
#define camrtc_trace_rtos_moved_task_to_ready_state \
	camrtc_trace_rtos_id(10)
#define camrtc_trace_rtos_queue_create \
	camrtc_trace_rtos_id(11)
#define camrtc_trace_rtos_queue_create_failed \
	camrtc_trace_rtos_id(12)
#define camrtc_trace_rtos_create_mutex \
	camrtc_trace_rtos_id(13)
#define camrtc_trace_rtos_create_mutex_failed \
	camrtc_trace_rtos_id(14)
#define camrtc_trace_rtos_give_mutex_recursive \
	camrtc_trace_rtos_id(15)
#define camrtc_trace_rtos_give_mutex_recursive_failed \
	camrtc_trace_rtos_id(16)
#define camrtc_trace_rtos_take_mutex_recursive \
	camrtc_trace_rtos_id(17)
#define camrtc_trace_rtos_take_mutex_recursive_failed \
	camrtc_trace_rtos_id(18)
#define camrtc_trace_rtos_create_counting_semaphore \
	camrtc_trace_rtos_id(19)
#define camrtc_trace_rtos_create_counting_semaphore_failed \
	camrtc_trace_rtos_id(20)
#define camrtc_trace_rtos_queue_send \
	camrtc_trace_rtos_id(21)
#define camrtc_trace_rtos_queue_send_failed \
	camrtc_trace_rtos_id(22)
#define camrtc_trace_rtos_queue_receive \
	camrtc_trace_rtos_id(23)
#define camrtc_trace_rtos_queue_peek \
	camrtc_trace_rtos_id(24)
#define camrtc_trace_rtos_queue_peek_from_isr \
	camrtc_trace_rtos_id(25)
#define camrtc_trace_rtos_queue_receive_failed \
	camrtc_trace_rtos_id(26)
#define camrtc_trace_rtos_queue_send_from_isr \
	camrtc_trace_rtos_id(27)
#define camrtc_trace_rtos_queue_send_from_isr_failed \
	camrtc_trace_rtos_id(28)
#define camrtc_trace_rtos_queue_receive_from_isr \
	camrtc_trace_rtos_id(29)
#define camrtc_trace_rtos_queue_receive_from_isr_failed \
	camrtc_trace_rtos_id(30)
#define camrtc_trace_rtos_queue_peek_from_isr_failed \
	camrtc_trace_rtos_id(31)
#define camrtc_trace_rtos_queue_delete \
	camrtc_trace_rtos_id(32)
#define camrtc_trace_rtos_task_create \
	camrtc_trace_rtos_id(33)
#define camrtc_trace_rtos_task_create_failed \
	camrtc_trace_rtos_id(34)
#define camrtc_trace_rtos_task_delete \
	camrtc_trace_rtos_id(35)
#define camrtc_trace_rtos_task_delay_until \
	camrtc_trace_rtos_id(36)
#define camrtc_trace_rtos_task_delay \
	camrtc_trace_rtos_id(37)
#define camrtc_trace_rtos_task_priority_set \
	camrtc_trace_rtos_id(38)
#define camrtc_trace_rtos_task_suspend \
	camrtc_trace_rtos_id(39)
#define camrtc_trace_rtos_task_resume \
	camrtc_trace_rtos_id(40)
#define camrtc_trace_rtos_task_resume_from_isr \
	camrtc_trace_rtos_id(41)
#define camrtc_trace_rtos_task_increment_tick \
	camrtc_trace_rtos_id(42)
#define camrtc_trace_rtos_timer_create \
	camrtc_trace_rtos_id(43)
#define camrtc_trace_rtos_timer_create_failed \
	camrtc_trace_rtos_id(44)
#define camrtc_trace_rtos_timer_command_send \
	camrtc_trace_rtos_id(45)
#define camrtc_trace_rtos_timer_expired \
	camrtc_trace_rtos_id(46)
#define camrtc_trace_rtos_timer_command_received \
	camrtc_trace_rtos_id(47)
#define camrtc_trace_rtos_malloc \
	camrtc_trace_rtos_id(48)
#define camrtc_trace_rtos_free \
	camrtc_trace_rtos_id(49)
#define camrtc_trace_rtos_event_group_create \
	camrtc_trace_rtos_id(50)
#define camrtc_trace_rtos_event_group_create_failed \
	camrtc_trace_rtos_id(51)
#define camrtc_trace_rtos_event_group_sync_block \
	camrtc_trace_rtos_id(52)
#define camrtc_trace_rtos_event_group_sync_end \
	camrtc_trace_rtos_id(53)
#define camrtc_trace_rtos_event_group_wait_bits_block \
	camrtc_trace_rtos_id(54)
#define camrtc_trace_rtos_event_group_wait_bits_end \
	camrtc_trace_rtos_id(55)
#define camrtc_trace_rtos_event_group_clear_bits \
	camrtc_trace_rtos_id(56)
#define camrtc_trace_rtos_event_group_clear_bits_from_isr \
	camrtc_trace_rtos_id(57)
#define camrtc_trace_rtos_event_group_set_bits \
	camrtc_trace_rtos_id(58)
#define camrtc_trace_rtos_event_group_set_bits_from_isr \
	camrtc_trace_rtos_id(59)
#define camrtc_trace_rtos_event_group_delete \
	camrtc_trace_rtos_id(60)
#define camrtc_trace_rtos_pend_func_call \
	camrtc_trace_rtos_id(61)
#define camrtc_trace_rtos_pend_func_call_from_isr \
	camrtc_trace_rtos_id(62)
#define camrtc_trace_rtos_queue_registry_add \
	camrtc_trace_rtos_id(63)

// camrtc_trace_dbg_ids
#define camrtc_trace_dbg_id(_subid) \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_DBG, (_subid))
#define camrtc_trace_dbg_unknown \
	camrtc_trace_dbg_id(1)
#define camrtc_trace_dbg_enter \
	camrtc_trace_dbg_id(2)
#define camrtc_trace_dbg_exit \
	camrtc_trace_dbg_id(3)
#define camrtc_trace_dbg_set_loglevel \
	camrtc_trace_dbg_id(4)

// camrtc_trace_vinotify_ids
#define camrtc_trace_vinotify_id(_subid) \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_VINOTIFY, (_subid))
#define camrtc_trace_vinotify_handle_msg \
	camrtc_trace_vinotify_id(1)
#define camrtc_trace_vinotify_event_ts64 \
	camrtc_trace_vinotify_id(2)
#define camrtc_trace_vinotify_error_ts64 \
	camrtc_trace_vinotify_id(3)

// camrtc_trace_vi_ids
#define camrtc_trace_vi_id(_subid) \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_VI, (_subid))
#define camrtc_trace_vi_frame_begin \
	camrtc_trace_vi_id(1)
#define camrtc_trace_vi_frame_end \
	camrtc_trace_vi_id(2)

// camrtc_trace_isp_ids
#define camrtc_trace_isp_id(_subid) \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_ISP, (_subid))
#define camrtc_trace_isp_task_begin \
	camrtc_trace_isp_id(1)
#define camrtc_trace_isp_task_end \
	camrtc_trace_isp_id(2)
#define	camrtc_trace_isp_falcon_traces_event \
	camrtc_trace_isp_id(3)

// camrtc_trace_nvcsi_ids
#define camrtc_trace_nvcsi_id(_subid) \
		CAMRTC_EVENT_MAKE_ID(CAMRTC_EVENT_TYPE_ARRAY, \
			CAMRTC_EVENT_MODULE_NVCSI, (_subid))
#define camrtc_trace_nvcsi_intr \
	camrtc_trace_nvcsi_id(1)

#pragma GCC diagnostic ignored "-Wpadded"

#endif /* INCLUDE_CAMRTC_TRACE_H */
