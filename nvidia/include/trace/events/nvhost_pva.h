/*
 * Nvhost event logging to ftrace.
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM nvhost_pva

#if !defined(_TRACE_NVHOST_PVA_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NVHOST_PVA_H

#include <linux/tracepoint.h>


TRACE_EVENT(nvhost_pva_write,

	TP_PROTO(
		u64 delta_time,
		const char *name,
		u8 major,
		u8 minor,
		u8 flags,
		u8 sequence,
		u32 arg1,
		u32 arg2
		),

	TP_ARGS(
		delta_time,
		name,
		major,
		minor,
		flags,
		sequence,
		arg1,
		arg2
		),

	TP_STRUCT__entry(
		__field(u64, delta_time)
		__field(const char *, name)
		__field(u8, major)
		__field(u8, minor)
		__field(u8, flags)
		__field(u8, sequence)
		__field(u32, arg1)
		__field(u32, arg2)
		),

	TP_fast_assign(
		__entry->delta_time = delta_time;
		__entry->name = name;
		__entry->major = major;
		__entry->minor = minor;
		__entry->flags = flags;
		__entry->sequence = sequence;
		__entry->arg1 = arg1;
		__entry->arg2 = arg2;
		),

	TP_printk("time: %llu\t %s\t major: 0x%x\tminor: 0x%x\tflags: 0x%x\t"
		"sequence: 0x%x\targ1: %u\targ2: %u",
		__entry->delta_time, __entry->name, __entry->major,
		__entry->minor, __entry->flags, __entry->sequence,
		__entry->arg1, __entry->arg2)
);

TRACE_EVENT(nvhost_pva_task_stats,

	TP_PROTO(
		const char *name,
		u64 queued_time,
		u64 head_time,
		u64 input_actions_complete,
		u64 vpu_assigned_time,
		u64 vpu_start_time,
		u64 vpu_complete_time,
		u64 complete_time,
		u8 vpu_assigned
		),

	TP_ARGS(
		name,
		queued_time,
		head_time,
		input_actions_complete,
		vpu_assigned_time,
		vpu_start_time,
		vpu_complete_time,
		complete_time,
		vpu_assigned
		),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u64, queued_time)
		__field(u64, head_time)
		__field(u64, input_actions_complete)
		__field(u64, vpu_assigned_time)
		__field(u64, vpu_start_time)
		__field(u64, vpu_complete_time)
		__field(u64, complete_time)
		__field(u8, vpu_assigned)
		),

	TP_fast_assign(
		__entry->name = name;
		__entry->queued_time = queued_time;
		__entry->head_time = head_time;
		__entry->input_actions_complete = input_actions_complete;
		__entry->vpu_assigned_time = vpu_assigned_time;
		__entry->vpu_start_time = vpu_start_time;
		__entry->vpu_complete_time = vpu_complete_time;
		__entry->complete_time = complete_time;
		__entry->vpu_assigned = vpu_assigned;
		),

	TP_printk("%s\tqueued_time: %llu\thead_time: %llu\t"
		"input_actions_complete: %llu\tvpu_assigned_time: %llu\t"
		"vpu_start_time: %llu\tvpu_complete_time: %llu\t"
		"complete_time: %llu\tvpu_assigned: %d",
		__entry->name, __entry->queued_time, __entry->head_time,
		__entry->input_actions_complete, __entry->vpu_assigned_time,
		__entry->vpu_start_time, __entry->vpu_complete_time,
		__entry->complete_time, __entry->vpu_assigned)
);

TRACE_EVENT(nvhost_pva_task_vpu_perf,

	TP_PROTO(
		const char *name,
		u32 index,
		u32 count,
		u32 sum,
		u64 sum_squared,
		u32 min,
		u32 max
		),

	TP_ARGS(
		name,
		index,
		count,
		sum,
		sum_squared,
		min,
		max
		),

	TP_STRUCT__entry(
		__field(const char *, name)
		__field(u32, index)
		__field(u32, count)
		__field(u32, sum)
		__field(u64, sum_squared)
		__field(u32, min)
		__field(u32, max)
		),

	TP_fast_assign(
		__entry->name = name;
		__entry->index = index;
		__entry->count = count;
		__entry->sum = sum;
		__entry->sum_squared = sum_squared;
		__entry->min = min;
		__entry->max = max;
		),

	TP_printk("%s\tindex: %u\tcount: %u\taverage: %u\t"
		"variance: %llu\tminimum: %u\t"
		"maximum: %u",
		__entry->name, __entry->index, __entry->count,
		__entry->sum / __entry->count,
		((u64)__entry->count * __entry->sum_squared -
			(u64)__entry->sum * (u64)__entry->sum)
			/ __entry->count / __entry->count,
		__entry->min, __entry->max)
);

#endif /*  _TRACE_NVHOST_PVA_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../../nvidia/include/trace/events

#define TRACE_INCLUDE_FILE nvhost_pva
/* This part must be outside protection */
#include <trace/define_trace.h>
