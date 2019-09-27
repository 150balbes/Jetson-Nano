/*
 * vivid.h
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM vivid

#if !defined(_TRACE_VIVID_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VIVID_H

#include <linux/tracepoint.h>

TRACE_EVENT(vivid_buffer_flow,
	TP_PROTO(const char *name, const char *msg, int num),
	TP_ARGS(name, msg, num),
	TP_STRUCT__entry(
		__string(name,	name)
		__string(msg,	msg)
		__field(int,	num)
	),
	TP_fast_assign(
		__assign_str(name, name);
		__assign_str(msg, msg);
		__entry->num = num;
	),
	TP_printk("%s:%s: buffer index %d", __get_str(name),
		__get_str(msg), (int)__entry->num)
);

TRACE_EVENT(vivid_timer,
	TP_PROTO(const char *name, const char *msg,
		unsigned int var1, unsigned int var2),
	TP_ARGS(name, msg, var1, var2),
	TP_STRUCT__entry(
		__string(name,	name)
		__string(msg,	msg)
		__field(unsigned int,	var1)
		__field(unsigned int,	var2)
	),
	TP_fast_assign(
		__assign_str(name, name);
		__assign_str(msg, msg);
		__entry->var1 = var1;
		__entry->var2 = var2;
	),
	TP_printk("%s:%s: wait %u: release %u",
		__get_str(name), __get_str(msg),
		__entry->var1, __entry->var2)
);

TRACE_EVENT(vivid_copybuf,
	TP_PROTO(const char *name, const char *msg1, unsigned int var1,
		const char *msg2, unsigned int var2),
	TP_ARGS(name, msg1, var1, msg2, var2),
	TP_STRUCT__entry(
		__string(name,	name)
		__string(msg1,	msg1)
		__field(unsigned int,	var1)
		__string(msg2,	msg2)
		__field(unsigned int,	var2)
	),
	TP_fast_assign(
		__assign_str(name, name);
		__assign_str(msg1, msg1);
		__entry->var1 = var1;
		__assign_str(msg2, msg2);
		__entry->var2 = var2;
	),
	TP_printk("%s:%s: buf %u: %s: buf %u",
		__get_str(name), __get_str(msg1), __entry->var1,
		__get_str(msg2), __entry->var2)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
