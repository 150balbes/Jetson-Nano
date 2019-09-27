/*
 * include/trace/events/atrace.h
 *
 * atrace specific events
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM atrace

#if !defined(_TRACE_ATRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_ATRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(async_atrace_begin,

	TP_PROTO(const char *trace_str, int trace_id, int tracing_cookie),

	TP_ARGS(trace_str, trace_id, tracing_cookie),

	TP_STRUCT__entry(
		__field(const char *, trace_str)
		__field(int, trace_id)
		__field(int, tracing_cookie)
	),

	TP_fast_assign(
		__entry->trace_str = trace_str;
		__entry->trace_id = trace_id;
		__entry->tracing_cookie = tracing_cookie;
	),

	TP_printk("tracing_mark_write: S|%d|%s|%d\n", __entry->trace_id,
		__entry->trace_str, __entry->tracing_cookie)
);

TRACE_EVENT(async_atrace_end,

	TP_PROTO(const char *trace_str, int trace_id, int tracing_cookie),

	TP_ARGS(trace_str, trace_id, tracing_cookie),

	TP_STRUCT__entry(
		__field(const char *, trace_str)
		__field(int, trace_id)
		__field(int, tracing_cookie)
	),

	TP_fast_assign(
		__entry->trace_str = trace_str;
		__entry->trace_id = trace_id;
		__entry->tracing_cookie = tracing_cookie;
	),

	TP_printk("tracing_mark_write: F|%d|%s|%d\n", __entry->trace_id,
		__entry->trace_str, __entry->tracing_cookie)
);

TRACE_EVENT(atrace_start,

	TP_PROTO(const char *trace_str, int trace_id),

	TP_ARGS(trace_str, trace_id),

	TP_STRUCT__entry(
		__field(const char *, trace_str)
		__field(int, trace_id)
	),

	TP_fast_assign(
		__entry->trace_str = trace_str;
		__entry->trace_id = trace_id;
	),

	TP_printk("tracing_mark_write: B|%d|%s\n", __entry->trace_id,
		__entry->trace_str)
);

TRACE_EVENT(atrace_end,

	TP_PROTO(const char *trace_str),

	TP_ARGS(trace_str),

	TP_STRUCT__entry(
		__field(const char *, trace_str)
	),

	TP_fast_assign(
		__entry->trace_str = trace_str;
	),

	TP_printk("tracing_mark_write: E|%s\n", __entry->trace_str)
);


TRACE_EVENT(atrace_counter,

	TP_PROTO(const char *trace_str, int trace_id, u64 trace_count),

	TP_ARGS(trace_str, trace_id, trace_count),

	TP_STRUCT__entry(
		__field(const char *, trace_str)
		__field(int, trace_id)
		__field(u64, trace_count)
	),

	TP_fast_assign(
		__entry->trace_str = trace_str;
		__entry->trace_id = trace_id;
		__entry->trace_count = trace_count;
	),

	TP_printk("tracing_mark_write: C|%d|%s|%lld\n", __entry->trace_id,
		__entry->trace_str, __entry->trace_count)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
