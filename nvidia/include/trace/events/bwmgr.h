/*
 * Bwmgr event logging to ftrace.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM bwmgr

#if !defined(_TRACE_BWMGR_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_BWMGR_H

#include <linux/ktime.h>
#include <linux/tracepoint.h>

TRACE_EVENT(tegra_bwmgr_set_emc,
	TP_PROTO(
		const char *handle,
		unsigned long val,
		const char *req
	),

	TP_ARGS(handle, val, req),

	TP_STRUCT__entry(
		__field(const char *, handle)
		__field(unsigned long, val)
		__field(const char *, req)
	),

	TP_fast_assign(
		__entry->handle = handle;
		__entry->val = val;
		__entry->req = req;
	),

	TP_printk("handle=%s, val=%lu, req=%s",
		__entry->handle,
		__entry->val,
		__entry->req
	)
);

TRACE_EVENT(tegra_bwmgr_update_efficiency,
	TP_PROTO(
		unsigned long cur_state,
		unsigned long prev_state
	),

	TP_ARGS(cur_state, prev_state),

	TP_STRUCT__entry(
		__field(unsigned long, cur_state)
		__field(unsigned long, prev_state)
	),

	TP_fast_assign(
		__entry->cur_state = cur_state;
		__entry->prev_state = prev_state;
	),

	TP_printk("bwmgr cooling state changed from (%lu) to (%lu)",
		__entry->prev_state,
		__entry->cur_state
	)
);

#endif /* _TRACE_BWMGR_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
