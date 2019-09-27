/*
 * include/trace/events/ina3221.h
 *
 * INA3221 specific channel events.
 *
 * Copyright (c) 2010-2018, NVIDIA CORPORATION.  All rights reserved.
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
#define TRACE_SYSTEM ina3221

#if !defined(_TRACE_INA3221_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_INA3221_H

#include <linux/tracepoint.h>

TRACE_EVENT(ina3221_power,

	TP_PROTO(const char *channel_name, unsigned long channel_power),

	TP_ARGS(channel_name, channel_power),

	TP_STRUCT__entry(
		__string(channel_name, channel_name)
		__field(unsigned long, channel_power)
	),

	TP_fast_assign(
		__assign_str(channel_name, channel_name);
		__entry->channel_power = channel_power;
	),

	TP_printk("channel_name=%s channel_power=%lu\n",
		__get_str(channel_name), __entry->channel_power)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
