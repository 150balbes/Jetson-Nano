/*
 * ov23850.h
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
#define TRACE_SYSTEM ov23850

#if !defined(_TRACE_OV23850_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_OV23850_H

#include <linux/tracepoint.h>

TRACE_EVENT(ov23850_s_stream,
	TP_PROTO(const char *name, int enable, int mode),
	TP_ARGS(name, enable, mode),
	TP_STRUCT__entry(
		__string(name,	name)
		__field(int,	enable)
		__field(int,	mode)
	),
	TP_fast_assign(
		__assign_str(name, name);
		__entry->enable = enable;
		__entry->mode = mode;
	),
	TP_printk("%s: on %d mode %d", __get_str(name),
		  __entry->enable, __entry->mode)
);


#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
