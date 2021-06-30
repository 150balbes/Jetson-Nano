/*
 * mipi_cal.h
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
#define TRACE_SYSTEM mipical

#if !defined(_TRACE_MIPI_CAL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MIPI_CAL_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(mipi_base,

	TP_PROTO(const char *name, int num),

	TP_ARGS(name, num),

	TP_STRUCT__entry(
		__string(name,	name)
		__field(int,	num)
	),

	TP_fast_assign(
		__assign_str(name, name);
		__entry->num = num;
	),

	TP_printk("%s : 0x%x", __get_str(name), (int)__entry->num)
);

DEFINE_EVENT(mipi_base, pad_enable,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(mipi_base, mipical_result,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(mipi_base, pad_disable,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(mipi_base, mipical,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
