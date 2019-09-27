/*
 * camera_common.h
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
#define TRACE_SYSTEM camera_common

#if !defined(_TRACE_CAMERA_COMMON_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_CAMERA_COMMON_H

#include <linux/tracepoint.h>

struct tegra_channel;
struct timespec;

DECLARE_EVENT_CLASS(channel_simple,
	TP_PROTO(const char *name),
	TP_ARGS(name),
	TP_STRUCT__entry(
		__string(name,	name)
	),
	TP_fast_assign(
		__assign_str(name, name);
	),
	TP_printk("%s", __get_str(name))
);

DEFINE_EVENT(channel_simple, tegra_channel_open,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DEFINE_EVENT(channel_simple, tegra_channel_close,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DEFINE_EVENT(channel_simple, tegra_channel_notify_status_callback,
	TP_PROTO(const char *name),
	TP_ARGS(name)
);

DECLARE_EVENT_CLASS(channel,
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

DEFINE_EVENT(channel, tegra_channel_set_stream,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(channel, csi_s_stream,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(channel, tegra_channel_set_power,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(channel, camera_common_s_power,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

DEFINE_EVENT(channel, csi_s_power,
	TP_PROTO(const char *name, int num),
	TP_ARGS(name, num)
);

TRACE_EVENT(tegra_channel_capture_setup,
	TP_PROTO(struct tegra_channel *chan, unsigned int index),
	TP_ARGS(chan, index),
	TP_STRUCT__entry(
		__field(unsigned int,	vnc_id)
		__field(unsigned int,	width)
		__field(unsigned int,	height)
		__field(unsigned int,	format)
	),
	TP_fast_assign(
		__entry->vnc_id = chan->vnc_id[index];
		__entry->width = chan->format.width;
		__entry->height = chan->format.height;
		__entry->format = chan->fmtinfo->img_fmt;
	),
	TP_printk("vnc_id %u W %u H %u fmt %x",
		  __entry->vnc_id, __entry->width, __entry->height,
		  __entry->format)
);

DECLARE_EVENT_CLASS(frame,
	TP_PROTO(const char *str, struct timespec ts),
	TP_ARGS(str, ts),
	TP_STRUCT__entry(
		__string(str,	str)
		__field(long,	tv_sec)
		__field(long,	tv_nsec)
	),
	TP_fast_assign(
		__assign_str(str, str);
		__entry->tv_sec = ts.tv_sec;
		__entry->tv_nsec = ts.tv_nsec;
	),
	TP_printk("%s:%ld.%ld", __get_str(str), __entry->tv_sec,
		  __entry->tv_nsec)
);

DEFINE_EVENT(frame, tegra_channel_capture_frame,
	TP_PROTO(const char *str, struct timespec ts),
	TP_ARGS(str, ts)
);

DEFINE_EVENT(frame, tegra_channel_capture_done,
	TP_PROTO(const char *str, struct timespec ts),
	TP_ARGS(str, ts)
);
#endif

/* This part must be outside protection */
#include <trace/define_trace.h>
