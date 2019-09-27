/*
 * Copyright (c) 2017 NVIDIA CORPORATION. All rights reserved.
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

#undef TRACE_SYSTEM
#define TRACE_SYSTEM tegra_ivc_rpc

#if !defined(_TRACE_TEGRA_RPC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_TEGRA_RPC_H

#include <linux/tracepoint.h>

TRACE_EVENT(rpc_send_msg,
	TP_PROTO(u32 seq_num, bool is_blocking),
	TP_ARGS(seq_num, is_blocking),
	TP_STRUCT__entry(
		__field(u32, seq_num)
		__field(bool, is_blocking)
	),
	TP_fast_assign(
		__entry->seq_num = seq_num;
		__entry->is_blocking = is_blocking;
	),
	TP_printk("seq_num=%u %s", __entry->seq_num,
		__entry->is_blocking ? "blocking" : "callback")
);

TRACE_EVENT(rpc_timer,
	TP_PROTO(u32 seq_num),
	TP_ARGS(seq_num),
	TP_STRUCT__entry(
		__field(u32, seq_num)
	),
	TP_fast_assign(
		__entry->seq_num = seq_num;
	),
	TP_printk("seq_num:%u", __entry->seq_num)
);

TRACE_EVENT(rpc_callback,
	TP_PROTO(u32 seq_num),
	TP_ARGS(seq_num),
	TP_STRUCT__entry(
		__field(u32, seq_num)
	),
	TP_fast_assign(
		__entry->seq_num = seq_num;
	),
	TP_printk("seq_num:%u", __entry->seq_num)
);

#endif /* _TRACE_TEGRA_RPC_H */

#include <trace/define_trace.h>
