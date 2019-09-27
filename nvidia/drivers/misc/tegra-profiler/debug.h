/*
 * drivers/misc/tegra-profiler/debug.h
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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
 */

#ifndef __QUADD_DEBUG_H
#define __QUADD_DEBUG_H

#include <linux/tegra_profiler.h>

/* #define QM_DEBUG_SAMPLES_ENABLE 1 */

#ifdef QM_DEBUG_SAMPLES_ENABLE
void qm_debug_handler_sample(struct pt_regs *regs);
void qm_debug_timer_forward(struct pt_regs *regs, u64 period);
void qm_debug_timer_start(struct pt_regs *regs, u64 period);
void qm_debug_timer_cancel(void);
void qm_debug_task_sched_in(pid_t prev_pid, pid_t current_pid,
			    int prev_nr_active);
void qm_debug_read_counter(struct quadd_event *event, u32 prev_val, u32 val);
void qm_debug_start_source(int source_type);
void qm_debug_stop_source(int source_type);
#else
static inline void qm_debug_handler_sample(struct pt_regs *regs)
{
}
static inline void qm_debug_timer_forward(struct pt_regs *regs, u64 period)
{
}
static inline void qm_debug_timer_start(struct pt_regs *regs, u64 period)
{
}
static inline void qm_debug_timer_cancel(void)
{
}
static inline void
qm_debug_task_sched_in(pid_t prev_pid, pid_t current_pid, int prev_nr_active)
{
}
static inline void
qm_debug_read_counter(struct quadd_event *event, u32 prev_val, u32 val)
{
}
static inline void qm_debug_start_source(int source_type)
{
}
static inline void qm_debug_stop_source(int source_type)
{
}
#endif

void quadd_test_delay(void);

#define QM_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static inline const char *
quadd_get_hw_event_str(int event)
{
	static const char * const str[] = {
		[QUADD_EVENT_HW_CPU_CYCLES]		= "cpu-cycles",

		[QUADD_EVENT_HW_INSTRUCTIONS]		= "instructions",
		[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS]	= "branch_instruction",
		[QUADD_EVENT_HW_BRANCH_MISSES]		= "branch_misses",
		[QUADD_EVENT_HW_BUS_CYCLES]		= "bus-cycles",

		[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES]	= "l1_d_read",
		[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES]	= "l1_d_write",
		[QUADD_EVENT_HW_L1_ICACHE_MISSES]	= "l1_i",

		[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES]	= "l2_d_read",
		[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES]	= "l2_d_write",
		[QUADD_EVENT_HW_L2_ICACHE_MISSES]	= "l2_i",
	};

	if (event >= QM_ARRAY_SIZE(str))
		return "invalid event";

	speculation_barrier();
	return str[event];
}

#endif	/* __QUADD_DEBUG_H */
