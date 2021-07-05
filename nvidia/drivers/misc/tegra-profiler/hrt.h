/*
 * drivers/misc/tegra-profiler/hrt.h
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __QUADD_HRT_H
#define __QUADD_HRT_H

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/hrtimer.h>
#include <linux/limits.h>

#include "backtrace.h"

struct quadd_thread_data {
	pid_t pid;
	pid_t tgid;
};

struct quadd_cpu_context {
	struct hrtimer hrtimer;

	struct quadd_callchain cc;
	char mmap_filename[PATH_MAX];

	struct quadd_thread_data active_thread;
	unsigned int is_sampling_enabled:1;
	unsigned int is_tracing_enabled:1;
};

static inline int hrt_is_active(struct quadd_cpu_context *cpu_ctx)
{
	return cpu_ctx->is_sampling_enabled || cpu_ctx->is_tracing_enabled;
}

struct timecounter;

struct quadd_hrt_ctx {
	struct quadd_cpu_context __percpu *cpu_ctx;

	u64 sample_period;
	unsigned long low_addr;

	struct quadd_ctx *quadd_ctx;

	atomic_t active;
	atomic_t mmap_active;

	atomic64_t counter_samples;
	atomic64_t skipped_samples;

	struct timer_list ma_timer;
	unsigned int ma_period;

	unsigned long vm_size_prev;
	unsigned long rss_size_prev;

	pid_t root_pid;
	struct list_head pid_list;
	raw_spinlock_t pid_list_lock;

	struct timecounter *tc;
	unsigned int use_arch_timer:1;
	unsigned int arch_timer_user_access:1;

	struct quadd_unw_methods um;
	unsigned int get_stack_offset:1;
};

struct task_struct;
struct pt_regs;

struct quadd_event_context {
	struct task_struct *task;
	struct pt_regs *regs;

	unsigned int user_mode:1;
	unsigned int is_sched:1;
};

#define QUADD_HRT_MIN_FREQ	100

struct quadd_record_data;
struct quadd_module_state;
struct quadd_iovec;

struct quadd_hrt_ctx *quadd_hrt_init(struct quadd_ctx *ctx);
void quadd_hrt_deinit(void);

int quadd_hrt_start(void);
void quadd_hrt_stop(void);

void
quadd_put_sample_this_cpu(struct quadd_record_data *data,
			  struct quadd_iovec *vec, int vec_count);
void
quadd_put_sample(struct quadd_record_data *data,
		 struct quadd_iovec *vec, int vec_count);

void quadd_hrt_get_state(struct quadd_module_state *state);
u64 quadd_get_time(void);
bool quadd_is_inherited(struct task_struct *task);

#endif	/* __KERNEL__ */

#endif	/* __QUADD_HRT_H */
