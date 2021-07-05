/*
 * drivers/misc/tegra-profiler/hrt.c
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/rculist.h>
#include <linux/random.h>
#include <clocksource/arm_arch_timer.h>

#include <asm/cputype.h>
#include <asm/irq_regs.h>
#include <asm/arch_timer.h>

#include <linux/tegra_profiler.h>

#include "quadd.h"
#include "hrt.h"
#include "comm.h"
#include "mmap.h"
#include "ma.h"
#include "power_clk.h"
#include "tegra.h"
#include "debug.h"

static struct quadd_hrt_ctx hrt = {
	.active = ATOMIC_INIT(0),
	.mmap_active = ATOMIC_INIT(0),
};

struct hrt_pid_node {
	struct list_head list;
	struct rcu_head rcu;
	pid_t pid;
};

static void pid_free_rcu(struct rcu_head *head)
{
	struct hrt_pid_node *entry =
		container_of(head, struct hrt_pid_node, rcu);
	kfree(entry);
}

static int pid_list_add(pid_t pid)
{
	struct hrt_pid_node *entry;

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry)
		return -ENOMEM;

	entry->pid = pid;
	INIT_LIST_HEAD(&entry->list);

	raw_spin_lock(&hrt.pid_list_lock);
	list_add_tail_rcu(&entry->list, &hrt.pid_list);
	raw_spin_unlock(&hrt.pid_list_lock);

	return 0;
}

static void pid_list_del(pid_t pid)
{
	struct hrt_pid_node *entry;

	raw_spin_lock(&hrt.pid_list_lock);
	list_for_each_entry(entry, &hrt.pid_list, list) {
		if (entry->pid == pid) {
			list_del_rcu(&entry->list);
			call_rcu(&entry->rcu, pid_free_rcu);
			break;
		}
	}
	raw_spin_unlock(&hrt.pid_list_lock);
}

static void pid_list_clear(void)
{
	struct hrt_pid_node *entry, *next;

	raw_spin_lock(&hrt.pid_list_lock);
	list_for_each_entry_safe(entry, next, &hrt.pid_list, list) {
		list_del_rcu(&entry->list);
		call_rcu(&entry->rcu, pid_free_rcu);
	}
	raw_spin_unlock(&hrt.pid_list_lock);
}

static int pid_list_search(pid_t pid)
{
	struct hrt_pid_node *entry;

	/* The possible PID wrapping around: should we somehow handle this? */
	rcu_read_lock();
	list_for_each_entry_rcu(entry, &hrt.pid_list, list) {
		if (entry->pid == pid) {
			rcu_read_unlock();
			return 1;
		}
	}
	rcu_read_unlock();

	return 0;
}

static inline u32 get_task_state(struct task_struct *task)
{
	return (u32)(task->state | task->exit_state);
}

static inline u64 get_posix_clock_monotonic_time(void)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static inline u64 get_arch_time(struct timecounter *tc)
{
	u64 frac = 0;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	u64 value;
#else
	cycle_t value;
#endif
	const struct cyclecounter *cc = tc->cc;

	value = cc->read(cc);
	return cyclecounter_cyc2ns(cc, value, 0, &frac);
}

u64 quadd_get_time(void)
{
	struct timecounter *tc = hrt.tc;

	return (tc && hrt.use_arch_timer) ?
		get_arch_time(tc) :
		get_posix_clock_monotonic_time();
}

static void
__put_sample(struct quadd_record_data *data,
	     struct quadd_iovec *vec,
	     int vec_count, int cpu_id)
{
	ssize_t err;
	struct quadd_comm_data_interface *comm = hrt.quadd_ctx->comm;

	err = comm->put_sample(data, vec, vec_count, cpu_id);
	if (err < 0)
		atomic64_inc(&hrt.skipped_samples);

	atomic64_inc(&hrt.counter_samples);
}

void
quadd_put_sample_this_cpu(struct quadd_record_data *data,
			  struct quadd_iovec *vec, int vec_count)
{
	__put_sample(data, vec, vec_count, -1);
}

void
quadd_put_sample(struct quadd_record_data *data,
		 struct quadd_iovec *vec, int vec_count)
{
	__put_sample(data, vec, vec_count, 0);
}

static void put_header(int cpuid, bool is_uncore)
{
	int vec_idx = 0;
	u32 uncore_freq;
	struct quadd_iovec vec[2];
	int nr_events = 0, max_events = QUADD_MAX_COUNTERS;
	struct quadd_event events[QUADD_MAX_COUNTERS];
	struct quadd_record_data record;
	struct quadd_header_data *hdr = &record.hdr;
	struct quadd_parameters *param = &hrt.quadd_ctx->param;
	unsigned int extra = param->reserved[QUADD_PARAM_IDX_EXTRA];
	struct quadd_ctx *ctx = hrt.quadd_ctx;
	struct quadd_event_source *pmu = ctx->pmu;
	struct quadd_event_source *carmel_pmu = ctx->carmel_pmu;

	hdr->time = quadd_get_time();

	record.record_type = QUADD_RECORD_TYPE_HEADER;

	hdr->magic = QUADD_HEADER_MAGIC;
	hdr->samples_version = QUADD_SAMPLES_VERSION;
	hdr->io_version = QUADD_IO_VERSION;

	hdr->flags = 0;

	if (param->backtrace)
		hdr->flags |= QUADD_HDR_FLAG_BACKTRACE;
	if (param->use_freq)
		hdr->flags |= QUADD_HDR_FLAG_USE_FREQ;

#ifdef QM_DEBUG_SAMPLES_ENABLE
	hdr->flags |= QUADD_HDR_FLAG_DEBUG_SAMPLES;
#endif

	hdr->freq = param->freq;
	hdr->ma_freq = param->ma_freq;
	hdr->power_rate_freq = param->power_rate_freq;

	if (hdr->power_rate_freq > 0)
		hdr->flags |= QUADD_HDR_FLAG_POWER_RATE;
	if (extra & QUADD_PARAM_EXTRA_GET_MMAP)
		hdr->flags |= QUADD_HDR_FLAG_GET_MMAP;

	hdr->extra_length = 0;

	if (param->backtrace) {
		struct quadd_unw_methods *um = &hrt.um;

		if (um->fp)
			hdr->flags |= QUADD_HDR_FLAG_BT_FP;
		if (um->ut)
			hdr->flags |= QUADD_HDR_FLAG_BT_UT;
		if (um->ut_ce)
			hdr->flags |= QUADD_HDR_FLAG_BT_UT_CE;
		if (um->dwarf)
			hdr->flags |= QUADD_HDR_FLAG_BT_DWARF;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0))
	if (hrt.use_arch_timer)
		hdr->flags |= QUADD_HDR_FLAG_USE_ARCH_TIMER;
#endif

	if (hrt.get_stack_offset)
		hdr->flags |= QUADD_HDR_FLAG_STACK_OFFSET;

	hdr->flags |= QUADD_HDR_FLAG_HAS_CPUID;

	if (quadd_mode_is_sampling(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_SAMPLING;
	if (quadd_mode_is_tracing(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_TRACING;
	if (quadd_mode_is_sample_all(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_SAMPLE_ALL;
	if (quadd_mode_is_trace_all(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_TRACE_ALL;
	if (quadd_mode_is_sample_tree(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_SAMPLE_TREE;
	if (quadd_mode_is_trace_tree(ctx))
		hdr->flags |= QUADD_HDR_FLAG_MODE_TRACE_TREE;

	if (ctx->pclk_cpufreq)
		hdr->flags |= QUADD_HDR_FLAG_CPUFREQ;

	if (is_uncore) {
		hdr->cpu_id = U8_MAX;
		hdr->flags |= QUADD_HDR_FLAG_UNCORE;

		uncore_freq = param->reserved[QUADD_PARAM_IDX_UNCORE_FREQ];

		if (carmel_pmu)
			nr_events =
				carmel_pmu->current_events(cpuid, events,
							   max_events);
	} else {
		hdr->cpu_id = cpuid;
		if (pmu)
			nr_events =
				pmu->current_events(cpuid, events, max_events);
	}

	hdr->nr_events = nr_events;

	vec[vec_idx].base = events;
	vec[vec_idx].len = nr_events * sizeof(events[0]);
	vec_idx++;

	if (is_uncore) {
		vec[vec_idx].base = &uncore_freq;
		vec[vec_idx].len = sizeof(uncore_freq);
		vec_idx++;
	}

	__put_sample(&record, &vec[0], vec_idx, cpuid);
}

static void
put_sched_sample(struct task_struct *task, bool is_sched_in, u64 ts)
{
	int vec_idx = 0;
	u32 vpid, vtgid;
	unsigned int flags;
	struct quadd_iovec vec[2];
	struct quadd_record_data record;
	struct quadd_sched_data *s = &record.sched;

	s->time = ts;
	s->flags = 0;
	s->cpu_id = quadd_get_processor_id(NULL, &flags);

	record.record_type = QUADD_RECORD_TYPE_SCHED;

	if (flags & QUADD_CPUMODE_TEGRA_POWER_CLUSTER_LP)
		s->flags |= QUADD_SCHED_FLAG_LP_MODE;
	if (is_sched_in)
		s->flags |= QUADD_SCHED_FLAG_SCHED_IN;
	if (task->flags & PF_KTHREAD)
		s->flags |= QUADD_SCHED_FLAG_PF_KTHREAD;

	s->pid = task_pid_nr(task);
	s->tgid = task_tgid_nr(task);

	s->task_state = get_task_state(task);

	if (!(task->flags & PF_EXITING)) {
		vpid = task_pid_vnr(task);
		vtgid = task_tgid_vnr(task);

		if (s->pid != vpid || s->tgid != vtgid) {
			vec[vec_idx].base = &vpid;
			vec[vec_idx].len = sizeof(vpid);
			vec_idx++;

			vec[vec_idx].base = &vtgid;
			vec[vec_idx].len = sizeof(vtgid);
			vec_idx++;

			s->flags |= QUADD_SCHED_FLAG_IS_VPID;
		}
	}

	quadd_put_sample_this_cpu(&record, vec, vec_idx);
}

static void put_comm_sample(struct task_struct *task, bool exec)
{
	char name[TASK_COMM_LEN];
	struct quadd_iovec vec;
	struct quadd_record_data record;
	struct quadd_comm_data *s = &record.comm;

	s->time = quadd_get_time();
	s->flags = 0;

	s->pid = (u32)task_pid_nr(task);
	s->tgid = (u32)task_tgid_nr(task);

	memset(name, 0, sizeof(name));
	get_task_comm(name, task);
	name[sizeof(name) - 1] = '\0';

	record.record_type = QUADD_RECORD_TYPE_COMM;

	if (exec)
		s->flags |= QUADD_COMM_FLAG_EXEC;

	vec.base = name;
	vec.len = ALIGN(strlen(name) + 1, sizeof(u64));

	s->length = (u16)vec.len;

	quadd_put_sample(&record, &vec, 1);
}

static int get_sample_data(struct quadd_sample_data *s,
			   struct pt_regs *regs,
			   struct task_struct *task)
{
	unsigned int flags, user_mode;
	struct quadd_ctx *quadd_ctx = hrt.quadd_ctx;

	s->cpu_id = quadd_get_processor_id(regs, &flags);

	user_mode = user_mode(regs);

	if (user_mode)
		s->flags |= QUADD_SAMPLE_FLAG_USER_MODE;
	if (flags & QUADD_CPUMODE_TEGRA_POWER_CLUSTER_LP)
		s->flags |= QUADD_SAMPLE_FLAG_LP_MODE;
	if (flags & QUADD_CPUMODE_THUMB)
		s->flags |= QUADD_SAMPLE_FLAG_THUMB_MODE;
	if (in_interrupt())
		s->flags |= QUADD_SAMPLE_FLAG_IN_INTERRUPT;
	if (task->flags & PF_KTHREAD)
		s->flags |= QUADD_SCHED_FLAG_PF_KTHREAD;

	/* For security reasons, hide IPs from the kernel space. */
	if (!user_mode && !quadd_ctx->collect_kernel_ips)
		s->ip = 0;
	else
		s->ip = instruction_pointer(regs);

	s->pid = task_pid_nr(task);
	s->tgid = task_tgid_nr(task);

	return 0;
}

static long
get_stack_offset(struct task_struct *task,
		 struct pt_regs *regs,
		 struct quadd_callchain *cc)
{
	unsigned long sp;
	struct vm_area_struct *vma;
	struct mm_struct *mm = task->mm;

	if (!regs || !mm)
		return -ENOMEM;

	sp = cc->nr > 0 ? cc->curr_sp :
		quadd_user_stack_pointer(regs);

	vma = find_vma(mm, sp);
	if (!vma)
		return -ENOMEM;

	return vma->vm_end - sp;
}

static void
read_all_sources(struct pt_regs *regs, struct task_struct *task, u64 ts)
{
	u32 vpid, vtgid;
	u32 state, extra_data = 0, urcs = 0, ts_delta;
	u64 ts_start, ts_end;
	int i, vec_idx = 0, bt_size = 0;
	int nr_events = 0, nr_positive_events = 0;
	struct pt_regs *user_regs;
	struct quadd_iovec vec[9];
	struct quadd_event_data events[QUADD_MAX_COUNTERS];
	u32 events_extra[QUADD_MAX_COUNTERS];
	struct quadd_event_context event_ctx;

	struct quadd_record_data record_data;
	struct quadd_sample_data *s = &record_data.sample;

	struct quadd_ctx *ctx = hrt.quadd_ctx;
	struct quadd_cpu_context *cpu_ctx = this_cpu_ptr(hrt.cpu_ctx);
	struct quadd_callchain *cc = &cpu_ctx->cc;

	if (!hrt_is_active(cpu_ctx))
		return;

	if (task->flags & PF_EXITING)
		return;

	s->time = ts_start = ts;
	s->flags = 0;

	if (ctx->pmu && ctx->get_pmu_info()->active)
		nr_events = ctx->pmu->read(events, ARRAY_SIZE(events));

	if (!nr_events)
		return;

	if (user_mode(regs))
		user_regs = regs;
	else
		user_regs = current_pt_regs();

	if (get_sample_data(s, regs, task))
		return;

	vec[vec_idx].base = &extra_data;
	vec[vec_idx].len = sizeof(extra_data);
	vec_idx++;

	cc->nr = 0;

	event_ctx.regs = user_regs;
	event_ctx.task = task;
	event_ctx.user_mode = user_mode(regs);
	event_ctx.is_sched = !in_interrupt();

	if (ctx->param.backtrace) {
		cc->um = hrt.um;

		bt_size = quadd_get_user_callchain(&event_ctx, cc, ctx);
		if (bt_size > 0) {
			int ip_size = cc->cs_64 ? sizeof(u64) : sizeof(u32);
			int nr_types = DIV_ROUND_UP(bt_size, 8);

			vec[vec_idx].base = cc->cs_64 ?
				(void *)cc->ip_64 : (void *)cc->ip_32;
			vec[vec_idx].len = bt_size * ip_size;
			vec_idx++;

			vec[vec_idx].base = cc->types;
			vec[vec_idx].len = nr_types * sizeof(cc->types[0]);
			vec_idx++;

			if (cc->cs_64)
				s->flags |= QUADD_SAMPLE_FLAG_IP64;
		}

		urcs |= (cc->urc_fp & QUADD_SAMPLE_URC_MASK) <<
			QUADD_SAMPLE_URC_SHIFT_FP;
		urcs |= (cc->urc_ut & QUADD_SAMPLE_URC_MASK) <<
			QUADD_SAMPLE_URC_SHIFT_UT;
		urcs |= (cc->urc_dwarf & QUADD_SAMPLE_URC_MASK) <<
			QUADD_SAMPLE_URC_SHIFT_DWARF;

		s->flags |= QUADD_SAMPLE_FLAG_URCS;

		vec[vec_idx].base = &urcs;
		vec[vec_idx].len = sizeof(urcs);
		vec_idx++;
	}
	s->callchain_nr = bt_size;

	if (hrt.get_stack_offset) {
		long offset = get_stack_offset(task, user_regs, cc);

		if (offset > 0) {
			u32 off = offset >> 2;

			off = min_t(u32, off, 0xffff);
			extra_data |= off << QUADD_SED_STACK_OFFSET_SHIFT;
		}
	}

	record_data.record_type = QUADD_RECORD_TYPE_SAMPLE;

	s->events_flags = 0;
	for (i = 0; i < nr_events; i++) {
		u32 value = (u32)events[i].delta;

		if (value > 0) {
			s->events_flags |= 1 << i;
			events_extra[nr_positive_events++] = value;
		}
	}

	if (nr_positive_events == 0)
		return;

	vec[vec_idx].base = events_extra;
	vec[vec_idx].len = nr_positive_events * sizeof(events_extra[0]);
	vec_idx++;

	state = get_task_state(task);
	if (state) {
		s->flags |= QUADD_SAMPLE_FLAG_STATE;
		vec[vec_idx].base = &state;
		vec[vec_idx].len = sizeof(state);
		vec_idx++;
	}

	ts_end = quadd_get_time();
	ts_delta = (u32)(ts_end - ts_start);

	vec[vec_idx].base = &ts_delta;
	vec[vec_idx].len = sizeof(ts_delta);
	vec_idx++;

	vpid = task_pid_vnr(task);
	vtgid = task_tgid_vnr(task);

	if (s->pid != vpid || s->tgid != vtgid) {
		vec[vec_idx].base = &vpid;
		vec[vec_idx].len = sizeof(vpid);
		vec_idx++;

		vec[vec_idx].base = &vtgid;
		vec[vec_idx].len = sizeof(vtgid);
		vec_idx++;

		s->flags |= QUADD_SAMPLE_FLAG_IS_VPID;
	}

	quadd_put_sample_this_cpu(&record_data, vec, vec_idx);
}

static enum hrtimer_restart hrtimer_handler(struct hrtimer *hrtimer)
{
	struct pt_regs *regs;

	regs = get_irq_regs();

	if (!atomic_read(&hrt.active))
		return HRTIMER_NORESTART;

	qm_debug_handler_sample(regs);

	if (regs)
		read_all_sources(regs, current, quadd_get_time());

	hrtimer_forward_now(hrtimer, ns_to_ktime(hrt.sample_period));
	qm_debug_timer_forward(regs, hrt.sample_period);

	return HRTIMER_RESTART;
}

static void start_hrtimer(struct quadd_cpu_context *cpu_ctx)
{
	u32 period = prandom_u32_max(hrt.sample_period);

	hrtimer_start(&cpu_ctx->hrtimer, ns_to_ktime(period),
		      HRTIMER_MODE_REL_PINNED);
	qm_debug_timer_start(NULL, period);
}

static void cancel_hrtimer(struct quadd_cpu_context *cpu_ctx)
{
	hrtimer_cancel(&cpu_ctx->hrtimer);
	qm_debug_timer_cancel();
}

static void init_hrtimer(struct quadd_cpu_context *cpu_ctx)
{
#if (defined(CONFIG_PREEMPT_RT_FULL) && \
		(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)))
	hrtimer_init(&cpu_ctx->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
#else
	hrtimer_init(&cpu_ctx->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#endif
	cpu_ctx->hrtimer.function = hrtimer_handler;
}

static inline bool
__is_profile_process(struct task_struct *task, bool is_trace, bool is_sample)
{
	pid_t root_pid = hrt.root_pid;
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (root_pid > 0 && task_tgid_nr(task) == root_pid)
		return true;

	if ((is_trace && quadd_mode_is_trace_tree(ctx)) ||
	    (is_sample && quadd_mode_is_sample_tree(ctx)))
		return pid_list_search(task_tgid_nr(task));

	return false;
}

static inline bool
validate_task(struct task_struct *task)
{
	return task && !is_idle_task(task);
}

static inline bool
is_sample_process(struct task_struct *task)
{
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (!validate_task(task) || !quadd_mode_is_sampling(ctx))
		return false;

	if (quadd_mode_is_sample_all(ctx))
		return true;

	return __is_profile_process(task, false, true);
}

static inline bool
is_trace_process(struct task_struct *task)
{
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (!validate_task(task) || !quadd_mode_is_tracing(ctx))
		return false;

	if (quadd_mode_is_trace_all(ctx))
		return true;

	return __is_profile_process(task, true, false);
}

static inline bool
is_profile_process(struct task_struct *task)
{
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (!validate_task(task) ||
	    (!quadd_mode_is_tracing(ctx) && !quadd_mode_is_sampling(ctx)))
		return false;

	if (quadd_mode_is_process_all(ctx))
		return true;

	return __is_profile_process(task, true, true);
}

static void
add_active_thread(struct quadd_cpu_context *cpu_ctx, pid_t pid, pid_t tgid)
{
	struct quadd_thread_data *t_data = &cpu_ctx->active_thread;

	if (unlikely(hrt_is_active(cpu_ctx)))
		pr_warn_once("%s: warning: active thread: %d\n",
			     __func__, (int)pid);

	t_data->pid = pid;
	t_data->tgid = tgid;
}

static void
remove_active_thread(struct quadd_cpu_context *cpu_ctx, pid_t pid)
{
	struct quadd_thread_data *t_data = &cpu_ctx->active_thread;

	t_data->pid = -1;
	t_data->tgid = -1;
}

static inline pid_t get_active_thread(struct quadd_cpu_context *cpu_ctx)
{
	struct quadd_thread_data *t_data = &cpu_ctx->active_thread;

	return t_data->pid;
}

static inline int
is_task_active(struct quadd_cpu_context *cpu_ctx, struct task_struct *task)
{
	return task_pid_nr(task) == get_active_thread(cpu_ctx);
}

void __quadd_task_sched_in(struct task_struct *prev,
			   struct task_struct *task)
{
	bool trace_flag, sample_flag;
	struct quadd_cpu_context *cpu_ctx = this_cpu_ptr(hrt.cpu_ctx);
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (likely(!atomic_read(&hrt.active)))
		return;

	sample_flag = is_sample_process(task);
	trace_flag = is_trace_process(task);

	if (sample_flag || trace_flag)
		add_active_thread(cpu_ctx, task->pid, task->tgid);

	if (trace_flag) {
		put_sched_sample(task, true, quadd_get_time());
		cpu_ctx->is_tracing_enabled = 1;
	}

	if (sample_flag) {
		if (likely(!cpu_ctx->is_sampling_enabled)) {
			if (ctx->pmu)
				ctx->pmu->start();

			if (quadd_mode_is_sampling_timer(ctx))
				start_hrtimer(cpu_ctx);

			cpu_ctx->is_sampling_enabled = 1;
		} else
			pr_warn_once("warning: sampling is already enabled\n");
	}
}

void __quadd_task_sched_out(struct task_struct *prev,
			    struct task_struct *next)
{
	u64 ts;
	struct pt_regs *user_regs;
	struct quadd_cpu_context *cpu_ctx = this_cpu_ptr(hrt.cpu_ctx);
	struct quadd_ctx *ctx = hrt.quadd_ctx;

	if (likely(!atomic_read(&hrt.active)))
		return;

	ts = quadd_get_time();

	if (quadd_mode_is_sampling(ctx) && cpu_ctx->is_sampling_enabled) {
		if (quadd_mode_is_sampling_sched(ctx) &&
		    is_task_active(cpu_ctx, prev)) {
			user_regs = task_pt_regs(prev);
			if (user_regs)
				read_all_sources(user_regs, prev, ts);
		}

		if (quadd_mode_is_sampling_timer(ctx))
			cancel_hrtimer(cpu_ctx);

		if (ctx->pmu)
			ctx->pmu->stop();

		cpu_ctx->is_sampling_enabled = 0;
	}

	if (quadd_mode_is_tracing(ctx) && cpu_ctx->is_tracing_enabled) {
		if (is_task_active(cpu_ctx, prev))
			put_sched_sample(prev, false, ts);
		cpu_ctx->is_tracing_enabled = 0;
	}

	remove_active_thread(cpu_ctx, prev->pid);
}

void __quadd_event_mmap(struct vm_area_struct *vma)
{
	if (likely(!atomic_read(&hrt.mmap_active)))
		return;

	if (!is_sample_process(current))
		return;

	quadd_process_mmap(vma, current);
}

bool quadd_is_inherited(struct task_struct *task)
{
	struct task_struct *p;

	if (unlikely(hrt.root_pid == 0))
		return false;

	for (p = task; p != &init_task;) {
		if (task_tgid_nr(p) == hrt.root_pid)
			return true;

		rcu_read_lock();
		p = rcu_dereference(p->real_parent);
		rcu_read_unlock();
	}

	return false;
}

void __quadd_event_fork(struct task_struct *task)
{
	pid_t tgid;

	if (likely(!atomic_read(&hrt.mmap_active)))
		return;

	if (!quadd_mode_is_process_tree(hrt.quadd_ctx))
		return;

	tgid = task_tgid_nr(task);
	if (pid_list_search(tgid))
		return;

	read_lock(&tasklist_lock);
	if (quadd_is_inherited(task)) {
		quadd_get_task_mmaps(hrt.quadd_ctx, task);
		pid_list_add(tgid);
	}
	read_unlock(&tasklist_lock);
}

void __quadd_event_exit(struct task_struct *task)
{
	pid_t tgid;

	if (likely(!atomic_read(&hrt.mmap_active)))
		return;

	if (!quadd_mode_is_process_tree(hrt.quadd_ctx))
		return;

	tgid = task_tgid_nr(task);
	if (!pid_list_search(tgid))
		return;

	read_lock(&tasklist_lock);
	if (quadd_is_inherited(task))
		pid_list_del(tgid);
	read_unlock(&tasklist_lock);
}

void __quadd_event_comm(struct task_struct *task, bool exec)
{
	if (likely(!atomic_read(&hrt.active)))
		return;

	if (!is_profile_process(task))
		return;

	put_comm_sample(task, exec);
}

static void reset_cpu_ctx(void)
{
	int cpu_id;
	struct quadd_cpu_context *cpu_ctx;
	struct quadd_thread_data *t_data;

	for_each_possible_cpu(cpu_id) {
		cpu_ctx = per_cpu_ptr(hrt.cpu_ctx, cpu_id);
		t_data = &cpu_ctx->active_thread;

		cpu_ctx->is_sampling_enabled = 0;
		cpu_ctx->is_tracing_enabled = 0;

		t_data->pid = -1;
		t_data->tgid = -1;
	}
}

static void get_initial_samples(struct quadd_ctx *ctx)
{
	struct task_struct *p, *t;

	if (quadd_mode_is_sampling(ctx))
		quadd_get_mmaps(ctx);

	if (quadd_mode_is_process_all(ctx)) {
		bool is_tree = quadd_mode_is_process_tree(ctx);

		read_lock(&tasklist_lock);
		for_each_process(p) {
			for_each_thread(p, t)
				put_comm_sample(t, false);

			if (is_tree && quadd_is_inherited(p))
				pid_list_add(task_pid_nr(p));
		}
		read_unlock(&tasklist_lock);
	} else if (quadd_mode_is_process_tree(ctx)) {
		read_lock(&tasklist_lock);
		for_each_process(p) {
			if (quadd_is_inherited(p)) {
				pid_list_add(task_pid_nr(p));
				for_each_thread(p, t)
					put_comm_sample(t, false);
			}
		}
		read_unlock(&tasklist_lock);
	} else {
		pid_t root_pid = hrt.root_pid;

		if (root_pid > 0) {
			read_lock(&tasklist_lock);
			p = get_pid_task(find_vpid(root_pid), PIDTYPE_PID);
			if (p) {
				for_each_thread(p, t)
					put_comm_sample(t, false);
				put_task_struct(p);
			}
			read_unlock(&tasklist_lock);
		}
	}
}

int quadd_hrt_start(void)
{
	int cpuid;
	u64 period;
	long freq;
	unsigned int extra;
	struct quadd_ctx *ctx = hrt.quadd_ctx;
	struct quadd_parameters *param = &ctx->param;

	freq = ctx->param.freq;
	freq = max_t(long, QUADD_HRT_MIN_FREQ, freq);
	period = NSEC_PER_SEC / freq;
	hrt.sample_period = period;
	hrt.root_pid = param->nr_pids > 0 ? param->pids[0] : 0;

	if (ctx->param.ma_freq > 0)
		hrt.ma_period = MSEC_PER_SEC / ctx->param.ma_freq;
	else
		hrt.ma_period = 0;

	atomic64_set(&hrt.counter_samples, 0);
	atomic64_set(&hrt.skipped_samples, 0);

	reset_cpu_ctx();

	extra = param->reserved[QUADD_PARAM_IDX_EXTRA];

	if (param->backtrace) {
		struct quadd_unw_methods *um = &hrt.um;

		um->fp = extra & QUADD_PARAM_EXTRA_BT_FP ? 1 : 0;
		um->ut = extra & QUADD_PARAM_EXTRA_BT_UT ? 1 : 0;
		um->ut_ce = extra & QUADD_PARAM_EXTRA_BT_UT_CE ? 1 : 0;
		um->dwarf = extra & QUADD_PARAM_EXTRA_BT_DWARF ? 1 : 0;

		pr_info("unw methods: fp/ut/ut_ce/dwarf: %u/%u/%u/%u\n",
			um->fp, um->ut, um->ut_ce, um->dwarf);
	}

	if (hrt.tc && (extra & QUADD_PARAM_EXTRA_USE_ARCH_TIMER) &&
	    (hrt.arch_timer_user_access ||
	     (extra & QUADD_PARAM_EXTRA_FORCE_ARCH_TIMER)))
		hrt.use_arch_timer = 1;
	else
		hrt.use_arch_timer = 0;

	pr_info("timer: %s\n", hrt.use_arch_timer ? "arch" : "monotonic clock");

	hrt.get_stack_offset =
		(extra & QUADD_PARAM_EXTRA_STACK_OFFSET) ? 1 : 0;

	for_each_possible_cpu(cpuid) {
		if (ctx->pmu->get_arch(cpuid))
			put_header(cpuid, false);
	}
	put_header(0, true);

	atomic_set(&hrt.mmap_active, 1);

	/* Enable the mmap events processing before quadd_get_mmaps()
	 * otherwise we can miss some events.
	 */
	smp_wmb();

	get_initial_samples(ctx);
	quadd_ma_start(&hrt);

	/* Enable the sampling only after quadd_get_mmaps() */
	smp_wmb();

	atomic_set(&hrt.active, 1);

	pr_info("Start hrt: freq/period: %ld/%llu\n", freq, period);
	return 0;
}

void quadd_hrt_stop(void)
{
	pr_info("Stop hrt, samples all/skipped: %lld/%lld\n",
		(long long)atomic64_read(&hrt.counter_samples),
		(long long)atomic64_read(&hrt.skipped_samples));

	quadd_ma_stop(&hrt);

	atomic_set(&hrt.active, 0);
	atomic_set(&hrt.mmap_active, 0);

	atomic64_set(&hrt.counter_samples, 0);
	atomic64_set(&hrt.skipped_samples, 0);

	pid_list_clear();

	/* reset_cpu_ctx(); */
}

void quadd_hrt_deinit(void)
{
	if (atomic_read(&hrt.active))
		quadd_hrt_stop();

	free_percpu(hrt.cpu_ctx);
}

void quadd_hrt_get_state(struct quadd_module_state *state)
{
	state->nr_all_samples = atomic64_read(&hrt.counter_samples);
	state->nr_skipped_samples = atomic64_read(&hrt.skipped_samples);
}

static void init_arch_timer(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	struct arch_timer_kvm_info *info;
#endif

	u32 cntkctl = arch_timer_get_cntkctl();

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 7, 0)
	info = arch_timer_get_kvm_info();
	hrt.tc = &info->timecounter;
#else
	hrt.tc = arch_timer_get_timecounter();
#endif

	hrt.arch_timer_user_access =
		(cntkctl & ARCH_TIMER_USR_VCT_ACCESS_EN) ? 1 : 0;
}

struct quadd_hrt_ctx *quadd_hrt_init(struct quadd_ctx *ctx)
{
	int cpu_id;
	u64 period;
	long freq;
	struct quadd_cpu_context *cpu_ctx;

	hrt.quadd_ctx = ctx;

	atomic_set(&hrt.active, 0);
	atomic_set(&hrt.mmap_active, 0);

	freq = ctx->param.freq;
	freq = max_t(long, QUADD_HRT_MIN_FREQ, freq);
	period = NSEC_PER_SEC / freq;
	hrt.sample_period = period;
	hrt.root_pid = 0;

	INIT_LIST_HEAD(&hrt.pid_list);
	raw_spin_lock_init(&hrt.pid_list_lock);

	if (ctx->param.ma_freq > 0)
		hrt.ma_period = MSEC_PER_SEC / ctx->param.ma_freq;
	else
		hrt.ma_period = 0;

	atomic64_set(&hrt.counter_samples, 0);
	init_arch_timer();

	hrt.cpu_ctx = alloc_percpu(struct quadd_cpu_context);
	if (!hrt.cpu_ctx)
		return ERR_PTR(-ENOMEM);

	for_each_possible_cpu(cpu_id) {
		cpu_ctx = per_cpu_ptr(hrt.cpu_ctx, cpu_id);

		cpu_ctx->is_sampling_enabled = 0;
		cpu_ctx->is_tracing_enabled = 0;

		cpu_ctx->active_thread.pid = -1;
		cpu_ctx->active_thread.tgid = -1;

		cpu_ctx->cc.hrt = &hrt;

		init_hrtimer(cpu_ctx);
	}

	return &hrt;
}
