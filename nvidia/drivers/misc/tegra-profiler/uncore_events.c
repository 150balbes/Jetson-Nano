/*
 * drivers/misc/tegra-profiler/uncore_events.c
 *
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/version.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>

#include <linux/tegra_profiler.h>

#include "uncore_events.h"
#include "hrt.h"
#include "comm.h"
#include "quadd.h"

enum {
	QUADD_UNCORE_STATE_ACTIVE = 0,
	QUADD_UNCORE_STATE_STOPPING,
	QUADD_UNCORE_STATE_INACTIVE,
};

struct uncore_ctx {
	struct quadd_event_source *carmel_pmu;
	struct source_info *carmel_info;

	u64 sample_period;
	struct hrtimer hrtimer;

	atomic_t state;
	atomic_t ref_count;
	raw_spinlock_t state_lock;

	struct quadd_ctx *quadd_ctx;
};

static struct uncore_ctx ctx = {
	.state = ATOMIC_INIT(QUADD_UNCORE_STATE_INACTIVE),
};

static inline bool is_source_active(struct source_info *si)
{
	return si->active != 0;
}

static inline bool is_uncore_active(void)
{
	return (ctx.carmel_pmu && is_source_active(ctx.carmel_info));
}

static void
put_sample(const struct quadd_event_data *events, int nr_events, u64 ts)
{
	int i, nr_positive = 0, vec_idx = 0;
	struct quadd_iovec vec[3];
	u32 extra_data = 0, ts_delta = 0, events_extra[QUADD_MAX_COUNTERS];
	struct quadd_record_data record;
	struct quadd_sample_data *s = &record.sample;

	record.record_type = QUADD_RECORD_TYPE_SAMPLE;

	s->time = ts;
	s->flags = 0;
	s->flags |= QUADD_SAMPLE_FLAG_UNCORE;

	s->cpu_id = U8_MAX;
	s->pid = s->tgid = U32_MAX;

	s->ip = 0;
	s->callchain_nr = 0;

	s->events_flags = 0;
	for (i = 0; i < nr_events; i++) {
		u32 value = (u32)events[i].delta;

		if (value > 0) {
			s->events_flags |= 1 << i;
			events_extra[nr_positive++] = value;
		}
	}

	if (nr_positive > 0) {
		vec[vec_idx].base = &extra_data;
		vec[vec_idx].len = sizeof(extra_data);
		vec_idx++;

		vec[vec_idx].base = events_extra;
		vec[vec_idx].len = nr_positive * sizeof(events_extra[0]);
		vec_idx++;

		vec[vec_idx].base = &ts_delta;
		vec[vec_idx].len = sizeof(ts_delta);
		vec_idx++;

		quadd_put_sample(&record, vec, vec_idx);
	}
}

static inline bool get_uncore_sources(void)
{
	int res = true;

	raw_spin_lock(&ctx.state_lock);
	if (unlikely(atomic_read(&ctx.state) != QUADD_UNCORE_STATE_ACTIVE)) {
		res = false;
		goto out;
	}
	atomic_inc(&ctx.ref_count);
out:
	raw_spin_unlock(&ctx.state_lock);
	return res;
}

static inline void put_uncore_sources(void)
{
	atomic_dec(&ctx.ref_count);
}

static void wait_for_close(void)
{
	raw_spin_lock(&ctx.state_lock);
	atomic_set(&ctx.state, QUADD_UNCORE_STATE_STOPPING);
	raw_spin_unlock(&ctx.state_lock);

	while (atomic_read(&ctx.ref_count) > 0)
		cpu_relax();
}

static void read_uncore_sources(u64 ts)
{
	int nr_events = 0;
	struct quadd_event_data events[QUADD_MAX_COUNTERS];

	if (ctx.carmel_pmu && is_source_active(ctx.carmel_info)) {
		if (!get_uncore_sources())
			return;

		nr_events = ctx.carmel_pmu->read(events, ARRAY_SIZE(events));
		put_uncore_sources();

		put_sample(events, nr_events, ts);
	}
}

static enum hrtimer_restart hrtimer_handler(struct hrtimer *hrtimer)
{
	u64 ts = quadd_get_time();

	if (unlikely(atomic_read(&ctx.state) != QUADD_UNCORE_STATE_ACTIVE))
		return HRTIMER_NORESTART;

	read_uncore_sources(ts);
	hrtimer_forward_now(hrtimer, ns_to_ktime(ctx.sample_period));

	return HRTIMER_RESTART;
}

static void start_hrtimer(void)
{
	hrtimer_start(&ctx.hrtimer, ns_to_ktime(ctx.sample_period),
		      HRTIMER_MODE_REL_PINNED);
}

static void cancel_hrtimer(void)
{
	hrtimer_cancel(&ctx.hrtimer);
}

static void init_hrtimer(void)
{
#if (defined(CONFIG_PREEMPT_RT_FULL) && \
		(LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)))
	hrtimer_init(&ctx.hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
#else
	hrtimer_init(&ctx.hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#endif
	ctx.hrtimer.function = hrtimer_handler;
}

int quadd_uncore_start(void)
{
	long freq;
	struct quadd_parameters *p = &ctx.quadd_ctx->param;

	if (atomic_read(&ctx.state) == QUADD_UNCORE_STATE_ACTIVE)
		return 0;

	if (!is_uncore_active())
		return 0;

	freq = p->reserved[QUADD_PARAM_IDX_UNCORE_FREQ];
	if (freq == 0)
		return 0;

	ctx.sample_period = NSEC_PER_SEC / freq;

	atomic_set(&ctx.state, QUADD_UNCORE_STATE_ACTIVE);
	atomic_set(&ctx.ref_count, 0);

	if (ctx.carmel_pmu && is_source_active(ctx.carmel_info)) {
		ctx.carmel_pmu->enable();
		ctx.carmel_pmu->start();
	}

	start_hrtimer();
	pr_info("uncore events: freq: %ld Hz\n", freq);

	return 0;
}

void quadd_uncore_stop(void)
{
	if (atomic_read(&ctx.state) != QUADD_UNCORE_STATE_ACTIVE)
		return;

	cancel_hrtimer();
	wait_for_close();

	if (ctx.carmel_pmu && is_source_active(ctx.carmel_info)) {
		ctx.carmel_pmu->stop();
		ctx.carmel_pmu->disable();
	}
	atomic_set(&ctx.state, QUADD_UNCORE_STATE_INACTIVE);
}

int quadd_uncore_init(struct quadd_ctx *quadd_ctx)
{
	ctx.quadd_ctx = quadd_ctx;
	ctx.carmel_pmu = quadd_ctx->carmel_pmu;
	ctx.carmel_info = &quadd_ctx->carmel_pmu_info;

	init_hrtimer();

	atomic_set(&ctx.state, QUADD_UNCORE_STATE_INACTIVE);
	atomic_set(&ctx.ref_count, 0);
	raw_spin_lock_init(&ctx.state_lock);

	return 0;
}

void quadd_uncore_deinit(void)
{
}
