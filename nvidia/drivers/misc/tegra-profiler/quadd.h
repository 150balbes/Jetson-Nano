/*
 * drivers/misc/tegra-profiler/quadd.h
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __QUADD_H
#define __QUADD_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include <linux/tegra_profiler.h>

struct quadd_event_data {
	int event_source;
	struct quadd_event event;

	u64 val;
	u64 prev_val;

	u64 delta;
	u64 max_count;
};

struct quadd_pmu_cntr_info {
	const char *name;
	u32 id;
};

#define QUADD_PMU_CNTR_INFO(__name, __id)				\
static const struct quadd_pmu_cntr_info quadd_pmu_cntr_##__name = {	\
	.name = __stringify(__name),					\
	.id   =  __id,							\
}

struct quadd_arch_info;
struct quadd_comm_data_interface;

struct quadd_event_source {
	const char *name;
	int	(*enable)(void);
	void	(*disable)(void);
	void	(*start)(void);
	void	(*stop)(void);
	int	(*read)(struct quadd_event_data *events, int max);
	int	(*set_events)(int cpuid, const struct quadd_event *events,
			      int size);
	int	(*supported_events)(int cpuid, struct quadd_event *events,
				    int max, unsigned int *raw_event_mask);
	int	(*current_events)(int cpuid, struct quadd_event *events,
				  int max);
	const struct quadd_arch_info		*(*get_arch)(int cpuid);
	const struct quadd_pmu_cntr_info	**pmu_cntrs;
};

struct source_info {
	struct quadd_event supp_events[QUADD_MAX_COUNTERS];
	int nr_supp_events;

	unsigned int raw_event_mask;

	unsigned int is_present:1;
	unsigned int active:1;
};

struct quadd_hrt_ctx;
struct quadd_module_state;

struct quadd_ctx {
	struct quadd_parameters param;
	struct quadd_comm_cap cap;

	struct quadd_event_source *pmu;
	struct source_info * (*get_pmu_info)(void);
	struct quadd_comm_cap_for_cpu * (*get_capabilities_for_cpu)(int cpuid);

	struct quadd_event_source *carmel_pmu;
	struct source_info carmel_pmu_info;

	struct quadd_comm_data_interface *comm;
	struct quadd_hrt_ctx *hrt;

	atomic_t started;
	atomic_t tegra_profiler_lock;

	unsigned int early_initialized:1;
	unsigned int initialized:1;

	unsigned int collect_kernel_ips:1;

	unsigned int mode_is_sampling:1;
	unsigned int mode_is_tracing:1;
	unsigned int mode_is_sample_all:1;
	unsigned int mode_is_trace_all:1;
	unsigned int mode_is_sample_tree:1;
	unsigned int mode_is_trace_tree:1;

	unsigned int mode_is_sampling_timer:1;
	unsigned int mode_is_sampling_sched:1;

	unsigned int pclk_cpufreq:1;

	struct list_head mmap_areas;
	raw_spinlock_t mmaps_lock;
};

static inline bool quadd_mode_is_sampling(struct quadd_ctx *ctx)
{
	return ctx->mode_is_sampling != 0;
}

static inline bool quadd_mode_is_tracing(struct quadd_ctx *ctx)
{
	return ctx->mode_is_tracing != 0;
}

static inline bool quadd_mode_is_sample_all(struct quadd_ctx *ctx)
{
	return ctx->mode_is_sample_all != 0;
}

static inline bool quadd_mode_is_trace_all(struct quadd_ctx *ctx)
{
	return ctx->mode_is_trace_all != 0;
}

static inline bool quadd_mode_is_sample_tree(struct quadd_ctx *ctx)
{
	return ctx->mode_is_sample_tree != 0;
}

static inline bool quadd_mode_is_trace_tree(struct quadd_ctx *ctx)
{
	return ctx->mode_is_trace_tree != 0;
}

static inline bool quadd_mode_is_process_tree(struct quadd_ctx *ctx)
{
	return (ctx->mode_is_sample_tree != 0 || ctx->mode_is_trace_tree != 0);
}

static inline bool quadd_mode_is_process_all(struct quadd_ctx *ctx)
{
	return (ctx->mode_is_sample_all != 0 || ctx->mode_is_trace_all != 0);
}

static inline bool quadd_mode_is_sampling_timer(struct quadd_ctx *ctx)
{
	return ctx->mode_is_sampling_timer != 0;
}

static inline bool quadd_mode_is_sampling_sched(struct quadd_ctx *ctx)
{
	return ctx->mode_is_sampling_sched != 0;
}

void quadd_get_state(struct quadd_module_state *state);
int quadd_late_init(void);

int tegra_profiler_try_lock(void);
void tegra_profiler_unlock(void);

#endif	/* __QUADD_H */
