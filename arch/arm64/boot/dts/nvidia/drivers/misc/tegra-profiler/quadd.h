/*
 * drivers/misc/tegra-profiler/quadd.h
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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

/* #define QUADD_USE_EMULATE_COUNTERS	1 */

struct quadd_comm_data_interface;
struct quadd_hrt_ctx;
struct quadd_module_state;
struct quadd_arch_info;

struct event_data {
	int event_source;
	struct quadd_event event;

	u32 val;
	u32 prev_val;
};

struct quadd_event_source_interface {
	int (*enable)(void);
	void (*disable)(void);
	void (*start)(void);
	void (*stop)(void);
	int (*read)(struct event_data *events, int max_events);
	int (*set_events)(int cpuid, const struct quadd_event *events,
			  int size);
	int (*get_supported_events)(int cpuid, struct quadd_event *events,
				    int max_events,
				    unsigned int *raw_event_mask);
	int (*get_current_events)(int cpuid, struct quadd_event *events,
				  int max_events);
	struct quadd_arch_info * (*get_arch)(int cpuid);
};

struct source_info {
	struct quadd_event supp_events[QUADD_MAX_COUNTERS];
	int nr_supp_events;

	unsigned int raw_event_mask;

	unsigned int is_present:1;
	unsigned int active:1;
};

struct quadd_ctx {
	struct quadd_parameters param;
	struct quadd_comm_cap cap;

	struct quadd_event_source_interface *pmu;
	struct source_info * (*get_pmu_info)(void);
	struct quadd_comm_cap_for_cpu * (*get_capabilities_for_cpu)(int cpuid);

	struct quadd_event_source_interface *pl310;
	struct source_info pl310_info;

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

void quadd_get_state(struct quadd_module_state *state);
int quadd_late_init(void);

int tegra_profiler_try_lock(void);
void tegra_profiler_unlock(void);

#endif	/* __QUADD_H */
