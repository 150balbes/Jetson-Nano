/*
 * drivers/misc/tegra-profiler/carmel_pmu.c
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
#include <linux/list.h>
#include <linux/bitmap.h>
#include <linux/errno.h>
#include <linux/topology.h>

#include <soc/tegra/chip-id.h>

#include <asm/sysreg.h>

#include "carmel_pmu.h"
#include "quadd.h"
#include "debug.h"

/*
 * Some parts of this code are taken from platform/tegra/tegra19_perf_uncore.c
 * and modified for tegra-profiler.
 */

/* Global registers */
#define SYS_NV_PMSELR_EL0     sys_reg(3, 3, 15, 5, 1)

/* Unit registers */
#define SYS_NV_PMCNTENSET_EL0 sys_reg(3, 3, 15, 4, 0)
#define SYS_NV_PMCNTENCLR_EL0 sys_reg(3, 3, 15, 4, 1)
#define SYS_NV_PMOVSSET_EL0   sys_reg(3, 3, 15, 4, 2)
#define SYS_NV_PMOVSCLR_EL0   sys_reg(3, 3, 15, 4, 3)
#define SYS_NV_PMCR_EL0       sys_reg(3, 3, 15, 4, 4)
#define SYS_NV_PMINTENSET_EL1 sys_reg(3, 0, 15, 2, 0)
#define SYS_NV_PMINTENCLR_EL1 sys_reg(3, 0, 15, 2, 1)

/* Counter registers */
#define SYS_NV_PMEVCNTR0_EL0  sys_reg(3, 3, 15, 0, 0)
#define SYS_NV_PMEVCNTR1_EL0  sys_reg(3, 3, 15, 0, 1)
#define SYS_NV_PMEVTYPER0_EL0 sys_reg(3, 3, 15, 2, 0)
#define SYS_NV_PMEVTYPER1_EL0 sys_reg(3, 3, 15, 2, 1)

/*
 * Format for raw events is 0xEEU
 * EE: event number
 * U:  unit (0-3 for L2s, 4 for uncore)
 * eg. all L2D_CACHE: r160,r161,r162,r163
 */
#define CARMEL_UNIT(__id)	(__id & 0xf)
#define CARMEL_EVENT(__id)	(__id >> 4)

#define L2D_CACHE				0x16
#define L2D_CACHE_REFILL			0x17
#define L2D_CACHE_WB				0x18

#define BUS_ACCESS				0x19
#define BUS_CYCLES				0x1D
#define L3D_CACHE_ALLOCATE			0x29
#define L3D_CACHE_REFILL			0x2A
#define L3D_CACHE				0x2B
#define L3D_CACHE_WB				0x2C

#define L2D_CACHE_LD				0x50
#define L2D_CACHE_ST				0x51
#define L2D_CACHE_REFILL_LD			0x52
#define L2D_CACHE_REFILL_ST			0x53
#define L2D_CACHE_REFILL_VICTIM			0x56

#define L2D_PREFETCH_C0				0xC5
#define L2D_PREFETCH_C1				0xC6

#define NV_INT_START				0x200
#define NV_INT_END				0x208

#define CARMEL_PMCR_E		(0x1 << 0) /* Enable all counters */
#define CARMEL_PMCR_P		(0x1 << 1) /* Reset all counters */

/* Counters reading CTR_INVAL are in a powered down unit */
#define CTR_INVAL 0xffffffff

/* L2 units in the Carmel CCPLEX */
#define NUM_L2S		0x4

/* Carmel uncore perfmon supports two counters per unit */
#define UNIT_CTRS	0x2

struct cntr_info {
	u32 prev_val;
	u32 id_raw;
	u32 id_hw;
};

struct carmel_unit {
	u32 id;
	u32 pmselr;
	bool is_used;
	bool is_available;
	struct cntr_info cntrs[UNIT_CTRS];
	DECLARE_BITMAP(used_ctrs, UNIT_CTRS);
	struct list_head next;
};

static struct carmel_pmu_ctx {
	struct carmel_unit clusters[NUM_L2S];
	struct carmel_unit scf;
	DECLARE_BITMAP(used_units, NUM_L2S + 1);
	struct list_head units;
} ctx;

static inline void set_unit(struct carmel_unit *unit)
{
	write_sysreg_s(unit->pmselr, SYS_NV_PMSELR_EL0);
	isb();
}

static inline u32 get_unit_pmselr(u32 g, u32 u)
{
	return ((g << 8) | u);
}

static inline void set_event_type(u32 val, u32 idx)
{
	if (idx == 0)
		write_sysreg_s(val, SYS_NV_PMEVTYPER0_EL0);
	else if (idx == 1)
		write_sysreg_s(val, SYS_NV_PMEVTYPER1_EL0);

	isb();
}

static inline u32 read_cntr(u32 idx)
{
	u32 val = 0;

	if (idx == 0)
		val = read_sysreg_s(SYS_NV_PMEVCNTR0_EL0);
	else if (idx == 1)
		val = read_sysreg_s(SYS_NV_PMEVCNTR1_EL0);

	return val;
}

static inline void write_cntr(u32 val, u32 idx)
{
	if (idx == 0)
		write_sysreg_s(val, SYS_NV_PMEVCNTR0_EL0);
	else if (idx == 1)
		write_sysreg_s(val, SYS_NV_PMEVCNTR1_EL0);
}

static int set_event(u32 val, u32 idx)
{
	u32 __val;

	set_event_type(val, idx);
	write_cntr(0, idx);
	isb();

	/* If counter is stuck at CTR_INVAL, the unit is powered down */
	__val = read_cntr(idx);
	if (__val == CTR_INVAL)
		return -ENODEV;

	return 0;
}

static void start_unit(struct carmel_unit *unit)
{
	unsigned long idx = 0;

	if (bitmap_empty(unit->used_ctrs, UNIT_CTRS))
		return;

	set_unit(unit);

	while (idx < UNIT_CTRS) {
		idx = find_next_bit(unit->used_ctrs, UNIT_CTRS, idx);
		if (idx != UNIT_CTRS) {
			unit->cntrs[idx].prev_val = 0;
			write_sysreg_s(BIT(idx), SYS_NV_PMCNTENSET_EL0);
		}
		idx++;
	}

	write_sysreg_s(CARMEL_PMCR_E, SYS_NV_PMCR_EL0);
}

static void stop_unit(struct carmel_unit *unit)
{
	unsigned long idx = 0;

	if (bitmap_empty(unit->used_ctrs, UNIT_CTRS))
		return;

	set_unit(unit);

	while (idx < UNIT_CTRS) {
		idx = find_next_bit(unit->used_ctrs, UNIT_CTRS, idx);
		if (idx != UNIT_CTRS)
			write_sysreg_s(BIT(idx), SYS_NV_PMCNTENCLR_EL0);
		idx++;
	}

	write_sysreg_s(CARMEL_PMCR_P, SYS_NV_PMCR_EL0);
}

static int
read_unit_cntrs(struct carmel_unit *unit,
		struct quadd_event_data *events, int max)
{
	unsigned long idx = 0;
	u32 val, prev_val, delta;
	struct cntr_info *cntr;
	struct quadd_event_data *curr, *end;

	if (unlikely(bitmap_empty(unit->used_ctrs, UNIT_CTRS)))
		return 0;

	curr = events;
	end = events + max;

	set_unit(unit);

	while (idx < UNIT_CTRS && curr < end) {
		idx = find_next_bit(unit->used_ctrs, UNIT_CTRS, idx);
		if (idx != UNIT_CTRS) {
			cntr = &unit->cntrs[idx];
			val = read_cntr(idx);
			if (val == CTR_INVAL)
				val = cntr->prev_val;

			curr->event_source =
				QUADD_EVENT_SOURCE_CARMEL_UNCORE_PMU;
			curr->max_count = U32_MAX;

			curr->event.type = QUADD_EVENT_TYPE_RAW_CARMEL_UNCORE;
			curr->event.id = cntr->id_raw;

			prev_val = cntr->prev_val;

			if (prev_val <= val)
				delta = val - prev_val;
			else
				delta = U32_MAX - prev_val + val;

			curr->val = val;
			curr->prev_val = prev_val;
			curr->delta = delta;

			cntr->prev_val = val;
			curr++;
		}
		idx++;
	}

	return curr - events;
}

static inline struct carmel_unit *get_unit(u32 id)
{
	switch (id) {
	case 0 ... (NUM_L2S - 1):
		return &ctx.clusters[id];
	case NUM_L2S:
		return &ctx.scf;
	default:
		return NULL;
	}
}

static int add_event(const struct quadd_event *event)
{
	int err;
	unsigned long idx;
	u32 unit_id, event_raw;
	struct cntr_info *cntr;
	struct carmel_unit *unit;

	unit_id = CARMEL_UNIT(event->id);
	event_raw = CARMEL_EVENT(event->id);

	unit = get_unit(unit_id);
	if (!unit || !unit->is_available)
		return -ENOENT;

	set_unit(unit);

	idx = find_first_zero_bit(unit->used_ctrs, UNIT_CTRS);
	if (idx >= UNIT_CTRS)
		return -EOPNOTSUPP;

	err = set_event(event_raw, idx);
	if (err < 0)
		return err;

	cntr = &unit->cntrs[idx];
	cntr->id_raw = event->id;
	cntr->id_hw = event_raw;

	set_bit(idx, unit->used_ctrs);
	set_bit(unit->id, ctx.used_units);

	unit->is_used = true;

	return 0;
}

static void clean_all_units(void)
{
	struct carmel_unit *unit;

	list_for_each_entry(unit, &ctx.units, next) {
		if (unit->is_used) {
			memset(unit->cntrs, 0, sizeof(unit->cntrs));
			bitmap_zero(unit->used_ctrs, UNIT_CTRS);
		}
	}
	bitmap_zero(ctx.used_units, NUM_L2S + 1);
}

static int carmel_pmu_enable(void)
{
	return 0;
}

static void carmel_pmu_disable(void)
{
	clean_all_units();
}

static void carmel_pmu_start(void)
{
	struct carmel_unit *unit;

	list_for_each_entry(unit, &ctx.units, next) {
		if (unit->is_used)
			start_unit(unit);
	}
}

static void carmel_pmu_stop(void)
{
	struct carmel_unit *unit;

	list_for_each_entry(unit, &ctx.units, next) {
		if (unit->is_used)
			stop_unit(unit);
	}
}

static int carmel_pmu_read(struct quadd_event_data *events, int max)
{
	struct carmel_unit *unit;
	struct quadd_event_data *curr, *end;

	if (max == 0)
		return 0;

	curr = events;
	end = events + max;

	list_for_each_entry(unit, &ctx.units, next) {
		if (unit->is_used) {
			curr += read_unit_cntrs(unit, curr, end - curr);
			if (curr >= end)
				break;
		}
	}

	return curr - events;
}

static int
carmel_pmu_set_events(int cpuid, const struct quadd_event *events, int size)
{
	int i, err;

	clean_all_units();

	for (i = 0; i < size; i++) {
		const struct quadd_event *event = &events[i];

		if (event->type == QUADD_EVENT_TYPE_RAW_CARMEL_UNCORE) {
			err = add_event(event);
			if (err < 0) {
				clean_all_units();
				return err;
			}
		}
	}

	return 0;
}

static int
supported_events(int cpuid, struct quadd_event *events,
		 int max, unsigned int *raw_event_mask)
{
	*raw_event_mask = 0x0fff;
	return 0;
}

static int
current_events(int cpuid, struct quadd_event *events, int max)
{
	struct quadd_event *curr, *end;
	struct carmel_unit *unit;

	if (max == 0)
		return 0;

	curr = events;
	end = curr + max;

	list_for_each_entry(unit, &ctx.units, next) {
		if (unit->is_used) {
			unsigned long idx = 0;

			while (idx < UNIT_CTRS) {
				idx = find_next_bit(unit->used_ctrs,
						    UNIT_CTRS, idx);
				if (idx != UNIT_CTRS) {
					curr->type =
					    QUADD_EVENT_TYPE_RAW_CARMEL_UNCORE;
					curr->id = unit->cntrs[idx].id_raw;

					if (++curr >= end)
						return curr - events;
				}
				idx++;
			}
		}
	}

	return curr - events;
}

static bool is_cluster_available(int cluster_id)
{
	int cpu;

	for_each_possible_cpu(cpu)
		if (cpu_topology[cpu].cluster_id == cluster_id)
			return true;

	return false;
}

QUADD_PMU_CNTR_INFO(l2d_cache, L2D_CACHE);
QUADD_PMU_CNTR_INFO(l2d_refill, L2D_CACHE_REFILL);
QUADD_PMU_CNTR_INFO(l2d_cache_wb, L2D_CACHE_WB);
QUADD_PMU_CNTR_INFO(l2d_ld, L2D_CACHE_LD);
QUADD_PMU_CNTR_INFO(l2d_st, L2D_CACHE_ST);
QUADD_PMU_CNTR_INFO(l2d_refill_ld, L2D_CACHE_REFILL_LD);
QUADD_PMU_CNTR_INFO(l2d_refill_st, L2D_CACHE_REFILL_ST);
QUADD_PMU_CNTR_INFO(l2d_refill_victim, L2D_CACHE_REFILL_VICTIM);
QUADD_PMU_CNTR_INFO(l2d_prefetch_c0, L2D_PREFETCH_C0);
QUADD_PMU_CNTR_INFO(l2d_prefetch_c1, L2D_PREFETCH_C1);
QUADD_PMU_CNTR_INFO(bus_access, BUS_ACCESS);
QUADD_PMU_CNTR_INFO(bus_cycles, BUS_CYCLES);
QUADD_PMU_CNTR_INFO(l3d_cache_allocate, L3D_CACHE_ALLOCATE);
QUADD_PMU_CNTR_INFO(l3d_cache_refill, L3D_CACHE_REFILL);
QUADD_PMU_CNTR_INFO(l3d_cache, L3D_CACHE);
QUADD_PMU_CNTR_INFO(l3d_cache_wb, L3D_CACHE_WB);

static const struct quadd_pmu_cntr_info *carmel_cntrs[] = {
	&quadd_pmu_cntr_l2d_cache,
	&quadd_pmu_cntr_l2d_refill,
	&quadd_pmu_cntr_l2d_cache_wb,
	&quadd_pmu_cntr_l2d_ld,
	&quadd_pmu_cntr_l2d_st,
	&quadd_pmu_cntr_l2d_refill_ld,
	&quadd_pmu_cntr_l2d_refill_st,
	&quadd_pmu_cntr_l2d_refill_victim,
	&quadd_pmu_cntr_l2d_prefetch_c0,
	&quadd_pmu_cntr_l2d_prefetch_c1,
	&quadd_pmu_cntr_bus_access,
	&quadd_pmu_cntr_bus_cycles,
	&quadd_pmu_cntr_l3d_cache_allocate,
	&quadd_pmu_cntr_l3d_cache_refill,
	&quadd_pmu_cntr_l3d_cache,
	&quadd_pmu_cntr_l3d_cache_wb,
	NULL,
};

static struct
quadd_event_source carmel_uncore_pmu_int = {
	.name			= "carmel_pmu",
	.enable			= carmel_pmu_enable,
	.disable		= carmel_pmu_disable,
	.start			= carmel_pmu_start,
	.stop			= carmel_pmu_stop,
	.read			= carmel_pmu_read,
	.set_events		= carmel_pmu_set_events,
	.supported_events	= supported_events,
	.current_events		= current_events,
	.pmu_cntrs		= carmel_cntrs,
};

struct quadd_event_source *
quadd_carmel_uncore_pmu_init(void)
{
	int i;
	struct carmel_unit *unit;

	if (tegra_get_chipid() != TEGRA_CHIPID_TEGRA19)
		return NULL;

	INIT_LIST_HEAD(&ctx.units);

	for (i = 0; i < NUM_L2S; i++) {
		unit = &ctx.clusters[i];
		unit->id = i;
		unit->pmselr = get_unit_pmselr(1, i);
		unit->is_used = false;
		unit->is_available = is_cluster_available(i);

		bitmap_zero(unit->used_ctrs, UNIT_CTRS);
		list_add(&unit->next, &ctx.units);
	}

	unit = &ctx.scf;
	unit->id = NUM_L2S;
	unit->pmselr = get_unit_pmselr(0, 0);
	unit->is_used = false;
	unit->is_available = true;

	bitmap_zero(unit->used_ctrs, UNIT_CTRS);
	list_add(&unit->next, &ctx.units);

	bitmap_zero(ctx.used_units, NUM_L2S + 1);

	return &carmel_uncore_pmu_int;
}

void quadd_carmel_uncore_pmu_deinit(void)
{
}
