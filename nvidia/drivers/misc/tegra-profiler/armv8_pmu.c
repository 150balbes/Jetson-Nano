/*
 * drivers/misc/tegra-profiler/armv8_pmu.c
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/printk.h>
#include <linux/types.h>
#include <linux/string.h>

#include <linux/version.h>
#include <linux/err.h>
#include <linux/bitmap.h>
#include <linux/slab.h>

#include <asm/cputype.h>
#include <asm/cpu.h>

#include "arm_pmu.h"
#include "armv8_pmu.h"
#include "armv8_events.h"
#include "quadd.h"
#include "debug.h"

struct quadd_pmu_info {
	DECLARE_BITMAP(used_cntrs, QUADD_MAX_PMU_COUNTERS);
	u64 prev_vals[QUADD_MAX_PMU_COUNTERS];
	int is_already_active;
};

struct quadd_cntrs_info {
	int pcntrs;
	int ccntr;

	raw_spinlock_t lock;
};

static DEFINE_PER_CPU(struct quadd_pmu_info, cpu_pmu_info);

static DEFINE_PER_CPU(struct quadd_pmu_ctx, pmu_ctx);

static unsigned int
quadd_armv8_pmuv3_arm_events_map[QUADD_EVENT_HW_MAX] = {
	[QUADD_EVENT_HW_INSTRUCTIONS] =
		QUADD_ARMV8_HW_EVENT_INSTR_EXECUTED,
	[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_BRANCH_MISSES] =
		QUADD_ARMV8_HW_EVENT_PC_BRANCH_MIS_PRED,
	[QUADD_EVENT_HW_BUS_CYCLES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,

	[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_ICACHE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_ICACHE_REFILL,

	[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES] =
		QUADD_ARMV8_HW_EVENT_L2_CACHE_REFILL,
	[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L2_CACHE_REFILL,
	[QUADD_EVENT_HW_L2_ICACHE_MISSES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
};

static unsigned int
quadd_armv8_pmuv3_a57_events_map[QUADD_EVENT_HW_MAX] = {
	[QUADD_EVENT_HW_INSTRUCTIONS] =
		QUADD_ARMV8_HW_EVENT_INSTR_EXECUTED,
	[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_BRANCH_MISSES] =
		QUADD_ARMV8_HW_EVENT_PC_BRANCH_MIS_PRED,
	[QUADD_EVENT_HW_BUS_CYCLES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,

	[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES] =
		QUADD_ARMV8_A57_HW_EVENT_L1D_CACHE_REFILL_LD,
	[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_A57_HW_EVENT_L1D_CACHE_REFILL_ST,
	[QUADD_EVENT_HW_L1_ICACHE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_ICACHE_REFILL,

	[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES] =
		QUADD_ARMV8_A57_HW_EVENT_L2D_CACHE_REFILL_LD,
	[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_A57_HW_EVENT_L2D_CACHE_REFILL_ST,
	[QUADD_EVENT_HW_L2_ICACHE_MISSES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
};

static unsigned
quadd_armv8_pmuv3_denver_events_map[QUADD_EVENT_HW_MAX] = {
	[QUADD_EVENT_HW_INSTRUCTIONS] =
		QUADD_ARMV8_HW_EVENT_INSTR_EXECUTED,
	[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_BRANCH_MISSES] =
		QUADD_ARMV8_HW_EVENT_PC_BRANCH_MIS_PRED,
	[QUADD_EVENT_HW_BUS_CYCLES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,

	[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_ICACHE_MISSES] =
		QUADD_ARMV8_HW_EVENT_L1_ICACHE_REFILL,

	[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_L2_ICACHE_MISSES] =
		QUADD_ARMV8_UNSUPPORTED_EVENT,
};

/*********************************************************************/

static inline u32
armv8_pmu_pmcr_read(void)
{
	u32 val;

	/* Read Performance Monitors Control Register */
	asm volatile("mrs %0, pmcr_el0" : "=r" (val));
	return val;
}

static inline void
armv8_pmu_pmcr_write(u32 val)
{
	isb();

	/* Write Performance Monitors Control Register */
	asm volatile("msr pmcr_el0, %0" : :
		     "r" (val & QUADD_ARMV8_PMCR_WR_MASK));
}

static inline u32
armv8_pmu_pmceid_read(void)
{
	u32 val;

	/* Read Performance Monitors Common Event Identification Register */
	asm volatile("mrs %0, pmceid0_el0" : "=r" (val));
	return val;
}

static inline u32
armv8_pmu_pmcntenset_read(void)
{
	u32 val;

	/* Read Performance Monitors Count Enable Set Register */
	asm volatile("mrs %0, pmcntenset_el0" : "=r" (val));
	return val;
}

static inline void
armv8_pmu_pmcntenset_write(u32 val)
{
	/* Write Performance Monitors Count Enable Set Register */
	asm volatile("msr pmcntenset_el0, %0" : : "r" (val));
}

static inline void
armv8_pmu_pmcntenclr_write(u32 val)
{
	/* Write Performance Monitors Count Enable Clear Register */
	asm volatile("msr pmcntenclr_el0, %0" : : "r" (val));
}

static inline void
armv8_pmu_pmselr_write(u32 val)
{
	/* Write Performance Monitors Event Counter Selection Register */
	asm volatile("msr pmselr_el0, %0" : :
		     "r" (val & QUADD_ARMV8_SELECT_MASK));
	isb();
}

static inline u64
armv8_pmu_pmccntr_read(void)
{
	u64 val;

	/* Read Performance Monitors Cycle Count Register */
	asm volatile("mrs %0, pmccntr_el0" : "=r" (val));
	return val;
}

static inline void
armv8_pmu_pmccntr_write(u64 val)
{
	/* Write Performance Monitors Selected Event Count Register */
	asm volatile("msr pmccntr_el0, %0" : : "r" (val));
}

static inline u32
armv8_pmu_pmxevcntr_read(void)
{
	u32 val;

	/* Read Performance Monitors Selected Event Count Register */
	asm volatile("mrs %0, pmxevcntr_el0" : "=r" (val));
	return val;
}

static inline void
armv8_pmu_pmxevcntr_write(u32 val)
{
	/* Write Performance Monitors Selected Event Count Register */
	asm volatile("msr pmxevcntr_el0, %0" : : "r" (val));
}

static inline void
armv8_pmu_pmxevtyper_write(u32 event)
{
	/* Write Performance Monitors Selected Event Type Register */
	asm volatile("msr pmxevtyper_el0, %0" : :
		     "r" (event & QUADD_ARMV8_EVTSEL_MASK));
}

static inline u32 __maybe_unused
armv8_pmu_pmintenset_read(void)
{
	u32 val;

	/* Read Performance Monitors Interrupt Enable Set Register */
	asm volatile("mrs %0, pmintenset_el1" : "=r" (val));
	return val;
}

static inline void __maybe_unused
armv8_pmu_pmintenset_write(u32 val)
{
	/* Write Performance Monitors Interrupt Enable Set Register */
	asm volatile("msr pmintenset_el1, %0" : : "r" (val));
}

static inline void __maybe_unused
armv8_pmu_pmintenclr_write(u32 val)
{
	/* Write Performance Monitors Interrupt Enable Clear Register */
	asm volatile("msr pmintenclr_el1, %0" : : "r" (val));
}

static inline u32 __maybe_unused
armv8_pmu_pmovsclr_read(void)
{
	u32 val;

	/* Read Performance Monitors Overflow Flag Status Register */
	asm volatile("mrs %0, pmovsclr_el0" : "=r" (val));
	return val;
}

static inline void
armv8_pmu_pmovsclr_write(u32 val)
{
	/* Write Performance Monitors Overflow Flag Status Register */
	asm volatile("msr pmovsclr_el0, %0" : : "r" (val));
}

static inline u32
armv8_id_afr0_el1_read(void)
{
	u32 val;

	/* Read Auxiliary Feature Register 0 */
	asm volatile("mrs %0, id_afr0_el1" : "=r" (val));
	return val;
}

static void enable_counter(int idx)
{
	armv8_pmu_pmcntenset_write(BIT(idx));
}

static void disable_counter(int idx)
{
	armv8_pmu_pmcntenclr_write(BIT(idx));
}

static void select_counter(unsigned int counter)
{
	armv8_pmu_pmselr_write(counter);
}

static int is_pmu_enabled(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 pmcr = armv8_pmu_pmcr_read();

	if (pmcr & QUADD_ARMV8_PMCR_E) {
		u32 pmcnten = armv8_pmu_pmcntenset_read();

		pmcnten &= local_pmu_ctx->counters_mask | QUADD_ARMV8_CCNT;
		return pmcnten ? 1 : 0;
	}

	return 0;
}

static u64 read_counter(int idx)
{
	u64 val;

	if (idx == QUADD_ARMV8_CCNT_BIT) {
		val = armv8_pmu_pmccntr_read();
	} else {
		select_counter(idx);
		val = armv8_pmu_pmxevcntr_read();
	}

	return val;
}

static void write_counter(int idx, u64 value)
{
	if (idx == QUADD_ARMV8_CCNT_BIT) {
		armv8_pmu_pmccntr_write(value);
	} else {
		select_counter(idx);
		armv8_pmu_pmxevcntr_write((u32)value);
	}
}

static int
get_free_counters(unsigned long *bitmap, int nbits, int *ccntr)
{
	int cc;
	u32 cntens;
	unsigned long cntens_bitmap;

	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	cntens = armv8_pmu_pmcntenset_read();
	cntens = ~cntens & (local_pmu_ctx->counters_mask | QUADD_ARMV8_CCNT);

	cntens_bitmap = cntens;

	bitmap_zero(bitmap, nbits);
	bitmap_copy(bitmap, &cntens_bitmap, BITS_PER_BYTE * sizeof(u32));

	cc = (cntens & QUADD_ARMV8_CCNT) ? 1 : 0;

	if (ccntr)
		*ccntr = cc;

	return bitmap_weight(bitmap, BITS_PER_BYTE * sizeof(u32)) - cc;
}

static void __maybe_unused
disable_interrupt(int idx)
{
	armv8_pmu_pmintenclr_write(BIT(idx));
}

static void
disable_all_interrupts(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 val = QUADD_ARMV8_CCNT | local_pmu_ctx->counters_mask;

	armv8_pmu_pmintenclr_write(val);
}

static void
reset_overflow_flags(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 val = QUADD_ARMV8_CCNT | local_pmu_ctx->counters_mask;

	armv8_pmu_pmovsclr_write(val);
}

static void
select_event(unsigned int idx, unsigned int event)
{
	select_counter(idx);
	armv8_pmu_pmxevtyper_write(event);
}

static void disable_all_counters(void)
{
	u32 val;
	u32 masked;
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	/* Disable all counters */
	val = armv8_pmu_pmcr_read();
	if (val & QUADD_ARMV8_PMCR_E)
		armv8_pmu_pmcr_write(val & ~QUADD_ARMV8_PMCR_E);

	masked = QUADD_ARMV8_CCNT | local_pmu_ctx->counters_mask;
	armv8_pmu_pmcntenclr_write(masked);
}

static void enable_all_counters(void)
{
	u32 val;

	/* Enable all counters */
	val = armv8_pmu_pmcr_read();
	val |= QUADD_ARMV8_PMCR_E | QUADD_ARMV8_PMCR_X;
	armv8_pmu_pmcr_write(val);
}

static void reset_all_counters(void)
{
	u32 val;

	val = armv8_pmu_pmcr_read();
	val |= QUADD_ARMV8_PMCR_P | QUADD_ARMV8_PMCR_C;
	armv8_pmu_pmcr_write(val);
}

static void quadd_init_pmu(void)
{
	reset_overflow_flags();
	disable_all_interrupts();
}

static void free_events(struct list_head *head)
{
	struct quadd_pmu_event_info *entry, *next;

	list_for_each_entry_safe(entry, next, head, list) {
		list_del(&entry->list);
		kfree(entry);
	}
}

static void free_used_events(void)
{
	int cpu_id;

	for_each_possible_cpu(cpu_id) {
		struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpu_id);

		if (local_pmu_ctx->current_map)
			free_events(&local_pmu_ctx->used_events);
	}
}

static int pmu_enable(void)
{
	pr_debug("pmu was reserved\n");
	return 0;
}

static void __pmu_disable(void *arg)
{
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);

	if (!pi->is_already_active) {
		pr_debug("[%d] reset all counters\n",
			 smp_processor_id());

		disable_all_counters();
		reset_all_counters();
	} else {
		int idx;

		for_each_set_bit(idx, pi->used_cntrs, QUADD_MAX_PMU_COUNTERS) {
			pr_debug("[%d] reset counter: %d\n",
				 smp_processor_id(), idx);

			disable_counter(idx);
			write_counter(idx, 0);
		}
	}
}

static void pmu_disable(void)
{
	on_each_cpu(__pmu_disable, NULL, 1);
	free_used_events();
	pr_debug("pmu was released\n");
}

static void pmu_start(void)
{
	int idx = 0, pcntrs, ccntr;
	u32 event;
	struct quadd_pmu_ctx *local_pmu_ctx;
	DECLARE_BITMAP(free_bitmap, QUADD_MAX_PMU_COUNTERS);
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);
	u64 *prevp = pi->prev_vals;
	struct quadd_pmu_event_info *ei;

	bitmap_zero(pi->used_cntrs, QUADD_MAX_PMU_COUNTERS);

	if (is_pmu_enabled()) {
		pi->is_already_active = 1;
	} else {
		disable_all_counters();
		quadd_init_pmu();

		pi->is_already_active = 0;
	}

	pcntrs = get_free_counters(free_bitmap, QUADD_MAX_PMU_COUNTERS, &ccntr);

	local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		int index;

		*prevp++ = 0;

		event = ei->hw_value;

		if (is_cpu_cycles(&ei->event)) {
			if (!ccntr) {
				pr_err_once("Error: cpu cycles counter is already occupied\n");
				return;
			}
			index = QUADD_ARMV8_CCNT_BIT;
		} else {
			if (!pcntrs--) {
				pr_err_once("Error: too many performance events\n");
				return;
			}

			index = find_next_bit(free_bitmap,
					      QUADD_MAX_PMU_COUNTERS, idx);
			if (index >= QUADD_MAX_PMU_COUNTERS) {
				pr_err_once("Error: too many events\n");
				return;
			}
			idx = index + 1;
			select_event(index, event);
		}
		set_bit(index, pi->used_cntrs);

		write_counter(index, 0);
		enable_counter(index);
	}

	if (!pi->is_already_active) {
		reset_all_counters();
		enable_all_counters();
	}

	qm_debug_start_source(QUADD_EVENT_SOURCE_PMU);
}

static void pmu_stop(void)
{
	int idx;
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);

	if (!pi->is_already_active) {
		disable_all_counters();
		reset_all_counters();
	} else {
		for_each_set_bit(idx, pi->used_cntrs, QUADD_MAX_PMU_COUNTERS) {
			disable_counter(idx);
			write_counter(idx, 0);
		}
	}

	qm_debug_stop_source(QUADD_EVENT_SOURCE_PMU);
}

static int
pmu_read(struct quadd_event_data *events, int max_events)
{
	u64 val, prev_val, delta;
	int idx = 0, i = 0;
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u64 *prevp = pi->prev_vals;
	struct quadd_pmu_event_info *ei;

	if (bitmap_empty(pi->used_cntrs, QUADD_MAX_PMU_COUNTERS)) {
		pr_err_once("Error: counters were not initialized\n");
		return 0;
	}

	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		int index;

		if (is_cpu_cycles(&ei->event)) {
			if (!test_bit(QUADD_ARMV8_CCNT_BIT, pi->used_cntrs)) {
				pr_err_once("Error: ccntr is not used\n");
				return 0;
			}
			index = QUADD_ARMV8_CCNT_BIT;
			events->max_count = U64_MAX;
		} else {
			index = find_next_bit(pi->used_cntrs,
					      QUADD_MAX_PMU_COUNTERS, idx);
			idx = index + 1;

			if (index >= QUADD_MAX_PMU_COUNTERS) {
				pr_err_once("Error: perf counter is not used\n");
				return 0;
			}
			events->max_count = U32_MAX;
		}

		val = read_counter(index);

		events->event_source = QUADD_EVENT_SOURCE_PMU;
		events->event = ei->event;

		prev_val = *prevp;

		if (prev_val <= val)
			delta = val - prev_val;
		else
			delta = events->max_count - prev_val + val;

		events->val = val;
		events->prev_val = prev_val;
		events->delta = delta;

		*prevp = val;

		qm_debug_read_counter(&events->event, events->prev_val,
				      events->val);

		if (++i >= max_events)
			break;

		events++;
		prevp++;
	}

	return i;
}

static void __get_free_counters(void *arg)
{
	int pcntrs, ccntr;
	DECLARE_BITMAP(free_bitmap, QUADD_MAX_PMU_COUNTERS);
	struct quadd_cntrs_info *ci = arg;

	pcntrs = get_free_counters(free_bitmap, QUADD_MAX_PMU_COUNTERS, &ccntr);

	raw_spin_lock(&ci->lock);

	ci->pcntrs = min_t(int, pcntrs, ci->pcntrs);

	if (!ccntr)
		ci->ccntr = 0;

	pr_debug("[%d] pcntrs/ccntr: %d/%d, free_bitmap: %#lx\n",
		 smp_processor_id(), pcntrs, ccntr, free_bitmap[0]);

	raw_spin_unlock(&ci->lock);
}

static int
set_events(int cpuid, const struct quadd_event *events, int size)
{
	int i, free_pcntrs, err;
	struct quadd_cntrs_info free_ci;
	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);

	free_events(&local_pmu_ctx->used_events);

	if (!events || !size)
		return 0;

	if (!local_pmu_ctx->current_map) {
		pr_err("Invalid current_map\n");
		return -ENODEV;
	}

	raw_spin_lock_init(&free_ci.lock);
	free_ci.pcntrs = QUADD_MAX_PMU_COUNTERS;
	free_ci.ccntr = 1;

	smp_call_function_single(cpuid, __get_free_counters, &free_ci, 1);

	free_pcntrs = free_ci.pcntrs;
	pr_debug("free counters: pcntrs/ccntr: %d/%d\n",
		 free_pcntrs, free_ci.ccntr);

	for (i = 0; i < size; i++) {
		unsigned int type, id;
		struct quadd_pmu_event_info *ei;

		type = events[i].type;
		id = events[i].id;

		if (type == QUADD_EVENT_TYPE_HARDWARE) {
			if (id >= QUADD_EVENT_HW_MAX) {
				err = -EINVAL;
				goto out_free;
			}
		} else if (type == QUADD_EVENT_TYPE_RAW) {
			if (id & ~local_pmu_ctx->raw_event_mask) {
				err = -EINVAL;
				goto out_free;
			}
		} else {
			err = -EINVAL;
			goto out_free;
		}

		ei = kzalloc(sizeof(*ei), GFP_KERNEL);
		if (!ei) {
			err = -ENOMEM;
			goto out_free;
		}

		INIT_LIST_HEAD(&ei->list);
		list_add_tail(&ei->list, &local_pmu_ctx->used_events);

		if (is_cpu_cycles(&events[i])) {
			ei->hw_value = QUADD_ARMV8_CPU_CYCLE_EVENT;
			if (!free_ci.ccntr) {
				pr_err("error: cpu cycles counter is already occupied\n");
				err = -EBUSY;
				goto out_free;
			}
		} else {
			if (!free_pcntrs--) {
				pr_err("error: too many performance events\n");
				err = -ENOSPC;
				goto out_free;
			}

			ei->hw_value = (type == QUADD_EVENT_TYPE_RAW) ? id :
				local_pmu_ctx->current_map[id];
		}

		ei->event = events[i];

		pr_debug("[%d] Event has been added: id: %#x (%s), hw value: %#x\n",
			 cpuid, id, type == QUADD_EVENT_TYPE_RAW ? "raw" : "hw",
			 ei->hw_value);
	}

	return 0;

out_free:
	free_events(&local_pmu_ctx->used_events);
	return err;
}

static int
supported_events(int cpuid, struct quadd_event *events,
		 int max_events, unsigned int *raw_event_mask)
{
	int i, nr_events = 0;

	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);

	if (!local_pmu_ctx->current_map)
		return 0;

	max_events = min_t(int, QUADD_EVENT_HW_MAX, max_events);

	for (i = 0; i < max_events; i++) {
		unsigned int event = local_pmu_ctx->current_map[i];

		if (event != QUADD_ARMV8_UNSUPPORTED_EVENT) {
			events[nr_events].type = QUADD_EVENT_TYPE_HARDWARE;
			events[nr_events].id = i;

			nr_events++;
		}
	}

	*raw_event_mask = local_pmu_ctx->raw_event_mask;

	return nr_events;
}

static int
current_events(int cpuid, struct quadd_event *events, int max_events)
{
	int i = 0;
	struct quadd_pmu_event_info *ei;
	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);

	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		events[i++] = ei->event;

		if (i >= max_events)
			break;
	}

	return i;
}

static const struct quadd_arch_info *get_arch(int cpuid)
{
	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);

	return local_pmu_ctx->current_map ? &local_pmu_ctx->arch : NULL;
}

static struct quadd_event_source pmu_armv8_int = {
	.name			= "armv8_pmuv3",
	.enable			= pmu_enable,
	.disable		= pmu_disable,
	.start			= pmu_start,
	.stop			= pmu_stop,
	.read			= pmu_read,
	.set_events		= set_events,
	.supported_events	= supported_events,
	.current_events		= current_events,
	.get_arch		= get_arch,
};

static int quadd_armv8_pmu_init_for_cpu(int cpuid)
{
	int idx, err = 0;
	u32 pmcr, idcode = 0, reg_midr;
	u64 aa64_dfr;
	u8 implementer;
	struct cpuinfo_arm64 *local_cpu_data = &per_cpu(cpu_data, cpuid);
	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);
	struct quadd_arch_info *arch = &local_pmu_ctx->arch;

	reg_midr = local_cpu_data->reg_midr;

	strncpy(arch->name, "Unknown", sizeof(arch->name));

	arch->type = QUADD_AA64_CPU_TYPE_UNKNOWN;
	arch->pmuver_is_set = 0;
	local_pmu_ctx->current_map = NULL;

	INIT_LIST_HEAD(&local_pmu_ctx->used_events);

	if (!reg_midr)
		return 0;

	implementer = MIDR_IMPLEMENTOR(reg_midr);

	aa64_dfr = local_cpu_data->reg_id_aa64dfr0;
	arch->pmuver = (aa64_dfr >> ID_AA64DFR0_PMUVER_SHIFT) &
			QUADD_AA64_ID_AA64DFR0_PMUVER_MASK;
	arch->pmuver_is_set = 1;

	if (implementer == 'A' || implementer == 'N') {

		strncpy(arch->name, "AA64 PmuV3", sizeof(arch->name));

		idx = sizeof(arch->name) - 1;
		arch->name[idx] = '\0';

		local_pmu_ctx->counters_mask =
			QUADD_ARMV8_COUNTERS_MASK_PMUV3;
		local_pmu_ctx->raw_event_mask =
			QUADD_ARMV8_EVTSEL_MASK;
		local_pmu_ctx->current_map =
			quadd_armv8_pmuv3_arm_events_map;

		pmcr = armv8_pmu_pmcr_read();

		idcode = (pmcr >> QUADD_ARMV8_PMCR_IDCODE_SHIFT) &
			QUADD_ARMV8_PMCR_IDCODE_MASK;

		pr_debug("imp: %#x, idcode: %#x\n", implementer, idcode);
	}

	switch (implementer) {
	case 'A':
		strncat(arch->name, " ARM",
			sizeof(arch->name) - strlen(arch->name));
		idx = sizeof(arch->name) - 1;
		arch->name[idx] = '\0';

		if (idcode == QUADD_AA64_CPU_IDCODE_CORTEX_A53) {
			arch->type = QUADD_AA64_CPU_TYPE_CORTEX_A53;

			strncat(arch->name, " CORTEX-A53",
				sizeof(arch->name) - strlen(arch->name));

		} else if (idcode == QUADD_AA64_CPU_IDCODE_CORTEX_A57) {
			arch->type = QUADD_AA64_CPU_TYPE_CORTEX_A57;
			local_pmu_ctx->current_map =
				quadd_armv8_pmuv3_a57_events_map;

			strncat(arch->name, " CORTEX-A57",
				sizeof(arch->name) - strlen(arch->name));
		} else {
			arch->type = QUADD_AA64_CPU_TYPE_ARM;
		}
		break;
	case 'N':
		strncat(arch->name, " NVIDIA (Denver)",
			sizeof(arch->name) - strlen(arch->name));
		arch->type = QUADD_AA64_CPU_TYPE_DENVER;
		local_pmu_ctx->current_map =
			quadd_armv8_pmuv3_denver_events_map;
		break;
	default:
		strncat(arch->name,
			" Unknown implementor code",
			sizeof(arch->name) - strlen(arch->name));
		arch->type = QUADD_AA64_CPU_TYPE_UNKNOWN_IMP;
		err = -ENODEV;
		break;
	}

	arch->name[sizeof(arch->name) - 1] = '\0';
	pr_info("[%d] arch: %s, pmuver: %#x\n",
		cpuid, arch->name, arch->pmuver);

	return err;
}

struct quadd_event_source *quadd_armv8_pmu_init(void)
{
	int cpuid, err;

	for_each_possible_cpu(cpuid) {
		err = quadd_armv8_pmu_init_for_cpu(cpuid);
		if (err < 0)
			return ERR_PTR(err);
	}

	return &pmu_armv8_int;
}

void quadd_armv8_pmu_deinit(void)
{
	free_used_events();
}
