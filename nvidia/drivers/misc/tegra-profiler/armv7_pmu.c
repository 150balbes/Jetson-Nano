/*
 * drivers/misc/tegra-profiler/armv7_pmu.c
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

#include <linux/err.h>
#include <linux/bitmap.h>
#include <linux/slab.h>
#include <asm/cputype.h>
#include <asm/cpu.h>

#include <linux/tegra_profiler.h>

#include "arm_pmu.h"
#include "armv7_pmu.h"
#include "armv7_events.h"
#include "quadd.h"
#include "debug.h"

static DEFINE_PER_CPU(struct quadd_pmu_ctx, pmu_ctx);

enum {
	QUADD_ARM_CPU_TYPE_UNKNOWN,
	QUADD_ARM_CPU_TYPE_CORTEX_A5,
	QUADD_ARM_CPU_TYPE_CORTEX_A8,
	QUADD_ARM_CPU_TYPE_CORTEX_A9,
	QUADD_ARM_CPU_TYPE_CORTEX_A15,
};

struct quadd_pmu_info {
	DECLARE_BITMAP(used_cntrs, QUADD_MAX_PMU_COUNTERS);
	u32 prev_vals[QUADD_MAX_PMU_COUNTERS];
	int is_already_active;
};

struct quadd_cntrs_info {
	int pcntrs;
	int ccntr;

	raw_spinlock_t lock;
};

static DEFINE_PER_CPU(struct quadd_pmu_info, cpu_pmu_info);

static unsigned int
quadd_armv7_a9_events_map[QUADD_EVENT_HW_MAX] = {
	[QUADD_EVENT_HW_INSTRUCTIONS] =
		QUADD_ARMV7_A9_HW_EVENT_INST_OUT_OF_RENAME_STAGE,
	[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS] =
		QUADD_ARMV7_HW_EVENT_PC_WRITE,
	[QUADD_EVENT_HW_BRANCH_MISSES] =
		QUADD_ARMV7_HW_EVENT_PC_BRANCH_MIS_PRED,
	[QUADD_EVENT_HW_BUS_CYCLES] =
		QUADD_ARMV7_HW_EVENT_CLOCK_CYCLES,

	[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES] =
		QUADD_ARMV7_HW_EVENT_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES] =
		QUADD_ARMV7_HW_EVENT_DCACHE_REFILL,
	[QUADD_EVENT_HW_L1_ICACHE_MISSES] =
		QUADD_ARMV7_HW_EVENT_IFETCH_MISS,

	[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES] =
		QUADD_ARMV7_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES] =
		QUADD_ARMV7_UNSUPPORTED_EVENT,
	[QUADD_EVENT_HW_L2_ICACHE_MISSES] =
		QUADD_ARMV7_UNSUPPORTED_EVENT,
};

static unsigned int
quadd_armv7_a15_events_map[QUADD_EVENT_HW_MAX] = {
	[QUADD_EVENT_HW_INSTRUCTIONS] =
				QUADD_ARMV7_HW_EVENT_INSTR_EXECUTED,
	[QUADD_EVENT_HW_BRANCH_INSTRUCTIONS] =
				QUADD_ARMV7_A15_HW_EVENT_SPEC_PC_WRITE,
	[QUADD_EVENT_HW_BRANCH_MISSES] =
				QUADD_ARMV7_HW_EVENT_PC_BRANCH_MIS_PRED,
	[QUADD_EVENT_HW_BUS_CYCLES] = QUADD_ARMV7_HW_EVENT_BUS_CYCLES,

	[QUADD_EVENT_HW_L1_DCACHE_READ_MISSES] =
				QUADD_ARMV7_A15_HW_EVENT_L1_DCACHE_READ_REFILL,
	[QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES] =
				QUADD_ARMV7_A15_HW_EVENT_L1_DCACHE_WRITE_REFILL,
	[QUADD_EVENT_HW_L1_ICACHE_MISSES] =
				QUADD_ARMV7_HW_EVENT_IFETCH_MISS,

	[QUADD_EVENT_HW_L2_DCACHE_READ_MISSES] =
				QUADD_ARMV7_A15_HW_EVENT_L2_DCACHE_READ_REFILL,
	[QUADD_EVENT_HW_L2_DCACHE_WRITE_MISSES] =
				QUADD_ARMV7_A15_HW_EVENT_L2_DCACHE_WRITE_REFILL,
	[QUADD_EVENT_HW_L2_ICACHE_MISSES] =
				QUADD_ARMV7_UNSUPPORTED_EVENT,
};

static inline u32
armv7_pmu_pmnc_read(void)
{
	u32 val;

	/* Read Performance MoNitor Control (PMNC) register */
	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r"(val));
	return val;
}

static inline void
armv7_pmu_pmnc_write(u32 val)
{
	isb();

	/* Write Performance MoNitor Control (PMNC) register */
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : :
		     "r"(val & QUADD_ARMV7_PMNC_MASK));
}

static inline u32
armv7_pmu_cntens_read(void)
{
	u32 val;

	/* Read CouNT ENable Set (CNTENS) register */
	asm volatile("mrc p15, 0, %0, c9, c12, 1" : "=r"(val));
	return val;
}

static inline void
armv7_pmu_cntens_write(u32 val)
{
	/* Write CouNT ENable Set (CNTENS) register */
	asm volatile("mcr p15, 0, %0, c9, c12, 1" : : "r" (val));
}

static inline void
armv7_pmu_cntenc_write(u32 val)
{
	/* Write CouNT ENable Clear (CNTENC) register */
	asm volatile("mcr p15, 0, %0, c9, c12, 2" : : "r" (val));
}

static inline void
armv7_pmu_pmnxsel_write(u32 val)
{
	/* Write Performance Counter SELection (PMNXSEL) register */
	asm volatile("mcr p15, 0, %0, c9, c12, 5" : :
		     "r" (val & QUADD_ARMV7_SELECT_MASK));
	isb();
}

static inline u32
armv7_pmu_ccnt_read(void)
{
	u32 val;

	/* Read Cycle CouNT (CCNT) register */
	asm volatile ("mrc p15, 0, %0, c9, c13, 0" : "=r"(val));
	return val;
}

static inline void
armv7_pmu_ccnt_write(u32 val)
{
	/* Write Cycle CouNT (CCNT) register */
	asm volatile ("mcr p15, 0, %0, c9, c13, 0" : : "r"(val));
}

static inline u32
armv7_pmu_pmcnt_read(void)
{
	u32 val;

	/* Read Performance Monitor CouNT (PMCNTx) registers */
	asm volatile ("mrc p15, 0, %0, c9, c13, 2" : "=r"(val));
	return val;
}

static inline void
armv7_pmu_pmcnt_write(u32 val)
{
	/* Write Performance Monitor CouNT (PMCNTx) registers */
	asm volatile ("mcr p15, 0, %0, c9, c13, 2" : : "r"(val));
}

static inline void
armv7_pmu_evtsel_write(u32 event)
{
	/* Write Event SELection (EVTSEL) register */
	asm volatile("mcr p15, 0, %0, c9, c13, 1" : :
		     "r" (event & QUADD_ARMV7_EVTSEL_MASK));
}

static inline u32
armv7_pmu_intens_read(void)
{
	u32 val;

	/* Read INTerrupt ENable Set (INTENS) register */
	asm volatile ("mrc p15, 0, %0, c9, c14, 1" : "=r"(val));
	return val;
}

static inline void
armv7_pmu_intens_write(u32 val)
{
	/* Write INTerrupt ENable Set (INTENS) register */
	asm volatile ("mcr p15, 0, %0, c9, c14, 1" : : "r"(val));
}

static inline void
armv7_pmu_intenc_write(u32 val)
{
	/* Write INTerrupt ENable Clear (INTENC) register */
	asm volatile ("mcr p15, 0, %0, c9, c14, 2" : : "r"(val));
}

static void enable_counter(int idx)
{
	armv7_pmu_cntens_write(1UL << idx);
}

static void disable_counter(int idx)
{
	armv7_pmu_cntenc_write(1UL << idx);
}

static void select_counter(unsigned int counter)
{
	armv7_pmu_pmnxsel_write(counter);
}

static int is_pmu_enabled(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 pmnc = armv7_pmu_pmnc_read();

	if (pmnc & QUADD_ARMV7_PMNC_E) {
		u32 cnten = armv7_pmu_cntens_read();

		cnten &= local_pmu_ctx->counters_mask | QUADD_ARMV7_CCNT;
		return cnten ? 1 : 0;
	}

	return 0;
}

static u32 read_counter(int idx)
{
	u32 val;

	if (idx == QUADD_ARMV7_CCNT_BIT) {
		val = armv7_pmu_ccnt_read();
	} else {
		select_counter(idx);
		val = armv7_pmu_pmcnt_read();
	}

	return val;
}

static void write_counter(int idx, u32 value)
{
	if (idx == QUADD_ARMV7_CCNT_BIT) {
		armv7_pmu_ccnt_write(value);
	} else {
		select_counter(idx);
		armv7_pmu_pmcnt_write(value);
	}
}

static int
get_free_counters(unsigned long *bitmap, int nbits, int *ccntr)
{
	int cc;
	u32 cntens;

	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	cntens = armv7_pmu_cntens_read();
	cntens = ~cntens & (local_pmu_ctx->counters_mask | QUADD_ARMV7_CCNT);

	bitmap_zero(bitmap, nbits);
	bitmap_copy(bitmap, (unsigned long *)&cntens,
		    BITS_PER_BYTE * sizeof(u32));

	cc = (cntens & QUADD_ARMV7_CCNT) ? 1 : 0;

	if (ccntr)
		*ccntr = cc;

	return bitmap_weight(bitmap, BITS_PER_BYTE * sizeof(u32)) - cc;
}

static u32
armv7_pmu_adjust_value(u32 value, const struct quadd_event *event)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	if (event->type != QUADD_EVENT_TYPE_HARDWARE)
		return value;

	/*
	 * Cortex A8/A9: l1 cache performance counters
	 * don't differentiate between read and write data accesses/misses,
	 * so currently we are divided by two
	 */
	if (local_pmu_ctx->l1_cache_rw &&
	    (local_pmu_ctx->arch.type == QUADD_ARM_CPU_TYPE_CORTEX_A8 ||
	    local_pmu_ctx->arch.type == QUADD_ARM_CPU_TYPE_CORTEX_A9) &&
	    (event->id == QUADD_EVENT_HW_L1_DCACHE_READ_MISSES ||
	    event->id == QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES)) {
		return value / 2;
	}

	return value;
}

static void __maybe_unused
disable_interrupt(int idx)
{
	armv7_pmu_intenc_write(1UL << idx);
}

static void
disable_all_interrupts(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 val = QUADD_ARMV7_CCNT | local_pmu_ctx->counters_mask;

	armv7_pmu_intenc_write(val);
}

static void
armv7_pmnc_reset_overflow_flags(void)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 val = QUADD_ARMV7_CCNT | local_pmu_ctx->counters_mask;

	asm volatile("mcr p15, 0, %0, c9, c12, 3" : : "r" (val));
}

static void
select_event(unsigned int idx, unsigned int event)
{
	select_counter(idx);
	armv7_pmu_evtsel_write(event);
}

static void disable_all_counters(void)
{
	u32 val;
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	/* Disable all counters */
	val = armv7_pmu_pmnc_read();
	if (val & QUADD_ARMV7_PMNC_E)
		armv7_pmu_pmnc_write(val & ~QUADD_ARMV7_PMNC_E);

	armv7_pmu_cntenc_write(QUADD_ARMV7_CCNT | local_pmu_ctx->counters_mask);
}

static void enable_all_counters(void)
{
	u32 val;

	/* Enable all counters */
	val = armv7_pmu_pmnc_read();
	val |= QUADD_ARMV7_PMNC_E | QUADD_ARMV7_PMNC_X;
	armv7_pmu_pmnc_write(val);
}

static void reset_all_counters(void)
{
	u32 val;

	val = armv7_pmu_pmnc_read();
	val |= QUADD_ARMV7_PMNC_P | QUADD_ARMV7_PMNC_C;
	armv7_pmu_pmnc_write(val);
}

static void quadd_init_pmu(void)
{
	armv7_pmnc_reset_overflow_flags();
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
	int cpu;

	for_each_possible_cpu(cpu) {
		struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpu);

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
	DECLARE_BITMAP(free_bitmap, QUADD_MAX_PMU_COUNTERS);
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 *prevp = pi->prev_vals;
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

	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		int index;

		*prevp++ = 0;

		event = ei->hw_value;

		if (is_cpu_cycles(&ei->event)) {
			if (!ccntr) {
				pr_err_once("Error: cpu cycles counter is already occupied\n");
				return;
			}
			index = QUADD_ARMV7_CCNT_BIT;
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
	u32 val, prev_val, delta;
	int idx = 0, i = 0;
	struct quadd_pmu_info *pi = this_cpu_ptr(&cpu_pmu_info);
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);
	u32 *prevp = pi->prev_vals;
	struct quadd_pmu_event_info *ei;

	if (bitmap_empty(pi->used_cntrs, QUADD_MAX_PMU_COUNTERS)) {
		pr_err_once("Error: counters were not initialized\n");
		return 0;
	}

	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		int index;

		if (is_cpu_cycles(&ei->event)) {
			if (!test_bit(QUADD_ARMV7_CCNT_BIT, pi->used_cntrs)) {
				pr_err_once("Error: ccntr is not used\n");
				return 0;
			}
			index = QUADD_ARMV7_CCNT_BIT;
		} else {
			index = find_next_bit(pi->used_cntrs,
					      QUADD_MAX_PMU_COUNTERS, idx);
			idx = index + 1;

			if (index >= QUADD_MAX_PMU_COUNTERS) {
				pr_err_once("Error: perf counter is not used\n");
				return 0;
			}
		}

		val = read_counter(index);
		val = armv7_pmu_adjust_value(val, &ei->event);

		events->event_source = QUADD_EVENT_SOURCE_PMU;
		events->event = ei->event;
		events->max_count = U32_MAX;

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
	int free_pcntrs, err;
	int i, nr_l1_r = 0, nr_l1_w = 0;
	struct quadd_cntrs_info free_ci;

	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpuid);

	local_pmu_ctx->l1_cache_rw = 0;

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

	on_each_cpu(__get_free_counters, &free_ci, 1);

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
			ei->hw_value = QUADD_ARMV7_CPU_CYCLE_EVENT;
			if (!free_ci.ccntr) {
				pr_err("Error: cpu cycles counter is already occupied\n");
				err = -EBUSY;
				goto out_free;
			}
		} else {
			if (!free_pcntrs--) {
				pr_err("Error: too many performance events\n");
				err = -ENOSPC;
				goto out_free;
			}

			ei->hw_value = (type == QUADD_EVENT_TYPE_RAW) ? id :
				local_pmu_ctx->current_map[id];
		}

		ei->event = events[i];

		if (type == QUADD_EVENT_TYPE_HARDWARE) {
			if (id == QUADD_EVENT_HW_L1_DCACHE_READ_MISSES)
				nr_l1_r++;
			else if (id == QUADD_EVENT_HW_L1_DCACHE_WRITE_MISSES)
				nr_l1_w++;
		}

		pr_debug("[%d] Event has been added: id: %#x (%s), hw value: %#x\n",
			 cpuid, id, type == QUADD_EVENT_TYPE_RAW ? "raw" : "hw",
			 ei->hw_value);
	}

	if (nr_l1_r > 0 && nr_l1_w > 0)
		local_pmu_ctx->l1_cache_rw = 1;

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
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	if (!local_pmu_ctx->current_map)
		return 0;

	max_events = min_t(int, QUADD_EVENT_HW_MAX, max_events);

	for (i = 0; i < max_events; i++) {
		unsigned int event = local_pmu_ctx->current_map[i];

		if (event != QUADD_ARMV7_UNSUPPORTED_EVENT) {
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
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	list_for_each_entry(ei, &local_pmu_ctx->used_events, list) {
		events[i++] = ei->event;

		if (i >= max_events)
			break;
	}

	return i;
}

static struct quadd_arch_info *get_arch(int cpuid)
{
	struct quadd_pmu_ctx *local_pmu_ctx = this_cpu_ptr(&pmu_ctx);

	return local_pmu_ctx->current_map ? &local_pmu_ctx->arch : NULL;
}

static struct quadd_event_source pmu_armv7_int = {
	.name			= "armv7_pmu",
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

static int quadd_armv7_pmu_init_for_cpu(int cpu)
{
	int err = 0;
	unsigned long cpuid, cpu_implementer, part_number;

	struct cpuinfo_arm *local_cpu_data = &per_cpu(cpu_data, cpu);
	struct quadd_pmu_ctx *local_pmu_ctx = &per_cpu(pmu_ctx, cpu);
	struct quadd_arch_info *arch = &local_pmu_ctx->arch;

	arch->type = QUADD_ARM_CPU_TYPE_UNKNOWN;
	arch->pmuver_is_set = 0;
	local_pmu_ctx->current_map = NULL;

	strncpy(arch->name, "Unknown", sizeof(arch->name));

	INIT_LIST_HEAD(&local_pmu_ctx->used_events);

	cpuid = local_cpu_data->cpuid;

	if (!cpuid)
		return 0;

	cpu_implementer = cpuid >> 24;
	part_number = cpuid & 0xFFF0;

	if (cpu_implementer == ARM_CPU_IMP_ARM) {
		switch (part_number) {
		case ARM_CPU_PART_CORTEX_A9:
			arch->type = QUADD_ARM_CPU_TYPE_CORTEX_A9;
			strncpy(arch->name, "Cortex A9", sizeof(arch->name));

			local_pmu_ctx->counters_mask =
				QUADD_ARMV7_COUNTERS_MASK_CORTEX_A9;
			local_pmu_ctx->raw_event_mask =
				QUADD_ARMV7_EVTSEL_MASK;
			local_pmu_ctx->current_map = quadd_armv7_a9_events_map;
			break;

		case ARM_CPU_PART_CORTEX_A15:
			arch->type = QUADD_ARM_CPU_TYPE_CORTEX_A15;
			strncpy(arch->name, "Cortex A15", sizeof(arch->name));

			local_pmu_ctx->counters_mask =
				QUADD_ARMV7_COUNTERS_MASK_CORTEX_A15;
			local_pmu_ctx->raw_event_mask =
				QUADD_ARMV7_EVTSEL_MASK;
			local_pmu_ctx->current_map = quadd_armv7_a15_events_map;
			break;

		default:
			arch->type = QUADD_ARM_CPU_TYPE_UNKNOWN;
			local_pmu_ctx->current_map = NULL;
			err = -ENODEV;
			break;
		}
	} else {
		err = -ENODEV;
	}

	arch->name[sizeof(arch->name) - 1] = '\0';
	pr_info("[%d] arch: %s\n", cpu, arch->name);

	return err;
}

struct quadd_event_source *quadd_armv7_pmu_init(void)
{
	int cpuid, err;

	for_each_possible_cpu(cpuid) {
		err = quadd_armv7_pmu_init_for_cpu(cpuid);
		if (err < 0)
			return ERR_PTR(err);
	}

	return &pmu_armv7_int;
}

void quadd_armv7_pmu_deinit(void)
{
	free_used_events();
}
