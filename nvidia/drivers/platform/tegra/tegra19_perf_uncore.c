 /*
 * Carmel Uncore PMU support
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 #include <linux/version.h>

 #if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)

/*
* perf events refactored include structure starting with 4.9
* This driver is only valid with kernel version 4.9 and greater
*/
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#else
#include <linux/sched_clock.h>
#endif

#include <asm/irq_regs.h>
#include <asm/sysreg.h>

#include <soc/tegra/chip-id.h>

// Global registers
#define SYS_NV_PMSELR_EL0     sys_reg(3, 3, 15, 5, 1)

// Unit registers
#define SYS_NV_PMCNTENSET_EL0 sys_reg(3, 3, 15, 4, 0)
#define SYS_NV_PMCNTENCLR_EL0 sys_reg(3, 3, 15, 4, 1)
#define SYS_NV_PMOVSSET_EL0   sys_reg(3, 3, 15, 4, 2)
#define SYS_NV_PMOVSCLR_EL0   sys_reg(3, 3, 15, 4, 3)
#define SYS_NV_PMCR_EL0       sys_reg(3, 3, 15, 4, 4)
#define SYS_NV_PMINTENSET_EL1 sys_reg(3, 0, 15, 2, 0)
#define SYS_NV_PMINTENCLR_EL1 sys_reg(3, 0, 15, 2, 1)

// Counter registers
#define SYS_NV_PMEVCNTR0_EL0  sys_reg(3, 3, 15, 0, 0)
#define SYS_NV_PMEVCNTR1_EL0  sys_reg(3, 3, 15, 0, 1)
#define SYS_NV_PMEVTYPER0_EL0 sys_reg(3, 3, 15, 2, 0)
#define SYS_NV_PMEVTYPER1_EL0 sys_reg(3, 3, 15, 2, 1)

#ifdef PMCDBG
#define PMCPRINT(msg, ...) pr_err("carmel_pmu CPU%d: %s:%d "msg, smp_processor_id(), __func__, __LINE__, ## __VA_ARGS__)
#else
#define PMCPRINT(msg, ...)
#endif


// NV_PMCR: config reg
#define CARMEL_PMCR_E		(0x1 << 0) /* Enable all counters */
#define CARMEL_PMCR_P		(0x1 << 1) /* Reset all counters */

// NV_PMSELR: unit selection register
#define PMSELR_GROUP_SCF	0x0
#define PMSELR_GROUP_L2		0x100

// L2 units in the Carmel CCPLEX
#define NUM_L2S		0x4

// Carmel uncore perfmon supports two counters per unit
#define UNIT_CTRS	0x2

// All uncore counters are 32 bit
#define MAX_PERIOD (1ULL << 32)

// Counters reading CTR_INVAL are in a powered down unit
#define CTR_INVAL 0xffffffff

/*
 * Format for raw events is 0xEEU
 * EE: event number
 * U:  unit (0-3 for L2s, 4 for uncore)
 * eg. all L2D_CACHE: perf stat -e r160,r161,r162,r163
 */
#define CARMEL_CONFIG_UNIT(_config)		(_config & 0xf)
#define CARMEL_CONFIG_EVENT(_config)	(_config >> 4)

#define L2D_CACHE					0x16
#define L2D_CACHE_REFILL			0x17
#define L2D_CACHE_WB				0x18

#define BUS_ACCESS					0x19
#define BUS_CYCLES					0x1D
#define L3D_CACHE_ALLOCATE			0x29
#define L3D_CACHE_REFILL			0x2A
#define L3D_CACHE					0x2B
#define L3D_CACHE_WB				0x2C

#define L2D_CACHE_LD				0x50
#define L2D_CACHE_ST				0x51
#define L2D_CACHE_REFILL_LD			0x52
#define L2D_CACHE_REFILL_ST			0x53
#define L2D_CACHE_REFILL_VICTIM		0x56

#define L2D_PREFETCH_C0				0xC5
#define L2D_PREFETCH_C1				0xC6

#define NV_INT_START				0x200
#define NV_INT_END					0x208


static const u32 pmevcntr[] = { SYS_NV_PMEVCNTR0_EL0, SYS_NV_PMEVCNTR1_EL0 };
static const u32 pmevtyper[] = { SYS_NV_PMEVTYPER0_EL0, SYS_NV_PMEVTYPER1_EL0 };

static void sys_counter_write(u32 reg, u32 val)
{
	switch(reg) {
		case SYS_NV_PMEVCNTR0_EL0: write_sysreg_s(val, SYS_NV_PMEVCNTR0_EL0); return;
		case SYS_NV_PMEVCNTR1_EL0: write_sysreg_s(val, SYS_NV_PMEVCNTR1_EL0); return;
		case SYS_NV_PMEVTYPER0_EL0: write_sysreg_s(val, SYS_NV_PMEVTYPER0_EL0); return;
		case SYS_NV_PMEVTYPER1_EL0: write_sysreg_s(val, SYS_NV_PMEVTYPER1_EL0); return;
		default:
			WARN(1, "Illegal counter write\n");
			return;
	}
}

static u32 sys_counter_read(u32 reg)
{
	switch(reg) {
		case SYS_NV_PMEVCNTR0_EL0: return read_sysreg_s(SYS_NV_PMEVCNTR0_EL0);
		case SYS_NV_PMEVCNTR1_EL0: return read_sysreg_s(SYS_NV_PMEVCNTR1_EL0);
		case SYS_NV_PMEVTYPER0_EL0: return read_sysreg_s(SYS_NV_PMEVTYPER0_EL0);
		case SYS_NV_PMEVTYPER1_EL0: return read_sysreg_s(SYS_NV_PMEVTYPER1_EL0);
		default:
			WARN(1, "Illegal counter read\n");
			return 0;
	}
}

struct uncore_unit {
	u32 unit_id;
	u32 nv_pmselr;
	struct list_head next;
	struct perf_event *events[UNIT_CTRS];
	DECLARE_BITMAP(used_ctrs, UNIT_CTRS);
};

struct uncore_pmu {
	struct platform_device *pdev;
	struct pmu pmu;
	struct uncore_unit clusters[NUM_L2S];
	struct uncore_unit scf;
	struct list_head units;
	DECLARE_BITMAP(used_units, NUM_L2S + 1);
	struct uncore_unit *cur_unit;
};

static inline struct uncore_pmu *to_uncore_pmu(struct pmu *pmu)
{
	return container_of(pmu, struct uncore_pmu, pmu);
}

static inline void set_unit(struct uncore_pmu *uncore_pmu, struct uncore_unit *uncore_unit)
{
	if (uncore_pmu->cur_unit != uncore_unit) {
		uncore_pmu->cur_unit = uncore_unit;
		write_sysreg_s(uncore_unit->nv_pmselr, SYS_NV_PMSELR_EL0);
	}
}

static inline struct uncore_unit *get_unit(struct uncore_pmu *uncore_pmu, u32 config_unit)
{
	switch (config_unit) {
		case 0 ... (NUM_L2S - 1):
			return &uncore_pmu->clusters[config_unit];
		case NUM_L2S:
			return &uncore_pmu->scf;
		default:
			return NULL;
	}
}

static void carmel_uncore_pmu_enable(struct pmu *pmu)
{
	struct uncore_unit *uncore_unit;
	struct uncore_pmu *uncore_pmu = to_uncore_pmu(pmu);

	list_for_each_entry(uncore_unit, &uncore_pmu->units, next) {
		if (test_bit(uncore_unit->unit_id, uncore_pmu->used_units)) {
			set_unit(uncore_pmu, uncore_unit);
			write_sysreg_s(CARMEL_PMCR_E, SYS_NV_PMCR_EL0);
		}
	}
}

static void carmel_uncore_pmu_disable(struct pmu *pmu)
{
	struct uncore_unit *uncore_unit;
	struct uncore_pmu *uncore_pmu = to_uncore_pmu(pmu);

	list_for_each_entry(uncore_unit, &uncore_pmu->units, next) {
		if (test_bit(uncore_unit->unit_id, uncore_pmu->used_units)) {
			set_unit(uncore_pmu, uncore_unit);
			write_sysreg_s(0, SYS_NV_PMCR_EL0);
		}
	}
}

/*
 * event_init: Verify this PMU can handle the desired event
 */
static int carmel_uncore_event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	u32 config_unit;
	u32 config_event;

	// Only support CPU tracking events (not task tracking)
	if (event->cpu == -1)
		return -ENOENT;

	config_unit = CARMEL_CONFIG_UNIT(event->attr.config);
	config_event = CARMEL_CONFIG_EVENT(event->attr.config);

	// Verify event is for this PMU and targets correct unit type
	switch (config_event) {
		case L2D_CACHE ... L2D_CACHE_WB:
		case L2D_CACHE_LD ... L2D_CACHE_REFILL_ST:
		case L2D_CACHE_REFILL_VICTIM:
		case L2D_PREFETCH_C0 ... L2D_PREFETCH_C1:
			if (config_unit >= NUM_L2S)
				return -ENOENT;
			break;
		case BUS_ACCESS:
		case BUS_CYCLES:
		case L3D_CACHE_ALLOCATE ... L3D_CACHE_WB:
			if (config_unit != NUM_L2S)
				return -ENOENT;
			break;
		case NV_INT_START ... NV_INT_END:
			break;
		default:
			return -ENOENT;
			break;
	}

	// Event is valid, hw not allocated yet
	hwc->idx = -1;
	hwc->config_base = event->attr.config;

	return 0;
}

/*
 * set_period: driver needs to be pointed at the right
 * uncore unit when this is called. Currently used by
 * carmel_uncore_event_add(), carmel_uncore_event_start(),
 * and carmel_uncore_handle_irq()
 */
static int carmel_uncore_event_set_period(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	s64 left = local64_read(&hwc->period_left);
	s64 period = hwc->sample_period;
	int idx = hwc->idx;
	int ovf = 0;
	u64 value;

	/*
	 * By the time we fielded the interrupt, the counter
	 * underflowed by a more than the desired period
	 */
	if (unlikely(left <= -period)) {
		left = period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ovf = 1;
	}
	// Underflow by less than or equal to desired period
	else if (unlikely(left <= 0)) {
		left += period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ovf = 1;
	}

	value = (u64)(-left) & (MAX_PERIOD - 1);

	local64_set(&hwc->prev_count, value);
	sys_counter_write(pmevcntr[idx], value);

	perf_event_update_userpage(event);

	return ovf;
}
/*
 * event_init has verified the event is valid, now event_add
 * allocates hardware resouces for the event (and optionally starts counting)
 */
static int carmel_uncore_event_add(struct perf_event *event, int flags)
{
	struct uncore_pmu *uncore_pmu;
	struct uncore_unit *uncore_unit;
	struct hw_perf_event *hwc = &event->hw;
	u32 unit;
	u32 idx;
	u32 ctr_val;

	// CPU0 does all uncore counting
	if (event->cpu != 0)
		return 0;

	uncore_pmu = to_uncore_pmu(event->pmu);
	unit = CARMEL_CONFIG_UNIT(event->attr.config);
	uncore_unit = get_unit(uncore_pmu, unit);

	set_unit(uncore_pmu, uncore_unit);

	idx = find_first_zero_bit(uncore_unit->used_ctrs, UNIT_CTRS);
	if (idx == UNIT_CTRS)
		// All counters are in use
		return -EOPNOTSUPP;

	// Program unit's event register
	sys_counter_write(pmevtyper[idx], CARMEL_CONFIG_EVENT(event->attr.config));

	sys_counter_write(pmevcntr[idx], 0);
	ctr_val = sys_counter_read(pmevcntr[idx]);

	// If counter is stuck at CTR_INVAL, the unit is powered down
	// (ie both cores in the L2 cluster are in C7)
	if (ctr_val == CTR_INVAL)
		return -ENODEV;

	set_bit(idx, uncore_unit->used_ctrs);
	set_bit(uncore_unit->unit_id, uncore_pmu->used_units);

	uncore_unit->events[idx] = event;

	hwc->idx = idx;
	hwc->state = PERF_HES_UPTODATE;

	carmel_uncore_event_set_period(event);

	if (flags & PERF_EF_START) {
		write_sysreg_s(BIT(idx), SYS_NV_PMINTENSET_EL1);
		write_sysreg_s(BIT(idx), SYS_NV_PMCNTENSET_EL0);
	} else
		hwc->state |= PERF_HES_STOPPED;

	return 0;
}

/*
 * event_update: driver needs to be pointed at the right
 * uncore unit when this is called. Currently used by
 * carmel_uncore_event_del() and carmel_uncore_event_read()
 */
static void carmel_uncore_event_update(struct perf_event *event, bool ovf)
{

	struct hw_perf_event *hwc = &event->hw;
	u32 idx = hwc->idx;
	u64 delta;
	u64 prev;
	u64 now;

	do {
		prev = local64_read(&hwc->prev_count);
		now = sys_counter_read(pmevcntr[idx]);
	} while (local64_cmpxchg(&hwc->prev_count, prev, now) != prev);

	if (prev > now)
		delta = MAX_PERIOD - prev + now;
	else
		// either an incremental read, or fielding an IRQ from a ctr overflow
		delta = now - prev + (ovf ? MAX_PERIOD : 0);

	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);
}

static void carmel_uncore_event_del(struct perf_event *event, int flags)
{
	struct uncore_pmu *uncore_pmu;
	struct uncore_unit *uncore_unit;
	struct hw_perf_event *hwc = &event->hw;
	u32 unit;
	u32 idx = hwc->idx;

	// CPU0 does all uncore counting
	if (event->cpu != 0)
		return;

	uncore_pmu = to_uncore_pmu(event->pmu);
	unit = CARMEL_CONFIG_UNIT(event->attr.config);
	uncore_unit = get_unit(uncore_pmu, unit);

	set_unit(uncore_pmu, uncore_unit);

	write_sysreg_s(BIT(idx), SYS_NV_PMINTENCLR_EL1);
	write_sysreg_s(BIT(idx), SYS_NV_PMCNTENCLR_EL0);

	carmel_uncore_event_update(event, false);
	sys_counter_write(pmevcntr[idx], 0);

	clear_bit(idx, uncore_unit->used_ctrs);
	uncore_unit->events[idx] = NULL;

	if (find_first_bit(uncore_unit->used_ctrs, UNIT_CTRS) == UNIT_CTRS)
		clear_bit(uncore_unit->unit_id, uncore_pmu->used_units);

	perf_event_update_userpage(event);

}

static void carmel_uncore_event_start(struct perf_event *event, int flags) {

	struct uncore_pmu *uncore_pmu;
	struct uncore_unit *uncore_unit;
	struct hw_perf_event *hwc = &event->hw;
	u32 unit;
	u32 idx = hwc->idx;

	// CPU0 does all uncore counting
	if (event->cpu != 0)
		return;


	uncore_pmu = to_uncore_pmu(event->pmu);
	unit = CARMEL_CONFIG_UNIT(event->attr.config);
	uncore_unit = get_unit(uncore_pmu, unit);

	set_unit(uncore_pmu, uncore_unit);

	hwc->state = 0;

	carmel_uncore_event_set_period(event);

	// Enable interrupt, start counter
	write_sysreg_s(BIT(idx), SYS_NV_PMINTENSET_EL1);
	write_sysreg_s(BIT(idx), SYS_NV_PMCNTENSET_EL0);
}

static void carmel_uncore_event_stop(struct perf_event *event, int flags)
{
	struct uncore_pmu *uncore_pmu;
	struct uncore_unit *uncore_unit;
	struct hw_perf_event *hwc = &event->hw;
	u32 unit;
	u32 idx = hwc->idx;

	// CPU0 does all uncore counting
	if (event->cpu != 0)
		return;

	uncore_pmu = to_uncore_pmu(event->pmu);
	unit = CARMEL_CONFIG_UNIT(event->attr.config);
	uncore_unit = get_unit(uncore_pmu, unit);

	set_unit(uncore_pmu, uncore_unit);

	if (hwc->state & PERF_HES_STOPPED)
		return;

	// Stop counter, disable interrupt
	write_sysreg_s(BIT(idx), SYS_NV_PMCNTENCLR_EL0);
	write_sysreg_s(BIT(idx), SYS_NV_PMINTENCLR_EL1);

	if (flags & PERF_EF_UPDATE)
		carmel_uncore_event_update(event, false);

	hwc->state |= PERF_HES_STOPPED | PERF_HES_UPTODATE;
}

static void carmel_uncore_event_read(struct perf_event *event)
{
	struct uncore_pmu *uncore_pmu;
	struct uncore_unit *uncore_unit;
	u32 unit;

	// CPU0 does all uncore counting
	if (event->cpu != 0)
		return;

	uncore_pmu = to_uncore_pmu(event->pmu);
	unit = CARMEL_CONFIG_UNIT(event->attr.config);
	uncore_unit = get_unit(uncore_pmu, unit);

	set_unit(uncore_pmu, uncore_unit);

	carmel_uncore_event_update(event, false);
}

/*
 * Handle counter overflows. We have one interrupt for all uncore
 * counters, so iterate through active units to find overflow bits
 */
static irqreturn_t carmel_uncore_handle_irq(int irq_num, void *data)
{
	struct uncore_pmu *uncore_pmu = data;
	struct uncore_unit *uncore_unit;
	struct pt_regs *regs = get_irq_regs();
	u64 start_clock, finish_clock;

	start_clock = sched_clock();

	list_for_each_entry(uncore_unit, &uncore_pmu->units, next) {
		u32 inten;
		u32 ovf;
		u32 idx;

		if (!test_bit(uncore_unit->unit_id, uncore_pmu->used_units))
			continue;

		set_unit(uncore_pmu, uncore_unit);

		inten = read_sysreg_s(SYS_NV_PMINTENCLR_EL1);
		ovf = read_sysreg_s(SYS_NV_PMOVSCLR_EL0);

		for_each_set_bit(idx, uncore_unit->used_ctrs, UNIT_CTRS) {

			// Find overflowed counters in unit
			if (BIT(idx) & inten & ovf) {
				struct perf_event *event = uncore_unit->events[idx];
				struct hw_perf_event *hwc = &event->hw;
				struct perf_sample_data data;

				carmel_uncore_event_update(event, true);

				perf_sample_data_init(&data, 0, hwc->last_period);

				if(!carmel_uncore_event_set_period(event))
					continue;

				if (perf_event_overflow(event, &data, regs))
					carmel_uncore_event_stop(event, 0);

			}
		}
		write_sysreg_s(ovf, SYS_NV_PMOVSCLR_EL0);
		write_sysreg_s(inten, SYS_NV_PMINTENCLR_EL1);
		write_sysreg_s(inten, SYS_NV_PMINTENSET_EL1);
	}

	finish_clock = sched_clock();
	perf_sample_event_took(finish_clock - start_clock);

	irq_work_run();

	return IRQ_HANDLED;
}

static ssize_t carmel_uncore_event_sysfs_show(struct device *dev,
											  struct device_attribute *attr, char *page)
{
	struct perf_pmu_events_attr *pmu_attr;
	pmu_attr = container_of(attr, struct perf_pmu_events_attr, attr);
	return sprintf(page, "event=0x%03llx\n", pmu_attr->id);
}

#define CARMEL_EVENT_ATTR(name, config) \
	PMU_EVENT_ATTR(name, carmel_event_attr_##name, \
				   config, carmel_uncore_event_sysfs_show)

CARMEL_EVENT_ATTR(l2d, L2D_CACHE);
CARMEL_EVENT_ATTR(l2d_refill, L2D_CACHE_REFILL);
CARMEL_EVENT_ATTR(l2d_cache_wb, L2D_CACHE_WB);
CARMEL_EVENT_ATTR(bus_access, BUS_ACCESS);
CARMEL_EVENT_ATTR(bus_cycles, BUS_CYCLES);
CARMEL_EVENT_ATTR(l3d_cache_allocate, L3D_CACHE_ALLOCATE);
CARMEL_EVENT_ATTR(l3d_cache_refill, L3D_CACHE_REFILL);
CARMEL_EVENT_ATTR(l3d_cache, L3D_CACHE);
CARMEL_EVENT_ATTR(l3d_cache_wb, L3D_CACHE_WB);
CARMEL_EVENT_ATTR(l2d_ld, L2D_CACHE_LD);
CARMEL_EVENT_ATTR(l2d_st, L2D_CACHE_ST);
CARMEL_EVENT_ATTR(l2d_refill_ld, L2D_CACHE_REFILL_LD);
CARMEL_EVENT_ATTR(l2d_refill_st, L2D_CACHE_REFILL_ST);
CARMEL_EVENT_ATTR(l2d_refill_victim, L2D_CACHE_REFILL_VICTIM);
CARMEL_EVENT_ATTR(l2d_prefetch_c0, L2D_PREFETCH_C0);
CARMEL_EVENT_ATTR(l2d_prefetch_c1, L2D_PREFETCH_C1);


static struct attribute *carmel_uncore_pmu_events[] = {
	&carmel_event_attr_l2d.attr.attr,
	&carmel_event_attr_l2d_refill.attr.attr,
	&carmel_event_attr_l2d_cache_wb.attr.attr,
	&carmel_event_attr_bus_access.attr.attr,
	&carmel_event_attr_bus_cycles.attr.attr,
	&carmel_event_attr_l3d_cache_allocate.attr.attr,
	&carmel_event_attr_l3d_cache_refill.attr.attr,
	&carmel_event_attr_l3d_cache.attr.attr,
	&carmel_event_attr_l3d_cache_wb.attr.attr,
	&carmel_event_attr_l2d_ld.attr.attr,
	&carmel_event_attr_l2d_st.attr.attr,
	&carmel_event_attr_l2d_refill_ld.attr.attr,
	&carmel_event_attr_l2d_refill_st.attr.attr,
	&carmel_event_attr_l2d_refill_victim.attr.attr,
	&carmel_event_attr_l2d_prefetch_c0.attr.attr,
	&carmel_event_attr_l2d_prefetch_c1.attr.attr,
	NULL,
};

static struct attribute_group carmel_uncore_pmu_events_group = {
	.name = "events",
	.attrs = carmel_uncore_pmu_events,
};

PMU_FORMAT_ATTR(unit,	"config:0-3");
PMU_FORMAT_ATTR(event,	"config:4-15");

static struct attribute *carmel_uncore_pmu_formats[] = {
	&format_attr_event.attr,
	&format_attr_unit.attr,
	NULL,
};

static struct attribute_group carmel_uncore_pmu_format_group = {
	.name = "format",
	.attrs = carmel_uncore_pmu_formats,
};

static const struct attribute_group *carmel_uncore_pmu_attr_grps[] = {
	&carmel_uncore_pmu_events_group,
	&carmel_uncore_pmu_format_group,
	NULL,
};

static int carmel_pmu_device_probe(struct platform_device *pdev)
{
	struct uncore_pmu *uncore_pmu;
	int err;
	int irq;
	u32 i;

	uncore_pmu = devm_kzalloc(&pdev->dev, sizeof(*uncore_pmu), GFP_KERNEL);
	if(!uncore_pmu)
		return -ENOMEM;

	INIT_LIST_HEAD(&uncore_pmu->units);

	for (i = 0; i < NUM_L2S; i++) {
		list_add(&uncore_pmu->clusters[i].next, &uncore_pmu->units);
		uncore_pmu->clusters[i].nv_pmselr = PMSELR_GROUP_L2 + i;
		uncore_pmu->clusters[i].unit_id = i;
	}

	list_add(&uncore_pmu->scf.next, &uncore_pmu->units);
	uncore_pmu->scf.nv_pmselr = PMSELR_GROUP_SCF;
	uncore_pmu->scf.unit_id = NUM_L2S;

	platform_set_drvdata(pdev, uncore_pmu);
	uncore_pmu->pmu = (struct pmu) {
		.name			= "carmel_pmu",
		.task_ctx_nr	= perf_invalid_context,
		.pmu_enable		= carmel_uncore_pmu_enable,
		.pmu_disable	= carmel_uncore_pmu_disable,
		.event_init		= carmel_uncore_event_init,
		.add			= carmel_uncore_event_add,
		.del			= carmel_uncore_event_del,
		.start			= carmel_uncore_event_start,
		.stop			= carmel_uncore_event_stop,
		.read			= carmel_uncore_event_read,
		.attr_groups	= carmel_uncore_pmu_attr_grps,
		.type			= PERF_TYPE_HARDWARE,
	};

	uncore_pmu->pdev = pdev;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to find IRQ for Carmel PMU\n");
		return irq;
	}

	err = devm_request_irq(&pdev->dev, irq, carmel_uncore_handle_irq,
							IRQF_NOBALANCING, "carmel-pmu", uncore_pmu);

	if (err) {
		dev_err(&pdev->dev, "Unable to register IRQ for Carmel PMU\n");
		return err;
	}

	err = perf_pmu_register(&uncore_pmu->pmu, uncore_pmu->pmu.name, -1);
	if (err) {
		dev_err(&pdev->dev, "Error %d registering Carmel PMU\n", err);
		return err;
	}

	dev_info(&pdev->dev, "Registered Carmel PMU\n");

	return 0;
}

static const struct of_device_id carmel_pmu_of_device_ids[] = {
	{.compatible = "nvidia,carmel-pmu"},
	{},
};

static struct platform_driver carmel_pmu_driver = {
	.driver = {
		.name = "carmel-pmu-drv",
		.of_match_table = carmel_pmu_of_device_ids,
	},
	.probe = carmel_pmu_device_probe,
};

static int __init register_pmu_driver(void)
{
	if (tegra_platform_is_silicon())
		return platform_driver_register(&carmel_pmu_driver);
	return 0;
}
device_initcall(register_pmu_driver);
#endif
