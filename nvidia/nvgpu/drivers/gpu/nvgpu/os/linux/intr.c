/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <trace/events/gk20a.h>
#include <linux/irqreturn.h>

#include <nvgpu/gk20a.h>

#include <nvgpu/atomic.h>
#include <nvgpu/unit.h>
#include "os_linux.h"

irqreturn_t nvgpu_intr_stall(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	u32 mc_intr_0;

	trace_mc_gk20a_intr_stall(g->name);

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	mc_intr_0 = g->ops.mc.intr_stall(g);
	if (unlikely(!mc_intr_0))
		return IRQ_NONE;

	g->ops.mc.intr_stall_pause(g);

	atomic_inc(&l->hw_irq_stall_count);

	trace_mc_gk20a_intr_stall_done(g->name);

	return IRQ_WAKE_THREAD;
}

irqreturn_t nvgpu_intr_thread_stall(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int hw_irq_count;

	nvgpu_log(g, gpu_dbg_intr, "interrupt thread launched");

	trace_mc_gk20a_intr_thread_stall(g->name);

	hw_irq_count = atomic_read(&l->hw_irq_stall_count);
	g->ops.mc.isr_stall(g);
	g->ops.mc.intr_stall_resume(g);
	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&l->sw_irq_stall_last_handled, hw_irq_count);

	nvgpu_cond_broadcast(&l->sw_irq_stall_last_handled_wq);

	trace_mc_gk20a_intr_thread_stall_done(g->name);

	return IRQ_HANDLED;
}

irqreturn_t nvgpu_intr_nonstall(struct gk20a *g)
{
	u32 non_stall_intr_val;
	u32 hw_irq_count;
	int ops_old, ops_new, ops = 0;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	if (!g->power_on)
		return IRQ_NONE;

	/* not from gpu when sharing irq with others */
	non_stall_intr_val = g->ops.mc.intr_nonstall(g);
	if (unlikely(!non_stall_intr_val))
		return IRQ_NONE;

	g->ops.mc.intr_nonstall_pause(g);

	ops = g->ops.mc.isr_nonstall(g);
	if (ops) {
		do {
			ops_old = atomic_read(&l->nonstall_ops);
			ops_new  = ops_old | ops;
		} while (ops_old != atomic_cmpxchg(&l->nonstall_ops,
						ops_old, ops_new));

		queue_work(l->nonstall_work_queue, &l->nonstall_fn_work);
	}

	hw_irq_count = atomic_inc_return(&l->hw_irq_nonstall_count);

	/* sync handled irq counter before re-enabling interrupts */
	atomic_set(&l->sw_irq_nonstall_last_handled, hw_irq_count);

	g->ops.mc.intr_nonstall_resume(g);

	nvgpu_cond_broadcast(&l->sw_irq_nonstall_last_handled_wq);

	return IRQ_HANDLED;
}

static void mc_gk20a_handle_intr_nonstall(struct gk20a *g, u32 ops)
{
	bool semaphore_wakeup, post_events;

	semaphore_wakeup =
		(((ops & GK20A_NONSTALL_OPS_WAKEUP_SEMAPHORE) != 0U) ?
					true : false);
	post_events = (((ops & GK20A_NONSTALL_OPS_POST_EVENTS) != 0U) ?
					true: false);

	if (semaphore_wakeup) {
		g->ops.semaphore_wakeup(g, post_events);
	}
}

void nvgpu_intr_nonstall_cb(struct work_struct *work)
{
	struct nvgpu_os_linux *l =
		container_of(work, struct nvgpu_os_linux, nonstall_fn_work);
	struct gk20a *g = &l->g;

	do {
		u32 ops;

		ops = atomic_xchg(&l->nonstall_ops, 0);
		mc_gk20a_handle_intr_nonstall(g, ops);
	} while (atomic_read(&l->nonstall_ops) != 0);
}
