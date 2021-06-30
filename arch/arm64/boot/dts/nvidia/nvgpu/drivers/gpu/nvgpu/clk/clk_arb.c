/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <nvgpu/bitops.h>
#include <nvgpu/lock.h>
#include <nvgpu/kmem.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/kref.h>
#include <nvgpu/log.h>
#include <nvgpu/barrier.h>
#include <nvgpu/cond.h>
#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>

#include "clk/clk.h"
#include "pstate/pstate.h"
#include "lpwr/lpwr.h"
#include "volt/volt.h"

int nvgpu_clk_notification_queue_alloc(struct gk20a *g,
				struct nvgpu_clk_notification_queue *queue,
				size_t events_number) {
	queue->notifications = nvgpu_kcalloc(g, events_number,
		sizeof(struct nvgpu_clk_notification));
	if (!queue->notifications)
		return -ENOMEM;
	queue->size = events_number;

	nvgpu_atomic_set(&queue->head, 0);
	nvgpu_atomic_set(&queue->tail, 0);

	return 0;
}

void nvgpu_clk_notification_queue_free(struct gk20a *g,
		struct nvgpu_clk_notification_queue *queue) {
	if (queue->size > 0) {
		nvgpu_kfree(g, queue->notifications);
		queue->size = 0;
		nvgpu_atomic_set(&queue->head, 0);
		nvgpu_atomic_set(&queue->tail, 0);
	}
}

static void nvgpu_clk_arb_queue_notification(struct gk20a *g,
				struct nvgpu_clk_notification_queue *queue,
				u32 alarm_mask) {

	u32 queue_index;
	u64 timestamp;

	queue_index = (nvgpu_atomic_inc_return(&queue->tail)) % queue->size;
	/* get current timestamp */
	timestamp = (u64) nvgpu_hr_timestamp();

	queue->notifications[queue_index].timestamp = timestamp;
	queue->notifications[queue_index].notification = alarm_mask;

}

void nvgpu_clk_arb_set_global_alarm(struct gk20a *g, u32 alarm)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	u64 current_mask;
	u32 refcnt;
	u32 alarm_mask;
	u64 new_mask;

	do {
		current_mask = nvgpu_atomic64_read(&arb->alarm_mask);
		/* atomic operations are strong so they do not need masks */

		refcnt = ((u32) (current_mask >> 32)) + 1;
		alarm_mask =  (u32) (current_mask &  ~0) | alarm;
		new_mask = ((u64) refcnt << 32) | alarm_mask;

	} while (unlikely(current_mask !=
			(u64)nvgpu_atomic64_cmpxchg(&arb->alarm_mask,
						current_mask, new_mask)));

	nvgpu_clk_arb_queue_notification(g, &arb->notification_queue, alarm);
}


int nvgpu_clk_arb_update_vf_table(struct nvgpu_clk_arb *arb)
{
	struct gk20a *g = arb->g;
	struct nvgpu_clk_vf_table *table;

	u32 i, j;
	int status = -EINVAL;
	u32 gpc2clk_voltuv = 0, mclk_voltuv = 0;
	u32 gpc2clk_voltuv_sram = 0, mclk_voltuv_sram = 0;
	u16 clk_cur;
	u32 num_points;

	struct clk_set_info *p5_info, *p0_info;

	table = NV_ACCESS_ONCE(arb->current_vf_table);
	/* make flag visible when all data has resolved in the tables */
	nvgpu_smp_rmb();

	table = (table == &arb->vf_table_pool[0]) ? &arb->vf_table_pool[1] :
		&arb->vf_table_pool[0];

	/* Get allowed memory ranges */
	if (g->ops.clk_arb.get_arbiter_clk_range(g, CTRL_CLK_DOMAIN_GPC2CLK,
						&arb->gpc2clk_min,
						&arb->gpc2clk_max) < 0) {
		nvgpu_err(g, "failed to fetch GPC2CLK range");
		goto exit_vf_table;
	}
	if (g->ops.clk_arb.get_arbiter_clk_range(g, CTRL_CLK_DOMAIN_MCLK,
						&arb->mclk_min,
						&arb->mclk_max) < 0) {
		nvgpu_err(g, "failed to fetch MCLK range");
		goto exit_vf_table;
	}

	table->gpc2clk_num_points = MAX_F_POINTS;
	table->mclk_num_points = MAX_F_POINTS;

	if (g->ops.clk.clk_domain_get_f_points(arb->g, CTRL_CLK_DOMAIN_GPC2CLK,
		&table->gpc2clk_num_points, arb->gpc2clk_f_points)) {
		nvgpu_err(g, "failed to fetch GPC2CLK frequency points");
		goto exit_vf_table;
	}

	if (g->ops.clk.clk_domain_get_f_points(arb->g, CTRL_CLK_DOMAIN_MCLK,
		&table->mclk_num_points, arb->mclk_f_points)) {
		nvgpu_err(g, "failed to fetch MCLK frequency points");
		goto exit_vf_table;
	}
	if (!table->mclk_num_points || !table->gpc2clk_num_points) {
		nvgpu_err(g, "empty queries to f points mclk %d gpc2clk %d",
			table->mclk_num_points, table->gpc2clk_num_points);
		status = -EINVAL;
		goto exit_vf_table;
	}

	memset(table->mclk_points, 0,
		table->mclk_num_points*sizeof(struct nvgpu_clk_vf_point));
	memset(table->gpc2clk_points, 0,
		table->gpc2clk_num_points*sizeof(struct nvgpu_clk_vf_point));

	p5_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P5, clkwhich_mclk);
	if (!p5_info) {
		nvgpu_err(g, "failed to get MCLK P5 info");
		goto exit_vf_table;
	}
	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich_mclk);
	if (!p0_info) {
		nvgpu_err(g, "failed to get MCLK P0 info");
		goto exit_vf_table;
	}

	for (i = 0, j = 0, num_points = 0, clk_cur = 0;
			i < table->mclk_num_points; i++) {

		if ((arb->mclk_f_points[i] >= arb->mclk_min) &&
			(arb->mclk_f_points[i] <= arb->mclk_max) &&
			(arb->mclk_f_points[i] != clk_cur)) {

			table->mclk_points[j].mem_mhz = arb->mclk_f_points[i];
			mclk_voltuv = mclk_voltuv_sram = 0;

			status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_MCLK,
				&table->mclk_points[j].mem_mhz, &mclk_voltuv,
				CTRL_VOLT_DOMAIN_LOGIC);
			if (status < 0) {
				nvgpu_err(g,
					"failed to get MCLK LOGIC voltage");
				goto exit_vf_table;
			}
			status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_MCLK,
				&table->mclk_points[j].mem_mhz,
				&mclk_voltuv_sram,
				CTRL_VOLT_DOMAIN_SRAM);
			if (status < 0) {
				nvgpu_err(g, "failed to get MCLK SRAM voltage");
				goto exit_vf_table;
			}

			table->mclk_points[j].uvolt = mclk_voltuv;
			table->mclk_points[j].uvolt_sram = mclk_voltuv_sram;
			clk_cur = table->mclk_points[j].mem_mhz;

			if ((clk_cur >= p5_info->min_mhz) &&
					(clk_cur <= p5_info->max_mhz))
				VF_POINT_SET_PSTATE_SUPPORTED(
					&table->mclk_points[j],
					CTRL_PERF_PSTATE_P5);
			if ((clk_cur >= p0_info->min_mhz) &&
					(clk_cur <= p0_info->max_mhz))
				VF_POINT_SET_PSTATE_SUPPORTED(
					&table->mclk_points[j],
					CTRL_PERF_PSTATE_P0);

			j++;
			num_points++;

		}
	}
	table->mclk_num_points = num_points;

	p5_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P5, clkwhich_gpc2clk);
	if (!p5_info) {
		status = -EINVAL;
		nvgpu_err(g, "failed to get GPC2CLK P5 info");
		goto exit_vf_table;
	}

	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich_gpc2clk);
	if (!p0_info) {
		status = -EINVAL;
		nvgpu_err(g, "failed to get GPC2CLK P0 info");
		goto exit_vf_table;
	}

	/* GPC2CLK needs to be checked in two passes. The first determines the
	 * relationships between GPC2CLK, SYS2CLK and XBAR2CLK, while the
	 * second verifies that the clocks minimum is satisfied and sets
	 * the voltages
	 */
	for (i = 0, j = 0, num_points = 0, clk_cur = 0;
			i < table->gpc2clk_num_points; i++) {
		struct set_fll_clk setfllclk;

		if ((arb->gpc2clk_f_points[i] >= arb->gpc2clk_min) &&
			(arb->gpc2clk_f_points[i] <= arb->gpc2clk_max) &&
			(arb->gpc2clk_f_points[i] != clk_cur)) {

			table->gpc2clk_points[j].gpc_mhz =
				arb->gpc2clk_f_points[i];
			setfllclk.gpc2clkmhz = arb->gpc2clk_f_points[i];
			status = clk_get_fll_clks(g, &setfllclk);
			if (status < 0) {
				nvgpu_err(g,
					"failed to get GPC2CLK slave clocks");
				goto exit_vf_table;
			}

			table->gpc2clk_points[j].sys_mhz =
				setfllclk.sys2clkmhz;
			table->gpc2clk_points[j].xbar_mhz =
				setfllclk.xbar2clkmhz;

			clk_cur = table->gpc2clk_points[j].gpc_mhz;

			if ((clk_cur >= p5_info->min_mhz) &&
					(clk_cur <= p5_info->max_mhz))
				VF_POINT_SET_PSTATE_SUPPORTED(
					&table->gpc2clk_points[j],
					CTRL_PERF_PSTATE_P5);
			if ((clk_cur >= p0_info->min_mhz) &&
					(clk_cur <= p0_info->max_mhz))
				VF_POINT_SET_PSTATE_SUPPORTED(
					&table->gpc2clk_points[j],
					CTRL_PERF_PSTATE_P0);

			j++;
			num_points++;
		}
	}
	table->gpc2clk_num_points = num_points;

	/* Second pass */
	for (i = 0, j = 0; i < table->gpc2clk_num_points; i++) {

		u16 alt_gpc2clk = table->gpc2clk_points[i].gpc_mhz;

		gpc2clk_voltuv = gpc2clk_voltuv_sram = 0;

		/* Check sysclk */
		p5_info = pstate_get_clk_set_info(g,
			VF_POINT_GET_PSTATE(&table->gpc2clk_points[i]),
			clkwhich_sys2clk);
		if (!p5_info) {
			status = -EINVAL;
			nvgpu_err(g, "failed to get SYS2CLK P5 info");
			goto exit_vf_table;
		}

		/* sys2clk below clk min, need to find correct clock */
		if (table->gpc2clk_points[i].sys_mhz < p5_info->min_mhz) {
			for (j = i + 1; j < table->gpc2clk_num_points; j++) {

				if (table->gpc2clk_points[j].sys_mhz >=
							p5_info->min_mhz) {


					table->gpc2clk_points[i].sys_mhz =
						p5_info->min_mhz;

					alt_gpc2clk = alt_gpc2clk <
						table->gpc2clk_points[j].
							gpc_mhz ?
						table->gpc2clk_points[j].
							gpc_mhz :
						alt_gpc2clk;
					break;
				}
			}
			/* no VF exists that satisfies condition */
			if (j == table->gpc2clk_num_points) {
				nvgpu_err(g, "NO SYS2CLK VF point possible");
				status = -EINVAL;
				goto exit_vf_table;
			}
		}

		/* Check xbarclk */
		p5_info = pstate_get_clk_set_info(g,
			VF_POINT_GET_PSTATE(&table->gpc2clk_points[i]),
			clkwhich_xbar2clk);
		if (!p5_info) {
			status = -EINVAL;
			nvgpu_err(g, "failed to get SYS2CLK P5 info");
			goto exit_vf_table;
		}

		/* xbar2clk below clk min, need to find correct clock */
		if (table->gpc2clk_points[i].xbar_mhz < p5_info->min_mhz) {
			for (j = i; j < table->gpc2clk_num_points; j++) {
				if (table->gpc2clk_points[j].xbar_mhz >=
							p5_info->min_mhz) {

					table->gpc2clk_points[i].xbar_mhz =
						p5_info->min_mhz;

					alt_gpc2clk = alt_gpc2clk <
						table->gpc2clk_points[j].
							gpc_mhz ?
						table->gpc2clk_points[j].
							gpc_mhz :
						alt_gpc2clk;
					break;
				}
			}
			/* no VF exists that satisfies condition */
			if (j == table->gpc2clk_num_points) {
				status = -EINVAL;
				nvgpu_err(g, "NO XBAR2CLK VF point possible");

				goto exit_vf_table;
			}
		}

		/* Calculate voltages */
		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_GPC2CLK,
						&alt_gpc2clk, &gpc2clk_voltuv,
						CTRL_VOLT_DOMAIN_LOGIC);
		if (status < 0) {
			nvgpu_err(g, "failed to get GPC2CLK LOGIC voltage");
			goto exit_vf_table;
		}

		status = clk_domain_get_f_or_v(g, CTRL_CLK_DOMAIN_GPC2CLK,
						&alt_gpc2clk,
						&gpc2clk_voltuv_sram,
						CTRL_VOLT_DOMAIN_SRAM);
		if (status < 0) {
			nvgpu_err(g, "failed to get GPC2CLK SRAM voltage");
			goto exit_vf_table;
		}

		table->gpc2clk_points[i].uvolt = gpc2clk_voltuv;
		table->gpc2clk_points[i].uvolt_sram = gpc2clk_voltuv_sram;
	}

	/* make table visible when all data has resolved in the tables */
	nvgpu_smp_wmb();
	arb->current_vf_table = table;

exit_vf_table:

	if (status < 0)
		nvgpu_clk_arb_set_global_alarm(g,
			EVENT(ALARM_VF_TABLE_UPDATE_FAILED));
	nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);

	return status;
}


static void nvgpu_clk_arb_run_vf_table_cb(struct nvgpu_clk_arb *arb)
{
	struct gk20a *g = arb->g;
	u32 err;

	/* get latest vf curve from pmu */
	err = clk_vf_point_cache(g);
	if (err) {
		nvgpu_err(g, "failed to cache VF table");
		nvgpu_clk_arb_set_global_alarm(g,
			EVENT(ALARM_VF_TABLE_UPDATE_FAILED));
		nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);

		return;
	}
	nvgpu_clk_arb_update_vf_table(arb);
}

u32 nvgpu_clk_arb_notify(struct nvgpu_clk_dev *dev,
				struct nvgpu_clk_arb_target *target,
				u32 alarm) {

	struct nvgpu_clk_session *session = dev->session;
	struct nvgpu_clk_arb *arb = session->g->clk_arb;
	struct nvgpu_clk_notification *notification;

	u32 queue_alarm_mask = 0;
	u32 enabled_mask = 0;
	u32 new_alarms_reported = 0;
	u32 poll_mask = 0;
	u32 tail, head;
	u32 queue_index;
	size_t size;
	int index;

	enabled_mask = nvgpu_atomic_read(&dev->enabled_mask);
	size = arb->notification_queue.size;

	/* queue global arbiter notifications in buffer */
	do {
		tail = nvgpu_atomic_read(&arb->notification_queue.tail);
		/* copy items to the queue */
		queue_index = nvgpu_atomic_read(&dev->queue.tail);
		head = dev->arb_queue_head;
		head = (tail - head) < arb->notification_queue.size ?
			head : tail - arb->notification_queue.size;

		for (index = head; _WRAPGTEQ(tail, index); index++) {
			u32 alarm_detected;

			notification = &arb->notification_queue.
						notifications[(index+1) % size];
			alarm_detected =
				NV_ACCESS_ONCE(notification->notification);

			if (!(enabled_mask & alarm_detected))
				continue;

			queue_index++;
			dev->queue.notifications[
				queue_index % dev->queue.size].timestamp =
					NV_ACCESS_ONCE(notification->timestamp);

			dev->queue.notifications[
				queue_index % dev->queue.size].notification =
					alarm_detected;

			queue_alarm_mask |= alarm_detected;
		}
	} while (unlikely(nvgpu_atomic_read(&arb->notification_queue.tail) !=
			(int)tail));

	nvgpu_atomic_set(&dev->queue.tail, queue_index);
	/* update the last notification we processed from global queue */

	dev->arb_queue_head = tail;

	/* Check if current session targets are met */
	if (enabled_mask & EVENT(ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE)) {
		if ((target->gpc2clk < session->target->gpc2clk)
			|| (target->mclk < session->target->mclk)) {

			poll_mask |= (NVGPU_POLLIN | NVGPU_POLLPRI);
			nvgpu_clk_arb_queue_notification(arb->g, &dev->queue,
				EVENT(ALARM_LOCAL_TARGET_VF_NOT_POSSIBLE));
		}
	}

	/* Check if there is a new VF update */
	if (queue_alarm_mask & EVENT(VF_UPDATE))
		poll_mask |= (NVGPU_POLLIN | NVGPU_POLLRDNORM);

	/* Notify sticky alarms that were not reported on previous run*/
	new_alarms_reported = (queue_alarm_mask |
			(alarm & ~dev->alarms_reported & queue_alarm_mask));

	if (new_alarms_reported & ~LOCAL_ALARM_MASK) {
		/* check that we are not re-reporting */
		if (new_alarms_reported & EVENT(ALARM_GPU_LOST))
			poll_mask |= NVGPU_POLLHUP;

		poll_mask |= (NVGPU_POLLIN | NVGPU_POLLPRI);
		/* On next run do not report global alarms that were already
		 * reported, but report SHUTDOWN always
		 */
		dev->alarms_reported = new_alarms_reported & ~LOCAL_ALARM_MASK &
							~EVENT(ALARM_GPU_LOST);
	}

	if (poll_mask) {
		nvgpu_atomic_set(&dev->poll_mask, poll_mask);
		nvgpu_clk_arb_event_post_event(dev);
	}

	return new_alarms_reported;
}

void nvgpu_clk_arb_clear_global_alarm(struct gk20a *g, u32 alarm)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	u64 current_mask;
	u32 refcnt;
	u32 alarm_mask;
	u64 new_mask;

	do {
		current_mask = nvgpu_atomic64_read(&arb->alarm_mask);
		/* atomic operations are strong so they do not need masks */

		refcnt = ((u32) (current_mask >> 32)) + 1;
		alarm_mask =  (u32) (current_mask & ~alarm);
		new_mask = ((u64) refcnt << 32) | alarm_mask;

	} while (unlikely(current_mask !=
			(u64)nvgpu_atomic64_cmpxchg(&arb->alarm_mask,
					current_mask, new_mask)));
}

/*
 * Process one scheduled work item.
 */
static void nvgpu_clk_arb_worker_process_item(
		struct nvgpu_clk_arb_work_item *work_item)
{
	struct gk20a *g = work_item->arb->g;

	clk_arb_dbg(g, " ");

	if (work_item->item_type == CLK_ARB_WORK_UPDATE_VF_TABLE)
		nvgpu_clk_arb_run_vf_table_cb(work_item->arb);
	else if (work_item->item_type == CLK_ARB_WORK_UPDATE_ARB)
		g->ops.clk_arb.clk_arb_run_arbiter_cb(work_item->arb);
}

/**
 * Tell the worker that one more work needs to be done.
 *
 * Increase the work counter to synchronize the worker with the new work. Wake
 * up the worker. If the worker was already running, it will handle this work
 * before going to sleep.
 */
static int nvgpu_clk_arb_worker_wakeup(struct gk20a *g)
{
	int put;

	clk_arb_dbg(g, " ");

	put = nvgpu_atomic_inc_return(&g->clk_arb_worker.put);
	nvgpu_cond_signal_interruptible(&g->clk_arb_worker.wq);

	return put;
}

/**
 * Test if there is some work pending.
 *
 * This is a pair for nvgpu_clk_arb_worker_wakeup to be called from the
 * worker. The worker has an internal work counter which is incremented once
 * per finished work item. This is compared with the number of queued jobs.
 */
static bool nvgpu_clk_arb_worker_pending(struct gk20a *g, int get)
{
	bool pending = nvgpu_atomic_read(&g->clk_arb_worker.put) != get;

	/* We don't need barriers because they are implicit in locking */
	return pending;
}

/**
 * Process the queued works for the worker thread serially.
 *
 * Flush all the work items in the queue one by one. This may block timeout
 * handling for a short while, as these are serialized.
 */
static void nvgpu_clk_arb_worker_process(struct gk20a *g, int *get)
{

	while (nvgpu_clk_arb_worker_pending(g, *get)) {
		struct nvgpu_clk_arb_work_item *work_item = NULL;

		nvgpu_spinlock_acquire(&g->clk_arb_worker.items_lock);
		if (!nvgpu_list_empty(&g->clk_arb_worker.items)) {
			work_item = nvgpu_list_first_entry(&g->clk_arb_worker.items,
				nvgpu_clk_arb_work_item, worker_item);
			nvgpu_list_del(&work_item->worker_item);
		}
		nvgpu_spinlock_release(&g->clk_arb_worker.items_lock);

		if (!work_item) {
			/*
			 * Woke up for some other reason, but there are no
			 * other reasons than a work item added in the items list
			 * currently, so warn and ack the message.
			 */
			nvgpu_warn(g, "Spurious worker event!");
			++*get;
			break;
		}

		nvgpu_clk_arb_worker_process_item(work_item);
		++*get;
	}
}

/*
 * Process all work items found in the clk arbiter work queue.
 */
static int nvgpu_clk_arb_poll_worker(void *arg)
{
	struct gk20a *g = (struct gk20a *)arg;
	struct gk20a_worker *worker = &g->clk_arb_worker;
	int get = 0;

	clk_arb_dbg(g, " ");

	while (!nvgpu_thread_should_stop(&worker->poll_task)) {
		int ret;

		ret = NVGPU_COND_WAIT_INTERRUPTIBLE(
				&worker->wq,
				nvgpu_clk_arb_worker_pending(g, get), 0);

		if (nvgpu_thread_should_stop(&worker->poll_task)) {
			break;
		}

		if (ret == 0)
			nvgpu_clk_arb_worker_process(g, &get);
	}
	return 0;
}

static int __nvgpu_clk_arb_worker_start(struct gk20a *g)
{
	char thread_name[64];
	int err = 0;

	if (nvgpu_thread_is_running(&g->clk_arb_worker.poll_task))
		return err;

	nvgpu_mutex_acquire(&g->clk_arb_worker.start_lock);

	/*
	 * Mutexes have implicit barriers, so there is no risk of a thread
	 * having a stale copy of the poll_task variable as the call to
	 * thread_is_running is volatile
	 */

	if (nvgpu_thread_is_running(&g->clk_arb_worker.poll_task)) {
		nvgpu_mutex_release(&g->clk_arb_worker.start_lock);
		return err;
	}

	snprintf(thread_name, sizeof(thread_name),
			"nvgpu_clk_arb_poll_%s", g->name);

	err = nvgpu_thread_create(&g->clk_arb_worker.poll_task, g,
			nvgpu_clk_arb_poll_worker, thread_name);

	nvgpu_mutex_release(&g->clk_arb_worker.start_lock);
	return err;
}

/**
 * Append a work item to the worker's list.
 *
 * This adds work item to the end of the list and wakes the worker
 * up immediately. If the work item already existed in the list, it's not added,
 * because in that case it has been scheduled already but has not yet been
 * processed.
 */
void nvgpu_clk_arb_worker_enqueue(struct gk20a *g,
		struct nvgpu_clk_arb_work_item *work_item)
{
	clk_arb_dbg(g, " ");

	/*
	 * Warn if worker thread cannot run
	 */
	if (WARN_ON(__nvgpu_clk_arb_worker_start(g))) {
		nvgpu_warn(g, "clk arb worker cannot run!");
		return;
	}

	nvgpu_spinlock_acquire(&g->clk_arb_worker.items_lock);
	if (!nvgpu_list_empty(&work_item->worker_item)) {
		/*
		 * Already queued, so will get processed eventually.
		 * The worker is probably awake already.
		 */
		nvgpu_spinlock_release(&g->clk_arb_worker.items_lock);
		return;
	}
	nvgpu_list_add_tail(&work_item->worker_item, &g->clk_arb_worker.items);
	nvgpu_spinlock_release(&g->clk_arb_worker.items_lock);

	nvgpu_clk_arb_worker_wakeup(g);
}

/**
 * Initialize the clk arb worker's metadata and start the background thread.
 */
int nvgpu_clk_arb_worker_init(struct gk20a *g)
{
	int err;

	nvgpu_atomic_set(&g->clk_arb_worker.put, 0);
	nvgpu_cond_init(&g->clk_arb_worker.wq);
	nvgpu_init_list_node(&g->clk_arb_worker.items);
	nvgpu_spinlock_init(&g->clk_arb_worker.items_lock);
	err = nvgpu_mutex_init(&g->clk_arb_worker.start_lock);
	if (err)
		goto error_check;

	err = __nvgpu_clk_arb_worker_start(g);
error_check:
	if (err) {
		nvgpu_err(g, "failed to start clk arb poller thread");
		return err;
	}
	return 0;
}

int nvgpu_clk_arb_init_arbiter(struct gk20a *g)
{
	int err = 0;

	if (!g->ops.clk.support_clk_freq_controller ||
		!g->ops.clk_arb.get_arbiter_clk_domains) {
		return 0;
	}

	nvgpu_mutex_acquire(&g->clk_arb_enable_lock);

	err = g->ops.clk_arb.arbiter_clk_init(g);

	nvgpu_mutex_release(&g->clk_arb_enable_lock);

	return err;
}

bool nvgpu_clk_arb_has_active_req(struct gk20a *g)
{
	return (nvgpu_atomic_read(&g->clk_arb_global_nr) > 0);
}

void nvgpu_clk_arb_send_thermal_alarm(struct gk20a *g)
{
	nvgpu_clk_arb_schedule_alarm(g,
		(0x1UL << NVGPU_EVENT_ALARM_THERMAL_ABOVE_THRESHOLD));
}

void nvgpu_clk_arb_schedule_alarm(struct gk20a *g, u32 alarm)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	nvgpu_clk_arb_set_global_alarm(g, alarm);
	nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);
}

static void nvgpu_clk_arb_worker_deinit(struct gk20a *g)
{
	nvgpu_atomic_inc(&g->clk_arb_worker.put);

	nvgpu_mutex_acquire(&g->clk_arb_worker.start_lock);
	nvgpu_thread_stop(&g->clk_arb_worker.poll_task);
	nvgpu_mutex_release(&g->clk_arb_worker.start_lock);
}

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	nvgpu_mutex_acquire(&g->clk_arb_enable_lock);

	if (arb) {
		nvgpu_clk_arb_worker_deinit(g);
		g->ops.clk_arb.clk_arb_cleanup(g->clk_arb);
	}

	nvgpu_mutex_release(&g->clk_arb_enable_lock);
}

int nvgpu_clk_arb_init_session(struct gk20a *g,
		struct nvgpu_clk_session **_session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	struct nvgpu_clk_session *session = *(_session);

	clk_arb_dbg(g, " ");

	if (!g->ops.clk.support_clk_freq_controller ||
		!g->ops.clk_arb.get_arbiter_clk_domains) {
		return 0;
	}

	session = nvgpu_kzalloc(g, sizeof(struct nvgpu_clk_session));
	if (!session)
		return -ENOMEM;
	session->g = g;

	nvgpu_ref_init(&session->refcount);

	session->zombie = false;
	session->target_pool[0].pstate = CTRL_PERF_PSTATE_P8;
	/* make sure that the initialization of the pool is visible
	 * before the update
	 */
	nvgpu_smp_wmb();
	session->target = &session->target_pool[0];

	nvgpu_init_list_node(&session->targets);
	nvgpu_spinlock_init(&session->session_lock);

	nvgpu_spinlock_acquire(&arb->sessions_lock);
	nvgpu_list_add_tail(&session->link, &arb->sessions);
	nvgpu_spinlock_release(&arb->sessions_lock);

	*_session = session;

	return 0;
}

void nvgpu_clk_arb_free_fd(struct nvgpu_ref *refcount)
{
	struct nvgpu_clk_dev *dev = container_of(refcount,
			struct nvgpu_clk_dev, refcount);
	struct nvgpu_clk_session *session = dev->session;
	struct gk20a *g = session->g;

	nvgpu_clk_notification_queue_free(g, &dev->queue);

	nvgpu_atomic_dec(&g->clk_arb_global_nr);
	nvgpu_kfree(g, dev);
}

void nvgpu_clk_arb_free_session(struct nvgpu_ref *refcount)
{
	struct nvgpu_clk_session *session = container_of(refcount,
			struct nvgpu_clk_session, refcount);
	struct nvgpu_clk_arb *arb = session->g->clk_arb;
	struct gk20a *g = session->g;
	struct nvgpu_clk_dev *dev, *tmp;

	clk_arb_dbg(g, " ");

	if (arb) {
		nvgpu_spinlock_acquire(&arb->sessions_lock);
		nvgpu_list_del(&session->link);
		nvgpu_spinlock_release(&arb->sessions_lock);
	}

	nvgpu_spinlock_acquire(&session->session_lock);
	nvgpu_list_for_each_entry_safe(dev, tmp, &session->targets,
			nvgpu_clk_dev, node) {
		nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);
		nvgpu_list_del(&dev->node);
	}
	nvgpu_spinlock_release(&session->session_lock);

	nvgpu_kfree(g, session);
}

void nvgpu_clk_arb_release_session(struct gk20a *g,
	struct nvgpu_clk_session *session)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	clk_arb_dbg(g, " ");

	session->zombie = true;
	nvgpu_ref_put(&session->refcount, nvgpu_clk_arb_free_session);
	if (arb)
		nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);
}

void nvgpu_clk_arb_schedule_vf_table_update(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	nvgpu_clk_arb_worker_enqueue(g, &arb->update_vf_table_work_item);
}

/* This function is inherently unsafe to call while arbiter is running
 * arbiter must be blocked before calling this function
 */
int nvgpu_clk_arb_get_current_pstate(struct gk20a *g)
{
	return NV_ACCESS_ONCE(g->clk_arb->actual->pstate);
}

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;

	if (lock)
		nvgpu_mutex_acquire(&arb->pstate_lock);
	else
		nvgpu_mutex_release(&arb->pstate_lock);
}

bool nvgpu_clk_arb_is_valid_domain(struct gk20a *g, u32 api_domain)
{
	u32 clk_domains = g->ops.clk_arb.get_arbiter_clk_domains(g);

	switch (api_domain) {
	case NVGPU_CLK_DOMAIN_MCLK:
		return (clk_domains & CTRL_CLK_DOMAIN_MCLK) != 0;

	case NVGPU_CLK_DOMAIN_GPCCLK:
		return (clk_domains & CTRL_CLK_DOMAIN_GPC2CLK) != 0;

	default:
		return false;
	}
}

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	int ret;

	switch (api_domain) {
	case NVGPU_CLK_DOMAIN_MCLK:
		ret = g->ops.clk_arb.get_arbiter_clk_range(g,
				CTRL_CLK_DOMAIN_MCLK, min_mhz, max_mhz);
		return ret;

	case NVGPU_CLK_DOMAIN_GPCCLK:
		ret = g->ops.clk_arb.get_arbiter_clk_range(g,
				CTRL_CLK_DOMAIN_GPC2CLK, min_mhz, max_mhz);
		if (!ret) {
			*min_mhz /= 2;
			*max_mhz /= 2;
		}
		return ret;

	default:
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
	u32 api_domain, u32 *max_points, u16 *fpoints)
{
	int err;
	u32 i;

	switch (api_domain) {
	case NVGPU_CLK_DOMAIN_GPCCLK:
		err = g->ops.clk_arb.get_arbiter_f_points(g,
			CTRL_CLK_DOMAIN_GPC2CLK, max_points, fpoints);
		if (err || !fpoints)
			return err;
		for (i = 0; i < *max_points; i++)
			fpoints[i] /= 2;
		return 0;
	case NVGPU_CLK_DOMAIN_MCLK:
		return g->ops.clk_arb.get_arbiter_f_points(g,
			CTRL_CLK_DOMAIN_MCLK, max_points, fpoints);
	default:
		return -EINVAL;
	}
}

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
		u32 api_domain, u16 *freq_mhz)
{
	int err = 0;
	struct nvgpu_clk_arb_target *target = session->target;

	if (!nvgpu_clk_arb_is_valid_domain(session->g, api_domain)) {
		return -EINVAL;
	}

	switch (api_domain) {
		case NVGPU_CLK_DOMAIN_MCLK:
			*freq_mhz = target->mclk;
			break;

		case NVGPU_CLK_DOMAIN_GPCCLK:
			*freq_mhz = target->gpc2clk / 2ULL;
			break;

		default:
			*freq_mhz = 0;
			err = -EINVAL;
	}
	return err;
}

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
		u32 api_domain, u16 *freq_mhz)
{
	struct nvgpu_clk_arb *arb = g->clk_arb;
	int err = 0;
	struct nvgpu_clk_arb_target *actual = arb->actual;

	if (!nvgpu_clk_arb_is_valid_domain(g, api_domain)) {
		return -EINVAL;
	}

	switch (api_domain) {
		case NVGPU_CLK_DOMAIN_MCLK:
			*freq_mhz = actual->mclk;
			break;

		case NVGPU_CLK_DOMAIN_GPCCLK:
			*freq_mhz = actual->gpc2clk / 2ULL;
			break;

		default:
			*freq_mhz = 0;
			err = -EINVAL;
	}
	return err;
}

unsigned long nvgpu_clk_measure_freq(struct gk20a *g, u32 api_domain)
{
	unsigned long freq = 0UL;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPC2CLK:
		freq = g->ops.clk.get_rate(g, CTRL_CLK_DOMAIN_GPCCLK) * 2UL;
		break;
	default:
		break;
	}
	return freq;
}

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
		u32 api_domain, u16 *freq_mhz)
{
	if (!nvgpu_clk_arb_is_valid_domain(g, api_domain)) {
		return -EINVAL;
	}

	switch (api_domain) {
	case NVGPU_CLK_DOMAIN_MCLK:
		*freq_mhz = g->ops.clk.measure_freq(g, CTRL_CLK_DOMAIN_MCLK) /
			1000000ULL;
		return 0;

	case NVGPU_CLK_DOMAIN_GPCCLK:
		*freq_mhz = g->ops.clk.measure_freq(g,
			CTRL_CLK_DOMAIN_GPC2CLK) / 2000000ULL;
		return 0;

	default:
		return -EINVAL;
	}
}
