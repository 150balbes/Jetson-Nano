/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>
#include <nvgpu/clk_arb.h>

#include "clk_arb_gp10b.h"

u32 gp10b_get_arbiter_clk_domains(struct gk20a *g)
{
	(void)g;
	clk_arb_dbg(g, " ");
	return CTRL_CLK_DOMAIN_GPC2CLK;
}

int gp10b_get_arbiter_f_points(struct gk20a *g,u32 api_domain,
				u32 *num_points, u16 *freqs_in_mhz)
{
	int ret = 0;
	u32 i;
	bool is_freq_list_available = false;

	if (*num_points != 0U) {
		is_freq_list_available = true;
	}

	clk_arb_dbg(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPC2CLK:
		ret = g->ops.clk.clk_domain_get_f_points(g, CTRL_CLK_DOMAIN_GPCCLK,
			num_points, freqs_in_mhz);

		/* multiply by 2 for GPC2CLK */
		if (ret == 0 && is_freq_list_available) {
			for (i = 0U; i < *num_points; i++) {
				freqs_in_mhz[i] *= 2U;
			}
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int gp10b_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	int ret = 0;

	clk_arb_dbg(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPC2CLK:
		ret = g->ops.clk.get_clk_range(g, CTRL_CLK_DOMAIN_GPCCLK,
			min_mhz, max_mhz);

		if (ret == 0) {
			*min_mhz *= 2U;
			*max_mhz *= 2U;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int gp10b_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u16 *default_mhz)
{
	int ret = 0;
	u16 min_mhz, max_mhz;

	clk_arb_dbg(g, " ");

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_GPC2CLK:
		ret = gp10b_get_arbiter_clk_range(g, api_domain,
			&min_mhz, &max_mhz);

		if (ret == 0) {
			*default_mhz = max_mhz;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

int gp10b_init_clk_arbiter(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb = NULL;
	u16 default_mhz;
	int err;
	int index;
	struct nvgpu_clk_vf_table *table;

	clk_arb_dbg(g, " ");

	if(g->clk_arb != NULL) {
		return 0;
	}

	arb = nvgpu_kzalloc(g, sizeof(struct nvgpu_clk_arb));
	if (arb == NULL) {
		return -ENOMEM;
	}

	arb->clk_arb_events_supported = false;

	err = nvgpu_mutex_init(&arb->pstate_lock);
	if (err != 0) {
		goto mutex_fail;
	}

	nvgpu_spinlock_init(&arb->sessions_lock);
	nvgpu_spinlock_init(&arb->users_lock);
	nvgpu_spinlock_init(&arb->requests_lock);

	arb->gpc2clk_f_points = nvgpu_kcalloc(g, MAX_F_POINTS, sizeof(u16));
	if (arb->gpc2clk_f_points == NULL) {
		err = -ENOMEM;
		goto init_fail;
	}

	for (index = 0; index < 2; index++) {
		table = &arb->vf_table_pool[index];
		table->gpc2clk_num_points = MAX_F_POINTS;

		table->gpc2clk_points = (struct nvgpu_clk_vf_point *)
			nvgpu_kcalloc(g, MAX_F_POINTS,
			sizeof(struct nvgpu_clk_vf_point));
		if (table->gpc2clk_points == NULL) {
			err = -ENOMEM;
			goto init_fail;
		}
	}

	g->clk_arb = arb;
	arb->g = g;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			CTRL_CLK_DOMAIN_GPC2CLK, &default_mhz);
	if (err < 0) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->gpc2clk_default_mhz = default_mhz;

	err = g->ops.clk_arb.get_arbiter_clk_range(g, CTRL_CLK_DOMAIN_GPC2CLK,
		&arb->gpc2clk_min, &arb->gpc2clk_max);

	if (err < 0) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->actual = &arb->actual_pool[0];

	nvgpu_atomic_set(&arb->req_nr, 0);

	nvgpu_atomic64_set(&arb->alarm_mask, 0);
	err = nvgpu_clk_notification_queue_alloc(g, &arb->notification_queue,
		DEFAULT_EVENT_NUMBER);
	if (err < 0) {
		goto init_fail;
	}

	nvgpu_init_list_node(&arb->users);
	nvgpu_init_list_node(&arb->sessions);
	nvgpu_init_list_node(&arb->requests);

	err = nvgpu_cond_init(&arb->request_wq);
	if (err < 0) {
		goto init_fail;
	}

	nvgpu_init_list_node(&arb->update_arb_work_item.worker_item);
	arb->update_arb_work_item.arb = arb;
	arb->update_arb_work_item.item_type = CLK_ARB_WORK_UPDATE_ARB;

	err = nvgpu_clk_arb_worker_init(g);
	if (err < 0) {
		goto init_fail;
	}

	/* This is set for the duration of the default req */
	nvgpu_atomic_inc(&g->clk_arb_global_nr);

	nvgpu_clk_arb_worker_enqueue(g, &arb->update_arb_work_item);

	do {
		/* Check that first run is completed */
		nvgpu_smp_mb();
		NVGPU_COND_WAIT_INTERRUPTIBLE(&arb->request_wq,
			nvgpu_atomic_read(&arb->req_nr) != 0, 0);
	} while (nvgpu_atomic_read(&arb->req_nr) == 0);

	/* Once the default request is completed, reduce the usage count */
	nvgpu_atomic_dec(&g->clk_arb_global_nr);

	return arb->status;

init_fail:
	nvgpu_kfree(g, arb->gpc2clk_f_points);

	for (index = 0; index < 2; index++) {
		nvgpu_kfree(g, arb->vf_table_pool[index].gpc2clk_points);
	}

	nvgpu_mutex_destroy(&arb->pstate_lock);

mutex_fail:
	nvgpu_kfree(g, arb);

	return err;
}

void gp10b_clk_arb_run_arbiter_cb(struct nvgpu_clk_arb *arb)
{
	struct nvgpu_clk_session *session;
	struct nvgpu_clk_dev *dev;
	struct nvgpu_clk_dev *tmp;
	struct nvgpu_clk_arb_target *target, *actual;
	struct gk20a *g = arb->g;

	bool gpc2clk_set;

	int status = 0;
	unsigned long rounded_rate = 0;

	u16 gpc2clk_target, gpc2clk_session_target;

	clk_arb_dbg(g, " ");

	/* Only one arbiter should be running */
	gpc2clk_target = 0;

	nvgpu_spinlock_acquire(&arb->sessions_lock);
	nvgpu_list_for_each_entry(session, &arb->sessions,
			nvgpu_clk_session, link) {
		if (session->zombie) {
			continue;
		}
		gpc2clk_set = false;
		target = (session->target == &session->target_pool[0] ?
				&session->target_pool[1] :
				&session->target_pool[0]);
		nvgpu_spinlock_acquire(&session->session_lock);
		if (nvgpu_list_empty(&session->targets) == 0) {
			/* Copy over state */
			target->gpc2clk = session->target->gpc2clk;
			/* Query the latest committed request */
			nvgpu_list_for_each_entry_safe(dev, tmp, &session->targets,
						nvgpu_clk_dev, node) {
				if (!gpc2clk_set &&
					dev->gpc2clk_target_mhz != (u16)0) {
					target->gpc2clk =
						dev->gpc2clk_target_mhz;
					gpc2clk_set = true;
				}
				nvgpu_ref_get(&dev->refcount);
				nvgpu_list_del(&dev->node);
				nvgpu_spinlock_acquire(&arb->requests_lock);
				nvgpu_list_add(&dev->node, &arb->requests);
				nvgpu_spinlock_release(&arb->requests_lock);
			}
			session->target = target;
		}
		nvgpu_spinlock_release(&session->session_lock);

		gpc2clk_target =
			gpc2clk_target > session->target->gpc2clk ?
			gpc2clk_target : session->target->gpc2clk;
	}
	nvgpu_spinlock_release(&arb->sessions_lock);

	gpc2clk_target = (gpc2clk_target > (u16)0) ? gpc2clk_target :
			arb->gpc2clk_default_mhz;

	if (gpc2clk_target < arb->gpc2clk_min) {
		gpc2clk_target = arb->gpc2clk_min;
	}

	if (gpc2clk_target > arb->gpc2clk_max) {
		gpc2clk_target = arb->gpc2clk_max;
	}

	gpc2clk_session_target = gpc2clk_target;

	if (arb->actual->gpc2clk == gpc2clk_target) {
		nvgpu_atomic_inc(&arb->req_nr);
		nvgpu_cond_signal_interruptible(&arb->request_wq);
		goto exit_arb;
	}

	nvgpu_mutex_acquire(&arb->pstate_lock);

  /* get the rounded_rate in terms of Hz for igpu
   * pass (gpcclk) freq = (gpc2clk) freq / 2
   */
	status = g->ops.clk.clk_get_round_rate(g,
		CTRL_CLK_DOMAIN_GPCCLK, (gpc2clk_session_target/2) * 1000000UL, &rounded_rate);

	clk_arb_dbg(g, "rounded_rate: %lu\n",
		rounded_rate);

	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		nvgpu_atomic_inc(&arb->req_nr);
		nvgpu_cond_signal_interruptible(&arb->request_wq);
		goto exit_arb;
	}

	/* the igpu set_rate accepts freq in Hz */
	status = g->ops.clk.set_rate(g, CTRL_CLK_DOMAIN_GPCCLK, rounded_rate);

	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		nvgpu_atomic_inc(&arb->req_nr);
		nvgpu_cond_signal_interruptible(&arb->request_wq);
		goto exit_arb;
	}

	actual = ((NV_ACCESS_ONCE(arb->actual)) == &arb->actual_pool[0] ?
			&arb->actual_pool[1] : &arb->actual_pool[0]);

	/* do not reorder this pointer */
	nvgpu_smp_rmb();
	actual->gpc2clk = gpc2clk_target;
	arb->status = 0;

	/* Make changes visible to other threads */
	nvgpu_smp_wmb();
	arb->actual = actual;

	/* status must be visible before atomic inc */
	nvgpu_smp_wmb();
	nvgpu_atomic_inc(&arb->req_nr);

	/* Unlock pstate change for PG */
	nvgpu_mutex_release(&arb->pstate_lock);

	nvgpu_cond_signal_interruptible(&arb->request_wq);

exit_arb:
	if (status < 0) {
		nvgpu_err(g, "Error in arbiter update");
	}

	/* notify completion for all requests */
	nvgpu_spinlock_acquire(&arb->requests_lock);
	nvgpu_list_for_each_entry_safe(dev, tmp, &arb->requests,
			nvgpu_clk_dev, node) {
		nvgpu_atomic_set(&dev->poll_mask, NVGPU_POLLIN | NVGPU_POLLRDNORM);
		nvgpu_clk_arb_event_post_event(dev);
		nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);
		nvgpu_list_del(&dev->node);
	}
	nvgpu_spinlock_release(&arb->requests_lock);

	clk_arb_dbg(g, "done");
}

void gp10b_clk_arb_cleanup(struct nvgpu_clk_arb *arb)
{
	struct gk20a *g = arb->g;
	int index;

	nvgpu_kfree(g, arb->gpc2clk_f_points);

	for (index = 0; index < 2; index++) {
		nvgpu_kfree(g,
			arb->vf_table_pool[index].gpc2clk_points);
	}

	nvgpu_mutex_destroy(&g->clk_arb->pstate_lock);
	nvgpu_kfree(g, g->clk_arb);

	g->clk_arb = NULL;
}
