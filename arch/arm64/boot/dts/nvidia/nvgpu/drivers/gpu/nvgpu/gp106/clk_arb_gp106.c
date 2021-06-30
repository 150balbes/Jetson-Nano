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

#include <nvgpu/gk20a.h>
#include <nvgpu/clk_arb.h>

#include "clk_arb_gp106.h"

u32 gp106_get_arbiter_clk_domains(struct gk20a *g)
{
	(void)g;
	return (CTRL_CLK_DOMAIN_MCLK|CTRL_CLK_DOMAIN_GPC2CLK);
}

int gp106_get_arbiter_f_points(struct gk20a *g,u32 api_domain,
				u32 *num_points, u16 *freqs_in_mhz)
{
	return g->ops.clk.clk_domain_get_f_points(g,
		api_domain, num_points, freqs_in_mhz);
}

int gp106_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
		u16 *min_mhz, u16 *max_mhz)
{
	enum nv_pmu_clk_clkwhich clkwhich;
	struct clk_set_info *p0_info;
	struct clk_set_info *p5_info;
	struct avfsfllobjs *pfllobjs =  &(g->clk_pmu.avfs_fllobjs);

	u16 limit_min_mhz;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_MCLK:
		clkwhich = clkwhich_mclk;
		break;

	case CTRL_CLK_DOMAIN_GPC2CLK:
		clkwhich = clkwhich_gpc2clk;
		break;

	default:
		return -EINVAL;
	}

	p5_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P5, clkwhich);
	if (!p5_info) {
		return -EINVAL;
	}

	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich);
	if (!p0_info) {
		return -EINVAL;
	}

	limit_min_mhz = p5_info->min_mhz;
	/* WAR for DVCO min */
	if (api_domain == CTRL_CLK_DOMAIN_GPC2CLK) {
		if ((pfllobjs->max_min_freq_mhz) &&
			(pfllobjs->max_min_freq_mhz >= limit_min_mhz)) {
			limit_min_mhz = pfllobjs->max_min_freq_mhz + 1;
		}
	}

	*min_mhz = limit_min_mhz;
	*max_mhz = p0_info->max_mhz;

	return 0;
}

int gp106_get_arbiter_clk_default(struct gk20a *g, u32 api_domain,
		u16 *default_mhz)
{
	enum nv_pmu_clk_clkwhich clkwhich;
	struct clk_set_info *p0_info;

	switch (api_domain) {
	case CTRL_CLK_DOMAIN_MCLK:
		clkwhich = clkwhich_mclk;
		break;

	case CTRL_CLK_DOMAIN_GPC2CLK:
		clkwhich = clkwhich_gpc2clk;
		break;

	default:
		return -EINVAL;
	}

	p0_info = pstate_get_clk_set_info(g,
			CTRL_PERF_PSTATE_P0, clkwhich);
	if (!p0_info) {
		return -EINVAL;
	}

	*default_mhz = p0_info->max_mhz;

	return 0;
}

int gp106_init_clk_arbiter(struct gk20a *g)
{
	struct nvgpu_clk_arb *arb;
	u16 default_mhz;
	int err;
	int index;
	struct nvgpu_clk_vf_table *table;

	clk_arb_dbg(g, " ");

	if (g->clk_arb != NULL) {
		return 0;
	}

	arb = nvgpu_kzalloc(g, sizeof(struct nvgpu_clk_arb));
	if (!arb)
		return -ENOMEM;

	arb->clk_arb_events_supported = true;

	err = nvgpu_mutex_init(&arb->pstate_lock);
	if (err)
		goto mutex_fail;
	nvgpu_spinlock_init(&arb->sessions_lock);
	nvgpu_spinlock_init(&arb->users_lock);
	nvgpu_spinlock_init(&arb->requests_lock);

	arb->mclk_f_points = nvgpu_kcalloc(g, MAX_F_POINTS, sizeof(u16));
	if (!arb->mclk_f_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	arb->gpc2clk_f_points = nvgpu_kcalloc(g, MAX_F_POINTS, sizeof(u16));
	if (!arb->gpc2clk_f_points) {
		err = -ENOMEM;
		goto init_fail;
	}

	for (index = 0; index < 2; index++) {
		table = &arb->vf_table_pool[index];
		table->gpc2clk_num_points = MAX_F_POINTS;
		table->mclk_num_points = MAX_F_POINTS;

		table->gpc2clk_points = nvgpu_kcalloc(g, MAX_F_POINTS,
			sizeof(struct nvgpu_clk_vf_point));
		if (!table->gpc2clk_points) {
			err = -ENOMEM;
			goto init_fail;
		}


		table->mclk_points = nvgpu_kcalloc(g, MAX_F_POINTS,
			sizeof(struct nvgpu_clk_vf_point));
		if (!table->mclk_points) {
			err = -ENOMEM;
			goto init_fail;
		}
	}

	g->clk_arb = arb;
	arb->g = g;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			CTRL_CLK_DOMAIN_MCLK, &default_mhz);
	if (err < 0) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->mclk_default_mhz = default_mhz;

	err =  g->ops.clk_arb.get_arbiter_clk_default(g,
			CTRL_CLK_DOMAIN_GPC2CLK, &default_mhz);
	if (err < 0) {
		err = -EINVAL;
		goto init_fail;
	}

	arb->gpc2clk_default_mhz = default_mhz;

	arb->actual = &arb->actual_pool[0];

	nvgpu_atomic_set(&arb->req_nr, 0);

	nvgpu_atomic64_set(&arb->alarm_mask, 0);
	err = nvgpu_clk_notification_queue_alloc(g, &arb->notification_queue,
		DEFAULT_EVENT_NUMBER);
	if (err < 0)
		goto init_fail;

	nvgpu_init_list_node(&arb->users);
	nvgpu_init_list_node(&arb->sessions);
	nvgpu_init_list_node(&arb->requests);

	nvgpu_cond_init(&arb->request_wq);

	nvgpu_init_list_node(&arb->update_vf_table_work_item.worker_item);
	nvgpu_init_list_node(&arb->update_arb_work_item.worker_item);
	arb->update_vf_table_work_item.arb = arb;
	arb->update_arb_work_item.arb = arb;
	arb->update_vf_table_work_item.item_type = CLK_ARB_WORK_UPDATE_VF_TABLE;
	arb->update_arb_work_item.item_type = CLK_ARB_WORK_UPDATE_ARB;

	err = nvgpu_clk_arb_worker_init(g);
	if (err < 0)
		goto init_fail;

#ifdef CONFIG_DEBUG_FS
	arb->debug = &arb->debug_pool[0];

	if (!arb->debugfs_set) {
		if (nvgpu_clk_arb_debugfs_init(g))
			arb->debugfs_set = true;
	}
#endif
	err = clk_vf_point_cache(g);
	if (err < 0)
		goto init_fail;

	err = nvgpu_clk_arb_update_vf_table(arb);
	if (err < 0)
		goto init_fail;
	do {
		/* Check that first run is completed */
		nvgpu_smp_mb();
		NVGPU_COND_WAIT_INTERRUPTIBLE(&arb->request_wq,
			nvgpu_atomic_read(&arb->req_nr), 0);
	} while (!nvgpu_atomic_read(&arb->req_nr));


	return arb->status;

init_fail:
	nvgpu_kfree(g, arb->gpc2clk_f_points);
	nvgpu_kfree(g, arb->mclk_f_points);

	for (index = 0; index < 2; index++) {
		nvgpu_kfree(g, arb->vf_table_pool[index].gpc2clk_points);
		nvgpu_kfree(g, arb->vf_table_pool[index].mclk_points);
	}

	nvgpu_mutex_destroy(&arb->pstate_lock);

mutex_fail:
	nvgpu_kfree(g, arb);

	return err;
}

static u8 nvgpu_clk_arb_find_vf_point(struct nvgpu_clk_arb *arb,
		u16 *gpc2clk, u16 *sys2clk, u16 *xbar2clk, u16 *mclk,
		u32 *voltuv, u32 *voltuv_sram, u32 *nuvmin, u32 *nuvmin_sram)
{
	u16 gpc2clk_target, mclk_target;
	u32 gpc2clk_voltuv, gpc2clk_voltuv_sram;
	u32 mclk_voltuv, mclk_voltuv_sram;
	u32 pstate = VF_POINT_INVALID_PSTATE;
	struct nvgpu_clk_vf_table *table;
	u32 index, index_mclk;
	struct nvgpu_clk_vf_point *mclk_vf = NULL;

	do {
		gpc2clk_target = *gpc2clk;
		mclk_target = *mclk;
		gpc2clk_voltuv = 0;
		gpc2clk_voltuv_sram = 0;
		mclk_voltuv = 0;
		mclk_voltuv_sram = 0;

		table = NV_ACCESS_ONCE(arb->current_vf_table);
		/* pointer to table can be updated by callback */
		nvgpu_smp_rmb();

		if (!table)
			continue;
		if ((!table->gpc2clk_num_points) || (!table->mclk_num_points)) {
			nvgpu_err(arb->g, "found empty table");
			goto find_exit;
		}
		/* First we check MCLK to find out which PSTATE we are
		 * are requesting, and from there try to find the minimum
		 * GPC2CLK on the same PSTATE that satisfies the request.
		 * If no GPC2CLK can be found, then we need to up the PSTATE
		 */

recalculate_vf_point:
		for (index = 0; index < table->mclk_num_points; index++) {
			if (table->mclk_points[index].mem_mhz >= mclk_target) {
				mclk_vf = &table->mclk_points[index];
				break;
			}
		}
		if (index == table->mclk_num_points) {
			mclk_vf = &table->mclk_points[index-1];
			index = table->mclk_num_points - 1;
		}
		index_mclk = index;

		/* round up the freq requests */
		for (index = 0; index < table->gpc2clk_num_points; index++) {
			pstate = VF_POINT_COMMON_PSTATE(
					&table->gpc2clk_points[index], mclk_vf);

			if ((table->gpc2clk_points[index].gpc_mhz >=
							gpc2clk_target) &&
					(pstate != VF_POINT_INVALID_PSTATE)) {
				gpc2clk_target =
					table->gpc2clk_points[index].gpc_mhz;
				*sys2clk =
					table->gpc2clk_points[index].sys_mhz;
				*xbar2clk =
					table->gpc2clk_points[index].xbar_mhz;

				gpc2clk_voltuv =
					table->gpc2clk_points[index].uvolt;
				gpc2clk_voltuv_sram =
					table->gpc2clk_points[index].uvolt_sram;
				break;
			}
		}

		if (index == table->gpc2clk_num_points) {
			pstate = VF_POINT_COMMON_PSTATE(
				&table->gpc2clk_points[index-1], mclk_vf);
			if (pstate != VF_POINT_INVALID_PSTATE) {
				gpc2clk_target =
					table->gpc2clk_points[index-1].gpc_mhz;
				*sys2clk =
					table->gpc2clk_points[index-1].sys_mhz;
				*xbar2clk  =
					table->gpc2clk_points[index-1].xbar_mhz;

				gpc2clk_voltuv =
					table->gpc2clk_points[index-1].uvolt;
				gpc2clk_voltuv_sram =
					table->gpc2clk_points[index-1].
						uvolt_sram;
			} else if (index_mclk >= table->mclk_num_points - 1) {
				/* There is no available combination of MCLK
				 * and GPC2CLK, we need to fail this
				 */
				gpc2clk_target = 0;
				mclk_target = 0;
				pstate = VF_POINT_INVALID_PSTATE;
				goto find_exit;
			} else {
				/* recalculate with higher PSTATE */
				gpc2clk_target = *gpc2clk;
				mclk_target = table->mclk_points[index_mclk+1].
									mem_mhz;
				goto recalculate_vf_point;
			}
		}

		mclk_target = mclk_vf->mem_mhz;
		mclk_voltuv = mclk_vf->uvolt;
		mclk_voltuv_sram = mclk_vf->uvolt_sram;

	} while (!table ||
		(NV_ACCESS_ONCE(arb->current_vf_table) != table));

find_exit:
	*voltuv = gpc2clk_voltuv > mclk_voltuv ? gpc2clk_voltuv : mclk_voltuv;
	*voltuv_sram = gpc2clk_voltuv_sram > mclk_voltuv_sram ?
		gpc2clk_voltuv_sram : mclk_voltuv_sram;
	/* noise unaware vmin */
	*nuvmin = mclk_voltuv;
	*nuvmin_sram = mclk_voltuv_sram;
	*gpc2clk = gpc2clk_target < *gpc2clk ? gpc2clk_target : *gpc2clk;
	*mclk = mclk_target;
	return pstate;
}

static int nvgpu_clk_arb_change_vf_point(struct gk20a *g, u16 gpc2clk_target,
	u16 sys2clk_target, u16 xbar2clk_target, u16 mclk_target, u32 voltuv,
	u32 voltuv_sram)
{
	struct set_fll_clk fllclk;
	struct nvgpu_clk_arb *arb = g->clk_arb;
	int status;

	fllclk.gpc2clkmhz = gpc2clk_target;
	fllclk.sys2clkmhz = sys2clk_target;
	fllclk.xbar2clkmhz = xbar2clk_target;

	fllclk.voltuv = voltuv;

	/* if voltage ascends we do:
	 * (1) FLL change
	 * (2) Voltage change
	 * (3) MCLK change
	 * If it goes down
	 * (1) MCLK change
	 * (2) Voltage change
	 * (3) FLL change
	 */

	/* descending */
	if (voltuv < arb->voltuv_actual) {
		status = g->ops.clk.mclk_change(g, mclk_target);
		if (status < 0)
			return status;

		status = volt_set_voltage(g, voltuv, voltuv_sram);
		if (status < 0)
			return status;

		status = clk_set_fll_clks(g, &fllclk);
		if (status < 0)
			return status;
	} else {
		status = clk_set_fll_clks(g, &fllclk);
		if (status < 0)
			return status;

		status = volt_set_voltage(g, voltuv, voltuv_sram);
		if (status < 0)
			return status;

		status = g->ops.clk.mclk_change(g, mclk_target);
		if (status < 0)
			return status;
	}

	return 0;
}

void gp106_clk_arb_run_arbiter_cb(struct nvgpu_clk_arb *arb)
{
	struct nvgpu_clk_session *session;
	struct nvgpu_clk_dev *dev;
	struct nvgpu_clk_dev *tmp;
	struct nvgpu_clk_arb_target *target, *actual;
	struct gk20a *g = arb->g;

	u32 pstate = VF_POINT_INVALID_PSTATE;
	u32 voltuv, voltuv_sram;
	bool mclk_set, gpc2clk_set;
	u32 nuvmin, nuvmin_sram;

	u32 alarms_notified = 0;
	u32 current_alarm;
	int status = 0;

	/* Temporary variables for checking target frequency */
	u16 gpc2clk_target, sys2clk_target, xbar2clk_target, mclk_target;
	u16 gpc2clk_session_target, mclk_session_target;

#ifdef CONFIG_DEBUG_FS
	u64 t0, t1;
	struct nvgpu_clk_arb_debug *debug;

#endif

	clk_arb_dbg(g, " ");

	/* bail out if gpu is down */
	if (nvgpu_atomic64_read(&arb->alarm_mask) & EVENT(ALARM_GPU_LOST))
		goto exit_arb;

#ifdef CONFIG_DEBUG_FS
	g->ops.ptimer.read_ptimer(g, &t0);
#endif

	/* Only one arbiter should be running */
	gpc2clk_target = 0;
	mclk_target = 0;

	nvgpu_spinlock_acquire(&arb->sessions_lock);
	nvgpu_list_for_each_entry(session, &arb->sessions,
			nvgpu_clk_session, link) {
		if (!session->zombie) {
			mclk_set = false;
			gpc2clk_set = false;
			target = (session->target == &session->target_pool[0] ?
					&session->target_pool[1] :
					&session->target_pool[0]);
			nvgpu_spinlock_acquire(&session->session_lock);
			if (!nvgpu_list_empty(&session->targets)) {
				/* Copy over state */
				target->mclk = session->target->mclk;
				target->gpc2clk = session->target->gpc2clk;
				/* Query the latest committed request */
				nvgpu_list_for_each_entry_safe(dev, tmp,
				 &session->targets, nvgpu_clk_dev, node) {
					if (!mclk_set && dev->mclk_target_mhz) {
						target->mclk =
							dev->mclk_target_mhz;
						mclk_set = true;
					}
					if (!gpc2clk_set &&
						dev->gpc2clk_target_mhz) {
						target->gpc2clk =
							dev->gpc2clk_target_mhz;
						gpc2clk_set = true;
					}
					nvgpu_ref_get(&dev->refcount);
					nvgpu_list_del(&dev->node);
					nvgpu_spinlock_acquire(
						&arb->requests_lock);
					nvgpu_list_add(
						&dev->node, &arb->requests);
					nvgpu_spinlock_release(&arb->requests_lock);
				}
				session->target = target;
			}
			nvgpu_spinlock_release(
				&session->session_lock);

			mclk_target = mclk_target > session->target->mclk ?
				mclk_target : session->target->mclk;

			gpc2clk_target =
				gpc2clk_target > session->target->gpc2clk ?
				gpc2clk_target : session->target->gpc2clk;
		}
	}
	nvgpu_spinlock_release(&arb->sessions_lock);

	gpc2clk_target = (gpc2clk_target > 0) ? gpc2clk_target :
			arb->gpc2clk_default_mhz;

	if (gpc2clk_target < arb->gpc2clk_min)
		gpc2clk_target = arb->gpc2clk_min;

	if (gpc2clk_target > arb->gpc2clk_max)
		gpc2clk_target = arb->gpc2clk_max;

	mclk_target = (mclk_target > 0) ? mclk_target :
			arb->mclk_default_mhz;

	if (mclk_target < arb->mclk_min)
		mclk_target = arb->mclk_min;

	if (mclk_target > arb->mclk_max)
		mclk_target = arb->mclk_max;

	sys2clk_target = 0;
	xbar2clk_target = 0;

	gpc2clk_session_target = gpc2clk_target;
	mclk_session_target = mclk_target;

	/* Query the table for the closest vf point to program */
	pstate = nvgpu_clk_arb_find_vf_point(arb, &gpc2clk_target,
		&sys2clk_target, &xbar2clk_target, &mclk_target, &voltuv,
		&voltuv_sram, &nuvmin, &nuvmin_sram);

	if (pstate == VF_POINT_INVALID_PSTATE) {
		arb->status = -EINVAL;
		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	if ((gpc2clk_target < gpc2clk_session_target) ||
			(mclk_target < mclk_session_target))
		nvgpu_clk_arb_set_global_alarm(g,
			EVENT(ALARM_TARGET_VF_NOT_POSSIBLE));

	if ((arb->actual->gpc2clk == gpc2clk_target) &&
		(arb->actual->mclk == mclk_target) &&
		(arb->voltuv_actual == voltuv)) {
		goto exit_arb;
	}

	/* Program clocks */
	/* A change in both mclk of gpc2clk may require a change in voltage */

	nvgpu_mutex_acquire(&arb->pstate_lock);
	status = nvgpu_lpwr_disable_pg(g, false);

	status = clk_pmu_freq_controller_load(g, false,
					CTRL_CLK_CLK_FREQ_CONTROLLER_ID_ALL);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}
	status = volt_set_noiseaware_vmin(g, nuvmin, nuvmin_sram);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	status = nvgpu_clk_arb_change_vf_point(g, gpc2clk_target,
		sys2clk_target, xbar2clk_target, mclk_target, voltuv,
		voltuv_sram);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	status = clk_pmu_freq_controller_load(g, true,
					 CTRL_CLK_CLK_FREQ_CONTROLLER_ID_ALL);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	status = nvgpu_lwpr_mclk_change(g, pstate);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	actual = NV_ACCESS_ONCE(arb->actual) == &arb->actual_pool[0] ?
			&arb->actual_pool[1] : &arb->actual_pool[0];

	/* do not reorder this pointer */
	nvgpu_smp_rmb();
	actual->gpc2clk = gpc2clk_target;
	actual->mclk = mclk_target;
	arb->voltuv_actual = voltuv;
	actual->pstate = pstate;
	arb->status = status;

	/* Make changes visible to other threads */
	nvgpu_smp_wmb();
	arb->actual = actual;

	status = nvgpu_lpwr_enable_pg(g, false);
	if (status < 0) {
		arb->status = status;
		nvgpu_mutex_release(&arb->pstate_lock);

		/* make status visible */
		nvgpu_smp_mb();
		goto exit_arb;
	}

	/* status must be visible before atomic inc */
	nvgpu_smp_wmb();
	nvgpu_atomic_inc(&arb->req_nr);

	/* Unlock pstate change for PG */
	nvgpu_mutex_release(&arb->pstate_lock);

	/* VF Update complete */
	nvgpu_clk_arb_set_global_alarm(g, EVENT(VF_UPDATE));

	nvgpu_cond_signal_interruptible(&arb->request_wq);

#ifdef CONFIG_DEBUG_FS
	g->ops.ptimer.read_ptimer(g, &t1);

	debug = arb->debug == &arb->debug_pool[0] ?
		&arb->debug_pool[1] : &arb->debug_pool[0];

	memcpy(debug, arb->debug, sizeof(arb->debug_pool[0]));
	debug->switch_num++;

	if (debug->switch_num == 1) {
		debug->switch_max = debug->switch_min =
			debug->switch_avg = (t1-t0)/1000;
		debug->switch_std = 0;
	} else {
		s64 prev_avg;
		s64 curr = (t1-t0)/1000;

		debug->switch_max = curr > debug->switch_max ?
			curr : debug->switch_max;
		debug->switch_min = debug->switch_min ?
			(curr < debug->switch_min ?
				curr : debug->switch_min) : curr;
		prev_avg = debug->switch_avg;
		debug->switch_avg = (curr +
			(debug->switch_avg * (debug->switch_num-1))) /
			debug->switch_num;
		debug->switch_std +=
			(curr - debug->switch_avg) * (curr - prev_avg);
	}
	/* commit changes before exchanging debug pointer */
	nvgpu_smp_wmb();
	arb->debug = debug;
#endif

exit_arb:
	if (status < 0) {
		nvgpu_err(g, "Error in arbiter update");
		nvgpu_clk_arb_set_global_alarm(g,
			EVENT(ALARM_CLOCK_ARBITER_FAILED));
	}

	current_alarm = (u32) nvgpu_atomic64_read(&arb->alarm_mask);
	/* notify completion for all requests */
	nvgpu_spinlock_acquire(&arb->requests_lock);
	nvgpu_list_for_each_entry_safe(dev, tmp, &arb->requests,
			nvgpu_clk_dev, node) {
		nvgpu_atomic_set(&dev->poll_mask,
		 NVGPU_POLLIN | NVGPU_POLLRDNORM);
		nvgpu_clk_arb_event_post_event(dev);
		nvgpu_ref_put(&dev->refcount, nvgpu_clk_arb_free_fd);
		nvgpu_list_del(&dev->node);
	}
	nvgpu_spinlock_release(&arb->requests_lock);

	nvgpu_atomic_set(&arb->notification_queue.head,
		nvgpu_atomic_read(&arb->notification_queue.tail));
	/* notify event for all users */
	nvgpu_spinlock_acquire(&arb->users_lock);
	nvgpu_list_for_each_entry(dev, &arb->users, nvgpu_clk_dev, link) {
		alarms_notified |=
			nvgpu_clk_arb_notify(dev, arb->actual, current_alarm);
	}
	nvgpu_spinlock_release(&arb->users_lock);

	/* clear alarms */
	nvgpu_clk_arb_clear_global_alarm(g, alarms_notified &
		~EVENT(ALARM_GPU_LOST));
}

void gp106_clk_arb_cleanup(struct nvgpu_clk_arb *arb)
{
	struct gk20a *g = arb->g;
	int index;

	nvgpu_kfree(g, arb->gpc2clk_f_points);
	nvgpu_kfree(g, arb->mclk_f_points);

	for (index = 0; index < 2; index++) {
		nvgpu_kfree(g,
			arb->vf_table_pool[index].gpc2clk_points);
		nvgpu_kfree(g, arb->vf_table_pool[index].mclk_points);
	}

	nvgpu_mutex_destroy(&g->clk_arb->pstate_lock);
	nvgpu_kfree(g, g->clk_arb);

	g->clk_arb = NULL;
}