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

#include <nvgpu/list.h>
#include <nvgpu/clk_arb.h>

/**
 * Stub imlementation of the clk_arb code. Yikes. Much of this probably could be
 * commonized if one were to think through the implementation but that is
 * probably weeks of work at a minimum.
 *
 * So for POSIX it will be stubbed.
 */

int nvgpu_clk_arb_init_arbiter(struct gk20a *g)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_get_arbiter_clk_range(struct gk20a *g, u32 api_domain,
					u16 *min_mhz, u16 *max_mhz)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_update_vf_table(struct nvgpu_clk_arb *arb)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_worker_init(struct gk20a *g)
{
	return -ENOSYS;
}

bool nvgpu_clk_arb_has_active_req(struct gk20a *g)
{
	return false;
}

int nvgpu_clk_arb_get_arbiter_actual_mhz(struct gk20a *g,
					 u32 api_domain, u16 *actual_mhz)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_get_arbiter_effective_mhz(struct gk20a *g,
					    u32 api_domain, u16 *effective_mhz)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_get_arbiter_clk_f_points(struct gk20a *g,
					   u32 api_domain,
					   u32 *max_points, u16 *fpoints)
{
	return -ENOSYS;
}

u32 nvgpu_clk_arb_get_arbiter_clk_domains(struct gk20a *g)
{
	return 0;
}

bool nvgpu_clk_arb_is_valid_domain(struct gk20a *g, u32 api_domain)
{
	return false;
}

void nvgpu_clk_arb_cleanup_arbiter(struct gk20a *g)
{
}

int nvgpu_clk_arb_install_session_fd(struct gk20a *g,
				     struct nvgpu_clk_session *session)
{
	return -ENOSYS;
}


int nvgpu_clk_arb_init_session(struct gk20a *g,
			       struct nvgpu_clk_session **_session)
{
	return -ENOSYS;
}

void nvgpu_clk_arb_release_session(struct gk20a *g,
				   struct nvgpu_clk_session *session)
{
}

int nvgpu_clk_arb_commit_request_fd(struct gk20a *g,
				    struct nvgpu_clk_session *session,
				    int request_fd)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_set_session_target_mhz(struct nvgpu_clk_session *session,
					 int fd, u32 api_domain, u16 target_mhz)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_get_session_target_mhz(struct nvgpu_clk_session *session,
					 u32 api_domain, u16 *target_mhz)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_install_event_fd(struct gk20a *g,
				   struct nvgpu_clk_session *session,
				   int *event_fd, u32 alarm_mask)
{
	return -ENOSYS;
}

int nvgpu_clk_arb_install_request_fd(struct gk20a *g,
				     struct nvgpu_clk_session *session,
				     int *event_fd)
{
	return -ENOSYS;
}

u32 nvgpu_clk_arb_notify(struct nvgpu_clk_dev *dev,
				struct nvgpu_clk_arb_target *target,
				u32 alarm)
{
	return 0;
}

void nvgpu_clk_arb_free_fd(struct nvgpu_ref *refcount)
{
}

void nvgpu_clk_arb_schedule_vf_table_update(struct gk20a *g)
{
}

int nvgpu_clk_arb_get_current_pstate(struct gk20a *g)
{
	return -ENOSYS;
}

void nvgpu_clk_arb_pstate_change_lock(struct gk20a *g, bool lock)
{
}

void nvgpu_clk_arb_send_thermal_alarm(struct gk20a *g)
{
}

void nvgpu_clk_arb_schedule_alarm(struct gk20a *g, u32 alarm)
{
}

void nvgpu_clk_arb_set_global_alarm(struct gk20a *g, u32 alarm)
{
}

void nvgpu_clk_arb_clear_global_alarm(struct gk20a *g, u32 alarm)
{
}

void nvgpu_clk_arb_event_post_event(struct nvgpu_clk_dev *dev)
{
}

void nvgpu_clk_arb_worker_enqueue(struct gk20a *g,
		struct nvgpu_clk_arb_work_item *work_item)
{
}

int nvgpu_clk_notification_queue_alloc(struct gk20a *g,
				struct nvgpu_clk_notification_queue *queue,
				size_t events_number)
{
	return -ENOSYS;
}
