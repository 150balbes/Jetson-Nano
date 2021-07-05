/*
 * Copyright (c) 2018-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/channel.h>
#include <nvgpu/ltc.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel_sync.h>
#include <nvgpu/vpr.h>

#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

#include "gk20a/fence_gk20a.h"

#include <trace/events/gk20a.h>

/*
 * Handle the submit synchronization - pre-fences and post-fences.
 */
static int nvgpu_submit_prepare_syncs(struct channel_gk20a *c,
				      struct nvgpu_channel_fence *fence,
				      struct channel_gk20a_job *job,
				      struct priv_cmd_entry **wait_cmd,
				      struct priv_cmd_entry **incr_cmd,
				      struct gk20a_fence **post_fence,
				      bool register_irq,
				      u32 flags)
{
	struct gk20a *g = c->g;
	bool need_sync_fence = false;
	bool new_sync_created = false;
	int wait_fence_fd = -1;
	int err = 0;
	bool need_wfi = !(flags & NVGPU_SUBMIT_FLAGS_SUPPRESS_WFI);
	bool pre_alloc_enabled = channel_gk20a_is_prealloc_enabled(c);

	if (g->aggressive_sync_destroy_thresh) {
		nvgpu_mutex_acquire(&c->sync_lock);
		if (!c->sync) {
			c->sync = nvgpu_channel_sync_create(c, false);
			if (!c->sync) {
				err = -ENOMEM;
				goto fail;
			}
			new_sync_created = true;
		}
		nvgpu_atomic_inc(&c->sync->refcount);
	}

	if (g->ops.fifo.resetup_ramfc && new_sync_created) {
		err = g->ops.fifo.resetup_ramfc(c);
		if (err) {
			goto fail;
		}
	}

	/*
	 * Optionally insert syncpt/semaphore wait in the beginning of gpfifo
	 * submission when user requested and the wait hasn't expired.
	 */
	if (flags & NVGPU_SUBMIT_FLAGS_FENCE_WAIT) {
		int max_wait_cmds = c->deterministic ? 1 : 0;

		if (!pre_alloc_enabled) {
			job->wait_cmd = nvgpu_kzalloc(g,
				sizeof(struct priv_cmd_entry));
		}

		if (!job->wait_cmd) {
			err = -ENOMEM;
			goto fail;
		}

		if (flags & NVGPU_SUBMIT_FLAGS_SYNC_FENCE) {
			wait_fence_fd = fence->id;
			err = c->sync->wait_fd(c->sync, wait_fence_fd,
					       job->wait_cmd, max_wait_cmds);
		} else {
			err = c->sync->wait_syncpt(c->sync, fence->id,
						   fence->value,
						   job->wait_cmd);
		}

		if (err) {
			goto clean_up_wait_cmd;
		}

		if (job->wait_cmd->valid) {
			*wait_cmd = job->wait_cmd;
		}
	}

	if ((flags & NVGPU_SUBMIT_FLAGS_FENCE_GET) &&
	    (flags & NVGPU_SUBMIT_FLAGS_SYNC_FENCE)) {
		need_sync_fence = true;
	}

	/*
	 * Always generate an increment at the end of a GPFIFO submission. This
	 * is used to keep track of method completion for idle railgating. The
	 * sync_pt/semaphore PB is added to the GPFIFO later on in submit.
	 */
	job->post_fence = gk20a_alloc_fence(c);
	if (!job->post_fence) {
		err = -ENOMEM;
		goto clean_up_wait_cmd;
	}
	if (!pre_alloc_enabled) {
		job->incr_cmd = nvgpu_kzalloc(g, sizeof(struct priv_cmd_entry));
	}

	if (!job->incr_cmd) {
		err = -ENOMEM;
		goto clean_up_post_fence;
	}

	if (flags & NVGPU_SUBMIT_FLAGS_FENCE_GET) {
		err = c->sync->incr_user(c->sync, wait_fence_fd, job->incr_cmd,
				 job->post_fence, need_wfi, need_sync_fence,
				 register_irq);
	} else {
		err = c->sync->incr(c->sync, job->incr_cmd,
				    job->post_fence, need_sync_fence,
				    register_irq);
	}
	if (!err) {
		*incr_cmd = job->incr_cmd;
		*post_fence = job->post_fence;
	} else {
		goto clean_up_incr_cmd;
	}

	if (g->aggressive_sync_destroy_thresh) {
		nvgpu_mutex_release(&c->sync_lock);
	}
	return 0;

clean_up_incr_cmd:
	free_priv_cmdbuf(c, job->incr_cmd);
	if (!pre_alloc_enabled) {
		job->incr_cmd = NULL;
	}
clean_up_post_fence:
	gk20a_fence_put(job->post_fence);
	job->post_fence = NULL;
clean_up_wait_cmd:
	if (job->wait_cmd) {
		free_priv_cmdbuf(c, job->wait_cmd);
	}
	if (!pre_alloc_enabled) {
		job->wait_cmd = NULL;
	}
fail:
	if (g->aggressive_sync_destroy_thresh) {
		nvgpu_mutex_release(&c->sync_lock);
	}
	*wait_cmd = NULL;
	return err;
}

static void nvgpu_submit_append_priv_cmdbuf(struct channel_gk20a *c,
		struct priv_cmd_entry *cmd)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *gpfifo_mem = &c->gpfifo.mem;
	struct nvgpu_gpfifo_entry x = {
		.entry0 = u64_lo32(cmd->gva),
		.entry1 = u64_hi32(cmd->gva) |
			pbdma_gp_entry1_length_f(cmd->size)
	};

	nvgpu_mem_wr_n(g, gpfifo_mem, c->gpfifo.put * sizeof(x),
			&x, sizeof(x));

	if (cmd->mem->aperture == APERTURE_SYSMEM) {
		trace_gk20a_push_cmdbuf(g->name, 0, cmd->size, 0,
				(u32 *)cmd->mem->cpu_va + cmd->off);
	}

	c->gpfifo.put = (c->gpfifo.put + 1U) & (c->gpfifo.entry_num - 1U);
}

static int nvgpu_submit_append_gpfifo_user_direct(struct channel_gk20a *c,
		struct nvgpu_gpfifo_userdata userdata,
		u32 num_entries)
{
	struct gk20a *g = c->g;
	struct nvgpu_gpfifo_entry *gpfifo_cpu = c->gpfifo.mem.cpu_va;
	u32 gpfifo_size = c->gpfifo.entry_num;
	u32 len = num_entries;
	u32 start = c->gpfifo.put;
	u32 end = start + len; /* exclusive */
	int err;

	nvgpu_speculation_barrier();
	if (end > gpfifo_size) {
		/* wrap-around */
		int length0 = gpfifo_size - start;
		int length1 = len - length0;

		err = g->os_channel.copy_user_gpfifo(
				gpfifo_cpu + start, userdata,
				0, length0);
		if (err) {
			return err;
		}

		err = g->os_channel.copy_user_gpfifo(
				gpfifo_cpu, userdata,
				length0, length1);
		if (err) {
			return err;
		}
	} else {
		err = g->os_channel.copy_user_gpfifo(
				gpfifo_cpu + start, userdata,
				0, len);
		if (err) {
			return err;
		}
	}

	return 0;
}

static void nvgpu_submit_append_gpfifo_common(struct channel_gk20a *c,
		struct nvgpu_gpfifo_entry *src, u32 num_entries)
{
	struct gk20a *g = c->g;
	struct nvgpu_mem *gpfifo_mem = &c->gpfifo.mem;
	/* in bytes */
	u32 gpfifo_size =
		c->gpfifo.entry_num * sizeof(struct nvgpu_gpfifo_entry);
	u32 len = num_entries * sizeof(struct nvgpu_gpfifo_entry);
	u32 start = c->gpfifo.put * sizeof(struct nvgpu_gpfifo_entry);
	u32 end = start + len; /* exclusive */

	if (end > gpfifo_size) {
		/* wrap-around */
		int length0 = gpfifo_size - start;
		int length1 = len - length0;
		struct nvgpu_gpfifo_entry *src2 = src + length0;

		nvgpu_mem_wr_n(g, gpfifo_mem, start, src, length0);
		nvgpu_mem_wr_n(g, gpfifo_mem, 0, src2, length1);
	} else {
		nvgpu_mem_wr_n(g, gpfifo_mem, start, src, len);
	}
}

/*
 * Copy source gpfifo entries into the gpfifo ring buffer, potentially
 * splitting into two memcpys to handle wrap-around.
 */
static int nvgpu_submit_append_gpfifo(struct channel_gk20a *c,
		struct nvgpu_gpfifo_entry *kern_gpfifo,
		struct nvgpu_gpfifo_userdata userdata,
		u32 num_entries)
{
	struct gk20a *g = c->g;
	int err;

	if (!kern_gpfifo && !c->gpfifo.pipe) {
		/*
		 * This path (from userspace to sysmem) is special in order to
		 * avoid two copies unnecessarily (from user to pipe, then from
		 * pipe to gpu sysmem buffer).
		 */
		err = nvgpu_submit_append_gpfifo_user_direct(c, userdata,
				num_entries);
		if (err) {
			return err;
		}
	} else if (!kern_gpfifo) {
		/* from userspace to vidmem, use the common path */
		err = g->os_channel.copy_user_gpfifo(c->gpfifo.pipe, userdata,
				0, num_entries);
		if (err) {
			return err;
		}

		nvgpu_submit_append_gpfifo_common(c, c->gpfifo.pipe,
				num_entries);
	} else {
		/* from kernel to either sysmem or vidmem, don't need
		 * copy_user_gpfifo so use the common path */
		nvgpu_submit_append_gpfifo_common(c, kern_gpfifo, num_entries);
	}

	trace_write_pushbuffers(c, num_entries);

	c->gpfifo.put = (c->gpfifo.put + num_entries) &
		(c->gpfifo.entry_num - 1U);

	return 0;
}

static int nvgpu_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvgpu_gpfifo_entry *gpfifo,
				struct nvgpu_gpfifo_userdata userdata,
				u32 num_entries,
				u32 flags,
				struct nvgpu_channel_fence *fence,
				struct gk20a_fence **fence_out,
				struct fifo_profile_gk20a *profile)
{
	struct gk20a *g = c->g;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct priv_cmd_entry *incr_cmd = NULL;
	struct gk20a_fence *post_fence = NULL;
	struct channel_gk20a_job *job = NULL;
	/* we might need two extra gpfifo entries - one for pre fence
	 * and one for post fence. */
	const u32 extra_entries = 2U;
	bool skip_buffer_refcounting = (flags &
			NVGPU_SUBMIT_FLAGS_SKIP_BUFFER_REFCOUNTING);
	int err = 0;
	bool need_job_tracking;
	bool need_deferred_cleanup = false;

	if (nvgpu_is_enabled(g, NVGPU_DRIVER_IS_DYING)) {
		return -ENODEV;
	}

	if (gk20a_channel_check_timedout(c)) {
		return -ETIMEDOUT;
	}

	if (c->usermode_submit_enabled) {
		return -EINVAL;
	}

	if (!nvgpu_mem_is_valid(&c->gpfifo.mem)) {
		return -ENOMEM;
	}

	/* fifo not large enough for request. Return error immediately.
	 * Kernel can insert gpfifo entries before and after user gpfifos.
	 * So, add extra_entries in user request. Also, HW with fifo size N
	 * can accept only N-1 entreis and so the below condition */
	if (c->gpfifo.entry_num - 1U < num_entries + extra_entries) {
		nvgpu_err(g, "not enough gpfifo space allocated");
		return -ENOMEM;
	}

	if ((flags & (NVGPU_SUBMIT_FLAGS_FENCE_WAIT |
		      NVGPU_SUBMIT_FLAGS_FENCE_GET)) &&
	    !fence) {
		return -EINVAL;
	}

	/* an address space needs to have been bound at this point. */
	if (!gk20a_channel_as_bound(c)) {
		nvgpu_err(g,
			    "not bound to an address space at time of gpfifo"
			    " submission.");
		return -EINVAL;
	}

	gk20a_fifo_profile_snapshot(profile, PROFILE_ENTRY);

	/* update debug settings */
	nvgpu_ltc_sync_enabled(g);

	nvgpu_log_info(g, "channel %d", c->chid);

	/*
	 * Job tracking is necessary for any of the following conditions:
	 *  - pre- or post-fence functionality
	 *  - channel wdt
	 *  - GPU rail-gating with non-deterministic channels
	 *  - VPR resize enabled with non-deterministic channels
	 *  - buffer refcounting
	 *
	 * If none of the conditions are met, then job tracking is not
	 * required and a fast submit can be done (ie. only need to write
	 * out userspace GPFIFO entries and update GP_PUT).
	 */
	need_job_tracking = (flags & NVGPU_SUBMIT_FLAGS_FENCE_WAIT) ||
			(flags & NVGPU_SUBMIT_FLAGS_FENCE_GET) ||
			c->timeout.enabled ||
			((nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE) ||
				nvgpu_is_vpr_resize_enabled()) &&
				!c->deterministic) ||
			!skip_buffer_refcounting;

	if (need_job_tracking) {
		bool need_sync_framework = false;

		/*
		 * If the channel is to have deterministic latency and
		 * job tracking is required, the channel must have
		 * pre-allocated resources. Otherwise, we fail the submit here
		 */
		if (c->deterministic && !channel_gk20a_is_prealloc_enabled(c)) {
			return -EINVAL;
		}

		need_sync_framework =
			nvgpu_channel_sync_needs_os_fence_framework(g) ||
			(flags & NVGPU_SUBMIT_FLAGS_SYNC_FENCE &&
			 flags & NVGPU_SUBMIT_FLAGS_FENCE_GET);

		/*
		 * Deferred clean-up is necessary for any of the following
		 * conditions:
		 * - channel's deterministic flag is not set
		 * - dependency on sync framework, which could make the
		 *   behavior of the clean-up operation non-deterministic
		 *   (should not be performed in the submit path)
		 * - channel wdt
		 * - GPU rail-gating with non-deterministic channels
		 * - buffer refcounting
		 *
		 * If none of the conditions are met, then deferred clean-up
		 * is not required, and we clean-up one job-tracking
		 * resource in the submit path.
		 */
		need_deferred_cleanup = !c->deterministic ||
					need_sync_framework ||
					c->timeout.enabled ||
					(nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE) &&
					 !c->deterministic) ||
					!skip_buffer_refcounting;

		/*
		 * For deterministic channels, we don't allow deferred clean_up
		 * processing to occur. In cases we hit this, we fail the submit
		 */
		if (c->deterministic && need_deferred_cleanup) {
			return -EINVAL;
		}

		if (!c->deterministic) {
			/*
			 * Get a power ref unless this is a deterministic
			 * channel that holds them during the channel lifetime.
			 * This one is released by gk20a_channel_clean_up_jobs,
			 * via syncpt or sema interrupt, whichever is used.
			 */
			err = gk20a_busy(g);
			if (err) {
				nvgpu_err(g,
					"failed to host gk20a to submit gpfifo");
				nvgpu_print_current(g, NULL, NVGPU_ERROR);
				return err;
			}
		}

		if (!need_deferred_cleanup) {
			/* clean up a single job */
			gk20a_channel_clean_up_jobs(c, false);
		}
	}


	/* Grab access to HW to deal with do_idle */
	if (c->deterministic) {
		nvgpu_rwsem_down_read(&g->deterministic_busy);
	}

	if (c->deterministic && c->deterministic_railgate_allowed) {
		/*
		 * Nope - this channel has dropped its own power ref. As
		 * deterministic submits don't hold power on per each submitted
		 * job like normal ones do, the GPU might railgate any time now
		 * and thus submit is disallowed.
		 */
		err = -EINVAL;
		goto clean_up;
	}

	trace_gk20a_channel_submit_gpfifo(g->name,
					  c->chid,
					  num_entries,
					  flags,
					  fence ? fence->id : 0,
					  fence ? fence->value : 0);

	nvgpu_log_info(g, "pre-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	/*
	 * Make sure we have enough space for gpfifo entries. Check cached
	 * values first and then read from HW. If no space, return EAGAIN
	 * and let userpace decide to re-try request or not.
	 */
	if (nvgpu_gp_free_count(c) < num_entries + extra_entries) {
		if (nvgpu_get_gp_free_count(c) < num_entries + extra_entries) {
			err = -EAGAIN;
			goto clean_up;
		}
	}

	if (gk20a_channel_check_timedout(c)) {
		err = -ETIMEDOUT;
		goto clean_up;
	}

	if (need_job_tracking) {
		err = channel_gk20a_alloc_job(c, &job);
		if (err) {
			goto clean_up;
		}

		err = nvgpu_submit_prepare_syncs(c, fence, job,
						 &wait_cmd, &incr_cmd,
						 &post_fence,
						 need_deferred_cleanup,
						 flags);
		if (err) {
			goto clean_up_job;
		}
	}

	gk20a_fifo_profile_snapshot(profile, PROFILE_JOB_TRACKING);

	if (wait_cmd) {
		nvgpu_submit_append_priv_cmdbuf(c, wait_cmd);
	}

	err = nvgpu_submit_append_gpfifo(c, gpfifo, userdata,
			num_entries);
	if (err) {
		goto clean_up_job;
	}

	/*
	 * And here's where we add the incr_cmd we generated earlier. It should
	 * always run!
	 */
	if (incr_cmd) {
		nvgpu_submit_append_priv_cmdbuf(c, incr_cmd);
	}

	if (fence_out) {
		*fence_out = gk20a_fence_get(post_fence);
	}

	if (need_job_tracking) {
		/* TODO! Check for errors... */
		gk20a_channel_add_job(c, job, skip_buffer_refcounting);
	}
	gk20a_fifo_profile_snapshot(profile, PROFILE_APPEND);

	g->ops.fifo.userd_gp_put(g, c);

	/* No hw access beyond this point */
	if (c->deterministic) {
		nvgpu_rwsem_up_read(&g->deterministic_busy);
	}

	trace_gk20a_channel_submitted_gpfifo(g->name,
				c->chid,
				num_entries,
				flags,
				post_fence ? post_fence->syncpt_id : 0,
				post_fence ? post_fence->syncpt_value : 0);

	nvgpu_log_info(g, "post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	gk20a_fifo_profile_snapshot(profile, PROFILE_END);

	nvgpu_log_fn(g, "done");
	return err;

clean_up_job:
	channel_gk20a_free_job(c, job);
clean_up:
	nvgpu_log_fn(g, "fail");
	gk20a_fence_put(post_fence);
	if (c->deterministic) {
		nvgpu_rwsem_up_read(&g->deterministic_busy);
	} else if (need_deferred_cleanup) {
		gk20a_idle(g);
	}

	return err;
}

int nvgpu_submit_channel_gpfifo_user(struct channel_gk20a *c,
				struct nvgpu_gpfifo_userdata userdata,
				u32 num_entries,
				u32 flags,
				struct nvgpu_channel_fence *fence,
				struct gk20a_fence **fence_out,
				struct fifo_profile_gk20a *profile)
{
	return nvgpu_submit_channel_gpfifo(c, NULL, userdata, num_entries,
			flags, fence, fence_out, profile);
}

int nvgpu_submit_channel_gpfifo_kernel(struct channel_gk20a *c,
				struct nvgpu_gpfifo_entry *gpfifo,
				u32 num_entries,
				u32 flags,
				struct nvgpu_channel_fence *fence,
				struct gk20a_fence **fence_out)
{
	struct nvgpu_gpfifo_userdata userdata = { NULL, NULL };

	return nvgpu_submit_channel_gpfifo(c, gpfifo, userdata, num_entries,
			flags, fence, fence_out, NULL);
}
