/*
 *
 * Nvgpu Channel Synchronization Abstraction
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_CHANNEL_SYNC_H
#define NVGPU_CHANNEL_SYNC_H

#include <nvgpu/atomic.h>

struct nvgpu_channel_sync;
struct priv_cmd_entry;
struct channel_gk20a;
struct gk20a_fence;
struct gk20a;
struct nvgpu_semaphore;

struct nvgpu_channel_sync {
	nvgpu_atomic_t refcount;

	/* Generate a gpu wait cmdbuf from syncpoint.
	 * Returns a gpu cmdbuf that performs the wait when executed
	 */
	int (*wait_syncpt)(struct nvgpu_channel_sync *s, u32 id, u32 thresh,
			   struct priv_cmd_entry *entry);

	/* Generate a gpu wait cmdbuf from sync fd.
	 * Returns a gpu cmdbuf that performs the wait when executed
	 */
	int (*wait_fd)(struct nvgpu_channel_sync *s, int fd,
		       struct priv_cmd_entry *entry, int max_wait_cmds);

	/* Increment syncpoint/semaphore.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired().
	 */
	int (*incr)(struct nvgpu_channel_sync *s,
		    struct priv_cmd_entry *entry,
		    struct gk20a_fence *fence,
		    bool need_sync_fence,
		    bool register_irq);

	/* Increment syncpoint/semaphore, so that the returned fence represents
	 * work completion (may need wfi) and can be returned to user space.
	 * Returns
	 *  - a gpu cmdbuf that performs the increment when executed,
	 *  - a fence that can be passed to wait_cpu() and is_expired(),
	 *  - a gk20a_fence that signals when the incr has happened.
	 */
	int (*incr_user)(struct nvgpu_channel_sync *s,
			 int wait_fence_fd,
			 struct priv_cmd_entry *entry,
			 struct gk20a_fence *fence,
			 bool wfi,
			 bool need_sync_fence,
			 bool register_irq);

	/* Reset the channel syncpoint/semaphore. */
	void (*set_min_eq_max)(struct nvgpu_channel_sync *s);

	/*
	 * Set the channel syncpoint/semaphore to safe state
	 * This should be used to reset User managed syncpoint since we don't
	 * track threshold values for those syncpoints
	 */
	void (*set_safe_state)(struct nvgpu_channel_sync *s);

	/* Returns the sync point id or negative number if no syncpt*/
	int (*syncpt_id)(struct nvgpu_channel_sync *s);

	/* Returns the sync point address of sync point or 0 if not supported */
	u64 (*syncpt_address)(struct nvgpu_channel_sync *s);

	/* Free the resources allocated by nvgpu_channel_sync_create. */
	void (*destroy)(struct nvgpu_channel_sync *s);
};

void channel_sync_semaphore_gen_wait_cmd(struct channel_gk20a *c,
	struct nvgpu_semaphore *sema, struct priv_cmd_entry *wait_cmd,
	u32 wait_cmd_size, u32 pos);

int channel_sync_syncpt_gen_wait_cmd(struct channel_gk20a *c,
		u32 id, u32 thresh, struct priv_cmd_entry *wait_cmd,
		u32 wait_cmd_size, u32 pos, bool preallocated);

void nvgpu_channel_sync_destroy(struct nvgpu_channel_sync *sync,
	bool set_safe_state);
struct nvgpu_channel_sync *nvgpu_channel_sync_create(struct channel_gk20a *c,
	bool user_managed);
bool nvgpu_channel_sync_needs_os_fence_framework(struct gk20a *g);

#endif /* NVGPU_CHANNEL_SYNC_H */
