/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef NVGPU_LINUX_CHANNEL_H
#define NVGPU_LINUX_CHANNEL_H

#include <linux/workqueue.h>
#include <linux/dma-buf.h>

#include <nvgpu/types.h>

struct channel_gk20a;
struct nvgpu_gpfifo;
struct nvgpu_submit_gpfifo_args;
struct nvgpu_channel_fence;
struct gk20a_fence;
struct fifo_profile_gk20a;
struct nvgpu_os_linux;

struct sync_fence;
struct sync_timeline;

struct nvgpu_channel_completion_cb {
	/*
	 * Signal channel owner via a callback, if set, in job cleanup with
	 * schedule_work. Means that something finished on the channel (perhaps
	 * more than one job).
	 */
	void (*fn)(struct channel_gk20a *, void *);
	void *user_data;
	/* Make access to the two above atomic */
	struct nvgpu_spinlock lock;
	/* Per-channel async work task, cannot reschedule itself */
	struct work_struct work;
};

struct nvgpu_error_notifier {
	struct dma_buf *dmabuf;
	void *vaddr;

	struct nvgpu_notification *notification;

	struct nvgpu_mutex mutex;
};

/*
 * This struct contains fence_related data.
 * e.g. sync_timeline for sync_fences.
 */
struct nvgpu_os_fence_framework {
	struct sync_timeline *timeline;
};

struct nvgpu_usermode_bufs_linux {
	/*
	 * Common low level info of these is stored in nvgpu_mems in
	 * channel_gk20a; these hold lifetimes for the actual dmabuf and its
	 * dma mapping.
	 */
	struct nvgpu_usermode_buf_linux {
		struct dma_buf *dmabuf;
		struct dma_buf_attachment *attachment;
		struct sg_table *sgt;
	} gpfifo, userd;
};

struct nvgpu_channel_linux {
	struct channel_gk20a *ch;

	struct nvgpu_os_fence_framework fence_framework;

	struct nvgpu_channel_completion_cb completion_cb;
	struct nvgpu_error_notifier error_notifier;

	struct dma_buf *cyclestate_buffer_handler;

	struct nvgpu_usermode_bufs_linux usermode;
};

u32 nvgpu_submit_gpfifo_user_flags_to_common_flags(u32 user_flags);
int nvgpu_init_channel_support_linux(struct nvgpu_os_linux *l);
void nvgpu_remove_channel_support_linux(struct nvgpu_os_linux *l);

struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data,
		int runlist_id,
		bool is_privileged_channel);

#endif
