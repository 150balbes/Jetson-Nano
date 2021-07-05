/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/err.h>
#include <nvgpu/errno.h>

#include <nvgpu/types.h>
#include <nvgpu/os_fence.h>
#include <nvgpu/linux/os_fence_android.h>
#include <nvgpu/nvhost.h>
#include <nvgpu/atomic.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/channel_sync.h>

#include "gk20a/mm_gk20a.h"

#include "../drivers/staging/android/sync.h"

int nvgpu_os_fence_syncpt_wait_gen_cmd(struct nvgpu_os_fence *s,
	struct priv_cmd_entry *wait_cmd,
	struct channel_gk20a *c,
	int max_wait_cmds)
{
	int err;
	int wait_cmd_size;
	int num_wait_cmds;
	int i;
	u32 wait_id;
	struct sync_pt *pt;

	struct sync_fence *sync_fence = (struct sync_fence *)s->priv;

	if (max_wait_cmds && sync_fence->num_fences > max_wait_cmds)
		return -EINVAL;

	/* validate syncpt ids */
	for (i = 0; i < sync_fence->num_fences; i++) {
		pt = sync_pt_from_fence(sync_fence->cbs[i].sync_pt);
		wait_id = nvgpu_nvhost_sync_pt_id(pt);
		if (!wait_id || !nvgpu_nvhost_syncpt_is_valid_pt_ext(
					c->g->nvhost_dev, wait_id)) {
			return -EINVAL;
		}
	}

	num_wait_cmds = nvgpu_nvhost_sync_num_pts(sync_fence);
	if (num_wait_cmds == 0)
		return 0;

	wait_cmd_size = c->g->ops.fifo.get_syncpt_wait_cmd_size();
	err = gk20a_channel_alloc_priv_cmdbuf(c,
		wait_cmd_size * num_wait_cmds, wait_cmd);
	if (err) {
		return err;
	}

	for (i = 0; i < sync_fence->num_fences; i++) {
		struct sync_pt *pt = sync_pt_from_fence(
			sync_fence->cbs[i].sync_pt);
		u32 wait_id = nvgpu_nvhost_sync_pt_id(pt);
		u32 wait_value = nvgpu_nvhost_sync_pt_thresh(pt);

		err = channel_sync_syncpt_gen_wait_cmd(c, wait_id, wait_value,
			wait_cmd, wait_cmd_size, i, true);
	}

	WARN_ON(i != num_wait_cmds);

	return 0;
}

static const struct nvgpu_os_fence_ops syncpt_ops = {
	.program_waits = nvgpu_os_fence_syncpt_wait_gen_cmd,
	.drop_ref = nvgpu_os_fence_android_drop_ref,
	.install_fence = nvgpu_os_fence_android_install_fd,
};

int nvgpu_os_fence_syncpt_create(
	struct nvgpu_os_fence *fence_out, struct channel_gk20a *c,
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id, u32 thresh)
{
	struct sync_fence *fence = nvgpu_nvhost_sync_create_fence(
		nvhost_dev, id, thresh, "fence");

	if (IS_ERR(fence)) {
		nvgpu_err(c->g, "error %d during construction of fence.", (int)PTR_ERR(fence));
		return PTR_ERR(fence);
	}

	nvgpu_os_fence_init(fence_out, c->g, &syncpt_ops, fence);

	return 0;
}

int nvgpu_os_fence_syncpt_fdget(struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c, int fd)
{
	struct sync_fence *fence = nvgpu_nvhost_sync_fdget(fd);

	if (fence == NULL) {
		return -ENOMEM;
	}

	nvgpu_os_fence_init(fence_out, c->g, &syncpt_ops, fence);

	return 0;
}
