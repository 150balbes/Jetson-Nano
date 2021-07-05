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

#include <nvgpu/errno.h>

#include <nvgpu/types.h>
#include <nvgpu/os_fence.h>
#include <nvgpu/linux/os_fence_android.h>
#include <nvgpu/semaphore.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/channel_sync.h>

#include "gk20a/mm_gk20a.h"

#include "sync_sema_android.h"

#include "../drivers/staging/android/sync.h"

int nvgpu_os_fence_sema_wait_gen_cmd(struct nvgpu_os_fence *s,
	struct priv_cmd_entry *wait_cmd,
	struct channel_gk20a *c,
	int max_wait_cmds)
{
	int err;
	int wait_cmd_size;
	int num_wait_cmds;
	int i;
	struct nvgpu_semaphore *sema;
	struct sync_fence *sync_fence = nvgpu_get_sync_fence(s);

	wait_cmd_size = c->g->ops.fifo.get_sema_wait_cmd_size();

	num_wait_cmds = sync_fence->num_fences;
	if (num_wait_cmds == 0)
		return 0;

	if (max_wait_cmds && num_wait_cmds > max_wait_cmds)
		return -EINVAL;

	err = gk20a_channel_alloc_priv_cmdbuf(c,
		wait_cmd_size * num_wait_cmds,
		wait_cmd);
	if (err) {
		return err;
	}

	for (i = 0; i < num_wait_cmds; i++) {
		struct sync_pt *pt = sync_pt_from_fence(
			sync_fence->cbs[i].sync_pt);

		sema = gk20a_sync_pt_sema(pt);
		channel_sync_semaphore_gen_wait_cmd(c, sema, wait_cmd,
			wait_cmd_size, i);
	}

	return 0;
}

static const struct nvgpu_os_fence_ops sema_ops = {
	.program_waits = nvgpu_os_fence_sema_wait_gen_cmd,
	.drop_ref = nvgpu_os_fence_android_drop_ref,
	.install_fence = nvgpu_os_fence_android_install_fd,
};

int nvgpu_os_fence_sema_create(
	struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c,
	struct nvgpu_semaphore *sema)
{
	struct sync_fence *fence;

	fence = gk20a_sync_fence_create(c, sema, "f-gk20a-0x%04x",
			nvgpu_semaphore_gpu_ro_va(sema));

	if (!fence) {
		nvgpu_err(c->g, "error constructing new fence: f-gk20a-0x%04x",
			(u32)nvgpu_semaphore_gpu_ro_va(sema));

		return -ENOMEM;
	}

	nvgpu_os_fence_init(fence_out, c->g, &sema_ops, fence);

	return 0;
}

int nvgpu_os_fence_sema_fdget(struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c, int fd)
{
	struct sync_fence *fence = gk20a_sync_fence_fdget(fd);

	if (!fence)
		return -EINVAL;

	nvgpu_os_fence_init(fence_out, c->g, &sema_ops, fence);

	return 0;
}
