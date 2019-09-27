/*
 * nvgpu os fence
 *
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

#ifndef NVGPU_OS_FENCE_H
#define NVGPU_OS_FENCE_H

#include <nvgpu/errno.h>

struct nvgpu_semaphore;
struct channel_gk20a;
struct priv_cmd_entry;
struct nvgpu_nvhost_dev;

/*
 * struct nvgpu_os_fence adds an abstraction to the earlier Android Sync
 * Framework, specifically the sync-fence mechanism and the newer DMA sync
 * APIs from linux-4.9. This abstraction provides the high-level definition
 * as well as APIs that can be used by other OSes in future to have their own
 * alternatives for the sync-framework.
 */
struct nvgpu_os_fence;

/*
 * struct nvgpu_os_fence depends on the following ops structure
 */
struct nvgpu_os_fence_ops {
	/*
	 * This API is used to iterate through multiple fence points within the
	 * fence and program the pushbuffer method for wait command.
	 */
	int (*program_waits)(struct nvgpu_os_fence *s,
		struct priv_cmd_entry *wait_cmd,
		struct channel_gk20a *c,
		int max_wait_cmds);

	/*
	 * This should be the last operation on the OS fence. The
	 * OS fence acts as a place-holder for the underlying fence
	 * implementation e.g. sync_fences. For each construct/fdget call
	 * there needs to be a drop_ref call. This reduces a reference count
	 * for the underlying sync_fence.
	 */
	void (*drop_ref)(struct nvgpu_os_fence *s);

	/*
	 * Used to install the fd in the corresponding OS. The underlying
	 * implementation varies from OS to OS.
	 */
	void (*install_fence)(struct nvgpu_os_fence *s, int fd);
};

/*
 * The priv structure here is used to contain the struct sync_fence
 * for LINUX_VERSION <= 4.9 and dma_fence for LINUX_VERSION > 4.9
 */
struct nvgpu_os_fence {
	void *priv;
	struct gk20a *g;
	const struct nvgpu_os_fence_ops *ops;
};

/*
 * This API is used to validate the nvgpu_os_fence
 */
static inline int nvgpu_os_fence_is_initialized(struct nvgpu_os_fence *fence)
{
	return (fence->ops != NULL);
}

#ifdef CONFIG_SYNC

int nvgpu_os_fence_sema_create(
	struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c,
	struct nvgpu_semaphore *sema);

int nvgpu_os_fence_fdget(
	struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c, int fd);

#else

static inline int nvgpu_os_fence_sema_create(
	struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c,
	struct nvgpu_semaphore *sema)
{
	return -ENOSYS;
}
static inline int nvgpu_os_fence_fdget(
	struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c, int fd)
{
	return -ENOSYS;
}

#endif /* CONFIG_SYNC */

#if defined(CONFIG_TEGRA_GK20A_NVHOST) && defined(CONFIG_SYNC)

int nvgpu_os_fence_syncpt_create(struct nvgpu_os_fence *fence_out,
	struct channel_gk20a *c, struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh);

#else

static inline int nvgpu_os_fence_syncpt_create(
	struct nvgpu_os_fence *fence_out, struct channel_gk20a *c,
	struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh)
{
	return -ENOSYS;
}

#endif /* CONFIG_TEGRA_GK20A_NVHOST && CONFIG_SYNC */

#endif /* NVGPU_OS_FENCE_H */
