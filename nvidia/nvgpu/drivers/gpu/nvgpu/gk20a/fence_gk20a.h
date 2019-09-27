/*
 * drivers/video/tegra/host/gk20a/fence_gk20a.h
 *
 * GK20A Fences
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
#ifndef NVGPU_GK20A_FENCE_GK20A_H
#define NVGPU_GK20A_FENCE_GK20A_H

#include <nvgpu/types.h>
#include <nvgpu/kref.h>
#include <nvgpu/os_fence.h>

struct platform_device;
struct nvgpu_semaphore;
struct channel_gk20a;
struct gk20a;
struct nvgpu_os_fence;

struct gk20a_fence_ops;

struct gk20a_fence {
	struct gk20a *g;

	/* Valid for all fence types: */
	bool valid;
	struct nvgpu_ref ref;
	const struct gk20a_fence_ops *ops;

	struct nvgpu_os_fence os_fence;

	/* Valid for fences created from semaphores: */
	struct nvgpu_semaphore *semaphore;
	struct nvgpu_cond *semaphore_wq;

	/* Valid for fences created from syncpoints: */
	struct nvgpu_nvhost_dev *nvhost_dev;
	u32 syncpt_id;
	u32 syncpt_value;

	/* Valid for fences part of a pre-allocated fence pool */
	struct nvgpu_allocator *allocator;
};

/* Fences can be created from semaphores or syncpoint (id, value) pairs */
int gk20a_fence_from_semaphore(
		struct gk20a_fence *fence_out,
		struct nvgpu_semaphore *semaphore,
		struct nvgpu_cond *semaphore_wq,
		struct nvgpu_os_fence os_fence);

int gk20a_fence_from_syncpt(
		struct gk20a_fence *fence_out,
		struct nvgpu_nvhost_dev *nvhost_dev,
		u32 id, u32 value,
		struct nvgpu_os_fence os_fence);

int gk20a_alloc_fence_pool(
		struct channel_gk20a *c,
		unsigned int count);

void gk20a_free_fence_pool(
		struct channel_gk20a *c);

struct gk20a_fence *gk20a_alloc_fence(
		struct channel_gk20a *c);

void gk20a_init_fence(struct gk20a_fence *f,
		const struct gk20a_fence_ops *ops,
		struct nvgpu_os_fence os_fence);

/* Fence operations */
void gk20a_fence_put(struct gk20a_fence *f);
struct gk20a_fence *gk20a_fence_get(struct gk20a_fence *f);
int gk20a_fence_wait(struct gk20a *g, struct gk20a_fence *f,
							unsigned long timeout);
bool gk20a_fence_is_expired(struct gk20a_fence *f);
bool gk20a_fence_is_valid(struct gk20a_fence *f);
int gk20a_fence_install_fd(struct gk20a_fence *f, int fd);

#endif /* NVGPU_GK20A_FENCE_GK20A_H */
