/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_NVHOST_H
#define NVGPU_NVHOST_H

#ifdef CONFIG_TEGRA_GK20A_NVHOST

#include <nvgpu/types.h>

struct nvgpu_nvhost_dev;
struct gk20a;
struct sync_pt;
struct sync_fence;
struct timespec;

int nvgpu_get_nvhost_dev(struct gk20a *g);
void nvgpu_free_nvhost_dev(struct gk20a *g);

int nvgpu_nvhost_module_busy_ext(struct nvgpu_nvhost_dev *nvhost_dev);
void nvgpu_nvhost_module_idle_ext(struct nvgpu_nvhost_dev *nvhost_dev);

void nvgpu_nvhost_debug_dump_device(struct nvgpu_nvhost_dev *nvhost_dev);

int nvgpu_nvhost_syncpt_is_expired_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh);
int nvgpu_nvhost_syncpt_wait_timeout_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh, u32 timeout, u32 *value, struct timespec *ts);

u32 nvgpu_nvhost_syncpt_incr_max_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 incrs);
void nvgpu_nvhost_syncpt_set_min_eq_max_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id);
int nvgpu_nvhost_syncpt_read_ext_check(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 *val);
u32 nvgpu_nvhost_syncpt_read_maxval(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id);
void nvgpu_nvhost_syncpt_set_safe_state(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id);

int nvgpu_nvhost_intr_register_notifier(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh, void (*callback)(void *, int), void *private_data);

const char *nvgpu_nvhost_syncpt_get_name(struct nvgpu_nvhost_dev *nvhost_dev,
	int id);
bool nvgpu_nvhost_syncpt_is_valid_pt_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id);
void nvgpu_nvhost_syncpt_put_ref_ext(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id);
u32 nvgpu_nvhost_get_syncpt_host_managed(struct nvgpu_nvhost_dev *nvhost_dev,
	u32 param,
	const char *syncpt_name);
u32 nvgpu_nvhost_get_syncpt_client_managed(struct nvgpu_nvhost_dev *nvhost_dev,
	const char *syncpt_name);

int nvgpu_nvhost_create_symlink(struct gk20a *g);
void nvgpu_nvhost_remove_symlink(struct gk20a *g);

#ifdef CONFIG_SYNC
u32 nvgpu_nvhost_sync_pt_id(struct sync_pt *pt);
u32 nvgpu_nvhost_sync_pt_thresh(struct sync_pt *pt);
int nvgpu_nvhost_sync_num_pts(struct sync_fence *fence);

struct sync_fence *nvgpu_nvhost_sync_fdget(int fd);
struct sync_fence *nvgpu_nvhost_sync_create_fence(
	struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh, const char *name);
#endif /* CONFIG_SYNC */

#ifdef CONFIG_TEGRA_T19X_GRHOST
int nvgpu_nvhost_syncpt_unit_interface_get_aperture(
		struct nvgpu_nvhost_dev *nvhost_dev,
		u64 *base, size_t *size);
u32 nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id);
int nvgpu_nvhost_syncpt_init(struct gk20a *g);
#else
static inline int nvgpu_nvhost_syncpt_unit_interface_get_aperture(
		struct nvgpu_nvhost_dev *nvhost_dev,
		u64 *base, size_t *size)
{
	return -EINVAL;
}
static inline u32 nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id)
{
	return 0;
}
static inline int nvgpu_nvhost_syncpt_init(struct gk20a *g)
{
	return 0;
}
#endif
#endif /* CONFIG_TEGRA_GK20A_NVHOST */
#endif /* NVGPU_NVHOST_H */
