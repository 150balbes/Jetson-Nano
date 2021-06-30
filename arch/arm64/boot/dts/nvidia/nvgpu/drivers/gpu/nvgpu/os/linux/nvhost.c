/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/nvhost.h>
#include <linux/nvhost_t194.h>
#include <uapi/linux/nvhost_ioctl.h>
#include <linux/of_platform.h>

#include <nvgpu/gk20a.h>
#include <nvgpu/nvhost.h>
#include <nvgpu/enabled.h>

#include "nvhost_priv.h"

#include "os_linux.h"
#include "module.h"

int nvgpu_get_nvhost_dev(struct gk20a *g)
{
	struct device_node *np = nvgpu_get_node(g);
	struct platform_device *host1x_pdev = NULL;
	const __be32 *host1x_ptr;

	host1x_ptr = of_get_property(np, "nvidia,host1x", NULL);
	if (host1x_ptr) {
		struct device_node *host1x_node =
			of_find_node_by_phandle(be32_to_cpup(host1x_ptr));

		host1x_pdev = of_find_device_by_node(host1x_node);
		if (!host1x_pdev) {
			nvgpu_warn(g, "host1x device not available");
			return -EPROBE_DEFER;
		}

	} else {
		if (nvgpu_has_syncpoints(g)) {
			nvgpu_warn(g, "host1x reference not found. assuming no syncpoints support");
			__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, false);
		}
		return 0;
	}

	g->nvhost_dev = nvgpu_kzalloc(g, sizeof(struct nvgpu_nvhost_dev));
	if (!g->nvhost_dev)
		return -ENOMEM;

	g->nvhost_dev->host1x_pdev = host1x_pdev;

	return 0;
}

void nvgpu_free_nvhost_dev(struct gk20a *g)
{
	nvgpu_kfree(g, g->nvhost_dev);
}

int nvgpu_nvhost_module_busy_ext(
	struct nvgpu_nvhost_dev *nvhost_dev)
{
	return nvhost_module_busy_ext(nvhost_dev->host1x_pdev);
}

void nvgpu_nvhost_module_idle_ext(
	struct nvgpu_nvhost_dev *nvhost_dev)
{
	nvhost_module_idle_ext(nvhost_dev->host1x_pdev);
}

void nvgpu_nvhost_debug_dump_device(
	struct nvgpu_nvhost_dev *nvhost_dev)
{
	nvhost_debug_dump_device(nvhost_dev->host1x_pdev);
}

const char *nvgpu_nvhost_syncpt_get_name(
	struct nvgpu_nvhost_dev *nvhost_dev, int id)
{
	return nvhost_syncpt_get_name(nvhost_dev->host1x_pdev, id);
}

bool nvgpu_nvhost_syncpt_is_valid_pt_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id)
{
	return nvhost_syncpt_is_valid_pt_ext(nvhost_dev->host1x_pdev, id);
}

int nvgpu_nvhost_syncpt_is_expired_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id, u32 thresh)
{
	return nvhost_syncpt_is_expired_ext(nvhost_dev->host1x_pdev,
			id, thresh);
}

u32 nvgpu_nvhost_syncpt_incr_max_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id, u32 incrs)
{
	return nvhost_syncpt_incr_max_ext(nvhost_dev->host1x_pdev, id, incrs);
}

int nvgpu_nvhost_intr_register_notifier(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id, u32 thresh,
	void (*callback)(void *, int), void *private_data)
{
	return nvhost_intr_register_notifier(nvhost_dev->host1x_pdev,
			id, thresh,
			callback, private_data);
}

void nvgpu_nvhost_syncpt_set_min_eq_max_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id)
{
	nvhost_syncpt_set_min_eq_max_ext(nvhost_dev->host1x_pdev, id);
}

void nvgpu_nvhost_syncpt_put_ref_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id)
{
	nvhost_syncpt_put_ref_ext(nvhost_dev->host1x_pdev, id);
}

u32 nvgpu_nvhost_get_syncpt_host_managed(
	struct nvgpu_nvhost_dev *nvhost_dev,
	u32 param, const char *syncpt_name)
{
	return nvhost_get_syncpt_host_managed(nvhost_dev->host1x_pdev,
			param, syncpt_name);
}

u32 nvgpu_nvhost_get_syncpt_client_managed(
	struct nvgpu_nvhost_dev *nvhost_dev,
	const char *syncpt_name)
{
	return nvhost_get_syncpt_client_managed(nvhost_dev->host1x_pdev,
			syncpt_name);
}

int nvgpu_nvhost_syncpt_wait_timeout_ext(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id,
	u32 thresh, u32 timeout, u32 *value, struct timespec *ts)
{
	return nvhost_syncpt_wait_timeout_ext(nvhost_dev->host1x_pdev,
		id, thresh, timeout, value, ts);
}

int nvgpu_nvhost_syncpt_read_ext_check(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id, u32 *val)
{
	return nvhost_syncpt_read_ext_check(nvhost_dev->host1x_pdev, id, val);
}

u32 nvgpu_nvhost_syncpt_read_maxval(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id)
{
	return nvhost_syncpt_read_maxval(nvhost_dev->host1x_pdev, id);
}

void nvgpu_nvhost_syncpt_set_safe_state(
	struct nvgpu_nvhost_dev *nvhost_dev, u32 id)
{
	u32 val;

	/*
	 * Add large number of increments to current value
	 * so that all waiters on this syncpoint are released
	 *
	 * We don't expect any case where more than 0x10000 increments
	 * are pending
	 */
	val = nvhost_syncpt_read_minval(nvhost_dev->host1x_pdev, id);
	val += 0x10000;

	nvhost_syncpt_set_minval(nvhost_dev->host1x_pdev, id, val);
	nvhost_syncpt_set_maxval(nvhost_dev->host1x_pdev, id, val);
}

int nvgpu_nvhost_create_symlink(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	int err = 0;

	if (g->nvhost_dev &&
			(dev->parent != &g->nvhost_dev->host1x_pdev->dev)) {
		err = sysfs_create_link(&g->nvhost_dev->host1x_pdev->dev.kobj,
				&dev->kobj,
				dev_name(dev));
	}

	return err;
}

void nvgpu_nvhost_remove_symlink(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);

	if (g->nvhost_dev &&
			(dev->parent != &g->nvhost_dev->host1x_pdev->dev)) {
		sysfs_remove_link(&g->nvhost_dev->host1x_pdev->dev.kobj,
				  dev_name(dev));
	}
}

#ifdef CONFIG_SYNC
u32 nvgpu_nvhost_sync_pt_id(struct sync_pt *pt)
{
	return nvhost_sync_pt_id(pt);
}

u32 nvgpu_nvhost_sync_pt_thresh(struct sync_pt *pt)
{
	return nvhost_sync_pt_thresh(pt);
}

struct sync_fence *nvgpu_nvhost_sync_fdget(int fd)
{
	return nvhost_sync_fdget(fd);
}

int nvgpu_nvhost_sync_num_pts(struct sync_fence *fence)
{
	return nvhost_sync_num_pts(fence);
}

struct sync_fence *nvgpu_nvhost_sync_create_fence(
	struct nvgpu_nvhost_dev *nvhost_dev,
	u32 id, u32 thresh, const char *name)
{
	struct nvhost_ctrl_sync_fence_info pt = {
		.id = id,
		.thresh = thresh,
	};

	return nvhost_sync_create_fence(nvhost_dev->host1x_pdev, &pt, 1, name);
}
#endif /* CONFIG_SYNC */

#ifdef CONFIG_TEGRA_T19X_GRHOST
int nvgpu_nvhost_syncpt_unit_interface_get_aperture(
		struct nvgpu_nvhost_dev *nvhost_dev,
		u64 *base, size_t *size)
{
	return nvhost_syncpt_unit_interface_get_aperture(
		nvhost_dev->host1x_pdev, (phys_addr_t *)base, size);
}

u32 nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(u32 syncpt_id)
{
	return nvhost_syncpt_unit_interface_get_byte_offset(syncpt_id);
}

int nvgpu_nvhost_syncpt_init(struct gk20a *g)
{
	int err = 0;

	if (!nvgpu_has_syncpoints(g))
		return -ENOSYS;

	err = nvgpu_get_nvhost_dev(g);
	if (err) {
		nvgpu_err(g, "host1x device not available");
		__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, false);
		return -ENOSYS;
	}

	err = nvgpu_nvhost_syncpt_unit_interface_get_aperture(
			g->nvhost_dev,
			&g->syncpt_unit_base,
			&g->syncpt_unit_size);
	if (err) {
		nvgpu_err(g, "Failed to get syncpt interface");
		__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, false);
		return -ENOSYS;
	}

	g->syncpt_size =
			nvgpu_nvhost_syncpt_unit_interface_get_byte_offset(1);
	nvgpu_info(g, "syncpt_unit_base %llx syncpt_unit_size %zx size %x\n",
			g->syncpt_unit_base, g->syncpt_unit_size,
			g->syncpt_size);

	return 0;
}
#endif
