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

#include <linux/device.h>
#include <nvgpu/vgpu/vgpu.h>

#include "os/linux/platform_gk20a.h"
#include "os/linux/os_linux.h"
#include "vgpu/ecc_vgpu.h"

static ssize_t vgpu_load_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_gpu_load_params *p = &msg.params.gpu_load;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_GET_GPU_LOAD;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err)
		return err;

	return snprintf(buf, PAGE_SIZE, "%u\n", p->load);
}
static DEVICE_ATTR(load, S_IRUGO, vgpu_load_show, NULL);

static ssize_t vgpu_ecc_stat_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_ecc_counter_params *p = &msg.params.ecc_counter;
	struct dev_ext_attribute *ext_attr = container_of(attr,
			struct dev_ext_attribute, attr);
	struct vgpu_ecc_stat *ecc_stat = ext_attr->var;
	int err;

	p->ecc_id = ecc_stat->ecc_id;

	msg.cmd = TEGRA_VGPU_CMD_GET_ECC_COUNTER_VALUE;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (unlikely(err)) {
		nvgpu_err(g, "ecc: cannot get ECC counter value: %d", err);
		return err;
	}

	return snprintf(buf, PAGE_SIZE, "%u\n", p->value);
}

static int vgpu_create_ecc_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	struct vgpu_ecc_stat *stats;
	struct dev_ext_attribute *attrs;
	int err, i, count;

	err = vgpu_ecc_get_info(g);
	if (unlikely(err)) {
		nvgpu_err(g, "ecc: cannot get ECC info: %d", err);
		return err;
	}

	stats = priv->ecc_stats;
	count = priv->ecc_stats_count;

	attrs = nvgpu_kzalloc(g, count * sizeof(*attrs));
	if (unlikely(!attrs)) {
		nvgpu_err(g, "ecc: no memory");
		vgpu_ecc_remove_info(g);
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		sysfs_attr_init(&attrs[i].attr.attr);
		attrs[i].attr.attr.name = stats[i].name;
		attrs[i].attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO);
		attrs[i].attr.show = vgpu_ecc_stat_show;
		attrs[i].attr.store = NULL;
		attrs[i].var = &stats[i];

		err = device_create_file(dev, &attrs[i].attr);
		if (unlikely(err)) {
			nvgpu_warn(g, "ecc: cannot create file \"%s\": %d",
				   stats[i].name, err);
		}
	}

	l->ecc_attrs = attrs;
	return 0;
}

static void vgpu_remove_ecc_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int i;

	if (l->ecc_attrs) {
		for (i = 0; i < priv->ecc_stats_count; i++)
			device_remove_file(dev, &l->ecc_attrs[i].attr);

		nvgpu_kfree(g, l->ecc_attrs);
		l->ecc_attrs = NULL;
	}

	vgpu_ecc_remove_info(g);
}

void vgpu_create_sysfs(struct device *dev)
{
	if (device_create_file(dev, &dev_attr_load))
		dev_err(dev, "Failed to create vgpu sysfs attributes!\n");

	vgpu_create_ecc_sysfs(dev);
}

void vgpu_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_load);
	vgpu_remove_ecc_sysfs(dev);
}
