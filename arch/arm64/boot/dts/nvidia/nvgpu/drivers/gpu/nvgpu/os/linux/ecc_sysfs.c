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

#include <nvgpu/ecc.h>
#include <nvgpu/gk20a.h>

#include "os_linux.h"

int nvgpu_ecc_sysfs_init(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	struct nvgpu_ecc *ecc = &g->ecc;
	struct dev_ext_attribute *attr;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct nvgpu_ecc_stat *stat;
	int i = 0, err;

	attr = nvgpu_kzalloc(g, sizeof(*attr) * ecc->stats_count);
	if (!attr)
		return -ENOMEM;

	nvgpu_list_for_each_entry(stat,
			&ecc->stats_list, nvgpu_ecc_stat, node) {
		if (i >= ecc->stats_count) {
			err = -EINVAL;
			nvgpu_err(g, "stats_list longer than stats_count %d",
					ecc->stats_count);
			break;
		}
		sysfs_attr_init(&attr[i].attr.attr);
		attr[i].attr.attr.name = stat->name;
		attr[i].attr.attr.mode = VERIFY_OCTAL_PERMISSIONS(S_IRUGO);
		attr[i].var =  &stat->counter;
		attr[i].attr.show = device_show_int;
		err = device_create_file(dev, &attr[i].attr);
		if (err) {
			nvgpu_err(g, "sysfs node create failed for %s\n",
					stat->name);
			break;
		}
		i++;
	}

	if (err) {
		while (i-- > 0)
			device_remove_file(dev, &attr[i].attr);
		nvgpu_kfree(g, attr);
		return err;
	}

	l->ecc_attrs = attr;

	return 0;
}

void nvgpu_ecc_sysfs_remove(struct gk20a *g)
{
	struct device *dev = dev_from_gk20a(g);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct nvgpu_ecc *ecc = &g->ecc;
	int i;

	for (i = 0; i < ecc->stats_count; i++)
		device_remove_file(dev, &l->ecc_attrs[i].attr);
	nvgpu_kfree(g, l->ecc_attrs);
	l->ecc_attrs = NULL;
}
