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

#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/iommu.h>
#include <linux/pci.h>

#include "of-tegra-smmu.h"

#include <dt-bindings/memory/tegra-swgroup.h>

struct tegra_iommu_group {
	int group_id;
	struct iommu_group *group;
	struct list_head list;
};

static LIST_HEAD(tegra_iommu_groups);

static struct tegra_iommu_group *tegra_create_iommu_group(struct device *dev,
						int group_id)
{
	struct tegra_iommu_group *group = NULL;

	group = kcalloc(1, sizeof(struct tegra_iommu_group),
				GFP_KERNEL);
	if (!group)
		return NULL;

	if (dev_is_pci(dev))
		group->group = pci_device_group(dev);
	else
		group->group = generic_device_group(dev);

	group->group_id = group_id;
	list_add_tail(&group->list, &tegra_iommu_groups);

	return group;

}

void tegra_smmu_remove_iommu_groups(void)
{
	struct tegra_iommu_group *group;
	struct tegra_iommu_group *tmp;
	list_for_each_entry_safe(group, tmp, &tegra_iommu_groups, list) {
		list_del(&group->list);
		kfree(group);
	}
}

struct iommu_group *tegra_smmu_of_get_group(struct device *dev)
{
	u32 group_id;
	int ret;
	struct tegra_iommu_group *group;

	ret = of_property_read_u32_index(dev->of_node, "iommu-group-id", 0,
						&group_id);
	if (ret)
		return NULL;

	list_for_each_entry(group, &tegra_iommu_groups, list) {
		if (group->group_id == group_id) {
			dev_info(dev, "Adding to tegra-iommu group %d\n",
					group_id);
			return group->group;
		}
	}

	dev_info(dev, "Creating tegra-iommu group %d\n", group_id);
	group = tegra_create_iommu_group(dev, group_id);
	if (!group)
		return NULL;

	return group->group;
}
