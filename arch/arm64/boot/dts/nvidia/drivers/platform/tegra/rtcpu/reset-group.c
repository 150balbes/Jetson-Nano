/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#include "reset-group.h"

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/reset.h>

struct camrtc_reset_group {
	struct device *device;
	const char *group_name;
	int nresets;
	struct reset_control *resets[];
};

static void camrtc_reset_group_release(struct device *dev, void *res)
{
	const struct camrtc_reset_group *grp = res;
	int i;

	for (i = 0; i < grp->nresets; i++) {
		if (grp->resets[i])
			reset_control_put(grp->resets[i]);
	}
}

struct camrtc_reset_group *camrtc_reset_group_get(
	struct device *dev,
	const char *group_name)
{
	struct camrtc_reset_group *grp;
	struct device_node *np;
	const char *group_property;
	size_t group_name_len;
	int index;
	int ret;

	if (!dev || !dev->of_node)
		return ERR_PTR(-EINVAL);

	np = dev->of_node;

	group_property = group_name ? group_name : "reset-names";
	group_name_len = group_name ? strlen(group_name) : 0;

	ret = of_property_count_strings(np, group_property);
	if (ret < 0)
		return ERR_PTR(-ENOENT);

	grp = devres_alloc(camrtc_reset_group_release,
			offsetof(struct camrtc_reset_group, resets[ret]) +
			group_name_len + 1,
			GFP_KERNEL);
	if (!grp)
		return ERR_PTR(-ENOMEM);

	grp->nresets = ret;
	grp->device = dev;
	grp->group_name = (char *)&grp->resets[grp->nresets];
	memcpy((char *)grp->group_name, group_name, group_name_len);

	for (index = 0; index < grp->nresets; index++) {
		char const *name;
		struct reset_control *reset;

		ret = of_property_read_string_index(np, group_property,
						index, &name);
		if (ret < 0)
			goto error;

		reset = of_reset_control_get(np, name);
		if (IS_ERR(reset)) {
			ret = PTR_ERR(reset);
			goto error;
		}

		grp->resets[index] = reset;
	}

	devres_add(dev, grp);
	return grp;

error:
	devres_free(grp);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(camrtc_reset_group_get);

static void camrtc_reset_group_error(
	const struct camrtc_reset_group *grp,
	char const *op,
	int index,
	int error)
{
	const char *name = "unnamed";

	of_property_read_string_index(grp->device->of_node,
				grp->group_name, index, &name);
	dev_WARN(grp->device, "%s reset %s (at %s[%d]): %d\n",
		op, name, grp->group_name, index, error);
}

void camrtc_reset_group_assert(const struct camrtc_reset_group *grp)
{
	int index, index0, err;

	if (IS_ERR_OR_NULL(grp))
		return;

	for (index = 1; index <= grp->nresets; index++) {
		index0 = grp->nresets - index;
		err = reset_control_assert(grp->resets[index0]);
		if (err < 0)
			camrtc_reset_group_error(grp, "assert", index0, err);
	}
}
EXPORT_SYMBOL_GPL(camrtc_reset_group_assert);

int camrtc_reset_group_deassert(const struct camrtc_reset_group *grp)
{
	int index, err;

	if (!grp)
		return 0;
	if (IS_ERR(grp))
		return -ENODEV;

	for (index = 0; index < grp->nresets; index++) {
		err = reset_control_deassert(grp->resets[index]);
		if (err < 0) {
			camrtc_reset_group_error(grp, "deassert", index, err);
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(camrtc_reset_group_deassert);
