/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/tegra_prod.h>

#define PROD_TUPLE_NUM (sizeof(struct prod_tuple)/sizeof(u32))

/* tegra_prod: Tegra Prod list for the given submodule
 * @n_prod_cells: Number of prod setting cells.
 */
struct tegra_prod {
	struct tegra_prod_config *prod_config;
	int num; /* number of tegra_prod*/
	int n_prod_cells;
};

struct prod_tuple {
	u32 index; /* Address base index */
	u32 addr;  /* offset address*/
	u32 mask;  /* mask */
	u32 val;   /* value */
};

struct tegra_prod_config {
	const char *name;
	struct prod_tuple *prod_tuple;
	int count; /* number of prod_tuple*/
	bool boot_init;
};

static int tegra_prod_get_child_tupple_count(struct device *dev,
					     struct device_node *np,
		int n_tupple)
{
	struct device_node *child;
	int count;
	int total_tupple = 0;

	count = of_property_count_u32_elems(np, "prod");
	if (count > 0) {
		if ((count < n_tupple) || (count % n_tupple != 0)) {
			dev_err(dev, "Node %s has invalid entries\n", np->name);
			return -EINVAL;
		}
		total_tupple = count / n_tupple;
	}

	for_each_available_child_of_node(np, child) {
		count = tegra_prod_get_child_tupple_count(dev, child, n_tupple);
		if (count < 0)
			return count;

		total_tupple += count;
	}

	return total_tupple;
}

static int tegra_prod_read_prod_data(struct device *dev,
				     struct device_node *np,
				     struct prod_tuple *p_tuple,
				     int n_tupple)
{
	u32 pval;
	int count;
	int t_count;
	int cnt;
	int index;
	int ret;

	count = of_property_count_u32_elems(np, "prod");
	if (count <= 0) {
		dev_dbg(dev, "Node %s: prod prop not found\n", np->name);
		return 0;
	}

	t_count = count / n_tupple;
	for (cnt = 0; cnt < t_count; cnt++, p_tuple++) {
		index = cnt * n_tupple;

		if (n_tupple == 4) {
			ret = of_property_read_u32_index(np, "prod", index,
						&pval);
			if (ret) {
				dev_err(dev, "Failed to parse prod of node %s\n",
					np->name);
				return -EINVAL;
			}

			p_tuple->index = pval;
			index++;
		} else {
			p_tuple->index = 0;
		}

		ret = of_property_read_u32_index(np, "prod", index, &pval);
		if (ret) {
			dev_err(dev, "Failed to parse address of node %s\n",
				np->name);
			return -EINVAL;
		}
		p_tuple->addr = pval;
		index++;

		ret = of_property_read_u32_index(np, "prod", index, &pval);
		if (ret) {
			dev_err(dev, "Failed to parse mask of node %s\n",
				np->name);
			return -EINVAL;
		}
		p_tuple->mask = pval;
		index++;

		ret = of_property_read_u32_index(np, "prod", index, &pval);
		if (ret) {
			dev_err(dev, "Failed to parse value of node %s\n",
				np->name);
			return -EINVAL;
		}
		p_tuple->val = pval;
	}

	return t_count;
}

static int tegra_prod_read_node_tupple(struct device *dev,
				       struct device_node *np,
				       struct prod_tuple *p_tuple,
				       int n_tupple)
{
	int ret = 0;
	int sindex;
	struct device_node *child;

	ret = tegra_prod_read_prod_data(dev, np, p_tuple, n_tupple);
	if (ret < 0)
		return -EINVAL;

	sindex = ret;
	p_tuple += ret;

	for_each_available_child_of_node(np, child) {
		ret = tegra_prod_read_node_tupple(dev, child,
						  p_tuple, n_tupple);
		if (ret < 0)
			return -EINVAL;
		sindex += ret;
		p_tuple += ret;
	}

	return sindex;
}

/* Process the tupples and optimise for the register configuration for
 * Same location.
 */
static void tegra_prod_optimise_tupple(struct prod_tuple *p_tuple,
				       int n_tupple)
{
	struct prod_tuple *ti, *tj;
	u32 mask;
	int i, j;

	for (i = 0; i < n_tupple; ++i) {
		ti = p_tuple + i;
		for (j = i + 1; j < n_tupple; ++j) {
			tj = p_tuple + j;
			if (ti->index != tj->index)
				continue;

			if (ti->addr != tj->addr)
				continue;

			mask = ti->mask & tj->mask;
			if (!mask)
				continue;

			ti->val &= ~mask;
			ti->mask &= ~mask;
		}
	}
}

/**
 * tegra_prod_parse_dt - Read the prod setting form Device tree.
 * @np:			device node from which the property value is to be read.
 * @np_prod:		Prod setting node.
 * @tegra_prod:	The list of tegra prods.
 *
 * Read the prod setting form DT according the prod name in tegra prod list.
 * prod tuple will be allocated dynamically according to the tuple number of
 * each prod in DT.
 *
 * Returns 0 on success.
 */

static int tegra_prod_parse_dt(struct device *dev,
			       const struct device_node *np,
		const struct device_node *np_prod,
		struct tegra_prod *tegra_prod)
{
	struct device_node *child;
	struct tegra_prod_config *t_prod;
	struct prod_tuple *p_tuple;
	int n_child;
	int n_tupple = 3;
	int ret;
	int count;
	u32 pval;
	bool mask_opt;

	if (!tegra_prod || !tegra_prod->prod_config) {
		dev_err(dev, "Node %s: Invalid tegra prods list.\n", np->name);
		return -EINVAL;
	};

	mask_opt = of_property_read_bool(np_prod, "enable-mask-optimisation");

	ret = of_property_read_u32(np_prod, "#prod-cells", &pval);
	if (!ret)
		n_tupple = pval;
	if ((n_tupple != 3) && (n_tupple != 4)) {
		dev_err(dev, "Node %s: Prod cells not supported\n", np->name);
		return -EINVAL;
	}
	tegra_prod->n_prod_cells = n_tupple;

	n_child = 0;
	for_each_available_child_of_node(np_prod, child) {
		t_prod = &tegra_prod->prod_config[n_child];
		t_prod->name = child->name;

		count = tegra_prod_get_child_tupple_count(dev, child, n_tupple);
		if (count < 0) {
			dev_err(dev, "Node %s: Child has not proper setting\n",
				child->name);
			return -EINVAL;
		}

		if (!count) {
			dev_err(dev, "Node %s: prod prop not found\n",
				child->name);
			return -EINVAL;
		}

		t_prod->count = count;

		t_prod->prod_tuple = devm_kcalloc(dev, t_prod->count,
						  sizeof(*p_tuple), GFP_KERNEL);
		if (!t_prod->prod_tuple)
			return -ENOMEM;

		t_prod->boot_init = of_property_read_bool(child,
						"nvidia,prod-boot-init");

		ret = tegra_prod_read_node_tupple(dev, child,
						  t_prod->prod_tuple, n_tupple);
		if (ret < 0) {
			dev_err(dev, "Node %s: Reading prod setting failed: %d\n",
				child->name, ret);
			return ret;
		}
		if (t_prod->count != ret) {
			dev_err(dev, "Node %s: prod read failed: exp %d read %d\n",
				child->name, t_prod->count, ret);
			return -EINVAL;
		}

		/* Optimise the prod configuration */
		if (mask_opt)
			tegra_prod_optimise_tupple(t_prod->prod_tuple,
						   t_prod->count);


		n_child++;
	}

	tegra_prod->num = n_child;

	return 0;
}

/**
 * tegra_prod_set_tuple - Only set a tuple.
 * @base:		base address of the register.
 * @prod_tuple:		the tuple to set.
 * @new_mask:		Mask override value, 0 means use from tupple.
 *
 * Returns 0 on success.
 */
static int tegra_prod_set_tuple(void __iomem **base,
				struct prod_tuple *prod_tuple,
				u32 new_mask)
{
	u32 reg;
	u32 mask = (new_mask) ? new_mask : prod_tuple->mask;

	if (!prod_tuple)
		return -EINVAL;

	reg = readl(base[prod_tuple->index] + prod_tuple->addr);
	reg = ((reg & ~mask) | (prod_tuple->val & mask));

	writel(reg, base[prod_tuple->index] + prod_tuple->addr);

	return 0;
}

/**
 * tegra_prod_set - Set one prod setting.
 * @base:		base address of the register.
 * @tegra_prod:		the prod setting to set.
 *
 * Set all the tuples in one tegra_prod.
 * Returns 0 on success.
 */
static int tegra_prod_set(void __iomem **base,
			  struct tegra_prod_config *tegra_prod)
{
	int i;
	int ret;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->count; i++) {
		ret = tegra_prod_set_tuple(base, &tegra_prod->prod_tuple[i], 0);
		if (ret)
			return ret;
	}

	return 0;
}

/**
 * tegra_prod_set_list - Set all the prod settings of the list in sequence.
 * @base:		base address of the register.
 * @tegra_prod:	the list of tegra prods.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_list(void __iomem **base,
		struct tegra_prod *tegra_prod)
{
	int i;
	int ret;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->num; i++) {
		ret = tegra_prod_set(base, &tegra_prod->prod_config[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_prod_set_list);

/**
 * tegra_prod_set_boot_init - Set all the prod settings of the list in sequence
 *			Which are needed for boot initialisation.
 * @base:		base address of the register.
 * @tegra_prod:	the list of tegra prods.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_boot_init(void __iomem **base,
		struct tegra_prod *tegra_prod)
{
	int i;
	int ret;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->num; i++) {
		if (!tegra_prod->prod_config[i].boot_init)
			continue;
		ret = tegra_prod_set(base, &tegra_prod->prod_config[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_prod_set_boot_init);

/**
 * tegra_prod_set_by_name - Set the prod setting according the name.
 * @base:		base address of the register.
 * @name:		the name of tegra prod need to set.
 * @tegra_prod:	the list of tegra prods.
 *
 * Find the tegra prod in the list according to the name. Then set
 * that tegra prod.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_by_name(void __iomem **base, const char *name,
		struct tegra_prod *tegra_prod)
{
	int i;
	struct tegra_prod_config *t_prod;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->num; i++) {
		t_prod = &tegra_prod->prod_config[i];
		if (!strcasecmp(t_prod->name, name))
			return tegra_prod_set(base, t_prod);
	}

	return -ENODEV;
}
EXPORT_SYMBOL(tegra_prod_set_by_name);

/**
 * tegra_prod_set_by_name_partially - Set the prod setting from list partially
 *				      under given prod name. The matching is done
 *				      qith index, offset and mask.
 * @base:		base address of the register.
 * @name:		the name of tegra prod need to set.
 * @tegra_prod:	the list of tegra prods.
 * @index:		Index of base address.
 * @offset:		Offset of the register.
 * @mask:		Mask field on given register.
 *
 * Find the tegra prod in the list according to the name. Then set
 * that tegra prod which has matching of index, offset and mask.
 *
 * Returns 0 on success.
 */
int tegra_prod_set_by_name_partially(void __iomem **base, const char *name,
				     struct tegra_prod *tegra_prod, u32 index,
				     u32 offset, u32 mask)
{
	struct tegra_prod_config *t_prod;
	int ret;
	int i;
	bool found = false;

	if (!tegra_prod)
		return -EINVAL;

	for (i = 0; i < tegra_prod->num; i++) {
		t_prod = &tegra_prod->prod_config[i];
		if (!strcasecmp(t_prod->name, name)) {
			found = true;
			break;
		}
	}

	if (!found)
		return -ENODEV;

	for (i = 0; i < t_prod->count; i++) {
		struct prod_tuple *ptuple = &t_prod->prod_tuple[i];;

		if ((ptuple->index != index) || (ptuple->addr != offset) ||
		    ((ptuple->mask & mask) != mask))
			continue;

		ret = tegra_prod_set_tuple(base, ptuple, mask);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_prod_set_by_name_partially);

bool tegra_prod_by_name_supported(struct tegra_prod *tegra_prod,
				  const char *name)
{
	int i;
	struct tegra_prod_config *t_prod;

	if (!tegra_prod)
		return false;

	for (i = 0; i < tegra_prod->num; i++) {
		t_prod = &tegra_prod->prod_config[i];
		if (!t_prod)
			break;

		if (!strcasecmp(t_prod->name, name))
			return true;
	}

	return false;
}
EXPORT_SYMBOL(tegra_prod_by_name_supported);

/**
 * tegra_prod_init - Init tegra prod list.
 # @dev:	Device handle.
 * @np:		device node from which the property value is to be read.
 *
 * Query all the prod settings under DT node & Init the tegra prod list
 * automatically.
 *
 * Returns 0 on success, -EINVAL for wrong prod number, -ENOMEM if faild
 * to allocate memory for tegra prod list.
 */
static struct tegra_prod *tegra_prod_init(struct device *dev,
					  const struct device_node *np)
{
	struct tegra_prod *tegra_prod;
	struct device_node *np_prod;
	int prod_num = 0;
	int ret;

	np_prod = of_get_child_by_name(np, "prod-settings");
	if (!np_prod)
		return ERR_PTR(-ENODEV);

	/* Check whether child is enabled or not */
	if (!of_device_is_available(np_prod)) {
		dev_err(dev, "Node %s: Node is not enabled\n", np_prod->name);
		return ERR_PTR(-ENODEV);
	}

	prod_num = of_get_child_count(np_prod);
	if (prod_num <= 0) {
		dev_err(dev, "Node %s: No child node for prod settings\n",
			np_prod->name);
		return  ERR_PTR(-ENODEV);
	}

	tegra_prod = devm_kzalloc(dev, sizeof(*tegra_prod), GFP_KERNEL);
	if (!tegra_prod)
		return  ERR_PTR(-ENOMEM);

	tegra_prod->prod_config = devm_kcalloc(dev, prod_num,
					       sizeof(*tegra_prod->prod_config),
					       GFP_KERNEL);
	if (!tegra_prod->prod_config)
		return ERR_PTR(-ENOMEM);

	tegra_prod->num = prod_num;

	ret = tegra_prod_parse_dt(dev, np, np_prod, tegra_prod);
	if (ret) {
		dev_err(dev, "Node %s: Faild to read the Prod Setting.\n",
			np->name);
		return ERR_PTR(ret);
	}

	return tegra_prod;
}

static void devm_tegra_prod_release(struct device *dev, void *res)
{
}

struct tegra_prod *devm_tegra_prod_get(struct device *dev)
{
	struct tegra_prod **ptr, *prod_list;

	ptr = devres_alloc(devm_tegra_prod_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	prod_list = tegra_prod_init(dev, dev->of_node);
	if (IS_ERR(prod_list)) {
		devres_free(ptr);
		return prod_list;
	}

	*ptr = prod_list;
	devres_add(dev, ptr);

	return prod_list;
}
EXPORT_SYMBOL(devm_tegra_prod_get);

struct tegra_prod *devm_tegra_prod_get_from_node(struct device *dev,
						 struct device_node *np)
{
	struct tegra_prod **ptr, *prod_list;

	ptr = devres_alloc(devm_tegra_prod_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	prod_list = tegra_prod_init(dev, np);
	if (IS_ERR(prod_list)) {
		devres_free(ptr);
		return prod_list;
	}

	*ptr = prod_list;
	devres_add(dev, ptr);

	return prod_list;
}
EXPORT_SYMBOL(devm_tegra_prod_get_from_node);
