/*
 * mods_tegraprod.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mods_internal.h"
#include <linux/err.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/tegra_prod.h>

#define MAX_REG_INFO_ENTRY	400
#define MAX_IO_MAP_ENTRY	200

static struct device *mods_tegra_prod_dev;

/**
 * mods_tegra_prod_init - Initialize tegra prod module.
 * @misc_dev:       pointer to mods device
 *
 * Returns 0 on success, others for error.
 */
int mods_tegra_prod_init(const struct miscdevice *misc_dev)
{
	if (!misc_dev) {
		mods_error_printk("mods device pointer is NULL\n");
		return -EINVAL;
	}

	mods_tegra_prod_dev = misc_dev->this_device;
	if (!mods_tegra_prod_dev) {
		mods_error_printk("this_device in mods device is NULL\n");
		return -EINVAL;
	}

	return OK;
}

/**
 * esc_mods_tegra_prod_iterate_dt - Find device_node by device name.
 * @MODS_TEGRA_PROD_ITERATOR: Input information, only the following
 *				members are used.
 *   +device_handle:	Last node pointer for current iteration
 *   +name:		Current node name
 *   +next_name:	Next node name
 *   +index:		Index of the device nodes shared the same name
 *			(NOTE: index starts from 0)
 *   +is_leaf:		Is the node with current name a leaf node
 *   +next_device_handle: Result node pointer for next iteration
 * Returns 0 on success, others for error or no matching node found.
 */
int esc_mods_tegra_prod_iterate_dt(
	struct file *fp,
	struct MODS_TEGRA_PROD_ITERATOR *iterator
)
{
	struct device_node *dev_node;
	struct device_node *child_node = NULL;
	struct resource res;
	int ret_resource = -1;
	int i;

	dev_node = (struct device_node *)iterator->device_handle;

	if (!iterator->name[0]) {
		mods_error_printk(
			"node name is missing for tegra prod value\n");
		return -EINVAL;
	}
	if (!iterator->next_name[0] && !iterator->is_leaf) {
		mods_error_printk("inner node with empty next_name\n");
		return -EINVAL;
	}

	/* Need to search several times since some nodes may share same name */
	for (i = 0; i <= iterator->index; ++i) {
		/* Search from Left to Right in DT */
		dev_node = of_find_node_by_name(dev_node, iterator->name);
		if (!dev_node) {
			mods_error_printk("node %s not found in device tree\n",
					iterator->name);
			return -EINVAL;
		}

		/*
		 * Search from Top to Bottom in DT
		 * Check: does this node need to be filtered out
		 */
		if (iterator->is_leaf) {
			/* Leaf node : search for "prod-settings" node */
			child_node = of_get_child_by_name(dev_node,
							"prod-settings");
			ret_resource = of_address_to_resource(
							dev_node, 0, &res);
		} else {
			/* Inner node : search for next device node */
			child_node = of_get_child_by_name(dev_node,
							iterator->next_name);
			ret_resource = -1;
		}
		if (!child_node && ret_resource < 0)
			--i;	/* Not a valid device node */
	}

	/* Return next_device_handler to ioctl caller */
	iterator->next_device_handle = (__u64)dev_node;
	return OK;
}

/**
 * mods_read_reg_info - Read register information (address and size)
 * @dev_node:		 Device node pointer
 * @reg_info:		 Register information array
 * @count_reg_cells:	 (OUT) Pointer to count of cells in "reg"
 * @count_addrsize_pair: (OUT) Pointer to count of each pair in "reg"
 * @count_address_cells: (OUT) Pointer to count of "address" cell in "reg"
 * @count_size_cells:	 (OUT) Pointer to count of "size" cell in "reg"
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
static int mods_read_reg_info(
	const struct device_node *dev_node,
	__u32 *reg_info,
	__u32 *count_reg_cells,
	__u32 *count_addrsize_pair,
	__u32 *count_address_cells,
	__u32 *count_size_cells
)
{
	struct device_node *parent_node;
	int ret;

	/* Get parent node to read the format of "reg" property */
	parent_node = dev_node->parent;
	ret = of_property_read_u32(parent_node, "#address-cells",
				count_address_cells);
	if (ret < 0) {
		mods_error_printk("Read #address-cells failed\n");
		return ret;

	} else if (*count_address_cells == 0) {
		mods_error_printk("#address-cells cannot be 0\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(parent_node, "#size-cells",
				count_size_cells);
	if (ret < 0) {
		mods_error_printk("Read #size-cells failed\n");
		return ret;

	} else if (*count_size_cells == 0) {
		mods_error_printk("#size-cells cannot be 0\n");
		return -EINVAL;
	}
	*count_addrsize_pair = *count_address_cells + *count_size_cells;

	/* Read count of cells in "reg" property */
	ret = of_property_count_u32_elems(dev_node, "reg");
	if (ret < 0) {
		mods_error_printk(
			"Unable to get count of cells in \"reg\" of node %s\n",
			dev_node->name);
		return ret;
	}

	*count_reg_cells = (__u32)ret;
	if (*count_reg_cells == 0) {
		mods_error_printk("No \"reg\" info is available\n");
		return -EINVAL;

	} else if (*count_reg_cells % *count_addrsize_pair != 0) {
		/*
		 * "reg" property may have more than 1 pairs of address
		 * and size. The length of each pair is represented by
		 * "count_addrsize_pair"; the total length of "reg"
		 * property is represented by "count_reg_cells". If
		 * "count_reg_cells" is not an integral multiple of
		 * "count_addrsize_pair", the read "reg" information is
		 * incomplete or incorrect.
		 */
		mods_error_printk(
			"\"reg\" property has invalid length : %d\n",
			*count_reg_cells);
		return -EINVAL;
	}

	/* Read register information from "reg" property */
	ret = of_property_read_u32_array(dev_node, "reg", reg_info,
					(size_t)(*count_reg_cells));
	if (ret < 0) {
		mods_error_printk(
			"Unable to read \"reg\" property of node %s\n",
			dev_node->name);
		return ret;
	}

	return OK;
}

/**
 * mods_batch_iounmap - Unmap all virtual memory mapped by IO address.
 * @io_base:		IO address array
 * @count_io_base:	Count of IO addresses are mapped
 *
 */
static void mods_batch_iounmap(
	void __iomem **io_base,
	__u32 count_io_base
)
{
	int i;

	if (!io_base) {
		mods_error_printk("IO base address array is NULL\n");
		return;
	}

	for (i = 0; i < count_io_base; ++i)
		iounmap(io_base[i]);
}

/**
 * mods_batch_iomap - Get all segments of IO address from a device node,
 *                    and create a virtual memory mapping.
 * @dev_node:		Device node pointer
 * @io_base:		(OUT) IO address array
 * @count_io_base:	(OUT) Pointer to the count of IO addresses are mapped
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR (NOTE: partially mapped IO memory will be unmapped on error)
 *
 */
static int mods_batch_iomap(
	const struct device_node *dev_node,
	void __iomem **io_base,
	__u32 *count_io_base
)
{
	__u32 reg_info[MAX_REG_INFO_ENTRY];
	__u32 count_reg_cells = 0;
	__u32 count_addrsize_pair, count_address_cells, count_size_cells;
	__u64 io_addr_base, io_addr_length;
	void __iomem *io_addr_mapped;
	int ret;
	int i;

	/* Check arguments */
	if (!dev_node) {
		mods_error_printk("Controller device handle is NULL\n");
		return -EINVAL;
	}
	if (!io_base) {
		mods_error_printk("IO base address array is NULL\n");
		return -EINVAL;
	}
	if (!count_io_base) {
		mods_error_printk("Count of IO base address array is NULL\n");
		return -EINVAL;
	}

	/* Get registers information */
	ret = mods_read_reg_info(
		dev_node,
		reg_info,
		&count_reg_cells,
		&count_addrsize_pair,
		&count_address_cells,
		&count_size_cells
	);
	if (ret < 0)
		return ret;

	/* IO mapping - using 64-bit physical address */
	*count_io_base = 0;
	for (i = 0; i < count_reg_cells;
		i += count_addrsize_pair, *count_io_base += 1) {

		/* Compute IO physical address */
		/* Lower 32-bit IO physical address in DTB */
		io_addr_base = reg_info[i + count_address_cells - 1];
		if (count_address_cells > 1) {
			/* Higher 32-bit IO physical address in DTB */
			io_addr_base += ((__u64)reg_info
					[i + count_address_cells - 2]) << 32;
		}

		/* Compute IO address space length */
		/* Lower 32-bit IO address space length in DTB */
		io_addr_length = reg_info[i + count_addrsize_pair - 1];
		if (count_size_cells > 1) {
			/* Higher 32-bit IO address space length in DTB */
			io_addr_length += ((__u64)reg_info
					[i + count_addrsize_pair - 2]) << 32;
		}

		/* Map physical address to virtual address space */
		io_addr_mapped = ioremap((resource_size_t)io_addr_base,
					(resource_size_t)io_addr_length);
		if (io_addr_mapped == NULL) {
			mods_error_printk(
			"Unable to map io address 0x%llx, length 0x%llx\n",
			io_addr_base, io_addr_length);
			/* Clean :
			 * Unmap the IO memory that have already been mapped
			 */
			mods_batch_iounmap(io_base, *count_io_base);
			return -ENXIO;
		}
		io_base[*count_io_base] = io_addr_mapped;
	}

	return OK;
}


/**
 * mods_tegra_get_prod_list - Get tegra_prod list by valid device node.
 * @dev_node:       Device node.
 *
 * Returns non-NULL on success, NULL on error.
 */
static struct tegra_prod *mods_tegra_get_prod_list(struct device_node *dev_node)
{
	struct tegra_prod *prod_list;

	if (!mods_tegra_prod_dev) {
		mods_error_printk("tegra prod is not initialized\n");
		return NULL;
	}

	if (!dev_node) {
		mods_error_printk("device node is NULL\n");
		return NULL;
	}

	prod_list = devm_tegra_prod_get_from_node(mods_tegra_prod_dev,
						dev_node);
	if (IS_ERR(prod_list)) {
		mods_error_printk("failed to get prod_list : %s\n",
				dev_node->name);
		return NULL;
	}

	return prod_list;
}

/**
 * mods_tegra_get_prod_info - Get tegra_prod_list & mapped IO addresses
 * @MODS_TEGRA_PROD_SET_TUPLE:  Input information, only the following
 *				members are used.
 *   +prod_node_handle:	Handle of device node contained prod values.
 *   +ctrl_node_handle:	Handle of controller device node.
 * @tegra_prod_list:	(OUT) Pointer to tegra_prod_list.
 * @ctrl_base:		(OUT) Array of mapped IO address.
 * @count_ctrl_base:	(OUT) Pointer to count of mapped IO address.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
static int mods_tegra_get_prod_info(
	const struct MODS_TEGRA_PROD_SET_TUPLE *tuple,
	struct tegra_prod **tegra_prod_list,
	void __iomem **ctrl_base,
	__u32 *count_ctrl_base
)
{
	struct device_node *prod_node, *ctrl_node;

	if (!tegra_prod_list || !ctrl_base || !count_ctrl_base) {
		mods_error_printk("Detected NULL pointer for out value.");
		return -EINVAL;
	}

	prod_node = (struct device_node *)tuple->prod_dev_handle;
	if (!prod_node) {
		mods_error_printk("Prod device handle is NULL\n");
		return -EINVAL;
	}

	*tegra_prod_list = mods_tegra_get_prod_list(prod_node);
	if (!(*tegra_prod_list)) {
		mods_error_printk(
		"Failed to get prod_list with prod handle 0x%llx\n",
		tuple->prod_dev_handle);
		return -EINVAL;
	}

	ctrl_node = (struct device_node *)tuple->ctrl_dev_handle;
	if (!ctrl_node) {
		mods_error_printk("Controller device handle is NULL\n");
		return -EINVAL;
	}

	return mods_batch_iomap(ctrl_node, ctrl_base, count_ctrl_base);
}

/**
 * esc_mods_tegra_prod_is_supported - Test the prod is supported by
 *					given prod name.
 * @MODS_TEGRA_PROD_IS_SUPPORTED:  Input information, only the following
				   members are used.
 *   +prod_node_handle:     Handle of device node contained prod values.
 *   +prod_name:            Prod name.
 *   +is_supported:         Result of supporting given prod configuration.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
int esc_mods_tegra_prod_is_supported(
	struct file *fp,
	struct MODS_TEGRA_PROD_IS_SUPPORTED *tuple
)
{
	struct device_node *prod_node;
	struct tegra_prod *tegra_prod;
	bool is_supported;

	prod_node = (struct device_node *)tuple->prod_dev_handle;
	if (!prod_node) {
		mods_error_printk("Prod device handle is NULL\n");
		return -EINVAL;
	}

	tegra_prod = mods_tegra_get_prod_list(prod_node);
	if (!tegra_prod) {
		mods_error_printk(
		"Failed to get prod_list with prod handle 0x%llx\n",
		tuple->prod_dev_handle);
		return -EINVAL;
	}

	is_supported = tegra_prod_by_name_supported(tegra_prod,
						tuple->prod_name);
	tuple->is_supported = is_supported ? 1 : 0;

	return OK;
}

/**
 * esc_mods_tegra_prod_set_prod_all - Read prod values from prod device node,
 *                                    and set prod values to controller device
 *				      node.
 * @MODS_TEGRA_PROD_SET_TUPLE:  Input information, only the following
 *				members are used.
 *   +prod_node_handle:	Handle of device node contained prod values.
 *   +ctrl_node_handle:	Handle of controller device node.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
int esc_mods_tegra_prod_set_prod_all(
	struct file *fp,
	struct MODS_TEGRA_PROD_SET_TUPLE *tuple
)
{
	int ret;
	struct tegra_prod *tegra_prod_list;
	void __iomem *ctrl_base[MAX_IO_MAP_ENTRY];
	__u32 count_ctrl_base;

	ret = mods_tegra_get_prod_info(tuple, &tegra_prod_list,
				ctrl_base, &count_ctrl_base);
	if (ret < 0)
		return ret;

	ret = tegra_prod_set_list(ctrl_base, tegra_prod_list);
	if (ret < 0)
		mods_error_printk("Set prod failed\n");

	mods_batch_iounmap(ctrl_base, count_ctrl_base);

	return ret;
}

/**
 * esc_mods_tegra_prod_set_prod_boot - Read prod values from prod device
 *				       node, and set prod values to
 *				       controller device node, which are
 *				       required for boot initialization.
 * @MODS_TEGRA_PROD_SET_TUPLE:  Input information, only the following
 *				members are used.
 *   +prod_node_handle:	Handle of device node contained prod values.
 *   +ctrl_node_handle:	Handle of controller device node.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
int esc_mods_tegra_prod_set_prod_boot(
	struct file *fp,
	struct MODS_TEGRA_PROD_SET_TUPLE *tuple
)
{
	int ret;
	struct tegra_prod *tegra_prod_list;
	void __iomem *ctrl_base[MAX_IO_MAP_ENTRY];
	__u32 count_ctrl_base;

	ret = mods_tegra_get_prod_info(tuple, &tegra_prod_list,
				ctrl_base, &count_ctrl_base);
	if (ret < 0)
		return ret;

	ret = tegra_prod_set_boot_init(ctrl_base, tegra_prod_list);
	if (ret < 0)
		mods_error_printk("Set boot init prod failed\n");

	mods_batch_iounmap(ctrl_base, count_ctrl_base);

	return ret;
}

/**
 * esc_mods_tegra_prod_set_prod_by_name - Read prod values from prod
 *					  device node, and set prod values
 *					  to controller device node based
 *					  on prod name.
 * @MODS_TEGRA_PROD_SET_TUPLE:  Input information, only the following
 *				members are used.
 *   +prod_node_handle:	Handle of device node contained prod values.
 *   +ctrl_node_handle:	Handle of controller device node.
 *   +prod_name:	Prod name.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
int esc_mods_tegra_prod_set_prod_by_name(
	struct file *fp,
	struct MODS_TEGRA_PROD_SET_TUPLE *tuple
)
{
	int ret;
	struct tegra_prod *tegra_prod_list;
	void __iomem *ctrl_base[MAX_IO_MAP_ENTRY];
	__u32 count_ctrl_base;

	ret = mods_tegra_get_prod_info(tuple, &tegra_prod_list,
				ctrl_base, &count_ctrl_base);
	if (ret < 0)
		return ret;

	ret = tegra_prod_set_by_name(ctrl_base, tuple->prod_name,
				tegra_prod_list);
	if (ret < 0) {
		mods_error_printk("Set prod by name \"%s\" failed\n",
				tuple->prod_name);
	}

	mods_batch_iounmap(ctrl_base, count_ctrl_base);

	return ret;
}

/**
 * esc_mods_tegra_prod_set_prod_exact - Read prod values from prod device
 *					node, and set prod values to
 *					controller device node based
 *					on prod name, index, offset, and mask.
 * @MODS_TEGRA_PROD_SET_TUPLE:	Input information, only the following
				members are used.
 *   +prod_node_handle:	Handle of device node contained prod values.
 *   +ctrl_node_handle:	Handle of controller device node.
 *   +prod_name:	Prod name.
 *   +index:		Index of base address.
 *   +offset:		Offset of the register.
 *   +mask:		Mask field on given register.
 *
 * Return Value :
 *    0   - OK
 * Others - ERROR
 *
 */
int esc_mods_tegra_prod_set_prod_exact(
	struct file *fp,
	struct MODS_TEGRA_PROD_SET_TUPLE *tuple
)
{
	int ret;
	struct tegra_prod *tegra_prod_list;
	void __iomem *ctrl_base[MAX_IO_MAP_ENTRY];
	__u32 count_ctrl_base;

	ret = mods_tegra_get_prod_info(tuple, &tegra_prod_list,
				ctrl_base, &count_ctrl_base);
	if (ret < 0)
		return ret;

	ret = tegra_prod_set_by_name_partially(ctrl_base, tuple->prod_name,
				tegra_prod_list, tuple->index, tuple->offset,
				tuple->mask);
	if (ret < 0) {
		mods_error_printk("Set prod exact by name \"%s\" failed\n",
				tuple->prod_name);
		mods_error_printk("index [%x]; offset [%x]; mask [%x]\n",
				tuple->index, tuple->offset, tuple->mask);
	}

	mods_batch_iounmap(ctrl_base, count_ctrl_base);

	return ret;
}

