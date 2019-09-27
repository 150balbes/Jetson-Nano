/*
 * Copyright (c) 2016, Harman Becker.  All rights reserved.
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/io.h>
#include "tegra_bootloader_debug.h"

static void *tegra_bl_mapped_boot_cfg_start;

static ssize_t bcp_data_read(struct file *filp, struct kobject *kobj,
			     struct bin_attribute *bin_attr,
			     char *buf, loff_t off, size_t count)
{
	uint8_t *data = (uint8_t *) tegra_bl_mapped_boot_cfg_start;
	/* read from the reserved memory */
	memcpy(buf, data + off, count);

	return count;
}

static struct bin_attribute bcp_attr_data = {
	.attr = {
		 .name = "boot_cfg",
		 .mode = 0444,
	},
	.size = 0,
	.read = bcp_data_read,
	.write = NULL,
};

static __init int bcp_init(void)
{
	int ret = 0;
	void __iomem *ptr_bl_boot_cfg_start = NULL;

	tegra_bl_mapped_boot_cfg_start = NULL;

	/* If the following variables are not set then we don't have BCP data */
	if (!tegra_bl_bcp_size || !tegra_bl_bcp_start)
		return -ENXIO;

	tegra_bl_mapped_boot_cfg_start = phys_to_virt(tegra_bl_bcp_start);
	if (!pfn_valid(__phys_to_pfn(tegra_bl_bcp_start))) {
		ptr_bl_boot_cfg_start =
		    ioremap(tegra_bl_bcp_start, tegra_bl_bcp_size);

		WARN_ON(!ptr_bl_boot_cfg_start);
		if (!ptr_bl_boot_cfg_start) {
			pr_error("%s: Failed to map tegra_bl_prof_start %08x\n",
				__func__, (unsigned int)tegra_bl_bcp_start);
			ret = -ENXIO;
			goto out_err;
		}

		tegra_bl_mapped_boot_cfg_start = ptr_bl_boot_cfg_start;
	}

	bcp_attr_data.size = tegra_bl_bcp_size;

	ret = sysfs_create_bin_file(kernel_kobj, &bcp_attr_data);
	if (ret) {
		pr_error("%s: sysfs_create_bin_file failed\n",
		       __func__);
		goto out_err;
	}
	return ret;

out_err:
	if (ptr_bl_boot_cfg_start)
		iounmap(ptr_bl_boot_cfg_start);
	return ret;
}

static void __exit bcp_exit(void)
{
	if (tegra_bl_mapped_boot_cfg_start)
		iounmap(tegra_bl_mapped_boot_cfg_start);

	sysfs_remove_bin_file(kernel_kobj, &bcp_attr_data);
}

module_init(bcp_init);
module_exit(bcp_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Driver to expose BCP data to user space");
MODULE_AUTHOR("Lukasz Taborowski <lukasz.taborowski@harman.com>");
