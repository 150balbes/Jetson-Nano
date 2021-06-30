/*
 * drivers/nvpmodel/nvpmodel_emc_cap.c
 *
 * NVIDIA Tegra Nvpmodel driver for Tegra chips
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/bwmgr_mc.h>

#define AUTHOR "Terry Wang <terwang@nvidia.com>"
#define DESCRIPTION "Nvpmodel clock cap driver"
#define MODULE_NAME "Nvpmodel_clk_cap"
#define VERSION "1.0"

/* Module information */
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION(DESCRIPTION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");

static struct kobject *clk_cap_kobject;
static unsigned long emc_iso_cap;

/* bandwidth manager handle */
struct tegra_bwmgr_client *bwmgr_handle;

struct nvpmodel_clk {
	struct kobj_attribute attr;
	struct clk *clk;
};

static struct nvpmodel_clk *clks;
static int num_clocks;

static ssize_t emc_iso_cap_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", emc_iso_cap);
}

static ssize_t emc_iso_cap_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	int error = 0;
	if (sscanf(buf, "%lu", &emc_iso_cap) != 1)
		return -EINVAL;

	error = tegra_bwmgr_set_emc(bwmgr_handle, emc_iso_cap,
				TEGRA_BWMGR_SET_EMC_ISO_CAP);
	if (error) {
		pr_warn("Nvpmodel failed to set EMC hz=%lu errno=%d\n",
			emc_iso_cap, error);
		return error;
	}

	return count;
}

static struct kobj_attribute emc_iso_cap_attribute =
	__ATTR(emc_iso_cap, 0660, emc_iso_cap_show, emc_iso_cap_store);

static ssize_t clk_cap_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	long rate;
	struct nvpmodel_clk *clk = container_of(attr, struct nvpmodel_clk, attr);

	rate = clk_round_rate(clk->clk, 1UL << 63);
	if (rate < 0) {
		pr_err("clk_round_rate failed: %ld\n", rate);
		return rate;
	}

	return sprintf(buf, "%ld\n", rate);
}

static ssize_t clk_cap_store(struct kobject *kobj,
				struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	unsigned long rate;
	int ret;
	struct nvpmodel_clk *clk = container_of(attr, struct nvpmodel_clk, attr);

	if (sscanf(buf, "%lu", &rate) != 1)
		return -EINVAL;

	ret = clk_set_max_rate(clk->clk, rate);
	if (ret) {
		pr_err("setting cap failed: %d\n", ret);
		return ret;
	}

	return count;
}

static void free_resources(void)
{
	int i;

	if (clks) {
		for (i = 0; i < num_clocks; i++) {
			if (clks[i].attr.attr.name)
				kfree_const(clks[i].attr.attr.name);
			if (clks[i].clk)
				clk_put(clks[i].clk);
		}
		kfree(clks);
		clks = NULL;
		num_clocks = 0;
	}
	if (bwmgr_handle) {
		tegra_bwmgr_unregister(bwmgr_handle);
		bwmgr_handle = NULL;
	}
	if (clk_cap_kobject) {
		kobject_put(clk_cap_kobject);
		clk_cap_kobject = NULL;
	}
}

static int __init nvpmodel_clk_cap_init(void)
{
	int error = 0;
	int i;
	const char *clk_name;
	struct device_node *dn;

	dn = of_find_compatible_node(NULL, NULL, "nvidia,nvpmodel");
	if (!dn || !of_device_is_available(dn)) {
		of_node_put(dn);
		return -ENODEV;
	}

	clk_cap_kobject = kobject_create_and_add("nvpmodel_emc_cap",
						 kernel_kobj);
	if (!clk_cap_kobject) {
		error = -ENOMEM;
		goto exit;
	}

	bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_NVPMODEL);
	if (IS_ERR_OR_NULL(bwmgr_handle)) {
		error = IS_ERR(bwmgr_handle) ?
			PTR_ERR(bwmgr_handle) : -ENODEV;
		pr_err("Nvpmodel can't register EMC bwmgr (%d)\n", error);
		goto exit;
	}

	error = sysfs_create_file(clk_cap_kobject,
				&emc_iso_cap_attribute.attr);
	if (error) {
		pr_err("failed to create emc_iso_cap sysfs: error %d\n", error);
		goto exit;
	}

	num_clocks = of_property_count_strings(dn, "clock-names");
	if (num_clocks <= 0) {
		num_clocks = 0;
		goto exit;
	}

	clks = kzalloc(sizeof(*clks) * num_clocks, GFP_KERNEL);
	if (!clks) {
		num_clocks = 0;
		pr_err("couldn't allocate clks!\n");
		error = -ENOMEM;
		goto exit;
	}

	for (i = 0; i < num_clocks; i++) {
		if (of_property_read_string_index(dn, "clock-names", i,
					&clk_name)) {
			pr_warn("couldn't get clock %d from device tree\n", i);
			continue;
		}

		clks[i].clk = of_clk_get(dn, i);
		if (IS_ERR(clks[i].clk)) {
			clks[i].clk = NULL;
			pr_warn("couldn't get clock: %s, error %d\n", clk_name,
					error);
			continue;
		}

		clks[i].attr.attr.name = kstrdup_const(clk_name, GFP_KERNEL);
		if (!clks[i].attr.attr.name) {
			pr_warn("couldn't allocate memory for clock %s\n",
					clk_name);
			continue;
		}
		sysfs_attr_init(&(clks[i].attr.attr));
		clks[i].attr.attr.mode = 0660;
		clks[i].attr.show = clk_cap_show;
		clks[i].attr.store = clk_cap_store;
		if (sysfs_create_file(clk_cap_kobject,
				&clks[i].attr.attr)) {
			pr_warn("failed to create %s cap sysfs file: error %d\n",
					clk_name, error);
			continue;
		}
	}
exit:
	if (error) {
		free_resources();
		pr_err("nvpmodel: initialization failed: error %d\n", error);
	} else {
		pr_info("nvpmodel: initialized successfully\n");
	}
	of_node_put(dn);
	return error;
}

static void __exit nvpmodel_clk_cap_exit(void)
{
	free_resources();
	pr_info("Module exit successfully \n");
}

module_init(nvpmodel_clk_cap_init);
module_exit(nvpmodel_clk_cap_exit);
