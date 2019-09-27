/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/platform/tegra/tegra18_cpu_map.h>
#include <linux/platform/tegra/tegra-cpu.h>
#include <linux/cpuhotplug.h>

enum cputype_t {
	INVALID = 0x00,
	CARMEL = 0x1,
	DENVER,
	A57,
	OTHERS,
	CPUTYPE_MAX = OTHERS + 1,
	CPUTYPE_FORCE_U32 = 0xFFFFFFFF
};

static struct kobject *tegra_cpu_type;
static struct kobject *tegra_cpu[CPUTYPE_MAX];
static enum cputype_t cpu_to_type[NR_CPUS];
static enum cpuhp_state assigned_cpu_state;

static struct kobject *create_dir(const char *name, struct kobject *parent)
{
	pr_debug("%s: about to create %s with parent %s\n",
		__func__, name, kobject_name(parent));
	return kobject_create_and_add(name, parent);
}

static void remove_dir(struct kobject *obj)
{
	if (!obj)
		return;
	pr_debug("%s: removing %s\n", __func__, kobject_name(obj));
	kobject_put(obj);
}

static int tegra_cpu_prepare_down(unsigned int cpu)
{
	char name[8];
	int i;
	enum cputype_t cpu_type;

	pr_debug("%s: cpu = %d\n", __func__, cpu);
	cpu_type = cpu_to_type[cpu];

	if (!tegra_cpu[cpu_type])
		return -ENODEV;

	snprintf(name, sizeof(name), "cpu%d", cpu);
	sysfs_remove_link(tegra_cpu[cpu_type], name);
	cpu_to_type[cpu] = INVALID;

	for_each_possible_cpu(i) {
		if (cpu_to_type[i] == cpu_type)
			return 0;
	}
	remove_dir(tegra_cpu[cpu_type]);
	return 0;
}

static int tegra_cpu_online(unsigned int cpu)
{
	struct device *target;
	enum cputype_t cpu_type;
	char *cpu_type_str;
	int ret;

	pr_debug("%s: cpu = %d\n", __func__, cpu);

	target = get_cpu_device(cpu);
	if (!target)
		return -ENODEV;

	if (tegra_is_cpu_carmel(cpu)) {
		cpu_type = CARMEL;
		cpu_type_str = "carmel";

	} else if (tegra18_is_cpu_denver(cpu)) {
		cpu_type = DENVER;
		cpu_type_str = "denver";

	} else if (tegra18_is_cpu_arm(cpu)) {
		cpu_type = A57;
		cpu_type_str = "a57";

	} else {
		cpu_type = OTHERS;
		cpu_type_str = "others";
	}

	if (!tegra_cpu[cpu_type]) {
		tegra_cpu[cpu_type] = create_dir(cpu_type_str, tegra_cpu_type);
		if (tegra_cpu[cpu_type] == NULL)
			return -ENOMEM;
	}

	ret = sysfs_create_link(tegra_cpu[cpu_type],
			&target->kobj, dev_name(target));
	if (ret)
		return ret;

	cpu_to_type[cpu] = cpu_type;
	return 0;
}

static int __init tegra_cpu_sysfs_init(void)
{
	int ret = -ENOMEM;

	tegra_cpu_type = create_dir("tegra_cpu_type",
		get_cpu_device(smp_processor_id())->kobj.parent);

	if (!tegra_cpu_type)
		goto err;

	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
	"cpu/tegra_cpu_sysfs:online", tegra_cpu_online, tegra_cpu_prepare_down);
	if (ret < 0) {
		remove_dir(tegra_cpu_type);
		goto err;
	}

	assigned_cpu_state = ret;
	ret = 0;
err:
	return ret;

}
module_init(tegra_cpu_sysfs_init);

static void __exit tegra_cpu_sysfs_exit(void)
{
	cpuhp_remove_state(assigned_cpu_state);
	remove_dir(tegra_cpu_type);
}
module_exit(tegra_cpu_sysfs_exit);

MODULE_AUTHOR("Achal Verma <achalv@nvidia.com>");
MODULE_DESCRIPTION("Tegra CPU Topology sysfs");
MODULE_LICENSE("GPL v2");
