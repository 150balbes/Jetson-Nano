/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/of.h>
#include <linux/sort.h>
#include <linux/debugfs.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/platform/tegra/emc_bwmgr.h>

#include "cpufreq_cpu_emc_table.h"

static int cmp_freq(const void *_a, const void *_b)
{
	const struct cpu_emc_mapping *a = _a, *b = _b;

	if (a->cpu_freq_khz < b->cpu_freq_khz)
		return -1;
	else if (a->cpu_freq_khz == b->cpu_freq_khz)
		return 0;
	else
		return 1;
}

struct cpu_emc_mapping*
tegra_cpufreq_cpu_emc_map_dt_init(struct device_node *node)
{
	struct property *prop;
	void *p;
	int len, entries;

	prop = of_find_property(node, "cpu_emc_map", &len);
	if (!prop)
		return NULL;

	/* ignore incomplete entry */
	len = rounddown(len, sizeof(struct cpu_emc_mapping));
	entries = len / sizeof(struct cpu_emc_mapping);

	/* append one zero'ed termination entry */
	p = kzalloc(len + sizeof(struct cpu_emc_mapping), GFP_KERNEL);
	if (!p)
		return NULL;

	if (of_property_read_u32_array(node, "cpu_emc_map", p,
		len / sizeof(uint32_t))) {
		kfree(p);
		return NULL;
	}

	/* sort table in ascending order */
	sort(p, entries, sizeof(struct cpu_emc_mapping), cmp_freq, NULL);

	return p;
}

unsigned long
tegra_cpu_to_emc_freq(uint32_t cpu_freq, struct cpu_emc_mapping *mapping)
{
	uint32_t emc_freq = 0;

	while (mapping->cpu_freq_khz) {
		if (cpu_freq < mapping->cpu_freq_khz)
			break;

		emc_freq = mapping->emc_freq_khz;
		mapping++;
	}

	return emc_freq;
}

static ssize_t cpu_emc_map_read(struct file *file, char __user *buf,
                size_t count, loff_t *ppos)
{
	struct cpu_emc_mapping *mapping = file->private_data;
	char *kbuf;
	int copied;
	ssize_t ret;

	if (count > PAGE_SIZE)
		kbuf = vmalloc(count);
	else
		kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	copied = snprintf(kbuf, count, "(cpufreq, emcfreq)\n");

	while (mapping->cpu_freq_khz) {
		uint32_t emc_freq_khz;

		emc_freq_khz = mapping->emc_freq_khz == UINT_MAX ?
			tegra_bwmgr_get_max_emc_rate()/1000 : mapping->emc_freq_khz;

		copied += sprintf(kbuf + copied, "%u %u\n",
			mapping->cpu_freq_khz, emc_freq_khz);
		mapping++;
	}

	ret = simple_read_from_buffer(buf, count, ppos, kbuf, copied);
	kvfree(kbuf);

	return ret;
}

static const struct file_operations cpu_emc_map_fops = {
	.open = simple_open,
	.read = cpu_emc_map_read,
	.llseek = default_llseek,
};

struct dentry *tegra_debugfs_create_cpu_emc_map(struct dentry *parent,
	struct cpu_emc_mapping *mapping)
{
	if (!parent || !mapping)
		return NULL;

	return debugfs_create_file("cpu_emc_map", S_IRUGO, parent,
		mapping, &cpu_emc_map_fops);
}
