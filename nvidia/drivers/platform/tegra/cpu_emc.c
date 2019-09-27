/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/cpu_emc.h>

#define CPU_EMC_TABLE_SRC_DT		1
#define CPU_EMC_TABLE_SRC_DEFAULT	0

static int cpu_emc_table_src;

struct cpu_emc {
	struct tegra_bwmgr_client *bwmgr;
	unsigned long max_rate;
	u32 *cpu_emc_table;
	int cpu_emc_table_size;
};

static struct cpu_emc cpemc;

static unsigned long default_emc_cpu_limit(unsigned long cpu_rate,
					   unsigned long emc_max_rate)
{
	if (cpu_rate >= 1300000)
		return emc_max_rate;	/* cpu >= 1.3GHz, emc max */
	else if (cpu_rate >= 975000)
		return 400000000;	/* cpu >= 975 MHz, emc 400 MHz */
	else if (cpu_rate >= 725000)
		return	200000000;	/* cpu >= 725 MHz, emc 200 MHz */
	else if (cpu_rate >= 500000)
		return	100000000;	/* cpu >= 500 MHz, emc 100 MHz */
	else if (cpu_rate >= 275000)
		return	50000000;	/* cpu >= 275 MHz, emc 50 MHz */
	else
		return 0;		/* emc min */
}

void set_cpu_to_emc_freq(u32 cpu_freq)
{
	unsigned long emc_freq = 0;
	int i;

	if(cpu_emc_table_src == CPU_EMC_TABLE_SRC_DEFAULT)
		emc_freq = default_emc_cpu_limit(cpu_freq, cpemc.max_rate);
	else{
		for (i = 0; i < cpemc.cpu_emc_table_size; i += 2) {
			if (cpu_freq < cpemc.cpu_emc_table[i])
				break;
		}

		if (i)
			emc_freq = min(cpemc.max_rate,
						cpemc.cpu_emc_table[i-1] * 1000UL);
	}

	tegra_bwmgr_set_emc(cpemc.bwmgr, emc_freq,
		TEGRA_BWMGR_SET_EMC_FLOOR);

	pr_debug("cpu freq(kHz):%u emc_freq(KHz) %lu\n", cpu_freq,
		emc_freq / 1000);
}

int set_cpu_emc_limit_table_source(int table_src)
{
	if (table_src != CPU_EMC_TABLE_SRC_DT &&
		table_src != CPU_EMC_TABLE_SRC_DEFAULT)
			return -1;

	cpu_emc_table_src = table_src;

	return 0;
}

int get_cpu_emc_limit_table_source(void)
{
	return cpu_emc_table_src;
}

static int register_with_emc_bwmgr(void)
{
	struct tegra_bwmgr_client *bwmgr;
	int ret = 0;

	bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_CPU_CLUSTER_0);
	if (IS_ERR_OR_NULL(bwmgr)) {
		pr_err("emc bw manager registration failed\n");
		ret = -ENODEV;
		goto err_out;
	}
	cpemc.bwmgr = bwmgr;
err_out:
	return ret;
}

static u32 *cpufreq_emc_table_get(int *table_size)
{
	int freqs_num, ret = 0;
	u32 *freqs = NULL;
	struct device_node *np = NULL;
	const char *propname = "emc-cpu-limit-table";

	/* Find emc scaling node */
	np = of_get_scaling_node("emc-scaling-data");
	if (!np)
		return ERR_PTR(-ENODATA);

	/* Read frequency table */
	if (!of_find_property(np, propname, &freqs_num)) {
		pr_err("%s: %s is not found\n", __func__, propname);
		ret = -ENODATA;
		goto _out;
	}

	/* must have even entries */
	if (!freqs_num || (freqs_num % (sizeof(*freqs) * 2))) {
		pr_err("%s: invalid %s size %d\n", __func__, propname,
			freqs_num);
		ret = -ENODATA;
		goto _out;
	}

	freqs = kzalloc(freqs_num, GFP_KERNEL);
	if (!freqs) {
		ret = -ENOMEM;
		goto _out;
	}

	freqs_num /= sizeof(*freqs);
	if (of_property_read_u32_array(np, propname, freqs, freqs_num)) {
		pr_err("%s: failed to read %s\n", __func__, propname);
		ret = -ENODATA;
		goto _out;
	}

	of_node_put(np);
	*table_size = freqs_num;
	return freqs;

_out:
	kfree(freqs);
	of_node_put(np);
	return ERR_PTR(ret);
}

static int cpu_emc_tbl_from_dt(void)
{
	int ret = 0;

	cpemc.cpu_emc_table =
		cpufreq_emc_table_get(&cpemc.cpu_emc_table_size);
	if (IS_ERR(cpemc.cpu_emc_table)) {
		ret = PTR_ERR(cpemc.cpu_emc_table);
		goto err_out;
	}

	cpemc.max_rate = tegra_bwmgr_get_max_emc_rate();
	set_cpu_emc_limit_table_source(CPU_EMC_TABLE_SRC_DT);

	return 0;
err_out:
	return ret;
}

int enable_cpu_emc_clk(void)
{
	int ret = 0;

	ret = register_with_emc_bwmgr();
	if (ret)
		goto err_out;

	ret = cpu_emc_tbl_from_dt();
	if (ret) {
		tegra_bwmgr_unregister(cpemc.bwmgr);
		goto err_out;
	}

err_out:
	return ret;
}

void disable_cpu_emc_clk(void)
{
	kfree(cpemc.cpu_emc_table);
	tegra_bwmgr_unregister(cpemc.bwmgr);
}
