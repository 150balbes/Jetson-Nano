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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>

#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/cpu-tegra.h>
#include <linux/platform/tegra/cpu_emc.h>

#include <soc/tegra/tegra-dvfs.h>

/* cpufreq transisition latency */
#define TEGRA_CPUFREQ_TRANSITION_LATENCY (300 * 1000)
#define IN_HZ(x) (x * 1000)
#define MAX_DVFS_FREQS	40

/*
 * Construct cpufreq scaling table, and set throttling/suspend levels.
 * Frequency table index must be sequential starting at 0 and frequencies
 * must be ascending.
 */
#define CPU_FREQ_STEP 102000 /* 102MHz cpu_g table step */
#define CPU_FREQ_TABLE_MAX_SIZE (2 * MAX_DVFS_FREQS + 1)


static struct freq_attr *tegra_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

/* cpufreq driver platform dependent data */
struct tegra_cpufreq_priv {
	struct platform_device *pdev;
	struct tegra_cpufreq_table_data tfrqtbl;
	u32 cpu_freq[CONFIG_NR_CPUS];
	struct notifier_block min_freq_notifier;
	struct notifier_block max_freq_notifier;
	struct clk *cpu_clk;
};
static struct tegra_cpufreq_priv *tfreq_priv;

static int cpu_freq_notify(struct notifier_block *b,
			   unsigned long l, void *v)
{

	u32 cpu;

	pr_debug("PM QoS %s %lu\n",
		b == &tfreq_priv->min_freq_notifier ? "min" : "max", l);

	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);

		if (policy) {
			cpufreq_update_policy(policy->cpu);
			cpufreq_cpu_put(policy);
		} else
			return -EINVAL;
	}
	return NOTIFY_OK;
}

/* Clipping policy object's min/max to pmqos limits */
static int tegra_boundaries_policy_notifier(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int qmin = 0;
	unsigned int qmax = UINT_MAX;

	if (event != CPUFREQ_ADJUST)
		return NOTIFY_OK;

	qmin = pm_qos_read_min_bound(PM_QOS_CPU_FREQ_BOUNDS);
	qmax = pm_qos_read_max_bound(PM_QOS_CPU_FREQ_BOUNDS);

	/*
	 * Clamp pmqos to stay within sysfs upper boundary
	 * but allow pmqos cap override sysfs min freq settings
	 */
	qmin = min(qmin, policy->user_policy.max);
	qmax = min(qmax, policy->user_policy.max);

	/* Apply pmqos limits on top of existing limits */
	policy->min = max(policy->min, qmin);
	policy->max = min(policy->max, qmax);

	if (policy->min > policy->max)
		policy->min = policy->max;

	return NOTIFY_OK;
}

static struct notifier_block tegra_boundaries_cpufreq_nb = {
	.notifier_call = tegra_boundaries_policy_notifier,
};

static void pm_qos_register_notifier(void)
{
	tfreq_priv->min_freq_notifier.notifier_call =
		tfreq_priv->max_freq_notifier.notifier_call =
			cpu_freq_notify;

	pm_qos_add_min_notifier(PM_QOS_CPU_FREQ_BOUNDS,
		&tfreq_priv->min_freq_notifier);
	pm_qos_add_max_notifier(PM_QOS_CPU_FREQ_BOUNDS,
		&tfreq_priv->max_freq_notifier);
}

struct device_node *of_get_scaling_node(const char *name)
{
	struct device *dev = &tfreq_priv->pdev->dev;
	struct device_node *scaling_np = NULL;
	struct device_node *np =
		of_find_compatible_node(NULL, NULL, "nvidia,tegra210-cpufreq");

	if (!np || !of_device_is_available(np)) {
		dev_info(dev, "%s: Tegra210 cpufreq node is not found\n",
			__func__);
		of_node_put(np);
		return NULL;
	}

	scaling_np = of_get_child_by_name(np, name);
	of_node_put(np);
	if (!scaling_np || !of_device_is_available(scaling_np)) {
		dev_info(dev, "%s: %s for cpufreq is not found\n",
			__func__, name);
		of_node_put(scaling_np);
		return NULL;
	}
	return scaling_np;
}

static int enable_cpu_clk(void)
{
	struct device *cpu_dev;
	struct device_node *np;
	struct clk *cpu_g, *old_parent;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -ENODEV;

	np = of_cpu_device_node_get(0);
	if (!np)
		return -ENODEV;

	tfreq_priv->cpu_clk = of_clk_get_by_name(np, "dfll");
	if (IS_ERR(tfreq_priv->cpu_clk)) {
		ret = -EPROBE_DEFER;
		goto dfll_fail;
	}

	cpu_g = of_clk_get_by_name(np, "cpu_g");
	if (IS_ERR(cpu_g)) {
		ret = PTR_ERR(cpu_g);
		goto cpu_clk_fail;
	}

	ret = clk_set_rate(tfreq_priv->cpu_clk, clk_get_rate(cpu_g));
	if (ret < 0)
		goto set_rate_fail;

	old_parent = clk_get_parent(cpu_g);

	ret = clk_set_parent(cpu_g, tfreq_priv->cpu_clk);
	if (ret)
		goto set_rate_fail;

	clk_put(cpu_g);

	return 0;
set_rate_fail:
	clk_put(cpu_g);
cpu_clk_fail:
	clk_put(tfreq_priv->cpu_clk);
dfll_fail:
	of_node_put(np);

	return ret;
}

static int cpufreq_table_make_from_dt(void)
{
	struct tegra_cpufreq_table_data *tftbl = &tfreq_priv->tfrqtbl;
	struct device *dev = &tfreq_priv->pdev->dev;
	struct cpufreq_frequency_table *ftbl;
	const char *propname = "freq-table";
	struct device_node *np = NULL;
	u32 *freqs = NULL;
	int i, j, freqs_num;
	int ret = 0;

	ftbl = devm_kzalloc(dev,
		sizeof(*ftbl) * CPU_FREQ_TABLE_MAX_SIZE, GFP_KERNEL);
	if (!ftbl)
		return -ENOMEM;

	/* Find cpufreq node */
	np = of_get_scaling_node("cpu-scaling-data");
	if (!np)
		return -ENODATA;

	/* Read frequency table */
	if (!of_find_property(np, propname, &freqs_num)) {
		dev_err(dev, "%s: %s is not found\n", __func__, propname);
		ret = -EINVAL;
		goto err_out;
	}

	if (!freqs_num) {
		dev_err(dev, "%s: invalid %s size 0\n", __func__, propname);
		ret = -EINVAL;
		goto err_out;
	}

	freqs = kzalloc(freqs_num, GFP_KERNEL);
	if (!freqs) {
		ret = -ENOMEM;
		goto err_out;
	}

	freqs_num /= sizeof(*freqs);
	if (of_property_read_u32_array(np, propname, freqs, freqs_num)) {
		dev_err(dev, "%s: failed to read %s\n", __func__, propname);
		ret = -EINVAL;
		goto err_out;
	}

	if (WARN_ON(freqs_num >= CPU_FREQ_TABLE_MAX_SIZE)) {
		ret = -EINVAL;
		goto err_out;
	}

	/* Fill in scaling table data */
	for (i = 0, j = 0; j < freqs_num; j++) {
		if (clk_round_rate(tfreq_priv->cpu_clk, freqs[j] * 1000) > 0) {
			ftbl[i].driver_data = 0;
			ftbl[i].frequency = freqs[j];
			i++;
		}
	}
	ftbl[i].driver_data = 0;
	ftbl[i].frequency = CPUFREQ_TABLE_END;
	tftbl->freq_table = ftbl;

	/* Set cpufreq suspend configuration */
	tftbl->preserve_across_suspend =
		of_property_read_bool(np, "preserve-across-suspend");

	/*
	 * Set fixed defaults for suspend and throttling indexes (not used,
	 * anyway, on Tegra21)
	 */
	tftbl->suspend_index = 0;
	tftbl->throttle_lowest_index = 0;
	tftbl->throttle_highest_index = i - 1;

err_out:
	kfree(freqs);
	of_node_put(np);
	return ret;
}

static int update_cpu_freq(u32 rate, u32 cpu)
{
	int ret = 0;

	ret = clk_set_rate(tfreq_priv->cpu_clk, IN_HZ(rate));

	return ret;
}

static int set_cpu_freq(struct cpufreq_policy *policy, unsigned int index)
{
	struct device *dev = &tfreq_priv->pdev->dev;
	struct cpufreq_frequency_table *ftbl;
	struct cpufreq_freqs freqs;
	u32 tgt_freq;
	int ret = 0;
	u32 cpu;

	if (!policy || (!cpu_online(policy->cpu)))
		return -EINVAL;

	ftbl = tfreq_priv->tfrqtbl.freq_table;

	tgt_freq = ftbl[index].frequency;
	freqs.old = tfreq_priv->cpu_freq[policy->cpu];

	if (policy->cur == tgt_freq)
		goto out;

	freqs.new = tgt_freq;

	cpufreq_freq_transition_begin(policy, &freqs);

	if (freqs.old != tgt_freq) {
		for_each_cpu(cpu, policy->cpus) {
			update_cpu_freq(tgt_freq, cpu);
			tfreq_priv->cpu_freq[cpu] = tgt_freq;
		}

		set_cpu_to_emc_freq(tgt_freq);
	}
	policy->cur = tgt_freq;

	cpufreq_freq_transition_end(policy, &freqs, ret);
out:
	dev_dbg(dev, "cpu: %d, oldfreq(kHz): %u, req freq(kHz): %u final freq(kHz):%u\n",
				policy->cpu, freqs.old, tgt_freq, policy->cur);
	return ret;
}

static unsigned int get_cpu_freq(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= CONFIG_NR_CPUS)
		return 0;

	rate = clk_get_rate(tfreq_priv->cpu_clk) / 1000;
	return rate;
}

static ssize_t table_src_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", get_cpu_emc_limit_table_source());
}

static ssize_t table_src_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int val;
	int ret =0;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	ret = set_cpu_emc_limit_table_source(val);
	if(ret)
		return ret;

	return count;
}

static struct kobj_attribute table_src_attr =
	__ATTR(table_src, 0644, table_src_show, table_src_store);

static struct kobject *tegra_cpu_emc_table_src_kobj;

static int percpu_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *ftbl = tfreq_priv->tfrqtbl.freq_table;
	u32 freq;
	int ret = 0;
	int idx;

	if (policy->cpu >= CONFIG_NR_CPUS)
		return -EINVAL;

	freq = get_cpu_freq(policy->cpu); /* boot freq */

	set_cpu_to_emc_freq(freq);

	cpufreq_table_validate_and_show(policy, ftbl);

	/* clip boot frequency to table entry */
	idx = cpufreq_frequency_table_target(policy, freq,
		CPUFREQ_RELATION_L);
	if (!ret && (freq != ftbl[idx].frequency)) {
		freq = ftbl[idx].frequency;
		ret = update_cpu_freq(freq, policy->cpu);
		if (!ret)
			freq = ftbl[idx].frequency;
	}
	policy->cur = freq;

	tfreq_priv->cpu_freq[policy->cpu] = policy->cur;

	policy->cpuinfo.transition_latency =
	TEGRA_CPUFREQ_TRANSITION_LATENCY;

	cpumask_copy(policy->cpus, cpu_possible_mask);

	tegra_cpu_emc_table_src_kobj =
		kobject_create_and_add("tegra_cpu_emc",
			kernel_kobj);

	if (!tegra_cpu_emc_table_src_kobj) {
		pr_err("%s: Couldn't create kobj\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(tegra_cpu_emc_table_src_kobj,
		&table_src_attr.attr);

	if (ret) {
		pr_err("%s, Couldn't create sysfs files\n", __func__);
		return ret;
	}

	return 0;
}

static int percpu_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, tfreq_priv->tfrqtbl.freq_table);

	return 0;
}

/* Cpufreq driver data */
static struct cpufreq_driver tegra_cpufreq_driver = {
	.name = "tegra-cpufreq",
	.flags = CPUFREQ_ASYNC_NOTIFICATION | CPUFREQ_STICKY |
				CPUFREQ_CONST_LOOPS,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_cpu_freq,
	.get = get_cpu_freq,
	.init = percpu_cpufreq_init,
	.exit = percpu_cpufreq_exit,
	.attr = tegra_cpufreq_attr,
};

static int tegra_cpufreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret = 0;

	tfreq_priv = devm_kzalloc(dev, sizeof(*tfreq_priv), GFP_KERNEL);
	if (!tfreq_priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, tfreq_priv);
	tfreq_priv->pdev = pdev;

	ret = enable_cpu_clk();
	if (ret)
		goto err_out;

	ret = cpufreq_table_make_from_dt();
	if (ret)
		goto err_out;

	ret = enable_cpu_emc_clk();
	if (ret)
		goto err_out;

	ret = cpufreq_register_driver(&tegra_cpufreq_driver);
	if (ret)
		goto err_out;

	pm_qos_register_notifier();

	cpufreq_register_notifier(&tegra_boundaries_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);

	dev_info(dev, "probe()...completed\n");
	return 0;
err_out:
	dev_err(dev, "probe()...err:%d\n", ret);
	return ret;
}

static int tegra_cpufreq_remove(struct platform_device *pdev)
{
	struct tegra_cpufreq_priv *priv = platform_get_drvdata(pdev);

	cpufreq_unregister_notifier(&tegra_boundaries_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);

	disable_cpu_emc_clk();

	platform_device_unregister(priv->pdev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tcpufreq_of_match[] = {
	{ .compatible = "nvidia,tegra210-cpufreq", .data = NULL, },
	{},
};
#endif

static struct platform_driver tegra_cpufreq_platdrv = {
	.driver	= {
		.name	= "tegra210-cpufreq",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tcpufreq_of_match),
	},
	.probe		= tegra_cpufreq_probe,
	.remove		= tegra_cpufreq_remove,
};

module_platform_driver(tegra_cpufreq_platdrv);
MODULE_AUTHOR("Puneet Saxena <puneets@nvidia.com>");
MODULE_DESCRIPTION("cpufreq platform driver for Nvidia Tegra210");
MODULE_LICENSE("GPL");
