/*
 *
 * Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/thermal.h>
#include <dt-bindings/thermal/nvidia,tegra-thermal-throttle.h>

#define KHZ_TO_HZ		1000
#define CPU_STEP_HZ		(76800 * KHZ_TO_HZ)
#define GPU_STEP_HZ		(102 * KHZ_TO_HZ * KHZ_TO_HZ)
#define MAX_CPU_CLUSTERS	4

struct tegra_throt_cdev_clk;
struct tegra_throt_cdev {
	struct thermal_cooling_device *cdev;
	struct tegra_throt_cdev_clk *clks;
	struct list_head cdev_list;
	char cdev_type[64];
	int num_clks;
	int max_state;
	int cur_state;
	int cutoff;
};

struct tegra_throt_clk {
	struct clk *clk;
	unsigned long max_rate;
	unsigned long min_rate;
	unsigned long throt_rate;
	unsigned long step_hz;
	struct list_head cdev_clk_list;
	int type;
	int steps;
	char *name;
	char **clk_names;
	int num_clk_handles;
};

struct tegra_throt_cdev_clk {
	struct tegra_throt_clk *throt_clk;
	struct tegra_throt_cdev *throt_cdev;
	struct list_head clk_list;
	int offset_hz;
	int slope_adj;
	unsigned long cdev_throt_rate;
};

static struct pm_qos_request tegra_throt_gpu_req;
static int tegra_throt_cpu_req;
static LIST_HEAD(tegra_throt_cdev_list);
static DEFINE_MUTEX(tegra_throt_lock);
static struct dentry *tegra_throt_root;
static unsigned long cpu_max_freq;
static unsigned long cpu_min_freq;

static char *cpu_clks[MAX_CPU_CLUSTERS] = { "cpu0", "cpu1", "cpu2", "cpu3" };

static char *gpu_clks[] = { "gpu" };

static struct tegra_throt_clk throt_clks[TEGRA_THROTTLE_MAX] = {
	{
		.name = "cpu",
		.clk_names = cpu_clks,
		.step_hz = CPU_STEP_HZ,
		.type = TEGRA_THROTTLE_CPU,
		.num_clk_handles = MAX_CPU_CLUSTERS,
	},
	{
		.name = "gpu",
		.clk_names = gpu_clks,
		.step_hz = GPU_STEP_HZ,
		.type = TEGRA_THROTTLE_GPU,
		.num_clk_handles = 1,
	},
};

static const struct of_device_id tegra_throt_of_match[] = {
	{
		.compatible = "nvidia,tegra-thermal-throttle",
		.data = &throt_clks,
	},
	{ },
};

static unsigned long tegra_throt_calc_rate(unsigned long idx,
				unsigned long maxf, unsigned long minf,
				unsigned long step, unsigned long offset,
				int slope_adj)
{
	unsigned long throt_rate, throt_amt;

	throt_amt = ((idx * step * 100) / slope_adj);
	throt_amt = (throt_amt > offset) ? (throt_amt - offset) : 0;
	throt_rate = (throt_amt > maxf) ? minf : (maxf - throt_amt);
	throt_rate = max(minf, min(throt_rate, maxf));

	return throt_rate;
}

static int tegra_throt_cpufreq_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;

	if (event != CPUFREQ_ADJUST)
		return 0;

	if (policy->max > tegra_throt_cpu_req)
		cpufreq_verify_within_limits(policy, 0, tegra_throt_cpu_req);

	return 0;
}

static struct notifier_block tegra_throt_cpufreq_nb = {
	.notifier_call = tegra_throt_cpufreq_notifier,
};

static struct tegra_throt_clk *tegra_throt_find_clk(
				struct tegra_throt_clk *pclks, int type)
{
	int i;

	for (i = 0; i < TEGRA_THROTTLE_MAX; i++)
		if (pclks[i].type == type)
			break;

	if (!IS_ERR(pclks[i].clk))
		return &pclks[i];

	return NULL;
}

static void tegra_throt_rate_set(int type, unsigned long rate)
{
	int cpu;

	rate /= KHZ_TO_HZ;
	switch (type) {
	case TEGRA_THROTTLE_CPU:
		for_each_present_cpu(cpu) {
			tegra_throt_cpu_req = rate;
			cpufreq_update_policy(cpu);
		}
		break;
	case TEGRA_THROTTLE_GPU:
		pm_qos_update_request(&tegra_throt_gpu_req, rate);
		break;
	default:
		pr_err("tegra_throt: incorrect type: %d\n", type);
		break;
	}
}

static int tegra_throt_set_cur_state(struct thermal_cooling_device *cdev,
					unsigned long cur_state)
{
	int i;
	struct tegra_throt_cdev *tcd = (struct tegra_throt_cdev *)cdev->devdata;

	tcd->cur_state = cur_state;
	for (i = 0; i < tcd->num_clks; i++) {
		unsigned long throt_rate = UINT_MAX;
		struct tegra_throt_cdev_clk *pos, *tcc = &tcd->clks[i];
		struct tegra_throt_clk *tclk = tcc->throt_clk;

		tcc->cdev_throt_rate = tegra_throt_calc_rate(cur_state,
					tclk->max_rate, tclk->min_rate,
					tclk->step_hz, tcc->offset_hz,
					tcc->slope_adj);

		list_for_each_entry(pos, &tclk->cdev_clk_list, clk_list)
			throt_rate = min(throt_rate, pos->cdev_throt_rate);

		/*
		 * Thermal framework takes only per cdev lock before calling
		 * here. Need to protect throt_rate which is per clock.
		 */
		mutex_lock(&tegra_throt_lock);
		if (throt_rate != tclk->throt_rate) {
			tegra_throt_rate_set(tclk->type, throt_rate);
			tclk->throt_rate = throt_rate;
			dev_dbg(&cdev->device, "type:%d throt_rate: %ldKHz\n",
					tclk->type, throt_rate/KHZ_TO_HZ);
		}
		mutex_unlock(&tegra_throt_lock);
	}

	return 0;
}

static int tegra_throt_get_cur_state(struct thermal_cooling_device *cdev,
					unsigned long *cur_state)
{
	struct tegra_throt_cdev *tcd = (struct tegra_throt_cdev *)cdev->devdata;
	*cur_state = tcd->cur_state;
	return 0;
}

static int tegra_throt_get_max_state(struct thermal_cooling_device *cdev,
					unsigned long *max_state)
{
	struct tegra_throt_cdev *tcd = (struct tegra_throt_cdev *)cdev->devdata;
	*max_state = tcd->max_state;
	return 0;
}

static const struct thermal_cooling_device_ops tegra_throt_ops = {
	.get_max_state = tegra_throt_get_max_state,
	.get_cur_state = tegra_throt_get_cur_state,
	.set_cur_state = tegra_throt_set_cur_state,
};

static int tegra_throt_calc_max_state(struct tegra_throt_cdev *tcd)
{
	int i, steps, max_state = 0;

	for (i = 0; i < tcd->num_clks; i++) {
		struct tegra_throt_cdev_clk *tcc = &tcd->clks[i];

		steps = (((tcc->offset_hz / tcc->throt_clk->step_hz) +
				tcc->throt_clk->steps) * tcc->slope_adj) / 100;
		if (tcd->cutoff)
			max_state = (max_state) ? min(steps, max_state) : steps;
		else
			max_state = max(steps, max_state);
	}

	return max_state;
}

#ifdef CONFIG_DEBUG_FS
static int cdev_freq_table_show(struct seq_file *s, void *data)
{
	int i, j;
	unsigned long throt_rate;
	struct tegra_throt_cdev *tcd = s->private;
	struct tegra_throt_cdev_clk *tcc;

	for (i = 0; i <= tcd->max_state; i++) {
		for (j = 0; j < tcd->num_clks; j++) {
			tcc = &tcd->clks[j];
			throt_rate = tegra_throt_calc_rate(i,
					tcc->throt_clk->max_rate,
					tcc->throt_clk->min_rate,
					tcc->throt_clk->step_hz, tcc->offset_hz,
					tcc->slope_adj);
			seq_printf(s, " %7lu", throt_rate);
		}
		seq_puts(s, "\n");
	}

	return 0;
}

static int table_open(struct inode *inode, struct file *file)
{
	return single_open(file, cdev_freq_table_show, inode->i_private);
}

static const struct file_operations ftable_fops = {
	.open		= table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int prop_get(void *data, u64 *val)
{
	*val = *((int *)data);
	return 0;
}

static int cutoff_set(void *data, u64 val)
{
	int *prop = (int *)data;
	struct tegra_throt_cdev *tcd = container_of(prop,
					struct tegra_throt_cdev, cutoff);
	*prop = (int)val;
	tcd->max_state = tegra_throt_calc_max_state(tcd);

	return 0;
}

static int offset_set(void *data, u64 val)
{
	int *prop = (int *)data;
	struct tegra_throt_cdev_clk *tcc = container_of(prop,
						struct tegra_throt_cdev_clk,
						offset_hz);
	*prop = val;
	tcc->throt_cdev->max_state = tegra_throt_calc_max_state(
							tcc->throt_cdev);
	return 0;
}


static int slope_set(void *data, u64 val)
{
	int *prop = (int *)data;
	struct tegra_throt_cdev_clk *tcc = container_of(prop,
						struct tegra_throt_cdev_clk,
						slope_adj);
	*prop = val;
	tcc->throt_cdev->max_state = tegra_throt_calc_max_state(
							tcc->throt_cdev);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cfops, prop_get, cutoff_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(ofops, prop_get, offset_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(sfops, prop_get, slope_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(tfops, prop_get, NULL, "%lld\n");

static void tegra_throt_dbgfs_remove(struct dentry *root)
{
	debugfs_remove_recursive(root);
}

static int tegra_throt_dbgfs_create(struct dentry **root)
{
	*root = debugfs_create_dir("tegra_throttle", NULL);
	return (!root) ? -ENODEV : 0;
}

static void tegra_throt_dbgfs_init(struct platform_device *pdev,
		struct tegra_throt_cdev *tcd, struct dentry *throt_root)
{
	int i;
	struct dentry *r, *cr, *f;

	if (!tcd)
		goto err;

	r = debugfs_create_dir(tcd->cdev_type, throt_root);
	f = r ? debugfs_create_file("cutoff", 0644, r, &tcd->cutoff, &cfops) :
		r;
	f = f ? debugfs_create_file("table", 0644, r, tcd, &ftable_fops) : f;
	if (!f)
		goto err;

	for (i = 0; i < tcd->num_clks; i++) {
		struct tegra_throt_cdev_clk *tcc = &tcd->clks[i];

		cr = debugfs_create_dir(tcc->throt_clk->name, r);
		if (!cr)
			goto err;

		f = debugfs_create_file("offset", 0644, cr, &tcc->offset_hz,
					&ofops);
		f = f ? debugfs_create_file("slope-adj", 0644, cr,
					&tcc->slope_adj, &sfops) : f;
		f = f ? debugfs_create_file("throt-rate", 0644, cr,
					&tcc->cdev_throt_rate, &tfops) : f;
		if (!f)
			goto err;
	}

	return;
err:
	dev_err(&pdev->dev, "failed to initialize debugfs\n");
}

#else /* CONFIG_DEBUG_FS */
static void tegra_throt_dbgfs_init(struct platform_device *pdev,
		struct tegra_throt_cdev *tcd, struct dentry *root)
{}

static void tegra_throt_dbgfs_remove(struct dentry *root)
{}

static int tegra_throt_dbgfs_create(struct dentry **root)
{
	return 0;
}
#endif

static int tegra_throt_cdev_clk_init(struct platform_device *pdev,
			struct tegra_throt_clk *pclks, struct device_node *np,
			struct tegra_throt_cdev *tcd)
{
	int i, j, ret = 0, n;
	u32 prop[3 * TEGRA_THROTTLE_MAX];
	struct tegra_throt_cdev_clk *tcc;
	struct tegra_throt_clk *tclk;


	n = of_property_count_u32_elems(np, "nvidia,throttle-clocks");
	if (n <= 0)
		return -ENODEV;

	if ((n % 3) != 0)
		return -EINVAL;

	ret = of_property_read_u32_array(np, "nvidia,throttle-clocks", prop, n);
	if (ret)
		return -EINVAL;

	tcd->num_clks = n / 3;
	tcd->clks = devm_kzalloc(&pdev->dev,
			(tcd->num_clks * sizeof(struct tegra_throt_cdev_clk)),
			GFP_KERNEL);
	if (IS_ERR_OR_NULL(tcd->clks))
		return -ENOMEM;

	for (i = 0, j = 0; j < tcd->num_clks; i = i + 3, j++) {
		tcc = &tcd->clks[j];
		tclk = tegra_throt_find_clk(pclks, prop[i]);
		if (!tclk)
			return -ENODEV;

		tcc->throt_clk = tclk;
		tcc->slope_adj = prop[i+1];
		if (!tcc->slope_adj)
			return -EINVAL;

		tcc->offset_hz = prop[i+2];
		dev_info(&pdev->dev, "cdev:%s clk:%d:%s off:%d slope-adj:%d\n",
				tcd->cdev_type, tcc->throt_clk->type,
				tcc->throt_clk->name, tcc->offset_hz,
				tcc->slope_adj);
		tcc->cdev_throt_rate = UINT_MAX;
		tcc->throt_cdev = tcd;
		list_add(&tcc->clk_list, &tclk->cdev_clk_list);
	}

	return 0;
}

static int tegra_throt_cdev_init(struct platform_device *pdev,
				struct tegra_throt_clk *pclks)
{
	const char *str;
	int val, cnt = 0;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ch;
	struct tegra_throt_cdev *tcd = NULL;

	if (!np)
		return -ENODEV;

	if (tegra_throt_dbgfs_create(&tegra_throt_root))
		return -ENODEV;

	for_each_child_of_node(np, ch) {
		if (!of_device_is_available(ch))
			continue;

		if (of_property_read_string(ch, "cdev-type", &str))
			continue;

		tcd = devm_kzalloc(&pdev->dev, sizeof(struct tegra_throt_cdev),
					GFP_KERNEL);
		if (IS_ERR_OR_NULL(tcd))
			return -ENOMEM;

		strlcpy(tcd->cdev_type, str, sizeof(tcd->cdev_type));
		val = tegra_throt_cdev_clk_init(pdev, pclks, ch, tcd);
		if (val) {
			dev_err(&pdev->dev, "cdev:%s clk init failed: %0x0x\n",
					tcd->cdev_type, val);
			devm_kfree(&pdev->dev, tcd);
			continue;
		}

		if (!of_property_read_u32(ch, "nvidia,cutoff", &val))
			tcd->cutoff = (val) ? 1 : 0;

		tcd->max_state = tegra_throt_calc_max_state(tcd);
		tcd->cdev = thermal_of_cooling_device_register(ch,
				tcd->cdev_type, tcd, &tegra_throt_ops);
		if (IS_ERR(tcd->cdev)) {
			devm_kfree(&pdev->dev, tcd);
			continue;
		}

		cnt++;
		list_add(&tcd->cdev_list, &tegra_throt_cdev_list);
		dev_info(&pdev->dev, "cdev:%s max_state:%d cutoff:%d\n",
				tcd->cdev_type, tcd->max_state, tcd->cutoff);
		tegra_throt_dbgfs_init(pdev, tcd, tegra_throt_root);
	}

	if (!cnt)
		dev_err(&pdev->dev, "Missing cooling devices\n");

	return (cnt) ? 0 : -ENODEV;
}

static int tegra_throt_freq_gov_init(int type)
{
	int ret = -EINVAL;

	switch (type) {
	case TEGRA_THROTTLE_CPU:
		tegra_throt_cpu_req = UINT_MAX;
		ret = cpufreq_register_notifier(&tegra_throt_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);
		if (ret)
			pr_info("tegra_throt: missing cpufreq: 0x%x\n", ret);

		break;
	case TEGRA_THROTTLE_GPU:
		pm_qos_add_request(&tegra_throt_gpu_req, PM_QOS_GPU_FREQ_MAX,
				PM_QOS_GPU_FREQ_MAX_DEFAULT_VALUE);
		ret = 0;
		break;
	}

	return ret;
}

static int tegra_throt_clk_get(struct platform_device *pdev,
				struct tegra_throt_clk *pclk)
{
	int i;
	int ret = -EINVAL;

	for (i = 0; i < pclk->num_clk_handles; i++) {
		if (!strncmp(pclk->clk_names[i], "cpu", 3)) {
			/* non CCF controlled clock */
			pclk->max_rate = cpu_max_freq * KHZ_TO_HZ;
			pclk->min_rate = cpu_min_freq * KHZ_TO_HZ;
			ret = 0;
		} else {
			/* CCF controlled clock */
			pclk->clk = devm_clk_get(&pdev->dev,
						pclk->clk_names[i]);
			if (!IS_ERR(pclk->clk)) {
				pclk->max_rate = clk_round_rate(pclk->clk,
								UINT_MAX);
				pclk->min_rate = clk_round_rate(pclk->clk, 0);
				ret = 0;
			}
		}
	}

	if (ret)
		dev_err(&pdev->dev, "failed to get clk:%s\n", pclk[i].name);

	return ret;
}

static int tegra_throt_clk_init(struct platform_device *pdev,
				struct tegra_throt_clk *pclks)
{
	int i, ret = -ENODEV;

	for (i = 0; i < TEGRA_THROTTLE_MAX; i++) {
		ret = tegra_throt_freq_gov_init(pclks[i].type);
		if (ret) {
			dev_err(&pdev->dev, "Missing frequency governors\n");
			continue;
		}

		if (tegra_throt_clk_get(pdev, &pclks[i]) != 0)
			continue;

		INIT_LIST_HEAD(&pclks[i].cdev_clk_list);
		pclks[i].steps = DIV_ROUND_UP((pclks[i].max_rate -
					pclks[i].min_rate), pclks[i].step_hz);
		pclks[i].throt_rate = UINT_MAX;
		dev_info(&pdev->dev, "clk:%s max:%ld, min:%ld steps:%d\n",
				pclks[i].name, pclks[i].max_rate,
				pclks[i].min_rate, pclks[i].steps);
		ret = 0;
	}

	if (ret)
		dev_err(&pdev->dev, "missing clocks\n");

	return ret;
}

static int tegra_throt_cpu_min_max(void)
{
	struct cpufreq_policy policy;
	int cpu;
	int cnt = 0;

	cpu_max_freq = 0;
	cpu_min_freq = UINT_MAX;
	/* In cases when cpufreq_single_policy is not set,  the per cpu max/min
	 * could differ. In such cases, choose the highest max frequency and
	 * lowest min frequency to cover all frequency points. Throttling slope
	 * could be increased if this is deemed too slow.
	 */
	for_each_online_cpu(cpu) {
		if (!cpufreq_get_policy(&policy, cpu)) {
			if (policy.cpuinfo.max_freq > cpu_max_freq)
				cpu_max_freq = policy.cpuinfo.max_freq;
			if (policy.cpuinfo.min_freq < cpu_min_freq)
				cpu_min_freq = policy.cpuinfo.min_freq;
			cnt++;
		}
	}

	/* continue with probe as long as one cpu is online */
	return (cnt) ? 0 : -ENODEV;
}

static int tegra_throt_remove(struct platform_device *pdev)
{
	struct tegra_throt_cdev *pos;

	dev_info(&pdev->dev, "remove\n");
	cpufreq_unregister_notifier(&tegra_throt_cpufreq_nb,
					CPUFREQ_POLICY_NOTIFIER);
	pm_qos_remove_request(&tegra_throt_gpu_req);
	tegra_throt_dbgfs_remove(tegra_throt_root);
	list_for_each_entry(pos, &tegra_throt_cdev_list, cdev_list)
		thermal_cooling_device_unregister(pos->cdev);

	return 0;
}

static int tegra_throt_probe(struct platform_device *pdev)
{
	int ret = 0;
	const struct of_device_id *match;
	struct tegra_throt_clk *pclks;

	ret = tegra_throt_cpu_min_max();
	if (ret != 0) {
		dev_info(&pdev->dev, "cpufreq policy is not ready defer\n");
		return -EPROBE_DEFER;
	}

	match = of_match_node(tegra_throt_of_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;

	pclks = (struct tegra_throt_clk *)match->data;
	if (!pclks)
		return -EINVAL;

	ret = tegra_throt_clk_init(pdev, pclks);
	if (ret)
		return ret;

	ret = tegra_throt_cdev_init(pdev, pclks);
	if (ret)
		tegra_throt_remove(pdev);

	return ret;
}

static struct platform_driver tegra_throttle_driver = {
	.driver = {
		.name = "tegra-thermal-throttle",
		.owner = THIS_MODULE,
		.of_match_table = tegra_throt_of_match,
	},
	.probe = tegra_throt_probe,
	.remove = tegra_throt_remove,
};

module_platform_driver(tegra_throttle_driver);
MODULE_AUTHOR("Srikar Srimath Tirumala <srikars@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Clock Thermal Throttle Driver");
MODULE_LICENSE("GPL v2");
