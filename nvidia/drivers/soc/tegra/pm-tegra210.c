/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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
#ifdef CONFIG_PM
#include <linux/kernel.h>
#include <linux/cpu.h>
#include <linux/cpu_pm.h>
#include <linux/cpuidle.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/tick.h>
#include <linux/version.h>
#include <soc/tegra/bpmp_t210_abi.h>
#include <soc/tegra/flowctrl.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_bpmp.h>

#include <asm/cpuidle.h>

#include "../../kernel/irq/internals.h"
#include "../../kernel/time/tick-internal.h"

enum tegra210_idle_index {
	C7_IDX = 0,
	CC6_IDX,
	CC7_IDX,
	IDLE_STATE_MAX,
};

struct tegra210_pm_data {
	bool cc4_no_retention;
	bool cc3_no_hvc;
	bool cc6_allow;
	/* Idle state index array */
	int idle_state_idx[IDLE_STATE_MAX];
};
static struct tegra210_pm_data t210_pm_data;

#ifdef CONFIG_PREEMPT_RT_FULL
static DEFINE_RAW_SPINLOCK(tegra210_cpu_pm_lock);
#else
static DEFINE_RWLOCK(tegra210_cpu_pm_lock);
#endif

static RAW_NOTIFIER_HEAD(tegra210_cpu_pm_chain);

static void tegra210_bpmp_enable_suspend(int mode, int flags)
{
	s32 mb[] = { cpu_to_le32(mode), cpu_to_le32(flags) };

	tegra_bpmp_send_receive_atomic(MRQ_ENABLE_SUSPEND,
				       &mb, sizeof(mb), NULL, 0);
}

static int tegra210_bpmp_suspend(void)
{
	tegra210_bpmp_enable_suspend(TEGRA_PM_SC7, 0);

	return 0;
}

static struct syscore_ops bpmp_sc7_suspend_ops = {
	.suspend = tegra210_bpmp_suspend,
	.save = tegra210_bpmp_suspend,
};

static int tegra_of_idle_state_idx_from_name(char *state_name)
{
	struct device_node *state_node, *cpu_node;
	int i, id_found = 0;

	cpu_node = of_cpu_device_node_get(0);

	for (i = 0; ; i++) {
		state_node = of_parse_phandle(cpu_node, "cpu-idle-states", i);
		if (!state_node)
			break;

		if (!of_device_is_available(state_node))
			continue;

		if (!strcmp(state_node->name, state_name))
			id_found = 1;

		of_node_put(state_node);

		if (id_found)
			break;
	}

	of_node_put(cpu_node);

	return id_found ? (i + 1) : 0;
}

static int proc_idle_state_enter(int cpu, int idle_state)
{
	u32 ctrl;
	ctrl = 0xffffffff;

	if (t210_pm_data.cc4_no_retention)
		ctrl &= ~BIT(1);

	/* We don't allow retention without HVC */
	if (t210_pm_data.cc3_no_hvc)
		ctrl &= ~(BIT(1) | BIT(0));

	flowctrl_write_cc4_ctrl(cpu, ctrl);

	if (idle_state == t210_pm_data.idle_state_idx[CC6_IDX] &&
			!t210_pm_data.cc6_allow)
		return -ENODEV;

	return 0;
}

static void proc_idle_state_exit(int cpu, int idle_state)
{
	flowctrl_write_cc4_ctrl(cpu, 0);
}

static int tegra210_cpu_pm_notifier(struct notifier_block *self,
				    unsigned long cmd, void *v)
{
	int idle_state = (long)v;
	int cpu = smp_processor_id();

	switch (cmd) {
	case CPU_PM_ENTER:
		if (proc_idle_state_enter(cpu, idle_state))
			return NOTIFY_BAD;
		break;
	case CPU_PM_EXIT:
		proc_idle_state_exit(cpu, idle_state);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block tegra210_cpu_pm_nb = {
	.notifier_call = tegra210_cpu_pm_notifier,
};

static int tegra210_cpu_pm_notify(enum cpu_pm_event event, void *v,
				  int nr_to_call, int *nr_calls)
{
	int ret;

	ret = __raw_notifier_call_chain(&tegra210_cpu_pm_chain, event, v,
					nr_to_call, nr_calls);

	return notifier_to_errno(ret);
}

int tegra210_cpu_pm_enter(void *idle_idx)
{
	int ret = 0;

#ifdef CONFIG_PREEMPT_RT_FULL
	unsigned long flags;
	raw_spin_lock_irqsave(&tegra210_cpu_pm_lock, flags);
#else
	read_lock(&tegra210_cpu_pm_lock);
#endif
	ret = tegra210_cpu_pm_notify(CPU_PM_ENTER, idle_idx, -1, NULL);
#ifdef CONFIG_PREEMPT_RT_FULL
	raw_spin_unlock_irqrestore(&tegra210_cpu_pm_lock, flags);
#else
	read_unlock(&tegra210_cpu_pm_lock);
#endif

	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_cpu_pm_enter);

int tegra210_cpu_pm_exit(void *idle_idx)
{
	int ret;

#ifdef CONFIG_PREEMPT_RT_FULL
	unsigned long flags;
	raw_spin_lock_irqsave(&tegra210_cpu_pm_lock, flags);
#else
	read_lock(&tegra210_cpu_pm_lock);
#endif
	ret = tegra210_cpu_pm_notify(CPU_PM_EXIT, idle_idx, -1, NULL);
#ifdef CONFIG_PREEMPT_RT_FULL
	raw_spin_unlock_irqrestore(&tegra210_cpu_pm_lock, flags);
#else
	read_unlock(&tegra210_cpu_pm_lock);
#endif

	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_cpu_pm_exit);

static int tegra210_cpu_pm_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

#ifdef CONFIG_PREEMPT_RT_FULL
	raw_spin_lock_irqsave(&tegra210_cpu_pm_lock, flags);
#else
	write_lock_irqsave(&tegra210_cpu_pm_lock, flags);
#endif
	ret = raw_notifier_chain_register(&tegra210_cpu_pm_chain, nb);
#ifdef CONFIG_PREEMPT_RT_FULL
	raw_spin_unlock_irqrestore(&tegra210_cpu_pm_lock, flags);
#else
	write_unlock_irqrestore(&tegra210_cpu_pm_lock, flags);
#endif

	return ret;
}

static void do_cc4_init(void)
{
	flowctrl_update(FLOW_CTLR_CC4_HVC_CONTROL,
			2 << 3 | FLOW_CTRL_CC4_HVC_ENABLE);
	flowctrl_update(FLOW_CTRL_CC4_RETENTION_CONTROL, 2 << 3);
	flowctrl_update(FLOW_CTRL_CC4_HVC_RETRY, 2);
}

static struct syscore_ops cc4_syscore_ops = {
	.restore = do_cc4_init,
	.resume = do_cc4_init
};

static int tegra210_cpuidle_cc4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator *reg;
	uint32_t uv;
	int r;

	do_cc4_init();

	/* T210 BPMP supports CC4 retention only with max77621 or ovr2. */
	t210_pm_data.cc4_no_retention = of_property_read_bool(dev->of_node,
							"cc4-no-retention");

	t210_pm_data.cc3_no_hvc = of_property_read_bool(dev->of_node,
							"cc3-no-hvc");

	/* If cc4-microvolt is not found, assume not max77621 */
	if (of_property_read_u32(dev->of_node, "cc4-microvolt", &uv))
		goto out;

	reg = regulator_get(dev, "vdd-cpu");
	if (IS_ERR(reg)) {
		dev_err(dev, "vdd-cpu regulator get failed\n");
		return PTR_ERR(reg);
	}

	r = regulator_set_sleep_voltage(reg, uv - 100000, uv + 100000);
	if (r)
		dev_err(dev, "failed to set retention voltage: %d\n", r);

	dev_info(dev, "retention voltage is %u uv\n", uv);
out:
	register_syscore_ops(&cc4_syscore_ops);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static struct dentry *cpuidle_debugfs_root;
static unsigned int state_enable;
static u64 idle_state;

static int fast_enable_show(struct seq_file *s, void *data)
{
	struct cpuidle_device *dev = __this_cpu_read(cpuidle_devices);
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	int i;

	if (!drv) {
		seq_puts(s, "Failed to get cpuidle driver\n");
		return 0;
	}

	seq_puts(s, "Usage: write state index to disable or enable\n");
	seq_puts(s, "  name	idx	Enabled\n");
	seq_puts(s, "--------------------------\n");

	for (i = drv->safe_state_index + 1; i < drv->state_count; i++) {
		seq_printf(s, "  %s	%d	%d\n", drv->states[i].name,
				i, !drv->states[i].disabled);
	}

	return 0;
}

static int fast_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, fast_enable_show, inode->i_private);
}

static ssize_t fast_enable_write(struct file *fp, const char __user *ubuf,
				 size_t count, loff_t *pos)
{
	int cpu;
	struct cpuidle_device *dev;
	struct cpuidle_driver *drv;

	if (kstrtouint_from_user(ubuf, count, 0, &state_enable) < 0)
		return -EINVAL;

	for_each_present_cpu(cpu) {
		dev = per_cpu(cpuidle_devices, cpu);
		drv = cpuidle_get_cpu_driver(dev);

		if (!drv) {
			pr_err("%s: Failed to get cpuidle driver on cpu:%d\n", __func__, cpu);
			return -ENOTSUPP;
		}

		if (state_enable <= drv->safe_state_index ||
				state_enable >= drv->state_count)
			return -EINVAL;

		if (drv->states[state_enable].disabled)
			drv->states[state_enable].disabled = false;
		else
			drv->states[state_enable].disabled = true;
	}

	return count;
}

static const struct file_operations fast_cluster_enable_fops = {
	.open   =       fast_enable_open,
	.read   =       seq_read,
	.llseek =       seq_lseek,
	.write  =       fast_enable_write,
	.release =      single_release,
};

static bool is_timer_irq(struct irq_desc *desc)
{
	return desc && desc->action && (desc->action->flags & IRQF_TIMER);
}

static void suspend_all_device_irqs(void)
{
	struct irq_desc *desc;
	int irq;

	for_each_irq_desc(irq, desc) {
		unsigned long flags;

		/* Don't disable the 'wakeup' interrupt */
		if (is_timer_irq(desc))
			continue;

		raw_spin_lock_irqsave(&desc->lock, flags);
		__disable_irq(desc);
		desc->istate |= IRQS_SUSPENDED;
		raw_spin_unlock_irqrestore(&desc->lock, flags);

		synchronize_hardirq(irq);
	}
}

static void resume_all_device_irqs(void)
{
        struct irq_desc *desc;
        int irq;

        for_each_irq_desc(irq, desc) {
                unsigned long flags;

		/* No need to re-enable the 'wakeup' interrupt */
		if (is_timer_irq(desc))
			continue;

                raw_spin_lock_irqsave(&desc->lock, flags);
		desc->istate &= ~IRQS_SUSPENDED;
                __enable_irq(desc);
                raw_spin_unlock_irqrestore(&desc->lock, flags);
        }

}
static int idle_write(void *data, u64 val)
{
	struct cpuidle_driver *drv;
	unsigned long timer_interval_us = (ulong)val;
	ktime_t time, interval, sleep;

	preempt_disable();
	drv = cpuidle_get_driver();

	if ((idle_state != t210_pm_data.idle_state_idx[C7_IDX]) &&
	    (idle_state != t210_pm_data.idle_state_idx[CC6_IDX])) {
		pr_err("%s: Request forced idle state C7/CC4: 1, CC6:2\n", __func__);
		preempt_enable_no_resched();
		return -EINVAL;
	}
	pr_info("CPU idle in state: %llu, duration: %lu us\n", idle_state, timer_interval_us);

	suspend_all_device_irqs();
	tick_nohz_idle_enter();
	stop_critical_timings();
	local_fiq_disable();
	local_irq_disable();

	interval = ktime_set(0, (NSEC_PER_USEC * timer_interval_us));

	time = ktime_get();
	sleep = ktime_add(time, interval);
	tick_program_event(sleep, true);

	arm_cpuidle_suspend(idle_state);

	sleep = ktime_sub(ktime_get(), time);
	time = ktime_sub(sleep, interval);
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 10, 0)
	pr_debug("idle: %lld, exit latency: %lld\n", sleep.tv64, time.tv64);
#else
	pr_debug("idle: %lld, exit latency: %lld\n", sleep, time);
#endif

	local_irq_enable();
	local_fiq_enable();
	start_critical_timings();
	tick_nohz_idle_exit();
	resume_all_device_irqs();
	preempt_enable_no_resched();

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(duration_us_fops, NULL, idle_write, "%llu\n");

static int debugfs_init(void)
{
	struct dentry *dfs_file;

	cpuidle_debugfs_root = debugfs_create_dir("cpuidle_t210", NULL);

	if (!cpuidle_debugfs_root) {
		pr_err("failed to create cpuidle_t210 node\n");
		return -ENOMEM;
	}

	dfs_file = debugfs_create_file("fast_cluster_states_enable", 0644,
			cpuidle_debugfs_root, NULL, &fast_cluster_enable_fops);
	if (!dfs_file) {
		pr_err("failed to create ast_cluster_states_enable\n");
		goto err_out;
	}

	dfs_file = debugfs_create_u64("forced_idle_state", 0644,
				      cpuidle_debugfs_root, &idle_state);

	if (!dfs_file) {
		pr_err("failed to create forced_idle_state\n");
		goto err_out;
	}


	dfs_file = debugfs_create_file("forced_idle_duration_us", 0200,
				cpuidle_debugfs_root, NULL, &duration_us_fops);

	if (!dfs_file) {
		pr_err("failed to create forced_idle_duration_us\n");
		goto err_out;
	}

	return 0;

err_out:
	debugfs_remove_recursive(cpuidle_debugfs_root);
	return -ENOMEM;
}
#else
static inline int debugfs_init(void) { return 0; }
#endif

static const struct of_device_id tegra210_cpuidle_of[] = {
	{ .compatible = "nvidia,tegra210-cpuidle" },
	{}
};

static struct platform_driver tegra210_cpuidle_driver = {
	.probe = tegra210_cpuidle_cc4_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = "cpuidle-tegra210",
		.of_match_table = tegra210_cpuidle_of,
	}
};

static int __init tegra210_cpuidle_init(void)
{
	struct cpuidle_device *dev = __this_cpu_read(cpuidle_devices);
	struct cpuidle_driver *drv = cpuidle_get_cpu_driver(dev);
	int i;

	if (tegra_get_chip_id() != TEGRA210)
		goto out;

	if (!dev || !drv) {
		pr_err("%s: no cpuidle devices or driver\n", __func__);
		return -ENODEV;
	}

	/*
	 * To avoid the race condition between DFLL clock ready
	 * and CC4 engagement. Put this in late_inticall.
	 */
	platform_driver_register(&tegra210_cpuidle_driver);

	debugfs_init();

	/*
	 * Disable CC6 during boot. They can be enabled later using the
	 * fast_cluster_enable knobs from userspace.
	 */
	for (i = drv->safe_state_index + 1; i < drv->state_count; i++)
		if (i == t210_pm_data.idle_state_idx[CC6_IDX])
			drv->states[i].disabled = true;

	t210_pm_data.cc6_allow = true;

out:
	return 0;
}
late_initcall(tegra210_cpuidle_init);

static int __init tegra210_pm_init(void)
{
	if (tegra_get_chip_id() != TEGRA210)
		goto out;

	/* Disable CC4 until DFLL clk is ready */
	t210_pm_data.cc4_no_retention = true;

	/*
	 * CC6 also needs to be disabled until DFLL clk is ready, but there is
	 * no explicit hook that notifies when the DFLL init is complete. The
	 * cluster states can be disabled in the arm cpuidle driver which may
	 * not be ready here. Use this flag until the tegra_cpuidle driver
	 * disables the arm cpuidle driver's states during late_init.
	 */
	t210_pm_data.cc6_allow = false;

	t210_pm_data.idle_state_idx[C7_IDX] =
				tegra_of_idle_state_idx_from_name("c7");
	t210_pm_data.idle_state_idx[CC6_IDX] =
				tegra_of_idle_state_idx_from_name("cc6");
	t210_pm_data.idle_state_idx[CC7_IDX] =
				tegra_of_idle_state_idx_from_name("cc7");

	tegra210_cpu_pm_register_notifier(&tegra210_cpu_pm_nb);
	register_syscore_ops(&bpmp_sc7_suspend_ops);
out:
	return 0;
}
device_initcall(tegra210_pm_init);
#endif
