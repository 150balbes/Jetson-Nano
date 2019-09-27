/*
 * arch/arm/mach-tegra/mcerr.c
 *
 * MC error code common to T3x and T11x. T20 has been left alone.
 *
 * Copyright (c) 2010-2018, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt) "mc-err: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/stat.h>
#include <linux/sched.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/atomic.h>

#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mcerr.h>
#include <linux/platform/tegra/tegra_emc_err.h>
#include <linux/platform/tegra/mc-regs-t18x.h>

static const struct of_device_id __mcerr_of_table_sentinel
	__used __section(__mcerr_of_table_end);
extern struct of_device_id __mcerr_of_table;

static bool mcerr_throttle_enabled = true;
u32  mcerr_silenced;
static atomic_t error_count;

static void unthrottle_prints(struct work_struct *work);
static DECLARE_DELAYED_WORK(unthrottle_prints_work, unthrottle_prints);
static struct dentry *mcerr_debugfs_dir;
u32 mc_int_mask;
static struct mcerr_ops *mcerr_ops;

static void unthrottle_prints(struct work_struct *work)
{
	atomic_set(&error_count, 0);
}

static void disable_interrupt(unsigned int irq)
{
	mc_writel(0, MC_INTMASK);
}

static void enable_interrupt(unsigned int irq)
{
	mc_writel(mc_int_mask, MC_INTMASK);
}

static irqreturn_t tegra_mcerr_thread(int irq, void *data)
{
	unsigned long count;

	cancel_delayed_work(&unthrottle_prints_work);
	count = atomic_inc_return(&error_count);

	if (mcerr_throttle_enabled && count >= MAX_PRINTS) {
		schedule_delayed_work(&unthrottle_prints_work, HZ/2);
		if (count == MAX_PRINTS)
			mcerr_pr("Too many MC errors; throttling prints\n");
		mcerr_ops->clear_interrupt(irq);
		goto exit;
	}

	mcerr_ops->log_mcerr_fault(irq);
exit:
	mcerr_ops->enable_interrupt(irq);

	return IRQ_HANDLED;
}

/*
 * The actual error handling takes longer than is ideal so this must be
 * threaded.
 */
static irqreturn_t tegra_mcerr_hard_irq(int irq, void *data)
{
#ifdef CONFIG_TEGRA_MC_TRACE_PRINTK
	trace_printk("MCERR detected.\n");
#endif
	 /*
	  * Disable MC Error interrupt till the MC Error info is logged.
	  * MC Errors can be lost as MC HW holds one MC error at a time.
	  * The first MC Error is good enough to point out potential memory
	  * access issues in SW and allow debugging further.
	  */
	mcerr_ops->disable_interrupt(irq);
	return IRQ_WAKE_THREAD;
}

/*
 * Print the MC err stats for each client.
 */
static int mcerr_default_debugfs_show(struct seq_file *s, void *v)
{
	int i, j;
	int do_print;

	seq_printf(s, "%-18s %-18s", "swgroup", "client");
	for (i = 0; i < (sizeof(u32) * 8); i++) {
		if (mcerr_ops->intr_descriptions[i])
			seq_printf(s, " %-12s",
				   mcerr_ops->intr_descriptions[i]);
	}
	seq_puts(s, "\n");

	for (i = 0; i < mcerr_ops->nr_clients; i++) {
		do_print = 0;

		/* Only print clients who actually have errors. */
		for (j = 0; j < (sizeof(u32) * 8); j++) {
			if (mcerr_ops->intr_descriptions[j] &&
			    mcerr_ops->mc_clients[i].intr_counts[j]) {
				do_print = 1;
				break;
			}
		}

		if (do_print) {
			seq_printf(s, "%-18s %-18s",
				   mcerr_ops->mc_clients[i].name,
				   mcerr_ops->mc_clients[i].swgroup);
			for (j = 0; j < (sizeof(u32) * 8); j++) {
				if (!mcerr_ops->intr_descriptions[j])
					continue;
				seq_printf(s, " %-12u",
					   mcerr_ops->mc_clients[i].intr_counts[j]);
			}
			seq_puts(s, "\n");
		}
	}

	return 0;
}

static int mcerr_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, mcerr_ops->mcerr_debugfs_show, NULL);
}

static const struct file_operations mcerr_debugfs_fops = {
	.open           = mcerr_debugfs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __get_throttle(void *data, u64 *val)
{
	*val = mcerr_throttle_enabled;
	return 0;
}

static int __set_throttle(void *data, u64 val)
{
	atomic_set(&error_count, 0);

	mcerr_throttle_enabled = (bool) val;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mcerr_throttle_debugfs_fops, __get_throttle,
			__set_throttle, "%llu\n");

int tegra_mcerr_init(struct dentry *mc_parent, struct platform_device *pdev)
{
	int irq;
	const void *prop;
	bool match_found = false;
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *matches = &__mcerr_of_table;

	for (; matches; matches++) {
		if (of_device_is_compatible(np, matches->compatible)) {
			const of_mcerr_init_fn init_fn = matches->data;

			mcerr_ops = init_fn(np);
			match_found = true;
			break;
		}
	}

	if (WARN_ON(match_found == false)) {
		pr_err("%s: no mcerr_ops found\n", __func__);
		return -EINVAL;
	}

	if (!mcerr_ops || !mcerr_ops->clear_interrupt ||
		!mcerr_ops->log_mcerr_fault) {
		pr_err("invalid mcerr ops. disabling mcerr.\n");
		goto fail;
	}

	mcerr_ops->mcerr_debugfs_show = mcerr_ops->mcerr_debugfs_show ?: mcerr_default_debugfs_show;
	mcerr_ops->enable_interrupt = mcerr_ops->enable_interrupt ?: enable_interrupt;
	mcerr_ops->disable_interrupt = mcerr_ops->disable_interrupt ?: disable_interrupt;

	if (mcerr_ops->nr_clients == 0 ||
	    mcerr_ops->intr_descriptions == NULL) {
		pr_err("Missing necessary chip_specific functionality!\n");
		return -ENODEV;
	}

	prop = of_get_property(pdev->dev.of_node, "int_mask", NULL);
	if (!prop) {
		pr_err("No int_mask prop for mcerr!\n");
		return -EINVAL;
	}

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq < 0) {
		pr_err("Unable to parse/map MC error interrupt\n");
		goto done;
	}

	if (request_threaded_irq(irq, tegra_mcerr_hard_irq,
				 tegra_mcerr_thread, 0, "mc_status", NULL)) {
		pr_err("Unable to register MC error interrupt\n");
		goto done;
	}

	mc_int_mask = be32_to_cpup(prop);
	/* clear any mc-err's that occured before. */
	mcerr_ops->clear_interrupt(irq);
	mc_writel(mc_int_mask, MC_INTMASK);
	pr_debug("Set intmask: 0x%x\n", mc_readl(MC_INTMASK));

	/* This need to be fixed to work for all SOC's. */
	if (IS_ENABLED(CONFIG_ARCH_TEGRA_18x_SOC)) {
		prop = of_get_property(pdev->dev.of_node,"compatible", NULL);
		if (prop && strcmp(prop, "nvidia,tegra-t18x-mc") == 0)
			tegra_emcerr_init(mc_parent, pdev);
	}

	if (!mc_parent)
		goto done;

	mcerr_debugfs_dir = debugfs_create_dir("err", mc_parent);
	if (mcerr_debugfs_dir == NULL) {
		pr_err("Failed to make debugfs node: %ld\n",
		       PTR_ERR(mcerr_debugfs_dir));
		goto done;
	}
	debugfs_create_file("mcerr", 0644, mcerr_debugfs_dir, NULL,
			    &mcerr_debugfs_fops);
	debugfs_create_file("mcerr_throttle", S_IRUGO | S_IWUSR,
			    mcerr_debugfs_dir, NULL,
			    &mcerr_throttle_debugfs_fops);
	debugfs_create_u32("quiet", 0644, mcerr_debugfs_dir, &mcerr_silenced);
done:
	return 0;
fail:
	pr_err("init failied\n");
	return -EINVAL;
}
