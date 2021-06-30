/*
 * arch/arm/mach-tegra/mc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2018, NVIDIA Corporation.  All rights reserved.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#define pr_fmt(fmt) "mc: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <linux/platform/tegra/mc-regs-t21x.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mcerr.h>
#include <linux/platform/tegra/tegra_emc.h>

#include <linux/platform/tegra/bwmgr_mc.h>

#include <soc/tegra/fuse.h>

#define MC_CLIENT_HOTRESET_CTRL		0x200
#define MC_CLIENT_HOTRESET_STAT		0x204
#define MC_CLIENT_HOTRESET_CTRL_1	0x970
#define MC_CLIENT_HOTRESET_STAT_1	0x974
#define MC_LATENCY_ALLOWANCE_BASE	MC_LATENCY_ALLOWANCE_AFI_0

#define MSSNVLINK_CYA_DESIGN_MODES		0x3c
#define MSS_NVLINK_L3_ALLOC_HINT		(1 << 2)

static DEFINE_SPINLOCK(tegra_mc_lock);
int mc_channels;
void __iomem *mc;
void __iomem *mc_regs[MC_MAX_CHANNELS];
unsigned int mssnvlink_hubs;
void __iomem *mssnvlink_regs[MC_MAX_MSSNVLINK_HUBS];

u32 tegra_mc_readl(u32 reg)
{
	return mc_readl(reg);
}
EXPORT_SYMBOL(tegra_mc_readl);

void tegra_mc_writel(u32 val, u32 reg)
{
	mc_writel(val, reg);
}
EXPORT_SYMBOL(tegra_mc_writel);

/*
 * Return carveout info for @co in @inf. If @nr is non-NULL then the number of
 * carveouts are also place in @*nr. If both @inf and @nr are NULL then the
 * validity of @co is checked and that is it.
 */
int mc_get_carveout_info(struct mc_carveout_info *inf, int *nr,
			 enum carveout_desc co)
{
#define MC_SECURITY_CARVEOUT(carveout, infop)				\
	do {								\
		(infop)->desc = co;					\
		(infop)->base = mc_readl(carveout ## _BOM) |		\
			((u64)mc_readl(carveout ## _BOM_HI) & 0x3) << 32; \
		(infop)->size = mc_readl(carveout ## _SIZE_128KB);	\
		(infop)->size <<= 17; /* Convert to bytes. */		\
	} while (0)

	if (!mc) {
		WARN(1, "Reading carveout info before MC init'ed!\n");
		return 0;
	}

	if (co >= MC_NR_CARVEOUTS)
		return -EINVAL;

	if (nr)
		*nr = MC_NR_CARVEOUTS;

	if (!inf)
		return 0;

	switch (co) {
	case MC_SECURITY_CARVEOUT1:
#ifdef MC_SECURITY_CARVEOUT1_BOM
		MC_SECURITY_CARVEOUT(MC_SECURITY_CARVEOUT1, inf);
		break;
#else
		return -ENODEV;
#endif
	case MC_SECURITY_CARVEOUT2:
#ifdef MC_SECURITY_CARVEOUT2_BOM
		MC_SECURITY_CARVEOUT(MC_SECURITY_CARVEOUT2, inf);
		break;
#else
		return -ENODEV;
#endif
	case MC_SECURITY_CARVEOUT4:
#ifdef MC_SECURITY_CARVEOUT4_BOM
		MC_SECURITY_CARVEOUT(MC_SECURITY_CARVEOUT4, inf);
		break;
#else
		return -ENODEV;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(mc_get_carveout_info);

/*
 * API to convert BW in bytes/s to clock frequency.
 *
 * bw: Bandwidth to convert. It can be in any unit - the resulting frequency
 *     will be in the same unit as passed. E.g KBps leads to KHz.
 */
unsigned long tegra_emc_bw_to_freq_req(unsigned long bw)
{
	return bwmgr_bw_to_freq(bw);
}
EXPORT_SYMBOL_GPL(tegra_emc_bw_to_freq_req);

/*
 * API to convert EMC clock frequency into theoretical available BW. This
 * does not account for a realistic utilization of the EMC bus. That is the
 * various overheads (refresh, bank commands, etc) that a real system sees
 * are not computed.
 *
 * freq: Frequency to convert. Like tegra_emc_bw_to_freq_req() it will work
 *       on any passed order of ten. The result will be on the same order.
 */
unsigned long tegra_emc_freq_req_to_bw(unsigned long freq)
{
	return bwmgr_freq_to_bw(freq);
}
EXPORT_SYMBOL_GPL(tegra_emc_freq_req_to_bw);

#define HOTRESET_READ_COUNT	5
static bool tegra_stable_hotreset_check(u32 stat_reg, u32 *stat)
{
	int i;
	u32 cur_stat;
	u32 prv_stat;
	unsigned long flags;

	spin_lock_irqsave(&tegra_mc_lock, flags);
	prv_stat = mc_readl(stat_reg);
	for (i = 0; i < HOTRESET_READ_COUNT; i++) {
		cur_stat = mc_readl(stat_reg);
		if (cur_stat != prv_stat) {
			spin_unlock_irqrestore(&tegra_mc_lock, flags);
			return false;
		}
	}
	*stat = cur_stat;
	spin_unlock_irqrestore(&tegra_mc_lock, flags);
	return true;
}

int tegra_mc_flush(int id)
{
	u32 rst_ctrl, rst_stat;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;
	unsigned int timeout;
	bool ret;

	if (!mc)
		return 0;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = mc_readl(rst_ctrl_reg);
	rst_ctrl |= (1 << id);
	mc_writel(rst_ctrl, rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);

	timeout = 0;
	do {
		bool exit = false;
		udelay(10);
		rst_stat = 0;
		ret = tegra_stable_hotreset_check(rst_stat_reg, &rst_stat);

		timeout++;

		/* keep lower timeout if we are running in qt or fpga */
		exit |= (timeout > 100) && (tegra_platform_is_qt() ||
			tegra_platform_is_fpga());
		/* otherwise have huge timeout (~1s) */
		exit |= timeout > 100000;
		if (exit) {
			WARN(1, "%s flush %d timeout\n", __func__, id);
			return -ETIMEDOUT;
		}

		if (!ret)
			continue;
	} while (!(rst_stat & (1 << id)));

	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush);

int tegra_mc_flush_done(int id)
{
	u32 rst_ctrl;
	u32 rst_ctrl_reg, rst_stat_reg;
	unsigned long flags;

	if (!mc)
		return 0;

	if (id < 32) {
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT;
	} else {
		id %= 32;
		rst_ctrl_reg = MC_CLIENT_HOTRESET_CTRL_1;
		rst_stat_reg = MC_CLIENT_HOTRESET_STAT_1;
	}

	spin_lock_irqsave(&tegra_mc_lock, flags);

	rst_ctrl = mc_readl(rst_ctrl_reg);
	rst_ctrl &= ~(1 << id);
	mc_writel(rst_ctrl, rst_ctrl_reg);

	spin_unlock_irqrestore(&tegra_mc_lock, flags);
	return 0;
}
EXPORT_SYMBOL(tegra_mc_flush_done);

/*
 * Map an MC register space. Each MC has a set of register ranges which must
 * be parsed. The first starting address in the set of ranges is returned as
 * it is expected that the DT file has the register ranges in ascending
 * order.
 *
 * device 0 = global channel.
 * device n = specific channel device-1, e.g device = 1 ==> channel 0.
 */
static void __iomem *tegra_mc_map_regs(struct platform_device *pdev, int device)
{
	struct resource res;
	const void *prop;
	void __iomem *regs;
	void __iomem *regs_start = NULL;
	u32 reg_ranges;
	int i, start;

	prop = of_get_property(pdev->dev.of_node, "reg-ranges", NULL);
	if (!prop) {
		pr_err("Failed to get MC MMIO region\n");
		pr_err("  device = %d: missing reg-ranges\n", device);
		return NULL;
	}

	reg_ranges = be32_to_cpup(prop);
	start = device * reg_ranges;

	for (i = 0; i < reg_ranges; i++) {
		regs = of_iomap(pdev->dev.of_node, start + i);
		if (!regs) {
			pr_err("Failed to get MC MMIO region\n");
			pr_err("  device = %d, range = %u\n", device, i);
			return NULL;
		}

		if (i == 0)
			regs_start = regs;
	}

	if (of_address_to_resource(pdev->dev.of_node, start, &res))
		return NULL;

	pr_info("mapped MMIO address: 0x%p -> 0x%lx\n",
		regs_start, (unsigned long)res.start);

	return regs_start;
}

/*
 * Map mssnvlink igpu hubs. In t19x, 4 igpu links supported
 */
static void enable_mssnvlinks(struct platform_device *pdev)
{
	struct device_node *dn = NULL;
	void __iomem *regs;
	int ret = 0, i;
	u32 reg_val;

	/* MSSNVLINK support is available in silicon or fpga only */
	if (!tegra_platform_is_silicon())
		return;

	dn = of_get_next_child(pdev->dev.of_node, NULL);
	if (!dn) {
		mssnvlink_hubs = UINT_MAX;
		dev_info(&pdev->dev, "No mssnvlink node\n");
		return;
	}

	ret = of_property_read_u32(dn, "mssnvlink_hubs", &mssnvlink_hubs);
	if (ret) {
		dev_err(&pdev->dev, "<mssnvlink_hubs> property missing in %s\n",
			pdev->dev.of_node->name);
			ret = -EINVAL;
			goto err_out;
	}

	if (mssnvlink_hubs > MC_MAX_MSSNVLINK_HUBS || mssnvlink_hubs < 1) {
		pr_err("Invalid number of mssnvlink hubs: %d\n", mssnvlink_hubs);
		ret = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < mssnvlink_hubs; i++) {
		regs = of_iomap(dn, i);
		if (!regs) {
			dev_err(&pdev->dev, "Failed to get MSSNVLINK aperture: %d\n", i);
			ret = PTR_ERR(regs);
			goto err_out;
		}
		mssnvlink_regs[i] = regs;
		reg_val = __raw_readl(regs + MSSNVLINK_CYA_DESIGN_MODES);
		reg_val |=  MSS_NVLINK_L3_ALLOC_HINT;
		__raw_writel(reg_val, regs + MSSNVLINK_CYA_DESIGN_MODES);
	}

err_out:
	WARN_ON(ret);
}

__weak const struct of_device_id tegra_mc_of_ids[] = {
	{ .compatible = "nvidia,tegra-mc" },
	{ .compatible = "nvidia,tegra-t18x-mc" },
	{ }
};

/*
 * MC driver init.
 */
static int tegra_mc_probe(struct platform_device *pdev)
{

#if defined(CONFIG_TEGRA_MC_EARLY_ACK)
	u32 reg;
#endif
	int i;
	const void *prop;
	struct dentry *mc_debugfs_dir = NULL;
	struct tegra_mc_data *mc_data;
	const struct of_device_id *match;

	if (!pdev->dev.of_node)
		return -EINVAL;

	match = of_match_device(tegra_mc_of_ids, &pdev->dev);
	if (!match) {
		pr_err("Missing DT entry!\n");
		return -EINVAL;
	}

	mc_data = (struct tegra_mc_data *)match->data;

	/*
	 * Channel count.
	 */
	prop = of_get_property(pdev->dev.of_node, "channels", NULL);
	if (!prop)
		mc_channels = 1;
	else
		mc_channels = be32_to_cpup(prop);

	if (mc_channels > MC_MAX_CHANNELS || mc_channels < 1) {
		pr_err("Invalid number of memory channels: %d\n", mc_channels);
		return -EINVAL;
	}

	/*
	 * IO mem.
	 */
	mc = tegra_mc_map_regs(pdev, 0);
	if (!mc)
		return -ENOMEM;

	/* Populate the rest of the channels... */
	if (mc_channels > 1) {
		for (i = 1; i <= mc_channels; i++) {
			mc_regs[i - 1] = tegra_mc_map_regs(pdev, i);
			if (!mc_regs[i - 1])
				return -ENOMEM;
		}
	} else {
		/* Make channel 0 the same as the MC broadcast range. */
		mc_regs[0] = mc;
	}

	enable_mssnvlinks(pdev);

#if defined(CONFIG_TEGRA_MC_EARLY_ACK)
	reg = mc_readl(MC_EMEM_ARB_OVERRIDE);
	reg |= 3;
#if defined(CONFIG_TEGRA_ERRATA_1157520)
	if (tegra_revision == TEGRA_REVISION_A01)
		reg &= ~2;
#endif
	mc_writel(reg, MC_EMEM_ARB_OVERRIDE);
#endif

#ifdef CONFIG_DEBUG_FS
	mc_debugfs_dir = debugfs_create_dir("mc", NULL);
	if (mc_debugfs_dir == NULL)
		pr_err("Failed to make debugfs node: %ld\n",
		       PTR_ERR(mc_debugfs_dir));
#endif

	tegra_mcerr_init(mc_debugfs_dir, pdev);

	return 0;
}

u32 __weak tegra_get_dvfs_clk_change_latency_nsec(unsigned long emc_freq_khz)
{
	return 2000;
}

static int tegra_mc_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mc_driver = {
	.driver = {
		.name	= "nv-tegra-mc",
		.of_match_table = tegra_mc_of_ids,
		.owner	= THIS_MODULE,
	},

	.probe		= tegra_mc_probe,
	.remove		= tegra_mc_remove,
};

static int __init tegra_mc_init(void)
{
	int ret;

	ret = platform_driver_register(&mc_driver);
	if (ret)
		return ret;

	return 0;
}
core_initcall(tegra_mc_init);

static void __exit tegra_mc_fini(void)
{
}
module_exit(tegra_mc_fini);
