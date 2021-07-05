/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/clk/tegra.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <soc/tegra/chip-id.h>

#include <linux/tegra-aon-clk.h>
#include "clk.h"
#include "clk-tegra-bpmp.h"

/* Needed for a nvdisp linsim clock hack */
#define CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0 0x800008
#define CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0 0x801004

static void __init tegra_bpmp_staged_clock_init(struct device_node *np)
{
	tegra_bpmp_clk_init(np, 1);
}

static void __init tegra_bpmp_clock_init(struct device_node *np)
{
	tegra_bpmp_clk_init(np, 0);
}

static void __init tegra186_clock_init(struct device_node *np)
{
	tegra_bpmp_clk_init(np, 0);

	/* Nvdisp sim clock hack */
	if (tegra_platform_is_fpga()) {
		void __iomem *base;
		base = of_iomap(np, 0);
		if (!base) {
			pr_err("ioremap Tegra186 CAR failed\n");
			return;
		}

		writel(0x3ff, base + CLK_RST_CONTROLLER_RST_DEV_NVDISPLAY0_CLR_0);
		writel(0xf, base + CLK_RST_CONTROLLER_CLK_OUT_ENB_NVDISPLAY0_SET_0);
	}
}

static void __init tegra_of_fake_clks_init(struct device_node *np)
{
	int r;

	r = tegra_fake_clks_init(np);

	pr_info("tegra_fake_clks init %s\n", r ? "failed" : "ok");
}

static const struct of_device_id tegra_clock_ids[] __initconst = {
	{ .compatible = "nvidia,tegra-fake-clks",
		.data = tegra_of_fake_clks_init },
	{ .compatible = "nvidia,tegra18x-car",
		.data = tegra186_clock_init },
	{ .compatible = "nvidia,tegra-bpmp-clks",
		.data = tegra_bpmp_clock_init },
	{ .compatible = "nvidia,tegra-bpmp-staged-clks",
		.data = tegra_bpmp_staged_clock_init },
#ifdef CONFIG_TEGRA_AON
	{ .compatible = "nvidia,tegra-aon-clks",
		.data = tegra_aon_clk_init
	},
#endif
	{}
};

#ifdef CONFIG_TEGRA_CLK_DEBUG
static struct tegra_pto_table emc_pto = {
	.clk_id = 0, .divider = 1, .pto_id = 36,
};
#endif
int __init tegra_bpmp_of_clk_init(void)
{
	struct device_node *dn;
	struct clk *emc_clk;

	of_clk_init(tegra_clock_ids);
	/* see if we have EMC proxy in DT */
	dn = of_find_compatible_node(NULL, NULL, "nvidia,tegra-bpmp-emc-clk");
	if (dn == NULL) {
		goto out;
	}
	emc_clk = of_clk_get(dn, 0);
	if (IS_ERR_OR_NULL(emc_clk)) {
		pr_err("%s: no bpmp emc clock.\n", __func__);
		return -ENODEV;
	}
	clk_register_clkdev(emc_clk, "emc", 0);
#ifdef CONFIG_TEGRA_CLK_DEBUG
	tegra_clk_debugfs_add(emc_clk);
	tegra_register_pto(emc_clk, &emc_pto);
#endif

out:
	return 0;
}
arch_initcall(tegra_bpmp_of_clk_init);
