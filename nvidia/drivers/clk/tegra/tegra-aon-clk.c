/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/device.h>
#include <linux/of_address.h>
#include <linux/tegra-aon.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/mutex.h>
#include <soc/tegra/tegra-aon-ivc-pllaon.h>

static int clk_aon_enable(struct clk_hw *hw)
{
	/* Send IVC to SPE */
	return tegra_aon_set_pllaon_state(1);
}

static void clk_aon_disable(struct clk_hw *hw)
{
	/* Send IVC to SPE */
	(void)tegra_aon_set_pllaon_state(0);
}

static int clk_aon_is_enabled(struct clk_hw *hw)
{
	/* Check status from SPE */
	return tegra_aon_get_pllaon_state();
}

const struct clk_ops tegra_clk_aon_gate_ops = {
	.is_prepared = clk_aon_is_enabled,
	.prepare = clk_aon_enable,
	.unprepare = clk_aon_disable,
};

struct tegra_clk_aon {
	struct clk_hw hw;
	int clk_num;
} aon_clk;

static DEFINE_MUTEX(clk_aon_lock);

static struct clk *tegra_of_aon_clk_src_onecell_get(
				struct of_phandle_args *clkspec,
				void *data)
{
	struct clk_init_data init;
	static struct clk *aon_clk_obj;
	const char *clk_name = "spe_pll_aon";

	mutex_lock(&clk_aon_lock);
	if (!aon_clk_obj) {
		memset(&init, 0, sizeof(struct clk_init_data));
		init.name = clk_name;
		init.flags = 0;
		init.ops = &tegra_clk_aon_gate_ops;
		aon_clk.clk_num = clkspec->args[1];
		aon_clk.hw.init = &init;
		aon_clk_obj = clk_register(NULL, &aon_clk.hw);
		if (IS_ERR(aon_clk_obj)) {
			pr_err("registration of pll_aon as aon clock failed clk_num: %d\n",
				aon_clk.clk_num);
		}
	}
	mutex_unlock(&clk_aon_lock);
	pr_info("PLL_AON clock registered\n");

	return aon_clk_obj;
}

int tegra_aon_clk_init(struct device_node *np)
{
	int r;

	r = of_clk_add_provider(np, tegra_of_aon_clk_src_onecell_get, NULL);
	pr_info("%s: %s\n", __func__, r ? "failed" : "ok");

	return r;
}
