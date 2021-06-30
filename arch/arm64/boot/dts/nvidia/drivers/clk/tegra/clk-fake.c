/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include "clk-tegra-bpmp.h"

struct fclk {
	struct clk_hw hw;
	unsigned long rate;
};

static int fclk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct fclk *fclk = container_of(hw, struct fclk, hw);

	fclk->rate = rate;

	return 0;
}

static unsigned long fclk_get_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct fclk *fclk = container_of(hw, struct fclk, hw);

	return fclk->rate;
}

static long fclk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	return rate;
}

static const struct clk_ops fclk_ops = {
	.set_rate = fclk_set_rate,
	.recalc_rate = fclk_get_rate,
	.round_rate = fclk_round_rate
};

static struct clk_onecell_data clk_data;

struct clk *tegra_fclk_init(int clk_num, char *name, size_t sz)
{
	struct fclk *fclk;
	struct clk_init_data init;

	fclk = kzalloc(sizeof(*fclk), GFP_KERNEL);
	if (!fclk)
		return ERR_PTR(-ENOMEM);

	snprintf(name, sz, "fclk%3d", clk_num);
	name[sz - 1] = 0;

	init.name = name;
	init.flags = 0;
	init.num_parents = 0;
	init.parent_names = NULL;
	init.ops = &fclk_ops;

	fclk->hw.init = &init;

	return clk_register(NULL, &fclk->hw);
}

static struct clk *tegra_of_clk_src_fclkget(struct of_phandle_args *clkspec,
		void *data)
{
	struct clk_onecell_data *clk_data = data;
	unsigned int idx = clkspec->args[0];
	char name[16];
	const size_t sz = sizeof(name);
	struct clk *clk;
	int r;

	if (idx >= clk_data->clk_num) {
		pr_err("%s() bad clk idx %d\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	if (clk_data->clks[idx])
		return clk_data->clks[idx];

	clk = tegra_fclk_init(idx, name, sz);

	if (IS_ERR_OR_NULL(clk)) {
		r = PTR_ERR(clk);
		pr_err("%s() failed to init clk %d (%d)\n", __func__, idx, r);
		clk = ERR_PTR(-EINVAL);
	}

	clk_data->clks[idx] = clk;

	return clk;
}

int tegra_fake_clks_init(struct device_node *np)
{
	const int num_clks = 1023;
	struct clk **pclks;

	pclks = kcalloc(num_clks + 1, sizeof(*pclks), GFP_KERNEL);
	if (!pclks) {
		WARN_ON(1);
		return -ENOMEM;
	}

	clk_data.clks = pclks;

	clk_data.clk_num = num_clks;

	return of_clk_add_provider(np, tegra_of_clk_src_fclkget, &clk_data);
}
