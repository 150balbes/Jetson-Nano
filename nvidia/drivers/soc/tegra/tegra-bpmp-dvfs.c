/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/export.h>
#include <linux/err.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra-bpmp-dvfs.h>

#define BPMP_CLK_CMD(cmd, id) ((id) | ((cmd) << 24))

static int64_t bpmp_dvfs_get_fmax_at_vmin(int clk_id)
{
	struct mrq_clk_request req;
	struct mrq_clk_response reply;
	int ret;

	req.cmd_and_id = BPMP_CLK_CMD(CMD_CLK_GET_FMAX_AT_VMIN, clk_id);
	ret = tegra_bpmp_send_receive(MRQ_CLK, &req, sizeof(req), &reply,
				      sizeof(reply));
	if (ret < 0)
		return ret;

	return reply.clk_get_fmax_at_vmin.rate;
}

/*
 * Get maximum frequency of the clock that guaranteed to be reachable at/below
 * minimum voltage at all temperatures.
 */
unsigned long tegra_bpmp_dvfs_get_fmax_at_vmin(int clk_id)
{
	int64_t rate;

	if (clk_id > 0) {
		rate = bpmp_dvfs_get_fmax_at_vmin(clk_id);
		if (rate > 0)
			return rate;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_bpmp_dvfs_get_fmax_at_vmin);

/*
 * Helper uitlity to retrieve clock id from clocks property in device tree
 * node.
 */
int tegra_bpmp_dvfs_get_clk_id(struct device_node *np, const char *name)
{
	int index;
	struct of_phandle_args clkspec;

	if (!np || !name)
		return -EINVAL;

	index = of_property_match_string(np, "clock-names", name);
	if (of_parse_phandle_with_args(np, "clocks", "#clock-cells", index,
				       &clkspec))
		return -ENOENT;

	return clkspec.args[0];
}
EXPORT_SYMBOL(tegra_bpmp_dvfs_get_clk_id);
