/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA_DRIVERS_CLK_H
#define __TEGRA_DRIVERS_CLK_H

int tegra_fake_clks_init(struct device_node *np);
struct clk *tegra_fclk_init(int clk_num, char *name, size_t sz);
int tegra_bpmp_clk_init(struct device_node *np, int staged);

#endif
