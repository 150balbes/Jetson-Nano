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

#ifndef _LINUX_TEGRA_CPU_EMC_H
#define _LINUX_TEGRA_CPU_EMC_H

struct device_node *of_get_scaling_node(const char *name);
int enable_cpu_emc_clk(void);
void disable_cpu_emc_clk(void);
void set_cpu_to_emc_freq(u32 cpu_freq);
int set_cpu_emc_limit_table_source(int table_source);
int get_cpu_emc_limit_table_source(void);

#endif
