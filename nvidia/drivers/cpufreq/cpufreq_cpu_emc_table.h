/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _CPUFREQ_CPU_EMC_TABLE_H
#define _CPUFREQ_CPU_EMC_TABLE_H

struct cpu_emc_mapping {
	uint32_t cpu_freq_khz;
	uint32_t emc_freq_khz;
};

extern struct cpu_emc_mapping*
tegra_cpufreq_cpu_emc_map_dt_init(struct device_node *);

extern unsigned long
tegra_cpu_to_emc_freq(uint32_t, struct cpu_emc_mapping *);

extern struct dentry * tegra_debugfs_create_cpu_emc_map(struct dentry *,
	struct cpu_emc_mapping *);

#endif
