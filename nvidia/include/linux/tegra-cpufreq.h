/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_CPUFREQ_H
#define _LINUX_TEGRA_CPUFREQ_H

enum tegra_cpufreq_msg_ids {
	TEGRA_CPU_FREQ_THROTTLE,
	TEGRA_CPU_FREQ_SET_RATE,
	TEGRA_CPU_FREQ_GET_RATE,
	MAX_IVC_MSG_ID,
};

int parse_hv_dt_data(struct device_node *dn);
int parse_t194_cpufreq_hv_dt(struct device_node *dn);
bool hv_is_set_speed_supported(void);
void tegra_update_cpu_speed_hv(uint32_t rate, uint8_t cpu);
uint32_t t194_get_cpu_speed_hv(uint32_t cpu);
void t194_update_cpu_speed_hv(uint32_t rate, uint32_t cpu);

#endif
