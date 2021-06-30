/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_CPU_H
#define _LINUX_TEGRA_CPU_H

#include <asm/cputype.h>

static inline int tegra_is_this_cpu_denver(void)
{
	return read_cpuid_implementor() == ARM_CPU_IMP_NVIDIA;
}

static inline int tegra_is_this_cpu_arm(void)
{
	return read_cpuid_implementor() == ARM_CPU_IMP_ARM;
}
int tegra_get_clk_counter(u32 mpidr, u32 midr, u32 *coreclk,
	u32 *refclk);
#endif
