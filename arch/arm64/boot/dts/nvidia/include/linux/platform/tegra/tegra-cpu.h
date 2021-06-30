/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <asm/cputype.h>
#include <asm/cpu.h>

#define MIDR_CPU_MASK		0xFF0FFFF0
#define MIDR_CPU_CARMEL		0x4E0F0040

static inline u8 tegra_is_cpu_carmel(u8 cpu)
{
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);
	return ((cpuinfo->reg_midr & MIDR_CPU_MASK) == MIDR_CPU_CARMEL);
}
