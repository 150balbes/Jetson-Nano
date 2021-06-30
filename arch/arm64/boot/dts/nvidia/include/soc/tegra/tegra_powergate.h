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

#ifndef _LINUX_TEGRA_POWERGATE_H
#define _LINUX_TEGRA_POWERGATE_H

#include <linux/kernel.h>
#include <linux/tegra-powergate.h>
#include <dt-bindings/soc/tegra210-powergate.h>
#include <dt-bindings/soc/tegra186-powergate.h>
#include <dt-bindings/soc/tegra194-powergate.h>

struct platform_device;

extern int TEGRA_POWERGATE_DISA;

#ifdef CONFIG_POWERGATE_TEGRA_BPMP
int tegra_bpmp_init_powergate(struct platform_device *pdev);
#else
static inline int tegra_bpmp_init_powergate(struct platform_device *pdev)
{
	return 0;
}
#endif

#endif
