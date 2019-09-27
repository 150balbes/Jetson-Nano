/*
 * Copyright (c) 2018 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_RTCPU_COVERAGE_H_
#define _LINUX_TEGRA_RTCPU_COVERAGE_H_

#include <linux/types.h>

struct tegra_rtcpu_coverage *tegra_rtcpu_coverage_create(struct device *dev);
int tegra_rtcpu_coverage_boot_sync(struct tegra_rtcpu_coverage *coverage);
void tegra_rtcpu_coverage_destroy(struct tegra_rtcpu_coverage *coverage);

#endif
