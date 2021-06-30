/*
 * Tegra CSI2 device common APIs
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __CSI2_H__
#define __CSI2_H__
#define TEGRA_CSICIL_CLK_MHZ 102

extern const struct tegra_csi_fops csi2_fops;

#if defined(CONFIG_ARCH_TEGRA_210_SOC)
static inline void csi_source_from_plld(void)
{
	tegra210_csi_source_from_plld();
}

static inline void csi_source_from_brick(void)
{
	tegra210_csi_source_from_brick();
}
#else
static inline void csi_source_from_plld(void)
{
}

static inline void csi_source_from_brick(void)
{
}
#endif

#endif
