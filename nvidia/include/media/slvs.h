/*
 * NVIDIA Tegra SLVS(-EC) Device
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Pekka Pessi <ppessi@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SLVS_H__
#define __SLVS_H__

#include <linux/platform_device.h>

struct tegra_mc_slvs;

struct tegra_mc_slvs *tegra_slvs_media_controller_init(
	struct platform_device *pdev);
void tegra_slvs_media_controller_remove(struct tegra_mc_slvs *slvs);

void tegra_slvs_media_controller_cil_notify(struct tegra_mc_slvs *slvs,
					u32 id, u32 events);

#endif
