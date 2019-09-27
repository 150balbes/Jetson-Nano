/*
 * Copyright (C) 2015-2017, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_NVADSP_DEV_T18X_H
#define __TEGRA_NVADSP_DEV_T18X_H

int nvadsp_acast_init(struct platform_device *pdev);
int nvadsp_reset_t18x_init(struct platform_device *pdev);
int nvadsp_os_t18x_init(struct platform_device *pdev);
int nvadsp_pm_t18x_init(struct platform_device *pdev);

#endif /* __TEGRA_NVADSP_DEV_T18X_H */
