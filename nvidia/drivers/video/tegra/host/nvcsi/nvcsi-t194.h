/*
 * drivers/video/tegra/host/nvcsi/nvcsi-t194.h
 *
 * Tegra T194 Graphics Host NVCSI 2
 *
 * Copyright (c) 2017-2018 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVCSI_T194_H__
#define __NVHOST_NVCSI_T194_H__

struct file_operations;
struct platform_device;

extern const struct file_operations tegra194_nvcsi_ctrl_ops;

int tegra194_nvcsi_finalize_poweron(struct platform_device *pdev);
int tegra194_nvcsi_prepare_poweroff(struct platform_device *pdev);

#if IS_ENABLED(CONFIG_TEGRA_T19X_GRHOST) && IS_ENABLED(CONFIG_TEGRA_GRHOST_NVCSI)
int tegra194_nvcsi_cil_sw_reset(int lanes, int enable);
#else
static int inline tegra194_nvcsi_cil_sw_reset(int lanes, int enable)
{
	return 0;
}
#endif

int t194_nvcsi_early_probe(struct platform_device *pdev);
int t194_nvcsi_late_probe(struct platform_device *pdev);

#endif
