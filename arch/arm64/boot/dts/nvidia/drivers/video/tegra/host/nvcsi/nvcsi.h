/*
 * drivers/video/tegra/host/nvcsi/nvcsi.h
 *
 * Tegra Graphics Host NVCSI
 *
 * Copyright (c) 2015-2018 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_NVCSI_H__
#define __NVHOST_NVCSI_H__

#define CFG_ERR_STATUS2VI_MASK_VC3			(0x1 << 24)
#define CFG_ERR_STATUS2VI_MASK_VC2			(0x1 << 16)
#define CFG_ERR_STATUS2VI_MASK_VC1			(0x1 << 8)
#define CFG_ERR_STATUS2VI_MASK_VC0			(0x1 << 0)

extern const struct file_operations tegra_nvcsi_ctrl_ops;

int nvcsi_finalize_poweron(struct platform_device *pdev);
int nvcsi_prepare_poweroff(struct platform_device *pdev);

#if IS_ENABLED(CONFIG_TEGRA_GRHOST_NVCSI)
int nvcsi_cil_sw_reset(int lanes, int enable);
#else
static int inline nvcsi_cil_sw_reset(int lanes, int enable)
{
	return 0;
}
#endif

struct tegra_csi_device *tegra_get_mc_csi(void);
#endif
