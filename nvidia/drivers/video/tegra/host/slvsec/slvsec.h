/*
 * drivers/video/tegra/host/slvsec/slvsec.h
 *
 * Tegra GRHOST SLVS-EC
 *
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_SLVSEC_H__
#define __NVHOST_SLVSEC_H__

#include <linux/platform_device.h>

struct file_operations;

extern const struct file_operations tegra_t194_slvsec_ctrl_ops;

int slvsec_finalize_poweron(struct platform_device *pdev);
int slvsec_prepare_poweroff(struct platform_device *pdev);

#endif
