/*
 * Virtualized GPU Linux Interfaces
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __VGPU_LINUX_H__
#define __VGPU_LINUX_H__

struct device;
struct platform_device;

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION

#include <nvgpu/vgpu/vgpu.h>

int vgpu_pm_prepare_poweroff(struct device *dev);
int vgpu_pm_finalize_poweron(struct device *dev);
int vgpu_probe(struct platform_device *dev);
int vgpu_remove(struct platform_device *dev);

void vgpu_create_sysfs(struct device *dev);
void vgpu_remove_sysfs(struct device *dev);

int vgpu_tegra_suspend(struct device *dev);
int vgpu_tegra_resume(struct device *dev);
#else
/* define placeholders for functions used outside of vgpu */

static inline int vgpu_pm_prepare_poweroff(struct device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_pm_finalize_poweron(struct device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_probe(struct platform_device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_remove(struct platform_device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_tegra_suspend(struct device *dev)
{
	return -ENOSYS;
}
static inline int vgpu_tegra_resume(struct device *dev)
{
	return -ENOSYS;
}
#endif

#endif
