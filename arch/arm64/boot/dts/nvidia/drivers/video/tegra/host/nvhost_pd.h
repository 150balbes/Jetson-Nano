/*
 * drivers/video/tegra/host/nvhost_pd.h
 *
 * Tegra Graphics Host Legacy Power Domain Provider
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#ifndef __NVHOST_PD_H
#define __NVHOST_PD_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
void nvhost_pd_slcg_install_workaround(struct nvhost_device_data *pdata,
				       struct generic_pm_domain *pd);
void nvhost_pd_slcg_remove_workaround(struct nvhost_device_data *pdata,
				      struct generic_pm_domain *pd);
#endif

int nvhost_domain_init(struct of_device_id *matches);

#endif
