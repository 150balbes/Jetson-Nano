/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _TEGRA_HV_PM_CTL_H
#define _TEGRA_HV_PM_CTL_H

extern int (*tegra_hv_pm_ctl_prepare_shutdown)(void);

int tegra_hv_pm_ctl_trigger_sys_suspend(void);
int tegra_hv_pm_ctl_trigger_sys_shutdown(void);
int tegra_hv_pm_ctl_trigger_sys_reboot(void);
int tegra_hv_pm_ctl_trigger_guest_suspend(u32 vmid);
int tegra_hv_pm_ctl_trigger_guest_resume(u32 vmid);

#endif /* _TEGRA_HV_PM_CTL_H */
