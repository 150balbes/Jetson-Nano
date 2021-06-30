/*
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __UAPI_TEGRA_ISC_MGR_H__
#define __UAPI_TEGRA_ISC_MGR_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define ISC_MGR_IOCTL_PWR_DN		_IOW('o', 1, __s16)
#define ISC_MGR_IOCTL_PWR_UP		_IOR('o', 2, __s16)
#define ISC_MGR_IOCTL_SET_PID		_IOW('o', 3, struct isc_mgr_sinfo)
#define ISC_MGR_IOCTL_SIGNAL		_IOW('o', 4, int)
#define ISC_MGR_IOCTL_DEV_ADD		_IOW('o', 5, struct isc_mgr_new_dev)
#define ISC_MGR_IOCTL_DEV_DEL		_IOW('o', 6, int)
#define ISC_MGR_IOCTL_PWR_INFO		_IOW('o', 7, struct isc_mgr_pwr_info)
#define ISC_MGR_IOCTL_PWM_ENABLE	_IOW('o', 8, int)
#define ISC_MGR_IOCTL_PWM_CONFIG	_IOW('o', 9, struct isc_mgr_pwm_info)

#define ISC_MGR_POWER_ALL	5
#define MAX_ISC_NAME_LENGTH	32

struct isc_mgr_new_dev {
	__u16 addr;
	__u8 reg_bits;
	__u8 val_bits;
	__u8 drv_name[MAX_ISC_NAME_LENGTH];
};

struct isc_mgr_sinfo {
	__s32 pid;
	__s32 sig_no;
	__u64 context;
};

struct isc_mgr_pwr_info {
	__s32 pwr_gpio;
	__s32 pwr_status;
};

struct isc_mgr_pwm_info {
	__u64 duty_ns;
	__u64 period_ns;
};

enum {
	ISC_MGR_PWM_DISABLE = 0,
	ISC_MGR_PWM_ENABLE,
};

enum {
	ISC_MGR_SIGNAL_RESUME = 0,
	ISC_MGR_SIGNAL_SUSPEND,
};

#endif  /* __UAPI_TEGRA_ISC_MGR_H__ */
