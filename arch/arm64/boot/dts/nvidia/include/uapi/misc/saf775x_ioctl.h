 /*
 * saf775x_ioctl.h  --  SAF775X Soc Audio driver IO control
 *
 * Copyright (c) 2014-2019 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHIN
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UAPI_SAF775X_IOCTL_H__
#define __UAPI_SAF775X_IOCTL_H__

#include <linux/ioctl.h>

struct saf775x_cmd {
	unsigned int reg;
	unsigned int reg_len;
	unsigned long val;
	unsigned int val_len;
};

struct saf775x_control_param {
	char name[20];
	unsigned int *reg;
	unsigned int num_reg;
	int val;
};

struct saf775x_control_info {
	char name[20];
	int min;
	int max;
	int step;
	int val;
};

#define SAF775X_CONTROL_SET_IOCTL	_IOW(0xF4, 0x01, struct saf775x_cmd)
#define SAF775x_CODEC_RESET_IOCTL	_IO(0xF4, 0x02)
#define SAF775X_CONTROL_GET_IOCTL	_IOR(0xF4, 0x03, struct saf775x_cmd)
#define SAF775X_CONTROL_GET_MIXER	_IOR(0xF4, 0x04, \
						struct saf775x_control_info)
#define SAF775X_CONTROL_SET_MIXER	_IOW(0xF4, 0x05, \
						struct saf775x_control_param)
#define SAF775X_CONTROL_SETIF		_IOW(0xF4, 0x6, unsigned int)
#define SAF775X_CONTROL_GETIF		_IO(0xF4, 0x7)
#define SAF775X_CONTROL_KEYCODE		_IOW(0xF4, 0x8, struct saf775x_cmd)

#endif /* __UAPI_SAF775X_IOCTL_H__ */
