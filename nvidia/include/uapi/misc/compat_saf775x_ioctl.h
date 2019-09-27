 /*
 * compat_saf775x_ioctl.h  --  SAF775X Soc Audio driver IO control
 *
 * Copyright (c) 2017-2019 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __UAPI_COMPAT_SAF775X_IOCTL_H__
#define __UAPI_COMPAT_SAF775X_IOCTL_H__

struct compat_saf775x_cmd {
	compat_uint_t reg;
	compat_uint_t reg_len;
	compat_ulong_t val;
	compat_uint_t val_len;
};

struct compat_saf775x_control_param {
	char name[20];
	compat_uint_t reg;
	compat_uint_t num_reg;
	compat_int_t val;
};

struct compat_saf775x_control_info {
	char name[20];
	compat_int_t min;
	compat_int_t max;
	compat_int_t step;
	compat_int_t val;
};

enum {
	SAF775X_CONTROL_SET_IOCTL_32 = _IOW(0xF4, 0x01,
					struct compat_saf775x_cmd),
	SAF775x_CODEC_RESET_IOCTL_32 = _IO(0xF4, 0x02),
	SAF775X_CONTROL_GET_IOCTL_32 = _IOR(0xF4, 0x03,
					struct compat_saf775x_cmd),
	SAF775X_CONTROL_GET_MIXER_32 = _IOR(0xF4, 0x04,
					struct compat_saf775x_control_info),
	SAF775X_CONTROL_SET_MIXER_32 = _IOW(0xF4, 0x05,
					struct compat_saf775x_control_param),
	SAF775X_CONTROL_SETIF_32 = _IOW(0xF4, 0x6, compat_uint_t),
	SAF775X_CONTROL_GETIF_32 = _IO(0xF4, 0x7),
	SAF775X_CONTROL_KEYCODE_32 = _IOW(0xF4, 0x8,
					struct compat_saf775x_cmd),
};

#if IS_ENABLED(CONFIG_COMPAT)
	long compat_saf775x_hwdep_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
#else
	#define compat_saf775x_hwdep_ioctl NULL
#endif /* CONFIG_COMPAT */

#endif /* __UAPI_COMPAT_SAF775X_IOCTL_H__ */
