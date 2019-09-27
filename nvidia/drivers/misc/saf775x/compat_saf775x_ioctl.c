/*
 * compat_saf775x_ioctl.c -- SAF775X Soc Audio driver IO control
 *
 * Copyright (c) 2017-2019 NVIDIA CORPORATION.  All rights reserved.
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


#include "saf775x_ioctl.h"
#include <uapi/misc/compat_saf775x_ioctl.h>
#include <linux/uaccess.h>
#include <linux/compat.h>

static int compat_get_saf775x_cmd(
			struct compat_saf775x_cmd __user *data32,
			struct saf775x_cmd __user *data)
{
	compat_ulong_t l;
	compat_uint_t u;
	int err;

	err = get_user(u, &data32->reg);
	err |= put_user(u, &data->reg);
	err |= get_user(u, &data32->reg_len);
	err |= put_user(u, &data->reg_len);
	err |= get_user(l, &data32->val);
	err |= put_user(l, &data->val);
	err |= get_user(u, &data32->val_len);
	err |= put_user(u, &data->val_len);

	return err;
}

static int compat_put_saf775x_cmd(
			struct compat_saf775x_cmd __user *data32,
			struct saf775x_cmd __user *data)
{
	compat_ulong_t l;
	compat_uint_t u;
	int err;

	err = get_user(u, &data->reg);
	err |= put_user(u, &data32->reg);
	err |= get_user(u, &data->reg_len);
	err |= put_user(u, &data32->reg_len);
	err |= get_user(l, &data->val);
	err |= put_user(l, &data32->val);
	err |= get_user(u, &data->val_len);
	err |= put_user(u, &data32->val_len);

	return err;
}

static int compat_get_saf775x_param(
			struct compat_saf775x_control_param __user *data32,
			struct saf775x_control_param __user *data)
{
	char name[20];
	compat_int_t i;
	compat_uint_t u;
	int err;

	err = copy_from_user(name, &data32->name, sizeof(name));
	err |= copy_to_user(&data->name, name, sizeof(name));
	err |= get_user(u, &data32->reg);
	err |= put_user(u, (compat_uint_t *)data->reg);
	err |= get_user(u, &data32->num_reg);
	err |= put_user(u, &data->num_reg);
	err |= get_user(i, &data32->val);
	err |= put_user(i, &data->val);
	return err;
}

static int compat_put_saf775x_param(
			struct compat_saf775x_control_param __user *data32,
			struct saf775x_control_param __user *data)
{
	char name[20];
	compat_int_t i;
	compat_uint_t u;
	int err;

	err = copy_from_user(name, &data->name, sizeof(name));
	err |= copy_to_user(&data32->name, name, sizeof(name));
	err |= get_user(u, (compat_uint_t *)data->reg);
	err |= put_user(u, &data32->reg);
	err |= get_user(u, &data->num_reg);
	err |= put_user(u, &data32->num_reg);
	err |= get_user(i, &data->val);
	err |= put_user(i, &data32->val);
	return err;
}

static int compat_get_saf775x_info(
			struct compat_saf775x_control_info __user *data32,
			struct saf775x_control_info __user *data)
{
	char name[20];
	compat_int_t i;
	int err;

	err = copy_from_user(name, &data32->name, sizeof(name));
	err |= copy_to_user(&data->name, name, sizeof(name));
	err |= get_user(i, &data32->min);
	err |= put_user(i, &data->min);
	err |= get_user(i, &data32->max);
	err |= put_user(i, &data->max);
	err |= get_user(i, &data32->step);
	err |= put_user(i, &data->step);
	err |= get_user(i, &data32->val);
	err |= put_user(i, &data->val);

	return err;
}

static int compat_put_saf775x_info(
			struct compat_saf775x_control_info __user *data32,
			struct saf775x_control_info __user *data)
{
	char name[20];
	compat_int_t i;
	int err;

	err = copy_from_user(name, &data->name, sizeof(name));
	err |= copy_to_user(&data32->name, name, sizeof(name));
	err |= get_user(i, &data->min);
	err |= put_user(i, &data32->min);
	err |= get_user(i, &data->max);
	err |= put_user(i, &data32->max);
	err |= get_user(i, &data->step);
	err |= put_user(i, &data32->step);
	err |= get_user(i, &data->val);
	err |= put_user(i, &data32->val);
	return err;
}

long compat_saf775x_hwdep_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	long ret;
	int err;
	struct compat_saf775x_cmd __user *saf775x_cmd32;
	struct saf775x_cmd __user *saf775x_cmd;
	struct compat_saf775x_control_param __user *saf775x_param32;
	struct saf775x_control_param __user *saf775x_param;
	struct compat_saf775x_control_info __user *saf775x_info32;
	struct saf775x_control_info __user *saf775x_info;

	if (!file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case SAF775X_CONTROL_SET_IOCTL_32:
	case SAF775X_CONTROL_GET_IOCTL_32:
	case SAF775X_CONTROL_KEYCODE_32:
		saf775x_cmd32 = compat_ptr(arg);
		saf775x_cmd = compat_alloc_user_space(sizeof(*saf775x_cmd));
		if (saf775x_cmd == NULL)
			return -EFAULT;
		err = compat_get_saf775x_cmd(saf775x_cmd32, saf775x_cmd);
		if (err)
			return err;
		if (cmd == SAF775X_CONTROL_SET_IOCTL_32)
			ret = file->f_op->unlocked_ioctl(file,
				SAF775X_CONTROL_SET_IOCTL,
				(unsigned long)saf775x_cmd);
		else if (cmd == SAF775X_CONTROL_GET_IOCTL_32)
			ret = file->f_op->unlocked_ioctl(file,
				SAF775X_CONTROL_GET_IOCTL,
				(unsigned long)saf775x_cmd);
		else
			ret = file->f_op->unlocked_ioctl(file,
				SAF775X_CONTROL_KEYCODE,
				(unsigned long)saf775x_cmd);
		err = compat_put_saf775x_cmd(saf775x_cmd32, saf775x_cmd);
		return ret ? ret : err;

	case SAF775x_CODEC_RESET_IOCTL_32:
		ret = file->f_op->unlocked_ioctl(file,
			SAF775x_CODEC_RESET_IOCTL, arg);
		break;

	case SAF775X_CONTROL_GET_MIXER_32:
		saf775x_info32 = compat_ptr(arg);
		saf775x_info = compat_alloc_user_space(sizeof(*saf775x_info));
		if (saf775x_info == NULL)
			return -EFAULT;
		err = compat_get_saf775x_info(saf775x_info32, saf775x_info);
		if (err){
			pr_err("compat_get_saf775x_info err=%d\n",err);
			return -EFAULT;
		}
		ret = file->f_op->unlocked_ioctl(file,
			SAF775X_CONTROL_GET_MIXER,
			(unsigned long)saf775x_info);
		err = compat_put_saf775x_info(saf775x_info32, saf775x_info);
		return ret ? ret : err;

	case SAF775X_CONTROL_SET_MIXER_32:
		saf775x_param32 = compat_ptr(arg);
		saf775x_param = compat_alloc_user_space(sizeof(*saf775x_param));
		if (saf775x_param == NULL)
			return -EFAULT;
		err = compat_get_saf775x_param(saf775x_param32, saf775x_param);
		if (err)
			return err;
		ret = file->f_op->unlocked_ioctl(file,
			SAF775X_CONTROL_SET_MIXER,
			(unsigned long)saf775x_param);
		err = compat_put_saf775x_param(saf775x_param32, saf775x_param);
		return ret ? ret : err;

	case SAF775X_CONTROL_SETIF_32:
		ret = file->f_op->unlocked_ioctl(file,
				SAF775X_CONTROL_SETIF, (unsigned int)arg);
		return ret;

	case SAF775X_CONTROL_GETIF_32:
		ret = file->f_op->unlocked_ioctl(file,
				SAF775X_CONTROL_GETIF, arg);
		return ret;

	default:
		return -EFAULT;
	}

	return ret;
}

