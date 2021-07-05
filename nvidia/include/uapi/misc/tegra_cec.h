/*
 * include/uapi/misc/tegra_cec.h
 *
 * Copyright (c) 2012-2020, NVIDIA CORPORATION.  All rights reserved.
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


#ifndef __UAPI_TEGRA_CEC_H
#define __UAPI_TEGRA_CEC_H

#define TEGRA_CEC_IOC_MAGIC 'C'

#define TEGRA_CEC_IOCTL_ERROR_RECOVERY		_IO(TEGRA_CEC_IOC_MAGIC, 1)
#define TEGRA_CEC_IOCTL_DUMP_REGISTERS		_IO(TEGRA_CEC_IOC_MAGIC, 2)
#define TEGRA_CEC_IOCTL_SET_RX_SNOOP		_IO(TEGRA_CEC_IOC_MAGIC, 3)
#define TEGRA_CEC_IOCTL_GET_RX_SNOOP		_IO(TEGRA_CEC_IOC_MAGIC, 4)
#define TEGRA_CEC_IOCTL_GET_POST_RECOVERY	_IO(TEGRA_CEC_IOC_MAGIC, 5)

#endif /* __UAPI_TEGRA_CEC_H */
