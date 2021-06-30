/*
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _LINUX_I2C_IVC_SINGLE_H
#define _LINUX_I2C_IVC_SINGLE_H

#include <linux/types.h>

struct i2c_msg;

int tegra_i2c_ivc_single_xfer(u32 reg_base,
		const struct i2c_msg *reqs, int num);

#endif /* _LINUX_I2C_IVC_SINGLE_H */
