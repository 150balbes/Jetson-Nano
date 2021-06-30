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

#ifndef _LINUX_I2C_RTCPU_COMMON_H
#define _LINUX_I2C_RTCPU_COMMON_H

u32 tegra_i2c_get_clk_freq(struct device_node *np);
u32 tegra_i2c_get_reg_base(struct device_node *np);

#endif /* _LINUX_I2C_RTCPU_COMMON_H */
