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

#ifndef _LINUX_I2C_RTCPU_CLK_CONFIG_H
#define _LINUX_I2C_RTCPU_CLK_CONFIG_H

/* Define speed modes */
#define I2C_STANDARD_MODE           100000
#define I2C_FAST_MODE               400000
#define I2C_FAST_MODE_PLUS          1000000
#define I2C_HS_MODE             3500000

#define I2C_CLK_DIVISOR             0x06c
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT 16
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE    8
#define I2C_CLK_DIVISOR_HS_MODE_MASK        0xFFFF

#define I2C_INTERFACE_TIMING_0                  0x94
#define I2C_TLOW_MASK                           0x3F
#define I2C_THIGH_SHIFT                         8
#define I2C_THIGH_MASK                          (0x3F << I2C_THIGH_SHIFT)

#define I2C_HS_INTERFACE_TIMING         0x9c
#define I2C_HS_TLOW_MASK            0x3F
#define I2C_HS_THIGH_SHIFT                         8
#define I2C_HS_THIGH_MASK           (0x3F << I2C_THIGH_SHIFT)

/*
 * I2C control data structure
 */
struct tegra_i2c_clk_config {
	struct clk *div_clk;
	struct clk *slow_clk;
	struct reset_control *rst;
	void __iomem *base;
	u32 bus_clk_rate;
	bool is_clkon_always;
	u16 clk_divisor_non_hs_mode;
	u32 low_clock_count;
	u32 high_clock_count;
	u32 hs_low_clock_count;
	u32 hs_high_clock_count;
	int clk_divisor_hs_mode;
};

int tegra_i2c_rtcpu_clock_enable(struct tegra_i2c_clk_config *i2c_config);
void tegra_i2c_rtcpu_clock_disable(struct tegra_i2c_clk_config *i2c_config);
int tegra_i2c_rtcpu_clock_init(struct tegra_i2c_clk_config *i2c_config);

#endif /* _LINUX_I2C_RTCPU_CLK_CONFIG_H */
