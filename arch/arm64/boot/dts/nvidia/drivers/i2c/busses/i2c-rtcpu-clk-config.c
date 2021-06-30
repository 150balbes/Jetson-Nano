/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/reset.h>

#include "i2c-rtcpu-clk-config.h"

static inline u32 i2c_camrtc_readl(struct tegra_i2c_clk_config *i2c_clk_config,
	unsigned long reg)
{
	return readl(i2c_clk_config->base + reg);
}

static inline void tegra_i2c_rtcpu_get_clk_parameters(
	struct tegra_i2c_clk_config *i2c_clk_config)
{
	u32 val;

	val = i2c_camrtc_readl(i2c_clk_config, I2C_INTERFACE_TIMING_0);
	i2c_clk_config->low_clock_count = val & I2C_TLOW_MASK;
	i2c_clk_config->high_clock_count = (val & I2C_THIGH_MASK)
		>> I2C_THIGH_SHIFT;

	val = i2c_camrtc_readl(i2c_clk_config, I2C_HS_INTERFACE_TIMING);
	i2c_clk_config->hs_low_clock_count = val & I2C_HS_TLOW_MASK;
	i2c_clk_config->hs_high_clock_count = ((val & I2C_HS_THIGH_MASK)
		>> I2C_HS_THIGH_SHIFT);

	val = i2c_camrtc_readl(i2c_clk_config, I2C_CLK_DIVISOR);
	i2c_clk_config->clk_divisor_hs_mode = val &
		I2C_CLK_DIVISOR_HS_MODE_MASK;
	i2c_clk_config->clk_divisor_non_hs_mode = (val >>
			I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT);
}

static inline int tegra_i2c_rtcpu_set_clk_rate(
	struct tegra_i2c_clk_config *i2c_clk_config)
{
	u32 clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE;
	int ret = 0;


	switch (i2c_clk_config->bus_clk_rate) {
	case I2C_HS_MODE:
		clk_multiplier = (i2c_clk_config->hs_low_clock_count +
				i2c_clk_config->hs_high_clock_count + 2);
		clk_multiplier *= (i2c_clk_config->clk_divisor_hs_mode + 1);
		break;
	case I2C_FAST_MODE_PLUS:
	case I2C_STANDARD_MODE:
	case I2C_FAST_MODE:
	default:
		clk_multiplier = (i2c_clk_config->low_clock_count +
				  i2c_clk_config->high_clock_count + 2);
		clk_multiplier *=
			(i2c_clk_config->clk_divisor_non_hs_mode + 1);
		break;
	}

	ret = clk_set_rate(i2c_clk_config->div_clk,
			   i2c_clk_config->bus_clk_rate * clk_multiplier);

	return ret;
}

int tegra_i2c_rtcpu_clock_enable(struct tegra_i2c_clk_config *i2c_clk_config)
{
	int ret;
	ret = clk_enable(i2c_clk_config->div_clk);
	if (ret < 0)
		goto err;

	if (i2c_clk_config->slow_clk) {
		ret = clk_enable(i2c_clk_config->slow_clk);
		if (ret < 0)
			clk_disable(i2c_clk_config->div_clk);
	}
err:
	return ret;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_clock_enable);

void tegra_i2c_rtcpu_clock_disable(struct tegra_i2c_clk_config *i2c_clk_config)
{
	clk_disable(i2c_clk_config->div_clk);
	if (i2c_clk_config->slow_clk)
		clk_disable(i2c_clk_config->slow_clk);
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_clock_disable);

int tegra_i2c_rtcpu_clock_init(struct tegra_i2c_clk_config *i2c_clk_config)
{
	u32 val;
	int err = 0;

	err = tegra_i2c_rtcpu_clock_enable(i2c_clk_config);
	if (err < 0)
		return err;

	reset_control_assert(i2c_clk_config->rst);
	udelay(2);
	reset_control_deassert(i2c_clk_config->rst);

	/* Make sure clock divisor programmed correctly */
	if (i2c_clk_config->bus_clk_rate == I2C_HS_MODE) {
		i2c_clk_config->clk_divisor_hs_mode = 2;
	} else {
		val = i2c_camrtc_readl(i2c_clk_config, I2C_CLK_DIVISOR);
		i2c_clk_config->clk_divisor_hs_mode =
			val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	}

	tegra_i2c_rtcpu_get_clk_parameters(i2c_clk_config);

	err = tegra_i2c_rtcpu_set_clk_rate(i2c_clk_config);
	if (err < 0)
		return err;

	tegra_i2c_rtcpu_clock_disable(i2c_clk_config);
	return err;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_clock_init);
