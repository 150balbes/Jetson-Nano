/*
 * vmipi.h
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
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

#ifndef VMIPI_H
#define VMIPI_H

struct tegra_mipi;

enum {
	TEGRA_VMIPI_CMD_CALIBRATE = 0,
	TEGRA_VMIPI_CMD_BIAS_PAD_ENABLE = 1,
	TEGRA_VMIPI_CMD_BIAS_PAD_DISABLE = 2,
};

struct tegra_vmipi_calibrate_params {
	u32 lanes;
};

struct tegra_vmipi_cmd_msg {
	u32 cmd;
	int ret;
	union {
		struct tegra_vmipi_calibrate_params calibrate;
	} params;
};

#ifdef CONFIG_TEGRA_MIPI_CAL
int tegra_vmipi_init(struct platform_device *pdev);
void tegra_vmipi_deinit(void);
int tegra_vmipi_bias_pad_enable(struct tegra_mipi *mipi);
int tegra_vmipi_bias_pad_disable(struct tegra_mipi *mipi);
int tegra_vmipi_calibration(struct tegra_mipi *mipi, int lanes);
#else
static inline int tegra_vmipi_init(struct platform_device *pdev)
{
	return 0;
}
static inline void tegra_vmipi_init(void) {}
static inline int tegra_vmipi_bias_pad_enable(struct tegra_mipi *mipi)
{
	return 0;
}
static inline int tegra_vmipi_bias_pad_disable(struct tegra_mipi *mipi)
{
	return 0;
}
static inline int tegra_vmipi_calibration(struct tegra_mipi *mipi,
	int lanes)
{
	return 0;
}
#endif
#endif
