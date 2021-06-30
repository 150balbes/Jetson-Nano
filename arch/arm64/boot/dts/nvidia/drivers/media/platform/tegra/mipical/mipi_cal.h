/*
 * mipi_cal.h
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION, All rights reserved.
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

#ifndef MIPI_CAL_H
#define MIPI_CAL_H

#define DSID	(1 << 31)
#define DSIC	(1 << 30)
#define DSIB	(1 << 29)
#define DSIA	(1 << 28)
#define CSIH	(1 << 27)
#define CSIG	(1 << 26)
#define CSIF	(1 << 25)
#define CSIE	(1 << 24)
#define CSID	(1 << 23)
#define CSIC	(1 << 22)
#define CSIB	(1 << 21)
#define CSIA	(1 << 20)
#define CPHY_MASK	1

#ifdef CONFIG_TEGRA_MIPI_CAL
int tegra_mipi_bias_pad_enable(void);
int tegra_mipi_bias_pad_disable(void);
int tegra_mipi_calibration(int lanes);
#else
static inline int tegra_mipi_bias_pad_enable(void)
{
	return 0;
}
static inline int tegra_mipi_bias_pad_disable(void)
{
	return 0;
}
static inline int tegra_mipi_calibration(int lanes)
{
	return 0;
}
#endif
#endif
