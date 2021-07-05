/*
 * tegra210_sfc_alt.h - Definitions for Tegra210 SFC driver
 *
 * Copyright (c) 2014-2019 NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA210_SFC_ALT_H__
#define __TEGRA210_SFC_ALT_H__

/*
 * SFC_AXBAR_RX registers are with respect to AXBAR.
 * The data is coming from AXBAR to SFC for playback.
 */
#define TEGRA210_SFC_AXBAR_RX_STATUS			0x0c
#define TEGRA210_SFC_AXBAR_RX_INT_STATUS		0x10
#define TEGRA210_SFC_AXBAR_RX_INT_MASK			0x14
#define TEGRA210_SFC_AXBAR_RX_INT_SET			0x18
#define TEGRA210_SFC_AXBAR_RX_INT_CLEAR			0x1c
#define TEGRA210_SFC_AXBAR_RX_CIF_CTRL			0x20
#define TEGRA210_SFC_AXBAR_RX_FREQ				0x24
#define TEGRA210_SFC_AXBAR_RX_CYA				0x28
#define TEGRA210_SFC_AXBAR_RX_DBG				0x2c

/*
 * SFC_AXBAR_TX registers are with respect to AXBAR.
 * The data is going out of SFC for playback.
 */
#define TEGRA210_SFC_AXBAR_TX_STATUS			0x4c
#define TEGRA210_SFC_AXBAR_TX_INT_STATUS		0x50
#define TEGRA210_SFC_AXBAR_TX_INT_MASK			0x54
#define TEGRA210_SFC_AXBAR_TX_INT_SET			0x58
#define TEGRA210_SFC_AXBAR_TX_INT_CLEAR			0x5c
#define TEGRA210_SFC_AXBAR_TX_CIF_CTRL			0x60
#define TEGRA210_SFC_AXBAR_TX_FREQ				0x64
#define TEGRA210_SFC_AXBAR_TX_CYA				0x68
#define TEGRA210_SFC_AXBAR_TX_DBG				0x6c

/* Register offsets from TEGRA210_SFC*_BASE */
#define TEGRA210_SFC_ENABLE						0x80
#define TEGRA210_SFC_SOFT_RESET					0x84
#define TEGRA210_SFC_CG							0x88
#define TEGRA210_SFC_STATUS						0x8c
#define TEGRA210_SFC_INT_STATUS					0x90
#define TEGRA210_SFC_CYA						0x94
#define TEGRA210_SFC_DBG						0xac
#define TEGRA210_SFC_COEF_RAM					0xbc
#define TEGRA210_SFC_AHUBRAMCTL_SFC_CTRL		0xc0
#define TEGRA210_SFC_AHUBRAMCTL_SFC_DATA		0xc4

/* Fields in TEGRA210_SFC_ENABLE */
#define TEGRA210_SFC_EN_SHIFT				0
#define TEGRA210_SFC_EN						(1 << TEGRA210_SFC_EN_SHIFT)

#define TEGRA210_SFC_BITS_8					1
#define TEGRA210_SFC_BITS_12				2
#define TEGRA210_SFC_BITS_16				3
#define TEGRA210_SFC_BITS_20				4
#define TEGRA210_SFC_BITS_24				5
#define TEGRA210_SFC_BITS_28				6
#define TEGRA210_SFC_BITS_32				7

#define TEGRA210_SFC_NUM_RATES 13

/* Fields in TEGRA210_SFC_COEF_RAM */
#define TEGRA210_SFC_COEF_RAM_COEF_RAM_EN	BIT(0)

#define TEGRA210_SFC_SOFT_RESET_EN              BIT(0)

/* SRC coefficients */
#define TEGRA210_SFC_COEF_RAM_DEPTH		64

struct tegra210_sfc {
	int srate_in;
	int srate_out;
	int format_in;
	int format_out;
	struct regmap *regmap;
	struct snd_pcm_hw_params in_hw_params;
	struct snd_pcm_hw_params out_hw_params;
	int stereo_conv_input;
	int mono_conv_output;
	unsigned int channels_via_control;
};

#endif
