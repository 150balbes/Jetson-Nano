/*
 * tc358840.h - Toshiba UH2C/D HDMI-CSI bridge driver
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright (c) 2016 - 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _TC358840_
#define _TC358840_

enum tc358840_csi_port {
	CSI_TX_NONE = 0,
	CSI_TX_0,
	CSI_TX_1,
	CSI_TX_BOTH
};

enum tc358840_clock_mode {
	CSI_NON_CONT_CLK = 0,
	CSI_CONT_CLK
};

enum tc358840_ddc5v_delays {
	DDC5V_DELAY_0MS,
	DDC5V_DELAY_50MS,
	DDC5V_DELAY_100MS,
	DDC5V_DELAY_200MS,
	DDC5V_DELAY_MAX = DDC5V_DELAY_200MS,
};

struct tc358840_platform_data {
	/* GPIOs */
	int reset_gpio;		/* Pin K8 */
#ifdef CONFIG_V4L2_FWNODE
	struct v4l2_fwnode_endpoint endpoint;
#else
	struct v4l2_of_endpoint endpoint;
#endif
	/* System clock connected to REFCLK (pin K9) */
	u32 refclk_hz;		/* 40 - 50 MHz */

	/* DDC +5V debounce delay */
	enum tc358840_ddc5v_delays ddc5v_delay;

	/* HDCP */
	/* TODO: Not yet implemented */
	bool enable_hdcp;

	/* CSI Output */
	enum tc358840_csi_port csi_port;

	/* CSI */
	/* The values in brackets can serve as a starting point. */
	u32 lineinitcnt;	/* (0x00000FA0) */
	u32 lptxtimecnt;	/* (0x00000004) */
	u32 tclk_headercnt;	/* (0x00180203) */
	u32 tclk_trailcnt;	/* (0x00040005) */
	u32 ths_headercnt;	/* (0x000D0004) */
	u32 twakeup;		/* (0x00003E80) */
	u32 tclk_postcnt;	/* (0x0000000A) */
	u32 ths_trailcnt;	/* (0x00080006) */
	u32 hstxvregcnt;	/* (0x00000020) */

	/* PLL */
	/* Bps pr lane is (refclk_hz / pll_prd) * pll_fbd */
	u16 pll_prd;		/* PRD from Macro + 1 (0x0A) */
	u16 pll_fbd;		/* FBD from Macro + 1 (0x7D) */
};

/* custom controls */
#define TEGRA_CAMERA_CID_USER_TC358840_BASE (TEGRA_CAMERA_CID_BASE + 0x1000)

/* Audio sample rate in Hz */
#define TC358840_CID_AUDIO_SAMPLING_RATE \
	(TEGRA_CAMERA_CID_USER_TC358840_BASE + 1)
/* Audio present status */
#define TC358840_CID_AUDIO_PRESENT \
	(TEGRA_CAMERA_CID_USER_TC358840_BASE + 2)
/* Splitter width */
#define TC358840_CID_SPLITTER_WIDTH \
	(TEGRA_CAMERA_CID_USER_TC358840_BASE + 3)

#endif /* _TC358840_ */
