/*
 * tc358870.h: HDMI to DSI bridge driver.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _HDMI2DSI_TC358870_
#define _HDMI2DSI_TC358870_

#include "panel/board-panel.h"

#define MAX_BRIDGE_INSTANCES	2
#define NUMOF_BIT_PER_BYTE      8
#define REG_LEN			2
#define MHZ_500			500000000
#define MHZ_250			250000000
#define MHZ_125			125000000
#define MHZ			1000000

/* DCS */
#define DATA0_MASK		0x00FF
#define DATA1_MASK		0xFF00
#define LONG_PKT_MASK		0x8000

#ifdef CONFIG_TEGRA_HDMI2DSI_TC358870
extern struct tegra_hdmi_out_ops tegra_hdmi2dsi_ops;
#else
#define tegra_hdmi2dsi_ops (*(struct tegra_hdmi_out_ops *) NULL)
#endif	/* CONFIG_TEGRA_HDMI2DSI_TC358870 */

enum tc358870_dsi_port {
	DSI_TX_NONE = 0,
	DSI_TX_0,
	DSI_TX_1,
	DSI_TX_BOTH
};

enum tc358870_clock_mode {
	DSI_NON_CONT_CLK = 0,
	DSI_CONT_CLK
};

enum tc358870_ddc5v_delays {
	DDC5V_DELAY_0MS,
	DDC5V_DELAY_50MS,
	DDC5V_DELAY_100MS,
	DDC5V_DELAY_200MS,
	DDC5V_DELAY_MAX = DDC5V_DELAY_200MS,
};

struct panel_out {
	u8 n_data_lanes;
	u8 pixel_format;
	u8 refresh_rate;
	struct tegra_dsi_cmd *dsi_init_cmd;
	u16 n_init_cmd;
	struct tegra_dsi_cmd *dsi_suspend_cmd;
	u16 n_suspend_cmd;
	struct tegra_dsi_cmd *dsi_postvideo_cmd;
	u16 n_postvideo_cmd;
};

struct tc358870_platform_data {

	struct regulator *dvdd;
	struct regulator *iovdd;
	int reset_gpio;		/* GPIOs Pin K8 */
	int bridge_instance;	/* bridge instance */
	int panel_connected;	/* panel connected */
	u32 refclk_hz;		/* 40 - 50 MHz */

	/* DDC +5V debounce delay */
	enum tc358870_ddc5v_delays ddc5v_delay;
	enum tc358870_dsi_port dsi_port;/* DSI output */

	bool enable_hdcp;	/* HDCP not yet implemented */

	/* DSI */
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
	u32 btacnt;		/* (0x00000020) */

	/* PLL */
	/* Bps pr lane is (refclk_hz / pll_prd) * pll_fbd */
	u16 pll_prd;		/* PRD from Macro + 1 (0x0A) */
	u16 pll_fbd;		/* FBD from Macro + 1 (0x7D) */
};

struct tc358870_state {
	struct tc358870_platform_data pdata;	/* Bridge structure */
	struct panel_out pout;			/* Panel structure */
	struct tegra_hdmi *hdmi;
	struct i2c_client *i2c_client;
	struct tegra_dc_mode *mode;
	struct tegra_panel_ops *out_ops;
	struct dentry *debugdir;
	bool enabled;
	bool power_down;

	struct mutex lock;

	/* edid  */
	u8 edid_blocks_written;
};

#endif /* _HDMI2DSI_TC358870_ */
