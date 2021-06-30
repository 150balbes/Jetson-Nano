/*
 * tc358870.c: HDMI to DSI bridge driver.
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

#include <linux/module.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/debugfs.h>

#include "panel/board-panel.h"
#include "dc.h"
#include "dc_priv.h"
#include "hdmi2dsi_tc358870.h"
#include "hdmi2dsi_tc358870_regs.h"
#include "edid.h"
#include "hdmi2.0.h"

static struct tc358870_state *gstate[MAX_BRIDGE_INSTANCES];
static int greset_gpio[MAX_BRIDGE_INSTANCES];

/* To get parent (struct tc358870_state) of memeber (i2c_client *) */
static inline struct tc358870_state *to_state(struct i2c_client *client)
{
	return container_of(&client, struct tc358870_state, i2c_client);
}

/* --------------- I2C --------------- */

static void i2c_rd(struct tc358870_state *state, u16 reg, u8 *values, u32 n)
{
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[REG_LEN] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = REG_LEN,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev,
			"%s: reading register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
	}

	dev_dbg(&client->dev, "I2C read %d bytes from 0x%04X = 0x%02X\n",
		n, reg, values[0]);

}

static void i2c_wr(struct tc358870_state *state, u16 reg, u8 *values, u32 n)
{
	struct i2c_client *client = state->i2c_client;
	int err, i;
	struct i2c_msg msg;
	u8 data[REG_LEN + n];

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = REG_LEN + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[REG_LEN + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		dev_err(&client->dev,
			"%s: writing register 0x%x from 0x%x failed\n",
			__func__, reg, client->addr);
		return;
	}

	dev_dbg(&client->dev, "I2C write %d bytes in  0x%04X = 0x%02X\n",
		n, reg, data[REG_LEN]);
}

static u8 i2c_rd8(struct tc358870_state *state, u16 reg)
{
	u8 val;

	i2c_rd(state, reg, &val, 1);

	return val;
}

static void i2c_wr8(struct tc358870_state *state, u16 reg, u8 val)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s reg = 0x%04x val = 0x%04x",
				__func__, reg, val);
	i2c_wr(state, reg, &val, 1);
}

static void i2c_wr8_and_or(struct tc358870_state *state, u16 reg,
		u8 mask, u8 val)
{
	i2c_wr8(state, reg, (i2c_rd8(state, reg) & mask) | val);
}

static u16 i2c_rd16(struct tc358870_state *state, u16 reg)
{
	u16 val;

	i2c_rd(state, reg, (u8 *)&val, 2);

	return val;
}

static void i2c_wr16(struct tc358870_state *state, u16 reg, u16 val)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s reg = 0x%04x val = 0x%04x",
				__func__, reg, val);

	i2c_wr(state, reg, (u8 *)&val, 2);
}

static u32 i2c_rd32(struct tc358870_state *state, u16 reg)
{
	u32 val;

	i2c_rd(state, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(struct tc358870_state *state, u16 reg, u32 val)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s reg = 0x%04x val = 0x%08x",
				__func__, reg, val);

	i2c_wr(state, reg, (u8 *)&val, 4);
}

static void i2c_wr32_and_or(struct tc358870_state *state, u32 reg,
				u32 mask, u32 val)
{
	i2c_wr32(state, reg, (i2c_rd32(state, reg) & mask) | val);
}

static inline bool is_hdmi(struct tc358870_state *state)
{
	return i2c_rd8(state, SYS_STATUS) & MASK_S_HDMI;
}

/* Ctl set */
static void tc358870_set_ctl(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr16(state, CONFCTL0, MASK_AUTOINDEX);
	i2c_wr16(state, SYSCTL, MASK_RESET_ALL);
	i2c_wr16(state, SYSCTL, MASK_NORMAL);
	i2c_wr16(state, CONFCTL1, MASK_INIT);
}

/* Split control */
static void tc358870_split_control(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr16(state, SPLITTX0_CTRL, MASK_SP_EN_SPLIT);
	i2c_wr16(state, SPLITTX0_SPLIT, MASK_SPLIT_TXHW_AUTO);
	i2c_wr16(state, SPLITTX1_CTRL, MASK_SP_EN_SPLIT);
}

/* HDMI Phy */
static void tc358870_hdmi_phy(struct tc358870_state *state, bool enable)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s: enable = %d\n", __func__, enable);

	if (enable) {
		i2c_wr8(state, PHY_CTL, MASK_POWERCTL | MASK_48_MHZ);
		i2c_wr8_and_or(state, PHY_ENB, ~MASK_ENABLE_PHY,
				MASK_ENABLE_PHY);
		i2c_wr8_and_or(state, EQ_BYPS, ~DISABLE_BYPS, DISABLE_BYPS);
		i2c_wr8(state, APPL_CTL, MASK_APLL_CPCTL | MASK_APLL_ON);
		i2c_wr8(state, DDCIO_CTL, MASK_DDC_PWR_ON);
	} else {
		i2c_wr8(state, PHY_CTL, MASK_48_MHZ);
		i2c_wr8(state, APPL_CTL, MASK_APLL_CPCTL);
		i2c_wr8_and_or(state, PHY_ENB, ~MASK_ENABLE_PHY, 0x0);
	}
}

/* HDMI Clock */
static void tc358870_hdmi_clock(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;
	struct tc358870_platform_data *pdata = &state->pdata;
	u32 lock_ref_freq;
	u16 sys_freq;
	u16 csc_freq;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* The refclk should be between 40 and 50 Mhz */
	WARN_ON((pdata->refclk_hz < MIN_REFCLK) ||
		(pdata->refclk_hz > MAX_REFCLK));

	/* System clock is set as clock frequency divided by 10000 */
	sys_freq = pdata->refclk_hz / SYS_FREQ_DIVIDER;
	i2c_wr16(state, SYS_FREQ0, sys_freq);

	/* Audio system frequency is set as clock frequency divided by 100 */
	lock_ref_freq = pdata->refclk_hz / LOCK_FREQ_DIVIDER;
	i2c_wr8(state, LOCK_REF_FREQA, lock_ref_freq & MASK_LOCK_REF_FREQA);
	i2c_wr16(state, LOCK_REF_FREQB, lock_ref_freq >> 8);

	/* Audio PLL setting */
	i2c_wr8(state, NCO_F0_MOD, MASK_NCO_F0_MOD_REG);

	/* CSC clock is set as clock frequency divided by 10000 */
	csc_freq = pdata->refclk_hz / CSC_FREQ_DIVIDER;
	i2c_wr16(state, SCLK_CSC0, csc_freq);
}

/* HDMI interrupt */
static void tc358870_hdmi_interrupt(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr8(state, MISC_INT, 0xFF);
	i2c_wr8(state, MISC_INTM, ~MASK_SYNC_CHG);
	i2c_wr16(state, INT_STATUS, MASK_INT_STATUS_MASK_ALL);
	i2c_wr16(state, INT_MASK, MASK_INT_MASK_MASK_ALL);
}

/* Video out format */
static void tc358870_set_vout(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr8(state, VOUT_SYNC0, M3_VSIZE_INIT | MASK_MODE_2);
}

/* HDMI System */
static void tc358870_set_hdmi_system(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr8(state, DDC_CTL, MASK_DDC5V_MODE_100MS);
	i2c_wr8(state, HPD_CTL, MASK_HPD_CTL0);
}

/* HDMI Source start access */
static void tc358870_hdmi_start_access(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr8(state, INIT_END, MASK_INIT_END);
}

/* HDMI Audio refclk */
static void tc358870_hdmi_audio_refclk(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;
	struct tc358870_platform_data *pdata = &state->pdata;
	u32 nco_48;
	u32 nco_44;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr8(state, FORCE_MUTE, MASK_MUTE_OFF);
	i2c_wr8(state, AUTO_CMD0, MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 |
			MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
			MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
	i2c_wr8(state, AUTO_CMD1, MASK_AUTO_MUTE9);
	i2c_wr8(state, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
	i2c_wr8(state, BUFINIT_START, SET_BUFINIT_START_MS(500));
	i2c_wr8(state, FS_MUTE, MASK_FS_UNMUTE_ALL);
	i2c_wr8(state, SDO_MODE1, MASK_SDO_FMT_I2S);

	/* For 48 kHz, 6.144 MHz * 2^28 = 1649267442 */
	nco_48 = NCO_48_CALC / (pdata->refclk_hz / MHZ);
	i2c_wr32(state, NCO_48F0A, nco_48);

	/* For 44 kHz, 5.644 MHz * 2^28 = 1515264462 */
	nco_44 = NCO_44_CALC / (pdata->refclk_hz / MHZ);
	i2c_wr32(state, NCO_44F0A, nco_44);

	i2c_wr8(state, HDMIAUDIO_MODE, MASK_NORMAL_AUDIO);
}

/* HDMI end of RX init */
static void tc358870_hdmi_end_rxinit(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr16(state, CONFCTL0, MASK_VTX0EN | MASK_VTX1EN | MASK_AUTOINDEX |
				MASK_AUDOUTSEL_I2S | MASK_ABUFEN);
	i2c_wr16(state, CONFCTL1, MASK_TX_OUT_FMT_RGB888);
}

/* Command transmission after video */
static void tc358870_transmission_after_video(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr32(state, DSITX0_BASE_ADDR + MODECONF,
			MASK_VSYNC_POL_SW | MASK_HSYNC_POL_SW);
	i2c_wr32(state, DSITX1_BASE_ADDR + MODECONF,
			MASK_VSYNC_POL_SW | MASK_HSYNC_POL_SW);
}

/* EDID */
static void tc358870_set_edid(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* EDID should be set from DT, if not, do it here. */
}

static void tc358870_set_pll(struct tc358870_state *state,
				enum tc358870_dsi_port port)
{
	struct tc358870_platform_data *pdata = &state->pdata;
	struct i2c_client *client = state->i2c_client;
	u16 base_addr;
	u16 pll_frs;
	u32 pllconf;
	u32 hsck;

	dev_dbg(&client->dev, "%s:\n", __func__);

	WARN_ON((pdata->dsi_port <= DSI_TX_NONE) ||
		(pdata->dsi_port > DSI_TX_BOTH));

	if (pdata->dsi_port == DSI_TX_NONE) {
		dev_err(&client->dev, "%s: No DSI port defined!\n", __func__);
		return;
	}

	base_addr = (port == DSI_TX_0) ? DSITX0_BASE_ADDR :
		DSITX1_BASE_ADDR;
	pllconf = SET_PLL_PRD(pdata->pll_prd) |
		SET_PLL_FBD(pdata->pll_fbd);

	hsck = (pdata->refclk_hz / pdata->pll_prd) *
		pdata->pll_fbd;

	if (hsck > MHZ_500)
		pll_frs = 0x0;
	else if (hsck > MHZ_250)
		pll_frs = 0x1;
	else if (hsck > MHZ_125)
		pll_frs = 0x2;
	else
		pll_frs = 0x3;

	dev_dbg(&client->dev, "%s: Updating PLL clock of DSI TX%d\n",
			__func__, port-1);

	i2c_wr32(state, base_addr + PLLCONF, pllconf | SET_PLL_FRS(pll_frs));
}

static void tc358870_set_dsi(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;
	struct tc358870_platform_data *pdata = &state->pdata;
	struct panel_out *pout = &state->pout;
	unsigned lanes = pout->n_data_lanes / 2;

	enum tc358870_dsi_port port;
	u16 base_addr;

	dev_dbg(&client->dev, "%s:\n", __func__);

	for (port = DSI_TX_0; port <= DSI_TX_1; port++) {
		base_addr = (port == DSI_TX_0) ? DSITX0_BASE_ADDR :
			DSITX1_BASE_ADDR;

		if (pdata->dsi_port != DSI_TX_BOTH &&
				pdata->dsi_port != port) {

			dev_info(&client->dev,
				"%s: Disabling DSI TX%d\n", __func__, port - 1);

			/* Disable DSI lanes (high Z) */
			i2c_wr32_and_or(state, base_addr + LANEEN,
					~(MASK_CLANEEN), 0);
			continue;
		}

		dev_dbg(&client->dev, "%s: Enabling DSI TX%d\n",
			__func__, port - 1);

		/* 0x0108 */
		i2c_wr32(state, base_addr + DSITX_CLKEN, MASK_DSITX_EN);
		/* 0x010C */
		i2c_wr32(state, base_addr + PPI_CLKEN, MASK_HSTXCLKEN);
		/* 0x02A0 */
		i2c_wr32(state, base_addr + MIPI_CLKEN, MASK_MP_ENABLE);

		tc358870_set_pll(state, port);

		usleep_range(10000, 11000);
		/* 0x02A0 */
		i2c_wr32(state, base_addr + MIPI_CLKEN,
				MASK_MP_ENABLE | MASK_MP_CKEN);
		/* 0x0118 */
		i2c_wr32(state, base_addr + LANEEN,
				(lanes & MASK_LANES) | MASK_CLANEEN);
		/* 0x0120 */
		i2c_wr32(state, base_addr + LINEINITCNT, pdata->lineinitcnt);
		/* 0x0124 */
		i2c_wr32(state, base_addr + HSTOCNT, MASK_ZERO_32);
		/* 0x0128 */
		i2c_wr32(state, base_addr + FUNCEN,
				MASK_VH_DLY_EN | MASK_IND_MODE_SEL_REG);
		/* 0x0130 */
		i2c_wr32(state, base_addr + DSI_TATO_COUNT,
				MASK_DSI_TATO_COUNT_INIT);
		/* 0x0134 */
		i2c_wr32(state, base_addr + DSI_PRESP_BTA_COUNT,
				MASK_DSI_PRESP_BTA_COUNT_INIT);
		/* 0x0138 */
		i2c_wr32(state, base_addr + DSI_PRESP_LPR_COUNT,
				MASK_DSI_PRESP_LPR_COUNT_INIT);
		/* 0x013C */
		i2c_wr32(state, base_addr + DSI_PRESP_LPW_COUNT,
				DSI_PRESP_LPW_COUNT);
		/* 0x0140 */
		i2c_wr32(state, base_addr + DSI_PRESP_HSR_COUNT,
				DSI_PRESP_LPW_COUNT);
		/* 0x0144 */
		i2c_wr32(state, base_addr + DSI_PRESP_HSW_COUNT,
				DSI_PRESP_LPW_COUNT);
		/* 0x0148 */
		i2c_wr32(state, base_addr + DSI_PR_TO_COUNT,
				MASK_DSI_PR_TO_COUNT_INIT);
		/* 0x014C */
		i2c_wr32(state, base_addr + DSI_LRX_H_TO_COUNT,
				MASK_DSI_LRX_H_TO_COUNT_INIT);
		/* 0x0150 */
		i2c_wr32(state, base_addr + FUNCMODE,
				MASK_EO_TP_EN | MASK_CRC_DIS | MASK_ECC_DIS);
		/* 0x0154 */
		i2c_wr32(state, base_addr + DSIRX_VC_ENABLE, MASK_RXVC0_EN);
		/* 0x0158 */
		i2c_wr32(state, base_addr + IND_TO_COUNT,
				MASK_INT_TO_COUNT_INIT);
		/* 0x0168 */
		i2c_wr32(state, base_addr + DSI_HSYNC_STOP_COUNT,
				MASK_INIT_HSYNC_STOP_COUNT);
		/* 0x0170 */
		i2c_wr32(state, base_addr + APF_VDELAY_CNT,
				MASK_INIT_APF_DELAY);
		/* 0x017C */
		i2c_wr32(state, base_addr + DSITX_MODE,
				MASK_BLANKPKT_EN | MASK_DSITX_MODE_EVENT);
		/* 0x018C */
		i2c_wr32(state, base_addr + DSI_HSYNC_WIDTH,
				MASK_HSYNC_WIDTH_INIT);
		/* 0x0190 */
		i2c_wr32(state, base_addr + DSI_HBPR, MASK_HBPR_INIT);
		/* 0x01A4 */
		i2c_wr32(state, base_addr + DSI_RX_STATE_INT_MASK,
				MASK_ZERO_32);
		/* 0x01C0 */
		i2c_wr32(state, base_addr + DSI_LPRX_THRESH_COUNT,
				MASK_LPRX_THRESH_COUNT_INIT);
		/* 0x0214 */
		i2c_wr32(state, base_addr + APP_SIDE_ERR_INT_MASK,
				MASK_ZERO_32);
		/* 0x021C */
		i2c_wr32(state, base_addr + DSI_RX_ERR_INT_MASK,
				MASK_ERR_REPORT_7);
		/* 0x0224 */
		i2c_wr32(state, base_addr + DSI_LPTX_INT_MASK, MASK_ZERO_32);
		/* 0x0254 */
		i2c_wr32(state, base_addr + LPTXTIMECNT, pdata->lptxtimecnt);
		/* 0x0258 */
		i2c_wr32(state, base_addr + TCLK_HEADERCNT,
				pdata->tclk_headercnt);
		/* 0x025C */
		i2c_wr32(state, base_addr + TCLK_TRAILCNT,
				pdata->tclk_trailcnt);
		/* 0x0260 */
		i2c_wr32(state, base_addr + THS_HEADERCNT,
				pdata->ths_headercnt);
		/* 0x0264 */
		i2c_wr32(state, base_addr + TWAKEUP, pdata->twakeup);
		/* 0x0268 */
		i2c_wr32(state, base_addr + TCLK_POSTCNT, pdata->tclk_postcnt);
		/* 0x026C */
		i2c_wr32(state, base_addr + THS_TRAILCNT, pdata->ths_trailcnt);
		/* 0x0270 */
		i2c_wr32(state, base_addr + HSTXVREGCNT, pdata->hstxvregcnt);
		/* 0x0274 */
		i2c_wr32(state, base_addr + HSTXVREGEN,
				((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0x0) |
				((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0x0) |
				((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0x0) |
				((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0x0) |
				((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0x0));
		/* 0x0278 */
		i2c_wr32(state, base_addr + PPI_DSI_BTA_COUNT, pdata->btacnt);
		/* 0x027C */
		i2c_wr32(state, base_addr + PPI_DPHYTX_ADJUST,
				MASK_LPTX_25_OUT_CUR);
		/* 0x011C */
		i2c_wr32(state, base_addr + DSITX_START, MASK_DSITX_START);
	}
}

/* Panel functions */
static int dcs_panel_command(struct tc358870_state *state,
		struct tegra_dsi_cmd *cmd, u32 n_cmd)
{
	struct i2c_client *client = state->i2c_client;
	struct tegra_dsi_cmd *cur_cmd = NULL;
	int err = 0;
	u32 index = 0;
	u16 data0 = 0, data1, len = 0;
	u8 *pdata = NULL;
	u16 status = 0;

	dev_dbg(&client->dev, "%s n_cmd is %d\n", __func__, n_cmd);
	if (cmd == NULL || n_cmd == 0) {
		dev_err(&client->dev, "%s: cmd is empty\n", __func__);
		err = -EINVAL;
		return err;
	}

	for (index = 0; index < n_cmd; index++) {
		cur_cmd = &cmd[index];

		status = i2c_rd16(state, DCSCMD_ST);
		dev_dbg(&client->dev, "dcs status = 0x%04x index = %d\n",
			status, index);

		if (cur_cmd->cmd_type == TEGRA_DSI_DELAY_MS) {
			/* DSI Delay Command in milliseconds */
			usleep_range(cur_cmd->sp_len_dly.delay_ms * 1000,
				(cur_cmd->sp_len_dly.delay_ms * 1000) + 500);
		} else if (cur_cmd->cmd_type == TEGRA_DSI_DELAY_US) {
			/* DSI Delay Command in microseconds */
			usleep_range(cur_cmd->sp_len_dly.delay_us,
				(cur_cmd->sp_len_dly.delay_us) + 100);
		} else if (cur_cmd->cmd_type == TEGRA_DSI_PACKET_CMD) {
			/* DSI Packet Command */
			if (cur_cmd->data_id == DSI_DCS_WRITE_0_PARAM) {
				/* DSI_DCS_WRITE_0_PARAM */
				data0 = cur_cmd->sp_len_dly.sp.data0 &
					DATA0_MASK;

				i2c_wr16(state, DCSCMD_Q,
					DSI_DCS_WRITE_0_PARAM);
				i2c_wr16(state, DCSCMD_Q, data0);
				dev_dbg(&client->dev, "0x%04x 0x%04x\n",
					DSI_DCS_WRITE_0_PARAM, data0);
			} else if (cur_cmd->data_id == DSI_DCS_WRITE_1_PARAM ||
					cur_cmd->data_id ==
					DSI_GENERIC_SHORT_WRITE_2_PARAMS) {
				/* DSI_DCS_WRITE_1_PARAM */
				data0 = cur_cmd->sp_len_dly.sp.data0 &
					DATA0_MASK;
				data1 = (cur_cmd->sp_len_dly.sp.data1 << 8) &
					DATA1_MASK;

				i2c_wr16(state, DCSCMD_Q, cur_cmd->data_id);
				i2c_wr16(state, DCSCMD_Q, data1 | data0);
				dev_dbg(&client->dev, "0x%04x 0x%04x\n",
					cur_cmd->data_id, data1 | data0);
			} else if (cur_cmd->data_id == DSI_DCS_LONG_WRITE) {
				/* DSI_DCS_LONG_WRITE */

				len = cur_cmd->sp_len_dly.data_len;
				pdata = cur_cmd->pdata;

				if (len == 0 && pdata == NULL) {
					/* Short packet with no payload */
					i2c_wr16(state, DCSCMD_Q,
						DSI_DCS_LONG_WRITE);
					i2c_wr16(state, DCSCMD_Q, data0);
				} else {
					i2c_wr16(state, DCSCMD_Q,
					LONG_PKT_MASK | DSI_DCS_LONG_WRITE);
					while (len) {
						if (len >= 2) {
							data0 =
							((u16 *) pdata)[0];
							len -= 2;
							pdata += 2;
						} else {
							data0 =
							((u8 *) pdata)[0] &
								DATA0_MASK;
							pdata += len;
							len = 0;
						}

						i2c_wr16(state, DCSCMD_Q,
							data0);
						dev_dbg(&client->dev,
							"0x%04x 0x%04x\n",
							LONG_PKT_MASK |
							DSI_DCS_LONG_WRITE,
							data0);
					}
				}
			}

		}
	}
	return err;
}

/* DCS commands post transmission */
static int dcs_post_transmission(struct tc358870_state *state)
{
	int err = 0;
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	err = dcs_panel_command(state, state->pout.dsi_postvideo_cmd,
				state->pout.n_postvideo_cmd);

	return err;
}

static int dcs_enable(struct tc358870_state *state)
{
	int err = 0;
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	i2c_wr16(state, DCSCMD_SEL, MASK_CMD_SEL_BOTH);
	i2c_wr32(state, DSITX0_BASE_ADDR + MODECONF,
			MASK_VSYNC_POL_SW | MASK_HSYNC_POL_SW | MASK_INDMODE);
	i2c_wr32(state, DSITX1_BASE_ADDR + MODECONF,
			MASK_VSYNC_POL_SW | MASK_HSYNC_POL_SW | MASK_INDMODE);

	err = dcs_panel_command(state, state->pout.dsi_init_cmd,
				state->pout.n_init_cmd);

	return err;
}

static int dcs_disable(struct tc358870_state *state)
{
	int err = 0;
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	err = dcs_panel_command(state, state->pout.dsi_suspend_cmd,
				state->pout.n_suspend_cmd);

	return err;
}

#ifdef CONFIG_OF
static struct tegra_dsi_cmd *parse_cmd_dt(struct i2c_client *client,
		const struct device_node *node,
		struct property *prop,
		u32 n_cmd)
{
	struct tegra_dsi_cmd *dsi_cmd = NULL;

	dsi_cmd = dsi_parse_cmd_dt(&client->dev, node, prop, n_cmd);

	return dsi_cmd;
}

static bool panel_parse_dt(struct panel_out *pout,
		struct i2c_client *client)
{
	struct device_node *np_panel = NULL;
	u32 temp;
	bool ret = true;

	np_panel = of_get_next_child(client->dev.of_node, NULL);

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-n-data-lanes", &temp)) {
		pout->n_data_lanes = (u8)temp;
		dev_dbg(&client->dev, "n data lanes %d\n", pout->n_data_lanes);
	}

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-pixel-format", &temp)) {
		pout->pixel_format = (u8)temp;
		dev_dbg(&client->dev, "pixel format %d\n", pout->pixel_format);
	}

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-refresh-rate", &temp)) {
		pout->refresh_rate = (u8)temp;
		dev_dbg(&client->dev, "refresh rate %d\n", pout->refresh_rate);
	}

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-n-init-cmd", &temp)) {
		pout->n_init_cmd = (u16)temp;
		dev_dbg(&client->dev, "n init cmd %d\n", pout->n_init_cmd);
	}

	pout->dsi_init_cmd = parse_cmd_dt(client, np_panel,
			of_find_property(np_panel, "nvidia,dsi-init-cmd", NULL),
			pout->n_init_cmd);
	if (pout->n_init_cmd &&
			IS_ERR_OR_NULL(pout->dsi_init_cmd)) {
		dev_err(&client->dev, "dsi: copy init cmd from dt failed\n");
		ret = false;
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-n-postvideo-cmd", &temp)) {
		pout->n_postvideo_cmd = (u16)temp;
		dev_dbg(&client->dev, "n postvideo cmd %d\n",
			pout->n_postvideo_cmd);
	}

	pout->dsi_postvideo_cmd = parse_cmd_dt(client, np_panel,
				of_find_property(np_panel,
					"nvidia,dsi-postvideo-cmd", NULL),
			pout->n_postvideo_cmd);
	if (pout->n_postvideo_cmd &&
			IS_ERR_OR_NULL(pout->dsi_postvideo_cmd)) {
		dev_err(&client->dev, "dsi: copy init cmd from dt failed\n");
		ret = false;
		goto parse_dsi_settings_fail;
	};

	if (!of_property_read_u32(np_panel,
				"nvidia,dsi-n-suspend-cmd", &temp)) {
		pout->n_suspend_cmd = (u16)temp;
		dev_dbg(&client->dev, "n suspend cmd %d\n",
				pout->n_suspend_cmd);
	}

	pout->dsi_suspend_cmd = parse_cmd_dt(client, np_panel,
			of_find_property(np_panel,
				"nvidia,dsi-suspend-cmd", NULL),
			pout->n_suspend_cmd);
	if (pout->n_suspend_cmd &&
			IS_ERR_OR_NULL(pout->dsi_suspend_cmd)) {
		dev_err(&client->dev, "dsi: copy suspend cmd from dt failed\n");
		ret = false;
		goto parse_dsi_settings_fail;
	};

parse_dsi_settings_fail:
	of_node_put(np_panel);
	return ret;

}

static bool tc358870_parse_dt(struct tc358870_platform_data *pdata,
		struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	const u32 *property;

	dev_dbg(&client->dev, "Device Tree Parameters:\n");

	pdata->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (pdata->reset_gpio == 0)
		return false;
	dev_dbg(&client->dev, "reset_gpio = %d\n", pdata->reset_gpio);

	property = of_get_property(node, "bridge-instance", NULL);
	if (property == NULL)
		return false;
	pdata->bridge_instance = be32_to_cpup(property);
	dev_dbg(&client->dev, "bridge_instance = %d\n",
			pdata->bridge_instance);

	property = of_get_property(node, "refclk_hz", NULL);
	if (property == NULL)
		return false;
	pdata->refclk_hz = be32_to_cpup(property);
	dev_dbg(&client->dev, "refclk_hz = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "ddc5v_delay", NULL);
	if (property == NULL)
		return false;
	pdata->ddc5v_delay = be32_to_cpup(property);
	if (pdata->ddc5v_delay > DDC5V_DELAY_MAX)
		pdata->ddc5v_delay = DDC5V_DELAY_MAX;
	dev_dbg(&client->dev, "ddc5v_delay = %d ms\n",
			50 * pdata->ddc5v_delay);

	property = of_get_property(node, "enable_hdcp", NULL);
	if (property == NULL)
		return false;
	pdata->enable_hdcp = be32_to_cpup(property);
	dev_dbg(&client->dev, "enable_hdcp = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "dsi_port", NULL);
	if (property == NULL)
		return false;
	pdata->dsi_port = be32_to_cpup(property);
	dev_dbg(&client->dev, "dsi_port = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lineinitcnt", NULL);
	if (property == NULL)
		return false;
	pdata->lineinitcnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "lineinitcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "lptxtimecnt", NULL);
	if (property == NULL)
		return false;
	pdata->lptxtimecnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "lptxtimecnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_headercnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_headercnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "tclk_headercnt = %d\n",
			be32_to_cpup(property));

	property = of_get_property(node, "tclk_trailcnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_trailcnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "tclk_trailcnt = %d\n",
			be32_to_cpup(property));

	property = of_get_property(node, "ths_headercnt", NULL);
	if (property == NULL)
		return false;
	pdata->ths_headercnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "ths_headercnt = %d\n",
			be32_to_cpup(property));

	property = of_get_property(node, "twakeup", NULL);
	if (property == NULL)
		return false;
	pdata->twakeup = be32_to_cpup(property);
	dev_dbg(&client->dev, "twakeup = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "tclk_postcnt", NULL);
	if (property == NULL)
		return false;
	pdata->tclk_postcnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "tclk_postcnt = %d\n",
			be32_to_cpup(property));

	property = of_get_property(node, "ths_trailcnt", NULL);
	if (property == NULL)
		return false;
	pdata->ths_trailcnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "ths_trailcnt = %d\n",
			be32_to_cpup(property));

	property = of_get_property(node, "hstxvregcnt", NULL);
	if (property == NULL)
		return false;
	pdata->hstxvregcnt = be32_to_cpup(property);
	dev_dbg(&client->dev, "hstxvregcnt = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_prd", NULL);
	if (property == NULL)
		return false;
	pdata->pll_prd = be32_to_cpup(property);
	dev_dbg(&client->dev, "pll_prd = %d\n", be32_to_cpup(property));

	property = of_get_property(node, "pll_fbd", NULL);
	if (property == NULL)
		return false;
	pdata->pll_fbd = be32_to_cpup(property);
	dev_dbg(&client->dev, "pll_fbd = %d\n", be32_to_cpup(property));

	return true;
}
#endif

static int tc358870_pwr_init(struct tc358870_platform_data *pdata,
		struct i2c_client *client)
{
	int err = 0;

	pdata->iovdd = regulator_get(&client->dev, "vdd-boe-1v8");
	if (IS_ERR_OR_NULL(pdata->iovdd)) {
		dev_err(&client->dev, "cannot get regulator vdd-boe-1v8\n");
		err = PTR_ERR(pdata->iovdd);
		pdata->iovdd = NULL;
		goto fail;
	}

	pdata->dvdd = regulator_get(&client->dev, "vdd-boe-1v2");
	if (IS_ERR_OR_NULL(pdata->dvdd)) {
		dev_err(&client->dev, "cannot get regulator vdd-boe-1v2\n");
		err = PTR_ERR(pdata->dvdd);
		pdata->dvdd = NULL;
		goto dvdd_fail;
	}

	if (pdata->dvdd) {
		err = regulator_enable(pdata->dvdd);
		if (err < 0) {
			dev_err(&client->dev,
				"cannot enable vdd-boe-1v8 %d\n", err);
			return -EINVAL;
		}
	}

	if (pdata->iovdd) {
		err = regulator_enable(pdata->iovdd);
		if (err < 0) {
			dev_err(&client->dev,
				"cannot enable vdd-boe-1v2 %d\n", err);
			return -EINVAL;
		}
	}

	return err;

dvdd_fail:
	if (pdata->iovdd) {
		regulator_put(pdata->iovdd);
		pdata->iovdd = NULL;
	}

fail:
	return err;

}

static int tc358870_verify_chipid(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;
	u16 cid = 0;

	cid = i2c_rd16(state, CHIPID_ADDR);
	if (cid != TC358870_CHIPID) {
		dev_err(&client->dev, "Invalid chip ID 0x%04X\n", cid);
		return -ENODEV;
	}

	dev_dbg(&client->dev, "TC358870 ChipID 0x%02x, Revision 0x%02x\n",
			(cid & MASK_CHIPID) >> 8, cid & MASK_REVID);

	return 0;
}

static int hdmi2dsi_tc358870_en_gpio(struct tc358870_state *state,
		bool enable)
{
	int err = 0;
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "Setting reset-gpio (gpio 0x%04X)\n",
		state->pdata.reset_gpio);

	err = gpio_is_valid(state->pdata.reset_gpio);
	if (!err) {
		dev_err(&client->dev, "reset GPIO is invalid!\n");
		return err;
	}
	if (enable)
		err = gpio_direction_output(state->pdata.reset_gpio,
				GPIOF_OUT_INIT_HIGH);
	else
		err = gpio_direction_output(state->pdata.reset_gpio,
				GPIOF_OUT_INIT_LOW);

	if (err < 0) {
		dev_err(&client->dev, "Failed to set reset GPIO 0x%04X: %d\n",
			state->pdata.reset_gpio, err);
		return err;
	}
	return err;
}

static void tc358870_initial_setup(struct tc358870_state *state)
{
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	tc358870_set_ctl(state);
	tc358870_set_dsi(state);
	tc358870_split_control(state);
	tc358870_hdmi_phy(state, true);
	tc358870_hdmi_clock(state);
	tc358870_hdmi_interrupt(state);
	tc358870_set_edid(state);
	tc358870_set_vout(state);
	tc358870_set_hdmi_system(state);
	tc358870_hdmi_audio_refclk(state);

	state->enabled = true;
	state->power_down = false;
}

static int hdmi2dsi_tc358870_init(struct tegra_hdmi *hdmi)
{
	int sor_num = 0;
	int err = 0;
	/* map the right bridge instance to hdmi instance.
	 * hdmi is identified by the sor_num and the bridge is
	 * identified by the bridge instance parsed from bridge DT
	 */
	struct tc358870_state *state = gstate[sor_num];
	struct i2c_client *client = NULL;

	if (hdmi == NULL) {
		pr_err("hdmi is NULL %s\n", __func__);
		return 0;
	}
	if (hdmi->dc->out_ops && hdmi->dc->out_ops->get_connector_instance)
		sor_num = hdmi->dc->out_ops->get_connector_instance(hdmi->dc);

	state = gstate[sor_num];

	if (state == NULL) {
		pr_err("state is NULL from the global %s\n", __func__);
		return -ENODEV;
	}
	client = state->i2c_client;

	state->hdmi = hdmi;
	state->mode = &hdmi->dc->mode;
	tegra_hdmi_set_outdata(hdmi, state);
	tc358870_initial_setup(state);

	return err;
}

static void hdmi2dsi_tc358870_destroy(struct tegra_hdmi *hdmi)
{
	struct tc358870_state *state = tegra_hdmi_get_outdata(hdmi);

	if (!state)
		return;

	hdmi2dsi_tc358870_en_gpio(state, false);
	mutex_destroy(&state->lock);
}

static void hdmi2dsi_tc358870_enable(struct tegra_hdmi *hdmi)
{
	struct tc358870_state *state = tegra_hdmi_get_outdata(hdmi);
	struct i2c_client *client = NULL;

	if (state && state->enabled) {
		state->enabled = false;
		return;
	}

	if (state)
		client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&state->lock);
	hdmi2dsi_tc358870_en_gpio(state, true);

	if (state->power_down == true) {
		tc358870_set_ctl(state);
		tc358870_set_dsi(state);
		tc358870_hdmi_phy(state, true);
	}

	i2c_wr8(state, VI_MUTE, MASK_AUTO_MUTE);

	if (state->out_ops && state->out_ops->enable)
		state->out_ops->enable(&state->i2c_client->dev);

	dcs_enable(state);

	tc358870_hdmi_start_access(state);
	tc358870_hdmi_end_rxinit(state);
	tc358870_transmission_after_video(state);

	msleep(1000);

	dcs_post_transmission(state);

	state->enabled = true;

	mutex_unlock(&state->lock);
}

static void hdmi2dsi_tc358870_disable(struct tegra_hdmi *hdmi)
{
	struct tc358870_state *state = tegra_hdmi_get_outdata(hdmi);
	struct i2c_client *client = state->i2c_client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&state->lock);

	if (state->enabled) {

		/* Suspend sequence for the panel.
		 * Set display off, enter sleep mode, etc... (panel specific)
		 */
		dcs_disable(state);

		/* Disable panel regulator, reset pin */
		if (state->out_ops && state->out_ops->disable)
			state->out_ops->disable(&state->i2c_client->dev);

		/* Disable video TX 0/1 enable */
		i2c_wr16(state, CONFCTL0, MASK_AUTOINDEX);

		/* Disable MIPI PLL/clock enable */
		i2c_wr32(state, DSITX0_BASE_ADDR + MIPI_CLKEN, 0x00000000);
		i2c_wr32(state, DSITX1_BASE_ADDR + MIPI_CLKEN, 0x00000000);

		/* Power down HDMI phy */
		tc358870_hdmi_phy(state, false);

		state->enabled = false;
		state->power_down = true;
	}
	mutex_unlock(&state->lock);
}

#ifdef CONFIG_PM
static void hdmi2dsi_tc358870_suspend(struct tegra_hdmi *hdmi)
{
	hdmi2dsi_tc358870_disable(hdmi);
}

static void hdmi2dsi_tc358870_resume(struct tegra_hdmi *hdmi)
{
	hdmi2dsi_tc358870_enable(hdmi);
}
#endif

struct tegra_hdmi_out_ops tegra_hdmi2dsi_ops = {
	.init = hdmi2dsi_tc358870_init,
	.destroy = hdmi2dsi_tc358870_destroy,
	.enable = hdmi2dsi_tc358870_enable,
	.disable = hdmi2dsi_tc358870_disable,
#ifdef CONFIG_PM
	.resume = hdmi2dsi_tc358870_resume,
	.suspend = hdmi2dsi_tc358870_suspend,
#endif	/* CONFIG_PM */
};

/* Global variables for debugfs */
static u16 d_bridge_id;
static u16 d_reg_addr;
static u16 read_size;
static u32 d_reg_value;

/* debugfs - read register */
static int tc358870_hdmi2dsi_regs_show(struct seq_file *s, void *data)
{
	u32 value;
	struct tc358870_state *state = gstate[d_bridge_id];

	mutex_lock(&state->lock);

	if (read_size == 8) {
		value = i2c_rd8(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%02x\n",
				d_reg_addr, (u8)value);
	} else if (read_size == 16) {
		value = i2c_rd16(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%04x\n",
				d_reg_addr, (u16)value);
	} else if (read_size == 32) {
		value = i2c_rd32(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%08x\n",
				d_reg_addr, value);
	}

	mutex_unlock(&state->lock);
	return 0;
}

/* debugfs - write register */
static int tc358870_hdmi2dsi_reg_write(struct seq_file *s, void *data)
{
	u32 value;
	struct tc358870_state *state = gstate[d_bridge_id];

	mutex_lock(&state->lock);

	if (read_size == 8) {
		i2c_wr8(state, d_reg_addr, (u8) d_reg_value);
		value = i2c_rd8(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%02x\n",
				d_reg_addr, (u8)value);
	} else if (read_size == 16) {
		i2c_wr16(state, d_reg_addr, (u16) d_reg_value);
		value = i2c_rd16(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%04x\n",
				d_reg_addr, (u16)value);
	} else if (read_size == 32) {
		i2c_wr32(state, d_reg_addr, (u32) d_reg_value);
		value = i2c_rd32(state, d_reg_addr);
		seq_printf(s, "reg = 0x%04x value = 0x%08x\n",
				d_reg_addr, value);
	}

	mutex_unlock(&state->lock);

	return 0;
}

static int tc358870_hdmi2dsi_reg_read_open(struct inode *inode,
						struct file *file)
{
	return single_open(file, tc358870_hdmi2dsi_regs_show, inode->i_private);
}

static int tc358870_hdmi2dsi_reg_write_open(struct inode *inode,
						struct file *file)
{
	return single_open(file, tc358870_hdmi2dsi_reg_write, inode->i_private);
}

static const struct file_operations reg_read_fops = {
	.open           = tc358870_hdmi2dsi_reg_read_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static const struct file_operations reg_write_fops = {
	.open           = tc358870_hdmi2dsi_reg_write_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void tc358870_hdmi2dsi_remove_debugfs(struct tc358870_state *state)
{
	debugfs_remove_recursive(state->debugdir);
};

static int tc358870_hdmi2dsi_create_debugfs(struct tc358870_state *state)
{
	struct dentry *pentry = NULL;
	int ret = 0;

	state->debugdir = debugfs_create_dir("tc358870_hdmi2dsi", NULL);

	if (!state->debugdir) {
		ret = -ENOTDIR;
		goto err;
	}
	pentry = debugfs_create_u16("bridge-id", 0644,
			state->debugdir, &d_bridge_id);
	pentry = debugfs_create_u16("addr", 0644,
			state->debugdir, &d_reg_addr);
	pentry = debugfs_create_u16("read_size", 0644,
			state->debugdir, &read_size);
	pentry = debugfs_create_u32("value", 0644,
			state->debugdir, &d_reg_value);
	pentry = debugfs_create_file("show", 0644,
			state->debugdir, state, &reg_read_fops);
	pentry = debugfs_create_file("set", 0644,
			state->debugdir, state, &reg_write_fops);

	if (pentry == NULL) {
		ret = ENOENT;
		goto err;
	}
	return ret;
err:
	tc358870_hdmi2dsi_remove_debugfs(state);
	dev_err(&state->i2c_client->dev, "Failed to create debugfs\n");
	return ret;
}

/* i2c driver ops */
static int tc358870_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct tc358870_state *state;
	struct device_node *panel_node;
	int i = 0;
	int err = 0;
	bool reset = true;

	state = devm_kzalloc(&client->dev,
			sizeof(struct tc358870_state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	if (&client->dev.of_node) {
		if (!tc358870_parse_dt(&state->pdata, client)) {
			dev_err(&client->dev,
					"Couldn't parse bridge device tree\n");
			devm_kfree(&client->dev, state);
			return -ENODEV;
		}

		if (!panel_parse_dt(&state->pout, client)) {
			dev_err(&client->dev,
					"Couldn't parse panel device tree\n");
			devm_kfree(&client->dev, state);
			return -ENODEV;
		}
		err = tc358870_pwr_init(&state->pdata, client);
		if (err) {
			dev_err(&client->dev,
					"Couldn't power init %d\n", err);
			devm_kfree(&client->dev, state);
			return -ENODEV;
		}
	} else {
		if (!client->dev.platform_data) {
			dev_err(&client->dev, "No platform data!\n");
			devm_kfree(&client->dev, state);
			return -ENODEV;
		}
		state->pdata = *(struct tc358870_platform_data *)
			client->dev.platform_data;
	}

	gstate[state->pdata.bridge_instance] = state;
	greset_gpio[state->pdata.bridge_instance] = state->pdata.reset_gpio;
	state->i2c_client = client;

	/* The panel node is the child node of the bridge */
	panel_node = of_get_next_child(client->dev.of_node, NULL);
	if (panel_node) {
		/*Get panel_ops for the panel_node */
		state->out_ops = tegra_dc_get_panel_ops(panel_node);
		if (!state->out_ops) {
			dev_err(&client->dev, "No panel ops found\n");
			devm_kfree(&client->dev, state);
			return -ENODATA;
		}
	} else {
		dev_err(&client->dev, "No panel node\n");
		devm_kfree(&client->dev, state);
		return -ENODEV;
	}

	/* Check if the gpio is already requested, for cases where
	 * multiple bridge instance share the same gpio. If so, skip request
	 */
	for (i = 0; i < MAX_BRIDGE_INSTANCES; i++) {
		if (i == state->pdata.bridge_instance)
			continue;

		if ((gstate[i]) &&
			(gstate[i]->pdata.reset_gpio ==
			 state->pdata.reset_gpio))
			reset = false;

	}
	if (reset) {
		err = devm_gpio_request_one(&client->dev,
				state->pdata.reset_gpio,
				GPIOF_OUT_INIT_HIGH, "tc358870-reset");
		if (err) {
			dev_err(&client->dev,
				"Failed to request Reset GPIO 0x%04X: %d\n",
				state->pdata.reset_gpio, err);
			devm_kfree(&client->dev, state);
			return err;
		}
		hdmi2dsi_tc358870_en_gpio(state, true);
	}

	i2c_set_clientdata(client, state);

	/* Verify chip ID */
	err = tc358870_verify_chipid(state);
	if (err)
		return err;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE_DATA)) {
		devm_kfree(&client->dev, state);
		return -EIO;
	}

	dev_dbg(&client->dev, "Chip found @ 7h%02X (%s)\n",
			client->addr, client->adapter->name);

	mutex_init(&state->lock);

	dev_info(&client->dev, "%s found @ 7h%02X (%s)\n", client->name,
			client->addr, client->adapter->name);

	tc358870_hdmi2dsi_create_debugfs(state);

	return err;
}

static int tc358870_remove(struct i2c_client *client)
{
	struct tc358870_state *state = to_state(client);

	tc358870_hdmi2dsi_remove_debugfs(state);
	devm_kfree(&client->dev, state);

	return 0;
}

static const struct i2c_device_id tc358870_id[] = {
	{ "tc358870", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc358870_id);

#ifdef CONFIG_OF
static const struct of_device_id tc358870_of_table[] = {
	{ .compatible = "toshiba,tc358870" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358870_of_table);
#endif

static struct i2c_driver tc358870_driver = {
	.driver = {
		.of_match_table = of_match_ptr(tc358870_of_table),
		.name = "tc358870",
		.owner = THIS_MODULE,
	},
	.probe = tc358870_probe,
	.remove = tc358870_remove,
	.id_table = tc358870_id,
};

module_i2c_driver(tc358870_driver);

MODULE_DESCRIPTION("Driver for Toshiba TC358870 HDMI to DSI Bridge");
MODULE_AUTHOR("Ishwary Balaji Gururajan (igururajan@nvidia.com)");
MODULE_LICENSE("GPL v2");
