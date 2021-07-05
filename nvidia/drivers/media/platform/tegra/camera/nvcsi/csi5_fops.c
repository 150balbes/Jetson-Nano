/*
 * Tegra CSI5 device common APIs
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <media/csi.h>
#include <media/mc_common.h>
#include <media/csi5_registers.h>
#include "nvhost_acm.h"
#include "nvcsi/nvcsi.h"
#include "csi5_fops.h"
#include <linux/tegra-capture-ivc.h>
#include <linux/nospec.h>
#include "soc/tegra/camrtc-capture-messages.h"

#include "mipical/mipi_cal.h"

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)

#define NVCSI_CIL_CLOCK_RATE 204000

#define TEMP_CHANNEL_ID (NUM_CAPTURE_CHANNELS + 1)
#define TPG_HBLANK 0
#define TPG_VBLANK 40800

/*
 * T19x TPG is generating 64 bits per cycle
 * it will insert (TPG_LANE_NUM-8) * nvcsi_clock cycles between
 * two 64bit pixel_packages to reduce framerate
 * TPG_LANE_NUM=8 means no blank insertion.
 * 7 means insert 1 clock between two 64bit pixel packages,
 * 6 means 2 clocks blank, …, 1 means 7 blank clocks.
 */
#define TPG_BLANK 6

static void csi5_phy_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base +
		CSI5_BASE_ADDRESS + (CSI5_PHY_OFFSET * index) + addr);
}

static inline u32 csi5_port_to_stream(u32 csi_port)
{
	return (csi_port < NVCSI_PORT_E) ?
		csi_port : (((csi_port - NVCSI_PORT_E) >> 1U) + NVCSI_PORT_E);
}

static int csi5_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	dev_dbg(csi->dev, "%s\n", __func__);

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi5_power_off(struct tegra_csi_device *csi)
{
	dev_dbg(csi->dev, "%s\n", __func__);

	nvhost_module_idle(csi->pdev);

	return 0;
}

static int csi5_stream_open(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%u, csi_port=%u\n",
		__func__, stream_id, csi_port);

	/* Open NVCSI stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_OPEN_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.phy_stream_open_req.stream_id = stream_id;
	msg.phy_stream_open_req.csi_port = csi_port;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return 0;
}

static void csi5_stream_close(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%u, csi_port=%u\n",
		__func__, stream_id, csi_port);

	/* Close NVCSI stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_PHY_STREAM_CLOSE_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.phy_stream_close_req.stream_id = stream_id;
	msg.phy_stream_close_req.csi_port = csi_port;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));
}

static int csi5_stream_set_config(struct tegra_csi_channel *chan, u32 stream_id,
	u32 csi_port, int csi_lanes)
{
	struct tegra_csi_device *csi = chan->csi;

	struct camera_common_data *s_data = chan->s_data;

	unsigned int cil_settletime = read_settle_time_from_dt(chan);
	unsigned int discontinuous_clk = read_discontinuous_clk_from_dt(chan);

	struct CAPTURE_CONTROL_MSG msg;
	struct nvcsi_brick_config brick_config;
	struct nvcsi_cil_config cil_config;
	bool is_cphy = (csi_lanes == 3);

	dev_dbg(csi->dev, "%s: stream_id=%u, csi_port=%u\n",
		__func__, stream_id, csi_port);

	/* Brick config */
	memset(&brick_config, 0, sizeof(brick_config));
	brick_config.phy_mode = (!is_cphy) ?
		NVCSI_PHY_TYPE_DPHY : NVCSI_PHY_TYPE_CPHY;

	/* CIL config */
	memset(&cil_config, 0, sizeof(cil_config));
	cil_config.num_lanes = csi_lanes;
	if (is_cphy)
		cil_config.lp_bypass_mode = 0;
	else
		cil_config.lp_bypass_mode = !discontinuous_clk;

	cil_config.t_clk_settle = is_cphy ? 1 : 33;
	cil_config.t_hs_settle = cil_settletime;
	cil_config.cil_clock_rate = NVCSI_CIL_CLOCK_RATE; /* hard-coding */

	if (s_data && !chan->pg_mode)
		cil_config.mipi_clock_rate = read_pixel_clk_from_dt(chan)/ 1000;
	else
		cil_config.mipi_clock_rate = csi->clk_freq / 1000;

	/* Set NVCSI stream config */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_SET_CONFIG_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_set_config_req.stream_id = stream_id;
	msg.csi_stream_set_config_req.csi_port = csi_port;
	msg.csi_stream_set_config_req.brick_config = brick_config;
	msg.csi_stream_set_config_req.cil_config = cil_config;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return 0;
}

static int csi5_stream_tpg_start(struct tegra_csi_channel *chan, u32 stream_id,
	u32 virtual_channel_id)
{
	int err = 0;
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[0];
	unsigned long csi_rate = 0;

	/* TPG native resolution */
	const size_t px_max = 0x4000;
	const size_t py_max = 0x2000;
	size_t hfreq = 0;
	size_t vfreq = 0;

	struct CAPTURE_CONTROL_MSG msg;
	union nvcsi_tpg_config *tpg_config = NULL;

	dev_dbg(csi->dev, "%s: stream_id=%u, virtual_channel_id=%d\n",
		__func__, stream_id, virtual_channel_id);

	/* Set TPG config for a virtual channel */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_SET_CONFIG_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	hfreq = px_max / port->format.width;
	vfreq = py_max / port->format.height;

	tpg_config = &(msg.csi_stream_tpg_set_config_req.tpg_config);

	tpg_config->t194.virtual_channel_id = virtual_channel_id;
	tpg_config->t194.datatype = port->core_format->img_dt;

	tpg_config->t194.lane_count = TPG_BLANK;
	tpg_config->t194.flags = NVCSI_TPG_FLAG_PATCH_MODE;

	tpg_config->t194.initial_frame_number = 1;
	tpg_config->t194.maximum_frame_number = 32768;
	tpg_config->t194.image_width = port->format.width;
	tpg_config->t194.image_height = port->format.height;

	tpg_config->t194.red_horizontal_init_freq = hfreq;
	tpg_config->t194.red_vertical_init_freq = vfreq;

	tpg_config->t194.green_horizontal_init_freq = hfreq;
	tpg_config->t194.green_vertical_init_freq = vfreq;

	tpg_config->t194.blue_horizontal_init_freq = hfreq;
	tpg_config->t194.blue_vertical_init_freq = vfreq;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	/* Enable TPG on a stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_START_RATE_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_tpg_start_req.stream_id = stream_id;
	msg.csi_stream_tpg_start_req.virtual_channel_id = virtual_channel_id;
	msg.csi_stream_tpg_start_rate_req.frame_rate = port->framerate;
	err = nvhost_module_get_rate(csi->pdev, &csi_rate, 0);
	if (err)
		return err;

	msg.csi_stream_tpg_start_rate_req.csi_clk_rate = csi_rate / 1000;
	tegra_capture_ivc_control_submit(&msg, sizeof(msg));

	return err;
}

static void csi5_stream_tpg_stop(struct tegra_csi_channel *chan, u32 stream_id,
	u32 virtual_channel_id)
{
	struct tegra_csi_device *csi = chan->csi;

	struct CAPTURE_CONTROL_MSG msg;

	dev_dbg(csi->dev, "%s: stream_id=%u, virtual_channel_id=%d\n",
		__func__, stream_id, virtual_channel_id);

	/* Disable TPG on a stream */
	memset(&msg, 0, sizeof(msg));
	msg.header.msg_id = CAPTURE_CSI_STREAM_TPG_STOP_REQ;
	msg.header.channel_id = TEMP_CHANNEL_ID;

	msg.csi_stream_tpg_stop_req.stream_id = stream_id;
	msg.csi_stream_tpg_stop_req.virtual_channel_id = virtual_channel_id;

	tegra_capture_ivc_control_submit(&msg, sizeof(msg));
}

static int csi5_start_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	int err = 0, num_lanes;
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[0];
	u32 csi_pt, st_id, vc_id;

	if (chan->pg_mode) {
		csi_pt = NVCSI_PORT_UNSPECIFIED;
		st_id = port->stream_id;
	} else {
		csi_pt = port->csi_port;
		st_id = csi5_port_to_stream(port->csi_port);
	}
	vc_id = port->virtual_channel_id;
	num_lanes = port->lanes;

	dev_dbg(csi->dev, "%s: csi_pt=%u, st_id=%u, vc_id=%u, pg_mode=0x%x\n",
		__func__, csi_pt, st_id, vc_id, chan->pg_mode);

	if (!chan->pg_mode)
		csi5_stream_set_config(chan, st_id, csi_pt, num_lanes);

	csi5_stream_open(chan, st_id, csi_pt);

	if (chan->pg_mode) {
		err = csi5_stream_tpg_start(chan, st_id, vc_id);
		if (err)
			return err;
	}

	return err;
}

static void csi5_stop_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[0];
	u32 csi_pt, st_id, vc_id;

	if (chan->pg_mode) {
		csi_pt = NVCSI_PORT_UNSPECIFIED;
		st_id = port->stream_id;
	} else {
		csi_pt = port->csi_port;
		st_id = csi5_port_to_stream(port->csi_port);
	}
	vc_id = port->virtual_channel_id;

	dev_dbg(csi->dev, "%s: csi_pt=%u, st_id=%u, vc_id=%u, pg_mode=0x%x\n",
		__func__, csi_pt, st_id, vc_id, chan->pg_mode);

	if (chan->pg_mode)
		csi5_stream_tpg_stop(chan, st_id, vc_id);

	csi5_stream_close(chan, st_id, csi_pt);
}

static int csi5_error_recover(struct tegra_csi_channel *chan, int port_idx)
{
	int err = 0;
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[0];

	csi5_stop_streaming(chan, port_idx);

	err = csi5_start_streaming(chan, port_idx);
	if (err) {
		dev_err(csi->dev, "failed to restart csi stream %d\n",
			csi5_port_to_stream(port->csi_port));
	}

	return err;
}

static int csi5_mipi_cal(struct tegra_csi_channel *chan)
{
	unsigned int lanes, num_ports, csi_port, addr;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi = chan->csi;
	u32 phy_mode = read_phy_mode_from_dt(chan);
	bool is_cphy = (phy_mode == CSI_PHY_MODE_CPHY);

	dev_dbg(csi->dev, "%s\n", __func__);

	lanes = 0;
	num_ports = 0;
	csi_port = 0;
	while (num_ports < chan->numports) {
		csi_port = chan->ports[num_ports].csi_port;
		dev_dbg(csi->dev, "csi_port:%d\n", csi_port);

		if (chan->numlanes <= 2) {
			lanes |= CSIA << csi_port;
			addr = (csi_port % 2 == 0 ?
				CSI5_NVCSI_CIL_A_SW_RESET :
				CSI5_NVCSI_CIL_B_SW_RESET);
			csi5_phy_write(chan, csi_port >> 1, addr,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		} else if (chan->numlanes == 3) {
			lanes |= (CSIA | CSIB) << csi_port;
			cila =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			cilb =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x00 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x01 << CSI5_PD_IO1_SHIFT);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_A_BASE + CSI5_PAD_CONFIG_0,
					cila);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_B_BASE + CSI5_PAD_CONFIG_0,
					cilb);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_A_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_B_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		} else {
			lanes |= (CSIA | CSIB) << csi_port;
			cila =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_CLK_SHIFT) |
				(0x00 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			cilb =  (0x01 << CSI5_E_INPUT_LP_IO0_SHIFT) |
				(0x01 << CSI5_E_INPUT_LP_IO1_SHIFT) |
				(0x01 << CSI5_PD_CLK_SHIFT) |
				(0x00 << CSI5_PD_IO0_SHIFT) |
				(0x00 << CSI5_PD_IO1_SHIFT);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_A_BASE + CSI5_PAD_CONFIG_0,
					cila);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_B_BASE + CSI5_PAD_CONFIG_0,
					cilb);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_A_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
			csi5_phy_write(chan, csi_port >> 1,
				CSI5_NVCSI_CIL_B_SW_RESET,
				CSI5_SW_RESET1_EN | CSI5_SW_RESET0_EN);
		}
		num_ports++;
	}
	speculation_barrier(); /* break_spec_p#6_1 */
	if (!lanes) {
		dev_err(csi->dev,
			"Selected no CSI lane, cannot do calibration");
		return -EINVAL;
	}
	lanes |= is_cphy ? CPHY_MASK : 0;
	return tegra_mipi_calibration(lanes);
}

static int csi5_hw_init(struct tegra_csi_device *csi)
{
	dev_dbg(csi->dev, "%s\n", __func__);

	csi->iomem[0] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_0_BASE;
	csi->iomem[1] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_2_BASE;
	csi->iomem[2] = csi->iomem_base + CSI5_TEGRA_CSI_STREAM_4_BASE;

	return 0;
}

struct tegra_csi_fops csi5_fops = {
	.csi_power_on = csi5_power_on,
	.csi_power_off = csi5_power_off,
	.csi_start_streaming = csi5_start_streaming,
	.csi_stop_streaming = csi5_stop_streaming,
	.csi_error_recover = csi5_error_recover,
	.mipical = csi5_mipi_cal,
	.hw_init = csi5_hw_init,
};
