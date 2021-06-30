/*
 * Tegra CSI4 device common APIs
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Frank Chen <frankc@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk/tegra.h>
#include <linux/of_graph.h>
#include <linux/string.h>

#include <media/csi.h>
#include <media/csi4_registers.h>
#include <media/tegra_camera_core.h>
#include <media/mc_common.h>

#include <uapi/linux/nvhost_ioctl.h>
#include "mipical/mipi_cal.h"
#include "nvcsi/nvcsi.h"
#include "nvhost_acm.h"

#include "csi4_fops.h"

#define NUM_CSI_PORTS (NVCSI_PORT_F + 1)

static atomic_t port_refcount[NUM_CSI_PORTS];

static void csi4_stream_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? CSI4_STREAM_OFFSET : 0x0;

	writel(val, csi->iomem[index >> 1] + cilb_offset + addr);
}

static u32 csi4_stream_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;
	u32 cilb_offset = (index & 0x1) ? CSI4_STREAM_OFFSET : 0x0;

	return readl(csi->iomem[index >> 1] + cilb_offset + addr);
}

static void csi4_phy_write(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr, u32 val)
{
	struct tegra_csi_device *csi = chan->csi;

	writel(val, csi->iomem_base +
		CSI4_BASE_ADDRESS + (CSI4_PHY_OFFSET * index) + addr);
}

static u32 csi4_phy_read(struct tegra_csi_channel *chan,
		unsigned int index, unsigned int addr)
{
	struct tegra_csi_device *csi = chan->csi;

	return readl(csi->iomem_base +
		CSI4_BASE_ADDRESS + (CSI4_PHY_OFFSET * index) + addr);
}

static void csi4_stream_init(struct tegra_csi_channel *chan, int csi_port)
{
	struct tegra_csi_device *csi = chan->csi;
	bool cil_a = (csi_port & 0x1) ? false : true;

	dev_dbg(csi->dev, "%s\n", __func__);

	if (cil_a) {
		csi4_stream_write(chan, csi_port, CILA_INTR_STATUS, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILA_ERR_INTR_STATUS, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILA_INTR_MASK, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILA_ERR_INTR_MASK, 0xffffffff);
	}

	if (!cil_a || (chan->numlanes > 2)) {
		csi4_stream_write(chan, csi_port, CILB_INTR_STATUS, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILB_ERR_INTR_STATUS, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILB_INTR_MASK, 0xffffffff);
		csi4_stream_write(chan, csi_port, CILB_ERR_INTR_MASK, 0xffffffff);
	}

	csi4_stream_write(chan, csi_port, INTR_STATUS, 0x3ffff);
	csi4_stream_write(chan, csi_port, ERR_INTR_STATUS, 0x7ffff);
	csi4_stream_write(chan, csi_port, ERROR_STATUS2VI_MASK, 0x0);
	csi4_stream_write(chan, csi_port, INTR_MASK, 0x0);
	csi4_stream_write(chan, csi_port, ERR_INTR_MASK, 0x0);
}

static void csi4_stream_config(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	int val;

	dev_dbg(csi->dev, "%s\n", __func__);

	csi4_stream_write(chan, port_idx, PPFSM_TIMEOUT_CTRL, 0);
	csi4_stream_write(chan, port_idx, PH_CHK_CTRL,
			CFG_PH_CRC_CHK_EN | CFG_PH_ECC_CHK_EN);
	csi4_stream_write(chan, port_idx, VC0_DPCM_CTRL, 0);
	csi4_stream_write(chan, port_idx, VC0_DT_OVERRIDE, 0);

	val = csi4_stream_read(chan, port_idx, VC0_DPCM_CTRL);
	dev_dbg(csi->dev, "%s (%d) read VC0_DPCM_CTRL = %08x\n",
			__func__, port_idx, val);
}

static void csi4_phy_config(
	struct tegra_csi_channel *chan, int csi_port,
	int csi_lanes, bool enable)
{
	struct tegra_csi_device *csi = chan->csi;
	int phy_num = csi_port >> 1;
	bool cil_a = (csi_port & 0x1) ? false : true;
	int cil_config;
	/* Clocks for the CSI interface */
	const unsigned int cil_clk_mhz = TEGRA_CSICIL_CLK_MHZ;
	unsigned int mipi_clk_mhz = 0;
	/* Calculated clock settling times for cil and csi clocks */
	unsigned int cil_settletime = read_settle_time_from_dt(chan);
	unsigned int discontinuous_clk = read_discontinuous_clk_from_dt(chan);
	unsigned int csi_settletime;
	u32 phy_mode = read_phy_mode_from_dt(chan);

	dev_dbg(csi->dev, "%s\n", __func__);


	if (phy_mode == CSI_PHY_MODE_CPHY) /* set to CPHY */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_PHY_CTRL, CPHY);
	else /* set to DPHY */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_PHY_CTRL, DPHY);

	/* read current NVCSI_CIL_CONFIG setting */
	cil_config = csi4_phy_read(chan, phy_num, NVCSI_CIL_CONFIG);
	dev_dbg(csi->dev, "NVCSI_CIL_CONFIG = %08x\n", cil_config);

	if (cil_a) {
		/* soft reset for data lane */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);
		/* reset CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			cil_config & ~DATA_LANE_A);
		/* disable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_PAD_CONFIG,
			PD_CLK | PD_IO0 | PD_IO1 | SPARE_IO0 | SPARE_IO1);

		/* setting up CIL B for 3 or 4 lane */
		if (csi_lanes > 2) {
			/* soft reset for data lane */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			/* reset CSI lane number */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
				cil_config & ~DATA_LANE_B);
			/* disable clock lane*/
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_PAD_CONFIG,
				PD_CLK | PD_IO0 | PD_IO1 |
				SPARE_IO0 | SPARE_IO1);
		}

		/* power down de-serializer is CIL B is not in use*/
		if ((cil_config & DATA_LANE_B) == 0)
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_PAD_CONFIG, PDVCLAMP);
	} else {
		/* soft reset for data lane */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
			SW_RESET1_EN | SW_RESET0_EN);
		/* reset CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			cil_config & ~DATA_LANE_B);
		/* disable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_PAD_CONFIG,
			PD_CLK | PD_IO0 | PD_IO1 | SPARE_IO0 | SPARE_IO1);

		/* power down de-serializer if CIL A is not in use*/
		if ((cil_config & DATA_LANE_A) == 0)
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_PAD_CONFIG, PDVCLAMP);
	}

	if (!enable)
		return;

	/* power on de-serializer */
	csi4_phy_write(chan, phy_num, NVCSI_CIL_PAD_CONFIG, 0);

	/* calculate MIPI settling times */
	if (chan->pg_mode || !(chan->s_data))
		mipi_clk_mhz = csi->clk_freq / 1000000;
	else
		mipi_clk_mhz = read_pixel_clk_from_dt(chan) / 1000000;

	dev_dbg(csi->dev, "cil core clock: %u, csi clock: %u", cil_clk_mhz,
		mipi_clk_mhz);

	csi_settletime = tegra_csi_clk_settling_time(csi, cil_clk_mhz);
	/* If cil_settletime is 0, calculate a settling time */
	if (!cil_settletime) {
		dev_dbg(csi->dev, "cil_settingtime was autocalculated");
		cil_settletime = tegra_csi_ths_settling_time(csi,
			cil_clk_mhz,
			mipi_clk_mhz);
	}

	dev_dbg(csi->dev, "csi settle time: %u, cil settle time: %u",
		csi_settletime, cil_settletime);

	if (cil_a) {
		/* set CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			(cil_config & ~DATA_LANE_A) |
			(csi_lanes << DATA_LANE_A_OFFSET));
		/* enable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_PAD_CONFIG,
			E_INPUT_LP_CLK | E_INPUT_LP_IO0 | E_INPUT_LP_IO1);
		/* setup settle time */
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_A_CONTROL,
			DEFAULT_DESKEW_COMPARE | DEFAULT_DESKEW_SETTLE |
			csi_settletime << CLK_SETTLE_SHIFT |
			!discontinuous_clk << T18X_BYPASS_LP_SEQ_SHIFT |
			cil_settletime << THS_SETTLE_SHIFT);
		/* release soft reset */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET, 0x0);

		/* setting up CIL B for 3 lane CPHY */
		if (phy_mode == CSI_PHY_MODE_CPHY) {
			/* set this to reset for pushing more settings */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_A_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);

			/* set CSI lane number */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
				csi_lanes << DATA_LANE_A_OFFSET);

			/* enable clock lane*/
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_PAD_CONFIG,
				E_INPUT_LP_IO0 | E_INPUT_LP_IO1 | PD_CLK |
				SPARE_IO0 | SPARE_IO1);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_PAD_CONFIG,
				E_INPUT_LP_IO0 | PD_IO1 | PD_CLK |
				SPARE_IO0 | SPARE_IO1);

			/* setup settle time */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_CONTROL,
				RESET_DESKEW_COMPARE | RESET_DESKEW_SETTLE |
				DEFAULT_CPHY_CLK_SETTLE | DEFAULT_THS_SETTLE);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_CONTROL,
				RESET_DESKEW_COMPARE | RESET_DESKEW_SETTLE |
				DEFAULT_CPHY_CLK_SETTLE | DEFAULT_THS_SETTLE);

			/* polarity swizzle */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_POLARITY_SWIZZLE_CTRL, 0x0);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_POLARITY_SWIZZLE_CTRL, 0x0);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_LANE_SWIZZLE_CTRL, 0x0);

			/* setup inadj controls */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_DPHY_INADJ_CTRL,
				DEFAULT_SW_SET_DPHY_INADJ_IO0 |
				DEFAULT_SW_SET_DPHY_INADJ_IO1 |
				DEFAULT_DPHY_INADJ_IO0 |
				DEFAULT_DPHY_INADJ_IO1);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_CPHY_INADJ_CTRL,
				DEFAULT_CPHY_EDGE_DELAY_TRIO0 |
				DEFAULT_CPHY_EDGE_DELAY_TRIO1);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_DPHY_INADJ_CTRL,
				DEFAULT_SW_SET_DPHY_INADJ_IO0 |
				DEFAULT_DPHY_INADJ_IO0);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_CPHY_INADJ_CTRL,
				DEFAULT_CPHY_EDGE_DELAY_TRIO0);

			/* release soft reset */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_A_SW_RESET, 0x0);
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_SW_RESET, 0x0);
		}

		/* setting up CIL B for 4 lane */
		if (csi_lanes >= 4) {
			/* set CSI lane number */
			csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
				csi_lanes << DATA_LANE_A_OFFSET);
			/* enable clock lane*/
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_PAD_CONFIG,
				E_INPUT_LP_IO0 | E_INPUT_LP_IO1 | PD_CLK);
			/* setup settle time */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_CONTROL,
				DEFAULT_DESKEW_COMPARE |
				DEFAULT_DESKEW_SETTLE |
				csi_settletime << CLK_SETTLE_SHIFT |
				!discontinuous_clk << T18X_BYPASS_LP_SEQ_SHIFT |
				cil_settletime << THS_SETTLE_SHIFT);
			/* release soft reset */
			csi4_phy_write(chan, phy_num,
				NVCSI_CIL_B_SW_RESET, 0x0);
		}
	} else {
		/* set CSI lane number */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_CONFIG,
			(cil_config & ~DATA_LANE_B) |
			(csi_lanes << DATA_LANE_B_OFFSET));
		/* enable clock lane*/
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_PAD_CONFIG,
			E_INPUT_LP_CLK | E_INPUT_LP_IO0 | E_INPUT_LP_IO1);
		/* setup settle time */
		csi4_phy_write(chan, phy_num,
			NVCSI_CIL_B_CONTROL,
			DEFAULT_DESKEW_COMPARE | DEFAULT_DESKEW_SETTLE |
			csi_settletime << CLK_SETTLE_SHIFT |
			!discontinuous_clk << T18X_BYPASS_LP_SEQ_SHIFT |
			cil_settletime << THS_SETTLE_SHIFT);
		/* release soft reset */
		csi4_phy_write(chan, phy_num, NVCSI_CIL_B_SW_RESET, 0x0);
	}
}

static void csi4_stream_check_status(struct tegra_csi_channel *chan,
	int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	int status = 0;

	dev_dbg(csi->dev, "%s\n", __func__);
	if (!chan->pg_mode) {
		status = csi4_stream_read(chan, port_idx, ERROR_STATUS2VI_VC0);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC0 = 0x%08x\n",
				__func__, port_idx, status);

		status = csi4_stream_read(chan, port_idx, ERROR_STATUS2VI_VC1);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC1 = 0x%08x\n",
				__func__, port_idx, status);

		status = csi4_stream_read(chan, port_idx, ERROR_STATUS2VI_VC2);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
				__func__, port_idx, status);

		status = csi4_stream_read(chan, port_idx, ERROR_STATUS2VI_VC3);
		if (status)
			dev_err(csi->dev,
				"%s (%d) ERROR_STATUS2VI_VC2 = 0x%08x\n",
				__func__, port_idx, status);
	}

	status = csi4_stream_read(chan, port_idx, INTR_STATUS);
	if (status)
		dev_err(csi->dev,
				"%s (%d) INTR_STATUS 0x%08x\n",
				__func__, port_idx, status);

	status = csi4_stream_read(chan, port_idx, ERR_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
				"%s (%d) ERR_INTR_STATUS 0x%08x\n",
				__func__, port_idx, status);
}

static void csi4_cil_check_status(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	int status = 0;

	dev_dbg(csi->dev, "%s %d\n", __func__, __LINE__);

	status = csi4_stream_read(chan, port_idx, CILA_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
			"%s (%d) CILA_INTR_STATUS 0x%08x\n",
			__func__, port_idx, status);

	status = csi4_stream_read(chan, port_idx, CILA_ERR_INTR_STATUS);
	if (status)
		dev_err(csi->dev,
			"%s (%d) CILA_ERR_INTR_STATUS 0x%08x\n",
			__func__, port_idx, status);
}


static int csi4_power_on(struct tegra_csi_device *csi)
{
	int err = 0;

	err = nvhost_module_busy(csi->pdev);
	if (err)
		dev_err(csi->dev, "%s:cannot enable csi\n", __func__);

	return err;
}

static int csi4_power_off(struct tegra_csi_device *csi)
{
	nvhost_module_idle(csi->pdev);

	return 0;
}

static void csi4_tpg_stop_streaming(struct tegra_csi_channel *chan,
				int ports_index)
{
	unsigned int csi_port = chan->ports[ports_index].stream_id;
	struct tegra_csi_device *csi = chan->csi;

	dev_dbg(csi->dev, "%s\n", __func__);
	csi4_stream_check_status(chan, csi_port);
	csi4_cil_check_status(chan, csi_port);
	csi4_stream_write(chan, csi_port, PP_EN_CTRL, 0);
	csi4_stream_write(chan, csi_port, TPG_EN_0, 0);
	csi4_stream_write(chan, csi_port, PG_CTRL, PG_DISABLE);
}

static int csi4_tpg_start_streaming(struct tegra_csi_channel *chan,
	int port_idx)
{
	struct tegra_csi_port *port = &chan->ports[port_idx];
	struct tegra_csi_device *csi = chan->csi;
	unsigned int val, csi_port, csi_lanes;

	if (!port->core_format) {
		dev_err(csi->dev, "Fail to find tegra video fmt");
		return -EINVAL;
	}

	csi_port = port->stream_id;
	csi_lanes = port->lanes;
	dev_dbg(csi->dev, "%s CSI port=%d, # lanes=%d\n",
			__func__, csi_port, csi_lanes);

	csi4_stream_write(chan, csi_port, PH_CHK_CTRL, 0);
	csi4_stream_write(chan, csi_port, INTR_MASK, PH_ECC_MULTI_BIT_ERR |
			PD_CRC_ERR_VC0 | PH_ECC_SINGLE_BIT_ERR_VC0);
	csi4_stream_write(chan, csi_port, ERR_INTR_MASK, PH_ECC_MULTI_BIT_ERR |
			PD_CRC_ERR_VC0 | PH_ECC_SINGLE_BIT_ERR_VC0);
	csi4_stream_write(chan, csi_port, ERROR_STATUS2VI_MASK,
			CFG_ERR_STATUS2VI_MASK_VC0 |
			CFG_ERR_STATUS2VI_MASK_VC1 |
			CFG_ERR_STATUS2VI_MASK_VC2 |
			CFG_ERR_STATUS2VI_MASK_VC3);
	/* calculate PG blank */
	csi4_stream_write(chan, csi_port, PG_BLANK,
			((port->v_blank & PG_VBLANK_MASK) << PG_VBLANK_OFFSET) |
			((port->h_blank & PG_HBLANK_MASK) << PG_HBLANK_OFFSET));
	csi4_stream_write(chan, csi_port, PG_PHASE, 0x0);
	csi4_stream_write(chan, csi_port, PG_RED_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_RED_FREQ_RATE, 0x0);
	csi4_stream_write(chan, csi_port, PG_GREEN_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_GREEN_FREQ_RATE, 0x0);
	csi4_stream_write(chan, csi_port, PG_BLUE_FREQ,
			(0x10 << PG_VERT_INIT_FREQ_OFFSET)|
			(0x10 << PG_HOR_INIT_FREQ_OFFSET));
	csi4_stream_write(chan, csi_port, PG_BLUE_FREQ_RATE, 0x0);
	/* calculate PG IMAGE SIZE and DT */
	mutex_lock(&chan->format_lock);
	val = port->format.height << HEIGHT_OFFSET |
		(port->format.width *
		(port->core_format->vf_code == TEGRA_VF_RAW10 ? 10 : 24) / 8);
	mutex_unlock(&chan->format_lock);
	csi4_stream_write(chan, csi_port, PG_IMAGE_SIZE, val);
	csi4_stream_write(chan, csi_port, PG_IMAGE_DT,
			port->core_format->img_dt);
	csi4_stream_write(chan, csi_port, PP_EN_CTRL, CFG_PP_EN);
	csi4_stream_write(chan, csi_port, TPG_EN_0, cfg_tpg_en);

	csi4_stream_write(chan, csi_port, PG_CTRL,
			((chan->pg_mode - 1) << PG_MODE_OFFSET) | PG_ENABLE);
	return 0;
}
static int csi4_hw_init(struct tegra_csi_device *csi)
{
	u32 i;

	csi->iomem[0] = csi->iomem_base + TEGRA_CSI_STREAM_0_BASE;
	csi->iomem[1] = csi->iomem_base + TEGRA_CSI_STREAM_2_BASE;
	csi->iomem[2] = csi->iomem_base + TEGRA_CSI_STREAM_4_BASE;

	for (i = 0; i < NUM_CSI_PORTS; i++)
		atomic_set(&port_refcount[i], 0);

	return 0;
}
static int csi4_start_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_idx];
	int csi_port, csi_lanes, ret = 0;

	csi_lanes = port->lanes;
	dev_dbg(csi->dev, "%s port_idx=%d, lanes=%d\n",
			__func__, port_idx, csi_lanes);

	if (chan->pg_mode)
		ret = csi4_tpg_start_streaming(chan, port_idx);
	else {
		csi_port = port->csi_port;
		if (atomic_inc_return(&port_refcount[csi_port]) == 1) {
			csi4_stream_init(chan, csi_port);
			csi4_stream_config(chan, csi_port);
			/* enable PHY */
			csi4_phy_config(chan, csi_port, csi_lanes, true);
			csi4_stream_write(chan, csi_port, PP_EN_CTRL,
						CFG_PP_EN);
		}
	}
	return ret;
}

static void csi4_stop_streaming(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[port_idx];
	int csi_port, csi_lanes;

	csi_lanes = port->lanes;
	dev_dbg(csi->dev, "%s port_idx=%d, lanes=%d\n",
		__func__, port_idx, csi_lanes);

	if (chan->pg_mode)
		csi4_tpg_stop_streaming(chan, port_idx);
	else {
		csi_port = port->csi_port;
		if (atomic_dec_return(&port_refcount[csi_port]) == 0) {
			/* disable PHY */
			csi4_phy_config(chan, csi_port, csi_lanes, false);
			csi4_stream_check_status(chan, csi_port);
			csi4_cil_check_status(chan, csi_port);
		}
	}
}

static void csi4_override_format(struct tegra_csi_channel *chan, int port_idx)
{
	struct tegra_csi_port *port = &chan->ports[port_idx];
	unsigned int val;
	int csi_port;

	if (!chan->pg_mode) {
		dev_err(chan->csi->dev, "%s non PG format update failed\n",
				__func__);
		return;
	}

	/* calculate PG IMAGE SIZE and DT */
	mutex_lock(&chan->format_lock);
	val = port->format.height << HEIGHT_OFFSET |
		(port->format.width *
		(port->core_format->vf_code == TEGRA_VF_RAW10 ? 10 : 24) / 8);
	mutex_unlock(&chan->format_lock);

	csi_port = port->stream_id;
	csi4_stream_write(chan, csi_port, PG_IMAGE_SIZE, val);
}

static int csi4_error_recover(struct tegra_csi_channel *chan, int port_idx)
{
	int err = 0;
	struct tegra_csi_device *csi = chan->csi;
	struct tegra_csi_port *port = &chan->ports[0];

	csi4_stop_streaming(chan, port_idx);

	err = csi4_start_streaming(chan, port_idx);
	if (err) {
		dev_err(csi->dev, "failed to restart csi stream %d\n",
			port->stream_id);
	}

	return err;
}

static int csi4_mipi_cal(struct tegra_csi_channel *chan)
{
	unsigned int lanes, num_ports, csi_port, addr;
	unsigned int cila, cilb;
	struct tegra_csi_device *csi = chan->csi;
	u32 phy_mode = read_phy_mode_from_dt(chan);
	bool is_cphy = (phy_mode == CSI_PHY_MODE_CPHY);

	lanes = 0;
	num_ports = 0;
	csi_port = 0;
	while (num_ports < chan->numports) {
		csi_port = chan->ports[num_ports].csi_port;
		dev_dbg(csi->dev, "csi_port: %u\n", csi_port);

		if (chan->numlanes <= 2) {
			lanes |= CSIA << csi_port;
			addr = (csi_port % 2 == 0 ?
				NVCSI_CIL_A_SW_RESET : NVCSI_CIL_B_SW_RESET);
			csi4_phy_write(chan, csi_port >> 1, addr,
				SW_RESET1_EN | SW_RESET0_EN);
		} else if (chan->numlanes == 3) {
			lanes |= (CSIA | CSIB) << csi_port;
			cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x01 << E_INPUT_LP_IO1_SHIFT) |
				(0x00 << E_INPUT_LP_CLK_SHIFT) |
				(0x01 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x00 << PD_IO1_SHIFT);
			cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x00 << E_INPUT_LP_IO1_SHIFT) |
				(0x00 << E_INPUT_LP_CLK_SHIFT) |
				(0x01 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x01 << PD_IO1_SHIFT);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_A_BASE + PAD_CONFIG_0, cila);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_B_BASE + PAD_CONFIG_0, cilb);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_A_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
		} else {
			lanes |= (CSIA | CSIB) << csi_port;
			cila =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x01 << E_INPUT_LP_IO1_SHIFT) |
				(0x01 << E_INPUT_LP_CLK_SHIFT) |
				(0x00 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x00 << PD_IO1_SHIFT);
			cilb =  (0x01 << E_INPUT_LP_IO0_SHIFT) |
				(0x01 << E_INPUT_LP_IO1_SHIFT) |
				(0x01 << PD_CLK_SHIFT) |
				(0x00 << PD_IO0_SHIFT) |
				(0x00 << PD_IO1_SHIFT);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_A_BASE + PAD_CONFIG_0, cila);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_B_BASE + PAD_CONFIG_0, cilb);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_A_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
			csi4_phy_write(chan, csi_port >> 1,
				NVCSI_CIL_B_SW_RESET,
				SW_RESET1_EN | SW_RESET0_EN);
		}
		num_ports++;
	}
	if (!lanes) {
		dev_err(csi->dev, "Selected no CSI lane, cannot do calibration");
		return -EINVAL;
	}
	lanes |= is_cphy ? CPHY_MASK : 0;
	return tegra_mipi_calibration(lanes);
}
struct tegra_csi_fops csi4_fops = {
	.csi_power_on = csi4_power_on,
	.csi_power_off = csi4_power_off,
	.csi_start_streaming = csi4_start_streaming,
	.csi_stop_streaming = csi4_stop_streaming,
	.csi_override_format = csi4_override_format,
	.csi_error_recover = csi4_error_recover,
	.mipical = csi4_mipi_cal,
	.hw_init = csi4_hw_init,
};
