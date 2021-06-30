/*
 * t19x-nvlink-endpt-link.c:
 * This file contains link state transition and link trainig code for the Tegra
 * NVLINK controller.
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <soc/tegra/fuse.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define SUBLINK_TIMEOUT_MS	200 /* msec */
#define R4TX_TIMEOUT_US		1000 /* usec */

const struct single_lane_params entry_100us_sl_params = {
	.fb_ic_inc = 1,
	.lp_ic_inc = 1,
	.fb_ic_dec = 1,
	.lp_ic_dec = 65535,
	.enter_thresh = 161100,
	.exit_thresh = 0,
	.ic_limit = 161100,
};

u32 t19x_nvlink_get_link_state(struct nvlink_device *ndev)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;

	return (nvlw_nvl_readl(tdev, NVL_LINK_STATE) &
				NVL_LINK_STATE_STATE_MASK);
}

u32 t19x_nvlink_get_link_mode(struct nvlink_device *ndev)
{
	u32 link_mode;
	u32 link_state = t19x_nvlink_get_link_state(ndev);

	switch (link_state) {
	case NVL_LINK_STATE_STATE_INIT:
		link_mode = NVLINK_LINK_OFF;
		break;
	case NVL_LINK_STATE_STATE_HWCFG:
		link_mode = NVLINK_LINK_DETECT;
		break;
	case NVL_LINK_STATE_STATE_SWCFG:
		link_mode = NVLINK_LINK_SAFE;
		break;
	case NVL_LINK_STATE_STATE_ACTIVE:
		link_mode = NVLINK_LINK_HS;
		break;
	case NVL_LINK_STATE_STATE_FAULT:
		link_mode = NVLINK_LINK_FAULT;
		break;
	case NVL_LINK_STATE_STATE_RCVY_AC:
		link_mode = NVLINK_LINK_RCVY_AC;
		break;
	case NVL_LINK_STATE_STATE_RCVY_SW:
		link_mode = NVLINK_LINK_RCVY_SW;
		break;
	case NVL_LINK_STATE_STATE_RCVY_RX:
		link_mode = NVLINK_LINK_RCVY_RX;
		break;
	default:
		nvlink_err("Invalid link state (link state = %u)", link_state);
		link_mode = NVLINK_LINK_OFF;
		break;
	}

	return link_mode;
}

u32 t19x_nvlink_get_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink)
{
	u32 reg_val;
	u8 state;
	u32 sublink_mode;
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;

	if (!is_rx_sublink) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL0_SLSM_STATUS_TX);
		state = (reg_val & NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_MASK) >>
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SHIFT;

		switch (state) {
		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_EIGHTH:
			sublink_mode = NVLINK_TX_SINGLE_LANE;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_HS:
			sublink_mode = NVLINK_TX_HS;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SAFE:
			sublink_mode = NVLINK_TX_SAFE;
			break;

		case NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_OFF:
			sublink_mode = NVLINK_TX_OFF;
			break;

		default:
			nvlink_err("Invalid TX sublink state"
				" (sublink state = %u)",
				state);
			sublink_mode = NVLINK_TX_OFF;
			break;
		}
	} else {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_SLSM_STATUS_RX);
		state = (reg_val & NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_MASK) >>
				NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SHIFT;

		switch (state) {
		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_EIGHTH:
			sublink_mode = NVLINK_RX_SINGLE_LANE;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_HS:
			sublink_mode = NVLINK_RX_HS;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SAFE:
			sublink_mode = NVLINK_RX_SAFE;
			break;

		case NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_OFF:
			sublink_mode = NVLINK_RX_OFF;
			break;

		default:
			nvlink_err("Invalid RX sublink state"
				" (sublink state = %u)",
				state);
			sublink_mode = NVLINK_RX_OFF;
			break;
		}
	}

	return sublink_mode;
}

void t19x_nvlink_get_tx_sublink_state(struct nvlink_device *ndev,
				u32 *tx_sublink_state)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;
	u32 reg_val;

	reg_val = nvlw_nvl_readl(tdev, NVL_SL0_SLSM_STATUS_TX);
	*tx_sublink_state = (reg_val &
			NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_MASK) >>
			NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SHIFT;
}

void t19x_nvlink_get_rx_sublink_state(struct nvlink_device *ndev,
				u32 *rx_sublink_state)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;
	u32 reg_val;

	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_SLSM_STATUS_RX);
	*rx_sublink_state = (reg_val &
			NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_MASK) >>
			NVL_SL1_SLSM_STATUS_RX_PRIMARY_STATE_SHIFT;
}

/* From the point of view of the Tegra endpoint, is the link in SAFE mode? */
static inline bool is_link_in_safe(struct tnvlink_link *tlink)
{
	struct nvlink_device *ndev = tlink->tdev->ndev;
	enum link_mode link_mode = t19x_nvlink_get_link_mode(ndev);
	enum tx_mode tx_mode = t19x_nvlink_get_sublink_mode(ndev, false);
	enum rx_mode rx_mode = t19x_nvlink_get_sublink_mode(ndev, true);

	return ((link_mode == NVLINK_LINK_SAFE) &&
		(tx_mode == NVLINK_TX_SAFE) &&
		(rx_mode == NVLINK_RX_SAFE));
}

/* From the point of view of the Tegra endpoint, is the link in HISPEED mode? */
static inline bool is_link_in_hs(struct tnvlink_link *tlink)
{
	struct nvlink_device *ndev = tlink->tdev->ndev;
	enum link_mode link_mode = t19x_nvlink_get_link_mode(ndev);
	enum tx_mode tx_mode = t19x_nvlink_get_sublink_mode(ndev, false);
	enum rx_mode rx_mode = t19x_nvlink_get_sublink_mode(ndev, true);

	return ((link_mode == NVLINK_LINK_HS) &&
		(tx_mode == NVLINK_TX_HS || tx_mode == NVLINK_TX_SINGLE_LANE) &&
		(rx_mode == NVLINK_RX_HS || rx_mode == NVLINK_RX_SINGLE_LANE));
}

bool is_link_connected(struct tnvlink_link *tlink)
{
	return (is_link_in_safe(tlink) || is_link_in_hs(tlink));
}

/* Configure and Start the PRBS generator */
static int t19x_nvlink_prbs_gen_en(struct tnvlink_dev *tdev)
{
	u32 reg_val;

	/* Not specified in doc but is required as per HW team */
	nvlw_nvl_writel(tdev, NVL_SL1_RXSLSM_TIMEOUT_2, 0);

	/*
	 * Minion will set these *_PBRS_* and *_SCRAM_* registers
	 * as a part of DLPL init.
	 *
	 * Note: A remote endpoint is expected to set exact same *_PBRS_*
	 * and *_SCRAM_* values. Otherwise, link training fails.
	 */

	reg_val = nvlw_nvl_readl(tdev, NVL_TXIOBIST_CONFIG);
	reg_val |= BIT(NVL_TXIOBIST_CONFIG_DPG_PRBSSEEDLD);
	nvlw_nvl_writel(tdev, NVL_TXIOBIST_CONFIG, reg_val);

	reg_val = nvlw_nvl_readl(tdev, NVL_TXIOBIST_CONFIG);
	reg_val &= ~BIT(NVL_TXIOBIST_CONFIG_DPG_PRBSSEEDLD);
	nvlw_nvl_writel(tdev, NVL_TXIOBIST_CONFIG, reg_val);

	return 0;
}

/* Put RX in calibration */
static int t19x_nvlink_rxcal_enable(struct tnvlink_dev *tdev)
{
	/* TODO: Move the RX calibration code from init_nvhs_phy() to here */
	return 0;
}

int t19x_nvlink_set_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;
	u32 rx_sublink_state, tx_sublink_state;
	u32 reg_val, timeout_us;
	int status = 0;

	timeout_us = SUBLINK_TIMEOUT_MS * 1000;

	t19x_nvlink_get_tx_sublink_state(ndev, &tx_sublink_state);
	t19x_nvlink_get_rx_sublink_state(ndev, &rx_sublink_state);

	/* Check if SLSM is ready to accept a sublink change request */
	do {
		reg_val = nvlw_nvl_readl(tdev, NVL_SUBLINK_CHANGE);
		if ((reg_val & NVL_SUBLINK_CHANGE_STATUS_MASK) ==
					NVL_SUBLINK_CHANGE_STATUS_DONE)
			break;

		usleep_range(DEFAULT_LOOP_SLEEP_US, DEFAULT_LOOP_SLEEP_US * 2);
		timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
	} while (timeout_us > 0);

	if (timeout_us <= 0) {
		nvlink_err("SLSM not ready to accept a state change request");
		return -EINVAL;
	}

	if (!is_rx_sublink) {
		switch (mode) {
		case NVLINK_TX_COMMON:
			/* TODO */
			nvlink_err("Putting the TX sublink in NVLINK_TX_COMMON"
				" mode is currently not supported by the"
				" driver");
			status = -EOPNOTSUPP;
			break;

		case NVLINK_TX_COMMON_DISABLE:
			nvlink_err("Putting the TX sublink in"
				" NVLINK_TX_COMMON_DISABLE mode is currently"
				" not supported by the driver");
			status = -EOPNOTSUPP;
			break;

		case NVLINK_TX_DATA_READY:
			/* TODO */
			nvlink_err("Putting the TX sublink in"
				" NVLINK_TX_DATA_READY mode is currently not"
				" supported by the driver");
			status = -EOPNOTSUPP;
			break;

		case NVLINK_TX_PRBS_EN:
			status = t19x_nvlink_prbs_gen_en(tdev);
			if (status) {
				nvlink_err("Unable to start PRBS generator"
					" for link");
				return status;
			}

			break;

		case NVLINK_TX_HS:
			if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_HS) {
				nvlink_dbg("TX already in High Speed mode");
				break;
			} else if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_OFF) {
				nvlink_err("TX cannot be taken from OFF to"
					" High Speed directly");
				return -EPERM;
			}

			nvlink_dbg("Changing TX sublink state to High Speed");
			reg_val = nvlw_nvl_readl(tdev, NVL_SUBLINK_CHANGE);
			reg_val &= ~NVL_SUBLINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_NEWSTATE_F(
					NVL_SUBLINK_CHANGE_NEWSTATE_HS);
			reg_val &= ~NVL_SUBLINK_CHANGE_SUBLINK_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_SUBLINK_F(
					NVL_SUBLINK_CHANGE_SUBLINK_TX);
			reg_val &= ~NVL_SUBLINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_ACTION_F(
					NVL_SUBLINK_CHANGE_ACTION_SLSM_CHANGE);
			nvlw_nvl_writel(tdev, NVL_SUBLINK_CHANGE, reg_val);

			timeout_us = SUBLINK_TIMEOUT_MS * 1000;
			do {
				reg_val = nvlw_nvl_readl(tdev,
							NVL_SUBLINK_CHANGE);
				if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_DONE)
					break;
				else if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_FAULT) {
					nvlink_err("Fault while changing TX"
						" sublink to High Speed");
					return -EPROTO;
				}
				usleep_range(DEFAULT_LOOP_SLEEP_US,
					DEFAULT_LOOP_SLEEP_US * 2);
				timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
			} while (timeout_us > 0);

			if (timeout_us <= 0) {
				nvlink_err("Timeout while waiting for TX"
					" sublink to go to High Speed");
				return -ETIMEDOUT;
			}

			break;

		case NVLINK_TX_SAFE:
			if (tx_sublink_state ==
				NVL_SL0_SLSM_STATUS_TX_PRIMARY_STATE_SAFE) {
				nvlink_dbg("TX already in Safe mode");
				break;
			}

			nvlink_dbg("Changing TX sublink state to Safe mode");
			reg_val = nvlw_nvl_readl(tdev, NVL_SUBLINK_CHANGE);
			reg_val &= ~NVL_SUBLINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_NEWSTATE_F(
					NVL_SUBLINK_CHANGE_NEWSTATE_SAFE);
			reg_val &= ~NVL_SUBLINK_CHANGE_SUBLINK_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_SUBLINK_F(
					NVL_SUBLINK_CHANGE_SUBLINK_TX);
			reg_val &= ~NVL_SUBLINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_SUBLINK_CHANGE_ACTION_F(
					NVL_SUBLINK_CHANGE_ACTION_SLSM_CHANGE);
			nvlw_nvl_writel(tdev, NVL_SUBLINK_CHANGE, reg_val);

			timeout_us = SUBLINK_TIMEOUT_MS * 1000;
			do {
				reg_val = nvlw_nvl_readl(tdev,
							NVL_SUBLINK_CHANGE);
				if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_DONE)
					break;
				else if ((reg_val &
				NVL_SUBLINK_CHANGE_STATUS_MASK) ==
				NVL_SUBLINK_CHANGE_STATUS_FAULT) {
					nvlink_err("Fault while changing TX"
						" sublink to SAFE MODE");
					return -EPROTO;
				}
				usleep_range(DEFAULT_LOOP_SLEEP_US,
					DEFAULT_LOOP_SLEEP_US * 2);
				timeout_us = timeout_us - DEFAULT_LOOP_SLEEP_US;
			} while (timeout_us > 0);

			if (timeout_us <= 0) {
				nvlink_err("Timeout while waiting for TX"
					" sublink to go to SAFE MODE");
				return -ETIMEDOUT;
			}

			break;

		case NVLINK_TX_ENABLE_PM:
			nvlink_dbg("Enabling Single-Lane (1/8th) mode for the"
				" TX sublink");
			reg_val = nvlw_nvltlc_readl(tdev,
						NVLTLC_TX_PWRM_IC_SW_CTRL);
			reg_val |=
				BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_SOFTWAREDESIRED);
			reg_val &=
				~BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_HARDWAREDISABLE);
			reg_val |=
				BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_COUNTSTART);
			nvlw_nvltlc_writel(tdev,
					NVLTLC_TX_PWRM_IC_SW_CTRL,
					reg_val);

			break;

		case NVLINK_TX_DISABLE_PM:
			nvlink_dbg("Disabling Single-Lane (1/8th) mode for the"
				" TX sublink");
			reg_val = nvlw_nvltlc_readl(tdev,
						NVLTLC_TX_PWRM_IC_SW_CTRL);
			reg_val &=
				~BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_SOFTWAREDESIRED);
			reg_val |=
				BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_HARDWAREDISABLE);
			reg_val &=
				~BIT(NVLTLC_TX_PWRM_IC_SW_CTRL_COUNTSTART);
			nvlw_nvltlc_writel(tdev,
					NVLTLC_TX_PWRM_IC_SW_CTRL,
					reg_val);

			break;

		default:
			nvlink_err("Invalid TX sublink mode"
				" (sublink mode = %u)",
				mode);
			status = -EINVAL;
			break;
		}
	} else {
		switch (mode) {
		case NVLINK_RX_RXCAL:
			status = t19x_nvlink_rxcal_enable(tdev);
			if (status) {
				nvlink_err("Unable to put RX"
					" in RXCAL for link");
				return status;
			}
			break;

		case NVLINK_RX_ENABLE_PM:
			nvlink_dbg("Enabling Single-Lane (1/8th) mode for the"
				" RX sublink");
			reg_val = nvlw_nvltlc_readl(tdev,
						NVLTLC_RX_PWRM_IC_SW_CTRL);
			reg_val |=
				BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_SOFTWAREDESIRED);
			reg_val &=
				~BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_HARDWAREDISABLE);
			reg_val |=
				BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_COUNTSTART);
			nvlw_nvltlc_writel(tdev,
					NVLTLC_RX_PWRM_IC_SW_CTRL,
					reg_val);

			break;

		case NVLINK_RX_DISABLE_PM:
			nvlink_dbg("Disabling Single-Lane (1/8th) mode for the"
				" RX sublink");
			reg_val = nvlw_nvltlc_readl(tdev,
						NVLTLC_RX_PWRM_IC_SW_CTRL);
			reg_val &=
				~BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_SOFTWAREDESIRED);
			reg_val |=
				BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_HARDWAREDISABLE);
			reg_val &=
				~BIT(NVLTLC_RX_PWRM_IC_SW_CTRL_COUNTSTART);
			nvlw_nvltlc_writel(tdev,
					NVLTLC_RX_PWRM_IC_SW_CTRL,
					reg_val);

			break;

		default:
			nvlink_err("Invalid RX sublink mode"
				" (sublink mode = %u)",
				mode);
			status = -EINVAL;
			break;
		}
	}

	return status;
}

int t19x_nvlink_set_link_mode(struct nvlink_device *ndev, u32 mode)
{
	struct tnvlink_dev *tdev = (struct tnvlink_dev *)ndev->priv;
	u32 link_state;
	u32 reg_val;
	int status = 0;

	link_state = t19x_nvlink_get_link_state(ndev);

	switch (mode) {
	case NVLINK_LINK_OFF:
		if (link_state == NVL_LINK_STATE_STATE_INIT) {
			nvlink_dbg("Link is already in OFF mode");
			break;
		} else if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_err("Link cannot be taken from ACTIVE state"
				" to OFF mode");
			return -EPERM;
		}

		/* Free the interrupt line on the device */
		devm_free_irq(tdev->dev, tdev->irq, tdev);

		status = t19x_nvlink_dev_car_disable(ndev);
		if (status < 0) {
			nvlink_err("set link mode to OFF failed");
		}

		/* WAR for Bug 2301575 */
		status = tegra_fuse_clock_disable();
		if (status < 0) {
			nvlink_err("failed to disable fuse clocks");
		} else {
			nvlink_dbg("fuse clocks are turned OFF");
		}
		break;

	case NVLINK_LINK_SAFE:
		if (link_state == NVL_LINK_STATE_STATE_SWCFG) {
			nvlink_dbg("Link is already in Safe mode");
			break;
		} else if (link_state == NVL_LINK_STATE_STATE_HWCFG) {
			nvlink_dbg("Link already transitioning to"
				" Safe mode");
			break;
		}

		nvlink_dbg("Changing Link state to Safe for link");

		if (link_state == NVL_LINK_STATE_STATE_INIT) {
			reg_val = nvlw_nvl_readl(tdev, NVL_LINK_CHANGE);
			reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
					NVL_LINK_CHANGE_NEWSTATE_HWCFG);
			reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
			reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
					NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
			reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_LINK_CHANGE_ACTION_F(
					NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
			nvlw_nvl_writel(tdev, NVL_LINK_CHANGE, reg_val);
		} else if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			/* TODO :
			 * Disable PM first since we are moving out of
			 * ACTIVE state.
			 */
			reg_val = nvlw_nvl_readl(tdev, NVL_LINK_CHANGE);
			reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
			reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
					NVL_LINK_CHANGE_NEWSTATE_SWCFG);
			reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
			reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
					NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
			reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
			reg_val |= NVL_LINK_CHANGE_ACTION_F(
					NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
			nvlw_nvl_writel(tdev, NVL_LINK_CHANGE, reg_val);
		}
		break;

	case NVLINK_LINK_HS:
		if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_dbg("Link is already in Active mode");
			break;
		} else if (link_state == NVL_LINK_STATE_STATE_INIT) {
			nvlink_err("Link cannot be taken from INIT state"
				" to Active mode");
			return -EPERM;
		}

		nvlink_dbg("changing Link state to Active...");
		reg_val = nvlw_nvl_readl(tdev, NVL_LINK_CHANGE);
		reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
		reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
				NVL_LINK_CHANGE_NEWSTATE_ACTIVE);
		reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
		reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
				NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
		reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
		reg_val |= NVL_LINK_CHANGE_ACTION_F(
				NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
		nvlw_nvl_writel(tdev, NVL_LINK_CHANGE, reg_val);

		break;

	case NVLINK_LINK_RCVY_AC:
		if (link_state != NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_err("Link is not in ACTIVE state. The link can"
				" only go to RCVY_AC state from ACTIVE.");
			status = -EPERM;
			break;
		}

		nvlink_dbg("Changing link state to RCVY_AC");
		reg_val = nvlw_nvl_readl(tdev, NVL_LINK_CHANGE);
		reg_val &= ~NVL_LINK_CHANGE_NEWSTATE_F(~0);
		reg_val |= NVL_LINK_CHANGE_NEWSTATE_F(
					NVL_LINK_CHANGE_NEWSTATE_RCVY_AC);
		reg_val &= ~NVL_LINK_CHANGE_OLDSTATE_MASK_F(~0);
		reg_val |= NVL_LINK_CHANGE_OLDSTATE_MASK_F(
					NVL_LINK_CHANGE_OLDSTATE_MASK_DONTCARE);
		reg_val &= ~NVL_LINK_CHANGE_ACTION_F(~0);
		reg_val |= NVL_LINK_CHANGE_ACTION_F(
				NVL_LINK_CHANGE_ACTION_LTSSM_CHANGE);
		nvlw_nvl_writel(tdev, NVL_LINK_CHANGE, reg_val);

		break;

	case NVLINK_LINK_ENABLE_PM:
		if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_dbg("Link is in Active state."
				" Enabling Single-Lane (1/8th) mode.");

			/* Enable Single-Lane mode for the TX sublink */
			status = t19x_nvlink_set_sublink_mode(ndev,
							false,
							NVLINK_TX_ENABLE_PM);
			if (status) {
				nvlink_err("Failed to enable SL (1/8th) mode"
					" for the TX sublink");
				break;
			}

			/* Enable Single-Lane mode for the RX sublink */
			status = t19x_nvlink_set_sublink_mode(ndev,
							true,
							NVLINK_RX_ENABLE_PM);
			if (status) {
				nvlink_err("Failed to enable SL (1/8th) mode"
					" for the RX sublink");
				break;
			}

			/*
			 * This is the final piece for enabling Single-Lane (SL)
			 * mode. We send the ENABLEPM command to the MINION to
			 * instruct MINION to enter/exit SL mode as per the
			 * programmed SL policy.
			 */
			status = minion_send_cmd(tdev,
					MINION_NVLINK_DL_CMD_COMMAND_ENABLEPM,
					0);
			if (status < 0) {
				nvlink_err("Error encountered while sending the"
					" ENABLEPM command to the MINION");
				nvlink_err("Failed to enable SL (1/8th) mode");
				minion_dump_pc_trace(tdev);
				minion_dump_registers(tdev);
				break;
			}
		} else {
			nvlink_err("Link is not in Active state."
				" Single-Lane (1/8th) mode can only be enabled"
				" from the Active state.");
			status = -EPERM;
			break;
		}

		break;

	case NVLINK_LINK_DISABLE_PM:
		if (link_state == NVL_LINK_STATE_STATE_ACTIVE) {
			nvlink_dbg("Link is in Active state."
				" Disabling Single-Lane (1/8th) mode.");

			/* Disable Single-Lane mode for the TX sublink */
			status = t19x_nvlink_set_sublink_mode(ndev,
							false,
							NVLINK_TX_DISABLE_PM);
			if (status) {
				nvlink_err("Failed to disable SL (1/8th) mode"
					" for the TX sublink");
				break;
			}

			/* Disable Single-Lane mode for the RX sublink */
			status = t19x_nvlink_set_sublink_mode(ndev,
							true,
							NVLINK_RX_DISABLE_PM);
			if (status) {
				nvlink_err("Failed to disable SL (1/8th) mode"
					" for the RX sublink");
				break;
			}

			/* Disable Single-Lane mode in MINION */
			status = minion_send_cmd(tdev,
					MINION_NVLINK_DL_CMD_COMMAND_DISABLEPM,
					0);
			if (status < 0) {
				nvlink_err("Error encountered while sending the"
					" DISABLEPM command to the MINION");
				nvlink_err("Failed to disable SL (1/8th) mode");
				minion_dump_pc_trace(tdev);
				minion_dump_registers(tdev);
				break;
			}
		} else {
			nvlink_err("Link is not in Active state."
				" Single-Lane (1/8th) mode can only be disabled"
				" from the Active state.");
			status = -EPERM;
			break;
		}

		break;

	case NVLINK_LINK_DISABLE_ERR_DETECT:
		/* Disable link interrupts */
		nvlink_disable_link_interrupts(tdev);

		break;

	case NVLINK_LINK_LANE_DISABLE:
		status = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_LANEDISABLE, 0);
		if (status < 0) {
			nvlink_err("Error setting link mode to LANE DISABLE");
			break;
		}

		break;

	case NVLINK_LINK_LANE_SHUTDOWN:
		status = minion_send_cmd(tdev,
				MINION_NVLINK_DL_CMD_COMMAND_LANESHUTDOWN, 0);
		if (status < 0) {
			nvlink_err("Error setting link mode to LANE SHUTDOWN");
			break;
		}

		break;

	/* TODO: other "case"s need to be implemented here */

	default:
		nvlink_err("Invalid link mode specified (link mode = %u)",
			mode);
		status = -EINVAL;
		break;
	}

	return status;
}

/* Initialize TLC settings which dictate Single-Lane (1/8th) mode policy */
void init_single_lane_params(struct tnvlink_dev *tdev)
{
	u32 reg_val = 0;
	struct single_lane_params *sl_params = &tdev->tlink.sl_params;

	nvlink_dbg("Initializing Single-Lane parameters");

	/* Idle counter increments */
	reg_val = NVLTLC_TX_PWRM_IC_INC_FBINC_F(sl_params->fb_ic_inc) |
		NVLTLC_TX_PWRM_IC_INC_LPINC_F(sl_params->lp_ic_inc);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_PWRM_IC_INC, reg_val);

	/* Idle counter decrements */
	reg_val = NVLTLC_TX_PWRM_IC_DEC_FBDEC_F(sl_params->fb_ic_dec) |
		NVLTLC_TX_PWRM_IC_DEC_LPDEC_F(sl_params->lp_ic_dec);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_PWRM_IC_DEC, reg_val);

	/* Entry threshold */
	nvlw_nvltlc_writel(tdev,
			NVLTLC_TX_PWRM_IC_LP_ENTER_THRESHOLD,
			sl_params->enter_thresh);

	/* Exit threshold */
	nvlw_nvltlc_writel(tdev,
			NVLTLC_TX_PWRM_IC_LP_EXIT_THRESHOLD,
			sl_params->exit_thresh);

	/* Idle counter saturation limit */
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_PWRM_IC_LIMIT, sl_params->ic_limit);
}


/* Enable ANO packets over link */
void nvlink_enable_AN0_packets(struct tnvlink_dev *tdev)
{
	u32 reg_val = 0;

	reg_val = nvlw_nvl_readl(tdev, NVL_LINK_CONFIG);
	reg_val |= BIT(NVL_LINK_CONFIG_LINK_EN);
	nvlw_nvl_writel(tdev, NVL_LINK_CONFIG, reg_val);
}

static int nvlink_retrain_link_from_off(struct tnvlink_dev *tdev)
{
	/* We don't need this for now */
	nvlink_err("Retraining the link from OFF mode is currently not"
		" supported by the driver");
	return -EOPNOTSUPP;
}

static int nvlink_retrain_link_from_safe(struct tnvlink_dev *tdev)
{
	int ret;
	struct nvlink_device *ndev = tdev->ndev;

	ret = nvlink_transition_intranode_conn_hs_to_safe(ndev);
	if (ret < 0) {
		nvlink_err("Transiting intranode conn to safe failed");
		return ret;
	}

	ret = nvlink_train_intranode_conn_safe_to_hs(ndev);
	if (ret < 0) {
		nvlink_err("Train intranode conn to HS failed");
		return ret;
	}

	/* The link has successfully retrained */
	tdev->tlink.error_recoveries++;

	return 0;
}

/* Retrain a link from either safe mode or off */
int nvlink_retrain_link(struct tnvlink_dev *tdev, bool from_off)
{
	struct nvlink_device *ndev = tdev->ndev;
	int ret;

	if (!ndev->is_master) {
		/* TODO :
		 * Not needed for loopback but needed for other topologies.
		 *
		 * If this is a slave endpoint requesting the retrain,
		 * kick off a request to the master instead.
		 * There is no need to (and indeed, we must not) hold
		 * the master endpoint lock here.
		 */
		nvlink_err("Link retraining from the slave endpoint is"
			" currently not supported");
		return -EPERM;
	}

	if (from_off)
		ret = nvlink_retrain_link_from_off(tdev);
	else
		ret = nvlink_retrain_link_from_safe(tdev);

	return ret;
}

int t19x_nvlink_write_discovery_token(struct tnvlink_dev *tdev, u64 token)
{
	int ret = 0;
	u32 reg_val = 0;
	u32 token_low_32 = (u32)(token & (u64)(0xffffffff));
	u32 token_high_32 = (u32)((token >> (u64)(32)) & (u64)(0xffffffff));

	/* Check if R4TX interface is ready to accept a new request */
	reg_val = nvlw_nvl_readl(tdev, NVL_SL0_R4TX_COMMAND);
	if (!(reg_val & BIT(NVL_SL0_R4TX_COMMAND_READY))) {
		nvlink_err("Unable to write discovery token because R4TX"
				" interface is not ready for a new request");
		ret = -EBUSY;
		goto fail;
	}

	/* Write the token in little endian format */
	nvlw_nvl_writel(tdev, NVL_SL0_R4TX_WDATA0, token_low_32);
	nvlw_nvl_writel(tdev, NVL_SL0_R4TX_WDATA1, token_high_32);

	/* Issue the command to write the token */
	reg_val &= ~NVL_SL0_R4TX_COMMAND_REQUEST_F(~0);
	reg_val |= NVL_SL0_R4TX_COMMAND_REQUEST_F(
					NVL_SL0_R4TX_COMMAND_REQUEST_WRITE);
	reg_val |= BIT(NVL_SL0_R4TX_COMMAND_COMPLETE);
	reg_val &= ~NVL_SL0_R4TX_COMMAND_WADDR_F(~0);
	reg_val |= NVL_SL0_R4TX_COMMAND_WADDR_F(NVL_SL0_R4TX_COMMAND_WADDR_SC);
	nvlw_nvl_writel(tdev, NVL_SL0_R4TX_COMMAND, reg_val);

	/* Wait for the token write command to complete */
	ret = wait_for_reg_cond_nvlink(tdev,
					NVL_SL0_R4TX_COMMAND,
					NVL_SL0_R4TX_COMMAND_COMPLETE,
					true,
					"NVL_SL0_R4TX_COMMAND_COMPLETE",
					nvlw_nvl_readl,
					&reg_val,
					R4TX_TIMEOUT_US);
	if (ret < 0)
		goto fail;

	nvlink_dbg("Successfully wrote discovery token. Token = 0x%llx.",
		token);
	goto exit;

fail:
	nvlink_err("Failed to write discovery token!");
exit:
	return ret;
}

int t19x_nvlink_read_discovery_token(struct tnvlink_dev *tdev, u64 *token)
{
	int ret = 0;
	u32 reg_val = 0;
	u32 token_low_32 = 0;
	u32 token_high_32 = 0;

	/* Check if R4TX interface is ready to accept a new request */
	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_R4LOCAL_COMMAND);
	if (!(reg_val & BIT(NVL_SL1_R4LOCAL_COMMAND_READY))) {
		nvlink_err("Unable to read discovery token because R4TX"
				" interface is not ready for a new request");
		ret = -EBUSY;
		goto fail;
	}

	/* Issue the command to read the token */
	reg_val &= ~NVL_SL1_R4LOCAL_COMMAND_REQUEST_F(~0);
	reg_val |= NVL_SL1_R4LOCAL_COMMAND_REQUEST_F(
					NVL_SL1_R4LOCAL_COMMAND_REQUEST_READ);
	reg_val |= BIT(NVL_SL1_R4LOCAL_COMMAND_COMPLETE);
	reg_val &= ~NVL_SL1_R4LOCAL_COMMAND_RADDR_F(~0);
	reg_val |= NVL_SL1_R4LOCAL_COMMAND_RADDR_F(
					NVL_SL1_R4LOCAL_COMMAND_RADDR_SC);
	nvlw_nvl_writel(tdev, NVL_SL1_R4LOCAL_COMMAND, reg_val);

	/* Wait for the token read command to complete */
	ret = wait_for_reg_cond_nvlink(tdev,
					NVL_SL1_R4LOCAL_COMMAND,
					NVL_SL1_R4LOCAL_COMMAND_COMPLETE,
					true,
					"NVL_SL1_R4LOCAL_COMMAND_COMPLETE",
					nvlw_nvl_readl,
					&reg_val,
					R4TX_TIMEOUT_US);
	if (ret < 0)
		goto fail;

	/* Read the token in little endian format */
	token_low_32 = nvlw_nvl_readl(tdev, NVL_SL1_R4LOCAL_RDATA0);
	token_high_32 = nvlw_nvl_readl(tdev, NVL_SL1_R4LOCAL_RDATA1);
	*token = ((u64)(token_high_32) << (u64)(32)) |
		((u64)(token_low_32) & (u64)(0xffffffff));

	nvlink_dbg("Successfully read discovery token. Token = 0x%llx.",
		*token);
	goto exit;

fail:
	nvlink_err("Failed to read discovery token!");
exit:
	return ret;
}


/*
 *-----------------------------------------------------------------------------*
 * TLC THROUGHPUT COUNTERS
 *-----------------------------------------------------------------------------*
 */

/* Reset the TLC throughput counters for both RX and TX */
int t19x_nvlink_reset_tp_counters(struct tnvlink_dev *tdev)
{
	int ret = 0;
	u32 data;

	if(tdev == NULL) {
		nvlink_err("Invalid tnvlink_dev pointer");
		return -EINVAL;
	}

	/* Reset TX tp counters */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL);
	data |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX0);
	data |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX1);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL, data);

	/* Reset RX tp counters */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL);
	data |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX0);
	data |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX1);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL, data);

	return ret;
}

/* Start/Stop the TLC throughput counters for both RX and TX */
int t19x_nvlink_freeze_tp_counters(struct tnvlink_dev *tdev, bool freeze)
{
	int ret = 0;
	u32 data_tx, data_rx;

	if(tdev == NULL) {
		nvlink_err("Invalid tnvlink_dev pointer");
		return -EINVAL;
	}


	/* TX tp counters */
	data_tx = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL);

	/* RX tp counters */
	data_rx = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL);

	if (!freeze) {
		tdev->is_tp_cntr_running = true;
		data_tx |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_ENTX0);
		data_tx |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_ENTX1);
		data_rx |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_ENRX0);
		data_rx |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_ENRX1);
	} else {
		tdev->is_tp_cntr_running = false;
		data_tx &= ~(BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_ENTX0));
		data_tx &= ~(BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_ENTX1));
		data_rx &= ~(BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_ENRX0));
		data_rx &= ~(BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_ENRX1));
	}

	nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL, data_tx);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL, data_rx);

	return ret;
}

/* Configure the TLC throughput counters 0 and 1 for both RX and TX
 * Configuration for cntr0-
 * 1. Setting CTRL0.UNIT of traffic the counter will counter to "PACKETS"
 * 2. Setting CTRL0.VCSETFILTERMODE to count both VCSET0 and 1
 * Configuration for cntr1-
 * 1. Setting CTRL0.UNIT of traffic the counter will counter to "CYCLES"
 * 2. Setting CTRL0.VCSETFILTERMODE to count both VCSET0 and 1
 * 3. Setting CTRL0.FLITFILTER to count idle cycles
 */
int t19x_nvlink_config_tp_counters(struct tnvlink_dev *tdev)
{
	int ret = 0;
	u32 data;

	if(tdev == NULL) {
		nvlink_err("Invalid tnvlink_dev pointer");
		return -EINVAL;
	}

	/* TX tp counter cntr0 */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0);
	data &= ~(NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_UNIT_M() |
			NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_M());
	data |= NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_UNIT_F(
			NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_UNIT_PACKETS);
	data |= NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_F(
			NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_INIT);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_CTRL_0, data);

	/* TX tp counter cntr1 */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0);
	data &= ~(NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_UNIT_M() |
			NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_M() |
			NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_M());
	data |= NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_UNIT_F(
			NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_UNIT_CYCLES);
	data |= NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_F(
			NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_INIT);
	data |= NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_F(
			NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_IDLE);
	nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_CTRL_0, data);

	/* RX tp counter cntr0 */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0);
	data &= ~(NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_UNIT_M() |
			NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_M());
	data |= NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_UNIT_F(
			NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_UNIT_PACKETS);
	data |= NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_F(
			NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0_VCSETFILTERMODE_INIT);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_CTRL_0, data);

	/* RX tp counter cntr1 */
	data = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0);
	data &= ~(NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_UNIT_M() |
			NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_M() |
			NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_M());
	data |= NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_UNIT_F(
			NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_UNIT_CYCLES);
	data |= NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_F(
			NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_VCSETFILTERMODE_INIT);
	data |= NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_F(
			NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0_FLITFILTER_IDLE);
	nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_CTRL_0, data);

	return ret;
};

/* Get the value of TLC throughput counters/controls for both RX and TX */
int t19x_nvlink_get_tp_counters(struct tnvlink_dev *tdev, u64 *tx0cnt,
					u64 *tx1cnt, u64 *rx0cnt, u64 *rx1cnt)
{
	int ret = 0;
	u64 tx0cntlo, tx1cntlo, rx0cntlo, rx1cntlo;
	u64 tx0cnthi, tx1cnthi, rx0cnthi, rx1cnthi;

	if(tdev == NULL) {
		nvlink_err("Invalid tnvlink_dev pointer");
		return -EINVAL;
	}

	/* TX tp counters */
	tx0cntlo = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_LO);
	tx0cnthi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_HI);
	tx1cntlo = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_LO);
	tx1cnthi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_HI);

	/* RX tp counters */
	rx0cntlo = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_LO);
	rx0cnthi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_HI);
	rx1cntlo = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_LO);
	rx1cnthi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_HI);

	*tx0cnt = ((u64)tx0cnthi << 32) | tx0cntlo;
	*tx1cnt = ((u64)tx1cnthi << 32) | tx1cntlo;
	*rx0cnt = ((u64)rx0cnthi << 32) | rx0cntlo;
	*rx1cnt = ((u64)rx1cnthi << 32) | rx1cntlo;

	nvlink_dbg("tx packets        : %llu\n", *tx0cnt);
	nvlink_dbg("tx idle cycles    : %llu\n", *tx1cnt);
	nvlink_dbg("rx packets        : %llu\n", *rx0cnt);
	nvlink_dbg("rx idle cycles    : %llu\n", *rx1cnt);

	return ret;
}

