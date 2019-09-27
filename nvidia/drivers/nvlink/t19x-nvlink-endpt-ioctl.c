/*
 * t19x-nvlink-endpt-ioctl.c:
 * This file adds various IOCTLs for the Tegra NVLINK controller.
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

#include <linux/uaccess.h>
#include <linux/clk.h>
#include <uapi/linux/tegra-nvlink-uapi.h>

#include "t19x-nvlink-endpt.h"
#include "nvlink-hw.h"

#define TNVLINK_LINK_ID_TO_MASK(link_id)	BIT(link_id)

static int get_nvlink_caps_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int get_nvlink_status_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int clear_counters_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_counters_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_err_info_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_error_recoveries_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int setup_eom_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int train_intranode_conn_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int get_lp_counters_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int clear_lp_counters_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int enable_shim_driver_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int enable_device_interrupts_ioctl(struct tnvlink_dev *tdev,
						void *ioctl_struct);
static int service_device_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int disable_device_interrupts_ioctl(struct tnvlink_dev *tdev,
						void *ioctl_struct);
static int inject_err_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int set_link_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_link_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int set_tx_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_tx_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int set_rx_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int get_rx_mode_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct);
static int write_discovery_token_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int read_discovery_token_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int get_local_pci_info_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int set_topology_info_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int interface_disable_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);
static int finalize_shutdown_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct);

struct tnvlink_ioctl {
	const char *const name;
	const size_t struct_size;
	int (*handler)(struct tnvlink_dev *, void *);
	bool is_rm_shim_ioctl;
};

static const struct tnvlink_ioctl ioctls[] = {
	[TNVLINK_IOCTL_GET_NVLINK_CAPS] = {
		.name			= "get_nvlink_caps",
		.struct_size		= sizeof(struct tegra_nvlink_caps),
		.handler		= get_nvlink_caps_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_GET_NVLINK_STATUS] = {
		.name			= "get_nvlink_status",
		.struct_size		= sizeof(struct tegra_nvlink_status),
		.handler		= get_nvlink_status_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_CLEAR_COUNTERS] = {
		.name			= "clear_counters",
		.struct_size = sizeof(struct tegra_nvlink_clear_counters),
		.handler		= clear_counters_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_GET_COUNTERS] = {
		.name			= "get_counters",
		.struct_size = sizeof(struct tegra_nvlink_get_counters),
		.handler		= get_counters_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_GET_ERR_INFO] = {
		.name			= "get_err_info",
		.struct_size = sizeof(struct tegra_nvlink_get_err_info),
		.handler		= get_err_info_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_GET_ERROR_RECOVERIES] = {
		.name			= "get_error_recoveries",
		.struct_size = sizeof(struct tegra_nvlink_get_error_recoveries),
		.handler		= get_error_recoveries_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_SETUP_EOM] = {
		.name			= "setup_eom",
		.struct_size		= sizeof(struct tegra_nvlink_setup_eom),
		.handler		= setup_eom_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_TRAIN_INTRANODE_CONN] = {
		.name			= "train_intranode_conn",
		.struct_size = sizeof(struct tegra_nvlink_train_intranode_conn),
		.handler		= train_intranode_conn_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_GET_LP_COUNTERS] = {
		.name			= "get_lp_counters",
		.struct_size = sizeof(struct tegra_nvlink_get_lp_counters),
		.handler		= get_lp_counters_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_CLEAR_LP_COUNTERS] = {
		.name			= "clear_lp_counters",
		.struct_size = sizeof(struct tegra_nvlink_clear_lp_counters),
		.handler		= clear_lp_counters_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_ENABLE_SHIM_DRIVER] = {
		.name			= "enable_shim_driver",
		.struct_size		= 0,
		.handler		= enable_shim_driver_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_ENABLE_DEVICE_INTERRUPTS] = {
		.name			= "enable_device_interrupts",
		.struct_size =
			sizeof(struct tegra_nvlink_enable_device_interrupts),
		.handler		= enable_device_interrupts_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_SERVICE_DEVICE] = {
		.name			= "service_device",
		.struct_size = sizeof(struct tegra_nvlink_service_device),
		.handler		= service_device_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_DISABLE_DEVICE_INTERRUPTS] = {
		.name			= "disable_device_interrupts",
		.struct_size =
			sizeof(struct tegra_nvlink_disable_device_interrupts),
		.handler		= disable_device_interrupts_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_INJECT_ERR] = {
		.name			= "inject_err",
		.struct_size = sizeof(struct tegra_nvlink_inject_err),
		.handler		= inject_err_ioctl,
		.is_rm_shim_ioctl	= false,
	},
	[TNVLINK_IOCTL_SET_LINK_MODE] = {
		.name			= "set_link_mode",
		.struct_size = sizeof(struct tegra_nvlink_set_link_mode),
		.handler		= set_link_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_GET_LINK_MODE] = {
		.name			= "get_link_mode",
		.struct_size = sizeof(struct tegra_nvlink_get_link_mode),
		.handler		= get_link_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_SET_TX_MODE] = {
		.name			= "set_tx_mode",
		.struct_size = sizeof(struct tegra_nvlink_set_tx_mode),
		.handler		= set_tx_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_GET_TX_MODE] = {
		.name			= "get_tx_mode",
		.struct_size = sizeof(struct tegra_nvlink_get_tx_mode),
		.handler		= get_tx_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_SET_RX_MODE] = {
		.name			= "set_rx_mode",
		.struct_size = sizeof(struct tegra_nvlink_set_rx_mode),
		.handler		= set_rx_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_GET_RX_MODE] = {
		.name			= "get_rx_mode",
		.struct_size = sizeof(struct tegra_nvlink_get_rx_mode),
		.handler		= get_rx_mode_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_WRITE_DISCOVERY_TOKEN] = {
		.name			= "write_discovery_token",
		.struct_size
			= sizeof(struct tegra_nvlink_write_discovery_token),
		.handler		= write_discovery_token_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_READ_DISCOVERY_TOKEN] = {
		.name			= "read_discovery_token",
		.struct_size = sizeof(struct tegra_nvlink_read_discovery_token),
		.handler		= read_discovery_token_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_GET_LOCAL_PCI_INFO] = {
		.name			= "get_local_pci_info",
		.struct_size = sizeof(struct tegra_nvlink_get_local_pci_info),
		.handler		= get_local_pci_info_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_SET_TOPOLOGY_INFO] = {
		.name			= "set_topology_info",
		.struct_size = sizeof(struct tegra_nvlink_set_topology_info),
		.handler		= set_topology_info_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_INTERFACE_DISABLE] = {
		.name			= "interface_disable",
		.struct_size		= 0,
		.handler		= interface_disable_ioctl,
		.is_rm_shim_ioctl	= true,
	},
	[TNVLINK_IOCTL_FINALIZE_SHUTDOWN] = {
		.name			= "finalize_shutdown",
		.struct_size		= 0,
		.handler		= finalize_shutdown_ioctl,
		.is_rm_shim_ioctl	= true,
	},
};

static bool is_nvlink_loopback_topology(struct tnvlink_dev *tdev)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct nvlink_link *link = &ndev->link;

	if (link->device_id == NVLINK_ENDPT_T19X &&
		link->remote_dev_info.device_id == NVLINK_ENDPT_T19X)
		return true;

	return false;
}

static int get_nvlink_caps_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_caps *caps =
				(struct tegra_nvlink_caps *)ioctl_struct;

	caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_SUPPORTED |
				TEGRA_CTRL_NVLINK_CAPS_VALID;
	/* Sysmem atomics are supported for NVLINK versions > 1.0 */
	if (NVLINK_IP_VERSION > TEGRA_NVLINK_VERSION_10)
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS;

	switch (NVLINK_IP_VERSION) {
	case TEGRA_NVLINK_VERSION_22:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2;
		break;
	case TEGRA_NVLINK_VERSION_20:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		break;
	default:
		caps->lowest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0;
		caps->highest_nvlink_version =
				TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0;
		caps->lowest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0;
		caps->highest_nci_version =
				TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0;

		/* Supported power states */
		caps->nvlink_caps |= TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0;
		break;
	}

	/*
	 * Discovered = discovered in discovery table
	 * Enabled = present in discovery table and not disabled through a
	 *           registry key
	 *
	 * Since we don't use the discovery table right now and we don't have
	 * registry keys for disabling links, just return 0x1 for both the
	 * disovered and enabled link masks.
	 * TODO: Set the discovered and enabled link masks based on the
	 *       discovery table + registry keys (if we implement these)
	 */
	caps->discovered_link_mask =
			TNVLINK_LINK_ID_TO_MASK(0);
	caps->enabled_link_mask =
			TNVLINK_LINK_ID_TO_MASK(0);

	return 0;
}

static int get_nvlink_status_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct nvlink_link *nlink = &ndev->link;
	struct tegra_nvlink_status *status =
				(struct tegra_nvlink_status *)ioctl_struct;
	struct nvlink_device_pci_info *local_pci_info =	&ndev->pci_info;
	struct nvlink_device_pci_info *remote_pci_info =
					&nlink->remote_dev_info.pci_info;
	u32 reg_val = 0;
	u32 state = 0;

	if (tdev->rm_shim_enabled) {
		nlink->is_connected =
			is_link_connected((struct tnvlink_link *)nlink->priv);
	}
	status->link_info.connected = nlink->is_connected;

	status->link_info.remote_device_link_number =
					nlink->remote_dev_info.link_id;
	status->link_info.local_device_link_number = nlink->link_id;

	status->link_info.local_device_info.device_type =
				TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA;
	status->link_info.local_device_info.domain = local_pci_info->domain;
	status->link_info.local_device_info.bus = local_pci_info->bus;
	status->link_info.local_device_info.device = local_pci_info->device;
	status->link_info.local_device_info.function = local_pci_info->function;
	status->link_info.local_device_info.pci_device_id =
						local_pci_info->pci_device_id;

	status->link_info.remote_device_info.domain = remote_pci_info->domain;
	status->link_info.remote_device_info.bus = remote_pci_info->bus;
	status->link_info.remote_device_info.device = remote_pci_info->device;
	status->link_info.remote_device_info.function =
						remote_pci_info->function;
	status->link_info.remote_device_info.pci_device_id =
						remote_pci_info->pci_device_id;

	if (nlink->remote_dev_info.device_id == NVLINK_ENDPT_T19X) {
		status->link_info.loop_property =
			TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPBACK;
		status->link_info.remote_device_info.device_type =
				TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA;
	} else if (nlink->remote_dev_info.device_id == NVLINK_ENDPT_GV100) {
		status->link_info.loop_property =
				TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_NONE;
		status->link_info.remote_device_info.device_type =
				TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_GPU;
	} else {
		nvlink_err("Invalid remote device ID");
		return -ENODEV;
	}

	status->link_info.caps |= TEGRA_CTRL_NVLINK_CAPS_VALID;

	status->enabled_link_mask =
			TNVLINK_LINK_ID_TO_MASK(nlink->link_id);
	status->link_info.phy_type = TEGRA_CTRL_NVLINK_STATUS_PHY_NVHS;
	status->link_info.sublink_width = 8;

	status->link_info.link_state = t19x_nvlink_get_link_state(ndev);

	t19x_nvlink_get_tx_sublink_state(ndev, &state);
	status->link_info.tx_sublink_status = (u8)state;

	t19x_nvlink_get_rx_sublink_state(ndev, &state);
	status->link_info.rx_sublink_status = (u8)state;

	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_CONFIG_RX);
	if (reg_val & BIT(NVL_SL1_CONFIG_RX_REVERSAL_OVERRIDE)) {
		/* Overridden */
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_LANE_REVERSE))
			status->link_info.bLane_reversal = true;
		else
			status->link_info.bLane_reversal = false;
	} else {
		/* Sensed in HW */
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_HW_LANE_REVERSE))
			status->link_info.bLane_reversal = true;
		else
			status->link_info.bLane_reversal = false;
	}

	switch (NVLINK_IP_VERSION) {
	case TEGRA_NVLINK_VERSION_22:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2;
		break;
	case TEGRA_NVLINK_VERSION_20:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_0;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_0;
		break;
	default:
		status->link_info.nvlink_version =
				TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_1_0;
		status->link_info.nci_version =
				TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_1_0;
		break;
	}
	status->link_info.phy_version =
				TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_1_0;

	status->link_info.nvlink_link_clockKHz = ndev->link_bitrate / 1000;
	if (tdev->refclk == NVLINK_REFCLK_150)
		status->link_info.nvlink_ref_clk_speedKHz = 150000;
	else if (tdev->refclk == NVLINK_REFCLK_156)
		status->link_info.nvlink_ref_clk_speedKHz = 156250;

	status->link_info.nvlink_common_clock_speedKHz =
				status->link_info.nvlink_link_clockKHz / 16;

	status->link_info.nvlink_link_clockMhz =
				status->link_info.nvlink_link_clockKHz / 1000;
	status->link_info.nvlink_ref_clk_speedMhz =
			status->link_info.nvlink_ref_clk_speedKHz / 1000;
	status->link_info.nvlink_common_clock_speedMhz =
			status->link_info.nvlink_common_clock_speedKHz / 1000;

	status->link_info.nvlink_ref_clk_type =
				TEGRA_CTRL_NVLINK_REFCLK_TYPE_NVHS;

	return 0;
}

static int clear_counters_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_clear_counters *clear_counters =
			(struct tegra_nvlink_clear_counters *)ioctl_struct;
	u32 reg_val = 0;
	u32 counter_mask = clear_counters->counter_mask;

	if (clear_counters->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_val = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)
			reg_val |= BIT(NVLTLC_TX_DEBUG_TP_CNTR_CTRL_RESETTX1);
		nvlw_nvltlc_writel(tdev, NVLTLC_TX_DEBUG_TP_CNTR_CTRL, reg_val);

		reg_val = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX0);
		if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)
			reg_val |= BIT(NVLTLC_RX_DEBUG_TP_CNTR_CTRL_RESETRX1);
		nvlw_nvltlc_writel(tdev, NVLTLC_RX_DEBUG_TP_CNTR_CTRL, reg_val);
	}

	if ((counter_mask) & (TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L0 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L1 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L2 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L3 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L4 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L5 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L6 |
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L7)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_LANE_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(tdev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_FLIT_CRC);
		reg_val |= BIT(NVL_SL1_ERROR_COUNT_CTRL_CLEAR_RATES);
		nvlw_nvl_writel(tdev, NVL_SL1_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL0_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_SL0_ERROR_COUNT_CTRL_CLEAR_REPLAY);
		nvlw_nvl_writel(tdev, NVL_SL0_ERROR_COUNT_CTRL, reg_val);
	}

	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_ERROR_COUNT_CTRL);
		reg_val |= BIT(NVL_ERROR_COUNT_CTRL_CLEAR_RECOVERY);
		nvlw_nvl_writel(tdev, NVL_ERROR_COUNT_CTRL, reg_val);
	}

	return 0;
}

static bool t19x_nvlink_is_lane_reversal(struct tnvlink_dev *tdev)
{
	u32 reg_val;
	bool lane_reversal;

	reg_val = nvlw_nvl_readl(tdev, NVL_SL1_CONFIG_RX);
	if (reg_val & BIT(NVL_SL1_CONFIG_RX_REVERSAL_OVERRIDE)) {
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_LANE_REVERSE))
			lane_reversal = true;
		else
			lane_reversal = false;
	} else {
		if (reg_val & BIT(NVL_SL1_CONFIG_RX_HW_LANE_REVERSE))
			lane_reversal = true;
		else
			lane_reversal = false;
	}

	return lane_reversal;
}

static void t19x_nvlink_get_lane_crc_errors(struct tnvlink_dev *tdev,
				struct tegra_nvlink_get_counters *get_counters)
{
	int i;
	int lane_id;
	u32 reg_val;
	u64 lane_crc_val;
	u32 counter_mask = get_counters->counter_mask;

	for (i = 0; i < TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_SIZE;
				i++) {
		if (counter_mask &
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(i)) {

			lane_id = i;

			if (t19x_nvlink_is_lane_reversal(tdev))
				lane_id = 7 - lane_id;

			if (lane_id < 4) {
				reg_val = nvlw_nvl_readl(tdev,
						NVL_SL1_ERROR_COUNT2_LANECRC);
			} else {
				reg_val = nvlw_nvl_readl(tdev,
						NVL_SL1_ERROR_COUNT3_LANECRC);
			}

			switch (lane_id) {
			case 0:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L0_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 1:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L1_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 2:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L2_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 3:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT2_LANECRC_L3_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 4:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L4_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 5:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L5_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 6:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L6_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			case 7:
				lane_crc_val = (u64)
					NVL_SL1_ERROR_COUNT3_LANECRC_L7_V(
					reg_val);
				get_counters->nvlink_counters[
					tegra_nvlink_counter(i)] = lane_crc_val;
				break;
			default:
				break;
			}
		}
	}
}

static int get_counters_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_get_counters *get_counters =
			(struct tegra_nvlink_get_counters *)ioctl_struct;
	u32 reg_val = 0;
	u64 reg_low = 0;
	u64 reg_hi = 0;
	u32 counter_mask = get_counters->counter_mask;

	if (get_counters->link_id != tdev->ndev->link.link_id) {
		nvlink_err("Invalid link ID specified");
		return -EINVAL;
	}

	if (counter_mask & (TEGRA_CTRL_NVLINK_COUNTER_TL_TX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0 |
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)) {
		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bTx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_TX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bTx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_TX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_TX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR0_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER)) {
			get_counters->bRx0_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR0_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX0)] |=
				(((u64) 0xffffffff & reg_hi) << 32);

		reg_low = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_LO);
		reg_hi = nvlw_nvltlc_readl(tdev, NVLTLC_RX_DEBUG_TP_CNTR1_HI);
		if (reg_hi & BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER)) {
			get_counters->bRx1_tl_counter_overflow = true;
			reg_hi &= ~BIT(NVLTLC_RX_DEBUG_TP_CNTR1_HI_ROLLOVER);
		}
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] = 0;
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				((u64) 0xffffffff & reg_low);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
				TEGRA_CTRL_NVLINK_COUNTER_TL_RX1)] |=
				(((u64) 0xffffffff & reg_hi) << 32);
	}

	/* Get the count of flit CRC errors */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL1_ERROR_COUNT1);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT)] =
			(u64)NVL_SL1_ERROR_COUNT1_FLIT_CRC_ERRORS_V(reg_val);
	}

	/* Get the count of lane CRC errors */
	t19x_nvlink_get_lane_crc_errors(tdev, get_counters);

	/* Get the count of replays for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_SL0_ERROR_COUNT4);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY)] =
			(u64) NVL_SL0_ERROR_COUNT4_REPLAY_EVENTS_V(reg_val);
	}

	/*  Get the count of HW recoveries for the link */
	if (counter_mask & TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY) {
		reg_val = nvlw_nvl_readl(tdev, NVL_ERROR_COUNT1);
		get_counters->nvlink_counters[TEGRA_BIT_IDX_32(
			TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY)] =
			(u64) NVL_ERROR_COUNT1_RECOVERY_EVENTS_V(reg_val);
	}

	return 0;
}

static int get_err_info_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct nvlink_link *nlink = &ndev->link;
	struct tegra_nvlink_get_err_info *get_err_info =
			(struct tegra_nvlink_get_err_info *)ioctl_struct;
	u32 reg_val = 0;
	u32 state = 0;
	bool excess_err_dl = false;

	get_err_info->link_mask = TNVLINK_LINK_ID_TO_MASK(nlink->link_id);

	get_err_info->link_err_info.tl_err_log = 0;
	get_err_info->link_err_info.tl_intr_en = 0;

	get_err_info->link_err_info.tlc_tx_err_status0 =
				tdev->tlink.tlc_tx_err_status0;
	get_err_info->link_err_info.tlc_rx_err_status0 =
				tdev->tlink.tlc_rx_err_status0;
	get_err_info->link_err_info.tlc_rx_err_status1 =
				tdev->tlink.tlc_rx_err_status1;

	get_err_info->link_err_info.tlc_tx_err_log_en0 =
				nvlw_nvltlc_readl(tdev, NVLTLC_TX_ERR_STATUS_0);
	get_err_info->link_err_info.tlc_rx_err_log_en0 =
				nvlw_nvltlc_readl(tdev, NVLTLC_RX_ERR_STATUS_0);
	get_err_info->link_err_info.tlc_rx_err_log_en1 =
				nvlw_nvltlc_readl(tdev, NVLTLC_RX_ERR_STATUS_1);

	/* Reset Errlog after clients get the value */
	tdev->tlink.tlc_tx_err_status0 = 0;
	tdev->tlink.tlc_rx_err_status0 = 0;
	tdev->tlink.tlc_rx_err_status1 = 0;

	/* MIF blocks doesn't exist on tegra. Hence 0 the mif err fields */
	get_err_info->link_err_info.mif_tx_err_status0 = 0;
	get_err_info->link_err_info.mif_rx_err_status0 = 0;

	t19x_nvlink_get_tx_sublink_state(ndev, &state);
	get_err_info->link_err_info.dl_speed_status_tx = state;

	t19x_nvlink_get_rx_sublink_state(ndev, &state);
	get_err_info->link_err_info.dl_speed_status_rx = state;

	if (nvlw_nvl_readl(tdev, NVL_INTR_STALL_EN) &
				BIT(NVL_INTR_STALL_EN_RX_SHORT_ERROR_RATE)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_INTR);
		if (reg_val & BIT(NVL_INTR_RX_SHORT_ERROR_RATE)) {
			excess_err_dl = true;
			nvlw_nvl_writel(tdev, NVL_INTR, reg_val);
		}
	}

	get_err_info->link_err_info.bExcess_error_dl = excess_err_dl;

	return 0;
}

/* Get the number of successful error recoveries */
static int get_error_recoveries_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct)
{
	struct tegra_nvlink_get_error_recoveries *get_err_recoveries =
		(struct tegra_nvlink_get_error_recoveries *)ioctl_struct;

	if (get_err_recoveries->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	get_err_recoveries->num_recoveries = tdev->tlink.error_recoveries;
	/* Clear the counts */
	tdev->tlink.error_recoveries = 0;

	return 0;
}

static int setup_eom_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_setup_eom *setup_eom =
				(struct tegra_nvlink_setup_eom *)ioctl_struct;

	if (setup_eom->link_id != tdev->ndev->link.link_id) {
		nvlink_err("Invalid link ID specified");
		return -EINVAL;
	}

	return minion_send_cmd(tdev, MINION_NVLINK_DL_CMD_COMMAND_CONFIGEOM,
			setup_eom->params);
}

static void nvlink_get_endpoint_state(struct tnvlink_dev *tdev,
			struct tegra_nvlink_link_state *link_state)
{
	struct nvlink_device *ndev = tdev->ndev;

	link_state->link_mode = t19x_nvlink_get_link_mode(ndev);
	link_state->tx_sublink_mode = t19x_nvlink_get_sublink_mode(ndev, 0);
	link_state->rx_sublink_mode = t19x_nvlink_get_sublink_mode(ndev, 1);
}


static int train_intranode_conn_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct)
{
	struct nvlink_device *ndev = tdev->ndev;
	struct tegra_nvlink_train_intranode_conn *train_intranode_conn =
		(struct tegra_nvlink_train_intranode_conn *)ioctl_struct;
	int ret;

	if (tdev->rm_shim_enabled) {
		nvlink_err("The TRAIN_INTRANODE_CONN IOCTL is currently"
			" disabled because the RM shim driver is enabled. When"
			" the shim driver is enabled,"
			" link state transitions/link training are handled by"
			" the RM NVLINK core driver. Therefore, the"
			" TRAIN_INTRANODE_CONN IOCTL is not needed when the RM"
			" shim driver is enabled. If you want to enable this"
			" IOCTL, disable the shim driver mode in the Tegra"
			" NVLINK endpoint driver.");
		ret = -ENOSYS;
		goto exit;
	}

	if (train_intranode_conn->src_end_point.node_id !=
			train_intranode_conn->dst_end_point.node_id) {
		nvlink_err("Source and destination node IDs don't match!");
		ret = -EINVAL;
		goto exit;
	}

	if (train_intranode_conn->src_end_point.link_index !=
			ndev->link.link_id) {
		nvlink_err("Source link ID mismatch!");
		ret = -EINVAL;
		goto exit;
	}

	if (train_intranode_conn->dst_end_point.link_index !=
			ndev->link.remote_dev_info.link_id) {
		nvlink_err("Destination link ID mismatch!");
		ret = -EINVAL;
		goto exit;
	}

	switch (train_intranode_conn->train_to) {
	case tegra_nvlink_train_conn_off_to_swcfg:
		ret = nvlink_transition_intranode_conn_off_to_safe(ndev);
		break;

	case tegra_nvlink_train_conn_swcfg_to_active:
		ret = nvlink_train_intranode_conn_safe_to_hs(ndev);
		break;

	case tegra_nvlink_train_conn_to_off:
		/* OFF state transitions are not supported/tested */
		nvlink_err("OFF state transitions are not supported");
		ret = -EINVAL;
		break;

	case tegra_nvlink_train_conn_active_to_swcfg:
		ret = nvlink_transition_intranode_conn_hs_to_safe(ndev);
		break;

	case tegra_nvlink_train_conn_swcfg_to_off:
		/* OFF state transitions are not supported/tested */
		nvlink_err("OFF state transitions are not supported");
		ret = -EINVAL;
		break;

	default:
		nvlink_err("Invalid training mode specified");
		ret = -EINVAL;
		break;
	}

	nvlink_get_endpoint_state(tdev, &train_intranode_conn->src_end_state);

	if (is_nvlink_loopback_topology(tdev)) {
		train_intranode_conn->dst_end_state.link_mode =
			train_intranode_conn->src_end_state.link_mode;
		train_intranode_conn->dst_end_state.tx_sublink_mode =
			train_intranode_conn->src_end_state.tx_sublink_mode;
		train_intranode_conn->dst_end_state.rx_sublink_mode =
			train_intranode_conn->src_end_state.rx_sublink_mode;
	} else {
		/* TODO: */
		/* Handle other topologies */
	}

exit:
	train_intranode_conn->status = ret;
	return ret;
}

/*
 * Get count for LP hardware counters that are specified in counter_valid_mask.
 * Clear unsupported ones in counter_valid_mask.
 */
static int get_lp_counters_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_get_lp_counters *get_lp_counters =
			(struct tegra_nvlink_get_lp_counters *)ioctl_struct;
	u32 counter_valid_mask_out = 0;
	u32 counter_valid_mask = get_lp_counters->counter_valid_mask;
	u32 reg_val;
	u32 cnt_idx;

	if (get_lp_counters->link_id != tdev->ndev->link.link_id) {
		nvlink_err("Invalid link ID specified");
		return -EINVAL;
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_NVHS;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_A);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_A_COUNT_TX_STATE_NVHS_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_EIGHTH;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_A);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_A_COUNT_TX_STATE_EIGHTH_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_OTHER;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_B);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_B_COUNT_TX_STATE_OTHER_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_ENTER;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_D);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_D_NUM_TX_LP_ENTER_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	cnt_idx = TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_EXIT;
	if (counter_valid_mask & BIT(cnt_idx)) {
		reg_val = nvlw_nvl_readl(tdev, NVL_STATS_D);
		get_lp_counters->counter_values[cnt_idx] =
				NVL_STATS_D_NUM_TX_LP_EXIT_V(reg_val);
		counter_valid_mask_out |= BIT(cnt_idx);
	}

	get_lp_counters->counter_valid_mask = counter_valid_mask_out;

	return 0;
}

static int clear_lp_counters_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_clear_lp_counters *clear_lp_counters =
			(struct tegra_nvlink_clear_lp_counters *)ioctl_struct;
	u32 reg_val = 0;

	if (clear_lp_counters->link_id != tdev->ndev->link.link_id) {
		nvlink_err("Invalid link ID specified");
		return -EINVAL;
	}

	reg_val = nvlw_nvl_readl(tdev, NVL_STATS_CTRL);
	reg_val |= BIT(NVL_STATS_CTRL_CLEAR_ALL);
	nvlw_nvl_writel(tdev, NVL_STATS_CTRL, reg_val);

	return 0;
}

static int enable_shim_driver_ioctl(struct tnvlink_dev *tdev,
				void *ioctl_struct)
{
	struct nvlink_device *ndev = tdev->ndev;

	tdev->rm_shim_enabled = true;
	ndev->device_id = NVLINK_ENDPT_T19X;
	ndev->link.device_id = ndev->device_id;
	ndev->is_master = false;
	/*
	 * Right now we only support a T19x+GV100 topology for the RM shim
	 * mode
	 */
	ndev->link.remote_dev_info.device_id = NVLINK_ENDPT_GV100;

	/*
	 * In RM shim driver mode we use the RM core driver instead of the Tegra
	 * core driver. Therefore, we're unregistering from the Tegra core
	 * driver.
	 */
	nvlink_unregister_link(&ndev->link);
	nvlink_unregister_device(ndev);

	return 0;
}

static int enable_device_interrupts_ioctl(struct tnvlink_dev *tdev,
						void *ioctl_struct)
{
	struct tegra_nvlink_enable_device_interrupts *device_interrupts =
		(struct tegra_nvlink_enable_device_interrupts *)ioctl_struct;

	if (device_interrupts->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	nvlink_enable_dl_interrupts(tdev);

	return 0;
}

static int service_device_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	int ret = 0;
	struct tegra_nvlink_service_device *service_device =
			(struct tegra_nvlink_service_device *)ioctl_struct;
	bool retrain_from_safe = false;

	if (service_device->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	ret = nvlink_service_dl_interrupts(tdev, &retrain_from_safe);
	if ((ret == 0) && retrain_from_safe) {
		service_device->retrain_from_safe_mask =
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id);
	}

	return ret;
}

static int disable_device_interrupts_ioctl(struct tnvlink_dev *tdev,
						void *ioctl_struct)
{
	struct tegra_nvlink_disable_device_interrupts *device_interrupts =
		(struct tegra_nvlink_disable_device_interrupts *)ioctl_struct;

	if (device_interrupts->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	nvlink_disable_dl_interrupts(tdev);

	return 0;
}

static int inject_err_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	int ret = 0;
	struct tegra_nvlink_inject_err *inject_err =
				(struct tegra_nvlink_inject_err *)ioctl_struct;

	if (inject_err->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	if (inject_err->is_fatal_error) {
		/*
		 * Choice of error is somewhat arbitrary. If needed, we can add
		 * a field in struct tegra_nvlink_inject_err to allow userspace
		 * to control which specific error they want to inject.
		 */
		nvlink_dbg("Injecting RAM data parity error on link");
		nvlw_nvltlc_writel(tdev,
				NVLTLC_RX_ERR_INJECT_0,
				BIT(NVLTLC_RX_ERR_INJECT_0_RXRAMDATAPARITYERR));
	} else {
		nvlink_dbg("Injecting RCVY_AC error on link");
		ret = t19x_nvlink_set_link_mode(tdev->ndev,
						NVLINK_LINK_RCVY_AC);
	}

	return ret;
}

static int set_link_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_set_link_mode *set_link_mode =
			(struct tegra_nvlink_set_link_mode *)ioctl_struct;

	if (set_link_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	return t19x_nvlink_set_link_mode(tdev->ndev, set_link_mode->link_mode);
}

static int get_link_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_get_link_mode *get_link_mode =
			(struct tegra_nvlink_get_link_mode *)ioctl_struct;

	if (get_link_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	get_link_mode->link_mode = t19x_nvlink_get_link_mode(tdev->ndev);

	return 0;
}

static int set_tx_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_set_tx_mode *set_tx_mode =
			(struct tegra_nvlink_set_tx_mode *)ioctl_struct;

	if (set_tx_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	return t19x_nvlink_set_sublink_mode(tdev->ndev,
						false,
						set_tx_mode->tx_mode);
}

static int get_tx_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_get_tx_mode *get_tx_mode =
			(struct tegra_nvlink_get_tx_mode *)ioctl_struct;

	if (get_tx_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	get_tx_mode->tx_mode = t19x_nvlink_get_sublink_mode(tdev->ndev, false);

	return 0;
}

static int set_rx_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_set_rx_mode *set_rx_mode =
			(struct tegra_nvlink_set_rx_mode *)ioctl_struct;

	if (set_rx_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	return t19x_nvlink_set_sublink_mode(tdev->ndev,
						true,
						set_rx_mode->rx_mode);
}

static int get_rx_mode_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	struct tegra_nvlink_get_rx_mode *get_rx_mode =
			(struct tegra_nvlink_get_rx_mode *)ioctl_struct;

	if (get_rx_mode->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	get_rx_mode->rx_mode = t19x_nvlink_get_sublink_mode(tdev->ndev, true);

	return 0;
}

static int write_discovery_token_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct)
{
	struct tegra_nvlink_write_discovery_token *write_discovery_token =
		(struct tegra_nvlink_write_discovery_token *)ioctl_struct;

	if (write_discovery_token->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	return t19x_nvlink_write_discovery_token(tdev,
						write_discovery_token->token);
}

static int read_discovery_token_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct)
{
	struct tegra_nvlink_read_discovery_token *read_discovery_token =
		(struct tegra_nvlink_read_discovery_token *)ioctl_struct;

	if (read_discovery_token->link_mask !=
			TNVLINK_LINK_ID_TO_MASK(tdev->ndev->link.link_id)) {
		nvlink_err("Invalid link mask specified");
		return -EINVAL;
	}

	return t19x_nvlink_read_discovery_token(tdev,
						&read_discovery_token->token);
}

static int get_local_pci_info_ioctl(struct tnvlink_dev *tdev,
					void *ioctl_struct)
{
	struct tegra_nvlink_get_local_pci_info *get_local_pci_info =
		(struct tegra_nvlink_get_local_pci_info *)ioctl_struct;
	struct tegra_nvlink_device_info *user_local_endpt =
					&get_local_pci_info->local_endpt;
	struct nvlink_device_pci_info *kernel_local_endpt =
							&tdev->ndev->pci_info;

	user_local_endpt->domain = kernel_local_endpt->domain;
	user_local_endpt->bus = kernel_local_endpt->bus;
	user_local_endpt->device = kernel_local_endpt->device;
	user_local_endpt->function = kernel_local_endpt->function;
	user_local_endpt->pci_device_id = kernel_local_endpt->pci_device_id;

	user_local_endpt->device_type =
				TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA;

	return 0;
}

static int set_topology_info_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	int ret = 0;
	struct tegra_nvlink_set_topology_info *set_topology_info =
			(struct tegra_nvlink_set_topology_info *)ioctl_struct;
	struct tegra_nvlink_device_info *user_remote_dev_info =
					&set_topology_info->remote_endpt;
	struct nvlink_device *ndev = tdev->ndev;
	struct nvlink_link *nlink = &ndev->link;
	struct nvlink_device_pci_info *kernel_local_pci_info = &ndev->pci_info;
	struct nvlink_device_pci_info *kernel_remote_pci_info =
					&nlink->remote_dev_info.pci_info;

	if (user_remote_dev_info->device_type !=
		TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_GPU) {
		nvlink_err("Invalid remote endpoint type (type = 0x%llx)",
			user_remote_dev_info->device_type);
		ret = -EINVAL;
		goto fail;
	}

	nlink->link_id = set_topology_info->local_link_id;
	nlink->remote_dev_info.link_id = set_topology_info->remote_link_id;

	kernel_remote_pci_info->domain = user_remote_dev_info->domain;
	kernel_remote_pci_info->bus = user_remote_dev_info->bus;
	kernel_remote_pci_info->device = user_remote_dev_info->device;
	kernel_remote_pci_info->function = user_remote_dev_info->function;
	kernel_remote_pci_info->pci_device_id =
					user_remote_dev_info->pci_device_id;
	if (memcmp(kernel_remote_pci_info,
			kernel_local_pci_info,
			sizeof(*kernel_remote_pci_info)) == 0) {
		nvlink_err("PCI information of remote and local devices is"
			" identical. This is an invalid configuration.");
		ret = -EINVAL;
		goto fail;
	}

	goto exit;
fail:
	/* In the case of an error, zero out all stored information. */
	nlink->link_id = 0;
	nlink->remote_dev_info.link_id = 0;
	memset(kernel_remote_pci_info,
		0,
		sizeof(*kernel_remote_pci_info));
exit:
	return ret;
}

static int interface_disable_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	return t19x_nvlink_dev_interface_disable(tdev->ndev);
}

static int finalize_shutdown_ioctl(struct tnvlink_dev *tdev, void *ioctl_struct)
{
	int ret = 0;

	ret = nvlink_set_init_state(tdev->ndev, NVLINK_DEV_OFF);
	if (ret < 0)
		nvlink_err("Failed to set init_state to NVLINK_DEV_OFF");

	return ret;
}

static long t19x_nvlink_endpt_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct tnvlink_dev *tdev = file->private_data;
	enum tnvlink_ioctl_num ioctl_num = _IOC_NR(cmd);
	u32 ioc_dir = _IOC_DIR(cmd);
	u32 arg_size = _IOC_SIZE(cmd);
	void *arg_copy = NULL;
	int ret = 0;

	if (!tdev) {
		nvlink_err("Invalid Tegra nvlink device");
		ret = -ENODEV;
		goto fail;
	}

	if ((_IOC_TYPE(cmd) != TEGRA_NVLINK_IOC_MAGIC) ||
		(ioctl_num < 0) ||
		(ioctl_num >= TNVLINK_IOCTL_NUM_IOCTLS)) {
		nvlink_err("Unsupported IOCTL call");
		return -EINVAL;
	}

	if (!tdev->rm_shim_enabled &&
		ioctls[ioctl_num].is_rm_shim_ioctl &&
		ioctl_num != TNVLINK_IOCTL_ENABLE_SHIM_DRIVER) {
		nvlink_err("The %s IOCTL can only be called when the RM shim"
			" driver is being used. Currently the RM shim driver is"
			" disabled. Therefore, the %s IOCTL has been"
			" disabled as well.",
			ioctls[ioctl_num].name,
			ioctls[ioctl_num].name);
		return -ENOSYS;
	}

	if (arg_size != ioctls[ioctl_num].struct_size) {
		nvlink_err("Invalid IOCTL struct passed from userspace");
		ret = -EINVAL;
		goto fail;
	}

	/* Only allocate a buffer if the IOCTL needs a buffer */
	if (!(ioc_dir & _IOC_NONE)) {
		arg_copy = kzalloc(arg_size, GFP_KERNEL);
		if (!arg_copy) {
			nvlink_err("Can't allocate memory for kernel IOCTL"
				" struct");
			ret = -ENOMEM;
			goto fail;
		}
	}

	if (ioc_dir & _IOC_WRITE) {
		if (copy_from_user(arg_copy, (void __user *)arg, arg_size)) {
			nvlink_err("Failed to copy data from userspace IOCTL"
				" struct into kernel IOCTL struct");
			ret = -EFAULT;
			goto fail;
		}
	}

	ret = ioctls[ioctl_num].handler(tdev, arg_copy);
	if (ret < 0)
		goto fail;

	if (ioc_dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, arg_copy, arg_size)) {
			nvlink_err("Failed to copy data from kernel IOCTL"
				" struct into userspace IOCTL struct");
			ret = -EFAULT;
			goto fail;
		}
	}

	nvlink_dbg("The %s IOCTL completed successfully!",
		ioctls[ioctl_num].name);
	goto cleanup;

fail:
	nvlink_err("The %s IOCTL failed!", ioctls[ioctl_num].name);
cleanup:
	kfree(arg_copy);
	return ret;
}

static int t19x_nvlink_endpt_open(struct inode *in, struct file *filp)
{
	int ret = 0;
	unsigned int minor = iminor(in);
	struct tnvlink_dev *tdev = container_of(in->i_cdev,
						struct tnvlink_dev,
						cdev);

	struct nvlink_device *ndev = tdev->ndev;

	if (minor > 0) {
		nvlink_err("Incorrect minor number");
		return -EBADFD;
	}

	if (ndev->is_master) {
		ret = nvlink_enumerate(ndev);
		if (ret < 0)
			nvlink_err("Failed to enable the link!");
		else
			nvlink_dbg("Link enabled successfully!");
	} else {
		/* This case is needed for the RM shim driver mode */
		nvlink_dbg("Because Tegra endpoint is not the NVLINK master,"
			" Tegra NVLINK's device node open will only perform"
			" Tegra NVLINK controller initialization. Link state"
			" transitions will be initiated later on by the NVLINK"
			" master endpoint.");
		ret = nvlink_initialize_endpoint(ndev);
	}
	filp->private_data = tdev;

	return ret;
}

static ssize_t t19x_nvlink_endpt_read(struct file *file,
				char __user *ubuf,
				size_t count,
				loff_t *offp)
{
	return 0;
}

static int t19x_nvlink_endpt_release(struct inode *inode, struct file *filp)
{
	return 0;
}

/* File ops for device node */
const struct file_operations t19x_nvlink_endpt_ops = {
	.owner = THIS_MODULE,
	.open = t19x_nvlink_endpt_open,
	.read = t19x_nvlink_endpt_read,
	.release = t19x_nvlink_endpt_release,
	.unlocked_ioctl = t19x_nvlink_endpt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = t19x_nvlink_endpt_ioctl,
#endif
};
