/*
 * tegra-nvlink-uapi.h:
 * This header contains the structures and variables needed for
 * the NVLINK userspace APIs exported by the Tegra NVLINK endpoint driver.
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TEGRA_NVLINK_UAPI_H
#define TEGRA_NVLINK_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if defined(__KERNEL__)
#include <linux/bitops.h>
#else
#define __user
#ifndef BIT
#define BIT(b) (1UL << (b))
#endif
#endif

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS */

#define TEGRA_NVLINK_VERSION_10				0x00000001U
#define TEGRA_NVLINK_VERSION_20				0x00000002U
#define TEGRA_NVLINK_VERSION_22				0x00000004U

#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_INVALID	(0x00000000U)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_1_0	(0x00000001U)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_0	(0x00000002U)
#define TEGRA_CTRL_NVLINK_CAPS_NVLINK_VERSION_2_2	(0x00000004U)

#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_INVALID	(0x00000000U)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_1_0		(0x00000001U)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_0		(0x00000002U)
#define TEGRA_CTRL_NVLINK_CAPS_NCI_VERSION_2_2		(0x00000004U)

#define TEGRA_CTRL_NVLINK_CAPS_SUPPORTED		BIT(0U)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_SUPPORTED		BIT(1U)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ACCESS		BIT(2U)
#define TEGRA_CTRL_NVLINK_CAPS_P2P_ATOMICS		BIT(3U)
#define TEGRA_CTRL_NVLINK_CAPS_SYSMEM_ATOMICS		BIT(4U)
#define TEGRA_CTRL_NVLINK_CAPS_PEX_TUNNELING		BIT(5U)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE		BIT(6U)
#define TEGRA_CTRL_NVLINK_CAPS_SLI_BRIDGE_SENSABLE	BIT(7U)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L0		BIT(8U)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L1		BIT(9U)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L2		BIT(10U)
#define TEGRA_CTRL_NVLINK_CAPS_POWER_STATE_L3		BIT(11U)
#define TEGRA_CTRL_NVLINK_CAPS_VALID			BIT(12U)

struct tegra_nvlink_caps {
	__u16 nvlink_caps;

	__u8 lowest_nvlink_version;
	__u8 highest_nvlink_version;
	__u8 lowest_nci_version;
	__u8 highest_nci_version;

	__u32 discovered_link_mask;
	__u32 enabled_link_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS */

/* NVLink link states */
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INIT		(0x00000000U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_HWCFG		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_SWCFG		(0x00000002U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_ACTIVE		(0x00000003U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_FAULT		(0x00000004U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_RECOVERY		(0x00000006U)
#define TEGRA_CTRL_NVLINK_STATUS_LINK_STATE_INVALID		(0xFFFFFFFFU)

/* NVLink Tx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_HIGH_SPEED_1	(0x00000000U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SINGLE_LANE	(0x00000004U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_TRAINING	(0x00000005U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_SAFE_MODE	(0x00000006U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_OFF		(0x00000007U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_RX_STATE_INVALID	(0x000000FFU)

/* NVLink Rx sublink states */
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_HIGH_SPEED_1	(0x00000000U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SINGLE_LANE	(0x00000004U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_TRAINING	(0x00000005U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_SAFE_MODE	(0x00000006U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_OFF		(0x00000007U)
#define TEGRA_CTRL_NVLINK_STATUS_SUBLINK_TX_STATE_INVALID	(0x000000FFU)

#define TEGRA_CTRL_NVLINK_STATUS_PHY_NVHS			(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_GRS			(0x00000002U)
#define TEGRA_CTRL_NVLINK_STATUS_PHY_INVALID			(0x000000FFU)

/* Version information */
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_1_0		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_0		(0x00000002U)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_2_2		(0x00000004U)
#define TEGRA_CTRL_NVLINK_STATUS_NVLINK_VERSION_INVALID		(0x000000FFU)

#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_1_0		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_0		(0x00000002U)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_2_2		(0x00000004U)
#define TEGRA_CTRL_NVLINK_STATUS_NCI_VERSION_INVALID		(0x000000FFU)

#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_1_0		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_NVHS_VERSION_INVALID		(0x000000FFU)

#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_1_0		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_GRS_VERSION_INVALID		(0x000000FFU)

/* Connection properties */
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_TRUE			(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_CONNECTED_FALSE		(0x00000000U)

#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPBACK		(0x00000001U)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_LOOPOUT		(0x00000002U)
#define TEGRA_CTRL_NVLINK_STATUS_LOOP_PROPERTY_NONE		(0x00000000U)

#define TEGRA_CTRL_NVLINK_STATUS_REMOTE_LINK_NUMBER_INVALID	(0x000000FFU)

/* NVLink REFCLK types */
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_INVALID			(0x00U)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_NVHS			(0x01U)
#define TEGRA_CTRL_NVLINK_REFCLK_TYPE_PEX			(0x02U)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_NONE	(0x00000000U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_PCI	(0x00000001U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_ID_FLAGS_UUID	(0x00000002U)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_EBRIDGE	(0x00000000U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NPU		(0x00000001U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_GPU		(0x00000002U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_SWITCH	(0x00000003U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_TEGRA		(0x00000004U)
#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_TYPE_NONE		(0x000000FFU)

#define TEGRA_CTRL_NVLINK_DEVICE_INFO_DEVICE_UUID_INVALID	(0xFFFFFFFFU)

struct tegra_nvlink_device_info {
	/* ID Flags */
	__u32 device_id_flags;

	/* PCI Information */
	__u16 domain;
	__u16 bus;
	__u16 device;
	__u16 function;
	__u32 pci_device_id;

	/* Device Type */
	__u64 device_type;

	/* Device UUID */
	__u8 device_uuid[16];
};

struct tegra_nvlink_link_status_info {
	/* Top level capablilites */
	__u16 caps;

	__u8 phy_type;
	__u8 sublink_width;

	/* Link and sublink states */
	__u32 link_state;
	__u8 rx_sublink_status;
	__u8 tx_sublink_status;

	/* Indicates that lane reveral is in effect on this link */
	bool bLane_reversal;

	__u8 nvlink_version;
	__u8 nci_version;
	__u8 phy_version;

	/* Clock information */
	__u32 nvlink_link_clockKHz;
	__u32 nvlink_common_clock_speedKHz;
	__u32 nvlink_ref_clk_speedKHz;
	__u8 nvlink_ref_clk_type;

	__u32 nvlink_link_clockMhz;
	__u32 nvlink_common_clock_speedMhz;
	__u32 nvlink_ref_clk_speedMhz;

	/* Connection information */
	bool connected;
	__u8 loop_property;
	__u8 remote_device_link_number;
	__u8 local_device_link_number;

	struct tegra_nvlink_device_info remote_device_info;
	struct tegra_nvlink_device_info local_device_info;
};

struct tegra_nvlink_status {
	__u32 enabled_link_mask;
	struct tegra_nvlink_link_status_info link_info;
};

/* TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS */

/* These are the bitmask definitions for different counter types */
#define TEGRA_CTRL_NVLINK_COUNTER_INVALID			0x00000000U

#define TEGRA_CTRL_NVLINK_COUNTER_TL_TX0			0x00000001U
#define TEGRA_CTRL_NVLINK_COUNTER_TL_TX1			0x00000002U
#define TEGRA_CTRL_NVLINK_COUNTER_TL_RX0			0x00000004U
#define TEGRA_CTRL_NVLINK_COUNTER_TL_RX1			0x00000008U

#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_FLIT		0x00010000U

#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(i) (1U << (i + 17U))
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_SIZE	8U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L0	0x00020000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L1	0x00040000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L2	0x00080000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L3	0x00100000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L4	0x00200000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L5	0x00400000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L6	0x00800000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L7	0x01000000U

#define TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_REPLAY		0x02000000U
#define TEGRA_CTRL_NVLINK_COUNTER_DL_TX_ERR_RECOVERY		0x04000000U

#define TEGRA_CTRL_NVLINK_COUNTER_MAX_TYPES			32U

/*
 * Return index of the bit that is set in 'n'. This assumes there is only
 * one such set bit in 'n'. Even if multiple bits are set,
 * result is in range of 0-31.
 */
#define TEGRA_BIT_IDX_32(n)						\
			((((n) & 0xFFFF0000U) ? 0x10U : 0U) |	\
			(((n) & 0xFF00FF00U) ? 0x08U : 0U) |	\
			(((n) & 0xF0F0F0F0U) ? 0x04U : 0U) |	\
			(((n) & 0xCCCCCCCCU) ? 0x02U : 0U) |	\
			(((n) & 0xAAAAAAAAU) ? 0x01U : 0U))

struct tegra_nvlink_clear_counters {
	__u32 link_mask;
	__u32 counter_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS */
#define tegra_nvlink_counter(x)	\
	TEGRA_BIT_IDX_32(TEGRA_CTRL_NVLINK_COUNTER_DL_RX_ERR_CRC_LANE_L(x))

struct tegra_nvlink_get_counters {
	__u8 link_id;
	__u32 counter_mask;
	bool bTx0_tl_counter_overflow;
	bool bTx1_tl_counter_overflow;
	bool bRx0_tl_counter_overflow;
	bool bRx1_tl_counter_overflow;
	__u64 nvlink_counters[TEGRA_CTRL_NVLINK_COUNTER_MAX_TYPES];
};

/* TEGRA_CTRL_CMD_NVLINK_GET_ERR_INFO */
struct tegra_nvlink_err_info {
	__u32 tl_err_log;
	__u32 tl_intr_en;
	__u32 tlc_tx_err_status0;
	__u32 tlc_rx_err_status0;
	__u32 tlc_rx_err_status1;
	__u32 tlc_tx_err_log_en0;
	__u32 tlc_rx_err_log_en0;
	__u32 tlc_rx_err_log_en1;
	__u32 mif_tx_err_status0;
	__u32 mif_rx_err_status0;
	__u32 dl_speed_status_tx;
	__u32 dl_speed_status_rx;
	bool bExcess_error_dl;
};

struct tegra_nvlink_get_err_info {
	__u32 link_mask;
	struct tegra_nvlink_err_info link_err_info;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_ERROR_RECOVERIES */
struct tegra_nvlink_get_error_recoveries {
	__u32 link_mask;
	__u32 num_recoveries;
};

/* TEGRA_CTRL_CMD_NVLINK_SETUP_EOM */
struct tegra_nvlink_setup_eom {
	__u8 link_id;
	__u32 params;
};

/* TEGRA_CTRL_NVLINK_TRAIN_INTRANODE_CONN */
enum tegra_ctrl_link_mode {
	TEGRA_CTRL_NVLINK_LINK_OFF,
	TEGRA_CTRL_NVLINK_LINK_HS,
	TEGRA_CTRL_NVLINK_LINK_SAFE,
	TEGRA_CTRL_NVLINK_LINK_FAULT,
	TEGRA_CTRL_NVLINK_LINK_RCVY_AC,
	TEGRA_CTRL_NVLINK_LINK_RCVY_SW,
	TEGRA_CTRL_NVLINK_LINK_RCVY_RX,
	TEGRA_CTRL_NVLINK_LINK_DETECT,
	TEGRA_CTRL_NVLINK_LINK_RESET,
	TEGRA_CTRL_NVLINK_LINK_ENABLE_PM,
	TEGRA_CTRL_NVLINK_LINK_DISABLE_PM,
	TEGRA_CTRL_NVLINK_LINK_DISABLE_ERR_DETECT,
	TEGRA_CTRL_NVLINK_LINK_LANE_DISABLE,
	TEGRA_CTRL_NVLINK_LINK_LANE_SHUTDOWN
};

enum tegra_ctrl_tx_mode {
	TEGRA_CTRL_NVLINK_TX_HS,
	TEGRA_CTRL_NVLINK_TX_ENABLE_PM,
	TEGRA_CTRL_NVLINK_TX_DISABLE_PM,
	TEGRA_CTRL_NVLINK_TX_SINGLE_LANE,
	TEGRA_CTRL_NVLINK_TX_SAFE,
	TEGRA_CTRL_NVLINK_TX_OFF,
	TEGRA_CTRL_NVLINK_TX_COMMON,
	TEGRA_CTRL_NVLINK_TX_COMMON_DISABLE,
	TEGRA_CTRL_NVLINK_TX_DATA_READY,
	TEGRA_CTRL_NVLINK_TX_PRBS_EN,
};

enum tegra_ctrl_rx_mode {
	TEGRA_CTRL_NVLINK_RX_HS,
	TEGRA_CTRL_NVLINK_RX_ENABLE_PM,
	TEGRA_CTRL_NVLINK_RX_DISABLE_PM,
	TEGRA_CTRL_NVLINK_RX_SINGLE_LANE,
	TEGRA_CTRL_NVLINK_RX_SAFE,
	TEGRA_CTRL_NVLINK_RX_OFF,
	TEGRA_CTRL_NVLINK_RX_RXCAL,
};

struct tegra_nvlink_pci_dev_info {
	__u16 domain;
	__u8 bus;
	__u8 device;
	__u8 function;
};

struct tegra_nvlink_endpoint {
	__u16 node_id;
	__u32 link_index;
	struct tegra_nvlink_pci_dev_info pci_info;
};

/* link and sublink state of an nvlink endpoint */
struct tegra_nvlink_link_state {
	__u64 link_mode;
	__u64 tx_sublink_mode;
	__u64 rx_sublink_mode;
};

enum tegra_nvlink_conn_train_type {
	tegra_nvlink_train_conn_off_to_swcfg = 0U,
	tegra_nvlink_train_conn_swcfg_to_active,
	tegra_nvlink_train_conn_to_off,
	tegra_nvlink_train_conn_active_to_swcfg,
	tegra_nvlink_train_conn_swcfg_to_off,
};

struct tegra_nvlink_train_intranode_conn {
	/* input fields */
	enum tegra_nvlink_conn_train_type train_to;
	struct tegra_nvlink_endpoint src_end_point;
	struct tegra_nvlink_endpoint dst_end_point;

	/* output fields */
	int status;
	struct tegra_nvlink_link_state src_end_state;
	struct tegra_nvlink_link_state dst_end_state;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_LP_COUNTERS */
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_NVHS		0U
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_EIGHTH	1U
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_COUNT_TX_OTHER	2U
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_ENTER	3U
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_NUM_TX_LP_EXIT	4U
#define TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_MAX_COUNTERS		5U

struct tegra_nvlink_get_lp_counters {
	/* input field */
	__u32 link_id;
	/* input, output field */
	__u32 counter_valid_mask;
	/* output field */
	__u32 counter_values[TEGRA_CTRL_NVLINK_GET_LP_COUNTERS_MAX_COUNTERS];
};

/* TEGRA_CTRL_CMD_NVLINK_CLEAR_LP_COUNTERS */
struct tegra_nvlink_clear_lp_counters {
	__u32 link_id;
};

/* TEGRA_CTRL_CMD_NVLINK_ENABLE_DEVICE_INTERRUPTS */
struct tegra_nvlink_enable_device_interrupts {
	__u32 link_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_SERVICE_DEVICE */
struct tegra_nvlink_service_device {
	__u32 link_mask;
	__u32 retrain_from_safe_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_DISABLE_DEVICE_INTERRUPTS */
struct tegra_nvlink_disable_device_interrupts {
	__u32 link_mask;
};

/* TEGRA_CTRL_CMD_NVLINK_INJECT_ERR */
struct tegra_nvlink_inject_err {
	__u32 link_mask;
	bool is_fatal_error;
};

/* TEGRA_CTRL_CMD_NVLINK_SET_LINK_MODE */
struct tegra_nvlink_set_link_mode {
	__u32 link_mask;
	enum tegra_ctrl_link_mode link_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_LINK_MODE */
struct tegra_nvlink_get_link_mode {
	__u32 link_mask;
	enum tegra_ctrl_link_mode link_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_SET_TX_MODE */
struct tegra_nvlink_set_tx_mode {
	__u32 link_mask;
	enum tegra_ctrl_tx_mode tx_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_TX_MODE */
struct tegra_nvlink_get_tx_mode {
	__u32 link_mask;
	enum tegra_ctrl_tx_mode tx_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_SET_RX_MODE */
struct tegra_nvlink_set_rx_mode {
	__u32 link_mask;
	enum tegra_ctrl_rx_mode rx_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_RX_MODE */
struct tegra_nvlink_get_rx_mode {
	__u32 link_mask;
	enum tegra_ctrl_rx_mode rx_mode;
};

/* TEGRA_CTRL_CMD_NVLINK_WRITE_DISCOVERY_TOKEN */
struct tegra_nvlink_write_discovery_token {
	__u32 link_mask;
	__u64 token;
};

/* TEGRA_CTRL_CMD_NVLINK_READ_DISCOVERY_TOKEN */
struct tegra_nvlink_read_discovery_token {
	__u32 link_mask;
	__u64 token;
};

/* TEGRA_CTRL_CMD_NVLINK_GET_LOCAL_PCI_INFO */
struct tegra_nvlink_get_local_pci_info {
	struct tegra_nvlink_device_info local_endpt;
};

/* TEGRA_CTRL_CMD_NVLINK_SET_TOPOLOGY_INFO */
struct tegra_nvlink_set_topology_info {
	struct tegra_nvlink_device_info remote_endpt;
	__u32 local_link_id;
	__u32 remote_link_id;
};

/* Enum to represent IOCTLs inside the Tegra NVLINK driver */
enum tnvlink_ioctl_num {
	TNVLINK_IOCTL_GET_NVLINK_CAPS,
	TNVLINK_IOCTL_GET_NVLINK_STATUS,
	TNVLINK_IOCTL_CLEAR_COUNTERS,
	TNVLINK_IOCTL_GET_COUNTERS,
	TNVLINK_IOCTL_GET_ERR_INFO,
	TNVLINK_IOCTL_GET_ERROR_RECOVERIES,
	TNVLINK_IOCTL_SETUP_EOM,
	TNVLINK_IOCTL_TRAIN_INTRANODE_CONN,
	TNVLINK_IOCTL_GET_LP_COUNTERS,
	TNVLINK_IOCTL_CLEAR_LP_COUNTERS,
	TNVLINK_IOCTL_ENABLE_SHIM_DRIVER,
	TNVLINK_IOCTL_ENABLE_DEVICE_INTERRUPTS,
	TNVLINK_IOCTL_SERVICE_DEVICE,
	TNVLINK_IOCTL_DISABLE_DEVICE_INTERRUPTS,
	TNVLINK_IOCTL_INJECT_ERR,
	TNVLINK_IOCTL_SET_LINK_MODE,
	TNVLINK_IOCTL_GET_LINK_MODE,
	TNVLINK_IOCTL_SET_TX_MODE,
	TNVLINK_IOCTL_GET_TX_MODE,
	TNVLINK_IOCTL_SET_RX_MODE,
	TNVLINK_IOCTL_GET_RX_MODE,
	TNVLINK_IOCTL_WRITE_DISCOVERY_TOKEN,
	TNVLINK_IOCTL_READ_DISCOVERY_TOKEN,
	TNVLINK_IOCTL_GET_LOCAL_PCI_INFO,
	TNVLINK_IOCTL_SET_TOPOLOGY_INFO,
	TNVLINK_IOCTL_INTERFACE_DISABLE,
	TNVLINK_IOCTL_FINALIZE_SHUTDOWN,
	TNVLINK_IOCTL_NUM_IOCTLS
};

/* TODO: choose a unique MAGIC number for ioctl implementation */
#define TEGRA_NVLINK_IOC_MAGIC	  'T'
#define	TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_CAPS				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_NVLINK_CAPS,		\
				struct tegra_nvlink_caps)
#define TEGRA_CTRL_CMD_NVLINK_GET_NVLINK_STATUS				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_NVLINK_STATUS,	\
				struct tegra_nvlink_status)
#define TEGRA_CTRL_CMD_NVLINK_CLEAR_COUNTERS				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_CLEAR_COUNTERS,		\
				struct tegra_nvlink_clear_counters)
#define TEGRA_CTRL_CMD_NVLINK_GET_COUNTERS				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_COUNTERS,		\
				struct tegra_nvlink_get_counters)
#define TEGRA_CTRL_CMD_NVLINK_GET_ERR_INFO				\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_ERR_INFO,		\
				struct tegra_nvlink_get_err_info)
#define TEGRA_CTRL_CMD_NVLINK_GET_ERROR_RECOVERIES			\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_GET_ERROR_RECOVERIES,		\
			struct tegra_nvlink_get_error_recoveries)
#define TEGRA_CTRL_CMD_NVLINK_SETUP_EOM					\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SETUP_EOM,		\
				struct tegra_nvlink_setup_eom)
#define TEGRA_CTRL_NVLINK_TRAIN_INTRANODE_CONN				\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_TRAIN_INTRANODE_CONN,		\
			struct tegra_nvlink_train_intranode_conn)
#define TEGRA_CTRL_CMD_NVLINK_GET_LP_COUNTERS				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_LP_COUNTERS,		\
				struct tegra_nvlink_get_lp_counters)
#define TEGRA_CTRL_CMD_NVLINK_CLEAR_LP_COUNTERS				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_CLEAR_LP_COUNTERS,	\
				struct tegra_nvlink_clear_lp_counters)
#define TEGRA_CTRL_CMD_NVLINK_ENABLE_SHIM_DRIVER			\
			_IO(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_ENABLE_SHIM_DRIVER)
#define TEGRA_CTRL_CMD_NVLINK_ENABLE_DEVICE_INTERRUPTS			\
		_IOW(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_ENABLE_DEVICE_INTERRUPTS,		\
			struct tegra_nvlink_enable_device_interrupts)
#define TEGRA_CTRL_CMD_NVLINK_SERVICE_DEVICE				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SERVICE_DEVICE,		\
				struct tegra_nvlink_service_device)
#define TEGRA_CTRL_CMD_NVLINK_DISABLE_DEVICE_INTERRUPTS			\
		_IOW(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_DISABLE_DEVICE_INTERRUPTS,	\
			struct tegra_nvlink_disable_device_interrupts)
#define TEGRA_CTRL_CMD_NVLINK_INJECT_ERR				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_INJECT_ERR,		\
				struct tegra_nvlink_inject_err)
#define TEGRA_CTRL_CMD_NVLINK_SET_LINK_MODE				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SET_LINK_MODE,		\
				struct tegra_nvlink_set_link_mode)
#define TEGRA_CTRL_CMD_NVLINK_GET_LINK_MODE				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_LINK_MODE,		\
				struct tegra_nvlink_get_link_mode)
#define TEGRA_CTRL_CMD_NVLINK_SET_TX_MODE				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SET_TX_MODE,		\
				struct tegra_nvlink_set_tx_mode)
#define TEGRA_CTRL_CMD_NVLINK_GET_TX_MODE				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_TX_MODE,		\
				struct tegra_nvlink_get_tx_mode)
#define TEGRA_CTRL_CMD_NVLINK_SET_RX_MODE				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SET_RX_MODE,		\
				struct tegra_nvlink_set_rx_mode)
#define TEGRA_CTRL_CMD_NVLINK_GET_RX_MODE				\
			_IOWR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_RX_MODE,		\
				struct tegra_nvlink_get_rx_mode)
#define TEGRA_CTRL_CMD_NVLINK_WRITE_DISCOVERY_TOKEN			\
		_IOW(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_WRITE_DISCOVERY_TOKEN,		\
			struct tegra_nvlink_write_discovery_token)
#define TEGRA_CTRL_CMD_NVLINK_READ_DISCOVERY_TOKEN			\
		_IOWR(TEGRA_NVLINK_IOC_MAGIC,				\
			TNVLINK_IOCTL_READ_DISCOVERY_TOKEN,		\
			struct tegra_nvlink_read_discovery_token)
#define TEGRA_CTRL_CMD_NVLINK_GET_LOCAL_PCI_INFO			\
			_IOR(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_GET_LOCAL_PCI_INFO,	\
				struct tegra_nvlink_get_local_pci_info)
#define TEGRA_CTRL_CMD_NVLINK_SET_TOPOLOGY_INFO				\
			_IOW(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_SET_TOPOLOGY_INFO,	\
				struct tegra_nvlink_set_topology_info)
#define TEGRA_CTRL_CMD_NVLINK_INTERFACE_DISABLE				\
			_IO(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_INTERFACE_DISABLE)
#define TEGRA_CTRL_CMD_NVLINK_FINALIZE_SHUTDOWN				\
			_IO(TEGRA_NVLINK_IOC_MAGIC,			\
				TNVLINK_IOCTL_FINALIZE_SHUTDOWN)

#endif /* TEGRA_NVLINK_UAPI_H */
