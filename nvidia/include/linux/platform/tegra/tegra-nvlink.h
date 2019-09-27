/*
 * tegra-nvlink.h:
 * This header contains the structures and APIs needed by Tegra NVLINK core and
 * endpoint drivers for interacting with each other.
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

#ifndef TEGRA_NVLINK_H
#define TEGRA_NVLINK_H

#include <linux/mutex.h>

#define NVLINK_MAX_DEVICES			2
#define NVLINK_MAX_LINKS			2
#define DEFAULT_LOOP_SLEEP_US                   100
#define DEFAULT_LOOP_TIMEOUT_US                 1000000
#define LINK_BITRATE_150MHZ_16GBPS		15500000000ULL
#define LINK_BITRATE_156MHZ_16GBPS		16145830000ULL
#define LINK_BITRATE_150MHZ_20GBPS		19200000000ULL
#define LINK_BITRATE_156MHZ_20GBPS		20001280000ULL

struct nvlink_link;
struct nvlink_device;

enum nvlink_log_categories {
	nvlink_log_err	= BIT(0),	/* Error prints - these will be printed
					   unconditionally */
	nvlink_log_dbg	= BIT(1),	/* Debug prints */
};

#ifdef CONFIG_DEBUG_FS
/* This is the root debugfs directory for the entire NVLINK driver stack */
extern struct dentry *nvlink_debugfs_root;

/* This is the parent debugfs directory for NVLINK tests */
extern struct dentry *nvlink_debugfs_tests;
#endif /* CONFIG_DEBUG_FS */

extern u32 nvlink_log_mask;
#define NVLINK_DEFAULT_LOG_MASK	nvlink_log_err

#define nvlink_print(log_mask, fmt, arg...)		\
	do {						\
		if ((log_mask) & nvlink_log_mask)	\
			printk("%s: %s: %d: " fmt "\n",	\
				NVLINK_MODULE_NAME,	\
				__func__,		\
				__LINE__,		\
				##arg);			\
	} while (0)

#define nvlink_err(fmt, arg...)	nvlink_print(nvlink_log_err, fmt, ##arg)
#define nvlink_dbg(fmt, arg...)	nvlink_print(nvlink_log_dbg, fmt, ##arg)

/* Enum to represent link speed. Nvlink 2.0 can support below 2 speeds */
enum nvlink_speed {
	NVLINK_SPEED_16,
	NVLINK_SPEED_20
};

/* Enum nvlink_endpt will be used to initialize device ID in device struct */
enum nvlink_endpt {
	NVLINK_ENDPT_T19X,
	NVLINK_ENDPT_GV100
};

/*
 * Link modes are SW defined. Some modes map to HW link state, while some
 * falicitate transiting to a power state or off state.
 */
enum link_mode {
	NVLINK_LINK_OFF,
	NVLINK_LINK_HS,
	NVLINK_LINK_SAFE,
	NVLINK_LINK_FAULT,
	NVLINK_LINK_RCVY_AC,
	NVLINK_LINK_RCVY_SW,
	NVLINK_LINK_RCVY_RX,
	NVLINK_LINK_DETECT,
	NVLINK_LINK_RESET,
	NVLINK_LINK_ENABLE_PM,
	NVLINK_LINK_DISABLE_PM,
	NVLINK_LINK_DISABLE_ERR_DETECT,
	NVLINK_LINK_LANE_DISABLE,
	NVLINK_LINK_LANE_SHUTDOWN
};

/*
 * TX_mode and RX_mode contains SW defined sublink modes. Some modes map to HW
 * sublink states while some are intermediate states needed for link training
 * and other sequences.
 */
enum tx_mode {
	NVLINK_TX_HS,
	NVLINK_TX_ENABLE_PM,
	NVLINK_TX_DISABLE_PM,
	NVLINK_TX_SINGLE_LANE,
	NVLINK_TX_SAFE,
	NVLINK_TX_OFF,
	NVLINK_TX_COMMON,
	NVLINK_TX_COMMON_DISABLE,
	NVLINK_TX_DATA_READY,
	NVLINK_TX_PRBS_EN,
};

enum rx_mode {
	NVLINK_RX_HS,
	NVLINK_RX_ENABLE_PM,
	NVLINK_RX_DISABLE_PM,
	NVLINK_RX_SINGLE_LANE,
	NVLINK_RX_SAFE,
	NVLINK_RX_OFF,
	NVLINK_RX_RXCAL,
};

/*
 * During nvlink device initialization, we use enum init_state to keep track of
 * what state we have reached. Based on the init state and the link mode, only
 * necessary steps will be executed. This allows us to call enumerate function
 * multiple times.
 *
 * NVLINK_DEV_OFF : The device is off and no part of nvlink controller hardware
 *		is out of reset and clocked.
 *
 * NVLINK_DEV_EARLY_INIT_DONE: The clocks are up, all resets deasserted, the
 * 		minion has booted and device level interrupts are initialized.
 *
 * NVLINK_LINK_EARLY_INIT_DONE: The link level initialization is done like -
 *		initialization of PHY, link interrupts and TLC buffers.
 *
 * NVLINK_DEV_INTERFACE_INIT_DONE: The memory interface is initialized.
 *
 * NVLINK_REG_INIT_DONE: The prod settings are incorporated. At this point
 * 		the link is ready to transition to safe mode and eventually to
 * 		High-Speed mode.
 */
enum init_state {
	NVLINK_DEV_OFF,
	NVLINK_DEV_EARLY_INIT_DONE,
	NVLINK_LINK_EARLY_INIT_DONE,
	NVLINK_DEV_INTERFACE_INIT_DONE,
	NVLINK_DEV_REG_INIT_DONE,
	NVLINK_INIT_STATE_INVALID
};

/*
 * These callbacks should be registered with core-driver during link
 * registration. These link_ops allow core-driver to enquire/set link and
 * sublink modes. Some help during link initializatiion.
 *
 * TODO: Pass struct nvlink_link as argument to below link_ops instead of using
 * 	struct nvlink_device. All the link level readl/writel functions need to
 * 	use link struct instead of device struct for above change.
 */
struct link_operations {
	u32 (*get_link_mode)(struct nvlink_device *ndev);
	int (*set_link_mode)(struct nvlink_device *ndev, u32 mode);
	u32 (*get_sublink_mode)(struct nvlink_device *ndev, bool is_rx_sublink);
	int (*set_sublink_mode)(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode);
	u32 (*get_link_state)(struct nvlink_device *ndev);
	void (*get_tx_sublink_state)(struct nvlink_device *ndev,
				u32 *tx_sublink_state);
	void (*get_rx_sublink_state)(struct nvlink_device *ndev,
				u32 *rx_sublink_state);
	int (*link_early_init)(struct nvlink_device *ndev);
};

/* These dev_ops expose interface between the core driver and endpoint device */
struct device_operations {
	int (*dev_early_init)(struct nvlink_device *ndev);
	int (*dev_interface_init)(struct nvlink_device *ndev);
	int (*dev_reg_init)(struct nvlink_device *ndev);
	int (*dev_interface_disable)(struct nvlink_device *ndev);
	int (*dev_shutdown)(struct nvlink_device *ndev);
};

struct nvlink_device_pci_info {
	u16 domain;
	u16 bus;
	u16 device;
	u16 function;
	u32 pci_device_id;
};

/*
 * The core-driver maintains the topology information. We use remote_device_info
 * so that the endpoint driver can pass the topology information to the core
 * driver.
 */
struct remote_device_info {
	/* Device id of  remote device connected */
	enum nvlink_endpt device_id;
	/* Link id of the remote link connected */
	u32 link_id;
	/* Information about device's PCI connection */
	struct nvlink_device_pci_info pci_info;
};

/* Structure representing the MINION ucode header */
struct minion_hdr {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	u32 *app_code_offsets;
	u32 *app_code_sizes;
	u32 *app_data_offsets;
	u32 *app_data_sizes;
	u32 ovl_offset;
	u32 ovl_size;
	u32 ucode_data_size;
};

/*
 * nvlink_link struct stores all link specific data which is relevant
 * to core-driver.
 */
struct nvlink_link {
	/*
	 * The link id is unique across the entire nvlink system. Same link_id
	 * should not be used in different device structs. This is a HACK we
	 * need while we hardcode the topology in device tree.
	 * TODO: Add an enum for link_id like we have for device_id.
	 */
	u32 link_id;
	/* ID of the device that this link belongs to */
	enum nvlink_endpt device_id;
	/* link mode TODO: Add locks to protect the link_mode changes */
	enum link_mode mode;
	/*
	 * is the link connected to an endpt. Useful for devices with multiple
	 * links. Currenly unused on Tegra.
	 * TODO: Set this before registering the link
	 */
	bool is_connected;
	/* Is single-lane mode supported for this link ? */
	bool is_sl_supported;
	/* Pointer to device info of connected end point */
	struct remote_device_info remote_dev_info;
	/*
	 * Pointer to struct containing callback functions to do link specific
	 * operation from core driver
	 */
	struct link_operations link_ops;
	/* Pointer to parent struct nvlink_device */
	struct nvlink_device *ndev;
	/* Pointer to implementations specific private data */
	void *priv;
};

/* nvlink_device struct stores all device specific data. */
struct nvlink_device {
	/* device_id */
	enum nvlink_endpt device_id;
	/* Information about device's PCI connection */
	struct nvlink_device_pci_info pci_info;
	/* init state */
	enum init_state init_state;
	/* Mutex to protect init_state access */
	struct mutex init_state_mutex;
	/*
	 * Only the master device can initiate enumeration and data transfer
	 * on nvlink. bool to check this device is master.
	 */
	bool is_master;
	/* NVlink Speed */
	enum nvlink_speed speed;
	/* The bitrate at which the link is operating */
	u64 link_bitrate;
	/* MINION FW - contains both the ucode header and image */
	const struct firmware *minion_fw;
	/* MINION ucode header */
	struct minion_hdr minion_hdr;
	/* MINION ucode image */
	const u8 *minion_img;
	/*nvlink link data. We assume there is single link per device*/
	struct nvlink_link link;
	/* Pointer to struct containing callback functions to do device specific
	* operation from core driver
	*/
	struct device_operations dev_ops;
	/* pointer to private data of this device */
	void *priv;
};

/* APIs used by endpoint drivers for interfacing with the core driver */
void nvlink_print_topology(void);
void __nvlink_dma_flush_area(const void *ptr, size_t size);
int nvlink_register_device(struct nvlink_device* device);
int nvlink_register_link(struct nvlink_link* link);
int nvlink_unregister_device(struct nvlink_device* device);
int nvlink_unregister_link(struct nvlink_link* link);
int nvlink_get_init_state(struct nvlink_device *ndev, enum init_state *state);
int nvlink_set_init_state(struct nvlink_device *ndev, enum init_state state);
int nvlink_enumerate(struct nvlink_device *ndev);
int nvlink_transition_intranode_conn_off_to_safe(struct nvlink_device *ndev);
int nvlink_train_intranode_conn_safe_to_hs(struct nvlink_device *ndev);
int nvlink_initialize_endpoint(struct nvlink_device *ndev);
int nvlink_transition_intranode_conn_hs_to_safe(struct nvlink_device *ndev);
int nvlink_shutdown(struct nvlink_device *ndev);
#endif /* TEGRA_NVLINK_H */
