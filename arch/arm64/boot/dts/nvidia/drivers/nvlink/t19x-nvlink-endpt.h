/*
 * t19x-nvlink-endpt.h:
 * This header contains the structures and APIs needed by the Tegra NVLINK
 * endpoint driver.
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

#ifndef T19X_NVLINK_ENDPT_H
#define T19X_NVLINK_ENDPT_H

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/mc-regs-t19x.h>
#include <linux/tegra_prod.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform/tegra/tegra-nvlink.h>

#define NVLINK_MODULE_NAME			"t19x-nvlink-endpt"
#define NVLINK_IP_VERSION			2 /* NVLINK VERSION 2.0 */
#define DEFAULT_IS_NEA				0

enum nvlink_refclk {
	NVLINK_REFCLK_150,
	NVLINK_REFCLK_156
};

/* Struct used for passing around error masks in error handling functions */
struct nvlink_link_error_masks {
	u32 dl;
	u32 tl;
	u32 tl_injected;
	u32 tlc_rx0;
	u32 tlc_rx0_injected;
	u32 tlc_rx1;
	u32 tlc_rx1_injected;
	u32 tlc_tx;
	u32 tlc_tx_injected;
};

/* Fatal Errors */
enum inforom_nvlink_fatal_err {
	/* NVLink 2.0 */
	TLC_RX_DL_DATA_PARITY,
	TLC_RX_DL_CTRL_PARITY,
	TLC_RX_RAM_DATA_PARITY,
	TLC_RX_RAM_HDR_PARITY,
	TLC_RX_DATA_POISONED_PKT_RCVD,
	TLC_TX_RAM_DATA_PARITY,
	TLC_TX_RAM_HDR_PARITY,
	TLC_TX_DL_FLOW_CONTROL_PARITY,
	DL_TX_RECOVERY_LONG,
	DL_TX_FAULT_RAM,
	DL_TX_FAULT_INTERFACE,
	DL_TX_FAULT_SUBLINK_CHANGE,
	DL_RX_FAULT_SUBLINK_CHANGE,
	DL_RX_FAULT_DL_PROTOCOL,
	DL_LTSSM_FAULT,
	TLC_RX_DL_HDR_PARITY,
	TLC_RX_INVALID_AE_FLIT_RCVD,
	TLC_RX_INVALID_BE_FLIT_RCVD,
	TLC_RX_INVALID_ADDR_ALIGN,
	TLC_RX_PKT_LEN,
	TLC_RX_RSVD_CMD_ENC,
	TLC_RX_RSVD_DAT_LEN_ENC,
	TLC_RX_RSVD_ADDR_TYPE,
	TLC_RX_RSVD_RSP_STATUS,
	TLC_RX_RSVD_PKT_STATUS,
	TLC_RX_RSVD_CACHE_ATTR_ENC_IN_PROBE_REQ,
	TLC_RX_RSVD_CACHE_ATTR_ENC_IN_PROBE_RESP,
	TLC_RX_DAT_LEN_GT_ATOMIC_REQ_MAX_SIZE,
	TLC_RX_DAT_LEN_GT_RMW_REQ_MAX_SIZE,
	TLC_RX_DAT_LEN_LT_ATR_RESP_MIN_SIZE,
	TLC_RX_INVALID_PO_FOR_CACHE_ATTR,
	TLC_RX_INVALID_COMPRESSED_RESP,
	TLC_RX_RESP_STATUS_TARGET,
	TLC_RX_RESP_STATUS_UNSUPPORTED_REQUEST,
	TLC_RX_HDR_OVERFLOW,
	TLC_RX_DATA_OVERFLOW,
	TLC_RX_STOMPED_PKT_RCVD,
	TLC_RX_CORRECTABLE_INTERNAL,
	TLC_RX_UNSUPPORTED_VC_OVERFLOW,
	TLC_RX_UNSUPPORTED_NVLINK_CREDIT_RELEASE,
	TLC_RX_UNSUPPORTED_NCISOC_CREDIT_RELEASE,
	TLC_TX_HDR_CREDIT_OVERFLOW,
	TLC_TX_DATA_CREDIT_OVERFLOW,
	TLC_TX_DL_REPLAY_CREDIT_OVERFLOW,
	TLC_TX_UNSUPPORTED_VC_OVERFLOW,
	TLC_TX_STOMPED_PKT_SENT,
	TLC_TX_DATA_POISONED_PKT_SENT,
	TLC_TX_RESP_STATUS_TARGET,
	TLC_TX_RESP_STATUS_UNSUPPORTED_REQUEST,
};

/*
 * This structure is used for storing parameters which describe the Single-Lane
 * (SL / 1/8th) mode policy. A few acronyms that are used in this structure are
 * as follows:
 *    - SL = Single-Lane / 1/8th mode - sublink low power mode where only 1 of
 *           the 8 lanes is used
 *    - FB = Full Bandwidth (i.e. HISPEED mode)
 *    - LP = Low Power (i.e. SL / 1/8th mode)
 *    - IC = Idle Counter - the idle counter is used to monitor traffic per
 *           sub-link
 */
struct single_lane_params {
	/* Idle counter increment in FB */
	u16 fb_ic_inc;

	/* Idle counter increment in LP */
	u16 lp_ic_inc;

	/* Idle counter decrement in FB */
	u16 fb_ic_dec;

	/* Idle counter decrement in LP */
	u16 lp_ic_dec;

	/* SL entry threshold */
	u32 enter_thresh;

	/* SL exit threshold */
	u32 exit_thresh;

	/* Idle counter saturation limit */
	u32 ic_limit;
};

/* Tegra endpoint driver's private link struct */
struct tnvlink_link {
	/* base address of DLPL */
	void __iomem *nvlw_nvl_base;
	/* base address of TL */
	void __iomem *nvlw_nvltlc_base;
	/* TLC errors status. TODO: Add more description here */
	u32 tlc_tx_err_status0;
	u32 tlc_rx_err_status0;
	u32 tlc_rx_err_status1;
	/* Successful error recoveries */
	u32 error_recoveries;
	/* Parameters which describe the selected Single-Lane policy */
	struct single_lane_params sl_params;
	/* Pointer to parent struct tnvlink_dev */
	struct tnvlink_dev *tdev;
	/* Pointer to parent struct nvlink_link */
	struct nvlink_link *nlink;
};

/* Tegra endpoint driver's private device struct */
struct tnvlink_dev {
	/* Are we using the RM shim driver? */
	bool rm_shim_enabled;
	/* base address of minion */
	void __iomem *nvlw_minion_base;
	/* base address of IOCTRL */
	void __iomem *nvlw_tioctrl_base;
	/* base address of NVLIPT */
	void __iomem *nvlw_nvlipt_base;
	/* base address of SYNC2X */
	void __iomem *nvlw_sync2x_base;
	/* base address of MSSNVLINK */
	void __iomem *mssnvlink_0_base;
	/* irq below represents the interrupt line going to GIC and LIC */
	int irq;
	struct class class;
	dev_t dev_t;
	struct cdev cdev;
	struct device *dev;
#ifdef CONFIG_DEBUG_FS
	/* This is the debugfs directory for the Tegra endpoint driver */
	struct dentry *tegra_debugfs;
	struct dentry *tegra_debugfs_file;
#endif /* CONFIG_DEBUG_FS  */
	/* clocks */
	struct clk *clk_nvhs_pll0_mgmt;
	struct clk *clk_pllrefe_vcoout_gated;
	struct clk *clk_nvlink_sys;
	struct clk *clk_pllnvhs;
	struct clk *clk_m;
	struct clk *clk_nvlink_pll_txclk;
	struct clk *clk_nvlink_tx;
	/* resets */
	struct reset_control *rst_mssnvl;
	struct reset_control *rst_nvhs_uphy_pm;
	struct reset_control *rst_nvhs_uphy;
	struct reset_control *rst_nvhs_uphy_pll0;
	struct reset_control *rst_nvhs_uphy_l0;
	struct reset_control *rst_nvhs_uphy_l1;
	struct reset_control *rst_nvhs_uphy_l2;
	struct reset_control *rst_nvhs_uphy_l3;
	struct reset_control *rst_nvhs_uphy_l4;
	struct reset_control *rst_nvhs_uphy_l5;
	struct reset_control *rst_nvhs_uphy_l6;
	struct reset_control *rst_nvhs_uphy_l7;
	struct reset_control *rst_nvlink;
	struct tegra_prod *prod_list;
	bool is_nea;
	/* Nvlink refclk*/
	enum nvlink_refclk refclk;
	bool is_tp_cntr_running;
	struct tnvlink_link tlink;
	struct nvlink_device *ndev;
};

extern const struct single_lane_params entry_100us_sl_params;
extern const struct file_operations t19x_nvlink_endpt_ops;

u32 nvlw_tioctrl_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_tioctrl_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

u32 nvlw_nvlipt_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_nvlipt_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

u32 nvlw_minion_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_minion_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

u32 nvlw_nvl_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_nvl_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

u32 nvlw_sync2x_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_sync2x_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

u32 nvlw_nvltlc_readl(struct tnvlink_dev *tdev, u32 reg);
void nvlw_nvltlc_writel(struct tnvlink_dev *tdev, u32 reg, u32 val);

int t19x_nvlink_dev_car_disable(struct nvlink_device *ndev);
int t19x_nvlink_suspend(struct device *dev);

int wait_for_reg_cond_nvlink(
			struct tnvlink_dev *tdev,
			u32 reg,
			u32 bit,
			bool check_for_bit_set,
			char *bit_name,
			u32 (*reg_readl)(struct tnvlink_dev *, u32),
			u32 *reg_val,
			u32 timeout_us);

int t19x_nvlink_dev_interface_disable(struct nvlink_device *ndev);

void minion_dump_pc_trace(struct tnvlink_dev *tdev);
void minion_dump_registers(struct tnvlink_dev *tdev);
int minion_boot(struct tnvlink_dev *tdev);
int init_nvhs_phy(struct tnvlink_dev *tdev);
int minion_send_cmd(struct tnvlink_dev *tdev,
				u32 cmd,
				u32 scratch0_val);
void nvlink_enable_AN0_packets(struct tnvlink_dev *tdev);

void nvlink_config_minion_falcon_intr(struct tnvlink_dev *tdev);
void nvlink_config_common_intr(struct tnvlink_dev *tdev);
void nvlink_enable_dl_interrupts(struct tnvlink_dev *tdev);
void nvlink_enable_link_interrupts(struct tnvlink_dev *tdev);
void nvlink_disable_link_interrupts(struct tnvlink_dev *tdev);
void minion_service_falcon_intr(struct tnvlink_dev *tdev);
void nvlink_disable_dl_interrupts(struct tnvlink_dev *tdev);
int nvlink_service_dl_interrupts(struct tnvlink_dev *tdev,
				bool *retrain_from_safe);
irqreturn_t t19x_nvlink_endpt_isr(int irq, void *dev_id);

void init_single_lane_params(struct tnvlink_dev *tdev);
u32 t19x_nvlink_get_link_state(struct nvlink_device *ndev);
u32 t19x_nvlink_get_link_mode(struct nvlink_device *ndev);
int t19x_nvlink_set_link_mode(struct nvlink_device *ndev, u32 mode);
void t19x_nvlink_get_tx_sublink_state(struct nvlink_device *ndev,
				u32 *tx_sublink_state);
void t19x_nvlink_get_rx_sublink_state(struct nvlink_device *ndev,
				u32 *rx_sublink_state);
u32 t19x_nvlink_get_sublink_mode(struct nvlink_device *ndev,
				bool is_rx_sublink);
int t19x_nvlink_set_sublink_mode(struct nvlink_device *ndev, bool is_rx_sublink,
				u32 mode);
bool is_link_connected(struct tnvlink_link *tlink);
int nvlink_retrain_link(struct tnvlink_dev *tdev, bool from_off);
int t19x_nvlink_write_discovery_token(struct tnvlink_dev *tdev, u64 token);
int t19x_nvlink_read_discovery_token(struct tnvlink_dev *tdev, u64 *token);
int t19x_nvlink_reset_tp_counters(struct tnvlink_dev *tdev);
int t19x_nvlink_freeze_tp_counters(struct tnvlink_dev *tdev, bool bFreeze);
int t19x_nvlink_config_tp_counters(struct tnvlink_dev *tdev);
int t19x_nvlink_get_tp_counters(struct tnvlink_dev *tdev, u64 *tx0cnt,
					u64 *tx1cnt, u64 *rx0cnt, u64 *rx1cnt);
#ifdef CONFIG_DEBUG_FS
void t19x_nvlink_endpt_debugfs_init(struct tnvlink_dev *tdev);
void t19x_nvlink_endpt_debugfs_deinit(struct tnvlink_dev *tdev);
#else
static inline void t19x_nvlink_endpt_debugfs_init(struct tnvlink_dev *tdev) {}
static inline void t19x_nvlink_endpt_debugfs_deinit(
						struct tnvlink_dev *tdev) {}
#endif /* CONFIG_DEBUG_FS  */

#endif /* T19X_NVLINK_ENDPT_H */
