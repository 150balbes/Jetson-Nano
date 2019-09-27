/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION. All rights reserved.
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
#ifndef _M_TTCAN_LINUX_H
#define  _M_TTCAN_LINUX_H

#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/net_tstamp.h>
#include <linux/spinlock.h>
#include <linux/clocksource.h>
#include <linux/platform/tegra/ptp-notifier.h>
#include <linux/mailbox_client.h>
#ifdef CONFIG_CLK_SRC_TEGRA18_US_TIMER
#include <linux/tegra-us-timer.h>
#endif

#include <asm/io.h>
#include "m_ttcan_ivc.h"

#define MTTCAN_RX_FIFO_INTR     (0xFF)
#define MTTCAN_RX_HP_INTR       (0x1 << 8)
#define MTTCAN_TX_EV_FIFO_INTR  (0xF << 12)

#define MTTCAN_ERR_INTR       (0x1FF9 << 17)
#define MTTCAN_BUS_OFF        (1 << 25)
#define MTTCAN_ERR_WARN       (1 << 24)
#define MTTCAN_ERR_PASS       (1 << 23)

#define MTT_CAN_NAPI_WEIGHT	64
#define MTT_CAN_TX_OBJ_NUM	32
#define MTT_CAN_MAX_MRAM_ELEMS	9
#define MTT_MAX_TX_CONF		4
#define MTT_MAX_RX_CONF		3

#define MTTCAN_POLL_TIME	50
#define MTTCAN_HWTS_ROLLOVER	250
/* block period in ms */
#define TX_BLOCK_PERIOD		200
#define TSC_REF_CLK_RATE	31250000

struct tegra_mttcan_soc_info {
	bool set_can_core_clk;
	unsigned long can_core_clk_rate;
	unsigned long can_clk_rate;
	bool use_external_timer;
};

struct can_gpio {
	int gpio;
	int active_low;
};

struct mttcan_priv {
	struct can_priv can;
	struct ttcan_controller *ttcan;
	const struct tegra_mttcan_soc_info *sinfo;
	struct delayed_work can_work;
	struct delayed_work drv_restart_work;
	struct napi_struct napi;
	struct net_device *dev;
	struct device *device;
	struct clk *can_clk, *host_clk, *core_clk;
	struct can_gpio gpio_can_en;
	struct can_gpio gpio_can_stb;
	struct timer_list timer;
	struct cyclecounter cc;
	struct timecounter tc;
	struct hwtstamp_config hwtstamp_config;
	struct mbox_client cl;
	struct completion xfer_completion;
	struct mbox_chan *mbox;
	raw_spinlock_t tc_lock; /* lock to protect timecounter infra */
	spinlock_t tslock; /* lock to protect ioctl */
	spinlock_t tx_lock; /* lock to protect transmit path */
	void __iomem *regs;
	void __iomem *mres;
	void *std_shadow;
	void *xtd_shadow;
	void *tmc_shadow;
	u32 gfc_reg;
	u32 xidam_reg;
	u32 irq_flags;
	u32 irq_ttflags;
	u32 irqstatus;
	u32 tt_irqstatus;
	u32 instance;
	int tt_intrs;
	int tt_param[2];
	u32 mram_param[MTT_CAN_MAX_MRAM_ELEMS];
	u32 tx_conf[MTT_MAX_TX_CONF]; /*<txb, txq, txq_mode, txb_dsize>*/
	u32 rx_conf[MTT_MAX_RX_CONF]; /*<rxb_dsize, rxq0_dsize, rxq1_dsize>*/
	bool poll;
	bool hwts_rx_en;
	u32 resp;
};

int mttcan_create_sys_files(struct device *dev);
void mttcan_delete_sys_files(struct device *dev);
#endif
