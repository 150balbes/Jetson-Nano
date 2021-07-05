/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/platform_device.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/phy/phy.h>
#include <linux/resource.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <linux/pci.h>
#include <linux/kfifo.h>

#define CTRL_0	(0)
#define CTRL_1	(1)
#define CTRL_2	(2)
#define CTRL_3	(3)
#define CTRL_4	(4)
#define CTRL_5	(5)

#define APPL_PINMUX				(0X0)
#define APPL_PINMUX_CLK_OUTPUT_IN_OVERRIDE_EN	BIT(4)
#define APPL_PINMUX_CLK_OUTPUT_IN_OVERRIDE	BIT(5)
#define APPL_PINMUX_PEX_RST_IN_OVERRIDE_EN	BIT(11)

#define APPL_CTRL				(0X4)
#define APPL_SYS_PRE_DET_STATE			BIT(6)
#define APPL_CTRL_LTSSM_EN			BIT(7)
#define APPL_CTRL_READY_ENTR_L23			BIT(12)
#define APPL_CTRL_HW_HOT_RST_EN			BIT(20)

#define APPL_INTR_EN_L0_0			0x8
#define APPL_INTR_EN_L0_0_SYS_INTR_EN		BIT(30)
#define APPL_INTR_EN_L0_0_PEX_RST_INT_EN		BIT(16)
#define APPL_INTR_EN_L0_0_PCI_CMD_EN_INT_EN	BIT(15)
#define APPL_INTR_EN_L0_0_ERROR_INT_EN		BIT(1)
#define APPL_INTR_EN_L0_0_LINK_STATE_INT_EN	BIT(0)

#define APPL_INTR_STATUS_L0			0xC
#define APPL_INTR_STATUS_L0_PEX_RST_INT_SHIFT	16
#define APPL_INTR_STATUS_L0_PEX_RST_INT		BIT(16)
#define APPL_INTR_STATUS_L0_PCI_CMD_EN_INT	BIT(15)
#define APPL_INTR_STATUS_L0_LINK_STATE_INT	BIT(0)

#define APPL_INTR_EN_L1_0			0x1C
#define APPL_INTR_EN_L1_0_LINK_REQ_RST_INT_EN	BIT(1)
#define APPL_INTR_EN_L1_0_HOT_RESET_DONE_INT_EN	BIT(30)

#define APPL_INTR_STATUS_L1			0x20
#define APPL_INTR_STATUS_L1_LINK_REQ_RST_CHGED	BIT(1)
#define APPL_INTR_STATUS_L1_HOT_RESET_DONE	BIT(30)

#define APPL_INTR_STATUS_L1_1			0x2C
#define APPL_INTR_STATUS_L1_2			0x30
#define APPL_INTR_STATUS_L1_3			0x34
#define APPL_INTR_STATUS_L1_6			0x3C
#define APPL_INTR_STATUS_L1_7			0x40
#define APPL_INTR_STATUS_L1_8			0x4C
#define APPL_INTR_STATUS_L1_9			0x54
#define APPL_INTR_STATUS_L1_10			0x58
#define APPL_INTR_STATUS_L1_11			0x64
#define APPL_INTR_STATUS_L1_13			0x74
#define APPL_INTR_STATUS_L1_14			0x78
#define APPL_INTR_STATUS_L1_15			0x7C
#define APPL_INTR_STATUS_L1_15_CFG_BME_CHGED	BIT(1)
#define APPL_INTR_STATUS_L1_17			0x88

#define APPL_MSI_CTRL_2				0xB0

#define APPL_LTR_MSG_1				0xC4
#define APPL_LTR_MSG_2				0xC8
#define APPL_LTR_MSG_2_LTR_MSG_REQ_STATE	BIT(3)

#define LTR_MSG_REQ				BIT(15)
#define LTR_MST_NO_SNOOP_SHIFT			16

#define APPL_DEBUG				0xd0
#define APPL_DEBUG_LTSSM_STATE_MASK		0x1f8
#define APPL_DEBUG_LTSSM_STATE_SHIFT		3
#define LTSSM_STATE_PRE_DETECT			0x5

#define APPL_DM_TYPE				0x100
#define APPL_DM_TYPE_MASK			0xF
#define APPL_DM_TYPE_EP				0x0

#define APPL_CFG_BASE_ADDR			0x104
#define APPL_CFG_BASE_ADDR_MASK			0xFFFFF000

#define APPL_CFG_IATU_DMA_BASE_ADDR		0x108
#define APPL_CFG_IATU_DMA_BASE_ADDR_MASK	0xFFFC0000

#define APPL_CFG_MISC				0x110
#define APPL_CFG_MISC_SLV_EP_MODE		BIT(14)
#define APPL_CFG_MISC_ARCACHE_MASK		0x3C00
#define APPL_CFG_MISC_ARCACHE_SHIFT		10
#define APPL_CFG_MISC_ARCACHE_VAL		3

#define APPL_CFG_SLCG_OVERRIDE			0x114
#define APPL_CFG_SLCG_OVERRIDE_SLCG_EN_MASTER	BIT(0)

#define APPL_GTH_PHY				0x138
#define APPL_GTH_PHY_RST			0x1

#define EP_CS_STATUS_COMMAND			0x4
#define EP_CS_STATUS_COMMAND_BME		BIT(2)

#define EP_CFG_LINK_CAP				0x7C
#define EP_CFG_LINK_CAP_MAX_SPEED_MASK		0xF

#define CFG_LINK_STATUS_CONTROL	0x80

#define CAP_SPCIE_CAP_OFF	0x154
#define CAP_SPCIE_CAP_OFF_DSP_TX_PRESET0_MASK	GENMASK(3, 0)

#define PL16G_CAP_OFF		0x188
#define PL16G_CAP_OFF_DSP_16G_TX_PRESET_MASK	GENMASK(3, 0)

#define MARGIN_PORT_CAP_STATUS_REG_MARGINING_READY	BIT(16)
#define MARGIN_PORT_CAP_STATUS_REG_MARGINING_SW_READY	BIT(17)

#define MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_MASK		GENMASK(2, 0)
#define MARGIN_LANE_CNTRL_STATUS_TYPE_MASK			GENMASK(5, 3)
#define MARGIN_LANE_CNTRL_STATUS_TYPE_SHIFT			3
#define MARGIN_LANE_CNTRL_STATUS_PAYLOAD_MASK			GENMASK(15, 8)
#define MARGIN_LANE_CNTRL_STATUS_PAYLOAD_SHIFT			8
#define MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_STATUS_MASK		GENMASK(18, 16)
#define MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_STATUS_SHIFT	16
#define MARGIN_LANE_CNTRL_STATUS_TYPE_STATUS_MASK		GENMASK(21, 19)
#define MARGIN_LANE_CNTRL_STATUS_TYPE_STATUS_SHIFT		19
#define MARGIN_LANE_CNTRL_STATUS_PAYLOAD_STATUS_MASK		GENMASK(31, 24)
#define MARGIN_LANE_CNTRL_STATUS_PAYLOAD_STATUS_SHIFT		24

#define EVENT_COUNTER_CONTROL_REG	0x1d8
#define EVENT_COUNTER_ALL_CLEAR		0x3
#define EVENT_COUNTER_ENABLE_ALL	0x7
#define EVENT_COUNTER_ENABLE_SHIFT	2
#define EVENT_COUNTER_EVENT_SEL_MASK	0xFF
#define EVENT_COUNTER_EVENT_SEL_SHIFT	16
#define EVENT_COUNTER_EVENT_Tx_L0S	0x2
#define EVENT_COUNTER_EVENT_Rx_L0S	0x3
#define EVENT_COUNTER_EVENT_L1		0x5
#define EVENT_COUNTER_EVENT_L1_1	0x7
#define EVENT_COUNTER_EVENT_L1_2	0x8
#define EVENT_COUNTER_GROUP_SEL_SHIFT	24
#define EVENT_COUNTER_GROUP_5		0x5

#define EVENT_COUNTER_DATA_REG		0x1dC

#define PORT_LOGIC_ACK_F_ASPM_CTRL	0x70C
#define ENTER_ASPM			BIT(30)
#define L0S_ENTRANCE_LAT_SHIFT		24
#define L0S_ENTRANCE_LAT_MASK		0x07000000
#define L1_ENTRANCE_LAT_SHIFT		27
#define L1_ENTRANCE_LAT_MASK		0x38000000
#define N_FTS_SHIFT			8
#define N_FTS_MASK			0xFF
#define N_FTS_VAL			52

#define CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF		0x718
#define CFG_TIMER_CTRL_ACK_NAK_SHIFT		(19)

#define CFG_LINK_CAP_L1SUB			0x1C4

#define PORT_LOGIC_GEN2_CTRL		0x80C
#define FTS_MASK			0xFF
#define FTS_VAL				52

#define GEN3_RELATED_OFF	0x890
#define GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL	BIT(0)
#define GEN3_RELATED_OFF_GEN3_EQ_DISABLE	BIT(16)
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT	24
#define GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK	GENMASK(25, 24)

#define GEN3_EQ_CONTROL_OFF	0x8a8
#define GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT	8
#define GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK	GENMASK(23, 8)
#define GEN3_EQ_CONTROL_OFF_FB_MODE_MASK	GENMASK(3, 0)

#define MISC_CONTROL_1				0X8BC
#define MISC_CONTROL_1_DBI_RO_WR_EN		BIT(0)

#define AUX_CLK_FREQ				0xB40

#define GEN4_LANE_MARGINING_1	0xb80
#define GEN4_LANE_MARGINING_1_NUM_TIMING_STEPS_MASK	GENMASK(5, 0)
#define GEN4_LANE_MARGINING_1_MAX_VOLTAGE_OFFSET_MASK	GENMASK(29, 24)
#define GEN4_LANE_MARGINING_1_MAX_VOLTAGE_OFFSET_SHIFT	24

#define GEN4_LANE_MARGINING_2	0xb84
#define GEN4_LANE_MARGINING_2_VOLTAGE_SUPPORTED		BIT(24)

#define PCIE_ATU_REGION_INDEX0	0 /* used for BAR-0 translations */
#define PCIE_ATU_REGION_INDEX1	1
#define PCIE_ATU_REGION_INDEX2	2
#define PCIE_ATU_REGION_INDEX3	3

#define PCIE_ATU_CR1		0x0
#define PCIE_ATU_TYPE_MEM	(0x0 << 0)
#define PCIE_ATU_TYPE_IO	(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0	(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1	(0x5 << 0)
#define PCIE_ATU_CR2		0x4
#define PCIE_ATU_ENABLE		(0x1 << 31)
#define PCIE_ATU_CR2_BAR_SHIFT	8
#define PCIE_ATU_CR2_MATCH_MODE_SHIFT	30
#define PCIE_ATU_CR2_MATCH_MODE_ADDR	0
#define PCIE_ATU_CR2_MATCH_MODE_BAR	1
#define PCIE_ATU_LOWER_BASE	0x8
#define PCIE_ATU_UPPER_BASE	0xC
#define PCIE_ATU_LIMIT		0x10
#define PCIE_ATU_LOWER_TARGET	0x14
#define PCIE_ATU_UPPER_TARGET	0x18

#define TSA_CONFIG_STATIC0_CSW_PCIE5W_0_SO_DEV_HUBID_SHIFT (15)
#define TSA_CONFIG_STATIC0_CSW_PCIE5W_0_SO_DEV_HUBID_HUB2 (2)

#define LTR_MSG_TIMEOUT (100 * 1000)
#define LTSSM_TIMEOUT 50
#define PERST_DEBOUNCE_TIME (5 * 1000)

#define EVENT_QUEUE_LEN	(256)

#define NUM_TIMING_STEPS 0x14
#define NUM_VOLTAGE_STEPS 0x14

/* Max error count limit is 0x3f, payload=(0xc0 | 0x3f) */
#define MAX_ERR_CNT_PAYLOAD 0xff
#define NORMAL_PAYLOAD 0x0f
#define CLR_ERR_PAYLOAD 0x55
/* payload[6] = 1 Left step margin
 * payload[6] = 0 Right step margin
 * payload[7] = 1 down step margin
 * payload[7] = 0 up step margin
 */
#define LEFT_STEP_PAYLOAD (0x1 << 6)
#define RIGHT_STEP_PAYLOAD (0x0 << 6)
#define DOWN_STEP_PAYLOAD (0x1 << 7)
#define UP_STEP_PAYLOAD (0x0 << 7)

#define LEFT_STEP 'L'
#define RIGHT_STEP 'R'
#define NO_STEP 'N'
#define DOWN_STEP 'D'
#define UP_STEP 'U'

/* Receiver number*/
#define EP_RCV_NO 6

/* Time in msec */
#define MARGIN_WIN_TIME 1000
#define MARGIN_READ_DELAY 100

enum ep_event {
	EP_EVENT_NONE = 0,
	EP_PEX_RST_DEASSERT,
	EP_PEX_RST_ASSERT,
	EP_PEX_HOT_RST_DONE,
	EP_PEX_BME_CHANGE,
};

enum margin_cmds {
	MARGIN_SET_ERR_COUNT,
	MARGIN_SET_NO_CMD,
	MARGIN_SET_X_OFFSET,
	MARGIN_SET_Y_OFFSET,
	MARGIN_SET_NORMAL,
	MARGIN_CLR_ERR,
};

struct margin_cmd {
	int margin_type;
	int rcv_no;
	int payload;
	int rxm_payload_check;
	int rxm_cmd_check;
};
struct tegra_pcie_dw_ep {
	struct device *dev;
	struct resource *appl_res;
	struct resource	*dbi_res;
	struct resource	*atu_dma_res;
	void __iomem *appl_base;
	void __iomem *dbi_base;
	void __iomem *atu_dma_base;
	struct clk *core_clk;
	struct reset_control *core_apb_rst;
	struct reset_control *core_rst;
	int irq;
	int phy_count;
	int pex_rst_gpio;
	int ep_state;
	struct phy **phy;
	struct task_struct *pcie_ep_task;
	struct mutex disable_lock;
	wait_queue_head_t wq;
	DECLARE_KFIFO(event_fifo, u32, EVENT_QUEUE_LEN);
	u32 bar0_size;
	u32 cid;
	u16 device_id;
	u32 disabled_aspm_states;
	u8 init_link_width;
	dma_addr_t dma_handle;
	void *cpu_virt;
	bool update_fc_fixup;
	enum ep_event event;
	struct regulator *pex_ctl_reg;
	struct margin_cmd mcmd;
	struct dentry *debugfs;

	struct tegra_bwmgr_client *emc_bw;
	u32 dvfs_tbl[4][4]; /* for x1/x2/x3/x4 and Gen-1/2/3/4 */

	u32 num_lanes;
	u32 max_speed;
	u32 cfg_link_cap_l1sub;
	u32 event_cntr_ctrl;
	u32 event_cntr_data;
	u32 margin_port_cap;
	u32 margin_lane_cntrl;
};

#define EP_STATE_DISABLED	0
#define EP_STATE_ENABLED	1

static unsigned int pcie_emc_client_id[] = {
	TEGRA_BWMGR_CLIENT_PCIE,
	TEGRA_BWMGR_CLIENT_PCIE_1,
	TEGRA_BWMGR_CLIENT_PCIE_2,
	TEGRA_BWMGR_CLIENT_PCIE_3,
	TEGRA_BWMGR_CLIENT_PCIE_4,
	TEGRA_BWMGR_CLIENT_PCIE_5
};

#define GEN1_CORE_CLK_FREQ	62500000
#define GEN2_CORE_CLK_FREQ	125000000
#define GEN3_CORE_CLK_FREQ	250000000
#define GEN4_CORE_CLK_FREQ	500000000

static unsigned int pcie_gen_freq[] = {
	GEN1_CORE_CLK_FREQ,
	GEN2_CORE_CLK_FREQ,
	GEN3_CORE_CLK_FREQ,
	GEN4_CORE_CLK_FREQ
};

static int tegra_pcie_power_on_phy(struct tegra_pcie_dw_ep *pcie);

static inline void prog_atu(struct tegra_pcie_dw_ep *pcie, int i, u32 val,
			    u32 reg)
{
	writel(val, pcie->atu_dma_base + (i * 0x200) + 0x100 + reg);
}

static void inbound_atu(struct tegra_pcie_dw_ep *pcie, int i, int type,
			u64 wire_addr, u64 int_addr, u32 size,
			bool match_mode, u8 bar)
{
	prog_atu(pcie, i, lower_32_bits(wire_addr), PCIE_ATU_LOWER_BASE);
	prog_atu(pcie, i, upper_32_bits(wire_addr), PCIE_ATU_UPPER_BASE);
	prog_atu(pcie, i, lower_32_bits(wire_addr + size - 1), PCIE_ATU_LIMIT);
	prog_atu(pcie, i, lower_32_bits(int_addr), PCIE_ATU_LOWER_TARGET);
	prog_atu(pcie, i, upper_32_bits(int_addr), PCIE_ATU_UPPER_TARGET);
	prog_atu(pcie, i, type, PCIE_ATU_CR1);
	prog_atu(pcie, i, PCIE_ATU_ENABLE | (bar << PCIE_ATU_CR2_BAR_SHIFT) |
		 (match_mode <<  PCIE_ATU_CR2_MATCH_MODE_SHIFT), PCIE_ATU_CR2);
}

static irqreturn_t tegra_pcie_irq_handler(int irq, void *arg)
{
	struct tegra_pcie_dw_ep *pcie = (struct tegra_pcie_dw_ep *)arg;
	u32 val = 0;

	val = readl(pcie->appl_base + APPL_INTR_STATUS_L0);
	dev_dbg(pcie->dev, "APPL_INTR_STATUS_L0 = 0x%08X\n", val);
	if (val & APPL_INTR_STATUS_L0_PEX_RST_INT) {
		/* clear any stale PEX_RST interrupt */
		writel(APPL_INTR_STATUS_L0_PEX_RST_INT,
		       pcie->appl_base + APPL_INTR_STATUS_L0);
		if (!kfifo_put(&pcie->event_fifo, EP_PEX_RST_DEASSERT)) {
			dev_err(pcie->dev, "EVENT: fifo is full\n");
			return IRQ_HANDLED;
		}
		wake_up(&pcie->wq);
	} else if (val & APPL_INTR_STATUS_L0_LINK_STATE_INT) {
		val = readl(pcie->appl_base + APPL_INTR_STATUS_L1);
		writel(val, pcie->appl_base + APPL_INTR_STATUS_L1);
		dev_dbg(pcie->dev, "APPL_INTR_STATUS_L1 = 0x%08X\n", val);
		if (val & APPL_INTR_STATUS_L1_HOT_RESET_DONE) {
			/* clear any stale PEX_RST interrupt */
			if (!kfifo_put(&pcie->event_fifo,
				       EP_PEX_HOT_RST_DONE)) {
				dev_err(pcie->dev, "EVENT: fifo is full\n");
				return IRQ_HANDLED;
			}
			wake_up(&pcie->wq);
		}
	} else if (val & APPL_INTR_STATUS_L0_PCI_CMD_EN_INT) {
		val = readl(pcie->appl_base + APPL_INTR_STATUS_L1_15);
		writel(val, pcie->appl_base + APPL_INTR_STATUS_L1_15);
		dev_dbg(pcie->dev, "APPL_INTR_STATUS_L1_15 = 0x%08X\n", val);
		if (val & APPL_INTR_STATUS_L1_15_CFG_BME_CHGED) {
			if (!kfifo_put(&pcie->event_fifo, EP_PEX_BME_CHANGE)) {
				dev_err(pcie->dev, "EVENT: fifo is full\n");
				return IRQ_HANDLED;
			}
			wake_up(&pcie->wq);
		}
	} else {
		dev_info(pcie->dev, "Random interrupt (STATUS = 0x%08X)\n",
			 val);
		writel(val, pcie->appl_base + APPL_INTR_STATUS_L0);
	}

	return IRQ_HANDLED;
}

static int bpmp_send_uphy_message_atomic(struct mrq_uphy_request *req, int size,
					 struct mrq_uphy_response *reply,
					 int reply_size)
{
	unsigned long flags;
	int err;

	local_irq_save(flags);
	err = tegra_bpmp_send_receive_atomic(MRQ_UPHY, req, size, reply,
					     reply_size);
	local_irq_restore(flags);

	return err;
}

static int bpmp_send_uphy_message(struct mrq_uphy_request *req, int size,
				  struct mrq_uphy_response *reply,
				  int reply_size)
{
	int err;

	err = tegra_bpmp_send_receive(MRQ_UPHY, req, size, reply, reply_size);
	if (err != -EAGAIN)
		return err;

	/*
	 * in case the mail systems worker threads haven't been started yet,
	 * use the atomic send/receive interface. This happens because the
	 * clocks are initialized before the IPC mechanism.
	 */
	return bpmp_send_uphy_message_atomic(req, size, reply, reply_size);
}

static int uphy_bpmp_pcie_ep_controller_pll_init(u32 id)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;

	req.cmd = CMD_UPHY_PCIE_EP_CONTROLLER_PLL_INIT;
	req.ep_ctrlr_pll_init.ep_controller = id;

	return bpmp_send_uphy_message(&req, sizeof(req), &resp, sizeof(resp));
}

static int uphy_bpmp_pcie_ep_controller_pll_off(u32 id)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;

	req.cmd = CMD_UPHY_PCIE_EP_CONTROLLER_PLL_OFF;
	req.ep_ctrlr_pll_off.ep_controller = id;

	return bpmp_send_uphy_message(&req, sizeof(req), &resp, sizeof(resp));
}

static int uphy_bpmp_pcie_controller_state_set(int controller, int enable)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;

	req.cmd = CMD_UPHY_PCIE_CONTROLLER_STATE;
	req.controller_state.pcie_controller = controller;
	req.controller_state.enable = enable;

	return bpmp_send_uphy_message(&req, sizeof(req), &resp, sizeof(resp));
}

static void disable_aspm_l0s(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + EP_CFG_LINK_CAP);
	val &= ~(PCI_EXP_LNKCTL_ASPM_L0S << 10);
	writel(val, pcie->dbi_base + EP_CFG_LINK_CAP);
}

static void disable_aspm_l10(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + EP_CFG_LINK_CAP);
	val &= ~(PCI_EXP_LNKCTL_ASPM_L1 << 10);
	writel(val, pcie->dbi_base + EP_CFG_LINK_CAP);
}

static void disable_aspm_l11(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + pcie->cfg_link_cap_l1sub);
	val &= ~PCI_L1SS_CAP_ASPM_L11S;
	writel(val, pcie->dbi_base + pcie->cfg_link_cap_l1sub);
}

static void disable_aspm_l12(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + pcie->cfg_link_cap_l1sub);
	val &= ~PCI_L1SS_CAP_ASPM_L12S;
	writel(val, pcie->dbi_base + pcie->cfg_link_cap_l1sub);
}

static void program_gen3_gen4_eq_presets(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + GEN3_RELATED_OFF);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	writel(val, pcie->dbi_base + GEN3_RELATED_OFF);

	val = readl(pcie->dbi_base + GEN3_EQ_CONTROL_OFF);
	val &= ~GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK;
	val |= (0x3ff << GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT);
	val &= ~GEN3_EQ_CONTROL_OFF_FB_MODE_MASK;
	writel(val, pcie->dbi_base + GEN3_EQ_CONTROL_OFF);

	val = readl(pcie->dbi_base + GEN3_RELATED_OFF);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	val |= (0x1 << GEN3_RELATED_OFF_RATE_SHADOW_SEL_SHIFT);
	writel(val, pcie->dbi_base + GEN3_RELATED_OFF);

	val = readl(pcie->dbi_base + GEN3_EQ_CONTROL_OFF);
	val &= ~GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_MASK;
	val |= (0x360 << GEN3_EQ_CONTROL_OFF_PSET_REQ_VEC_SHIFT);
	val &= ~GEN3_EQ_CONTROL_OFF_FB_MODE_MASK;
	writel(val, pcie->dbi_base + GEN3_EQ_CONTROL_OFF);

	val = readl(pcie->dbi_base + GEN3_RELATED_OFF);
	val &= ~GEN3_RELATED_OFF_RATE_SHADOW_SEL_MASK;
	writel(val, pcie->dbi_base + GEN3_RELATED_OFF);
}

static void pex_ep_event_pex_rst_assert(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;
	int ret = 0, count = 0;

	mutex_lock(&pcie->disable_lock);
	if (pcie->ep_state == EP_STATE_DISABLED) {
		mutex_unlock(&pcie->disable_lock);
		return;
	}

	/* disable LTSSM */
	val = readl(pcie->appl_base + APPL_CTRL);
	val &= ~APPL_CTRL_LTSSM_EN;
	writel(val, pcie->appl_base + APPL_CTRL);

	ret = readl_poll_timeout(pcie->appl_base + APPL_DEBUG, val,
				 ((val & APPL_DEBUG_LTSSM_STATE_MASK) >>
				 APPL_DEBUG_LTSSM_STATE_SHIFT) ==
				 LTSSM_STATE_PRE_DETECT,
				 1, LTSSM_TIMEOUT);
	if (ret)
		dev_info(pcie->dev, "Link didn't go to detect state\n");

	reset_control_assert(pcie->core_rst);

	for (count = 0; count < pcie->phy_count; count++)
		phy_power_off(pcie->phy[count]);

	reset_control_assert(pcie->core_apb_rst);
	clk_disable_unprepare(pcie->core_clk);

	/*
	 * If PCIe partition is ungated it will request PLL power ON,
	 * so PLL sequencer will be in SEQ_ON state. To turn off the
	 * PLL sequencer, power gate PCIe partition.
	 */
	ret = pm_runtime_put_sync(pcie->dev);
	if (ret < 0)
		dev_err(pcie->dev, "runtime suspend failed: %d\n", ret);

	if (!(pcie->cid == CTRL_4 && pcie->num_lanes == 1)) {
		/* Resets PLL CAL_VALID and RCAL_VALID */
		ret = uphy_bpmp_pcie_ep_controller_pll_off(pcie->cid);
		if (ret)
			dev_err(pcie->dev, "UPHY off failed for PCIe EP:%d\n",
				ret);
	}

	pcie->ep_state = EP_STATE_DISABLED;
	mutex_unlock(&pcie->disable_lock);
	dev_info(pcie->dev, "EP deinit done\n");
}

static void pex_ep_event_pex_rst_deassert(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;
	int ret = 0;

	if (pcie->ep_state == EP_STATE_ENABLED)
		return;

	ret = pm_runtime_get_sync(pcie->dev);
	if (ret < 0) {
		dev_err(pcie->dev, "runtime resume failed: %d\n", ret);
		return;
	}

	if (!(pcie->cid == CTRL_4 && pcie->num_lanes == 1)) {
		ret = uphy_bpmp_pcie_ep_controller_pll_init(pcie->cid);
		if (ret) {
			dev_err(pcie->dev, "UPHY init failed for PCIe EP:%d\n",
				ret);
			goto pll_fail;
		}
	}

	ret = clk_prepare_enable(pcie->core_clk);
	if (ret) {
		dev_err(pcie->dev, "Failed to enable core clock\n");
		goto pll_fail;
	}

	reset_control_deassert(pcie->core_apb_rst);

	ret = tegra_pcie_power_on_phy(pcie);
	if (ret) {
		dev_err(pcie->dev, "failed to power_on phy\n");
		goto phy_fail;
	}

	/* clear any stale interrupt statuses */
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L0);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_1);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_2);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_3);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_6);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_7);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_8);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_9);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_10);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_11);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_13);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_14);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_15);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_17);

	/* configure this core for EP mode operation */
	val = readl(pcie->appl_base + APPL_DM_TYPE);
	val &= ~APPL_DM_TYPE_MASK;
	val |= APPL_DM_TYPE_EP;
	writel(val, pcie->appl_base + APPL_DM_TYPE);

	writel(0x0, pcie->appl_base + APPL_CFG_SLCG_OVERRIDE);

	val = readl(pcie->appl_base + APPL_CTRL);
	val |= APPL_SYS_PRE_DET_STATE;
	val |= APPL_CTRL_HW_HOT_RST_EN;
	writel(val, pcie->appl_base + APPL_CTRL);

	val = readl(pcie->appl_base + APPL_CFG_MISC);
	val |= APPL_CFG_MISC_SLV_EP_MODE;
	val |= (APPL_CFG_MISC_ARCACHE_VAL << APPL_CFG_MISC_ARCACHE_SHIFT);
	writel(val, pcie->appl_base + APPL_CFG_MISC);

	val = readl(pcie->appl_base + APPL_PINMUX);
	val |= APPL_PINMUX_CLK_OUTPUT_IN_OVERRIDE_EN;
	val |= APPL_PINMUX_CLK_OUTPUT_IN_OVERRIDE;
	writel(val, pcie->appl_base + APPL_PINMUX);

	if (tegra_platform_is_fpga()) {
		val = readl(pcie->appl_base + APPL_PINMUX);
		val &= ~APPL_PINMUX_PEX_RST_IN_OVERRIDE_EN;
		writel(val, pcie->appl_base + APPL_PINMUX);
	}

	/* update CFG base address */
	writel(pcie->dbi_res->start & APPL_CFG_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_BASE_ADDR);

	/* update iATU_DMA base address */
	writel(pcie->atu_dma_res->start &
	       APPL_CFG_IATU_DMA_BASE_ADDR_MASK,
	       pcie->appl_base + APPL_CFG_IATU_DMA_BASE_ADDR);

	/* enable PEX_RST interrupt generation */
	val = readl(pcie->appl_base + APPL_INTR_EN_L0_0);
	val |= APPL_INTR_EN_L0_0_SYS_INTR_EN;
	if (tegra_platform_is_fpga())
		val |= APPL_INTR_EN_L0_0_PEX_RST_INT_EN;
	val |= APPL_INTR_EN_L0_0_LINK_STATE_INT_EN;
	val |= APPL_INTR_EN_L0_0_PCI_CMD_EN_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L0_0);

	val = readl(pcie->appl_base + APPL_INTR_EN_L1_0);
	val |= APPL_INTR_EN_L1_0_HOT_RESET_DONE_INT_EN;
	writel(val, pcie->appl_base + APPL_INTR_EN_L1_0);

	reset_control_deassert(pcie->core_rst);

	/* FPGA specific PHY initialization */
	if (tegra_platform_is_fpga()) {
		val = readl(pcie->appl_base + APPL_GTH_PHY);
		val &= ~APPL_GTH_PHY_RST;
		writel(val, pcie->appl_base + APPL_GTH_PHY);
		usleep_range(900, 1100);

		val = readl(pcie->appl_base + APPL_GTH_PHY);
		val &= 0xFFFF0000;
		val |= 0x780; /* required for multiple L1.2 entries */
		val |= APPL_GTH_PHY_RST;
		writel(val, pcie->appl_base + APPL_GTH_PHY);
		usleep_range(900, 1100);
	}

	/* Enable only 1MB of BAR */
	writel(pcie->bar0_size - 1, pcie->dbi_base + 0x1010);
	writel(0x00000000, pcie->dbi_base + 0x1014);

	val = readl(pcie->dbi_base + AUX_CLK_FREQ);
	val &= ~(0x3FF);
	if (tegra_platform_is_fpga())
		val |= 0x6;
	else
		val |= 19;	/* CHECK: for Silicon */
	writel(val, pcie->dbi_base + AUX_CLK_FREQ);

	inbound_atu(pcie, PCIE_ATU_REGION_INDEX0, PCIE_ATU_TYPE_MEM,
		    0x0, pcie->dma_handle, pcie->bar0_size,
		    PCIE_ATU_CR2_MATCH_MODE_BAR, 0);

	if (pcie->update_fc_fixup) {
		val = readl(pcie->dbi_base +
			    CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF);
		val |= 0x1 << CFG_TIMER_CTRL_ACK_NAK_SHIFT;
		writel(val, pcie->dbi_base +
		       CFG_TIMER_CTRL_MAX_FUNC_NUM_OFF);
	}

	program_gen3_gen4_eq_presets(pcie);

	val = readl(pcie->dbi_base + MISC_CONTROL_1);
	val |= MISC_CONTROL_1_DBI_RO_WR_EN;
	writel(val, pcie->dbi_base + MISC_CONTROL_1);

	/* Program T_cmrt and T_pwr_on values */
	val = readl(pcie->dbi_base + pcie->cfg_link_cap_l1sub);
	val &= ~(PCI_L1SS_CAP_CM_RTM_MASK | PCI_L1SS_CAP_PWRN_VAL_MASK);
	val |= (0x3C << PCI_L1SS_CAP_CM_RTM_SHIFT);	/* 60us */
	val |= (0x14 << PCI_L1SS_CAP_PWRN_VAL_SHIFT);	/* 40us */
	writel(val, pcie->dbi_base + pcie->cfg_link_cap_l1sub);

	/* Program L0s and L1 entrance latencies */
	val = readl(pcie->dbi_base + PORT_LOGIC_ACK_F_ASPM_CTRL);
	val &= ~(L0S_ENTRANCE_LAT_MASK | L1_ENTRANCE_LAT_MASK);
	val |= (0x3 << L0S_ENTRANCE_LAT_SHIFT);	/* 4us */
	val |= (0x5 << L1_ENTRANCE_LAT_SHIFT);	/* 32us */
	val |= ENTER_ASPM;
	writel(val, pcie->dbi_base + PORT_LOGIC_ACK_F_ASPM_CTRL);

	if (pcie->disabled_aspm_states & 0x1)
		disable_aspm_l0s(pcie); /* Disable L0s */
	if (pcie->disabled_aspm_states & 0x2) {
		disable_aspm_l10(pcie); /* Disable L1 */
		disable_aspm_l11(pcie); /* Disable L1.1 */
		disable_aspm_l12(pcie); /* Disable L1.2 */
	}
	if (pcie->disabled_aspm_states & 0x4)
		disable_aspm_l11(pcie); /* Disable L1.1 */
	if (pcie->disabled_aspm_states & 0x8)
		disable_aspm_l12(pcie); /* Disable L1.2 */

	/* Enable ASPM counters */
	val = EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	writel(val, pcie->dbi_base + pcie->event_cntr_ctrl);

	val = readl(pcie->dbi_base + GEN3_RELATED_OFF);
	val &= ~GEN3_RELATED_OFF_GEN3_ZRXDC_NONCOMPL;
	writel(val, pcie->dbi_base + GEN3_RELATED_OFF);

	writew(pcie->device_id, pcie->dbi_base + PCI_DEVICE_ID);

	/* Configure N_FTS & FTS */
	val = readl(pcie->dbi_base + PORT_LOGIC_ACK_F_ASPM_CTRL);
	val &= ~(N_FTS_MASK << N_FTS_SHIFT);
	val |= N_FTS_VAL << N_FTS_SHIFT;
	writel(val, pcie->dbi_base + PORT_LOGIC_ACK_F_ASPM_CTRL);

	val = readl(pcie->dbi_base + PORT_LOGIC_GEN2_CTRL);
	val &= ~FTS_MASK;
	val |= FTS_VAL;
	writel(val, pcie->dbi_base + PORT_LOGIC_GEN2_CTRL);

	if (pcie->max_speed >= 1 && pcie->max_speed <= 4) {
		val = readl(pcie->dbi_base + EP_CFG_LINK_CAP);
		val &= ~EP_CFG_LINK_CAP_MAX_SPEED_MASK;
		val |= pcie->max_speed;
		writel(val, pcie->dbi_base + EP_CFG_LINK_CAP);
	}

	writew(PCI_CLASS_MEMORY_OTHER,
	       pcie->dbi_base + PCI_CLASS_DEVICE);

	val = readl(pcie->dbi_base + MISC_CONTROL_1);
	val &= ~MISC_CONTROL_1_DBI_RO_WR_EN;
	writel(val, pcie->dbi_base + MISC_CONTROL_1);

	clk_set_rate(pcie->core_clk, GEN4_CORE_CLK_FREQ);

	/* enable LTSSM */
	val = readl(pcie->appl_base + APPL_CTRL);
	val |= APPL_CTRL_LTSSM_EN;
	writel(val, pcie->appl_base + APPL_CTRL);

	pcie->ep_state = EP_STATE_ENABLED;
	dev_info(pcie->dev, "EP init done\n");

	return;

phy_fail:
	reset_control_assert(pcie->core_apb_rst);
	clk_disable_unprepare(pcie->core_clk);
pll_fail:
	ret = pm_runtime_put_sync(pcie->dev);
	if (ret < 0)
		dev_err(pcie->dev, "runtime suspend failed: %d\n", ret);

	return;
}

static void pex_ep_event_hot_rst_done(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0;

	/* SW FixUp required during hot reset */
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L0);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_1);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_2);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_3);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_6);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_7);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_8);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_9);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_10);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_11);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_13);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_14);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_15);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_INTR_STATUS_L1_17);
	writel(0xFFFFFFFF, pcie->appl_base + APPL_MSI_CTRL_2);

	val = readl(pcie->appl_base + APPL_CTRL);
	val |= APPL_CTRL_LTSSM_EN;
	writel(val, pcie->appl_base + APPL_CTRL);
}

static inline int find_width_index(unsigned long width)
{
	if (width & (width-1))
		return -1;
	switch (width) {
	case PCIE_LNK_X1:
		return 0;
	case PCIE_LNK_X2:
		return 1;
	case PCIE_LNK_X4:
		return 2;
	case PCIE_LNK_X8:
		return 3;
	default :
		return -1;
	}
}

static void pex_ep_event_bme_change(struct tegra_pcie_dw_ep *pcie)
{
	u32 val = 0, speed = 0;
	unsigned long freq, width = 0;
	int width_index;

	/* If EP doesn't advertise L1SS, just return */
	val = readl(pcie->dbi_base + pcie->cfg_link_cap_l1sub);
	if (!(val & (PCI_L1SS_CAP_ASPM_L11S | PCI_L1SS_CAP_ASPM_L12S)))
		return;

	/* Check if BME is set to '1' */
	val = readl(pcie->dbi_base + EP_CS_STATUS_COMMAND);
	if (val & EP_CS_STATUS_COMMAND_BME) {
		ktime_t timeout;

		/* 110us for both snoop and no-snoop */
		val = 110 | (2 << PCI_LTR_SCALE_SHIFT) | LTR_MSG_REQ;
		val |= (val << LTR_MST_NO_SNOOP_SHIFT);
		writel(val, pcie->appl_base + APPL_LTR_MSG_1);
		/* Send LTR upstream */
		val = readl(pcie->appl_base + APPL_LTR_MSG_2);
		val |= APPL_LTR_MSG_2_LTR_MSG_REQ_STATE;
		writel(val, pcie->appl_base + APPL_LTR_MSG_2);

		timeout = ktime_add_us(ktime_get(), LTR_MSG_TIMEOUT);
		for (;;) {
			val = readl(pcie->appl_base + APPL_LTR_MSG_2);
			if (!(val & APPL_LTR_MSG_2_LTR_MSG_REQ_STATE))
				break;
			if (ktime_after(ktime_get(), timeout))
				break;
			usleep_range(1000, 1100);
		}
		if (val & APPL_LTR_MSG_2_LTR_MSG_REQ_STATE)
			dev_err(pcie->dev, "LTR_MSG sending failed\n");
	}

	/* Make EMC FLOOR freq request based on link width and speed */
	val = readl(pcie->dbi_base + CFG_LINK_STATUS_CONTROL);
	width = ((val >> 16) & PCI_EXP_LNKSTA_NLW) >> 4;
	width_index = find_width_index(width);
	if (width_index == -1) {
		dev_err(pcie->dev, "error in %s", __func__);
		dev_err(pcie->dev, "width in CFG_LINK_STATUS_CONTROL is"
			"wrong\n");
		return;
	}
	speed = ((val >> 16) & PCI_EXP_LNKSTA_CLS);
	freq = pcie->dvfs_tbl[width][speed - 1];
	dev_dbg(pcie->dev, "EMC Freq requested = %lu\n", freq);

	if (tegra_bwmgr_set_emc(pcie->emc_bw, freq, TEGRA_BWMGR_SET_EMC_FLOOR))
		dev_err(pcie->dev, "can't set emc clock[%lu]\n", freq);

	speed = ((val >> 16) & PCI_EXP_LNKSTA_CLS);
	clk_set_rate(pcie->core_clk, pcie_gen_freq[speed - 1]);
}

static int pcie_ep_work_thread(void *p)
{
	struct tegra_pcie_dw_ep *pcie = (struct tegra_pcie_dw_ep *)p;
	u32 event = 0;

	while (!kthread_should_stop()) {
		wait_event_interruptible(pcie->wq,
					 !kfifo_is_empty(&pcie->event_fifo) ||
					 kthread_should_stop());
		if (kthread_should_stop())
			break;
		if (!kfifo_get(&pcie->event_fifo, &event)) {
			dev_warn(pcie->dev, "empty kfifo\n");
			continue;
		}

		switch (event) {
		case EP_PEX_RST_DEASSERT:
			dev_dbg(pcie->dev, "EP_EVENT: EP_PEX_RST_DEASSERT\n");
			pex_ep_event_pex_rst_deassert(pcie);
			break;

		case EP_PEX_RST_ASSERT:
			dev_dbg(pcie->dev, "EP_EVENT: EP_PEX_RST_ASSERT\n");
			pex_ep_event_pex_rst_assert(pcie);
			break;

		case EP_PEX_HOT_RST_DONE:
			dev_dbg(pcie->dev, "EP_EVENT: EP_PEX_HOT_RST_DONE\n");
			pex_ep_event_hot_rst_done(pcie);
			break;

		case EP_PEX_BME_CHANGE:
			dev_dbg(pcie->dev, "EP_EVENT: EP_PEX_BME_CHANGE\n");
			pex_ep_event_bme_change(pcie);
			break;

		default:
			dev_warn(pcie->dev, "Invalid PCIe EP event\n");
			break;
		}
	}
	return 0;
}

static void tegra_pcie_disable_phy(struct tegra_pcie_dw_ep *pcie)
{
	int phy_count = pcie->phy_count;

	while (phy_count--)
		phy_exit(pcie->phy[phy_count]);
}

static int tegra_pcie_init_phy(struct tegra_pcie_dw_ep *pcie)
{
	int phy_count = pcie->phy_count;
	int ret;
	int i;

	for (i = 0; i < phy_count; i++) {
		ret = phy_init(pcie->phy[i]);
		if (ret < 0)
			goto err_phy_init;
	}

	return 0;

	while (i >= 0) {
		phy_exit(pcie->phy[i]);
err_phy_init:
		i--;
	}

	return ret;
}

static int tegra_pcie_power_on_phy(struct tegra_pcie_dw_ep *pcie)
{
	int phy_count = pcie->phy_count;
	int ret;
	int i;

	for (i = 0; i < phy_count; i++) {
		ret = phy_power_on(pcie->phy[i]);
		if (ret < 0)
			goto err_phy_power_on;
	}

	return 0;

	while (i >= 0) {
		phy_power_off(pcie->phy[i]);
err_phy_power_on:
		i--;
	}

	return ret;
}

static irqreturn_t pex_rst_isr(int irq, void *arg)
{
	struct tegra_pcie_dw_ep *pcie = arg;

	if (gpio_get_value(pcie->pex_rst_gpio)) {
		dev_dbg(pcie->dev, "EVENT: EP_PEX_RST_DEASSERT\n");
		if (!kfifo_put(&pcie->event_fifo, EP_PEX_RST_DEASSERT)) {
			dev_err(pcie->dev, "EVENT: fifo is full\n");
			return IRQ_HANDLED;
		}
	} else {
		dev_dbg(pcie->dev, "EVENT: EP_PEX_RST_ASSERT\n");
		if (!kfifo_put(&pcie->event_fifo, EP_PEX_RST_ASSERT)) {
			dev_err(pcie->dev, "EVENT: fifo is full\n");
			return IRQ_HANDLED;
		}
	}

	wake_up(&pcie->wq);
	return IRQ_HANDLED;
}

static void setup_margin_cmd(struct tegra_pcie_dw_ep *pcie,
			     enum margin_cmds mcmd,
			     int rcv_no, int payload)
{
	switch (mcmd) {
	case MARGIN_SET_ERR_COUNT:
		pcie->mcmd.margin_type = 2;
		pcie->mcmd.rxm_payload_check = 1;
		break;
	case MARGIN_SET_NO_CMD:
		pcie->mcmd.margin_type = 7;
		pcie->mcmd.rxm_payload_check = 1;
		break;
	case MARGIN_SET_X_OFFSET:
		pcie->mcmd.margin_type = 3;
		pcie->mcmd.rxm_payload_check = 0;
		break;
	case MARGIN_SET_Y_OFFSET:
		pcie->mcmd.margin_type = 4;
		pcie->mcmd.rxm_payload_check = 0;
		break;
	case MARGIN_SET_NORMAL:
		pcie->mcmd.margin_type = 2;
		pcie->mcmd.rxm_payload_check = 1;
		break;
	case MARGIN_CLR_ERR:
		pcie->mcmd.margin_type = 2;
		pcie->mcmd.rxm_payload_check = 1;
		break;
	}
	pcie->mcmd.rcv_no = rcv_no;
	pcie->mcmd.payload = payload;
	pcie->mcmd.rxm_cmd_check = 1;
}

static void issue_margin_cmd(struct tegra_pcie_dw_ep *pcie)
{
	u32 val, offset;
	int i;

	for (i = 0; i < pcie->init_link_width; i++) {
		offset = pcie->margin_lane_cntrl + 4 * i;
		val = readl(pcie->dbi_base + offset);
		val &= ~MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_MASK;
		val |= pcie->mcmd.rcv_no;
		val &= ~MARGIN_LANE_CNTRL_STATUS_TYPE_MASK;
		val |= (pcie->mcmd.margin_type <<
			MARGIN_LANE_CNTRL_STATUS_TYPE_SHIFT);
		val &= ~MARGIN_LANE_CNTRL_STATUS_PAYLOAD_MASK;
		val |= (pcie->mcmd.payload <<
			MARGIN_LANE_CNTRL_STATUS_PAYLOAD_SHIFT);
		writel(val, pcie->dbi_base + offset);
	}
}

static void read_margin_status(struct tegra_pcie_dw_ep *pcie,
			       struct seq_file *s, int step, char side)
{
	u32 val, offset;
	int rcv_no, margin_type, payload, i;

	for (i = 0; i < pcie->init_link_width; i++) {
		offset = pcie->margin_lane_cntrl + 4 * i;
		val = readl(pcie->dbi_base + offset);
		rcv_no = (val & MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_STATUS_MASK)
			>> MARGIN_LANE_CNTRL_STATUS_RCV_NUMBER_STATUS_SHIFT;
		margin_type = (val & MARGIN_LANE_CNTRL_STATUS_TYPE_STATUS_MASK)
				>> MARGIN_LANE_CNTRL_STATUS_TYPE_STATUS_SHIFT;
		payload = (val & MARGIN_LANE_CNTRL_STATUS_PAYLOAD_STATUS_MASK)
			>> MARGIN_LANE_CNTRL_STATUS_PAYLOAD_STATUS_SHIFT;
		if (pcie->mcmd.rxm_cmd_check) {
			if (pcie->mcmd.rcv_no != rcv_no)
				seq_printf(s, "Rcv no. check fail: rcv_no=%d "
					   "status rcv_no=%d\n",
					   pcie->mcmd.rcv_no, rcv_no);
			if (pcie->mcmd.margin_type != margin_type)
				seq_printf(s, "Margin type check fail: type=%d "
					   "status type=%d\n",
					   pcie->mcmd.margin_type, margin_type);
		}
		if (pcie->mcmd.rxm_payload_check) {
			if (pcie->mcmd.payload != payload)
				seq_printf(s, "Payload check fail: payload=%d "
					   "status payload=%d\n",
					   pcie->mcmd.payload, payload);
		}
		if ((margin_type == 3) || (margin_type == 4))
			seq_printf(s, "%s Lane=%d Side=%c Step=%d Error=0x%x\n",
				   dev_name(pcie->dev), i, side, step,
				   (payload & 0x3f));
	}
}

static int verify_timing_margin(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw_ep *pcie = (struct tegra_pcie_dw_ep *)(s->private);
	u32 val = 0;
	int i = 0;

	val = readl(pcie->dbi_base + CFG_LINK_STATUS_CONTROL);
	pcie->init_link_width = ((val >> 16) & PCI_EXP_LNKSTA_NLW) >>
				PCI_EXP_LNKSTA_NLW_SHIFT;

	val = readl(pcie->dbi_base + pcie->margin_port_cap);
	if (!(val & MARGIN_PORT_CAP_STATUS_REG_MARGINING_SW_READY) &&
	    !(val & MARGIN_PORT_CAP_STATUS_REG_MARGINING_READY)) {
		seq_puts(s, "Lane margining is not ready\n");
		return 0;
	}

	val = readl(pcie->dbi_base + GEN4_LANE_MARGINING_1);
	val &= ~GEN4_LANE_MARGINING_1_NUM_TIMING_STEPS_MASK;
	val |= NUM_TIMING_STEPS;
	writel(val, pcie->dbi_base + GEN4_LANE_MARGINING_1);

	setup_margin_cmd(pcie, MARGIN_SET_ERR_COUNT, EP_RCV_NO,
			 MAX_ERR_CNT_PAYLOAD);
	issue_margin_cmd(pcie);
	msleep(MARGIN_READ_DELAY);
	read_margin_status(pcie, s, i, NO_STEP);

	for (i = 1; i <= NUM_TIMING_STEPS; i++) {
		/* Step Margin to timing offset to left of default
		 * payload = offset | (0x10 << 6)
		 */
		setup_margin_cmd(pcie, MARGIN_SET_X_OFFSET, EP_RCV_NO,
				 i | LEFT_STEP_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_WIN_TIME);
		read_margin_status(pcie, s, i, LEFT_STEP);

		setup_margin_cmd(pcie, MARGIN_SET_NORMAL, EP_RCV_NO,
				 NORMAL_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);

		setup_margin_cmd(pcie, MARGIN_CLR_ERR, EP_RCV_NO,
				 CLR_ERR_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);
	}

	for (i = 1; i <= NUM_TIMING_STEPS; i++) {
		/* Step Margin to timing offset to right of default
		 * payload = offset | (0x00 << 6)
		 */
		setup_margin_cmd(pcie, MARGIN_SET_X_OFFSET, EP_RCV_NO,
				 i | RIGHT_STEP_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_WIN_TIME);
		read_margin_status(pcie, s, i, RIGHT_STEP);

		setup_margin_cmd(pcie, MARGIN_SET_NORMAL, EP_RCV_NO,
				 NORMAL_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);

		setup_margin_cmd(pcie, MARGIN_CLR_ERR, EP_RCV_NO,
				 CLR_ERR_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);
	}

	return 0;
}

static int verify_voltage_margin(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw_ep *pcie = (struct tegra_pcie_dw_ep *)(s->private);
	u32 val = 0;
	int i = 0;

	val = readl(pcie->dbi_base + pcie->margin_port_cap);
	if (!(val & MARGIN_PORT_CAP_STATUS_REG_MARGINING_SW_READY) &&
	    !(val & MARGIN_PORT_CAP_STATUS_REG_MARGINING_READY)) {
		seq_puts(s, "Lane margining is not ready\n");
		return 0;
	}

	val = readl(pcie->dbi_base + GEN4_LANE_MARGINING_1);
	val &= ~GEN4_LANE_MARGINING_1_MAX_VOLTAGE_OFFSET_MASK;
	val |= (NUM_VOLTAGE_STEPS <<
		GEN4_LANE_MARGINING_1_MAX_VOLTAGE_OFFSET_SHIFT);
	writel(val, pcie->dbi_base + GEN4_LANE_MARGINING_1);

	val = readl(pcie->dbi_base + MISC_CONTROL_1);
	val |= MISC_CONTROL_1_DBI_RO_WR_EN;
	writel(val, pcie->dbi_base + MISC_CONTROL_1);
	val = readl(pcie->dbi_base + GEN4_LANE_MARGINING_2);
	val |= GEN4_LANE_MARGINING_2_VOLTAGE_SUPPORTED;
	writel(val, pcie->dbi_base + GEN4_LANE_MARGINING_2);
	val = readl(pcie->dbi_base + MISC_CONTROL_1);
	val &= ~MISC_CONTROL_1_DBI_RO_WR_EN;
	writel(val, pcie->dbi_base + MISC_CONTROL_1);

	setup_margin_cmd(pcie, MARGIN_SET_ERR_COUNT, EP_RCV_NO,
			 MAX_ERR_CNT_PAYLOAD);
	issue_margin_cmd(pcie);
	msleep(MARGIN_READ_DELAY);
	read_margin_status(pcie, s, i, NO_STEP);

	for (i = 1; i <= NUM_VOLTAGE_STEPS; i++) {
		/* Step Margin to voltage offset to down of default
		 * payload = offset | (0x1 << 7)
		 */
		setup_margin_cmd(pcie, MARGIN_SET_Y_OFFSET, EP_RCV_NO,
				 i | DOWN_STEP_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_WIN_TIME);
		read_margin_status(pcie, s, i, DOWN_STEP);

		setup_margin_cmd(pcie, MARGIN_SET_NORMAL, EP_RCV_NO,
				 NORMAL_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);

		setup_margin_cmd(pcie, MARGIN_CLR_ERR, EP_RCV_NO,
				 CLR_ERR_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);
	}

	for (i = 1; i <= NUM_VOLTAGE_STEPS; i++) {
		/* Step Margin to voltage offset to up of default
		 * payload = offset | (0x00 << 7)
		 */
		setup_margin_cmd(pcie, MARGIN_SET_Y_OFFSET, EP_RCV_NO,
				 i | UP_STEP_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_WIN_TIME);
		read_margin_status(pcie, s, i, UP_STEP);

		setup_margin_cmd(pcie, MARGIN_SET_NORMAL, EP_RCV_NO,
				 NORMAL_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);

		setup_margin_cmd(pcie, MARGIN_CLR_ERR, EP_RCV_NO,
				 CLR_ERR_PAYLOAD);
		issue_margin_cmd(pcie);
		msleep(MARGIN_READ_DELAY);
		read_margin_status(pcie, s, i, NO_STEP);
	}

	return 0;
}

static inline u32 event_counter_prog(struct tegra_pcie_dw_ep *pcie, u32 event)
{
	u32 val = 0;

	val = readl(pcie->dbi_base + pcie->event_cntr_ctrl);
	val &= ~(EVENT_COUNTER_EVENT_SEL_MASK << EVENT_COUNTER_EVENT_SEL_SHIFT);
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	val |= event << EVENT_COUNTER_EVENT_SEL_SHIFT;
	val |= EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	writel(val, pcie->dbi_base + pcie->event_cntr_ctrl);
	val = readl(pcie->dbi_base + pcie->event_cntr_data);
	return val;
}

static int aspm_state_cnt(struct seq_file *s, void *data)
{
	struct tegra_pcie_dw_ep *pcie = (struct tegra_pcie_dw_ep *)(s->private);
	u32 val = 0;

	seq_printf(s, "Tx L0s entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_Tx_L0S));

	seq_printf(s, "Rx L0s entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_Rx_L0S));

	seq_printf(s, "Link L1 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1));

	seq_printf(s, "Link L1.1 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1_1));

	seq_printf(s, "Link L1.2 entry count : %u\n",
		   event_counter_prog(pcie, EVENT_COUNTER_EVENT_L1_2));

	/* Clear all counters */
	writel(EVENT_COUNTER_ALL_CLEAR,
	       pcie->dbi_base + pcie->event_cntr_ctrl);

	/* Re-enable counting */
	val = EVENT_COUNTER_ENABLE_ALL << EVENT_COUNTER_ENABLE_SHIFT;
	val |= EVENT_COUNTER_GROUP_5 << EVENT_COUNTER_GROUP_SEL_SHIFT;
	writel(val, pcie->dbi_base + pcie->event_cntr_ctrl);

	return 0;
}

#define DEFINE_ENTRY(__name)	\
static int __name ## _open(struct inode *inode, struct file *file)	\
{									\
	return single_open(file, __name, inode->i_private); \
}									\
static const struct file_operations __name ## _fops = {	\
	.open		= __name ## _open,	\
	.read		= seq_read,	\
	.llseek		= seq_lseek,	\
	.release	= single_release,	\
}

DEFINE_ENTRY(verify_timing_margin);
DEFINE_ENTRY(verify_voltage_margin);
DEFINE_ENTRY(aspm_state_cnt);

static int init_debugfs(struct tegra_pcie_dw_ep *pcie)
{
	struct dentry *d;

	d = debugfs_create_file("verify_timing_margin", 0444, pcie->debugfs,
				(void *)pcie, &verify_timing_margin_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for verify_timing_margin failed\n");

	d = debugfs_create_file("verify_voltage_margin", 0444, pcie->debugfs,
				(void *)pcie, &verify_voltage_margin_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for verify_voltage_margin failed\n");

	d = debugfs_create_file("aspm_state_cnt", 0444, pcie->debugfs,
				(void *)pcie, &aspm_state_cnt_fops);
	if (!d)
		dev_err(pcie->dev, "debugfs for aspm_state_cnt failed\n");

	return 0;
}

static int tegra_pcie_dw_ep_probe(struct platform_device *pdev)
{
	struct tegra_pcie_dw_ep *pcie;
	struct device_node *np = pdev->dev.of_node;
	struct phy **phy;
	struct pinctrl *pin = NULL;
	struct pinctrl_state *pin_state = NULL;
	struct gpio_desc *gpiod;
	char *name;
	int phy_count;
	u32 i = 0, val = 0, addr = 0;
	int irq;
	int ret = 0;

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;
	pcie->ep_state = EP_STATE_DISABLED;
	mutex_init(&pcie->disable_lock);

	ret = of_property_read_u32(np, "num-lanes", &pcie->num_lanes);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read num-lanes: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32_array(np, "nvidia,dvfs-tbl",
					 &pcie->dvfs_tbl[0][0], 16);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read EMC DVFS table: %d\n", ret);
		return ret;
	}

	of_property_read_u32(np, "nvidia,margin-port-cap",
			     &pcie->margin_port_cap);
	if (ret < 0)
		dev_err(pcie->dev, "fail to read margin-port-cap: %d\n", ret);

	of_property_read_u32(np, "nvidia,margin-lane-cntrl",
			     &pcie->margin_lane_cntrl);
	if (ret < 0)
		dev_err(pcie->dev, "fail to read margin-lane-cntrl: %d\n", ret);

	of_property_read_u32(np, "nvidia,event-cntr-ctrl",
			     &pcie->event_cntr_ctrl);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read event-cntr-ctrl: %d\n", ret);
		pcie->event_cntr_ctrl = EVENT_COUNTER_CONTROL_REG;
	}

	of_property_read_u32(np, "nvidia,event-cntr-data",
			     &pcie->event_cntr_data);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read event-cntr-data: %d\n", ret);
		pcie->event_cntr_data = EVENT_COUNTER_DATA_REG;
	}

	ret = of_property_read_u32(np, "nvidia,cfg-link-cap-l1sub",
				   &pcie->cfg_link_cap_l1sub);
	if (ret < 0) {
		dev_err(pcie->dev, "fail to read cfg-link-cap-l1sub: %d\n",
			ret);
		pcie->cfg_link_cap_l1sub = CFG_LINK_CAP_L1SUB;
	}

	of_property_read_u32(np, "nvidia,max-speed", &pcie->max_speed);

	ret = of_property_read_u32_index(np, "nvidia,controller-id", 1,
					 &pcie->cid);
	if (ret) {
		dev_err(pcie->dev, "Controller-ID is missing in DT: %d\n", ret);
		return ret;
	}

	if (pcie->cid != CTRL_5) {
		ret = uphy_bpmp_pcie_controller_state_set(pcie->cid, true);
		if (ret) {
			dev_err(pcie->dev, "Enabling controller-%d failed:%d\n",
				pcie->cid, ret);
			return ret;
		}
	}

	pcie->pex_ctl_reg = devm_regulator_get(&pdev->dev, "vddio-pex-ctl");
	if (IS_ERR(pcie->pex_ctl_reg)) {
		dev_err(&pdev->dev, "fail to get regulator: %ld\n",
			PTR_ERR(pcie->pex_ctl_reg));
		ret = PTR_ERR(pcie->pex_ctl_reg);
		goto fail_regulator;
	}
	ret = regulator_enable(pcie->pex_ctl_reg);
	if (ret < 0) {
		dev_err(&pdev->dev, "regulator enable failed: %d\n", ret);
		goto fail_regulator;
	}

	ret = of_property_read_u32(np, "nvidia,tsa-config", &addr);
	if (!ret) {
		void __iomem *tsa_addr;

		tsa_addr = ioremap(addr, 4);
		val = readl(tsa_addr);
		val |= TSA_CONFIG_STATIC0_CSW_PCIE5W_0_SO_DEV_HUBID_HUB2 <<
		       TSA_CONFIG_STATIC0_CSW_PCIE5W_0_SO_DEV_HUBID_SHIFT;
		writel(val, tsa_addr);
		iounmap(tsa_addr);
	}

	pin = devm_pinctrl_get(pcie->dev);
	if (IS_ERR(pin)) {
		ret = PTR_ERR(pin);
		dev_err(pcie->dev, "pinctrl_get failed: %d\n", ret);
		goto fail_pinctrl;
	}
	pin_state = pinctrl_lookup_state(pin, "pex_rst");
	if (!IS_ERR(pin_state)) {
		ret = pinctrl_select_state(pin, pin_state);
		if (ret < 0) {
			dev_err(pcie->dev, "setting pex_rst state fail: %d\n",
				ret);
			goto fail_pinctrl;
		}
	}
	pin_state = pinctrl_lookup_state(pin, "clkreq");
	if (!IS_ERR(pin_state)) {
		ret = pinctrl_select_state(pin, pin_state);
		if (ret < 0) {
			dev_err(pcie->dev, "setting clkreq state fail: %d\n",
				ret);
			goto fail_pinctrl;
		}
	}

	pcie->core_clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(pcie->core_clk)) {
		dev_err(&pdev->dev, "Failed to get core clock\n");
		ret = PTR_ERR(pcie->core_clk);
		goto fail_pinctrl;
	}

	pcie->appl_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						      "appl");
	if (!pcie->appl_res) {
		dev_err(&pdev->dev, "missing appl space\n");
		ret = PTR_ERR(pcie->appl_res);
		goto fail_pinctrl;
	}
	pcie->appl_base = devm_ioremap_resource(&pdev->dev, pcie->appl_res);
	if (IS_ERR(pcie->appl_base)) {
		dev_err(&pdev->dev, "mapping appl space failed\n");
		ret = PTR_ERR(pcie->appl_base);
		goto fail_pinctrl;
	}

	pcie->core_apb_rst = devm_reset_control_get(pcie->dev, "core_apb_rst");
	if (IS_ERR(pcie->core_apb_rst)) {
		dev_err(pcie->dev, "PCIE : core_apb_rst reset is missing\n");
		ret = PTR_ERR(pcie->core_apb_rst);
		goto fail_pinctrl;
	}

	phy_count = of_property_count_strings(np, "phy-names");
	if (phy_count < 0) {
		dev_err(pcie->dev, "unable to find phy entries\n");
		ret = phy_count;
		goto fail_pinctrl;
	}

	phy = devm_kcalloc(pcie->dev, phy_count, sizeof(*phy), GFP_KERNEL);
	if (!phy) {
		ret = PTR_ERR(phy);
		goto fail_pinctrl;
	}

	for (i = 0; i < phy_count; i++) {
		name = kasprintf(GFP_KERNEL, "pcie-p2u-%u", i);
		phy[i] = devm_phy_get(pcie->dev, name);
		kfree(name);
		if (IS_ERR(phy[i])) {
			ret = PTR_ERR(phy[i]);
			goto fail_pinctrl;
		}
	}

	pcie->phy_count = phy_count;
	pcie->phy = phy;

	ret = tegra_pcie_init_phy(pcie);
	if (ret) {
		dev_err(pcie->dev, "failed to init phy\n");
		goto fail_pinctrl;
	}

	pcie->dbi_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "config");
	if (!pcie->dbi_res) {
		dev_err(&pdev->dev, "missing config space\n");
		ret = PTR_ERR(pcie->dbi_res);
		goto fail_dbi_res;
	}
	pcie->dbi_base = devm_ioremap_resource(&pdev->dev, pcie->dbi_res);
	if (IS_ERR(pcie->dbi_base)) {
		dev_err(&pdev->dev, "mapping dbi space failed\n");
		ret = PTR_ERR(pcie->dbi_base);
		goto fail_dbi_res;
	}

	pcie->atu_dma_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "atu_dma");
	if (!pcie->atu_dma_res) {
		dev_err(&pdev->dev, "missing atu_dma space\n");
		ret = PTR_ERR(pcie->atu_dma_res);
		goto fail_dbi_res;
	}
	pcie->atu_dma_base = devm_ioremap_resource(&pdev->dev,
						   pcie->atu_dma_res);
	if (IS_ERR(pcie->atu_dma_base)) {
		dev_err(&pdev->dev, "mapping atu_dma space failed\n");
		ret = PTR_ERR(pcie->atu_dma_base);
		goto fail_dbi_res;
	}

	ret = of_property_read_u16(np, "nvidia,device-id", &pcie->device_id);
	if (ret) {
		dev_err(pcie->dev, "Device-ID is missing in DT: %d\n", ret);
		goto fail_dbi_res;
	}

	ret = of_property_read_u32(np, "nvidia,bar0-size", &pcie->bar0_size);
	if (ret) {
		dev_info(pcie->dev, "Setting default BAR0 size to 1MB\n");
		pcie->bar0_size = SZ_1M;
	}

	pcie->cpu_virt = dma_alloc_coherent(pcie->dev, pcie->bar0_size,
					    &pcie->dma_handle, GFP_KERNEL);
	if (!pcie->cpu_virt) {
		dev_err(pcie->dev, "BAR memory alloc failed\n");
		ret = -ENOMEM;
		goto fail_dbi_res;
	}
	dev_info(pcie->dev, "EP BAR DMA addr = 0x%llX\n", pcie->dma_handle);

	if (of_property_read_bool(pdev->dev.of_node, "nvidia,update_fc_fixup"))
		pcie->update_fc_fixup = true;

	/* Program what ASPM states sould get advertised */
	of_property_read_u32(np, "nvidia,disable-aspm-states",
			     &pcie->disabled_aspm_states);

	INIT_KFIFO(pcie->event_fifo);

	init_waitqueue_head(&pcie->wq);

	pcie->emc_bw = tegra_bwmgr_register(pcie_emc_client_id[pcie->cid]);
	if (IS_ERR_OR_NULL(pcie->emc_bw)) {
		dev_err(pcie->dev, "bwmgr registration failed\n");
		ret = -ENOENT;
		goto fail_alloc;
	}

	pcie->pcie_ep_task = kthread_run(pcie_ep_work_thread, (void *)pcie,
					 "pcie_ep_work");
	if (IS_ERR(pcie->pcie_ep_task)) {
		dev_err(pcie->dev, "failed to create pcie_ep_work thread\n");
		ret = PTR_ERR(pcie->pcie_ep_task);
		goto fail_bwmgr;
	}

	pcie->core_rst = devm_reset_control_get(pcie->dev, "core_rst");
	if (IS_ERR(pcie->core_rst)) {
		dev_err(pcie->dev, "PCIE : core_rst reset is missing\n");
		ret = PTR_ERR(pcie->core_rst);
		goto fail_thread;
	}

	pcie->irq = platform_get_irq_byname(pdev, "intr");
	if (!pcie->irq) {
		dev_err(pcie->dev, "failed to get intr interrupt\n");
		ret = -ENODEV;
		goto fail_thread;
	}

	ret = devm_request_irq(&pdev->dev, pcie->irq, tegra_pcie_irq_handler,
			       IRQF_SHARED, "tegra-pcie-intr", pcie);
	if (ret) {
		dev_err(pcie->dev, "failed to request \"intr\" irq\n");
		goto fail_thread;
	}

	pcie->pex_rst_gpio = of_get_named_gpio(np, "nvidia,pex-rst-gpio", 0);
	if (!gpio_is_valid(pcie->pex_rst_gpio)) {
		dev_err(pcie->dev, "pex-rst-gpio is missing\n");
		ret = pcie->pex_rst_gpio;
		goto fail_thread;
	}
	ret = devm_gpio_request(pcie->dev, pcie->pex_rst_gpio, "pex_rst_gpio");
	if (ret < 0) {
		dev_err(pcie->dev, "pex_rst_gpio request failed\n");
		goto fail_thread;
	}
	ret = gpio_direction_input(pcie->pex_rst_gpio);
	if (ret < 0) {
		dev_err(pcie->dev, "pex_rst_gpio direction input failed\n");
		goto fail_thread;
	}
	if (pcie->cid == CTRL_5) {
		gpiod = gpio_to_desc(pcie->pex_rst_gpio);
		if (!gpiod) {
			dev_err(pcie->dev, "Unable to get gpio desc\n");
			ret = -EINVAL;
			goto fail_thread;
		}
		ret = gpiod_set_debounce(gpiod, PERST_DEBOUNCE_TIME);
		if (ret < 0) {
			dev_err(pcie->dev, "Unable to set gpio debounce time\n");
			goto fail_thread;
		}
	}
	irq = gpio_to_irq(pcie->pex_rst_gpio);
	if (irq < 0) {
		dev_err(pcie->dev, "Unable to get irq for pex_rst_gpio\n");
		ret = irq;
		goto fail_thread;
	}
	ret = devm_request_irq(pcie->dev, (unsigned int)irq, pex_rst_isr,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "pex_rst", (void *)pcie);
	if (ret < 0) {
		dev_err(pcie->dev, "Unable to request irq for pex_rst\n");
		goto fail_thread;
	}

	name = kasprintf(GFP_KERNEL, "pcie-ep-%u", pcie->cid);
	if (!name) {
		ret = -ENOMEM;
		goto fail_thread;
	}
	pcie->debugfs = debugfs_create_dir(name, NULL);
	if (!pcie->debugfs)
		dev_err(pcie->dev, "debugfs creation failed\n");
	else
		init_debugfs(pcie);
	kfree(name);

	platform_set_drvdata(pdev, pcie);
	pm_runtime_enable(pcie->dev);

	return ret;

fail_thread:
	kthread_stop(pcie->pcie_ep_task);
fail_bwmgr:
	tegra_bwmgr_unregister(pcie->emc_bw);
fail_alloc:
	dma_free_coherent(pcie->dev, pcie->bar0_size, pcie->cpu_virt,
			  pcie->dma_handle);
fail_dbi_res:
	tegra_pcie_disable_phy(pcie);
fail_pinctrl:
	regulator_disable(pcie->pex_ctl_reg);
fail_regulator:
	if (pcie->cid != CTRL_5)
		uphy_bpmp_pcie_controller_state_set(pcie->cid, false);
	return ret;
}

static int tegra_pcie_dw_ep_remove(struct platform_device *pdev)
{
	struct tegra_pcie_dw_ep *pcie = platform_get_drvdata(pdev);
	int ret = 0;

	debugfs_remove_recursive(pcie->debugfs);

	kthread_stop(pcie->pcie_ep_task);
	pex_ep_event_pex_rst_assert(pcie);

	pm_runtime_disable(pcie->dev);

	tegra_bwmgr_unregister(pcie->emc_bw);

	dma_free_coherent(pcie->dev, pcie->bar0_size, pcie->cpu_virt,
			  pcie->dma_handle);

	tegra_pcie_disable_phy(pcie);

	regulator_disable(pcie->pex_ctl_reg);

	if (pcie->cid != CTRL_5)
		uphy_bpmp_pcie_controller_state_set(pcie->cid, false);

	return ret;
}

static const struct of_device_id tegra_pcie_dw_ep_of_match[] = {
	{ .compatible = "nvidia,tegra194-pcie-ep", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_pcie_dw_ep_of_match);

/*
 * Powergate driver registers gate/ungate callback functions to power domain.
 * PCIe EP driver need to register runtime pm callback functions to gate/ungate
 * power partition and there is no other work to do in these functions.
 */
static int tegra_pcie_dw_ep_runtime_suspend(struct device *dev)
{
	return 0;
}

static int tegra_pcie_dw_ep_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tegra_pcie_dw_ep_pm_ops = {
	.runtime_suspend = tegra_pcie_dw_ep_runtime_suspend,
	.runtime_resume = tegra_pcie_dw_ep_runtime_resume,
};

static struct platform_driver tegra_pcie_dw_ep_driver = {
	.probe		= tegra_pcie_dw_ep_probe,
	.remove		= tegra_pcie_dw_ep_remove,
	.driver = {
		.name	= "tegra-pcie-dw-ep",
		.of_match_table = tegra_pcie_dw_ep_of_match,
#ifdef CONFIG_PM
		.pm = &tegra_pcie_dw_ep_pm_ops,
#endif
	},
};

module_platform_driver(tegra_pcie_dw_ep_driver);
MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("Nvidia PCIe End-Point controller driver");
MODULE_LICENSE("GPL v2");
