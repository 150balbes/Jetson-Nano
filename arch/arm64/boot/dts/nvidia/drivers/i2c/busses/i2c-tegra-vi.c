/*
 * drivers/i2c/busses/i2c-tegra-vi.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
 *
 * Copyright (C) 2015 Google, Inc.
 * Author: Tomasz Figa <tfiga@chromium.org>
 *
 * Copyright (C) 2010-2019 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/iopoll.h>
#include <linux/of_gpio.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/tegra_prod.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/pm_runtime.h>
#include <soc/tegra/tegra_powergate.h>
#include <asm/unaligned.h>

#define TEGRA_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define BYTES_PER_FIFO_WORD 4

#define VI_I2C_REG_SHIFT			2
#define VI_I2C_REG_OFFSET			0xc00

#define I2C_CNFG				0x000
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT		12
#define I2C_CNFG_PACKET_MODE_EN			(1<<10)
#define I2C_CNFG_NEW_MASTER_FSM			(1<<11)
#define I2C_CNFG_MULTI_MASTER_MODE		(1<<17)
#define I2C_STATUS				0x01C
#define I2C_SL_CNFG				0x020
#define I2C_SL_CNFG_NACK			(1<<1)
#define I2C_SL_CNFG_NEWSL			(1<<2)
#define I2C_SL_ADDR1				0x02c
#define I2C_SL_ADDR2				0x030
#define I2C_TLOW_SEXT				0x034
#define I2C_TX_FIFO				0x050
#define I2C_RX_FIFO				0x054
#define I2C_PACKET_TRANSFER_STATUS		0x058
#define I2C_FIFO_CONTROL			0x05c
#define I2C_FIFO_CONTROL_TX_FLUSH		(1<<1)
#define I2C_FIFO_CONTROL_RX_FLUSH		(1<<0)
#define I2C_FIFO_CONTROL_RX_TRIG_1		(0<<2)
#define I2C_FIFO_CONTROL_RX_TRIG_4		(3<<2)
#define I2C_FIFO_CONTROL_RX_TRIG_8		(7<<2)
#define I2C_FIFO_CONTROL_TX_TRIG_1		(0<<5)
#define I2C_FIFO_CONTROL_TX_TRIG_4		(3<<5)
#define I2C_FIFO_CONTROL_TX_TRIG_8		(7<<5)
#define I2C_FIFO_CONTROL_TX_TRIG_SHIFT		5
#define I2C_FIFO_CONTROL_RX_TRIG_SHIFT		2
#define I2C_FIFO_STATUS				0x060
#define I2C_FIFO_STATUS_TX_MASK			0xF0
#define I2C_FIFO_STATUS_TX_SHIFT		4
#define I2C_FIFO_STATUS_RX_MASK			0x0F
#define I2C_FIFO_STATUS_RX_SHIFT		0
#define I2C_INT_MASK				0x064
#define I2C_INT_STATUS				0x068
#define I2C_INT_BUS_CLEAR_DONE			(1<<11)
#define I2C_INT_PACKET_XFER_COMPLETE		(1<<7)
#define I2C_INT_ALL_PACKETS_XFER_COMPLETE	(1<<6)
#define I2C_INT_TX_FIFO_OVERFLOW		(1<<5)
#define I2C_INT_RX_FIFO_UNDERFLOW		(1<<4)
#define I2C_INT_NO_ACK				(1<<3)
#define I2C_INT_ARBITRATION_LOST		(1<<2)
#define I2C_INT_TX_FIFO_DATA_REQ		(1<<1)
#define I2C_INT_RX_FIFO_DATA_REQ		(1<<0)
#define I2C_CLK_DIVISOR				0x06c
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT	16
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE	8
#define I2C_CLK_DIVISOR_HS_MODE_MASK		0xFFFF

#define I2C_BUS_CLEAR_CNFG			0x084
#define I2C_TLOW				4
#define I2C_TLOW_SHIFT				0
#define I2C_THIGH				2
#define I2C_THIGH_SHIFT				8
#define I2C_INTERFACE_TIMING_1			0x098
#define I2C_HS_INTERFACE_TIMING_0		0x09C
#define I2C_HS_INTERFACE_TIMING_1		0x0A0

#define DVC_CTRL_REG1				0x000
#define DVC_CTRL_REG1_INTR_EN			(1<<10)
#define DVC_CTRL_REG2				0x004
#define DVC_CTRL_REG3				0x008
#define DVC_CTRL_REG3_SW_PROG			(1<<26)
#define DVC_CTRL_REG3_I2C_DONE_INTR_EN		(1<<30)
#define DVC_STATUS				0x00c
#define DVC_STATUS_I2C_DONE_INTR		(1<<30)

#define I2C_ERR_NONE				0x00
#define I2C_ERR_NO_ACK				0x01
#define I2C_ERR_ARBITRATION_LOST		0x02
#define I2C_ERR_UNKNOWN_INTERRUPT		0x04

#define PACKET_HEADER0_HEADER_SIZE_SHIFT	28
#define PACKET_HEADER0_PACKET_ID_SHIFT		16
#define PACKET_HEADER0_CONT_ID_SHIFT		12
#define PACKET_HEADER0_CONT_ID_MASK		0xF
#define PACKET_HEADER0_PROTOCOL_I2C		(1<<4)

#define I2C_HEADER_HIGHSPEED_MODE		(1<<22)
#define I2C_HEADER_CONT_ON_NAK			(1<<21)
#define I2C_HEADER_SEND_START_BYTE		(1<<20)
#define I2C_HEADER_READ				(1<<19)
#define I2C_HEADER_10BIT_ADDR			(1<<18)
#define I2C_HEADER_IE_ENABLE			(1<<17)
#define I2C_HEADER_REPEAT_START			(1<<16)
#define I2C_HEADER_CONTINUE_XFER		(1<<15)
#define I2C_HEADER_MASTER_ADDR_SHIFT		12
#define I2C_HEADER_SLAVE_ADDR_SHIFT		1

#define I2C_BUS_CLEAR_CNFG			0x084
#define I2C_BC_SCLK_THRESHOLD			(9<<16)
#define I2C_BC_STOP_COND			(1<<2)
#define I2C_BC_TERMINATE			(1<<1)
#define I2C_BC_ENABLE				(1<<0)

#define I2C_BUS_CLEAR_STATUS			0x088
#define I2C_BC_STATUS				(1<<0)

#define I2C_CONFIG_LOAD				0x08C
#define I2C_MSTR_CONFIG_LOAD			(1 << 0)
#define I2C_SLV_CONFIG_LOAD			(1 << 1)
#define I2C_TIMEOUT_CONFIG_LOAD			(1 << 2)

#define I2C_CLKEN_OVERRIDE			0x090
#define I2C_MST_CORE_CLKEN_OVR			(1 << 0)

#define I2C_INTERFACE_TIMING_0                  0x94
#define I2C_TLOW_MASK                           0x3F
#define I2C_THIGH_SHIFT                         8
#define I2C_THIGH_MASK                          (0x3F << I2C_THIGH_SHIFT)

#define I2C_MASTER_RESET_CONTROL		0x0A8
#define I2C_CMD_ADDR0                           0x004
#define I2C_CMD_ADDR1                           0x008
#define I2C_CMD_DATA1                           0x00C
#define I2C_CMD_DATA2                           0x010
#define I2C_DEBUG_CONTROL                       0x0A4
#define I2C_TLOW_SEXT                           0x034
#define I2C_INTERRUPT_SET_REGISTER              0x074

#define I2C_MAX_TRANSFER_LEN			4096
#define I2C_CONFIG_LOAD_TIMEOUT			1000000

/* Define speed modes */
#define I2C_STANDARD_MODE			100000
#define I2C_FAST_MODE				400000
#define I2C_FAST_MODE_PLUS			1000000
#define I2C_HS_MODE				3500000

/* Max DMA buffer size = max packet transfer length + size of pkt header */
#define I2C_DMA_MAX_BUF_LEN			(I2C_MAX_TRANSFER_LEN + 12)
#define DATA_DMA_DIR_TX				(1 << 0)
#define DATA_DMA_DIR_RX				(1 << 1)

/* Upto I2C_PIO_MODE_MAX_LEN bytes, controller will use PIO mode,
 * above this, controller will use DMA to fill FIFO.
 * Here MAX PIO len is 20 bytes excluding packet header
 */
#define I2C_PIO_MODE_MAX_LEN			(20)

/* Packet header size in words */
#define I2C_PACKET_HEADER_SIZE			(3)

/*
 * msg_end_type: The bus control which need to be send at end of transfer.
 * @MSG_END_STOP: Send stop pulse at end of transfer.
 * @MSG_END_REPEAT_START: Send repeat start at end of transfer.
 * @MSG_END_CONTINUE: The following on message is coming and so do not send
 *		stop or repeat start.
 */
enum msg_end_type {
	MSG_END_STOP,
	MSG_END_REPEAT_START,
	MSG_END_CONTINUE,
};

/**
 * struct tegra_i2c_hw_feature : Different HW support on Tegra
 * @has_continue_xfer_support: Continue transfer supports.
 * @has_per_pkt_xfer_complete_irq: Has enable/disable capability for transfer
 *		complete interrupt per packet basis.
 * @has_single_clk_source: The i2c controller has single clock source. Tegra30
 *		and earlier Socs has two clock sources i.e. div-clk and
 *		fast-clk.
 * @has_config_load_reg: Has the config load register to load the new
 *		configuration.
 * @has_regulator: Controller requires extrnal regulator to be powered on
 *		during transfers.
 * @has_powergate: Controller is located inside a powergate partition.
 * @is_vi: Identifies the VI i2c controller, has a different register layout,
 *		and needs more clocks.
 * @powergate: Powergate partition ID, if applicable.
 * @clk_divisor_hs_mode: Clock divisor in HS mode.
 * @clk_divisor_std_fast_mode: Clock divisor in standard/fast mode. It is
 *		applicable if there is no fast clock source i.e. single clock
 *		source.
 */

struct tegra_i2c_hw_feature {
	bool has_continue_xfer_support;
	bool has_per_pkt_xfer_complete_irq;
	bool has_single_clk_source;
	bool has_config_load_reg;
	int clk_divisor_hs_mode;
	int clk_multiplier_hs_mode;
	int clk_divisor_std_fast_mode;
	u16 clk_divisor_fast_plus_mode;
	bool has_multi_master_mode;
	bool has_slcg_override_reg;
	bool has_sw_reset_reg;
	bool has_hw_arb_support;
	bool has_reg_write_buffering;
	bool has_slcg_support;
	bool has_regulator;
	bool has_powergate;
	bool is_vi;
	int powergate_id;
};

/**
 * struct tegra_i2c_dev	- per device i2c context
 * @dev: device reference for power management
 * @hw: Tegra i2c hw feature.
 * @adapter: core i2c layer adapter information
 * @div_clk: clock reference for div clock of i2c controller.
 * @fast_clk: clock reference for fast clock of i2c controller.
 * @base: ioremapped registers cookie
 * @cont_id: i2c controller id, used for for packet header
 * @irq: irq number of transfer complete interrupt
 * @is_dvc: identifies the DVC i2c controller, has a different register layout
 * @msg_complete: transfer completion notifier
 * @msg_err: error code for completed message
 * @msg_buf: pointer to current message data
 * @msg_buf_remaining: size of unsent data in the message buffer
 * @msg_read: identifies read transfers
 * @bus_clk_rate: current i2c bus clock rate
 * @is_suspended: prevents i2c controller accesses after suspend is called
 */
struct tegra_i2c_dev {
	struct device *dev;
	const struct tegra_i2c_hw_feature *hw;
	struct i2c_adapter adapter;
	struct clk *div_clk;
	struct clk *fast_clk;
	struct reset_control *rst;
	void __iomem *base;
	phys_addr_t phys_addr;
	int cont_id;
	int irq;
	bool irq_disabled;
	int is_dvc;
	struct completion msg_complete;
	int msg_add;
	int msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
	int msg_read;
	u32 bus_clk_rate;
	u16 clk_divisor_non_hs_mode;
	bool is_suspended;
	bool is_multimaster_mode;
	bool is_periph_reset_done;
	spinlock_t xfer_lock;
	struct i2c_bus_recovery_info bri;
	int scl_gpio;
	int sda_gpio;
	struct i2c_algo_bit_data bit_data;
	const struct i2c_algorithm *bit_algo;
	bool bit_banging_xfer_after_shutdown;
	bool is_shutdown;
	u32 low_clock_count;
	u32 high_clock_count;
	struct tegra_prod *prod_list;
	int clk_divisor_hs_mode;
	u16 hs_master_code;
	struct dma_async_tx_descriptor *rx_dma_desc;
	struct dma_chan *rx_dma_chan;
	u32 *rx_dma_buf;
	dma_addr_t rx_dma_phys;
	struct dma_async_tx_descriptor *tx_dma_desc;
	struct dma_chan *tx_dma_chan;
	u32 *tx_dma_buf;
	dma_addr_t tx_dma_phys;
	unsigned dma_buf_size;
	bool is_curr_dma_xfer;
	struct completion rx_dma_complete;
	struct completion tx_dma_complete;
	int curr_direction;
	int rx_dma_len;
	dma_cookie_t rx_cookie;
	bool enable_dma_mode;
	bool is_clkon_always;
	struct clk *slow_clk;
	struct clk *host1x_clk;
	struct regulator *reg;
};

static void dvc_writel(struct tegra_i2c_dev *i2c_dev,
	u32 val, unsigned long reg)
{
	writel(val, i2c_dev->base + reg);
}

static u32 dvc_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + reg);
}

/*
 * i2c_writel and i2c_readl will offset the register if necessary to talk
 * to the I2C block inside the DVC block
 */
static unsigned long tegra_i2c_reg_addr(struct tegra_i2c_dev *i2c_dev,
	unsigned long reg)
{
	if (i2c_dev->is_dvc)
		reg += (reg >= I2C_TX_FIFO) ? 0x10 : 0x40;
	else if (i2c_dev->hw->is_vi)
		reg = VI_I2C_REG_OFFSET + (reg << VI_I2C_REG_SHIFT);
	return reg;
}

static void i2c_writel(struct tegra_i2c_dev *i2c_dev, u32 val,
	unsigned long reg)
{
	writel(val, i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));

	/* Read back register to make sure that register writes completed */
	if (i2c_dev->hw->has_reg_write_buffering) {
		if (reg != I2C_TX_FIFO)
			readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
		else
			readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev,
						I2C_PACKET_TRANSFER_STATUS));
	}
}

static u32 i2c_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
}

static void i2c_writesl(struct tegra_i2c_dev *i2c_dev, u32 *data,
	unsigned long reg, int len)
{
	while (len--)
		i2c_writel(i2c_dev, *data++, reg);
}

static void i2c_readsl(struct tegra_i2c_dev *i2c_dev, void *data,
	unsigned long reg, int len)
{
	readsl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg), data, len);
}

static inline void tegra_i2c_gpio_setscl(void *data, int state)
{
	struct tegra_i2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->scl_gpio, state);
}

static inline int tegra_i2c_gpio_getscl(void *data)
{
	struct tegra_i2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->scl_gpio);
}

static inline void tegra_i2c_gpio_setsda(void *data, int state)
{
	struct tegra_i2c_dev *i2c_dev = data;

	gpio_set_value(i2c_dev->sda_gpio, state);
}

static inline int tegra_i2c_gpio_getsda(void *data)
{
	struct tegra_i2c_dev *i2c_dev = data;

	return gpio_get_value(i2c_dev->sda_gpio);
}

static int tegra_i2c_gpio_request(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	ret = gpio_request_one(i2c_dev->scl_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-scl");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->scl_gpio, ret);
		return ret;
	}

	ret = gpio_request_one(i2c_dev->sda_gpio,
				GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN,
				"i2c-gpio-sda");
	if (ret < 0) {
		dev_err(i2c_dev->dev, "GPIO request for gpio %d failed %d\n",
				i2c_dev->sda_gpio, ret);
		gpio_free(i2c_dev->scl_gpio);
		return ret;
	}
	return ret;
}

static void tegra_i2c_gpio_free(struct tegra_i2c_dev *i2c_dev)
{
	gpio_free(i2c_dev->scl_gpio);
	gpio_free(i2c_dev->sda_gpio);
}

static int tegra_i2c_gpio_xfer(struct i2c_adapter *adap,
	struct i2c_msg msgs[], int num)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int ret;

	ret = tegra_i2c_gpio_request(i2c_dev);
	if (ret < 0)
		return ret;

	ret = i2c_dev->bit_algo->master_xfer(adap, msgs, num);
	if (ret < 0)
		dev_err(i2c_dev->dev, "i2c-bit-algo xfer failed %d\n", ret);

	tegra_i2c_gpio_free(i2c_dev);
	return ret;
}

static int tegra_i2c_gpio_init(struct tegra_i2c_dev *i2c_dev)
{
	struct i2c_algo_bit_data *bit_data = &i2c_dev->bit_data;

	bit_data->setsda = tegra_i2c_gpio_setsda;
	bit_data->getsda = tegra_i2c_gpio_getsda;
	bit_data->setscl = tegra_i2c_gpio_setscl;
	bit_data->getscl = tegra_i2c_gpio_getscl;
	bit_data->data = i2c_dev;
	bit_data->udelay = 5; /* 100KHz */
	bit_data->timeout = HZ; /* 10 ms*/
	i2c_dev->bit_algo = &i2c_bit_algo;
	i2c_dev->adapter.algo_data = bit_data;
	return 0;
}

static void tegra_i2c_rx_dma_complete(void *args)
{
	struct tegra_i2c_dev *i2c_dev = args;
	struct dma_tx_state state;

	dma_sync_single_for_cpu(i2c_dev->dev, i2c_dev->rx_dma_phys,
			i2c_dev->dma_buf_size, DMA_FROM_DEVICE);
	dmaengine_tx_status(i2c_dev->rx_dma_chan, i2c_dev->rx_cookie, &state);
	memcpy(i2c_dev->msg_buf, i2c_dev->rx_dma_buf, i2c_dev->rx_dma_len);
	dma_sync_single_for_device(i2c_dev->dev, i2c_dev->rx_dma_phys,
			i2c_dev->dma_buf_size, DMA_FROM_DEVICE);
	complete(&i2c_dev->rx_dma_complete);
}

static void tegra_i2c_tx_dma_complete(void *args)
{
	struct tegra_i2c_dev *i2c_dev = args;

	complete(&i2c_dev->tx_dma_complete);
}

static int tegra_i2c_start_tx_dma(struct tegra_i2c_dev *i2c_dev, int len)
{
	reinit_completion(&i2c_dev->tx_dma_complete);
	dev_dbg(i2c_dev->dev, "Starting tx dma for len:%d\n", len);
	i2c_dev->tx_dma_desc = dmaengine_prep_slave_single(i2c_dev->tx_dma_chan,
				i2c_dev->tx_dma_phys, len, DMA_MEM_TO_DEV,
				DMA_PREP_INTERRUPT |  DMA_CTRL_ACK);
	if (!i2c_dev->tx_dma_desc) {
		dev_err(i2c_dev->dev, "Not able to get desc for Tx\n");
		return -EIO;
	}

	i2c_dev->tx_dma_desc->callback = tegra_i2c_tx_dma_complete;
	i2c_dev->tx_dma_desc->callback_param = i2c_dev;

	dmaengine_submit(i2c_dev->tx_dma_desc);
	dma_async_issue_pending(i2c_dev->tx_dma_chan);
	return 0;
}

static int tegra_i2c_start_rx_dma(struct tegra_i2c_dev *i2c_dev, int len)
{
	reinit_completion(&i2c_dev->rx_dma_complete);
	i2c_dev->rx_dma_len = len;

	dev_dbg(i2c_dev->dev, "Starting rx dma for len:%d\n", len);
	i2c_dev->rx_dma_desc = dmaengine_prep_slave_single(i2c_dev->rx_dma_chan,
				i2c_dev->rx_dma_phys, len, DMA_DEV_TO_MEM,
				DMA_PREP_INTERRUPT |  DMA_CTRL_ACK);
	if (!i2c_dev->rx_dma_desc) {
		dev_err(i2c_dev->dev, "Not able to get desc for Rx\n");
		return -EIO;
	}

	i2c_dev->rx_dma_desc->callback = tegra_i2c_rx_dma_complete;
	i2c_dev->rx_dma_desc->callback_param = i2c_dev;

	dmaengine_submit(i2c_dev->rx_dma_desc);
	dma_async_issue_pending(i2c_dev->rx_dma_chan);
	return 0;
}


static int tegra_i2c_init_dma_param(struct tegra_i2c_dev *i2c_dev,
							bool dma_to_memory)
{
	struct dma_chan *dma_chan;
	u32 *dma_buf;
	dma_addr_t dma_phys;
	int ret;
	struct dma_slave_config dma_sconfig;

	dma_chan = dma_request_slave_channel_reason(i2c_dev->dev,
						dma_to_memory ? "rx" : "tx");
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(i2c_dev->dev,
				"Dma channel is not available: %d\n", ret);
		return ret;
	}

	dma_buf = dma_alloc_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
							&dma_phys, GFP_KERNEL);
	if (!dma_buf) {
		dev_err(i2c_dev->dev, "Not able to allocate the dma buffer\n");
		dma_release_channel(dma_chan);
		return -ENOMEM;
	}

	if (dma_to_memory) {
		dma_sconfig.src_addr = i2c_dev->phys_addr + I2C_RX_FIFO;
		dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.src_maxburst = 0;
	} else {
		dma_sconfig.dst_addr = i2c_dev->phys_addr + I2C_TX_FIFO;
		dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.dst_maxburst = 0;
	}

	ret = dmaengine_slave_config(dma_chan, &dma_sconfig);
	if (ret)
		goto scrub;
	if (dma_to_memory) {
		i2c_dev->rx_dma_chan = dma_chan;
		i2c_dev->rx_dma_buf = dma_buf;
		i2c_dev->rx_dma_phys = dma_phys;
	} else {
		i2c_dev->tx_dma_chan = dma_chan;
		i2c_dev->tx_dma_buf = dma_buf;
		i2c_dev->tx_dma_phys = dma_phys;
	}
	return 0;

scrub:
	dma_free_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
							dma_buf, dma_phys);
	dma_release_channel(dma_chan);
	return ret;
}

static void tegra_i2c_deinit_dma_param(struct tegra_i2c_dev *i2c_dev,
	bool dma_to_memory)
{
	u32 *dma_buf;
	dma_addr_t dma_phys;
	struct dma_chan *dma_chan;

	if (dma_to_memory) {
		dma_buf = i2c_dev->rx_dma_buf;
		dma_chan = i2c_dev->rx_dma_chan;
		dma_phys = i2c_dev->rx_dma_phys;
		i2c_dev->rx_dma_chan = NULL;
		i2c_dev->rx_dma_buf = NULL;
	} else {
		dma_buf = i2c_dev->tx_dma_buf;
		dma_chan = i2c_dev->tx_dma_chan;
		dma_phys = i2c_dev->tx_dma_phys;
		i2c_dev->tx_dma_buf = NULL;
		i2c_dev->tx_dma_chan = NULL;
	}
	if (!dma_chan)
		return;

	dma_free_coherent(i2c_dev->dev, i2c_dev->dma_buf_size,
			  dma_buf, dma_phys);
	dma_release_channel(dma_chan);
}

static void tegra_i2c_mask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);

	int_mask &= ~mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static void tegra_i2c_unmask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);

	int_mask |= mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static int tegra_i2c_flush_fifos(struct tegra_i2c_dev *i2c_dev)
{
	unsigned long timeout = jiffies + HZ;
	u32 val = i2c_readl(i2c_dev, I2C_FIFO_CONTROL);

	val |= I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH;
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	while (i2c_readl(i2c_dev, I2C_FIFO_CONTROL) &
		(I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(i2c_dev->dev, "timeout waiting for fifo flush\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}
	return 0;
}

static int tegra_i2c_empty_rx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	__le32 val_le;
	int rx_fifo_avail;
	u8 *buf = i2c_dev->msg_buf;
	size_t buf_remaining = i2c_dev->msg_buf_remaining;
	int words_to_transfer;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	rx_fifo_avail = (val & I2C_FIFO_STATUS_RX_MASK) >>
		I2C_FIFO_STATUS_RX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;
	if (words_to_transfer > rx_fifo_avail)
		words_to_transfer = rx_fifo_avail;

	i2c_readsl(i2c_dev, buf, I2C_RX_FIFO, words_to_transfer);

	buf += words_to_transfer * BYTES_PER_FIFO_WORD;
	buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
	rx_fifo_avail -= words_to_transfer;

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent overwriting past the end of buf
	 */
	if (rx_fifo_avail > 0 && buf_remaining > 0) {
		BUG_ON(buf_remaining > 3);
		val = i2c_readl(i2c_dev, I2C_RX_FIFO);
		val_le = cpu_to_le32(val);
		memcpy(buf, &val_le, buf_remaining);
		buf_remaining = 0;
		rx_fifo_avail--;
	}

	BUG_ON(rx_fifo_avail > 0 && buf_remaining > 0);
	i2c_dev->msg_buf_remaining = buf_remaining;
	i2c_dev->msg_buf = buf;
	return 0;
}

static int tegra_i2c_fill_tx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	__le32 val_le;
	int tx_fifo_avail;
	u8 *buf = i2c_dev->msg_buf;
	size_t buf_remaining = i2c_dev->msg_buf_remaining;
	int words_to_transfer;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	tx_fifo_avail = (val & I2C_FIFO_STATUS_TX_MASK) >>
		I2C_FIFO_STATUS_TX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;

	/* It's very common to have < 4 bytes, so optimize that case. */
	if (words_to_transfer) {
		if (words_to_transfer > tx_fifo_avail)
			words_to_transfer = tx_fifo_avail;

		/*
		 * Update state before writing to FIFO.  If this casues us
		 * to finish writing all bytes (AKA buf_remaining goes to 0) we
		 * have a potential for an interrupt (PACKET_XFER_COMPLETE is
		 * not maskable).  We need to make sure that the isr sees
		 * buf_remaining as 0 and doesn't call us back re-entrantly.
		 */
		buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
		tx_fifo_avail -= words_to_transfer;
		i2c_dev->msg_buf_remaining = buf_remaining;
		i2c_dev->msg_buf = buf +
			words_to_transfer * BYTES_PER_FIFO_WORD;
		barrier();

		i2c_writesl(i2c_dev, (u32 *)buf, I2C_TX_FIFO,
			    words_to_transfer);

		buf += words_to_transfer * BYTES_PER_FIFO_WORD;
	}

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent reading past the end of buf, which could cross a page
	 * boundary and fault.
	 */
	if (tx_fifo_avail > 0 && buf_remaining > 0) {
		BUG_ON(buf_remaining > 3);
		memcpy(&val_le, buf, buf_remaining);
		val = le32_to_cpu(val_le);

		/* Again update before writing to FIFO to make sure isr sees. */
		i2c_dev->msg_buf_remaining = 0;
		i2c_dev->msg_buf = NULL;
		barrier();

		i2c_writel(i2c_dev, val, I2C_TX_FIFO);
	}

	return 0;
}

/*
 * One of the Tegra I2C blocks is inside the DVC (Digital Voltage Controller)
 * block.  This block is identical to the rest of the I2C blocks, except that
 * it only supports master mode, it has registers moved around, and it needs
 * some extra init to get it into I2C mode.  The register moves are handled
 * by i2c_readl and i2c_writel
 */
static void tegra_dvc_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val = 0;

	val = dvc_readl(i2c_dev, DVC_CTRL_REG3);
	val |= DVC_CTRL_REG3_SW_PROG;
	val |= DVC_CTRL_REG3_I2C_DONE_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG3);

	val = dvc_readl(i2c_dev, DVC_CTRL_REG1);
	val |= DVC_CTRL_REG1_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG1);
}

/*
 * One of the Tegra I2C blocks is inside the VI (Video Interface?)
 * block.  This block is identical to the rest of the I2C blocks, except that
 * it only supports master mode, it has registers moved around, and it needs
 * some extra init to get it into I2C mode.  The register moves are handled
 * by i2c_readl and i2c_writel.
 */
static void tegra_vi_init(struct tegra_i2c_dev *i2c_dev)
{
	i2c_writel(i2c_dev, (I2C_TLOW << I2C_TLOW_SHIFT) |
		(I2C_THIGH << I2C_THIGH_SHIFT), I2C_INTERFACE_TIMING_0);
	i2c_writel(i2c_dev, 0x04070404, I2C_INTERFACE_TIMING_1);
	i2c_writel(i2c_dev, 0x308, I2C_HS_INTERFACE_TIMING_0);
	i2c_writel(i2c_dev, 0x0B0B0B, I2C_HS_INTERFACE_TIMING_1);
	i2c_writel(i2c_dev, 0x90004, I2C_BUS_CLEAR_CNFG);
	i2c_writel(i2c_dev, 0x0, I2C_TLOW_SEXT);
}

static inline int tegra_i2c_clock_enable(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_enable(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling fast clk failed, err %d\n", ret);
			return ret;
		}
	}
	ret = clk_enable(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev,
			"Enabling div clk failed, err %d\n", ret);
		goto err_fast_disable;
	}
	if (i2c_dev->hw->is_vi) {
		ret = clk_prepare_enable(i2c_dev->slow_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling slow clk failed, err %d\n", ret);
			goto err_div_disable;
		}

		ret = clk_prepare_enable(i2c_dev->host1x_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling host1x clk failed, err %d\n", ret);
			goto err_slow_disable;
		}
	}

	return 0;

err_slow_disable:
	if (i2c_dev->hw->is_vi)
		clk_disable_unprepare(i2c_dev->slow_clk);
err_div_disable:
	clk_disable(i2c_dev->div_clk);
err_fast_disable:
	if (!i2c_dev->hw->has_single_clk_source)
		clk_disable(i2c_dev->fast_clk);

	return ret;
}

static inline void tegra_i2c_clock_disable(struct tegra_i2c_dev *i2c_dev)
{
	if (i2c_dev->hw->is_vi) {
		clk_disable_unprepare(i2c_dev->host1x_clk);
		clk_disable_unprepare(i2c_dev->slow_clk);
	}
	clk_disable(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_disable(i2c_dev->fast_clk);
}

static int tegra_i2c_wait_for_config_load(struct tegra_i2c_dev *i2c_dev)
{
	if (i2c_dev->hw->has_config_load_reg) {
		u32 val;
		int err;

		i2c_writel(i2c_dev, I2C_MSTR_CONFIG_LOAD, I2C_CONFIG_LOAD);
		if (!in_atomic())
			err = readx_poll_timeout(readl, i2c_dev->base +
						tegra_i2c_reg_addr(i2c_dev,
						I2C_CONFIG_LOAD), val, val == 0,
						1000, I2C_CONFIG_LOAD_TIMEOUT);
		else
			err = readx_poll_timeout_atomic(readl, i2c_dev->base +
						tegra_i2c_reg_addr(i2c_dev,
						I2C_CONFIG_LOAD), val, val == 0,
						1000, I2C_CONFIG_LOAD_TIMEOUT);

		if (err) {
			dev_warn(i2c_dev->dev,
				 "timeout waiting for config load\n");
			return err;
		}
	}

	return 0;
}

static int tegra_i2c_set_clk_rate(struct tegra_i2c_dev *i2c_dev)
{
	u32 clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE;
	int ret = 0;


	switch (i2c_dev->bus_clk_rate) {
	case I2C_HS_MODE:
		clk_multiplier = i2c_dev->hw->clk_multiplier_hs_mode;
		clk_multiplier *= (i2c_dev->clk_divisor_hs_mode + 1);
		break;
	case I2C_FAST_MODE_PLUS:
	case I2C_STANDARD_MODE:
	case I2C_FAST_MODE:
	default:
		clk_multiplier = (i2c_dev->low_clock_count +
				  i2c_dev->high_clock_count + 2);
		clk_multiplier *= (i2c_dev->clk_divisor_non_hs_mode + 1);
		break;
	}

	ret = clk_set_rate(i2c_dev->div_clk,
			   i2c_dev->bus_clk_rate * clk_multiplier);
	if (ret) {
		dev_err(i2c_dev->dev, "Clock rate change failed %d\n", ret);
		return ret;
	}

	return ret;
}

static void tegra_i2c_config_prod_settings(struct tegra_i2c_dev *i2c_dev)
{
	char *prod_name;
	int ret;

	switch (i2c_dev->bus_clk_rate) {
	case I2C_FAST_MODE:
		prod_name = "prod_c_fm";
		break;
	case I2C_FAST_MODE_PLUS:
		prod_name = "prod_c_fmplus";
		break;
	case I2C_HS_MODE:
		prod_name = "prod_c_hs";
		break;
	case I2C_STANDARD_MODE:
	default:
		prod_name = "prod_c_sm";
		break;
	}

	ret = tegra_prod_set_by_name(&i2c_dev->base, prod_name,
				     i2c_dev->prod_list);
	if (ret == 0)
		dev_dbg(i2c_dev->dev, "setting prod: %s\n", prod_name);
}

static void tegra_i2c_get_clk_parameters(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;

	val = i2c_readl(i2c_dev, I2C_INTERFACE_TIMING_0);
	i2c_dev->low_clock_count = val & I2C_TLOW_MASK;
	i2c_dev->high_clock_count = (val & I2C_THIGH_MASK) >> I2C_THIGH_SHIFT;

	val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
	i2c_dev->clk_divisor_hs_mode = val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	i2c_dev->clk_divisor_non_hs_mode = (val >>
			I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT);
}

static int tegra_i2c_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int err = 0;
	u32 clk_divisor;

	err = tegra_i2c_clock_enable(i2c_dev);
	if (err < 0) {
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", err);
		return err;
	}

	if (i2c_dev->hw->has_sw_reset_reg) {
		if (i2c_dev->is_periph_reset_done) {
			/* If already controller reset is done through */
			/* clock reset control register, then use SW reset */
			i2c_writel(i2c_dev, 1, I2C_MASTER_RESET_CONTROL);
			udelay(2);
			i2c_writel(i2c_dev, 0, I2C_MASTER_RESET_CONTROL);
			goto skip_periph_reset;
		}
	}
	reset_control_assert(i2c_dev->rst);
	udelay(2);
	reset_control_deassert(i2c_dev->rst);

skip_periph_reset:
	if (i2c_dev->is_dvc)
		tegra_dvc_init(i2c_dev);

	/* configuring below register to default as per TRM*/
	i2c_writel(i2c_dev, 0, I2C_TLOW_SEXT);
	i2c_writel(i2c_dev, 0, I2C_CMD_ADDR0);
	i2c_writel(i2c_dev, 0, I2C_CMD_ADDR1);
	i2c_writel(i2c_dev, 0, I2C_CMD_DATA1);
	i2c_writel(i2c_dev, 0, I2C_CMD_DATA2);
	i2c_writel(i2c_dev, 0, I2C_DEBUG_CONTROL);
	i2c_writel(i2c_dev, 0, I2C_INTERRUPT_SET_REGISTER);
	i2c_writel(i2c_dev, 0, I2C_FIFO_CONTROL);

	val = I2C_CNFG_NEW_MASTER_FSM | I2C_CNFG_PACKET_MODE_EN;
	if (i2c_dev->bus_clk_rate != I2C_HS_MODE)
		val |= (0x2 << I2C_CNFG_DEBOUNCE_CNT_SHIFT);

	if (i2c_dev->hw->has_multi_master_mode)
		val |= I2C_CNFG_MULTI_MASTER_MODE;

	i2c_writel(i2c_dev, val, I2C_CNFG);
	i2c_writel(i2c_dev, 0, I2C_INT_MASK);

	if (i2c_dev->hw->is_vi)
		tegra_vi_init(i2c_dev);

	/* Make sure clock divisor programmed correctly */
	if (i2c_dev->bus_clk_rate == I2C_HS_MODE) {
		i2c_dev->clk_divisor_hs_mode = i2c_dev->hw->clk_divisor_hs_mode;
	} else {
		val = i2c_readl(i2c_dev, I2C_CLK_DIVISOR);
		i2c_dev->clk_divisor_hs_mode =
			val & I2C_CLK_DIVISOR_HS_MODE_MASK;
	}

	clk_divisor = i2c_dev->clk_divisor_hs_mode;
	clk_divisor |= i2c_dev->clk_divisor_non_hs_mode <<
					I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT;
	i2c_writel(i2c_dev, clk_divisor, I2C_CLK_DIVISOR);

	if (i2c_dev->prod_list)
		tegra_i2c_config_prod_settings(i2c_dev);

	tegra_i2c_get_clk_parameters(i2c_dev);

	err = tegra_i2c_set_clk_rate(i2c_dev);
	if (err < 0)
		return err;

	if (!i2c_dev->is_dvc && !i2c_dev->hw->is_vi) {
		u32 sl_cfg = i2c_readl(i2c_dev, I2C_SL_CNFG);

		sl_cfg |= I2C_SL_CNFG_NACK | I2C_SL_CNFG_NEWSL;
		i2c_writel(i2c_dev, sl_cfg, I2C_SL_CNFG);
		i2c_writel(i2c_dev, 0xfc, I2C_SL_ADDR1);
		i2c_writel(i2c_dev, 0x00, I2C_SL_ADDR2);

	}

	val = 7 << I2C_FIFO_CONTROL_TX_TRIG_SHIFT |
		0 << I2C_FIFO_CONTROL_RX_TRIG_SHIFT;
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	err = tegra_i2c_flush_fifos(i2c_dev);
	if (err)
		goto err;

	if (i2c_dev->is_multimaster_mode && i2c_dev->hw->has_slcg_override_reg)
		i2c_writel(i2c_dev, I2C_MST_CORE_CLKEN_OVR, I2C_CLKEN_OVERRIDE);

	err = tegra_i2c_wait_for_config_load(i2c_dev);
	if (err)
		goto err;

	if (i2c_dev->irq_disabled) {
		i2c_dev->irq_disabled = 0;
		enable_irq(i2c_dev->irq);
	}

err:
	tegra_i2c_clock_disable(i2c_dev);
	return err;
}

static int tegra_i2c_power_enable(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	if (i2c_dev->hw->has_regulator) {
		ret = regulator_enable(i2c_dev->reg);
		if (ret)
			return ret;
	}

	if (i2c_dev->hw->has_powergate) {
		ret = tegra_unpowergate_partition(i2c_dev->hw->powergate_id);
		if (ret)
			goto err_regulator;

		ret = tegra_i2c_init(i2c_dev);
		if (ret)
			goto err_powergate;
	}

	return 0;

err_regulator:
	if (i2c_dev->hw->has_regulator)
		regulator_disable(i2c_dev->reg);
err_powergate:
	if (i2c_dev->hw->has_powergate)
		tegra_powergate_partition(i2c_dev->hw->powergate_id);

	return ret;
}

static void tegra_i2c_power_disable(struct tegra_i2c_dev *i2c_dev)
{
	if (i2c_dev->hw->has_regulator)
		regulator_disable(i2c_dev->reg);

	if (i2c_dev->hw->has_powergate)
		tegra_powergate_partition(i2c_dev->hw->powergate_id);
}

static int tegra_i2c_disable_packet_mode(struct tegra_i2c_dev *i2c_dev)
{
	u32 cnfg;

	cnfg = i2c_readl(i2c_dev, I2C_CNFG);
	if (cnfg & I2C_CNFG_PACKET_MODE_EN)
		i2c_writel(i2c_dev, cnfg & ~I2C_CNFG_PACKET_MODE_EN, I2C_CNFG);

	return tegra_i2c_wait_for_config_load(i2c_dev);
}

static irqreturn_t tegra_i2c_isr(int irq, void *dev_id)
{
	u32 status;
	const u32 status_err = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
	struct tegra_i2c_dev *i2c_dev = dev_id;
	unsigned long flags;
	u32 mask;

	spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

	status = i2c_readl(i2c_dev, I2C_INT_STATUS);

	if (status == 0) {
		dev_warn(i2c_dev->dev, "irq status 0 %08x %08x %08x\n",
			 i2c_readl(i2c_dev, I2C_PACKET_TRANSFER_STATUS),
			 i2c_readl(i2c_dev, I2C_STATUS),
			 i2c_readl(i2c_dev, I2C_CNFG));
		i2c_dev->msg_err |= I2C_ERR_UNKNOWN_INTERRUPT;

		if (!i2c_dev->irq_disabled) {
			disable_irq_nosync(i2c_dev->irq);
			i2c_dev->irq_disabled = 1;
		}
		goto err;
	}

	if (unlikely(status & status_err)) {
		tegra_i2c_disable_packet_mode(i2c_dev);
		if (status & I2C_INT_NO_ACK)
			i2c_dev->msg_err |= I2C_ERR_NO_ACK;
		if (status & I2C_INT_ARBITRATION_LOST)
			i2c_dev->msg_err |= I2C_ERR_ARBITRATION_LOST;
		goto err;
	}

	if (i2c_dev->hw->has_hw_arb_support &&
		(status & I2C_INT_BUS_CLEAR_DONE))
		goto err;

	if (i2c_dev->msg_read && (status & I2C_INT_RX_FIFO_DATA_REQ)) {
		if (i2c_dev->msg_buf_remaining)
			tegra_i2c_empty_rx_fifo(i2c_dev);
		else
			BUG();
	}

	if (!i2c_dev->msg_read && (status & I2C_INT_TX_FIFO_DATA_REQ)) {
		if (i2c_dev->msg_buf_remaining)
			tegra_i2c_fill_tx_fifo(i2c_dev);
		else
			tegra_i2c_mask_irq(i2c_dev, I2C_INT_TX_FIFO_DATA_REQ);
	}

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	if (status & I2C_INT_PACKET_XFER_COMPLETE) {
		BUG_ON(i2c_dev->msg_buf_remaining);
		complete(&i2c_dev->msg_complete);
	}
	goto done;
err:
	/* An error occurred, mask all interrupts */
	mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST |
		I2C_INT_PACKET_XFER_COMPLETE | I2C_INT_TX_FIFO_DATA_REQ |
		I2C_INT_RX_FIFO_DATA_REQ;

	if (i2c_dev->hw->has_hw_arb_support)
		mask |= I2C_INT_BUS_CLEAR_DONE;

	/* An error occurred, mask all interrupts */
	tegra_i2c_mask_irq(i2c_dev, mask);

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	if (i2c_dev->is_curr_dma_xfer) {
		if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
			dmaengine_terminate_all(i2c_dev->tx_dma_chan);
			complete(&i2c_dev->tx_dma_complete);
		} else {
			dmaengine_terminate_all(i2c_dev->rx_dma_chan);
			complete(&i2c_dev->rx_dma_complete);
		}
	}

	complete(&i2c_dev->msg_complete);
done:
	spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);
	return IRQ_HANDLED;
}

static int tegra_i2c_issue_bus_clear(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	if (i2c_dev->hw->has_hw_arb_support) {
		reinit_completion(&i2c_dev->msg_complete);
		i2c_writel(i2c_dev, I2C_BC_ENABLE
				| I2C_BC_SCLK_THRESHOLD
				| I2C_BC_STOP_COND
				| I2C_BC_TERMINATE
				, I2C_BUS_CLEAR_CNFG);
		if (i2c_dev->hw->has_config_load_reg) {
			ret = tegra_i2c_wait_for_config_load(i2c_dev);
			if (ret)
				return ret;
		}
		tegra_i2c_unmask_irq(i2c_dev, I2C_INT_BUS_CLEAR_DONE);

		ret = wait_for_completion_timeout(&i2c_dev->msg_complete,
						  TEGRA_I2C_TIMEOUT);
		if (ret == 0)
			dev_err(i2c_dev->dev, "timed out for bus clear\n");

		if (!(i2c_readl(i2c_dev, I2C_BUS_CLEAR_STATUS) & I2C_BC_STATUS))
			dev_warn(i2c_dev->dev, "Un-recovered Arb lost\n");
	} else {
		ret = i2c_recover_bus(&i2c_dev->adapter);
		if (ret) {
			dev_warn(i2c_dev->dev, "Un-recovered Arb lost\n");
			return ret;
		}
	}

	return 0;
}

static void tegra_i2c_fill_packet_header(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state,
	u32 *packet_header)
{
	u32 io_header = 0;

	/* Generic header packet */
	packet_header[0] = (0 << PACKET_HEADER0_HEADER_SIZE_SHIFT)
		| PACKET_HEADER0_PROTOCOL_I2C |
		(i2c_dev->cont_id << PACKET_HEADER0_CONT_ID_SHIFT) |
		(1 << PACKET_HEADER0_PACKET_ID_SHIFT);

	/* Payload size */
	packet_header[1] = msg->len - 1;

	/* IO header */
	io_header = I2C_HEADER_IE_ENABLE;

	if (end_state == MSG_END_CONTINUE)
		io_header |= I2C_HEADER_CONTINUE_XFER;
	else if (end_state == MSG_END_REPEAT_START)
		io_header |= I2C_HEADER_REPEAT_START;

	if (msg->flags & I2C_M_TEN) {
		io_header |= msg->addr;
		io_header |= I2C_HEADER_10BIT_ADDR;
	} else {
		io_header |=
			msg->addr << I2C_HEADER_SLAVE_ADDR_SHIFT;
	}

	if (msg->flags & I2C_M_IGNORE_NAK)
		io_header |= I2C_HEADER_CONT_ON_NAK;
	if (msg->flags & I2C_M_RD)
		io_header |= I2C_HEADER_READ;

	if (i2c_dev->bus_clk_rate == I2C_HS_MODE) {
		io_header |= I2C_HEADER_HIGHSPEED_MODE;
		io_header |= (i2c_dev->hs_master_code & 0x7)
			<< I2C_HEADER_MASTER_ADDR_SHIFT;
	}
	packet_header[2] = io_header;
}

static void tegra_i2c_config_fifo_trig(struct tegra_i2c_dev *i2c_dev,
		int len)
{
	u32 val;
	u8 maxburst = 0;
	struct dma_slave_config dma_sconfig;

	val = i2c_readl(i2c_dev, I2C_FIFO_CONTROL);
	if (len & 0xF) {
		val |= I2C_FIFO_CONTROL_TX_TRIG_1
			| I2C_FIFO_CONTROL_RX_TRIG_1;
		maxburst = 1;
	} else if (((len) >> 4) & 0x1) {
		val |= I2C_FIFO_CONTROL_TX_TRIG_4
			| I2C_FIFO_CONTROL_RX_TRIG_4;
		maxburst = 4;
	} else {
		val |= I2C_FIFO_CONTROL_TX_TRIG_8
			| I2C_FIFO_CONTROL_RX_TRIG_8;
		maxburst = 8;
	}
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
		dma_sconfig.dst_addr = i2c_dev->phys_addr + I2C_TX_FIFO;
		dma_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.dst_maxburst = maxburst;
		dmaengine_slave_config(i2c_dev->tx_dma_chan, &dma_sconfig);
	} else {
		dma_sconfig.src_addr = i2c_dev->phys_addr + I2C_RX_FIFO;
		dma_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dma_sconfig.src_maxburst = maxburst;
		dmaengine_slave_config(i2c_dev->rx_dma_chan, &dma_sconfig);
	}
}

static int tegra_i2c_start_dma_based_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state)
{
	int ret = 0;
	u32 packet_header[3];
	u32 dma_xfer_len;
	u32 int_mask;
	unsigned long flags = 0;

	i2c_dev->is_curr_dma_xfer = true;

	/* set msg_buf_remaining to zero as complete xfer is done thru DMA*/
	i2c_dev->msg_buf_remaining = 0;

	/* Enable error interrupts */
	int_mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST
					| I2C_INT_TX_FIFO_OVERFLOW;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	if (!(msg->flags & I2C_M_RD)) {
		i2c_dev->curr_direction = DATA_DMA_DIR_TX;
		dma_sync_single_for_cpu(i2c_dev->dev, i2c_dev->tx_dma_phys,
				i2c_dev->dma_buf_size, DMA_TO_DEVICE);
		/* Fill packet header */
		tegra_i2c_fill_packet_header(i2c_dev, msg, end_state,
					     packet_header);

		dma_xfer_len = I2C_PACKET_HEADER_SIZE;
		memcpy(i2c_dev->tx_dma_buf, packet_header, (dma_xfer_len * 4));

		/* Copy data payload to dma buffer*/
		memcpy(i2c_dev->tx_dma_buf + dma_xfer_len, i2c_dev->msg_buf,
				msg->len);
		/* make the dma buffer to read by dma */
		dma_sync_single_for_device(i2c_dev->dev, i2c_dev->tx_dma_phys,
				i2c_dev->dma_buf_size, DMA_TO_DEVICE);
		dma_xfer_len += DIV_ROUND_UP(msg->len, 4);

		/* Round up final length for DMA xfer in bytes*/
		dma_xfer_len = DIV_ROUND_UP(dma_xfer_len * 4, 4) * 4;

		tegra_i2c_config_fifo_trig(i2c_dev, dma_xfer_len);

		/* Acquire the lock before posting the data to FIFO */
		spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

		ret = tegra_i2c_start_tx_dma(i2c_dev, dma_xfer_len);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Starting tx dma failed, err %d\n", ret);
			goto exit;
		}
	} else {
		i2c_dev->curr_direction = DATA_DMA_DIR_RX;
		/* Round up final length for DMA xfer */
		dma_xfer_len = DIV_ROUND_UP(msg->len, 4) * 4;
		tegra_i2c_config_fifo_trig(i2c_dev, dma_xfer_len);

		ret = tegra_i2c_start_rx_dma(i2c_dev, dma_xfer_len);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Starting rx dma failed, err %d\n", ret);
			return ret;
		}
		/* Fill packet header */
		tegra_i2c_fill_packet_header(i2c_dev, msg, end_state,
					     packet_header);

		/* Acquire the lock before posting the data to FIFO */
		spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

		/* Transfer packet header through PIO */
		i2c_writel(i2c_dev, packet_header[0], I2C_TX_FIFO);
		i2c_writel(i2c_dev, packet_header[1], I2C_TX_FIFO);
		i2c_writel(i2c_dev, packet_header[2], I2C_TX_FIFO);
	}

	if (i2c_dev->hw->has_per_pkt_xfer_complete_irq)
		int_mask |= I2C_INT_PACKET_XFER_COMPLETE;

	tegra_i2c_unmask_irq(i2c_dev, int_mask);

exit:
	spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);
	return ret;
}

static int tegra_i2c_start_pio_based_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state)
{

	u32 val;
	u32 int_mask;
	u32 packet_header[3];
	unsigned long flags = 0;

	i2c_dev->is_curr_dma_xfer = false;
	val = 7 << I2C_FIFO_CONTROL_TX_TRIG_SHIFT |
		0 << I2C_FIFO_CONTROL_RX_TRIG_SHIFT;
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	i2c_dev->msg_add = msg->addr;

	/* Enable error interrupts */
	int_mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST
					| I2C_INT_TX_FIFO_OVERFLOW;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	/* Fill packet header */
	tegra_i2c_fill_packet_header(i2c_dev, msg, end_state, packet_header);

	/* Acquire the lock before posting the data to FIFO */
	spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

	i2c_writel(i2c_dev, packet_header[0], I2C_TX_FIFO);
	i2c_writel(i2c_dev, packet_header[1], I2C_TX_FIFO);
	i2c_writel(i2c_dev, packet_header[2], I2C_TX_FIFO);

	if (!(msg->flags & I2C_M_RD))
		tegra_i2c_fill_tx_fifo(i2c_dev);

	if (i2c_dev->hw->has_per_pkt_xfer_complete_irq)
		int_mask |= I2C_INT_PACKET_XFER_COMPLETE;

	if (msg->flags & I2C_M_RD)
		int_mask |= I2C_INT_RX_FIFO_DATA_REQ;
	else if (i2c_dev->msg_buf_remaining)
		int_mask |= I2C_INT_TX_FIFO_DATA_REQ;

	tegra_i2c_unmask_irq(i2c_dev, int_mask);
	spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);

	return 0;
}

static void tegra_i2c_pre_xfer_config(struct tegra_i2c_dev *i2c_dev,
		struct i2c_msg *msg)
{
	tegra_i2c_flush_fifos(i2c_dev);

	i2c_dev->is_curr_dma_xfer = false;
	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;
	i2c_dev->msg_err = I2C_ERR_NONE;
	i2c_dev->msg_read = (msg->flags & I2C_M_RD);
	reinit_completion(&i2c_dev->msg_complete);

	i2c_writel(i2c_dev, 0, I2C_INT_MASK);
}

static int tegra_i2c_handle_xfer_error(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	/* Prints errors */
	if (i2c_dev->msg_err & I2C_ERR_UNKNOWN_INTERRUPT)
		dev_warn(i2c_dev->dev, "unknown interrupt Add 0x%02x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_ERR_NO_ACK)
		dev_warn(i2c_dev->dev, "no acknowledge from address 0x%x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_ERR_ARBITRATION_LOST)
		dev_warn(i2c_dev->dev, "arb lost in communicate to add 0x%x\n",
				i2c_dev->msg_add);
	if (i2c_dev->msg_err & I2C_INT_TX_FIFO_OVERFLOW)
		dev_warn(i2c_dev->dev, "Tx fifo overflow to add 0x%x\n",
				i2c_dev->msg_add);
	/*
	 * NACK interrupt is generated before the I2C controller generates the
	 * STOP condition on the bus.
	 * So wait for 2 clock periods before resetting
	 * the controller so that STOP condition has been delivered properly.
	 */
	if (i2c_dev->msg_err == I2C_ERR_NO_ACK)
		udelay(DIV_ROUND_UP(2 * 1000000, i2c_dev->bus_clk_rate));

	ret = tegra_i2c_init(i2c_dev);
	if (ret) {
		WARN_ON(1);
		return ret;
	}

	/* Arbitration Lost occurs, Start recovery */
	if (i2c_dev->msg_err == I2C_ERR_ARBITRATION_LOST) {
		if (!i2c_dev->is_multimaster_mode) {
			ret = tegra_i2c_issue_bus_clear(i2c_dev);
			if (ret)
				return ret;
		}
		return -EAGAIN;
	}

	if (i2c_dev->msg_err == I2C_ERR_NO_ACK)
		return -EREMOTEIO;

	return -EIO;
}

static void tegra_i2c_reg_dump(struct tegra_i2c_dev *i2c_dev,
		struct i2c_msg *msg)
{
	dev_err(i2c_dev->dev, "--- register dump for debugging ----\n");
	dev_err(i2c_dev->dev, "I2C_CNFG - 0x%x\n",
			i2c_readl(i2c_dev, I2C_CNFG));
	dev_err(i2c_dev->dev, "I2C_PACKET_TRANSFER_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_PACKET_TRANSFER_STATUS));
	dev_err(i2c_dev->dev, "I2C_FIFO_CONTROL - 0x%x\n",
			i2c_readl(i2c_dev, I2C_FIFO_CONTROL));
	dev_err(i2c_dev->dev, "I2C_FIFO_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_FIFO_STATUS));
	dev_err(i2c_dev->dev, "I2C_INT_MASK - 0x%x\n",
			i2c_readl(i2c_dev, I2C_INT_MASK));
	dev_err(i2c_dev->dev, "I2C_INT_STATUS - 0x%x\n",
			i2c_readl(i2c_dev, I2C_INT_STATUS));
	dev_err(i2c_dev->dev, "msg->len - %d\n", msg->len);
	dev_err(i2c_dev->dev, "is_msg_write - %d\n",
			!(msg->flags & I2C_M_RD));
	dev_err(i2c_dev->dev, "buf_remaining - %zd\n",
			i2c_dev->msg_buf_remaining);
}

static int tegra_i2c_xfer_msg(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state)
{
	u32 int_mask = 0;
	unsigned long time_left;
	int ret;

	if (msg->len == 0)
		return -EINVAL;

	if (i2c_dev->enable_dma_mode && (msg->len > I2C_PIO_MODE_MAX_LEN)
			&& !(i2c_dev->tx_dma_chan && i2c_dev->rx_dma_chan)) {
		ret = tegra_i2c_init_dma_param(i2c_dev, true);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			return ret;
		ret = tegra_i2c_init_dma_param(i2c_dev, false);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			return ret;
	}

	tegra_i2c_pre_xfer_config(i2c_dev, msg);

	if ((msg->len > I2C_PIO_MODE_MAX_LEN) && i2c_dev->tx_dma_chan
				&& i2c_dev->rx_dma_chan)
		ret = tegra_i2c_start_dma_based_xfer(i2c_dev, msg, end_state);
	else
		ret = tegra_i2c_start_pio_based_xfer(i2c_dev, msg, end_state);

	if (ret)
		return ret;

	dev_dbg(i2c_dev->dev, "unmasked irq: %02x\n",
		i2c_readl(i2c_dev, I2C_INT_MASK));

	if (i2c_dev->is_curr_dma_xfer) {
		if (i2c_dev->curr_direction == DATA_DMA_DIR_TX) {
			time_left = wait_for_completion_timeout(
					&i2c_dev->tx_dma_complete,
					TEGRA_I2C_TIMEOUT);
			if (time_left == 0) {
				dev_err(i2c_dev->dev, "tx dma timeout\n");
				dmaengine_terminate_all(i2c_dev->tx_dma_chan);
				goto end_xfer;
			}
		} else if (i2c_dev->curr_direction == DATA_DMA_DIR_RX) {
			time_left = wait_for_completion_timeout(
					&i2c_dev->rx_dma_complete,
					TEGRA_I2C_TIMEOUT);
			if (time_left == 0) {
				dev_err(i2c_dev->dev, "rx dma timeout\n");
				dmaengine_terminate_all(i2c_dev->rx_dma_chan);
				goto end_xfer;
			}
		}
	}

	time_left = wait_for_completion_timeout(&i2c_dev->msg_complete,
						TEGRA_I2C_TIMEOUT);
	if (time_left == 0) {
		tegra_i2c_reg_dump(i2c_dev, msg);
		if (i2c_dev->is_curr_dma_xfer) {
			if (i2c_dev->curr_direction == DATA_DMA_DIR_TX)
				dmaengine_terminate_all(i2c_dev->tx_dma_chan);
			else if (i2c_dev->curr_direction == DATA_DMA_DIR_RX)
				dmaengine_terminate_all(i2c_dev->rx_dma_chan);
		}
	}

end_xfer:
	int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);
	tegra_i2c_mask_irq(i2c_dev, int_mask);

	if (time_left == 0) {
		dev_err(i2c_dev->dev,
			"i2c transfer timed out, addr 0x%04x, data 0x%02x\n",
			msg->addr, msg->buf[0]);

		ret = tegra_i2c_init(i2c_dev);
		if (!ret)
			ret = -ETIMEDOUT;
		else
			WARN_ON(1);
		return ret;
	}

	dev_dbg(i2c_dev->dev, "transfer complete: %d %d %d\n",
		ret, completion_done(&i2c_dev->msg_complete), i2c_dev->msg_err);

	if (likely(i2c_dev->msg_err == I2C_ERR_NONE))
		return 0;

	return tegra_i2c_handle_xfer_error(i2c_dev);
}

static int tegra_i2c_split_i2c_msg_xfer(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_type)
{
	int size, len, ret;
	struct i2c_msg temp_msg;
	u8 *buf = msg->buf;
	enum msg_end_type temp_end_type;

	size = msg->len;
	temp_msg.flags = msg->flags;
	temp_msg.addr = msg->addr;
	temp_end_type = end_type;
	do {
		temp_msg.buf = buf;
		len = min(size, I2C_MAX_TRANSFER_LEN);
		temp_msg.len = len;
		size -= len;
		if ((len == I2C_MAX_TRANSFER_LEN) && size)
			end_type = MSG_END_CONTINUE;
		else
			end_type = temp_end_type;
		ret = tegra_i2c_xfer_msg(i2c_dev, &temp_msg, end_type);
		if (ret)
			return ret;
		buf += len;
	} while (size != 0);

	return ret;
}

static int tegra_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
	int num)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	if (i2c_dev->is_suspended)
		return -EBUSY;

	ret = tegra_i2c_power_enable(i2c_dev);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Regulator enable failed %d\n", ret);
		return ret;
	}

	if (i2c_dev->is_shutdown && i2c_dev->bit_banging_xfer_after_shutdown)
		return tegra_i2c_gpio_xfer(adap, msgs, num);

	pm_runtime_get_sync(&adap->dev);
	ret = tegra_i2c_clock_enable(i2c_dev);
	if (ret < 0) {
		tegra_i2c_power_disable(i2c_dev);
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", ret);
		pm_runtime_put(&adap->dev);
		return ret;
	}

	if (adap->bus_clk_rate != i2c_dev->bus_clk_rate) {
		i2c_dev->bus_clk_rate = adap->bus_clk_rate;
		ret = tegra_i2c_set_clk_rate(i2c_dev);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < num; i++) {
		enum msg_end_type end_type = MSG_END_STOP;

		if (i < (num - 1)) {
			if (msgs[i + 1].flags & I2C_M_NOSTART)
				end_type = MSG_END_CONTINUE;
			else
				end_type = MSG_END_REPEAT_START;
		}
		if (msgs[i].len > I2C_MAX_TRANSFER_LEN)
			ret = tegra_i2c_split_i2c_msg_xfer(i2c_dev, &msgs[i],
							   end_type);
		else
			ret = tegra_i2c_xfer_msg(i2c_dev, &msgs[i], end_type);
		if (ret)
			break;
	}
	tegra_i2c_clock_disable(i2c_dev);
	tegra_i2c_power_disable(i2c_dev);
	pm_runtime_put(&adap->dev);
	return ret ?: i;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	u32 ret = I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK) |
		  I2C_FUNC_10BIT_ADDR |	I2C_FUNC_PROTOCOL_MANGLING;

	if (i2c_dev->hw->has_continue_xfer_support)
		ret |= I2C_FUNC_NOSTART;
	return ret;
}

static void tegra_i2c_parse_dt(struct tegra_i2c_dev *i2c_dev)
{
	struct device_node *np = i2c_dev->dev->of_node;
	int ret;
	u32 prop;

	ret = of_property_read_u32(np,
		"clock-frequency", &i2c_dev->bus_clk_rate);
	if (ret)
		i2c_dev->bus_clk_rate = 100000; /* default clock rate */

	i2c_dev->is_multimaster_mode =
		of_property_read_bool(np, "multi-master");

	i2c_dev->scl_gpio = of_get_named_gpio(np, "scl-gpio", 0);
	i2c_dev->sda_gpio = of_get_named_gpio(np, "sda-gpio", 0);

	i2c_dev->bit_banging_xfer_after_shutdown = of_property_read_bool(np,
			"nvidia,bit-banging-xfer-after-shutdown");

	ret = of_property_read_u32(np, "nvidia,hs-master-code", &prop);
	if (!ret)
		i2c_dev->hs_master_code = prop;

	i2c_dev->enable_dma_mode = of_property_read_bool(np,
			"nvidia,enable-dma-mode");

	i2c_dev->is_clkon_always = of_property_read_bool(np,
			"nvidia,clock-always-on");
}

static const struct i2c_algorithm tegra_i2c_algo = {
	.master_xfer	= tegra_i2c_xfer,
	.functionality	= tegra_i2c_func,
};

/* payload size is only 12 bit */
static struct i2c_adapter_quirks tegra_i2c_quirks = {
	.max_read_len = 4096,
	.max_write_len = 4096,
};

static const struct tegra_i2c_hw_feature tegra20_i2c_hw = {
	.has_continue_xfer_support = false,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_multiplier_hs_mode = 12,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = false,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra30_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_multiplier_hs_mode = 12,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = false,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra114_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_multiplier_hs_mode = 3,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra124_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra210_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
};

static const struct tegra_i2c_hw_feature tegra210_vii2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = false,
	.has_hw_arb_support = true,
	.has_reg_write_buffering = true,
	.has_slcg_support = false,
	.has_regulator = true,
	.has_powergate = true,
	.powergate_id = TEGRA210_POWER_DOMAIN_VENC,
	.is_vi = true,
};

static const struct tegra_i2c_hw_feature tegra186_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 2,
	.clk_multiplier_hs_mode = 13,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.has_sw_reset_reg = true,
	.has_hw_arb_support = true,
	.has_reg_write_buffering = false,
	.has_slcg_support = true,
};

/* Match table for of_platform binding */
static const struct of_device_id tegra_i2c_of_match[] = {
	{ .compatible = "nvidia,tegra210-vii2c",
		.data = &tegra210_vii2c_hw, },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_i2c_of_match);

static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev;
	struct resource *res;
	struct clk *div_clk;
	struct clk *fast_clk;
	void __iomem *base;
	phys_addr_t phys_addr;
	int irq;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	phys_addr = res->start;
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no irq resource\n");
		return -EINVAL;
	}
	irq = res->start;

	div_clk = devm_clk_get(&pdev->dev, "vii2c");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock");
		return PTR_ERR(div_clk);
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->base = base;
	i2c_dev->phys_addr = phys_addr;
	i2c_dev->div_clk = div_clk;
	i2c_dev->adapter.algo = &tegra_i2c_algo;
	i2c_dev->adapter.quirks = &tegra_i2c_quirks;
	i2c_dev->irq = irq;
	i2c_dev->dev = &pdev->dev;
	i2c_dev->dma_buf_size = I2C_DMA_MAX_BUF_LEN;

	i2c_dev->rst = devm_reset_control_get(&pdev->dev, "vii2c");
	if (IS_ERR(i2c_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset");
		return PTR_ERR(i2c_dev->rst);
	}

	tegra_i2c_parse_dt(i2c_dev);

	i2c_dev->hw = &tegra20_i2c_hw;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_i2c_of_match, &pdev->dev);
		i2c_dev->hw = match->data;
		i2c_dev->is_dvc = of_device_is_compatible(pdev->dev.of_node,
						"nvidia,tegra20-i2c-dvc");
	} else if (pdev->id == 3) {
		i2c_dev->is_dvc = 1;
	}
	init_completion(&i2c_dev->msg_complete);
	init_completion(&i2c_dev->tx_dma_complete);
	init_completion(&i2c_dev->rx_dma_complete);

	if (!i2c_dev->hw->has_single_clk_source) {
		fast_clk = devm_clk_get(&pdev->dev, "fast-clk");
		if (IS_ERR(fast_clk)) {
			dev_err(&pdev->dev, "missing fast clock");
			return PTR_ERR(fast_clk);
		}
		i2c_dev->fast_clk = fast_clk;
	}

	if (i2c_dev->hw->is_vi) {
		i2c_dev->slow_clk = devm_clk_get(&pdev->dev, "i2cslow");
		if (IS_ERR(i2c_dev->slow_clk)) {
			dev_err(&pdev->dev, "missing slow clock");
			return PTR_ERR(i2c_dev->slow_clk);
		}
		i2c_dev->host1x_clk = devm_clk_get(&pdev->dev, "host1x");
		if (IS_ERR(i2c_dev->host1x_clk)) {
			dev_err(&pdev->dev, "missing host1x clock");
			return PTR_ERR(i2c_dev->host1x_clk);
		}
	}

	if (i2c_dev->hw->has_regulator) {
		i2c_dev->reg = devm_regulator_get(&pdev->dev, "avdd_dsi_csi");
		if (IS_ERR(i2c_dev->reg)) {
			if (PTR_ERR(i2c_dev->reg) != -EPROBE_DEFER)
				dev_err(&pdev->dev, "could not get regulator");
			return PTR_ERR(i2c_dev->reg);
		}
	}

	spin_lock_init(&i2c_dev->xfer_lock);

	i2c_dev->prod_list = devm_tegra_prod_get(&pdev->dev);
	if (IS_ERR_OR_NULL(i2c_dev->prod_list)) {
		dev_dbg(&pdev->dev, "Prod-setting not available\n");
		i2c_dev->prod_list = NULL;
	}

	platform_set_drvdata(pdev, i2c_dev);

	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_prepare(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
			return ret;
		}
	}

	i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_std_fast_mode;
	if (i2c_dev->hw->clk_divisor_fast_plus_mode &&
		(i2c_dev->bus_clk_rate == I2C_FAST_MODE_PLUS))
		i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_fast_plus_mode;

	ret = tegra_i2c_set_clk_rate(i2c_dev);
	if (ret < 0)
		return ret;

	ret = clk_prepare(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
		goto unprepare_fast_clk;
	}

	if (!i2c_dev->hw->has_powergate) {
		ret = tegra_i2c_init(i2c_dev);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize i2c controller");
			goto unprepare_div_clk;
		}
	}

	if (i2c_dev->is_multimaster_mode || i2c_dev->hw->has_slcg_support)
		i2c_dev->is_clkon_always = true;

	if (i2c_dev->is_clkon_always) {
		ret = tegra_i2c_clock_enable(i2c_dev);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "div_clk enable failed %d\n",
				ret);
			goto unprepare_div_clk;
		}
	}

	if (i2c_dev->enable_dma_mode) {
		ret = tegra_i2c_init_dma_param(i2c_dev, true);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			goto disable_clk;
		ret = tegra_i2c_init_dma_param(i2c_dev, false);
		if (ret && (ret != -EPROBE_DEFER) && (ret != -ENODEV))
			goto disable_clk;
	}

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq,
			tegra_i2c_isr, 0, dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %i\n", i2c_dev->irq);
		goto disable_clk;
	}

	pm_runtime_enable(&pdev->dev);
	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	strlcpy(i2c_dev->adapter.name, "Tegra I2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.bus_clk_rate = i2c_dev->bus_clk_rate;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;
	i2c_dev->bri.scl_gpio = i2c_dev->scl_gpio;
	i2c_dev->bri.sda_gpio = i2c_dev->sda_gpio;
	i2c_dev->bri.recover_bus = i2c_generic_gpio_recovery;
	i2c_dev->adapter.bus_recovery_info = &i2c_dev->bri;

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		goto disable_clk;
	}
	i2c_dev->cont_id = i2c_dev->adapter.nr & PACKET_HEADER0_CONT_ID_MASK;
	tegra_i2c_gpio_init(i2c_dev);

	return 0;

disable_clk:
	tegra_i2c_clock_disable(i2c_dev);

unprepare_div_clk:
	clk_unprepare(i2c_dev->div_clk);

unprepare_fast_clk:
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	return ret;
}

static int tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);
	pm_runtime_disable(&i2c_dev->adapter.dev);

	if (i2c_dev->tx_dma_chan)
		tegra_i2c_deinit_dma_param(i2c_dev, false);
	if (i2c_dev->rx_dma_chan)
		tegra_i2c_deinit_dma_param(i2c_dev, true);

	if (i2c_dev->is_clkon_always)
		tegra_i2c_clock_disable(i2c_dev);
	pm_runtime_disable(&pdev->dev);

	clk_unprepare(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	return 0;
}

static void tegra_i2c_shutdown(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	dev_info(i2c_dev->dev, "Bus is shutdown down..\n");
	i2c_shutdown_adapter(&i2c_dev->adapter);
	i2c_dev->is_shutdown = true;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_i2c_suspend(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = true;

	if (i2c_dev->is_clkon_always)
		tegra_i2c_clock_disable(i2c_dev);

	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static int tegra_i2c_resume(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	int ret;

	i2c_lock_adapter(&i2c_dev->adapter);

	if (!i2c_dev->hw->has_powergate) {
		ret = tegra_i2c_init(i2c_dev);
		if (ret) {
			i2c_unlock_adapter(&i2c_dev->adapter);
			return ret;
		}
	}

	if (i2c_dev->is_clkon_always) {
		ret = tegra_i2c_clock_enable(i2c_dev);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "clock enable failed %d\n",
				ret);
			return ret;
		}
	}
	i2c_dev->is_suspended = false;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tegra_i2c_pm, tegra_i2c_suspend, tegra_i2c_resume);
#define TEGRA_I2C_PM	(&tegra_i2c_pm)
#else
#define TEGRA_I2C_PM	NULL
#endif

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.late_shutdown = tegra_i2c_shutdown,
	.driver  = {
		.name  = "tegra-vii2c",
		.of_match_table = tegra_i2c_of_match,
		.pm    = TEGRA_I2C_PM,
	},
};

static int __init tegra_i2c_init_driver(void)
{
	return platform_driver_register(&tegra_i2c_driver);
}

static void __exit tegra_i2c_exit_driver(void)
{
	platform_driver_unregister(&tegra_i2c_driver);
}

subsys_initcall(tegra_i2c_init_driver);
module_exit(tegra_i2c_exit_driver);

MODULE_DESCRIPTION("nVidia Tegra2 I2C Bus Controller driver");
MODULE_AUTHOR("Colin Cross");
MODULE_LICENSE("GPL v2");
