/*
 * NVIDIA tegra i2c slave driver
 *
 * Copyright (C) 2017-2018 NVIDIA CORPORATION. All rights reserved.
 *
 * Author: Shardar Shariff Md <smohammed@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/workqueue.h>

#define I2C_SLV_TIMEOUT (msecs_to_jiffies(10000))
#define I2C_SL_CNFG				0x20
#define I2C_SL_CNFG_RESP			BIT(0)
#define I2C_SL_CNFG_NACK			BIT(1)
#define I2C_SL_CNFG_NEW_SL			BIT(2)
#define I2C_SL_CNFG_ENABLE_SL			BIT(3)
#define I2C_SL_CNFG_PKT_MODE_EN			BIT(4)
#define I2C_SL_CNFG_ACK_WITHHOLD_EN		BIT(5)
#define I2C_SL_CNFG_ACK_LAST_BYTE		BIT(6)
#define I2C_SL_CNFG_ACK_LAST_BYTE_VALID		BIT(7)
#define I2C_SL_CNFG_FIFO_XFER_EN		BIT(20)
#define I2C_SL_CNFG_XFER_ERR_CLK_STRETCH_EN	BIT(21)

#define I2C_SL_RCVD				0x24

#define I2C_SL_STATUS				0x28
#define I2C_SL_STATUS_RNW			BIT(1)
#define I2C_SL_STATUS_RCVD			BIT(2)
#define I2C_SL_STATUS_SL_IRQ			BIT(3)
#define I2C_SL_STATUS_END_TRANS			BIT(4)

#define I2C_SL_ADDR1				0x2c
#define I2C_SL_ADDR2				0x30
#define I2C_SL_ADDR2_TEN_BIT_ADDR_MODE		BIT(0)
#define I2C_SL_ADDR2_HI_ADDR_SHIFT		1
#define I2C_SL_ADDR2_MASK			0x1FFFF
#define I2C_7BIT_ADDR_MASK			0xFF
#define I2C_10BIT_ADDR_MASK			0x3FF
#define I2C_10BIT_HI_ADDR_SHIFT			8

#define I2C_TLOW_SEXT				0x34
#define I2C_SL_DELAY_COUNT			0x3c
#define I2C_SL_DELAY_COUNT_RESET		0x1e

#define I2C_SL_INT_MASK				0x40
#define I2C_SL_INT_RCVD				BIT(2)
#define I2C_SL_INT_SL_IRQ			BIT(4)

#define I2C_SL_INT_SOURCE			0x44
#define I2C_SL_INT_SET				0x48

#define I2C_INTERRUPT_MASK_REGISTER		0x64
#define I2C_INTERRUPT_SLV_RFIFO_DATA_REQ_EN	BIT(16)
#define I2C_INTERRUPT_SLV_TFIFO_DATA_REQ_EN	BIT(17)
#define I2C_INTERRUPT_RX_BUF_FILLED_INT_EN	BIT(23)
#define I2C_INTERRUPT_TX_BUFFER_REQ_INT_EN	BIT(24)
#define I2C_INTERRUPT_SLV_PKT_XFER_ERR_INT_EN	BIT(25)

#define I2C_INTERRUPT_STATUS_REGISTER		0x68
#define I2C_INTERRUPT_SLV_RFIFO_DATA_REQ	BIT(16)
#define I2C_INTERRUPT_SLV_TFIFO_DATA_REQ	BIT(17)
#define I2C_INTERRUPT_SLV_RX_BUFFER_FILLED	BIT(23)
#define I2C_INTERRUPT_SLV_TX_BUFFER_REQ		BIT(24)
#define I2C_INTERRUPT_SLV_PKT_XFER_ERR		BIT(25)

#define I2C_INTERRUPT_SOURCE_REGISTER		0x70
#define I2C_INTERRUPT_SLV_WR2RD			BIT(26)
#define I2C_INTERRUPT_SET_REGISTER		0x74

#define I2C_SLV_TX_FIFO				0x78
#define I2C_SLV_RX_FIFO				0x7c

#define I2C_SLV_PACKET_STATUS			0x80
#define I2C_SLV_PACKET_TRANSFER_BYTENUM_MASK	0xFFFF
#define I2C_SLV_PACKET_TRANSFER_BYTENUM_SHIFT	0

#define I2C_CONFIG_LOAD				0x8c
#define I2C_TIMEOUT_CONFIG_LOAD			BIT(2)
#define I2C_CONFIG_LOAD_SLV			BIT(1)
#define I2C_CONFIG_LOAD_TIMEOUT			1000000

#define I2C_CLKEN_OVERRIDE			0x90
#define I2C_DEBUG_CONTROL			0xa4

#define I2C_SLV_PAYLOAD				0xbc

#define I2C_SLV_FIFO_CONTROL			0xc0
#define I2C_SLV_FIFO_RX_FLUSH			BIT(0)
#define I2C_SLV_FIFO_TX_FLUSH			BIT(1)
#define I2C_SLV_FIFO_RX_FIFO_TRIG_SHIFT		4
#define I2C_SLV_FIFO_RX_FIFO_TRIG_1		(0 << 4)
#define I2C_SLV_FIFO_TX_FIFO_TRIG_SHIFT		16
#define I2C_SLV_FIFO_TX_FIFO_TRIG_1		(0 << 16)
#define I2C_SLV_FIFO_TX_FIFO_TRIG_2		(1 << 16)
#define I2C_SLV_FIFO_TX_FIFO_TRIG_3		(2 << 16)
#define I2C_SLV_FIFO_TX_FIFO_TRIG_4		(3 << 16)

#define I2C_SLV_FIFO_STATUS			0xc4
#define I2C_SLV_RX_FIFO_FULL_CNT_MASK		0xff
#define I2C_SLV_FIFO_STATUS_RX_SHIFT		0
#define I2C_SLV_TX_FIFO_EMPTY_CNT_MASK		(0xff << 16)
#define I2C_SLV_FIFO_STATUS_TX_SHIFT		16

#define I2C_FIFO_DEPTH				512
#define I2C_TX_FIFO_THRESHOLD			4
#define I2C_RX_FIFO_THRESHOLD			1
#define BYTES_PER_FIFO_WORD			4

#define I2C_SLV_CLK_RATE			204000000

struct tegra_i2cslv_dev {
	struct device *dev;
	struct i2c_adapter adap;
	struct clk *div_clk;
	u32 clk_rate;
	struct reset_control *rstc;
	void __iomem *base;
	struct i2c_client *slave;
	raw_spinlock_t xfer_lock;
	u8 *rx_buffer;
	u32 *tx_buffer;
	u32 rx_count;
	bool rx_in_progress;
	bool tx_in_progress;
	u32 buffer_size;
	struct delayed_work work;
	bool failed_xfer;
};

static inline u32 tegra_i2cslv_readl(struct tegra_i2cslv_dev *i2cslv_dev,
				     u32 reg)
{
	return readl(i2cslv_dev->base + reg);
}

static inline void tegra_i2cslv_writel(struct tegra_i2cslv_dev *i2cslv_dev,
				       u32 val, u32 reg)
{
	writel(val, i2cslv_dev->base + reg);
}

static void tegra_i2cslv_mask_irq(struct tegra_i2cslv_dev *i2c_dev, u32 mask)
{
	u32 int_mask;

	int_mask = tegra_i2cslv_readl(i2c_dev,
			I2C_INTERRUPT_MASK_REGISTER) & ~mask;
	tegra_i2cslv_writel(i2c_dev, int_mask, I2C_INTERRUPT_MASK_REGISTER);
}

static void tegra_i2cslv_unmask_irq(struct tegra_i2cslv_dev *i2c_dev, u32 mask)
{
	u32 int_mask;

	int_mask = tegra_i2cslv_readl(i2c_dev,
			I2C_INTERRUPT_MASK_REGISTER) | mask;
	tegra_i2cslv_writel(i2c_dev, int_mask, I2C_INTERRUPT_MASK_REGISTER);
}

static void tegra_i2cslv_dump_reg(struct tegra_i2cslv_dev *i2cslv_dev)
{
	dev_warn(i2cslv_dev->dev, "I2C_I2C_SL_INT_SOURCE_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_SL_INT_SOURCE));
	dev_warn(i2cslv_dev->dev, "I2C_INTERRUPT_STATUS_REGISTER_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_INTERRUPT_STATUS_REGISTER));
	dev_warn(i2cslv_dev->dev, "I2C_I2C_SL_STATUS_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_SL_STATUS));
	dev_warn(i2cslv_dev->dev, "I2C_INTERRUPT_SOURCE_REGISTER 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_INTERRUPT_SOURCE_REGISTER));
	dev_warn(i2cslv_dev->dev, "I2C_I2C_SL_CNFG_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_SL_CNFG));
	dev_warn(i2cslv_dev->dev, "I2C_SL_ADDR1 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_SL_ADDR1));
	dev_warn(i2cslv_dev->dev, "I2C_INTERRUPT_MASK_REGISTER_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_INTERRUPT_MASK_REGISTER));
	dev_warn(i2cslv_dev->dev, "I2C_I2C_SL_INT_MASK_0 0x%x\n",
		 tegra_i2cslv_readl(i2cslv_dev, I2C_SL_INT_MASK));
}

static int tegra_i2cslv_load_config(struct tegra_i2cslv_dev *i2cslv_dev)
{
	u32 i2c_load_config_reg;
	u32 val;
	int ret = 0;

	i2c_load_config_reg = tegra_i2cslv_readl(i2cslv_dev, I2C_CONFIG_LOAD);
	i2c_load_config_reg |= I2C_CONFIG_LOAD_SLV;
	tegra_i2cslv_writel(i2cslv_dev, i2c_load_config_reg, I2C_CONFIG_LOAD);

	if (in_interrupt())
		ret = readl_poll_timeout_atomic(i2cslv_dev->base +
				I2C_CONFIG_LOAD, val,
				!(val & I2C_CONFIG_LOAD_SLV),
				1000, I2C_CONFIG_LOAD_TIMEOUT);
	else
		ret = readl_poll_timeout(i2cslv_dev->base + I2C_CONFIG_LOAD,
				val, !(val & I2C_CONFIG_LOAD_SLV),
				1000, I2C_CONFIG_LOAD_TIMEOUT);
	if (ret) {
		dev_err(i2cslv_dev->dev, "ERR unable to load i2cslv config\n");
		return ret;
	}

	return 0;
}

static void tegra_i2cslv_read_from_fifo(struct tegra_i2cslv_dev *i2cslv_dev)
{
	struct i2c_slave_data data;
	u32 reg, cnt, byte_cnt, buf_remaining;
	u32 rx_fifo_avail, words_to_transfer;
	u32 *buf32 = (u32 *)i2cslv_dev->rx_buffer;
	u8 *buf = i2cslv_dev->rx_buffer;
	u32 curr_bytes_transferred;

	reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SLV_PACKET_STATUS);
	byte_cnt = (reg & I2C_SLV_PACKET_TRANSFER_BYTENUM_MASK) >>
		I2C_SLV_PACKET_TRANSFER_BYTENUM_SHIFT;
	reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SLV_FIFO_STATUS);
	rx_fifo_avail = (reg & I2C_SLV_RX_FIFO_FULL_CNT_MASK) >>
		I2C_SLV_FIFO_STATUS_RX_SHIFT;

	if (rx_fifo_avail) {
		/* Read the data from FIFO if present*/
		buf_remaining = byte_cnt - i2cslv_dev->rx_count;
		curr_bytes_transferred = buf_remaining;
		words_to_transfer = (buf_remaining / BYTES_PER_FIFO_WORD);
		if (words_to_transfer > rx_fifo_avail)
			words_to_transfer = rx_fifo_avail;

		for (cnt = 0; cnt < words_to_transfer; cnt++) {
			buf32[cnt] = tegra_i2cslv_readl(i2cslv_dev,
					I2C_SLV_RX_FIFO);
		}
		buf += words_to_transfer * BYTES_PER_FIFO_WORD;
		buf_remaining -= (words_to_transfer * BYTES_PER_FIFO_WORD);
		rx_fifo_avail -= words_to_transfer;

		if (rx_fifo_avail > 0 && buf_remaining > 0) {
			WARN_ON(buf_remaining > 3);
			reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SLV_RX_FIFO);
			reg = cpu_to_le32(reg);
			memcpy(buf, &reg, buf_remaining);
			buf_remaining = 0;
			rx_fifo_avail--;
		}
		data.buf = i2cslv_dev->rx_buffer;
		data.size = curr_bytes_transferred;
		if (data.size)
			i2c_slave_event(i2cslv_dev->slave,
					I2C_SLAVE_WRITE_BUFFER_RECEIVED,
					&data);
	}
	i2cslv_dev->rx_count = 0;
}

/* tegra_i2cslv_handle_rx - To get the data from bus and provide the
 *			    data to client driver
 */
static void tegra_i2cslv_handle_rx(struct tegra_i2cslv_dev *i2cslv_dev,
		u32 i2c_int_src, u32 i2c_slv_sts)
{
	struct i2c_slave_data data;
	u32 reg;

	if (!i2cslv_dev->rx_in_progress) {
		/* Address received and Master Write/Slave Read*/
		/* Read the data from FIFO, only 1 byte will be received */
		i2cslv_dev->rx_buffer[0] = tegra_i2cslv_readl(i2cslv_dev,
				I2C_SLV_RX_FIFO);
		data.buf = i2cslv_dev->rx_buffer;
		data.size = 1;
		i2c_slave_event(i2cslv_dev->slave,
				I2C_SLAVE_WRITE_BUFFER_REQUESTED,
				&data);
		i2cslv_dev->rx_in_progress = true;
		if (i2cslv_dev->buffer_size > I2C_FIFO_DEPTH)
			tegra_i2cslv_unmask_irq(i2cslv_dev,
					I2C_INTERRUPT_SLV_RFIFO_DATA_REQ_EN);
		i2cslv_dev->rx_count = 0;
	} else {
		tegra_i2cslv_read_from_fifo(i2cslv_dev);
	}
	/* Clear the RX buffer filled interrupt*/
	tegra_i2cslv_writel(i2cslv_dev, I2C_INTERRUPT_SLV_RX_BUFFER_FILLED,
			    I2C_INTERRUPT_STATUS_REGISTER);
	/* Release the SCL line */
	reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SL_CNFG);
	reg |= I2C_SL_CNFG_ACK_LAST_BYTE_VALID;
	tegra_i2cslv_writel(i2cslv_dev, reg, I2C_SL_CNFG);
}

static void tegra_i2cslv_empty_rxfifo(struct tegra_i2cslv_dev *i2cslv_dev)
{
	struct i2c_slave_data data;
	u32 rx_fifo_avail;
	u32 reg, cnt;
	u32 *buf32 = (u32 *)i2cslv_dev->rx_buffer;

	if (i2cslv_dev->rx_in_progress) {
		reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SLV_FIFO_STATUS);
		rx_fifo_avail = (reg & I2C_SLV_RX_FIFO_FULL_CNT_MASK)
			>> I2C_SLV_FIFO_STATUS_RX_SHIFT;

		if (rx_fifo_avail) {
			if (rx_fifo_avail > I2C_RX_FIFO_THRESHOLD)
				rx_fifo_avail = I2C_RX_FIFO_THRESHOLD;

			for (cnt = 0; cnt < rx_fifo_avail; cnt++) {
				buf32[cnt] = tegra_i2cslv_readl(i2cslv_dev,
						I2C_SLV_RX_FIFO);
			}

			data.buf = i2cslv_dev->rx_buffer;
			data.size = rx_fifo_avail * BYTES_PER_FIFO_WORD;
			i2c_slave_event(i2cslv_dev->slave,
					I2C_SLAVE_WRITE_BUFFER_RECEIVED,
					&data);
			i2cslv_dev->rx_count += data.size;
		}
	}
}

/* tegra_i2cslv_handle_tx - To get the data byte fron client driver and
 *			    send it to master over bus.
 */
void tegra_i2cslv_handle_tx(struct tegra_i2cslv_dev *i2cslv_dev,
		u32 i2c_int_src, u32 i2c_slv_sts)
{
	struct i2c_slave_data data;
	int cnt, words_to_transfer;

	i2c_slave_event(i2cslv_dev->slave, I2C_SLAVE_READ_BUFFER_REQUESTED,
			&data);
	i2cslv_dev->tx_buffer = (u32 *)data.buf;
	i2cslv_dev->tx_in_progress = true;

	/* Data size is greater than FIFO depth */
	if (data.size > I2C_FIFO_DEPTH) {
		tegra_i2cslv_unmask_irq(i2cslv_dev,
				I2C_INTERRUPT_SLV_TFIFO_DATA_REQ_EN);
	} else {
		/* Rounds down to not include partial word at the end of buf */
		words_to_transfer = ALIGN(data.size, BYTES_PER_FIFO_WORD);

		/* Fill the data to TFIFO */
		for (cnt = 0; cnt < words_to_transfer; cnt++)
			tegra_i2cslv_writel(i2cslv_dev,
					    i2cslv_dev->tx_buffer[cnt],
					    I2C_SLV_TX_FIFO);
		i2cslv_dev->tx_buffer += words_to_transfer;
	}

	/*  Clear the TX Buffer request interrupt */
	tegra_i2cslv_writel(i2cslv_dev, I2C_INTERRUPT_SLV_TX_BUFFER_REQ,
			    I2C_INTERRUPT_STATUS_REGISTER);
}

void tegra_i2cslv_fill_txfifo(struct tegra_i2cslv_dev *i2cslv_dev)
{
	u32 cnt;

	for (cnt = 0; cnt < I2C_TX_FIFO_THRESHOLD; cnt++)
		tegra_i2cslv_writel(i2cslv_dev, i2cslv_dev->tx_buffer[cnt],
				    I2C_SLV_TX_FIFO);
	i2cslv_dev->tx_buffer += I2C_TX_FIFO_THRESHOLD;
}

/*
 * tegra_i2cslv_handle_stop - To handle end of transfer.
 */
static void tegra_i2cslv_handle_stop(struct tegra_i2cslv_dev *i2cslv_dev,
		u32 i2c_int_src, u32 i2c_slv_sts)
{
	struct i2c_slave_data data;
	u32 reg, byte_cnt;

	if (i2c_int_src & I2C_INTERRUPT_SLV_PKT_XFER_ERR) {
		reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SLV_PACKET_STATUS);
		byte_cnt = (reg & I2C_SLV_PACKET_TRANSFER_BYTENUM_MASK) >>
			I2C_SLV_PACKET_TRANSFER_BYTENUM_SHIFT;

		if (i2cslv_dev->rx_in_progress) {
			tegra_i2cslv_read_from_fifo(i2cslv_dev);
			/* Clear RxFifo data req interrupt */
			tegra_i2cslv_mask_irq(i2cslv_dev,
					I2C_INTERRUPT_SLV_RFIFO_DATA_REQ_EN);
			i2cslv_dev->rx_in_progress = false;
		} else if (i2cslv_dev->tx_in_progress) {
			/* Flush the TX FIFO */
			reg = tegra_i2cslv_readl(i2cslv_dev,
						 I2C_SLV_FIFO_CONTROL);
			reg |= I2C_SLV_FIFO_TX_FLUSH;
			tegra_i2cslv_writel(i2cslv_dev, reg,
					    I2C_SLV_FIFO_CONTROL);

			data.size = byte_cnt;
			/* Notify to client number of bytes read by master */
			i2c_slave_event(i2cslv_dev->slave,
					I2C_SLAVE_READ_BUFFER_COUNT,
					&data);
			/* Clear TxFifo data req interrupt */
			tegra_i2cslv_mask_irq(i2cslv_dev,
					I2C_INTERRUPT_SLV_TFIFO_DATA_REQ_EN);
			i2cslv_dev->tx_in_progress = false;
		}

		/* Clear the interrupts */
		tegra_i2cslv_writel(i2cslv_dev, I2C_INTERRUPT_SLV_PKT_XFER_ERR,
				I2C_INTERRUPT_STATUS_REGISTER);
	}
	/* Clear the interrupt */
	tegra_i2cslv_writel(i2cslv_dev, I2C_SL_STATUS_END_TRANS,
			I2C_SL_STATUS);
	i2c_slave_event(i2cslv_dev->slave, I2C_SLAVE_STOP, &data);
}

static int tegra_i2cslv_init(struct tegra_i2cslv_dev *i2cslv_dev)
{
	u32 reg;
	u32 hi_addr;

	i2cslv_dev->buffer_size = i2cslv_dev->slave->buffer_size;
	i2cslv_dev->rx_count = 0;

	if (i2cslv_dev->slave->flags & I2C_CLIENT_TEN) {
		/* Program the 10-bit slave address */
		tegra_i2cslv_writel(i2cslv_dev, i2cslv_dev->slave->addr &
				I2C_7BIT_ADDR_MASK, I2C_SL_ADDR1);
		hi_addr = ((i2cslv_dev->slave->addr & I2C_10BIT_ADDR_MASK) >>
			I2C_10BIT_HI_ADDR_SHIFT);
		reg = I2C_SL_ADDR2_TEN_BIT_ADDR_MODE |
			(hi_addr << I2C_SL_ADDR2_HI_ADDR_SHIFT);
		tegra_i2cslv_writel(i2cslv_dev, reg, I2C_SL_ADDR2);
	} else {
		/* Program the 7-bit slave address */
		tegra_i2cslv_writel(i2cslv_dev, i2cslv_dev->slave->addr &
				I2C_7BIT_ADDR_MASK, I2C_SL_ADDR1);
		reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SL_ADDR2);
		reg &= ~(I2C_SL_ADDR2_MASK);
		tegra_i2cslv_writel(i2cslv_dev, reg, I2C_SL_ADDR2);
	}

	/* Configure FIFO controls */
	reg = I2C_SLV_FIFO_RX_FIFO_TRIG_1 | I2C_SLV_FIFO_TX_FIFO_TRIG_4;
	tegra_i2cslv_writel(i2cslv_dev, reg, I2C_SLV_FIFO_CONTROL);

	/* Configure interrupts */
	tegra_i2cslv_writel(i2cslv_dev, 0, I2C_SL_INT_MASK);
	tegra_i2cslv_writel(i2cslv_dev, 0, I2C_INTERRUPT_MASK_REGISTER);
	tegra_i2cslv_writel(i2cslv_dev, I2C_SL_STATUS_END_TRANS |
			    I2C_SL_INT_RCVD,
			    I2C_SL_INT_MASK);
	tegra_i2cslv_unmask_irq(i2cslv_dev, I2C_INTERRUPT_RX_BUF_FILLED_INT_EN |
				I2C_INTERRUPT_SLV_PKT_XFER_ERR_INT_EN |
				I2C_INTERRUPT_TX_BUFFER_REQ_INT_EN);

	/* Configure Max payload */
	tegra_i2cslv_writel(i2cslv_dev, 0xFFFF, I2C_SLV_PAYLOAD);

	/* Configure CNFG register */
	reg = tegra_i2cslv_readl(i2cslv_dev, I2C_SL_CNFG);
	reg |= (I2C_SL_CNFG_NEW_SL | I2C_SL_CNFG_ENABLE_SL |
		I2C_SL_CNFG_FIFO_XFER_EN | I2C_SL_CNFG_XFER_ERR_CLK_STRETCH_EN |
		I2C_SL_CNFG_ACK_WITHHOLD_EN | I2C_SL_CNFG_ACK_LAST_BYTE);
	tegra_i2cslv_writel(i2cslv_dev, reg, I2C_SL_CNFG);

	i2cslv_dev->tx_in_progress = false;
	i2cslv_dev->rx_in_progress = false;
	i2cslv_dev->failed_xfer = false;

	return tegra_i2cslv_load_config(i2cslv_dev);
}

static void tegra_i2cslv_reset_init(struct work_struct *work)
{
	struct tegra_i2cslv_dev *i2cslv_dev = container_of(work,
							struct tegra_i2cslv_dev, work.work);
	unsigned long flags;

	if (!i2cslv_dev->failed_xfer)
		dev_err(i2cslv_dev->dev, "Slave Xfer Timeout\n");

	raw_spin_lock_irqsave(&i2cslv_dev->xfer_lock, flags);
	tegra_i2cslv_dump_reg(i2cslv_dev);
	raw_spin_unlock_irqrestore(&i2cslv_dev->xfer_lock, flags);

	dev_err(i2cslv_dev->dev, "Slave controller reset\n");

	reset_control_reset(i2cslv_dev->rstc);
	tegra_i2cslv_init(i2cslv_dev);
}

static irqreturn_t tegra_i2cslv_isr(int irq, void *dev_id)
{
	struct tegra_i2cslv_dev *i2cslv_dev = dev_id;
	u32 i2c_int_src;
	u32 i2c_slv_sts;
	unsigned long flags;

	raw_spin_lock_irqsave(&i2cslv_dev->xfer_lock, flags);
	i2c_int_src = tegra_i2cslv_readl(i2cslv_dev,
					 I2C_INTERRUPT_SOURCE_REGISTER);
	i2c_slv_sts = tegra_i2cslv_readl(i2cslv_dev, I2C_SL_STATUS);

	if (i2c_slv_sts & I2C_SL_STATUS_RCVD) {
		if (unlikely(i2cslv_dev->tx_in_progress)) {
			dev_err(i2cslv_dev->dev, "Previous transaction was unsuccessful\n");
			i2cslv_dev->failed_xfer = true;
			schedule_delayed_work(&i2cslv_dev->work, 0);
			goto done;
		}
		schedule_delayed_work(&i2cslv_dev->work, I2C_SLV_TIMEOUT);
		tegra_i2cslv_writel(i2cslv_dev, I2C_SL_STATUS_RCVD,
				I2C_SL_STATUS);
		goto done;
	}

	if (i2c_int_src & I2C_INTERRUPT_SLV_RX_BUFFER_FILLED) {
		tegra_i2cslv_handle_rx(i2cslv_dev, i2c_int_src, i2c_slv_sts);
		goto done;
	}

	if (i2c_int_src & I2C_INTERRUPT_SLV_TX_BUFFER_REQ) {
		tegra_i2cslv_handle_tx(i2cslv_dev, i2c_int_src, i2c_slv_sts);
		goto done;
	}

	if (i2c_int_src & I2C_INTERRUPT_SLV_TFIFO_DATA_REQ) {
		tegra_i2cslv_fill_txfifo(i2cslv_dev);
		goto done;
	}

	if (i2c_int_src & I2C_INTERRUPT_SLV_RFIFO_DATA_REQ) {
		tegra_i2cslv_empty_rxfifo(i2cslv_dev);
		goto done;
	}

	/* STOP: End of transfer */
	if (i2c_slv_sts & I2C_SL_STATUS_END_TRANS) {
		tegra_i2cslv_handle_stop(i2cslv_dev, i2c_int_src, i2c_slv_sts);
		cancel_delayed_work(&i2cslv_dev->work);
		goto done;
	}

	dev_err(i2cslv_dev->dev, "missed irq, int_src=0x%x slv_sts=0x%x\n",
		i2c_int_src, i2c_slv_sts);
	tegra_i2cslv_dump_reg(i2cslv_dev);
	tegra_i2cslv_init(i2cslv_dev);
done:
	raw_spin_unlock_irqrestore(&i2cslv_dev->xfer_lock, flags);
	return IRQ_HANDLED;
}

static int tegra_reg_slave(struct i2c_client *slave)
{
	struct tegra_i2cslv_dev *i2cslv_dev =
		i2c_get_adapdata(slave->adapter);
	int ret;

	if (i2cslv_dev->slave)
		return -EBUSY;

	i2cslv_dev->slave = slave;

	i2cslv_dev->rx_buffer = devm_kzalloc(i2cslv_dev->dev, 0xFF, GFP_KERNEL);
	if (!i2cslv_dev->rx_buffer)
		return -ENOMEM;

	ret = clk_enable(i2cslv_dev->div_clk);
	if (ret < 0) {
		dev_err(i2cslv_dev->dev, "Enable div-clk failed: %d\n", ret);
		return ret;
	}
	ret = clk_set_rate(i2cslv_dev->div_clk, i2cslv_dev->clk_rate);
	if (ret < 0)
		dev_warn(i2cslv_dev->dev, "Unable to set rate: %d\n", ret);

	return tegra_i2cslv_init(i2cslv_dev);
}

static void tegra_i2cslv_deinit(struct tegra_i2cslv_dev *i2cslv_dev)
{
	tegra_i2cslv_writel(i2cslv_dev, 0, I2C_INTERRUPT_MASK_REGISTER);
	tegra_i2cslv_writel(i2cslv_dev, 0, I2C_SL_INT_MASK);
	clk_disable(i2cslv_dev->div_clk);
}

static int tegra_unreg_slave(struct i2c_client *slave)
{
	struct tegra_i2cslv_dev *i2cslv_dev =
		i2c_get_adapdata(slave->adapter);

	WARN_ON(!i2cslv_dev->slave);
	tegra_i2cslv_deinit(i2cslv_dev);

	return 0;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SLAVE;
}

static const struct i2c_algorithm tegra_i2cslv_algo = {
	.functionality  = tegra_i2c_func,
	.reg_slave      = tegra_reg_slave,
	.unreg_slave    = tegra_unreg_slave,
};

static int tegra_i2cslv_probe(struct platform_device *pdev)
{
	struct tegra_i2cslv_dev *i2cslv_dev;
	struct i2c_adapter *adap;
	struct resource *res;
	phys_addr_t phys_addr;
	void __iomem *base;
	struct clk *div_clk, *parent_clk;
	struct reset_control *rstc;
	int irq, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No IO memory resource\n");
		return -ENODEV;
	}
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

	div_clk = devm_clk_get(&pdev->dev, "div-clk");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock");
		return PTR_ERR(div_clk);
	}

	parent_clk = devm_clk_get(&pdev->dev, "parent");
	if (IS_ERR(parent_clk)) {
		dev_err(&pdev->dev, "Unable to get parent_clk err:%ld\n",
				PTR_ERR(parent_clk));
	} else {
		ret = clk_set_parent(div_clk, parent_clk);
		if (ret < 0)
			dev_warn(&pdev->dev, "Couldn't set parent clock : %d\n",
				ret);
	}
	ret = clk_prepare(div_clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "Clock prepare failed %d\n", ret);
		return ret;
	}

	rstc = devm_reset_control_get(&pdev->dev, "i2c");
	if (IS_ERR(rstc)) {
		ret = PTR_ERR(rstc);
		dev_err(&pdev->dev, "Reset control is not found: %d\n", ret);
		return ret;
	}
	/* Reset the controller */
	reset_control_reset(rstc);

	i2cslv_dev = devm_kzalloc(&pdev->dev, sizeof(*i2cslv_dev), GFP_KERNEL);
	if (!i2cslv_dev)
		return -ENOMEM;

	i2cslv_dev->dev = &pdev->dev;
	i2cslv_dev->base = base;
	i2cslv_dev->div_clk = div_clk;
	i2cslv_dev->rstc = rstc;
	raw_spin_lock_init(&i2cslv_dev->xfer_lock);

	ret = of_property_read_u32(i2cslv_dev->dev->of_node, "clock-frequency",
				   &i2cslv_dev->clk_rate);
	if (ret)
		i2cslv_dev->clk_rate = I2C_SLV_CLK_RATE;

	adap = &i2cslv_dev->adap;
	adap->algo = &tegra_i2cslv_algo;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	i2c_set_adapdata(adap, i2cslv_dev);
	platform_set_drvdata(pdev, i2cslv_dev);
	strlcpy(adap->name, pdev->name, sizeof(adap->name));

	ret = devm_request_irq(&pdev->dev, irq, tegra_i2cslv_isr,
			       0, dev_name(&pdev->dev), i2cslv_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register ISR for IRQ %d\n", irq);
		return ret;
	}

	ret = i2c_add_adapter(&i2cslv_dev->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		return ret;
	}

	INIT_DELAYED_WORK(&i2cslv_dev->work, tegra_i2cslv_reset_init);

	return 0;
}

static int tegra_i2cslv_remove(struct platform_device *pdev)
{
	struct tegra_i2cslv_dev *i2cslv_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2cslv_dev->adap);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_i2cslv_suspend(struct device *dev)
{
	struct tegra_i2cslv_dev *i2cslv_dev = dev_get_drvdata(dev);
	unsigned long flags;

	raw_spin_lock_irqsave(&i2cslv_dev->xfer_lock, flags);

	tegra_i2cslv_deinit(i2cslv_dev);

	raw_spin_unlock_irqrestore(&i2cslv_dev->xfer_lock, flags);

	return 0;
}

static int tegra_i2cslv_resume(struct device *dev)
{
	struct tegra_i2cslv_dev *i2cslv_dev = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	raw_spin_lock_irqsave(&i2cslv_dev->xfer_lock, flags);

	ret = clk_enable(i2cslv_dev->div_clk);
	if (ret < 0) {
		dev_err(i2cslv_dev->dev, "Enable div-clk failed: %d\n", ret);
		goto exit;
	}
	ret = tegra_i2cslv_init(i2cslv_dev);
	if (ret)
		goto exit;

exit:
	raw_spin_unlock_irqrestore(&i2cslv_dev->xfer_lock, flags);
	return ret;
}
#endif

static const struct dev_pm_ops tegra_i2cslv_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_i2cslv_suspend, tegra_i2cslv_resume)
};

static const struct of_device_id tegra_i2cslv_of_match[] = {
	{.compatible = "nvidia,tegra194-i2c-slave",},
	{}
};

MODULE_DEVICE_TABLE(of, tegra_i2cslv_of_match);

static struct platform_driver tegra_i2cslv_driver = {
	.probe = tegra_i2cslv_probe,
	.remove = tegra_i2cslv_remove,
	.driver = {
		   .name = "tegra194-i2cslv",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(tegra_i2cslv_of_match),
		   .pm = &tegra_i2cslv_pm_ops,
		   },
};

module_platform_driver(tegra_i2cslv_driver);

MODULE_AUTHOR("Shardar Shariff Md <smohammed@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra194 I2C slave driver");
MODULE_LICENSE("GPL v2");
