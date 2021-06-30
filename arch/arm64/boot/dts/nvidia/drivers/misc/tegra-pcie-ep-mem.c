/*
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
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/debugfs.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/random.h>

#define MODULENAME "tegra_ep_mem"

#define DMA_REG_SIZE (1 * 1024 * 1024)

#define DMA_RD_CHNL_NUM			2
#define DMA_WR_CHNL_NUM			4

/* Common registers */
#define DMA_WRITE_ENGINE_EN_OFF		0xC
#define DMA_WRITE_ENGINE_EN_OFF_ENABLE	BIT(0)

#define DMA_WRITE_DOORBELL_OFF		0x10
#define DMA_WRITE_DOORBELL_OFF_WR_STOP	BIT(31)

#define DMA_READ_ENGINE_EN_OFF		0x2C
#define DMA_READ_ENGINE_EN_OFF_ENABLE	BIT(0)

#define DMA_READ_DOORBELL_OFF		0x30
#define DMA_READ_DOORBELL_OFF_RD_STOP	BIT(31)

#define DMA_WRITE_INT_STATUS_OFF		0x4C
#define DMA_WRITE_INT_MASK_OFF		0x54
#define DMA_WRITE_INT_CLEAR_OFF		0x58

#define DMA_WRITE_DONE_IMWR_LOW_OFF	0x60
#define DMA_WRITE_DONE_IMWR_HIGH_OFF	0x64
#define DMA_WRITE_ABORT_IMWR_LOW_OFF	0x68
#define DMA_WRITE_ABORT_IMWR_HIGH_OFF	0x6C

#define DMA_WRITE_IMWR_DATA_OFF_BASE	0x70

#define DMA_READ_INT_STATUS_OFF		0xA0
#define DMA_READ_INT_MASK_OFF		0xA8
#define DMA_READ_INT_CLEAR_OFF		0xAC

#define DMA_READ_DONE_IMWR_LOW_OFF	0xCC
#define DMA_READ_DONE_IMWR_HIGH_OFF	0xD0
#define DMA_READ_ABORT_IMWR_LOW_OFF	0xD4
#define DMA_READ_ABORT_IMWR_HIGH_OFF	0xD8

#define DMA_READ_IMWR_DATA_OFF_BASE	0xDC

/* Channel specific registers */
#define DMA_CH_CONTROL1_OFF_WRCH		0x0
#define DMA_CH_CONTROL1_OFF_WRCH_LLE	BIT(9)
#define DMA_CH_CONTROL1_OFF_WRCH_RIE	BIT(4)
#define DMA_CH_CONTROL1_OFF_WRCH_LIE	BIT(3)
#define DMA_CH_CONTROL1_OFF_WRCH_LLP	BIT(2)
#define DMA_TRANSFER_SIZE_OFF_WRCH	0x8
#define DMA_SAR_LOW_OFF_WRCH		0xC
#define DMA_SAR_HIGH_OFF_WRCH		0x10
#define DMA_DAR_LOW_OFF_WRCH		0x14
#define DMA_DAR_HIGH_OFF_WRCH		0x18
#define DMA_LLP_LOW_OFF_WRCH		0x1C
#define DMA_LLP_HIGH_OFF_WRCH		0x20

#define DMA_CH_CONTROL1_OFF_RDCH		(0x0 + 0x100)
#define DMA_CH_CONTROL1_OFF_RDCH_LLE	BIT(9)
#define DMA_CH_CONTROL1_OFF_RDCH_RIE	BIT(4)
#define DMA_CH_CONTROL1_OFF_RDCH_LIE	BIT(3)
#define DMA_CH_CONTROL1_OFF_RDCH_LLP	BIT(2)
#define DMA_TRANSFER_SIZE_OFF_RDCH	(0x8 + 0x100)
#define DMA_SAR_LOW_OFF_RDCH		(0xC + 0x100)
#define DMA_SAR_HIGH_OFF_RDCH		(0x10 + 0x100)
#define DMA_DAR_LOW_OFF_RDCH		(0x14 + 0x100)
#define DMA_DAR_HIGH_OFF_RDCH		(0x18 + 0x100)
#define DMA_LLP_LOW_OFF_RDCH		(0x1C + 0x100)
#define DMA_LLP_HIGH_OFF_RDCH		(0x20 + 0x100)

static unsigned long alloc_size = 0xA00000;

module_param(alloc_size, ulong, 0660);
MODULE_PARM_DESC(alloc_size, "Allocation Size");

static dma_addr_t dma_addr;
static void *p_cpu_addr;

struct ep_pvt {
	struct pci_dev *pdev;
	void __iomem *mmio_addr;	/* memory map DMA addresses */
	struct dentry *debugfs;
	u64 src;
	u64 dst;
	u32 size;
	u8 channel;
	struct mutex wr_lock[DMA_WR_CHNL_NUM];	/* write lock */
	struct mutex rd_lock[DMA_RD_CHNL_NUM];	/* read lock */
	struct completion wr_cpl[DMA_WR_CHNL_NUM];
	struct completion rd_cpl[DMA_RD_CHNL_NUM];
	ktime_t wr_start_time;
	ktime_t wr_end_time;
	ktime_t rd_start_time;
	ktime_t rd_end_time;
	unsigned long wr_busy;
	unsigned long rd_busy;
};

struct dma_tx {
	u64 src;
	u64 dst;
	u32 size;
	u8 channel;
	bool ll;
};

struct dma_ll_element_1 {
	u32 cb:1;
	u32 tcb:1;
	u32 llp:1;
	u32 lie:1;
	u32 rie:1;
};

struct dma_ll {
	struct dma_ll_element_1 ele_1;
	u32 size;
	u32 sar_low;
	u32 sar_high;
	u32 dar_low;
	u32 dar_high;
};

static inline void dma_common_wr16(void __iomem *p, u32 val, u32 offset)
{
	writew(val, offset + p);
}

static inline u16 dma_common_rd16(void __iomem *p, u32 offset)
{
	return readw(offset + p);
}

static inline void dma_common_wr(void __iomem *p, u32 val, u32 offset)
{
	writel(val, offset + p);
}

static inline u32 dma_common_rd(void __iomem *p, u32 offset)
{
	return readl(offset + p);
}

static inline void dma_channel_wr(void __iomem *p, u8 channel, u32 val,
				  u32 offset)
{
	writel(val, (0x200 * (channel + 1)) + offset + p);
}

static inline u32 dma_channel_rd(void __iomem *p, u8 channel, u32 offset)
{
	return readl((0x200 * (channel + 1)) + offset + p);
}

static irqreturn_t ep_isr(int irq, struct ep_pvt *ep)
{
	u32 val = 0, bit = 0;
	int handled = 0;

	handled = 1;

	dev_dbg(&ep->pdev->dev, "ISR called\n");

	/* in ISR, acquire lock for busy-data */
	val = dma_common_rd(ep->mmio_addr, DMA_WRITE_INT_STATUS_OFF);
	/* check the status of all busy marked channels */
	for_each_set_bit(bit, &ep->wr_busy, DMA_WR_CHNL_NUM) {
		dev_dbg(&ep->pdev->dev, "bit = %u\n", bit);
		if (BIT(bit) & val) {
			dma_common_wr(ep->mmio_addr, BIT(bit),
				      DMA_WRITE_INT_CLEAR_OFF);
			/* send completion for appropriate channel */
			complete(&ep->wr_cpl[bit]);
			/* clear status */
			ep->wr_busy &= ~(BIT(bit));
		}
	}

	val = dma_common_rd(ep->mmio_addr, DMA_READ_INT_STATUS_OFF);
	/* check the status of all busy marked channels */
	for_each_set_bit(bit, &ep->rd_busy, DMA_RD_CHNL_NUM) {
		dev_dbg(&ep->pdev->dev, "bit = %u\n", bit);
		if (BIT(bit) & val) {
			dma_common_wr(ep->mmio_addr, BIT(bit),
				      DMA_READ_INT_CLEAR_OFF);
			/* send completion for appropriate channel */
			complete(&ep->rd_cpl[bit]);
			/* clear status */
			ep->rd_busy &= ~(BIT(bit));
		}
	}

	return IRQ_RETVAL(handled);
}

static int dma_write(struct ep_pvt *ep, struct dma_tx *tx)
{
	struct device *dev = &ep->pdev->dev;
	u32 val = 0;
	u16 val_16 = 0;
	int ret = 0;

	if (tx->channel > 3) {
		dev_err(dev, "Invalid Channel. Should be with in [0~3]\n");
		return -EINVAL;
	}

	/* acquire lock for channel HW */
	mutex_lock(&ep->wr_lock[tx->channel]);

	/* Enable Write Engine */
	dma_common_wr(ep->mmio_addr, DMA_WRITE_ENGINE_EN_OFF_ENABLE,
		      DMA_WRITE_ENGINE_EN_OFF);

	/* Mask DONE and ABORT interrupts */
	val = dma_common_rd(ep->mmio_addr, DMA_WRITE_INT_MASK_OFF);
	val |= 1 << tx->channel;		/* DONE */
	val |= 1 << ((tx->channel) + 16);	/* ABORT */
	dma_common_wr(ep->mmio_addr, val, DMA_WRITE_INT_MASK_OFF);

	pci_read_config_dword(ep->pdev, ep->pdev->msi_cap + PCI_MSI_ADDRESS_LO,
			      &val);
	dma_common_wr(ep->mmio_addr, val, DMA_WRITE_DONE_IMWR_LOW_OFF);
	dma_common_wr(ep->mmio_addr, val, DMA_WRITE_ABORT_IMWR_LOW_OFF);

	pci_read_config_dword(ep->pdev, ep->pdev->msi_cap + PCI_MSI_ADDRESS_HI,
			      &val);
	dma_common_wr(ep->mmio_addr, val, DMA_WRITE_DONE_IMWR_HIGH_OFF);
	dma_common_wr(ep->mmio_addr, val, DMA_WRITE_ABORT_IMWR_HIGH_OFF);

	pci_read_config_word(ep->pdev, ep->pdev->msi_cap + PCI_MSI_DATA_64,
			     &val_16);
	dma_common_wr16(ep->mmio_addr, val_16,
			DMA_WRITE_IMWR_DATA_OFF_BASE + (2 * tx->channel));

	val = dma_channel_rd(ep->mmio_addr, tx->channel,
			     DMA_CH_CONTROL1_OFF_WRCH);
	if (tx->ll) {
		val = DMA_CH_CONTROL1_OFF_WRCH_LLE;
	} else {
		val = DMA_CH_CONTROL1_OFF_WRCH_RIE |
		      DMA_CH_CONTROL1_OFF_WRCH_LIE;
	}
	dma_channel_wr(ep->mmio_addr, tx->channel, val,
		       DMA_CH_CONTROL1_OFF_WRCH);

	if (tx->ll) {
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_LLP_LOW_OFF_WRCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_LLP_HIGH_OFF_WRCH);
	} else {
		dma_channel_wr(ep->mmio_addr, tx->channel, tx->size,
			       DMA_TRANSFER_SIZE_OFF_WRCH);

		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_SAR_LOW_OFF_WRCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_SAR_HIGH_OFF_WRCH);

		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->dst & 0xFFFFFFFF),
			       DMA_DAR_LOW_OFF_WRCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->dst >> 32) & 0xFFFFFFFF),
			       DMA_DAR_HIGH_OFF_WRCH);
	}
	/* acquire lock for busy-data and mark it as busy and then release */
	ep->wr_busy |= 1 << tx->channel;

	ep->wr_start_time = ktime_get();
	/* start DMA (ring the door bell) */
	/* ring the door bell with channel number */
	dma_common_wr(ep->mmio_addr, ep->channel, DMA_WRITE_DOORBELL_OFF);

	/* wait for completion or timeout */
	ret = wait_for_completion_timeout(&ep->wr_cpl[tx->channel],
					  msecs_to_jiffies(5000));
	ep->wr_end_time = ktime_get();
	if (ret == 0) {
		dev_err(dev, "DMA write operation timed out no interrupt\n");
		ret = -ETIMEDOUT;
		/* if timeout, clear the mess, sanitize channel */
		dma_common_wr(ep->mmio_addr, DMA_WRITE_DOORBELL_OFF_WR_STOP |
			      ep->channel, DMA_WRITE_DOORBELL_OFF);
		goto exit;
	}
	dev_info(dev, "DMA write: Size: %u bytes, Time diff: %lld ns\n",
		 tx->size,
		 ktime_to_ns(ep->wr_end_time) - ktime_to_ns(ep->wr_start_time));

exit:
	mutex_unlock(&ep->wr_lock[tx->channel]);
	return ret;
}

static int dma_read(struct ep_pvt *ep, struct dma_tx *tx)
{
	struct device *dev = &ep->pdev->dev;
	u32 val = 0;
	u16 val_16 = 0;
	int ret = 0;

	if (tx->channel > 1) {
		dev_err(dev, "Invalid Channel. Should be with in [0~1]\n");
		return -EINVAL;
	}

	/* acquire lock for channel HW */
	mutex_lock(&ep->rd_lock[tx->channel]);

	/* program registers */
	/* Enable Read Engine */
	dma_common_wr(ep->mmio_addr, DMA_READ_ENGINE_EN_OFF_ENABLE,
		      DMA_READ_ENGINE_EN_OFF);

	/* Mask DONE and ABORT interrupts */
	val = dma_common_rd(ep->mmio_addr, DMA_READ_INT_MASK_OFF);
	val |= 1 << tx->channel;		/* DONE */
	val |= 1 << ((tx->channel) + 16);	/* ABORT */
	dma_common_wr(ep->mmio_addr, val, DMA_READ_INT_MASK_OFF);

	pci_read_config_dword(ep->pdev, ep->pdev->msi_cap + PCI_MSI_ADDRESS_LO,
			      &val);
	dma_common_wr(ep->mmio_addr, val, DMA_READ_DONE_IMWR_LOW_OFF);
	dma_common_wr(ep->mmio_addr, val, DMA_READ_ABORT_IMWR_LOW_OFF);

	pci_read_config_dword(ep->pdev, ep->pdev->msi_cap + PCI_MSI_ADDRESS_HI,
			      &val);
	dma_common_wr(ep->mmio_addr, val, DMA_READ_DONE_IMWR_HIGH_OFF);
	dma_common_wr(ep->mmio_addr, val, DMA_READ_ABORT_IMWR_HIGH_OFF);

	pci_read_config_word(ep->pdev, ep->pdev->msi_cap + PCI_MSI_DATA_64,
			     &val_16);
	dma_common_wr16(ep->mmio_addr, val_16,
			DMA_READ_IMWR_DATA_OFF_BASE + (2 * tx->channel));

	val = dma_channel_rd(ep->mmio_addr, tx->channel,
			     DMA_CH_CONTROL1_OFF_RDCH);
	if (tx->ll) {
		val = DMA_CH_CONTROL1_OFF_RDCH_LLE;
	} else {
		val = DMA_CH_CONTROL1_OFF_RDCH_RIE |
		      DMA_CH_CONTROL1_OFF_RDCH_LIE;
	}
	dma_channel_wr(ep->mmio_addr, tx->channel, val,
		       DMA_CH_CONTROL1_OFF_RDCH);

	if (tx->ll) {
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_LLP_LOW_OFF_RDCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_LLP_HIGH_OFF_RDCH);
	} else {
		dma_channel_wr(ep->mmio_addr, tx->channel, tx->size,
			       DMA_TRANSFER_SIZE_OFF_RDCH);

		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->src & 0xFFFFFFFF),
			       DMA_SAR_LOW_OFF_RDCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->src >> 32) & 0xFFFFFFFF),
			       DMA_SAR_HIGH_OFF_RDCH);

		dma_channel_wr(ep->mmio_addr, tx->channel,
			       (tx->dst & 0xFFFFFFFF),
			       DMA_DAR_LOW_OFF_RDCH);
		dma_channel_wr(ep->mmio_addr, tx->channel,
			       ((tx->dst >> 32) & 0xFFFFFFFF),
			       DMA_DAR_HIGH_OFF_RDCH);
	}

	/* acquire lock for busy-data and mark it as busy and then release */
	ep->rd_busy |= 1 << tx->channel;

	ep->rd_start_time = ktime_get();
	/* start DMA (ring the door bell) */
	/* ring the door bell with channel number */
	dma_common_wr(ep->mmio_addr, ep->channel, DMA_READ_DOORBELL_OFF);

	/* wait for completion or timeout */
	ret = wait_for_completion_timeout(&ep->rd_cpl[tx->channel],
					  msecs_to_jiffies(5000));
	ep->rd_end_time = ktime_get();
	if (ret == 0) {
		dev_err(dev, "DMA read operation timed out no interrupt\n");
		ret = -ETIMEDOUT;
		/* if timeout, clear the mess, sanitize channel */
		dma_common_wr(ep->mmio_addr, DMA_READ_DOORBELL_OFF_RD_STOP |
			      ep->channel, DMA_READ_DOORBELL_OFF);
		goto exit;
	}
	dev_info(dev, "DMA read: Size: %u bytes, Time diff: %lld ns",
		 tx->size,
		 ktime_to_ns(ep->rd_end_time) - ktime_to_ns(ep->rd_start_time));

exit:
	mutex_unlock(&ep->rd_lock[tx->channel]);
	return ret;
}

static int write(struct seq_file *s, void *data)
{
	struct ep_pvt *ep = (struct ep_pvt *)(s->private);
	struct dma_tx tx;
	void __iomem *bar_mem;
	int ret = 0;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->src;
	tx.dst = ep->dst;
	tx.size = ep->size;
	tx.channel = ep->channel;

	/* fill source with random data */
	bar_mem = pci_ioremap_bar(ep->pdev, 0);
	if (!bar_mem) {
		dev_err(&ep->pdev->dev, "Cannot map BAR0, aborting\n");
		ret = -ENOMEM;
		goto err_remap;
	}
	get_random_bytes(__io_virt(bar_mem), ep->size);

	ret = dma_write(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev, "something is wrong in write\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (!memcmp(__io_virt(bar_mem), phys_to_virt(ep->dst), ep->size))
		dev_info(&ep->pdev->dev, "DMA-Write test PASSED\n");
	else
		dev_info(&ep->pdev->dev, "DMA-Write test FAILED\n");
err_out:
	iounmap(bar_mem);
err_remap:
	return ret;
}

static int write_ll(struct seq_file *s, void *data)
{
	struct ep_pvt *ep = (struct ep_pvt *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	void __iomem *bar_mem;
	struct dma_ll *ll;

	/* fill source with random data */
	bar_mem = pci_ioremap_bar(ep->pdev, 0);
	if (!bar_mem) {
		dev_err(&ep->pdev->dev, "Cannot map BAR0, aborting\n");
		ret = -ENOMEM;
		goto err_remap;
	}

	/* create linked list to be sent to ep's local memory */
	ll = (struct dma_ll *)(phys_to_virt(ep->dst));

	/* leave first 64K for LL element preparation */
	memset((ll + 0), 0x0, sizeof(struct dma_ll));
	(ll + 0)->size = (64 * 1024);
	(ll + 0)->sar_low = ep->src + (64 * 1024 * 1);
	(ll + 0)->dar_low = ep->dst + (64 * 1024 * 1);
	get_random_bytes(__io_virt(bar_mem) + (64 * 1024 * 1), 64 * 1024);

	memset((ll + 1), 0x0, sizeof(struct dma_ll));
	(ll + 1)->size = (64 * 1024);
	(ll + 1)->sar_low = ep->src + (64 * 1024 * 2);
	(ll + 1)->dar_low = ep->dst + (64 * 1024 * 2);
	get_random_bytes(__io_virt(bar_mem) + (64 * 1024 * 2), 64 * 1024);

	memset((ll + 2), 0x0, sizeof(struct dma_ll));
	(ll + 2)->ele_1.llp = 1;
	(ll + 2)->sar_low = (4 * sizeof(struct dma_ll)) + ep->src;

	memset((ll + 4), 0x0, sizeof(struct dma_ll));
	(ll + 4)->ele_1.rie = 1;
	(ll + 4)->ele_1.lie = 1;
	(ll + 4)->size = (64 * 1024);
	(ll + 4)->sar_low = ep->src + (64 * 1024 * 4);
	(ll + 4)->dar_low = ep->dst + (64 * 1024 * 4);
	get_random_bytes(__io_virt(bar_mem) + (64 * 1024 * 4), 64 * 1024);

	memset((ll + 5), 0x0, sizeof(struct dma_ll));
	(ll + 5)->ele_1.llp = 1;
	(ll + 5)->ele_1.tcb = 1;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->dst;
	tx.dst = ep->src;
	tx.size = 6 * sizeof(struct dma_ll);
	tx.channel = ep->channel;
	ret = dma_read(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev,
			"something is wrong in write-ll's initial read\n");
		ret = -EIO;
		goto err_out;
	}

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->src;
	tx.channel = ep->channel;
	tx.ll = 1;
	ret = dma_write(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev, "something is wrong in write-ll\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 1)),
		   phys_to_virt(ep->dst + (64 * 1024 * 1)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Write-LL Chunk-1 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 2)),
		   phys_to_virt(ep->dst + (64 * 1024 * 2)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Write-LL Chunk-2 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 4)),
		   phys_to_virt(ep->dst + (64 * 1024 * 4)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Write-LL Chunk-3 FAILED\n");
		goto err_out;
	}
	dev_info(&ep->pdev->dev, "DMA-Write-LL PASSED\n");

err_out:
	iounmap(bar_mem);
err_remap:
	return ret;
}

static int read(struct seq_file *s, void *data)
{
	struct ep_pvt *ep = (struct ep_pvt *)(s->private);
	struct dma_tx tx;
	void __iomem *bar_mem;
	int ret = 0;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->src;
	tx.dst = ep->dst;
	tx.size = ep->size;
	tx.channel = ep->channel;

	/* fill source with random data */
	bar_mem = pci_ioremap_bar(ep->pdev, 0);
	if (!bar_mem) {
		dev_err(&ep->pdev->dev, "Cannot map BAR0, aborting\n");
		ret = -ENOMEM;
		goto err_remap;
	}
	get_random_bytes(phys_to_virt(ep->src), ep->size);

	ret = dma_read(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev, "something is wrong in read\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (!memcmp(__io_virt(bar_mem), phys_to_virt(ep->src), ep->size))
		dev_info(&ep->pdev->dev, "DMA-Read test PASSED\n");
	else
		dev_info(&ep->pdev->dev, "DMA-Read test FAILED\n");

err_out:
	iounmap(bar_mem);
err_remap:
	return ret;
}

static int read_ll(struct seq_file *s, void *data)
{
	struct ep_pvt *ep = (struct ep_pvt *)(s->private);
	struct dma_tx tx;
	int ret = 0;
	void __iomem *bar_mem;
	struct dma_ll *ll;

	/* fill source with random data */
	bar_mem = pci_ioremap_bar(ep->pdev, 0);
	if (!bar_mem) {
		dev_err(&ep->pdev->dev, "Cannot map BAR0, aborting\n");
		ret = -ENOMEM;
		goto err_remap;
	}

	/* create linked list to be sent to ep's local memory */
	ll = (struct dma_ll *)(phys_to_virt(ep->src));

	/* leave first 64K for LL element preparation */
	memset((ll + 0), 0x0, sizeof(struct dma_ll));
	(ll + 0)->size = (64 * 1024);
	(ll + 0)->sar_low = ep->src + (64 * 1024 * 1);
	(ll + 0)->dar_low = ep->dst + (64 * 1024 * 1);
	get_random_bytes(phys_to_virt(ep->src + (64 * 1024 * 1)), 64 * 1024);

	memset((ll + 1), 0x0, sizeof(struct dma_ll));
	(ll + 1)->size = (64 * 1024);
	(ll + 1)->sar_low = ep->src + (64 * 1024 * 2);
	(ll + 1)->dar_low = ep->dst + (64 * 1024 * 2);
	get_random_bytes(phys_to_virt(ep->src + (64 * 1024 * 2)), 64 * 1024);

	memset((ll + 2), 0x0, sizeof(struct dma_ll));
	(ll + 2)->ele_1.llp = 1;
	(ll + 2)->sar_low = (4 * sizeof(struct dma_ll)) + ep->dst;

	memset((ll + 4), 0x0, sizeof(struct dma_ll));
	(ll + 4)->ele_1.rie = 1;
	(ll + 4)->ele_1.lie = 1;
	(ll + 4)->size = (64 * 1024);
	(ll + 4)->sar_low = ep->src + (64 * 1024 * 4);
	(ll + 4)->dar_low = ep->dst + (64 * 1024 * 4);
	get_random_bytes(phys_to_virt(ep->src + (64 * 1024 * 4)), 64 * 1024);

	memset((ll + 5), 0x0, sizeof(struct dma_ll));
	(ll + 5)->ele_1.llp = 1;
	(ll + 5)->ele_1.tcb = 1;

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->src;
	tx.dst = ep->dst;
	tx.size = 6 * sizeof(struct dma_ll);
	tx.channel = ep->channel;
	ret = dma_read(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev,
			"something is wrong in read-ll's initial read\n");
		ret = -EIO;
		goto err_out;
	}

	memset(&tx, 0x0, sizeof(struct dma_tx));
	tx.src = ep->dst;
	tx.channel = ep->channel;
	tx.ll = 1;
	ret = dma_read(ep, &tx);
	if (ret < 0) {
		dev_err(&ep->pdev->dev, "something is wrong in read-ll\n");
		ret = -EIO;
		goto err_out;
	}

	/* compare copied data */
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 1)),
		   phys_to_virt(ep->src + (64 * 1024 * 1)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Read-LL Chunk-1 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 2)),
		   phys_to_virt(ep->src + (64 * 1024 * 2)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Read-LL Chunk-2 FAILED\n");
		goto err_out;
	}
	if (memcmp((void *)(__io_virt(bar_mem) + (64 * 1024 * 4)),
		   phys_to_virt(ep->src + (64 * 1024 * 4)),
		   64 * 1024)) {
		dev_err(&ep->pdev->dev, "DMA-Read-LL Chunk-3 FAILED\n");
		goto err_out;
	}
	dev_info(&ep->pdev->dev, "DMA-Read-LL PASSED\n");

err_out:
	iounmap(bar_mem);
err_remap:
	return ret;
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

/* common */
DEFINE_ENTRY(write);
DEFINE_ENTRY(write_ll);
DEFINE_ENTRY(read);
DEFINE_ENTRY(read_ll);

static int init_debugfs(struct ep_pvt *ep)
{
	struct dentry *d;

	d = debugfs_create_x64("src", 0644, ep->debugfs,
			       &ep->src);
	if (!d)
		pr_err("debugfs for src addr failed\n");

	d = debugfs_create_x64("dst", 0644, ep->debugfs,
			       &ep->dst);
	if (!d)
		pr_err("debugfs for dst addr failed\n");

	d = debugfs_create_x32("size", 0644, ep->debugfs,
			       &ep->size);
	if (!d)
		pr_err("debugfs for size failed\n");

	d = debugfs_create_x8("channel", 0644, ep->debugfs,
			      &ep->channel);
	if (!d)
		pr_err("debugfs for channel failed\n");

	d = debugfs_create_file("write", 0444, ep->debugfs, (void *)ep,
				&write_fops);
	if (!d)
		pr_err("debugfs for write failed\n");

	d = debugfs_create_file("write_ll", 0444, ep->debugfs, (void *)ep,
				&write_ll_fops);
	if (!d)
		pr_err("debugfs for write failed\n");

	d = debugfs_create_file("read", 0444, ep->debugfs, (void *)ep,
				&read_fops);
	if (!d)
		pr_err("debugfs for read failed\n");

	d = debugfs_create_file("read_ll", 0444, ep->debugfs, (void *)ep,
				&read_ll_fops);
	if (!d)
		pr_err("debugfs for read failed\n");
	return 0;
}

static int ep_test_dma_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct ep_pvt *ep;
	void __iomem *ioaddr;
	int ret = 0, i = 0;
	u16 val16 = 0;

	ep = devm_kzalloc(&pdev->dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pdev = pdev;
	pci_set_drvdata(pdev, ep);

	ret = pci_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "enabling device failed\n");
		goto fail_enable;
	}
	dev_dbg(&pdev->dev, "device is enabled\n");

	/* Set bus-master bit enable for PCIe */
	pci_set_master(pdev);
	dev_dbg(&pdev->dev, "device Bus-Master is enabled\n");

	/* Request for resources */
	ret = pci_request_regions(pdev, MODULENAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "region request failed\n");
		goto fail_region_request;
	}

	/* ioremap MMIO region */
	ioaddr = ioremap(pci_resource_start(pdev, 4), DMA_REG_SIZE);
	if (!ioaddr) {
		dev_err(&pdev->dev, "region remap failed\n");
		ret = -EFAULT;
		goto fail_region_remap;
	}
	ep->mmio_addr = ioaddr;

	/* Allocate and initialize shared control data */
	dev_dbg(&pdev->dev, "Allocation size requested = 0x%lX\n", alloc_size);
	p_cpu_addr = dma_alloc_coherent(&pdev->dev, alloc_size, &dma_addr,
					GFP_KERNEL);
	if (!p_cpu_addr) {
		dev_err(&pdev->dev, "memory allocation failed...!\n");
		ret = -ENOMEM;
		goto fail_dma_alloc;
	}
	dev_info(&pdev->dev,
		 "Allocated memory for DMA operation @ 0x%llX, size=0x%lX\n",
		 dma_addr, alloc_size);

	/* Register IRQ handler (request_irq()) */
	if (pci_enable_msi(pdev) < 0) {
		dev_err(&pdev->dev, "enabling MSI interrupt failed\n");
		ret = -ENODEV;
		goto fail_msi_enable;
	}
	ret = request_irq(pdev->irq, (irq_handler_t)ep_isr, IRQF_SHARED,
			  "pcie_ep_isr", ep);
	if (ret < 0) {
		dev_err(&pdev->dev, "registering isr failed\n");
		goto fail_isr;
	}

	/* Enable DMA/processing engines. */
	for (i = 0; i < DMA_WR_CHNL_NUM; i++) {
		mutex_init(&ep->wr_lock[i]);
		init_completion(&ep->wr_cpl[i]);
	}

	for (i = 0; i < DMA_RD_CHNL_NUM; i++) {
		mutex_init(&ep->rd_lock[i]);
		init_completion(&ep->rd_cpl[i]);
	}

	pci_read_config_word(pdev, pdev->msi_cap + PCI_MSI_FLAGS, &val16);
	if (!(val16 & PCI_MSI_FLAGS_ENABLE))
		dev_warn(&pdev->dev, "MSI interrupts are not enabled\n");

	ep->debugfs = debugfs_create_dir("tegra_pcie_ep", NULL);
	if (!ep->debugfs)
		dev_err(&pdev->dev, "debugfs creation failed\n");
	else
		init_debugfs(ep);

	return ret;

fail_isr:
	pci_disable_msi(pdev);
fail_msi_enable:
	dma_free_coherent(&pdev->dev, alloc_size, p_cpu_addr, dma_addr);
fail_dma_alloc:
	iounmap(ep->mmio_addr);
fail_region_remap:
	pci_release_regions(pdev);
fail_region_request:
	pci_clear_master(pdev);
fail_enable:
	devm_kfree(&pdev->dev, ep);
	return ret;
}

static void ep_test_dma_remove(struct pci_dev *pdev)
{
	struct ep_pvt *ep = pci_get_drvdata(pdev);

	debugfs_remove_recursive(ep->debugfs);
	free_irq(pdev->irq, ep);
	pci_disable_msi(pdev);
	dma_free_coherent(&pdev->dev, alloc_size, p_cpu_addr, dma_addr);
	iounmap(ep->mmio_addr);
	pci_release_regions(pdev);
	pci_clear_master(pdev);
	devm_kfree(&pdev->dev, ep);
}

static const struct pci_device_id ep_pci_tbl[] = {
	{ PCI_DEVICE(0x10DE, 0x1AD4)},
	{ PCI_DEVICE(0x10DE, 0x1AD5)},
	{},
};

MODULE_DEVICE_TABLE(pci, ep_pci_tbl);

static struct pci_driver ep_pci_driver = {
	.name		= MODULENAME,
	.id_table	= ep_pci_tbl,
	.probe		= ep_test_dma_probe,
	.remove		= ep_test_dma_remove,
};

/* Driver Entry Point */
module_pci_driver(ep_pci_driver);

MODULE_DESCRIPTION("Tegra PCIe end point mode client driver");
MODULE_LICENSE("GPL");
