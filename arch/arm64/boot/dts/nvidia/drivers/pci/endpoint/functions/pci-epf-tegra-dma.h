/*
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef PCIE_EPF_TEGRA_DMA_H
#define PCIE_EPF_TEGRA_DMA_H

#define DMA_RD_CHNL_NUM			2
#define DMA_WR_CHNL_NUM			4

/* Common registers */
#define DMA_WRITE_ENGINE_EN_OFF		0xC
#define DMA_WRITE_ENGINE_EN_OFF_ENABLE	BIT(0)
#define DMA_WRITE_ENGINE_EN_OFF_DISABLE	0

#define DMA_WRITE_DOORBELL_OFF		0x10
#define DMA_WRITE_DOORBELL_OFF_WR_STOP	BIT(31)

#define DMA_READ_ENGINE_EN_OFF		0x2C
#define DMA_READ_ENGINE_EN_OFF_ENABLE	BIT(0)
#define DMA_READ_ENGINE_EN_OFF_DISABLE	0

#define DMA_READ_DOORBELL_OFF		0x30
#define DMA_READ_DOORBELL_OFF_RD_STOP	BIT(31)

#define DMA_WRITE_INT_STATUS_OFF	0x4C
#define DMA_WRITE_INT_MASK_OFF		0x54
#define DMA_WRITE_INT_CLEAR_OFF		0x58

#define DMA_WRITE_DONE_IMWR_LOW_OFF	0x60
#define DMA_WRITE_DONE_IMWR_HIGH_OFF	0x64
#define DMA_WRITE_ABORT_IMWR_LOW_OFF	0x68
#define DMA_WRITE_ABORT_IMWR_HIGH_OFF	0x6C

#define DMA_WRITE_IMWR_DATA_OFF_BASE	0x70

#define DMA_WRITE_LINKED_LIST_ERR_EN_OFF	0x90
#define DMA_READ_INT_STATUS_OFF		0xA0
#define DMA_READ_INT_MASK_OFF		0xA8
#define DMA_READ_INT_CLEAR_OFF		0xAC

#define DMA_READ_LINKED_LIST_ERR_EN_OFF	0xC4
#define DMA_READ_DONE_IMWR_LOW_OFF	0xCC
#define DMA_READ_DONE_IMWR_HIGH_OFF	0xD0
#define DMA_READ_ABORT_IMWR_LOW_OFF	0xD4
#define DMA_READ_ABORT_IMWR_HIGH_OFF	0xD8

#define DMA_READ_IMWR_DATA_OFF_BASE	0xDC

/* Channel specific registers */
#define DMA_CH_CONTROL1_OFF_WRCH		0x0
#define DMA_CH_CONTROL1_OFF_WRCH_LLE		BIT(9)
#define DMA_CH_CONTROL1_OFF_WRCH_CCS		BIT(8)
#define DMA_CH_CONTROL1_OFF_WRCH_CS_MASK	GENMASK(6, 5)
#define DMA_CH_CONTROL1_OFF_WRCH_CS_SHIFT	5
#define DMA_CH_CONTROL1_OFF_WRCH_RIE		BIT(4)
#define DMA_CH_CONTROL1_OFF_WRCH_LIE		BIT(3)
#define DMA_CH_CONTROL1_OFF_WRCH_LLP		BIT(2)
#define DMA_CH_CONTROL1_OFF_WRCH_CB		BIT(0)

#define DMA_TRANSFER_SIZE_OFF_WRCH		0x8
#define DMA_SAR_LOW_OFF_WRCH			0xC
#define DMA_SAR_HIGH_OFF_WRCH			0x10
#define DMA_DAR_LOW_OFF_WRCH			0x14
#define DMA_DAR_HIGH_OFF_WRCH			0x18
#define DMA_LLP_LOW_OFF_WRCH			0x1C
#define DMA_LLP_HIGH_OFF_WRCH			0x20

#define DMA_CH_CONTROL1_OFF_RDCH		0x100
#define DMA_CH_CONTROL1_OFF_RDCH_LLE		BIT(9)
#define DMA_CH_CONTROL1_OFF_RDCH_CCS		BIT(8)
#define DMA_CH_CONTROL1_OFF_RDCH_CS_MASK	GENMASK(6, 5)
#define DMA_CH_CONTROL1_OFF_RDCH_CS_SHIFT	5
#define DMA_CH_CONTROL1_OFF_RDCH_RIE		BIT(4)
#define DMA_CH_CONTROL1_OFF_RDCH_LIE		BIT(3)
#define DMA_CH_CONTROL1_OFF_RDCH_LLP		BIT(2)
#define DMA_CH_CONTROL1_OFF_RDCH_CB		BIT(0)

#define DMA_TRANSFER_SIZE_OFF_RDCH		0x108
#define DMA_SAR_LOW_OFF_RDCH			0x10c
#define DMA_SAR_HIGH_OFF_RDCH			0x110
#define DMA_DAR_LOW_OFF_RDCH			0x114
#define DMA_DAR_HIGH_OFF_RDCH			0x118
#define DMA_LLP_LOW_OFF_RDCH			0x11c
#define DMA_LLP_HIGH_OFF_RDCH			0x120

static inline void dma_common_wr(void __iomem *p, u32 val, u32 offset)
{
	writel(val, p + offset);
}

static inline void dma_common_wr16(void __iomem *p, u16 val, u32 offset)
{
	writew(val, p + offset);
}

static inline void dma_common_wr8(void __iomem *p, u16 val, u32 offset)
{
	writeb(val, p + offset);
}

static inline u32 dma_common_rd(void __iomem *p, u32 offset)
{
	return readl(p + offset);
}

static inline void dma_channel_wr(void __iomem *p, u8 channel, u32 val,
				  u32 offset)
{
	writel(val, (0x200 * (channel + 1)) + p + offset);
}

static inline u32 dma_channel_rd(void __iomem *p, u8 channel, u32 offset)
{
	return readl((0x200 * (channel + 1)) + p + offset);
}

struct tvnet_dma_ctrl {
	u32 cb:1;
	u32 tcb:1;
	u32 llp:1;
	u32 lie:1;
	u32 rie:1;
};

struct tvnet_dma_desc {
	union {
		struct tvnet_dma_ctrl ctrl_e;
		u32 ctrl_d;
	} ctrl_reg;
	u32 size;
	u32 sar_low;
	u32 sar_high;
	u32 dar_low;
	u32 dar_high;
};

#endif
