/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#define ENABLE_DMA 1

#define DMA_WR_DATA_CH 0
#define DMA_RD_DATA_CH 0

/* Network link timeout 5 sec */
#define LINK_TIMEOUT 5000

#define TVNET_DEFAULT_MTU 64512
#define TVNET_MIN_MTU 68
#define TVNET_MAX_MTU TVNET_DEFAULT_MTU

#define TVNET_NAPI_WEIGHT	64

#define RING_COUNT 256

/* Allocate 100% extra desc to handle the drift between empty & full buffer */
#define DMA_DESC_COUNT (2 * RING_COUNT)


/* DMA base offset starts at 0x20000 from ATU_DMA base */
#define DMA_OFFSET 0x20000

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
	volatile union {
		struct tvnet_dma_ctrl ctrl_e;
		u32 ctrl_d;
	} ctrl_reg;
	u32 size;
	u32 sar_low;
	u32 sar_high;
	u32 dar_low;
	u32 dar_high;
};

enum irq_type {
	/* No IRQ available in this slot */
	IRQ_NOT_AVAILABLE = 0,
	/* Use irq_{addr,val} fields */
	IRQ_SIMPLE = 1,
	/* Perform a dummy DMA reading */
	IRQ_DUMMY_DMA = 2,
};

struct irq_md {
	u32 irq_type;
	/* Simple method: Write to this */
	/* Dummy DMA method: Read from this */
	u64 irq_addr;
	/* Simple method: Write this value */
	/* Dummy DMA method: Don’t use this value */
	u32 irq_val;
	u32 reserved[4];
};

enum ring_buf {
	H2EP_CTRL,
	EP2H_CTRL,
	EP2H_EMPTY_BUF,
	EP2H_FULL_BUF,
	H2EP_FULL_BUF,
	H2EP_EMPTY_BUF,
};

struct ring_buf_md {
	u32 h2ep_offset;
	u32 h2ep_size;
	u32 ep2h_offset;
	u32 ep2h_size;
};

struct bar_md {
	/* IRQ generation for control packets */
	struct irq_md irq_ctrl;
	/* IRQ generation for data packets */
	struct irq_md irq_data;
	/* Ring buffers counter offset */
	u32 ep_own_cnt_offset;
	u32 host_own_cnt_offset;
	/* Ring buffers location offset */
	struct ring_buf_md ctrl_md;
	struct ring_buf_md ep2h_md;
	struct ring_buf_md h2ep_md;
	/* RAM region for use by host when programming EP DMA controller */
	u32 host_dma_offset;
	u32 host_dma_size;
	/* Endpoint will map all RX packet buffers into this region */
	u64 bar0_base_phy;
	u32 ep_rx_pkt_offset;
	u32 ep_rx_pkt_size;
};

enum ctrl_msg_type {
	CTRL_MSG_RESERVED,
	CTRL_MSG_LINK_UP,
	CTRL_MSG_LINK_DOWN,
	CTRL_MSG_LINK_DOWN_ACK,
};

struct ctrl_msg {
	u32 msg_id; /* enum ctrl_msg_type */
	union {
		u32 reserved[7];
	} u;
};

enum data_msg_type {
	DATA_MSG_RESERVED,
	DATA_MSG_EMPTY_BUF,
	DATA_MSG_FULL_BUF,
};

struct data_msg {
	u32 msg_id; /* enum data_msg_type */
	union {
		struct {
			u32 buffer_len;
			u64 pcie_address;
		} empty_buffer;
		struct {
			u32 packet_size;
			u64 pcie_address;
		} full_buffer;
		u32 reserved[7];
	} u;
};

struct tvnet_counter {
	u32 *rd;
	u32 *wr;
};

struct ep_own_cnt {
	u32 h2ep_ctrl_rd_cnt;
	u32 ep2h_ctrl_wr_cnt;
	u32 ep2h_empty_rd_cnt;
	u32 ep2h_full_wr_cnt;
	u32 h2ep_full_rd_cnt;
	u32 h2ep_empty_wr_cnt;
};

struct ep_ring_buf {
	struct ep_own_cnt *ep_cnt;
	/* Endpoint written message buffers */
	struct ctrl_msg *ep2h_ctrl_msgs;
	struct data_msg *ep2h_full_msgs;
	struct data_msg *h2ep_empty_msgs;
};

struct host_own_cnt {
	u32 h2ep_ctrl_wr_cnt;
	u32 ep2h_ctrl_rd_cnt;
	u32 ep2h_empty_wr_cnt;
	u32 ep2h_full_rd_cnt;
	u32 h2ep_full_wr_cnt;
	u32 h2ep_empty_rd_cnt;
};

struct host_ring_buf {
	struct host_own_cnt *host_cnt;
	/* Host written message buffers */
	struct ctrl_msg *h2ep_ctrl_msgs;
	struct data_msg *ep2h_empty_msgs;
	struct data_msg *h2ep_full_msgs;
};

struct ep2h_empty_list {
	int len;
	dma_addr_t iova;
	struct sk_buff *skb;
	struct list_head list;
};

struct h2ep_empty_list {
	int size;
#if ENABLE_DMA
	struct sk_buff *skb;
#else
	struct page *page;
	void *virt;
#endif
	dma_addr_t iova;
	struct list_head list;
};

enum dir_link_state {
	DIR_LINK_STATE_DOWN,
	DIR_LINK_STATE_UP,
	DIR_LINK_STATE_SENT_DOWN,
};

enum os_link_state {
	OS_LINK_STATE_UP,
	OS_LINK_STATE_DOWN,
};

#if ENABLE_DMA
struct dma_desc_cnt {
	u32 rd_cnt;
	u32 wr_cnt;
};
#endif

static inline bool tvnet_ivc_empty(struct tvnet_counter *counter)
{
	u32 rd, wr;

	wr = READ_ONCE(*counter->wr);
	rd = READ_ONCE(*counter->rd);

	if (wr - rd > RING_COUNT)
		return true;

	return wr == rd;
}

static inline bool tvnet_ivc_full(struct tvnet_counter *counter)
{
	u32 rd, wr;

	wr = READ_ONCE(*counter->wr);
	rd = READ_ONCE(*counter->rd);

	return wr - rd >= RING_COUNT;
}

static inline u32 tvnet_ivc_rd_available(struct tvnet_counter *counter)
{
	u32 rd, wr;

	wr = READ_ONCE(*counter->wr);
	rd = READ_ONCE(*counter->rd);

	return wr - rd;
}

static inline u32 tvnet_ivc_wr_available(struct tvnet_counter *counter)
{
	u32 rd, wr;

	wr = READ_ONCE(*counter->wr);
	rd = READ_ONCE(*counter->rd);

	return (RING_COUNT - (wr - rd));
}

static inline void tvnet_ivc_advance_wr(struct tvnet_counter *counter)
{
	WRITE_ONCE(*counter->wr, READ_ONCE(*counter->wr) + 1);

	/* BAR0 mmio address is wc mem, add mb to make sure cnts are updated */
	smp_mb();
}

static inline void tvnet_ivc_advance_rd(struct tvnet_counter *counter)
{
	WRITE_ONCE(*counter->rd, READ_ONCE(*counter->rd) + 1);

	/* BAR0 mmio address is wc mem, add mb to make sure cnts are updated */
	smp_mb();
}

static inline u32 tvnet_ivc_get_wr_cnt(struct tvnet_counter *counter)
{
	return READ_ONCE(*counter->wr);
}

static inline u32 tvnet_ivc_get_rd_cnt(struct tvnet_counter *counter)
{
	return READ_ONCE(*counter->rd);
}


#endif
