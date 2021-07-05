/*
 * Copyright (c) 2019-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/tegra_vnet.h>

struct tvnet_priv {
	struct net_device *ndev;
	struct napi_struct napi;
	struct pci_dev *pdev;
	void __iomem *mmio_base;
	void __iomem *msix_tbl;
	void __iomem *dma_base;
	struct bar_md *bar_md;
	struct ep_ring_buf ep_mem;
	struct host_ring_buf host_mem;
	struct list_head ep2h_empty_list;
	/* To protect ep2h empty list */
	spinlock_t ep2h_empty_lock;
	struct tvnet_dma_desc *dma_desc;
#if ENABLE_DMA
	struct dma_desc_cnt desc_cnt;
#endif
	enum dir_link_state tx_link_state;
	enum dir_link_state rx_link_state;
	enum os_link_state os_link_state;
	/* To synchronize network link state machine*/
	struct mutex link_state_lock;
	wait_queue_head_t link_state_wq;

	struct tvnet_counter h2ep_ctrl;
	struct tvnet_counter ep2h_ctrl;
	struct tvnet_counter h2ep_empty;
	struct tvnet_counter h2ep_full;
	struct tvnet_counter ep2h_empty;
	struct tvnet_counter ep2h_full;
};

#if ENABLE_DMA
/* Program MSI settings in EP DMA for interrupts from EP DMA */
static void tvnet_host_write_dma_msix_settings(struct tvnet_priv *tvnet)
{
	u32 val;
	u16 val16;

	val = readl(tvnet->msix_tbl + PCI_MSIX_ENTRY_LOWER_ADDR);
	dma_common_wr(tvnet->dma_base, val, DMA_READ_DONE_IMWR_LOW_OFF);
	dma_common_wr(tvnet->dma_base, val, DMA_READ_ABORT_IMWR_LOW_OFF);

	val = readl(tvnet->msix_tbl + PCI_MSIX_ENTRY_UPPER_ADDR);
	dma_common_wr(tvnet->dma_base, val, DMA_READ_DONE_IMWR_HIGH_OFF);
	dma_common_wr(tvnet->dma_base, val, DMA_READ_ABORT_IMWR_HIGH_OFF);

	val16 = readw(tvnet->msix_tbl + PCI_MSIX_ENTRY_DATA);
	dma_common_wr16(tvnet->dma_base, val16, DMA_READ_IMWR_DATA_OFF_BASE);
}
#endif

static void tvnet_host_raise_ep_ctrl_irq(struct tvnet_priv *tvnet)
{
	struct irq_md *irq = &tvnet->bar_md->irq_ctrl;

	if (irq->irq_type == IRQ_SIMPLE) {
		/* Can write any value to generate sync point irq */
		writel(0x1, tvnet->mmio_base + irq->irq_addr);
		/* BAR0 mmio address is wc mem, add mb to make sure
		 * multiple interrupt writes are not combined.
		 */
		mb();
	} else {
		pr_err("%s: invalid irq type: %d\n", __func__, irq->irq_type);
	}
}

static void tvnet_host_raise_ep_data_irq(struct tvnet_priv *tvnet)
{
	struct irq_md *irq = &tvnet->bar_md->irq_data;

	if (irq->irq_type == IRQ_SIMPLE) {
		/* Can write any value to generate sync point irq */
		writel(0x1, tvnet->mmio_base + irq->irq_addr);
		/* BAR0 mmio address is wc mem, add mb to make sure
		 * multiple interrupt writes are not combined.
		 */
		mb();
	} else {
		pr_err("%s: invalid irq type: %d\n", __func__, irq->irq_type);
	}
}

static void tvnet_host_read_ctrl_msg(struct tvnet_priv *tvnet,
				     struct ctrl_msg *msg)
{
	struct ep_ring_buf *ep_mem = &tvnet->ep_mem;
	struct ctrl_msg *ctrl_msg = ep_mem->ep2h_ctrl_msgs;
	u32 idx;

	if (tvnet_ivc_empty(&tvnet->ep2h_ctrl)) {
		pr_debug("%s: EP2H ctrl ring is empty\n", __func__);
		return;
	}

	idx = tvnet_ivc_get_rd_cnt(&tvnet->ep2h_ctrl) % RING_COUNT;
	memcpy(msg, &ctrl_msg[idx], sizeof(*msg));
	tvnet_ivc_advance_rd(&tvnet->ep2h_ctrl);
}

/* TODO Handle error case */
static int tvnet_host_write_ctrl_msg(struct tvnet_priv *tvnet,
				     struct ctrl_msg *msg)
{
	struct host_ring_buf *host_mem = &tvnet->host_mem;
	struct ctrl_msg *ctrl_msg = host_mem->h2ep_ctrl_msgs;
	u32 idx;

	if (tvnet_ivc_full(&tvnet->h2ep_ctrl)) {
		/* Raise an interrupt to let host process EP2H ring */
		tvnet_host_raise_ep_ctrl_irq(tvnet);
		pr_info("%s: EP2H ctrl ring is full\n", __func__);
		return -EAGAIN;
	}

	idx = tvnet_ivc_get_wr_cnt(&tvnet->h2ep_ctrl) % RING_COUNT;
	memcpy(&ctrl_msg[idx], msg, sizeof(*msg));
	/* BAR0 mmio address is wc mem, add mb to make sure ctrl msg is written
	 * before updating counters.
	 */
	mb();
	tvnet_ivc_advance_wr(&tvnet->h2ep_ctrl);
	tvnet_host_raise_ep_ctrl_irq(tvnet);

	return 0;
}

static void tvnet_host_alloc_empty_buffers(struct tvnet_priv *tvnet)
{
	struct net_device *ndev = tvnet->ndev;
	struct host_ring_buf *host_mem = &tvnet->host_mem;
	struct data_msg *ep2h_empty_msg = host_mem->ep2h_empty_msgs;
	struct ep2h_empty_list *ep2h_empty_ptr;
	struct device *d = &tvnet->pdev->dev;
	unsigned long flags;

	while (!tvnet_ivc_full(&tvnet->ep2h_empty)) {
		struct sk_buff *skb;
		dma_addr_t iova;
		int len = ndev->mtu + ETH_HLEN;
		u32 idx;

		skb = netdev_alloc_skb(ndev, len);
		if (!skb) {
			pr_err("%s: alloc skb failed\n", __func__);
			break;
		}
		iova = dma_map_single(d, skb->data, len, DMA_FROM_DEVICE);
		if (dma_mapping_error(d, iova)) {
			pr_err("%s: dma map failed\n", __func__);
			dev_kfree_skb_any(skb);
			break;
		}

		ep2h_empty_ptr = kmalloc(sizeof(*ep2h_empty_ptr), GFP_KERNEL);
		if (!ep2h_empty_ptr) {
			dma_unmap_single(d, iova, len, DMA_FROM_DEVICE);
			dev_kfree_skb_any(skb);
			break;
		}
		ep2h_empty_ptr->skb = skb;
		ep2h_empty_ptr->iova = iova;
		ep2h_empty_ptr->len = len;
		spin_lock_irqsave(&tvnet->ep2h_empty_lock, flags);
		list_add_tail(&ep2h_empty_ptr->list, &tvnet->ep2h_empty_list);
		spin_unlock_irqrestore(&tvnet->ep2h_empty_lock, flags);

		idx = tvnet_ivc_get_wr_cnt(&tvnet->ep2h_empty) %
					RING_COUNT;
		ep2h_empty_msg[idx].u.empty_buffer.pcie_address = iova;
		ep2h_empty_msg[idx].u.empty_buffer.buffer_len = len;
		/* BAR0 mmio address is wc mem, add mb to make sure that empty
		 * buffers are updated before updating counters.
		 */
		mb();
		tvnet_ivc_advance_wr(&tvnet->ep2h_empty);

		tvnet_host_raise_ep_ctrl_irq(tvnet);
	}
}

static void tvnet_host_free_empty_buffers(struct tvnet_priv *tvnet)
{
	struct ep2h_empty_list *ep2h_empty_ptr, *temp;
	struct device *d = &tvnet->pdev->dev;
	unsigned long flags;

	spin_lock_irqsave(&tvnet->ep2h_empty_lock, flags);
	list_for_each_entry_safe(ep2h_empty_ptr, temp, &tvnet->ep2h_empty_list,
				 list) {
		list_del(&ep2h_empty_ptr->list);
		dma_unmap_single(d, ep2h_empty_ptr->iova, ep2h_empty_ptr->len,
				 DMA_FROM_DEVICE);
		dev_kfree_skb_any(ep2h_empty_ptr->skb);
		kfree(ep2h_empty_ptr);
	}
	spin_unlock_irqrestore(&tvnet->ep2h_empty_lock, flags);
}

static void tvnet_host_stop_tx_queue(struct tvnet_priv *tvnet)
{
	struct net_device *ndev = tvnet->ndev;

	netif_stop_queue(ndev);
	/* Get tx lock to make sure that there is no ongoing xmit */
	netif_tx_lock_bh(ndev);
	netif_tx_unlock_bh(ndev);
}

static void tvnet_host_stop_rx_work(struct tvnet_priv *tvnet)
{
	/* wait for interrupt handle to return to ensure rx is stopped */
	synchronize_irq(pci_irq_vector(tvnet->pdev, 1));
}

static void tvnet_host_clear_data_msg_counters(struct tvnet_priv *tvnet)
{
	struct host_ring_buf *host_mem = &tvnet->host_mem;
	struct host_own_cnt *host_cnt = host_mem->host_cnt;
	struct ep_ring_buf *ep_mem = &tvnet->ep_mem;
	struct ep_own_cnt *ep_cnt = ep_mem->ep_cnt;

	host_cnt->ep2h_empty_wr_cnt = 0;
	ep_cnt->ep2h_empty_rd_cnt = 0;
	host_cnt->h2ep_full_wr_cnt = 0;
	ep_cnt->h2ep_full_rd_cnt = 0;
}

static void tvnet_host_update_link_state(struct net_device *ndev,
					 enum os_link_state state)
{
	if (state == OS_LINK_STATE_UP) {
		netif_start_queue(ndev);
		netif_carrier_on(ndev);
	} else if (state == OS_LINK_STATE_DOWN) {
		netif_carrier_off(ndev);
		netif_stop_queue(ndev);
	} else {
		pr_err("%s: invalid sate: %d\n", __func__, state);
	}
}

/* OS link state machine */
static void tvnet_host_update_link_sm(struct tvnet_priv *tvnet)
{
	struct net_device *ndev = tvnet->ndev;
	enum os_link_state old_state = tvnet->os_link_state;

	if ((tvnet->rx_link_state == DIR_LINK_STATE_UP) &&
	    (tvnet->tx_link_state == DIR_LINK_STATE_UP))
		tvnet->os_link_state = OS_LINK_STATE_UP;
	else
		tvnet->os_link_state = OS_LINK_STATE_DOWN;

	if (tvnet->os_link_state != old_state)
		tvnet_host_update_link_state(ndev, tvnet->os_link_state);
}

/* One way link state machine*/
static void tvnet_host_user_link_up_req(struct tvnet_priv *tvnet)
{
	struct ctrl_msg msg;

	tvnet_host_clear_data_msg_counters(tvnet);
	tvnet_host_alloc_empty_buffers(tvnet);
	msg.msg_id = CTRL_MSG_LINK_UP;
	tvnet_host_write_ctrl_msg(tvnet, &msg);
	tvnet->rx_link_state = DIR_LINK_STATE_UP;
	tvnet_host_update_link_sm(tvnet);
}

static void tvnet_host_user_link_down_req(struct tvnet_priv *tvnet)
{
	struct ctrl_msg msg;

	tvnet->rx_link_state = DIR_LINK_STATE_SENT_DOWN;
	msg.msg_id = CTRL_MSG_LINK_DOWN;
	tvnet_host_write_ctrl_msg(tvnet, &msg);
	tvnet_host_update_link_sm(tvnet);
}

static void tvnet_host_rcv_link_up_msg(struct tvnet_priv *tvnet)
{
	tvnet->tx_link_state = DIR_LINK_STATE_UP;
	tvnet_host_update_link_sm(tvnet);
}

static void tvnet_host_rcv_link_down_msg(struct tvnet_priv *tvnet)
{
	struct ctrl_msg msg;

	/* Stop using empty buffers of remote system */
	tvnet_host_stop_tx_queue(tvnet);
	msg.msg_id = CTRL_MSG_LINK_DOWN_ACK;
	tvnet_host_write_ctrl_msg(tvnet, &msg);
	tvnet->tx_link_state = DIR_LINK_STATE_DOWN;
	tvnet_host_update_link_sm(tvnet);
}

static void tvnet_host_rcv_link_down_ack(struct tvnet_priv *tvnet)
{
	/* Stop using empty buffers(which are full in rx) of local system */
	tvnet_host_stop_rx_work(tvnet);
	tvnet_host_free_empty_buffers(tvnet);
	tvnet->rx_link_state = DIR_LINK_STATE_DOWN;
	wake_up_interruptible(&tvnet->link_state_wq);
	tvnet_host_update_link_sm(tvnet);
}

static int tvnet_host_open(struct net_device *ndev)
{
	struct tvnet_priv *tvnet = netdev_priv(ndev);

	mutex_lock(&tvnet->link_state_lock);
	if (tvnet->rx_link_state == DIR_LINK_STATE_DOWN)
		tvnet_host_user_link_up_req(tvnet);
	napi_enable(&tvnet->napi);
	mutex_unlock(&tvnet->link_state_lock);

	return 0;
}

static int tvnet_host_close(struct net_device *ndev)
{
	struct tvnet_priv *tvnet = netdev_priv(ndev);
	int ret = 0;

	mutex_lock(&tvnet->link_state_lock);
	napi_disable(&tvnet->napi);
	if (tvnet->rx_link_state == DIR_LINK_STATE_UP)
		tvnet_host_user_link_down_req(tvnet);

	ret = wait_event_interruptible_timeout(tvnet->link_state_wq,
					       (tvnet->rx_link_state ==
					       DIR_LINK_STATE_DOWN),
					       msecs_to_jiffies(LINK_TIMEOUT));
	ret = (ret > 0) ? 0 : -ETIMEDOUT;
	if (ret < 0) {
		pr_err("%s: link state machine failed: tx_state: %d rx_state: %d err: %d",
		       __func__, tvnet->tx_link_state, tvnet->rx_link_state,
		       ret);
		tvnet->rx_link_state = DIR_LINK_STATE_UP;
	}
	mutex_unlock(&tvnet->link_state_lock);

	return ret;
}

static int tvnet_host_change_mtu(struct net_device *ndev, int new_mtu)
{
	bool set_down = false;

	if (new_mtu > TVNET_MAX_MTU || new_mtu < TVNET_MIN_MTU) {
		pr_err("MTU range is %d to %d\n", TVNET_MIN_MTU,
		       TVNET_MAX_MTU);
		return -EINVAL;
	}

	if (netif_running(ndev)) {
		set_down = true;
		tvnet_host_close(ndev);
	}

	pr_info("changing MTU from %d to %d\n", ndev->mtu, new_mtu);
	ndev->mtu = new_mtu;

	if (set_down)
		tvnet_host_open(ndev);

	return 0;
}

static netdev_tx_t tvnet_host_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct tvnet_priv *tvnet = netdev_priv(ndev);
	struct host_ring_buf *host_mem = &tvnet->host_mem;
	struct data_msg *h2ep_full_msg = host_mem->h2ep_full_msgs;
	struct skb_shared_info *info = skb_shinfo(skb);
	struct ep_ring_buf *ep_mem = &tvnet->ep_mem;
	struct data_msg *h2ep_empty_msg = ep_mem->h2ep_empty_msgs;
	struct device *d = &tvnet->pdev->dev;
#if ENABLE_DMA
	struct tvnet_dma_desc *dma_desc = tvnet->dma_desc;
	struct dma_desc_cnt *desc_cnt = &tvnet->desc_cnt;
	u32 desc_widx, desc_ridx, val;
	u32 ctrl_d;
	unsigned long timeout;
#endif
	dma_addr_t src_iova;
	dma_addr_t dst_iova;
	u32 rd_idx;
	u32 wr_idx;
	void *dst_virt;
	int len;

	/* TODO Not expecting skb frags, remove this after testing */
	WARN_ON(info->nr_frags);

	/* Check if H2EP_EMPTY_BUF available to read */
	if (!tvnet_ivc_rd_available(&tvnet->h2ep_empty)) {
		tvnet_host_raise_ep_ctrl_irq(tvnet);
		pr_debug("%s: No H2EP empty msg, stop tx\n", __func__);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

	/* Check if H2EP_FULL_BUF available to write */
	if (tvnet_ivc_full(&tvnet->h2ep_full)) {
		tvnet_host_raise_ep_ctrl_irq(tvnet);
		pr_debug("%s: No H2EP full buf, stop tx\n", __func__);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}

#if ENABLE_DMA
	/* Check if dma desc available */
	if ((desc_cnt->wr_cnt - desc_cnt->rd_cnt) >= DMA_DESC_COUNT) {
		pr_debug("%s: dma descriptors are not available\n", __func__);
		netif_stop_queue(ndev);
		return NETDEV_TX_BUSY;
	}
#endif

	len = skb_headlen(skb);

	src_iova = dma_map_single(d, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(d, src_iova)) {
		pr_err("%s: dma_map_single failed\n", __func__);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* Get H2EP empty msg */
	rd_idx = tvnet_ivc_get_rd_cnt(&tvnet->h2ep_empty) %
				RING_COUNT;
	dst_iova = h2ep_empty_msg[rd_idx].u.empty_buffer.pcie_address;
	dst_virt = tvnet->mmio_base + (dst_iova - tvnet->bar_md->bar0_base_phy);
	/* Advance read count after all failure cases complated, to avoid
	 * dangling buffer at endpoint.
	 */
	tvnet_ivc_advance_rd(&tvnet->h2ep_empty);
	/* Raise an interrupt to let EP populate H2EP_EMPTY_BUF ring */
	tvnet_host_raise_ep_ctrl_irq(tvnet);

#if ENABLE_DMA
	/* Trigger DMA write from src_iova to dst_iova */
	desc_widx = desc_cnt->wr_cnt % DMA_DESC_COUNT;
	dma_desc[desc_widx].size = len;
	dma_desc[desc_widx].sar_low = lower_32_bits(src_iova);
	dma_desc[desc_widx].sar_high = upper_32_bits(src_iova);
	dma_desc[desc_widx].dar_low = lower_32_bits(dst_iova);
	dma_desc[desc_widx].dar_high = upper_32_bits(dst_iova);
	/* CB bit should be set at the end */
	mb();
	/* RIE is not required for polling mode */
	ctrl_d = DMA_CH_CONTROL1_OFF_RDCH_RIE;
	ctrl_d |= DMA_CH_CONTROL1_OFF_RDCH_LIE;
	ctrl_d |= DMA_CH_CONTROL1_OFF_RDCH_CB;
	dma_desc[desc_widx].ctrl_reg.ctrl_d = ctrl_d;
	/*
	 * Read after write to avoid EP DMA reading LLE before CB is written to
	 * EP's system memory.
	 */
	ctrl_d = dma_desc[desc_widx].ctrl_reg.ctrl_d;

	/* DMA write should not go out of order wrt CB bit set */
	mb();

	timeout = jiffies + msecs_to_jiffies(1000);
	dma_common_wr(tvnet->dma_base, DMA_RD_DATA_CH, DMA_READ_DOORBELL_OFF);

	desc_cnt->wr_cnt++;

	while (true) {
		val = dma_common_rd(tvnet->dma_base, DMA_READ_INT_STATUS_OFF);
		if (val == BIT(DMA_RD_DATA_CH)) {
			dma_common_wr(tvnet->dma_base, val,
				      DMA_READ_INT_CLEAR_OFF);
			break;
		}
		if (time_after(jiffies, timeout)) {
			pr_err("dma took more time, reset dma engine\n");
			dma_common_wr(tvnet->dma_base,
				      DMA_READ_ENGINE_EN_OFF_DISABLE,
				      DMA_READ_ENGINE_EN_OFF);
			mdelay(1);
			dma_common_wr(tvnet->dma_base,
				      DMA_READ_ENGINE_EN_OFF_ENABLE,
				      DMA_READ_ENGINE_EN_OFF);
			desc_cnt->wr_cnt--;
			dma_unmap_single(d, src_iova, len, DMA_TO_DEVICE);
			return NETDEV_TX_BUSY;
		}
	}

	desc_ridx = tvnet->desc_cnt.rd_cnt % DMA_DESC_COUNT;
	/* Clear DMA cycle bit and increment rd_cnt */
	dma_desc[desc_ridx].ctrl_reg.ctrl_e.cb = 0;
	mb();

	tvnet->desc_cnt.rd_cnt++;
#else
	/* Copy skb->data to endpoint dst address, use CPU virt addr */
	memcpy(dst_virt, skb->data, len);
	/* BAR0 mmio address is wc mem, add mb to make sure that complete
	 * skb->data is written before updating counters.
	 */
	mb();
#endif

	/* Push dst to H2EP full ring */
	wr_idx = tvnet_ivc_get_wr_cnt(&tvnet->h2ep_full) %
				RING_COUNT;
	h2ep_full_msg[wr_idx].u.full_buffer.packet_size = len;
	h2ep_full_msg[wr_idx].u.full_buffer.pcie_address = dst_iova;
	h2ep_full_msg[wr_idx].msg_id = DATA_MSG_FULL_BUF;
	/* BAR0 mmio address is wc mem, add mb to make sure that full
	 * buffer is written before updating counters.
	 */
	mb();
	tvnet_ivc_advance_wr(&tvnet->h2ep_full);
	tvnet_host_raise_ep_data_irq(tvnet);

	/* Free skb */
	dma_unmap_single(d, src_iova, len, DMA_TO_DEVICE);
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;
}

static const struct net_device_ops tvnet_host_netdev_ops = {
	.ndo_open = tvnet_host_open,
	.ndo_stop = tvnet_host_close,
	.ndo_start_xmit	= tvnet_host_start_xmit,
	.ndo_change_mtu = tvnet_host_change_mtu,
};

static void tvnet_host_setup_bar0_md(struct tvnet_priv *tvnet)
{
	struct ep_ring_buf *ep_mem = &tvnet->ep_mem;
	struct host_ring_buf *host_mem = &tvnet->host_mem;

	tvnet->bar_md = (struct bar_md *)tvnet->mmio_base;

	ep_mem->ep_cnt = (struct ep_own_cnt *)(tvnet->mmio_base +
					tvnet->bar_md->ep_own_cnt_offset);
	ep_mem->ep2h_ctrl_msgs = (struct ctrl_msg *)(tvnet->mmio_base +
					tvnet->bar_md->ctrl_md.ep2h_offset);
	ep_mem->ep2h_full_msgs = (struct data_msg *)(tvnet->mmio_base +
					tvnet->bar_md->ep2h_md.ep2h_offset);
	ep_mem->h2ep_empty_msgs = (struct data_msg *)(tvnet->mmio_base +
					tvnet->bar_md->h2ep_md.ep2h_offset);

	host_mem->host_cnt = (struct host_own_cnt *)(tvnet->mmio_base +
					tvnet->bar_md->host_own_cnt_offset);
	host_mem->h2ep_ctrl_msgs = (struct ctrl_msg *)(tvnet->mmio_base +
					tvnet->bar_md->ctrl_md.h2ep_offset);
	host_mem->ep2h_empty_msgs = (struct data_msg *)(tvnet->mmio_base +
					tvnet->bar_md->ep2h_md.h2ep_offset);
	host_mem->h2ep_full_msgs = (struct data_msg *)(tvnet->mmio_base +
					tvnet->bar_md->h2ep_md.h2ep_offset);

	tvnet->dma_desc = (struct tvnet_dma_desc *)(tvnet->mmio_base +
					tvnet->bar_md->host_dma_offset);

	tvnet->h2ep_ctrl.rd = &ep_mem->ep_cnt->h2ep_ctrl_rd_cnt;
	tvnet->h2ep_ctrl.wr = &host_mem->host_cnt->h2ep_ctrl_wr_cnt;
	tvnet->ep2h_ctrl.rd = &host_mem->host_cnt->ep2h_ctrl_rd_cnt;
	tvnet->ep2h_ctrl.wr = &ep_mem->ep_cnt->ep2h_ctrl_wr_cnt;
	tvnet->h2ep_empty.rd = &host_mem->host_cnt->h2ep_empty_rd_cnt;
	tvnet->h2ep_empty.wr = &ep_mem->ep_cnt->h2ep_empty_wr_cnt;
	tvnet->h2ep_full.rd = &ep_mem->ep_cnt->h2ep_full_rd_cnt;
	tvnet->h2ep_full.wr = &host_mem->host_cnt->h2ep_full_wr_cnt;
	tvnet->ep2h_empty.rd = &ep_mem->ep_cnt->ep2h_empty_rd_cnt;
	tvnet->ep2h_empty.wr = &host_mem->host_cnt->ep2h_empty_wr_cnt;
	tvnet->ep2h_full.rd = &host_mem->host_cnt->ep2h_full_rd_cnt;
	tvnet->ep2h_full.wr = &ep_mem->ep_cnt->ep2h_full_wr_cnt;
}

static void tvnet_host_process_ctrl_msg(struct tvnet_priv *tvnet)
{
	struct ctrl_msg msg;

	while (tvnet_ivc_rd_available(&tvnet->ep2h_ctrl)) {
		tvnet_host_read_ctrl_msg(tvnet, &msg);
		if (msg.msg_id == CTRL_MSG_LINK_UP)
			tvnet_host_rcv_link_up_msg(tvnet);
		else if (msg.msg_id == CTRL_MSG_LINK_DOWN)
			tvnet_host_rcv_link_down_msg(tvnet);
		else if (msg.msg_id == CTRL_MSG_LINK_DOWN_ACK)
			tvnet_host_rcv_link_down_ack(tvnet);
	}
}

static int tvnet_host_process_ep2h_msg(struct tvnet_priv *tvnet)
{
	struct ep_ring_buf *ep_mem = &tvnet->ep_mem;
	struct data_msg *data_msg = ep_mem->ep2h_full_msgs;
	struct device *d = &tvnet->pdev->dev;
	struct ep2h_empty_list *ep2h_empty_ptr;
	struct net_device *ndev = tvnet->ndev;
	int count = 0;

	while ((count < TVNET_NAPI_WEIGHT) &&
	       tvnet_ivc_rd_available(&tvnet->ep2h_full)) {
		struct sk_buff *skb;
		u64 pcie_address;
		u32 len;
		int idx, found = 0;
		unsigned long flags;

		/* Read EP2H full msg */
		idx = tvnet_ivc_get_rd_cnt(&tvnet->ep2h_full) %
					RING_COUNT;
		len = data_msg[idx].u.full_buffer.packet_size;
		pcie_address = data_msg[idx].u.full_buffer.pcie_address;

		spin_lock_irqsave(&tvnet->ep2h_empty_lock, flags);
		list_for_each_entry(ep2h_empty_ptr, &tvnet->ep2h_empty_list,
				    list) {
			if (ep2h_empty_ptr->iova == pcie_address) {
				found = 1;
				break;
			}
		}
		WARN_ON(!found);
		list_del(&ep2h_empty_ptr->list);
		spin_unlock_irqrestore(&tvnet->ep2h_empty_lock, flags);

		/* Advance H2EP full buffer after search in local list */
		tvnet_ivc_advance_rd(&tvnet->ep2h_full);
		/* If EP2H network queue is stopped due to lack of EP2H_FULL
		 * queue, raising ctrl irq will help.
		 */
		tvnet_host_raise_ep_ctrl_irq(tvnet);

		dma_unmap_single(d, pcie_address, ndev->mtu + ETH_HLEN, DMA_FROM_DEVICE);
		skb = ep2h_empty_ptr->skb;
		skb_put(skb, len);
		skb->protocol = eth_type_trans(skb, ndev);
		napi_gro_receive(&tvnet->napi, skb);

		/* Free EP2H empty list element */
		kfree(ep2h_empty_ptr);
		count++;
	}

	return count;
}

static irqreturn_t tvnet_irq_ctrl(int irq, void *data)
{
	struct net_device *ndev = data;
	struct tvnet_priv *tvnet = netdev_priv(ndev);

	if (netif_queue_stopped(ndev)) {
		if ((tvnet->os_link_state == OS_LINK_STATE_UP) &&
		    tvnet_ivc_rd_available(&tvnet->h2ep_empty) &&
		    !tvnet_ivc_full(&tvnet->h2ep_full)) {
			pr_debug("%s: wake net tx queue\n", __func__);
			netif_wake_queue(ndev);
		}
	}

	if (tvnet_ivc_rd_available(&tvnet->ep2h_ctrl))
		tvnet_host_process_ctrl_msg(tvnet);

	if (!tvnet_ivc_full(&tvnet->ep2h_empty) &&
	    (tvnet->os_link_state == OS_LINK_STATE_UP))
		tvnet_host_alloc_empty_buffers(tvnet);

	return IRQ_HANDLED;
}

static irqreturn_t tvnet_irq_data(int irq, void *data)
{
	struct net_device *ndev = data;
	struct tvnet_priv *tvnet = netdev_priv(ndev);

	if (tvnet_ivc_rd_available(&tvnet->ep2h_full)) {
		disable_irq_nosync(pci_irq_vector(tvnet->pdev, 1));
		napi_schedule(&tvnet->napi);
	}

	return IRQ_HANDLED;
}

static int tvnet_host_poll(struct napi_struct *napi, int budget)
{
	struct tvnet_priv *tvnet = container_of(napi, struct tvnet_priv, napi);
	int work_done;

	work_done = tvnet_host_process_ep2h_msg(tvnet);
	trace_printk("work_done: %d budget: %d\n", work_done, budget);
	if (work_done < budget) {
		napi_complete(napi);
		enable_irq(pci_irq_vector(tvnet->pdev, 1));
		mmiowb();
	}

	return work_done;
}

static int tvnet_host_probe(struct pci_dev *pdev,
			    const struct pci_device_id *pci_id)
{
	struct tvnet_priv *tvnet;
	struct net_device *ndev;
	int ret;

	dev_dbg(&pdev->dev, "%s: PCIe VID: 0x%x DID: 0x%x\n", __func__,
		pci_id->vendor, pci_id->device);
	ndev = alloc_etherdev(sizeof(struct tvnet_priv));
	if (!ndev) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "alloc_etherdev() failed");
		goto fail;
	}

	eth_hw_addr_random(ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->netdev_ops = &tvnet_host_netdev_ops;
	tvnet = netdev_priv(ndev);
	tvnet->ndev = ndev;
	tvnet->pdev = pdev;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "pci_enable_device() failed: %d\n", ret);
		goto free_netdev;
	}

	/*
	 * In CPU memory write case, skb->data buffer is copied to dst in BAR.
	 * Unaligned dword skb->data pointer comes in start_xmit, so use
	 * write combine mapping for BAR.
	 */
#if ENABLE_DMA
	tvnet->mmio_base = devm_ioremap(&pdev->dev,
					pci_resource_start(pdev, 0),
					pci_resource_len(pdev, 0));
#else
	tvnet->mmio_base = devm_ioremap_wc(&pdev->dev,
					   pci_resource_start(pdev, 0),
					   pci_resource_len(pdev, 0));
#endif
	if (!tvnet->mmio_base) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "BAR0 ioremap() failed\n");
		goto pci_disable;
	}

	/* MSI-X vector table is saved in BAR2 */
	tvnet->msix_tbl = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 2),
				       pci_resource_len(pdev, 2));
	if (!tvnet->msix_tbl) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "BAR2 ioremap() failed\n");
		goto pci_disable;
	}

	tvnet->dma_base = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 4),
				       pci_resource_len(pdev, 4));
	if (!tvnet->dma_base) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "BAR4 ioremap() failed\n");
		goto pci_disable;
	}

	pci_set_master(pdev);
	pci_set_drvdata(pdev, tvnet);

	/* Setup BAR0 meta data */
	tvnet_host_setup_bar0_md(tvnet);

	netif_napi_add(ndev, &tvnet->napi, tvnet_host_poll, TVNET_NAPI_WEIGHT);

	ndev->mtu = TVNET_DEFAULT_MTU;

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&pdev->dev, "register_netdev() fail: %d\n", ret);
		goto pci_disable;
	}
	netif_carrier_off(ndev);

	tvnet->rx_link_state = DIR_LINK_STATE_DOWN;
	tvnet->tx_link_state = DIR_LINK_STATE_DOWN;
	tvnet->os_link_state = OS_LINK_STATE_DOWN;
	mutex_init(&tvnet->link_state_lock);
	init_waitqueue_head(&tvnet->link_state_wq);

	ret = pci_alloc_irq_vectors(pdev, 2, 2, PCI_IRQ_MSIX |
				    PCI_IRQ_AFFINITY);
	if (ret <= 0) {
		dev_err(&pdev->dev, "pci_alloc_irq_vectors() fail: %d\n", ret);
		ret = -EIO;
		goto unreg_netdev;
	}

	ret = request_irq(pci_irq_vector(pdev, 0), tvnet_irq_ctrl, 0,
			  ndev->name, ndev);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq() fail: %d\n", ret);
		goto disable_msi;
	}

	ret = request_irq(pci_irq_vector(pdev, 1), tvnet_irq_data, 0,
			  ndev->name, ndev);
	if (ret < 0) {
		dev_err(&pdev->dev, "request_irq() fail: %d\n", ret);
		goto fail_request_irq_ctrl;
	}

#if ENABLE_DMA
	tvnet_host_write_dma_msix_settings(tvnet);
#endif

	INIT_LIST_HEAD(&tvnet->ep2h_empty_list);
	spin_lock_init(&tvnet->ep2h_empty_lock);

	return 0;

fail_request_irq_ctrl:
	free_irq(pci_irq_vector(pdev, 0), ndev);
disable_msi:
	pci_free_irq_vectors(pdev);
unreg_netdev:
	unregister_netdev(ndev);
pci_disable:
	netif_napi_del(&tvnet->napi);
	pci_disable_device(pdev);
free_netdev:
	free_netdev(ndev);
fail:
	return ret;
}

static const struct pci_device_id tvnet_host_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_NVIDIA,
		     PCI_DEVICE_ID_NVIDIA_JETSON_AGX_NETWORK) },
	{0,},
};

static struct pci_driver tvnet_pci_driver = {
	.name		= "tvnet",
	.id_table	= tvnet_host_pci_tbl,
	.probe		= tvnet_host_probe,
};

module_pci_driver(tvnet_pci_driver);

MODULE_DESCRIPTION("PCI TEGRA VIRTUAL NETWORK DRIVER");
MODULE_AUTHOR("Manikanta Maddireddy <mmaddireddy@nvidia.com>");
MODULE_LICENSE("GPL v2");
