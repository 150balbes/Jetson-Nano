/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#define DEBUG_SECCAN
#undef DEBUG_SECCAN
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <soc/tegra/chip-id.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/slab.h>

#include <linux/tegra-ivc.h>
#define DRV_NAME "tegra_hv_seccan"

#define MTT_CAN_NAPI_WEIGHT 64
#define MTT_CAN_TX_OBJ_NUM  32
#define MTT_CAN_MAX_MRAM_ELEMS  9
#define MTT_MAX_TX_CONF     4
#define MTT_MAX_RX_CONF     3

/* this is defined in MTT CAN for CAN flag */
#define CAN_BRS_FLAG 0x01
#define CAN_ESI_FLAG 0x02
#define CAN_FD_FLAG  0x04
#define CAN_DIR_RX   0x08

#define DEFAULT_MAX_TX_DELAY_MSECS  100

/* CAN packet message header */
struct seccanpkt_hdr {
	u32 cmd;
	u32 len;
	/* reserved for CAN local protocol - i.e. type, vm source, etc. */
	u32 reserve;
};

/* packet header size */
#define HDR_SIZE	sizeof(struct seccanpkt_hdr)

#define F_CNTRL		BIT(0) /* control frame (0 = data frame) */
#define F_CNTRL_CMD(x)	((u32)((x) & 0xff) << 24) /* control frame command */

#define F_CNTRL_CMD_STATUS	F_CNTRL_CMD(0)	  /* link status cmd */
#define F_STATUS_UP		BIT(1) /* link status is up */
#define F_STATUS_PAUSE		BIT(2) /* link status is pause */

#define F_STATUS_PENDING	BIT(23)	/* pending link status update */

#define F_DATA_FIRST		BIT(1)	/* first chunk of a frame */
#define F_DATA_LAST		BIT(2)	/* last chunk of a frame */
#define F_DATA_FSIZE_SHIFT	16
#define F_DATA_FSIZE_MASK	(~0 << F_DATA_FSIZE_SHIFT)
#define F_DATA_FSIZE(x) \
	(((u32)(x) << F_DATA_FSIZE_SHIFT) & F_DATA_FSIZE_MASK)

struct hv_seccan_stats {
	struct u64_stats_sync tx_syncp;
	struct u64_stats_sync rx_syncp;
	u64 tx_bytes;
	u64 tx_packets;
	u64 tx_drops;

	u64 rx_bytes;
	u64 rx_packets;
	u64 rx_drops;

	/* internal tx stats */
	u64 tx_linearize_fail;
	u64 tx_queue_full;
	u64 tx_wq_fail;
	u64 tx_ivc_write_fail;
	/* internal rx stats */
	u64 rx_bad_frame;
	u64 rx_bad_packet;
	u64 rx_unexpected_packet;
	u64 rx_alloc_fail;
	u64 rx_overflow;
};

struct hv_seccan_priv {
	struct tegra_hv_ivc_cookie *ivck;
	struct hv_seccan_stats __percpu *stats;
	struct can_priv can;
	struct platform_device *pdev;
	struct device *device;
	struct net_device *ndev;
	struct napi_struct napi;
	u32 tx_next;
	u32 tx_echo;
	u32 tx_object;
	u32 tx_obj_cancelled;
	u32 max_tx_delay;
	wait_queue_head_t waitq;
};

static s32 nv_seccan_rx(struct hv_seccan_priv *hvn, s32 limit)
{
	s32 nr;
	struct net_device *ndev = hvn->ndev;
	struct net_device_stats *stats = &ndev->stats;
	struct sk_buff *skb;
	u8 *buf;
	struct seccanpkt_hdr *hdr;
	struct canfd_frame *fd_frame;
#ifdef DEBUG_SECCAN
	s32 i;
#endif

	nr = 0;
	/* grabbing a frame can fail for the following reasons:
	 * 1. the channel is empty / peer is uncooperative
	 * 2. the channel is under reset / peer has restarted
	 */
	buf = tegra_hv_ivc_read_get_next_frame(hvn->ivck);
	if (!IS_ERR(buf)) {
		hdr = (struct seccanpkt_hdr *)buf;
		skb = alloc_canfd_skb(ndev, &fd_frame);
		if (!skb) {
			stats->rx_dropped += hdr->len;
		} else {
			memcpy(fd_frame, buf +  HDR_SIZE, hdr->len);
			netif_receive_skb(skb);
			stats->rx_bytes += hdr->len;
			stats->rx_packets++;
#ifdef DEBUG_SECCAN
			/* DEBUG:print out the received data for debug only */
			for (i = 0; i < hdr->len ; i++)
				netdev_info(ndev, "0x%02x, ",
					    (buf[i + HDR_SIZE]));
			netdev_info(ndev, "\n");
#endif
			nr++;
		}
	}
	(void)tegra_hv_ivc_read_advance(hvn->ivck);
	return nr;
}

static void nv_seccan_tx_complete(struct hv_seccan_priv *hvn)
{
	struct net_device *ndev = hvn->ndev;

	if (tegra_hv_ivc_can_write(hvn->ivck)) {
		wake_up_interruptible_all(&hvn->waitq);
		netif_wake_queue(ndev);
	}
}

static irqreturn_t nv_seccan_interrupt(s32 irq, void *data)
{
	struct net_device *ndev = data;
	struct hv_seccan_priv *hvn = netdev_priv(ndev);

	netdev_dbg(ndev, "CAN interrupt\n");
	/* until this function returns 0, the channel is unusable */
	if (tegra_hv_ivc_channel_notified(hvn->ivck) == 0) {
		if (tegra_hv_ivc_can_write(hvn->ivck))
			wake_up_interruptible_all(&hvn->waitq);

		if (tegra_hv_ivc_can_read(hvn->ivck))
			napi_schedule(&hvn->napi);
	}
	return IRQ_HANDLED;
}

static s32 nv_seccan_change_mtu(struct net_device *ndev, s32 new_mtu)
{
	s32 rtval;

	netdev_dbg(ndev, "change mtu to %d\n", new_mtu);
	if (ndev->flags & IFF_UP) {
		/* Do not allow changing the MTU while running */
		rtval = -EBUSY;
	} else {
		if (new_mtu != CAN_MTU && new_mtu != CANFD_MTU)	{
			rtval = -EINVAL;
		} else {
			ndev->mtu = new_mtu;
			rtval = 0;
		}
	}
	return rtval;
}

static void *nv_seccan_xmit_ivc_buffer(struct hv_seccan_priv *hvn)
{
	void *p;
	s32 ret;
	/* grabbing a frame can fail for the following reasons:
	 * 1. the channel is full / peer is uncooperative
	 * 2. the channel is under reset / peer has restarted
	 */
	netdev_dbg(hvn->ndev, "CAN xmit: getting for IVC buffer\n");
	p = tegra_hv_ivc_write_get_next_frame(hvn->ivck);
	if (IS_ERR(p)) {
		ret = wait_event_interruptible_timeout(
				hvn->waitq,
				!IS_ERR(p = tegra_hv_ivc_write_get_next_frame(
						hvn->ivck)),
				msecs_to_jiffies(hvn->max_tx_delay));
		if (ret <= 0) {
			net_warn_ratelimited(
					"%s: timed out after %u ms\n",
					hvn->ndev->name,
					hvn->max_tx_delay);
		}
	}
	return p;
}

static netdev_tx_t nv_seccan_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct hv_seccan_priv *hvn = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *frame = (struct canfd_frame *)skb->data;
	struct seccanpkt_hdr *buf;
	u32 size = CAN_MTU;

	netdev_dbg(ndev, "CAN xmit\n");
	if (!(can_dropped_invalid_skb(ndev, skb))) {
		if (can_is_canfd_skb(skb)) {
			frame->flags |= CAN_FD_FLAG;
			size = CANFD_MTU;
		}
		buf = (struct seccanpkt_hdr *)nv_seccan_xmit_ivc_buffer(hvn);
		if (!IS_ERR(buf)) {
			buf->len = size;
			buf->cmd = F_DATA_FSIZE(buf->len) |
				F_DATA_FIRST |
				F_DATA_LAST;
			netdev_dbg(ndev, "CAN xmit buf %u, 0x%x\n",
				   buf->len, buf->cmd);
			(void)memcpy((u8 *)buf + HDR_SIZE, (u8 *)frame, size);
			(void)tegra_hv_ivc_write_advance(hvn->ivck);
			stats->tx_bytes += size;
			stats->tx_packets++;
		} else {
			netdev_err(ndev, "Tx message queue full\n");
			kfree_skb(skb);
			ndev->stats.tx_dropped++;
			netif_stop_queue(ndev);
		}
	}
	return NETDEV_TX_OK;
}

static s32 nv_seccan_open(struct net_device *ndev)
{
	struct hv_seccan_priv *hvn = netdev_priv(ndev);

	netdev_dbg(ndev, "CAN open\n");
	napi_enable(&hvn->napi);
	netif_start_queue(ndev);
	/* check if there are already packets in our queue,
	 * and if so, we need to schedule a call to handle them
	 */
	if (tegra_hv_ivc_can_read(hvn->ivck))
		napi_schedule(&hvn->napi);

	return 0;
}

static int nv_seccan_stop(struct net_device *ndev)
{
	struct hv_seccan_priv *hvn = netdev_priv(ndev);

	netdev_dbg(ndev, "CAN stop\n");
	netif_stop_queue(ndev);
	napi_disable(&hvn->napi);

	return 0;
}

static s32 nv_seccan_poll(struct napi_struct *napi, s32 budget)
{
	s32 work_done = 0;
	struct hv_seccan_priv *hvn = container_of(napi,
			struct hv_seccan_priv, napi);

	nv_seccan_tx_complete(hvn);
	work_done = nv_seccan_rx(hvn, budget);
	if (work_done < budget) {
		napi_complete(napi);
		/* if an interrupt occurs after nv_seccan_rx() but before
		 * napi_complete(), we lose the call to napi_schedule().
		 */
		if (tegra_hv_ivc_can_read(hvn->ivck))
			napi_reschedule(napi);
	}
	return work_done;
}

static const struct net_device_ops nv_seccandev_ops = {
	.ndo_open = nv_seccan_open,
	.ndo_stop = nv_seccan_stop,
	.ndo_start_xmit = nv_seccan_xmit,
	.ndo_change_mtu = nv_seccan_change_mtu,
};

static struct net_device *alloc_seccan_dev(void)
{
	struct net_device *ndev = NULL;
	struct hv_seccan_priv *hvn;

	ndev = alloc_candev(sizeof(struct hv_seccan_priv), MTT_CAN_TX_OBJ_NUM);
	if (ndev) {
		ndev->flags = (IFF_NOARP | IFF_ECHO);
		ndev->type  = ARPHRD_CAN; /* the netdevice hardware type */
		ndev->mtu  = CANFD_MTU;
		hvn = netdev_priv(ndev);
		hvn->ndev = ndev;
	}
	return ndev;
}

static s32 tegra_hv_seccan_probe(struct platform_device *pdev)
{
	s32 ret;
	struct device *dev = &pdev->dev;
	struct device_node *dn, *hv_dn;
	struct net_device *ndev = NULL;
	struct hv_seccan_priv *hvn = NULL;
	u32 id;

	dev_info(dev, "Starting sec can driver\n");
	if (!is_tegra_hypervisor_mode()) {
		dev_info(dev, "Hypervisor is not present\n");
		return -ENODEV;
	}

	dn = dev->of_node;
	if (!dn) {
		dev_err(dev, "No OF data\n");
		return -EINVAL;
	}

	hv_dn = of_parse_phandle(dn, "ivc", 0);
	if (!hv_dn) {
		dev_err(dev, "Failed to parse phandle of ivc prop\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(dn, "ivc", 1, &id);
	if (ret != 0) {
		dev_err(dev, "Failed to read IVC property ID\n");
		goto out_of_put;
	}

	ndev = alloc_seccan_dev();
	if (!ndev) {
		dev_err(dev, "CAN device allocation failed\n");
		ret = -ENOMEM;
		goto out_of_put;
	}

	hvn = netdev_priv(ndev);

	hvn->stats = alloc_percpu(struct hv_seccan_stats);
	if (!hvn->stats) {
		dev_err(dev, "Failed to allocate per-cpu stats\n");
		ret = -ENOMEM;
		goto out_free_ndev;
	}

	hvn->ivck = tegra_hv_ivc_reserve(hv_dn, id, NULL);
	of_node_put(hv_dn);
	hv_dn = NULL;

	if (IS_ERR_OR_NULL(hvn->ivck)) {
		dev_err(dev, "Failed to reserve IVC channel %d\n", id);
		ret = PTR_ERR(hvn->ivck);
		hvn->ivck = NULL;
		goto out_free_stats;
	}
	hvn->max_tx_delay = DEFAULT_MAX_TX_DELAY_MSECS;
	/* make sure the frame size is sufficient */
	if (hvn->ivck->frame_size <= HDR_SIZE + 4) {
		dev_err(dev, "IVC frame size too small to support CAN COMM\n");
		ret = -EINVAL;
		goto out_unreserve;
	}

	dev_info(dev, "Reserved IVC channel #%d - frame_size=%d\n",
		 id, hvn->ivck->frame_size);

	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, ndev);
	ndev->netdev_ops = &nv_seccandev_ops;

	hvn->pdev = pdev;
	hvn->ndev = ndev;
	ndev->irq = hvn->ivck->irq;

	init_waitqueue_head(&hvn->waitq);

	netif_napi_add(ndev, &hvn->napi, nv_seccan_poll, MTT_CAN_NAPI_WEIGHT);
	ret = register_candev(ndev);
	if (ret) {
		dev_err(dev, "Failed to register netdev\n");
		goto out_free_wq;
	}

	/* start the channel reset process asynchronously. until the reset
	 * process completes, any attempt to use the ivc channel will return
	 * an error (e.g., all transmits will fail).
	 */
	tegra_hv_ivc_channel_reset(hvn->ivck);

	/* the interrupt request must be the last action */
	ret = devm_request_irq(dev, ndev->irq, nv_seccan_interrupt, 0,
			       dev_name(dev), ndev);
	if (ret != 0) {
		dev_err(dev, "Could not request irq #%d\n", ndev->irq);
		goto out_unreg_netdev;
	}

	dev_info(dev, "ready\n");
	return ret;

out_unreg_netdev:
	unregister_netdev(ndev);

out_free_wq:
	netif_napi_del(&hvn->napi);

out_unreserve:
	tegra_hv_ivc_unreserve(hvn->ivck);

out_free_stats:
	free_percpu(hvn->stats);

out_free_ndev:
	free_netdev(ndev);

out_of_put:
	of_node_put(hv_dn);

	return ret;
}

static s32 tegra_hv_seccan_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct hv_seccan_priv *hvn = netdev_priv(ndev);

	platform_set_drvdata(pdev, NULL);
	devm_free_irq(dev, ndev->irq, dev);
	unregister_netdev(ndev);
	netif_napi_del(&hvn->napi);
	tegra_hv_ivc_unreserve(hvn->ivck);
	free_percpu(hvn->stats);
	free_netdev(ndev);
	dev_info(dev, "Exit from sec can driver\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tegra_hv_seccan_match[] = {
	{ .compatible = "nvidia,tegra-hv-seccan", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_seccan_match);
#endif /* CONFIG_OF */

static struct platform_driver tegra_hv_seccan_driver = {
	.probe	= tegra_hv_seccan_probe,
	.remove	= tegra_hv_seccan_remove,
	.driver	= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tegra_hv_seccan_match),
	},
};

module_platform_driver(tegra_hv_seccan_driver);

MODULE_AUTHOR("Yong Zhang <yongz@nvidia.com>");
MODULE_DESCRIPTION("socketCAN device over Tegra Hypervisor IVC channel");
MODULE_LICENSE("GPL");
