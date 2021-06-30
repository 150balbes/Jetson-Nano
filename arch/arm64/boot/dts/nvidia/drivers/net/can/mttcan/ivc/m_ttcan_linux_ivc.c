/*
 * "drivers/staging/mttcan/m_ttcan_linux_ivc.c"
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Parts of code are taken from "drivers/staging/mttcan/m_ttcan_linux.c"
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

#include <linux/platform_device.h>
#include "m_ttcan.h"

#define KHZ		1000
#define MHZ		(1000 * KHZ)
#define MTTCAN_CLK	(40 * MHZ)

static const struct can_bittiming_const mttcan_normal_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 255,
	.tseg2_min = 0,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 127,
	.sjw_max = 127,
	.brp_min = 1,
	.brp_max = 511,
	.brp_inc = 1,
};

static const struct can_bittiming_const mttcan_data_bittiming_const = {
	.name = KBUILD_MODNAME,
	.tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
	.tseg1_max = 31,
	.tseg2_min = 0,		/* Time segment 2 = phase_seg2 */
	.tseg2_max = 15,
	.sjw_max = 15,
	.brp_min = 1,
	.brp_max = 15,
	.brp_inc = 1,
};

static struct platform_device_id mttcan_id_table[] = {
	[0] = {
	       .name = "mttcan_ivc",
	       .driver_data = 0,
	       }, {
		   }
};

MODULE_DEVICE_TABLE(platform, mttcan_id_table);

static const struct of_device_id mttcan_of_table[] = {
	{ .compatible = "bosch,mttcan-ivc", .data = &mttcan_id_table[0]},
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mttcan_of_table);

static int mttcan_read_rcv_list(struct net_device *dev,
				struct list_head *rcv)
{
	int rec_msgs = 0;
	unsigned long flags;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_rx_msg_list *rx;
	struct net_device_stats *stats = &dev->stats;
	struct list_head *cur, *next, rx_q;

	if (list_empty(rcv))
		return 0;

	INIT_LIST_HEAD(&rx_q);

	spin_lock_irqsave(&priv->ttcan->lock, flags);
	priv->ttcan->rxb_mem = 0;
	priv->ttcan->list_status &= ~(BUFFER & 0xFF);
	list_splice_init(rcv, &rx_q);
	spin_unlock_irqrestore(&priv->ttcan->lock, flags);

	list_for_each_safe(cur, next, &rx_q) {
		struct sk_buff *skb;
		struct canfd_frame *fd_frame;
		struct can_frame *frame;
		list_del_init(cur);

		rx = list_entry(cur, struct ttcan_rx_msg_list, recv_list);
		if (rx->msg.flags & CAN_FD_FLAG) {
			skb = alloc_canfd_skb(dev, &fd_frame);
			if (!skb) {
				stats->rx_dropped++;
				return 0;
			}
			memcpy(fd_frame, &rx->msg, sizeof(struct canfd_frame));
			stats->rx_bytes += fd_frame->len;
		} else {
			skb = alloc_can_skb(dev, &frame);
			if (!skb) {
				stats->rx_dropped++;
				return 0;
			}
			frame->can_id =  rx->msg.can_id;
			frame->can_dlc = rx->msg.d_len;
			memcpy(frame->data, &rx->msg.data, frame->can_dlc);
			stats->rx_bytes += frame->can_dlc;
		}

		kfree(rx);
		netif_receive_skb(skb);
		stats->rx_packets++;
		rec_msgs++;
	}
	return rec_msgs;
}

static int mttcan_state_change(struct net_device *dev,
			       enum can_state error_type, u32 ecr)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	struct net_device_stats *stats = &dev->stats;
	struct can_frame *cf;
	struct sk_buff *skb;
	struct can_berr_counter bec;

	/* propagate the error condition to the CAN stack */
	skb = alloc_can_err_skb(dev, &cf);
	if (unlikely(!skb))
		return 0;

	bec.rxerr = (ecr & MTT_ECR_REC_MASK) >> MTT_ECR_REC_SHIFT;
	bec.txerr = (ecr & MTT_ECR_TEC_MASK) >> MTT_ECR_TEC_SHIFT;

	switch (error_type) {
	case CAN_STATE_ERROR_WARNING:
		/* error warning state */
		priv->can.can_stats.error_warning++;
		priv->can.state = CAN_STATE_ERROR_WARNING;
		cf->can_id |= CAN_ERR_CRTL;
		cf->data[1] = (bec.txerr > bec.rxerr) ?
		    CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;

		break;
	case CAN_STATE_ERROR_PASSIVE:
		/* error passive state */
		priv->can.can_stats.error_passive++;
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
		cf->can_id |= CAN_ERR_CRTL;
		if (ecr & MTT_ECR_RP_MASK)
			cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
		if (bec.txerr > 127)
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;

		cf->data[6] = bec.txerr;
		cf->data[7] = bec.rxerr;
		break;
	case CAN_STATE_BUS_OFF:
		/* bus-off state */
		priv->can.can_stats.bus_off++;
		priv->can.state = CAN_STATE_BUS_OFF;
		cf->can_id |= CAN_ERR_BUSOFF;
		can_bus_off(dev);
		break;
	default:
		break;
	}
	netif_receive_skb(skb);
	stats->rx_packets++;
	stats->rx_bytes += cf->can_dlc;

	return 1;
}

static void mttcan_tx_complete(struct net_device *dev, u32 completed_tx)
{
	u32 msg_no;

	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_controller *ttcan = priv->ttcan;
	struct net_device_stats *stats = &dev->stats;

	/* apply mask to consider only active CAN Tx transactions */
	completed_tx &= ttcan->tx_object;

	while (completed_tx) {
		msg_no = ffs(completed_tx) - 1;
		can_get_echo_skb(dev, msg_no);
		can_led_event(dev, CAN_LED_EVENT_TX);
		clear_bit(msg_no, &ttcan->tx_object);
		stats->tx_packets++;
		stats->tx_bytes += ttcan->tx_buf_dlc[msg_no];
		completed_tx &= ~(1U << msg_no);
	}

	if (netif_queue_stopped(dev))
		netif_wake_queue(dev);
}

static int mttcan_do_set_bittiming(struct net_device *dev)
{
	u32 bittiming, dbittiming;
	struct mttcan_priv *priv = netdev_priv(dev);
	const struct can_bittiming *bt = &priv->can.bittiming;
	const struct can_bittiming *dbt = &priv->can.data_bittiming;

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		priv->ttcan->bt_config.fd_flags = CAN_FD_FLAG  | CAN_BRS_FLAG;
	else
		priv->ttcan->bt_config.fd_flags = 0;

	bittiming = bt->sjw + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2;
	if (bt->brp)
		bittiming = bittiming / bt->brp;

	/* CAN IVC driver only support 500KHz bit rate for now */
	if ((MTTCAN_CLK / bittiming) != (500 * KHZ)) {
		netdev_err(dev, "Normal bitrate configuration failed\n");
		return -EIO;
	}

	if (priv->ttcan->bt_config.fd_flags & CAN_FD_FLAG) {
		dbittiming = dbt->sjw + dbt->prop_seg
			+ dbt->phase_seg1 + dbt->phase_seg2;
		if (dbt->brp)
			dbittiming = dbittiming / dbt->brp;

		/* CAN IVC driver only support 2MHz data bit rate for now */
		if ((MTTCAN_CLK / dbittiming) != (2 * MHZ)) {
			netdev_err(dev, "Fast bitrate configuration failed\n");
			return -EIO;
		}
	}

	netdev_info(priv->dev, "Bitrate set\n");
	return 0;
}

static int mttcan_ivc_enable(struct mttcan_priv *priv, bool flag)
{
	int status;
	struct ttcan_ivc_msg msg;
	struct ivc_ttcanfd_frame fr;

	fr.cmdid = MTTCAN_CMD_CAN_ENABLE;
	fr.ext_cmdid = flag;
	msg.length = sizeof(struct ivc_ttcanfd_frame);
	msg.data = (void *)&fr;

	status = mbox_send_message(priv->mbox, (void *)&msg);
	if (status < 0) {
		dev_err(priv->device, "mbox_send_message failed %d\n", status);
		return -EBUSY;
	}
	return 0;
}

static void mttcan_start(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_controller *ttcan = priv->ttcan;
	u32 psr = 0;

	/* set bit timing and start controller */
	mttcan_do_set_bittiming(dev);

	/* Enable CAN messages over IVC */
	mttcan_ivc_enable(priv, true);

	/* Set current state of CAN controller *
	 * It is assumed the controller is reset durning probing time
	 * It should be in sane state at first start but not guaranteed
	 */
	psr = ttcan->proto_state;

	if (psr & MTT_PSR_BO_MASK) {
		/* Bus off */
		priv->can.state = CAN_STATE_BUS_OFF;
		can_bus_off(dev);
	} else if (psr & MTT_PSR_EP_MASK) {
		/* Error Passive */
		priv->can.state = CAN_STATE_ERROR_PASSIVE;
	} else if (psr & MTT_PSR_EW_MASK) {
		/* Error Warning */
		priv->can.state = CAN_STATE_ERROR_WARNING;
	} else {
		/* Error Active */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
	}
}

static void mttcan_stop(struct mttcan_priv *priv)
{
	/* Disable CAN messages over IVC */
	mttcan_ivc_enable(priv, false);

	priv->can.state = CAN_STATE_STOPPED;
}


static int mttcan_set_mode(struct net_device *dev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		mttcan_start(dev);
		netif_wake_queue(dev);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static struct net_device *alloc_mttcan_dev(void)
{
	struct net_device *dev;
	struct mttcan_priv *priv;

	dev = alloc_candev(sizeof(struct mttcan_priv), MTT_CAN_TX_OBJ_NUM);
	if (!dev)
		return NULL;

	/* TODO:- check if we need to disable local loopback */
	dev->flags = (IFF_NOARP | IFF_ECHO);

	priv = netdev_priv(dev);

	priv->dev = dev;
	priv->can.bittiming_const = &mttcan_normal_bittiming_const;
	priv->can.data_bittiming_const = &mttcan_data_bittiming_const;
	priv->can.do_set_bittiming = mttcan_do_set_bittiming;
	priv->can.do_set_mode = mttcan_set_mode;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_FD |
		CAN_CTRLMODE_BERR_REPORTING;

	return dev;
}

static int process_rx_mesg_ivc(struct ttcan_controller *ttcan, u32 *addr)
{
	struct ttcanfd_frame ttcanfd;
	ttcan_read_rx_msg_ram(ttcan, (u64)addr, &ttcanfd);
	return add_msg_controller_list(ttcan, &ttcanfd, &ttcan->rx_b, BUFFER);
}

static void mttcan_ivc_rcv_msg(struct mbox_client *cl, void *mssg)
{
	struct net_device *dev = (struct net_device *)dev_get_drvdata(cl->dev);
	struct net_device_stats *stats = &dev->stats;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct ttcan_ivc_msg *msg = mssg;
	struct ivc_ttcanfd_frame *fr = (struct ivc_ttcanfd_frame *)msg->data;

	switch (fr->cmdid) {
	case MTTCAN_MSG_RX:
		if (process_rx_mesg_ivc(priv->ttcan,
				&fr->payload.data[0]) == 0) {
			mttcan_read_rcv_list(dev, &priv->ttcan->rx_b);
			memcpy(&priv->resp, msg->data, sizeof(priv->resp));
		} else {
			stats->rx_dropped++;
			netdev_err(dev, "Rx message dropped\n");
		}
		break;

	case MTTCAN_MSG_TX_COMPL:
		mttcan_tx_complete(dev, fr->payload.data[0]);
		break;

	case MTTCAN_MSG_STAT_CHG:
		if (!mttcan_state_change(dev, fr->payload.data[0],
				fr->payload.data[1]))
			netdev_err(dev, "State change failed\n");
		break;

	case MTTCAN_MSG_BERR_CHG:
		/* TODO: Handle TX event buffer */
		break;

	case MTTCAN_MSG_RX_LOST_FRAME:
		/* TODO: Handle TX event buffer */
		break;

	case MTTCAN_MSG_TXEVT:
		/* TODO: Handle TX event buffer */
		break;

	default:
		netdev_err(dev, "Invalid IVC CMD ID (%#x)\n", fr->cmdid);
		/* TODO: Update error count */
		break;
	}
	complete(&priv->xfer_completion);
}

static int mttcan_ivc_send_req(struct mttcan_priv *priv,
	       struct canfd_frame *frame,
	       int msg_no)
{
	int status;
	struct ttcan_ivc_msg msg;
	struct ivc_ttcanfd_frame fr;

	if (msg_no < 0)
		return -ENOMEM;

	fr.cmdid = MTTCAN_MSG_TX;
	fr.ext_cmdid = msg_no;
	memcpy(&fr.payload.data[0], frame, sizeof(struct canfd_frame));
	msg.length = sizeof(struct ivc_ttcanfd_frame);
	msg.data = (void *)&fr;

	status = mbox_send_message(priv->mbox, (void *)&msg);
	if (status < 0) {
		dev_err(priv->device, "mbox_send_message failed %d\n", status);
		return -EBUSY;
	}
	priv->ttcan->tx_buf_dlc[msg_no] = frame->len;
	return msg_no;
}

static int mttcan_open(struct net_device *dev)
{
	int err;

	err = open_candev(dev);
	if (err) {
		netdev_err(dev, "failed to open can device\n");
		return err;
	}

	can_led_event(dev, CAN_LED_EVENT_OPEN);

	mttcan_start(dev);
	netif_start_queue(dev);

	return 0;
}

static int mttcan_close(struct net_device *dev)
{
	struct mttcan_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	mttcan_stop(priv);
	close_candev(dev);

	can_led_event(dev, CAN_LED_EVENT_STOP);
	return 0;
}

static netdev_tx_t mttcan_start_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	int msg_no = -1;
	int err;
	struct mttcan_priv *priv = netdev_priv(dev);
	struct canfd_frame *frame = (struct canfd_frame *)skb->data;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	if (can_is_canfd_skb(skb))
		frame->flags |= CAN_FD_FLAG;

	msg_no = ffs(~priv->ttcan->tx_object) - 1;
	if (msg_no < 0) {
		netdev_warn(dev, "No Tx space left\n");
		netif_stop_queue(dev);
		smp_mb();
		return NETDEV_TX_BUSY;
	}

	can_put_echo_skb(skb, dev, msg_no);

	/* State management for Tx complete/cancel processing */
	if (test_and_set_bit(msg_no, &priv->ttcan->tx_object))
		netdev_err(dev, "Writing to occupied echo_skb buffer\n");
	clear_bit(msg_no, &priv->ttcan->tx_obj_cancelled);

	err = mttcan_ivc_send_req(priv, frame, msg_no);
	if (err < 0) {
		netdev_warn(dev, "Tx IVC failed\n");
		netif_stop_queue(dev);
		smp_mb();
		return NETDEV_TX_BUSY;
	}

	return NETDEV_TX_OK;
}

static int mttcan_change_mtu(struct net_device *dev, int new_mtu)
{
	if (dev->flags & IFF_UP)
		return -EBUSY;

	if (new_mtu != CANFD_MTU)
		dev->mtu = new_mtu;
	return 0;
}

static const struct net_device_ops mttcan_netdev_ops = {
	.ndo_open = mttcan_open,
	.ndo_stop = mttcan_close,
	.ndo_start_xmit = mttcan_start_xmit,
	.ndo_change_mtu = mttcan_change_mtu,
};

static int register_mttcan_dev(struct net_device *dev)
{
	int err;

	dev->netdev_ops = &mttcan_netdev_ops;
	err = register_candev(dev);
	if (!err)
		devm_can_led_init(dev);

	return err;
}

static int mttcan_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct net_device *dev;
	struct mttcan_priv *priv;
	const struct of_device_id *match;
	struct device_node *np;

	match = of_match_device(mttcan_of_table, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Failed to find matching dt id\n");
		dev_err(&pdev->dev, "probe failed\n");
		return -EINVAL;
	}

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "No valid device node, probe failed\n");
		return -EINVAL;
	}

	dev = alloc_mttcan_dev();
	if (!dev) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "CAN device allocation failed\n");
		goto exit;
	}

	priv = netdev_priv(dev);

	/* allocate the mttcan device */

	priv->device = &pdev->dev;

	priv->can.clock.freq = MTTCAN_CLK;

	/* allocate controller struct memory and set fields */
	priv->ttcan =
	    devm_kzalloc(priv->device, sizeof(struct ttcan_controller),
			 GFP_KERNEL);
	if (!priv->ttcan) {
		dev_err(priv->device,
			"cannot allocate memory for ttcan_controller\n");
		goto exit_free_device;
	}
	memset(priv->ttcan, 0, sizeof(struct ttcan_controller));
	priv->ttcan->id = priv->instance;
	INIT_LIST_HEAD(&priv->ttcan->rx_b);

	platform_set_drvdata(pdev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	ret = register_mttcan_dev(dev);
	if (ret) {
		dev_err(&pdev->dev, "registering %s failed (err=%d)\n",
			KBUILD_MODNAME, ret);
		goto exit_free_device;
	}

	/* Configure mailbox for IVC to AON */
	priv->cl.dev = &pdev->dev;
	priv->cl.tx_block = false;
	priv->cl.tx_tout = TX_BLOCK_PERIOD;
	priv->cl.knows_txdone = false;
	priv->cl.rx_callback = mttcan_ivc_rcv_msg;
	priv->mbox = mbox_request_channel(&priv->cl, 0);
	if (IS_ERR(priv->mbox)) {
		dev_warn(&pdev->dev,
			 "can't get mailbox channel (%d)\n",
			 (int)PTR_ERR(priv->mbox));
		ret = PTR_ERR(priv->mbox);
		goto exit_unreg_candev;
	}
	init_completion(&priv->xfer_completion);

	dev_info(&dev->dev, "%s device registered\n", KBUILD_MODNAME);
	return 0;

exit_unreg_candev:
	unregister_candev(dev);
exit_free_device:
	platform_set_drvdata(pdev, NULL);
	free_candev(dev);
exit:
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}

static int mttcan_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(dev);

	mbox_free_channel(priv->mbox);

	dev_info(&dev->dev, "%s\n", __func__);

	unregister_candev(dev);
	platform_set_drvdata(pdev, NULL);
	free_candev(dev);

	return 0;
}

#ifdef CONFIG_PM
static int mttcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(ndev);

	if (netif_running(ndev)) {
		netif_stop_queue(ndev);
		netif_device_detach(ndev);
	}

	if (ndev->flags & IFF_UP)
		mttcan_stop(priv);

	priv->can.state = CAN_STATE_SLEEPING;
	return 0;
}

static int mttcan_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct mttcan_priv *priv = netdev_priv(ndev);

	if (ndev->flags & IFF_UP)
		mttcan_start(ndev);

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	if (netif_running(ndev)) {
		netif_device_attach(ndev);
		netif_start_queue(ndev);
	}
	return 0;
}
#endif

static struct platform_driver mttcan_plat_driver_ivc = {
	.driver = {
		   .name = KBUILD_MODNAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(mttcan_of_table),
		   },
	.probe = mttcan_probe,
	.remove = mttcan_remove,
#ifdef CONFIG_PM
	.suspend = mttcan_suspend,
	.resume = mttcan_resume,
#endif
	.id_table = mttcan_id_table,
};

module_platform_driver(mttcan_plat_driver_ivc);
MODULE_AUTHOR("Manoj Chourasia <mchourasia@nvidia.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Platform CAN bus IVC driver for Bosch M_TTCAN controller");
