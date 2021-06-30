/*
 * IVC based library for AUDIO server
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/tegra-ivc.h>
#include <linux/spinlock.h>
#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>

#include "tegra_virt_alt_ivc.h"
#include "tegra_virt_alt_ivc_common.h"

static struct nvaudio_ivc_ctxt *saved_ivc_ctxt;

static void nvaudio_ivc_deinit(struct nvaudio_ivc_ctxt *ictxt);
static int nvaudio_ivc_init(struct nvaudio_ivc_ctxt *ictxt);

int nvaudio_ivc_send_retry(struct nvaudio_ivc_ctxt *ictxt,
		struct nvaudio_ivc_msg *msg, int size)
{
	int err = 0;
	int dcnt = 50;

	if (!ictxt || !ictxt->ivck || !msg || !size)
		return -EINVAL;

	err = nvaudio_ivc_send(ictxt, msg, size);

	while (err < 0 && dcnt--) {
		udelay(100);
		err = nvaudio_ivc_send(ictxt, msg, size);
	}
	return (dcnt < 0) ? -ETIMEDOUT : err;

}
EXPORT_SYMBOL_GPL(nvaudio_ivc_send_retry);

int nvaudio_ivc_send(struct nvaudio_ivc_ctxt *ictxt,
		struct nvaudio_ivc_msg *msg, int size)
{
	int len = 0;
	unsigned long flags = 0;
	int err = 0;
	int dcnt = 50;

	if (!ictxt || !ictxt->ivck || !msg || !size)
		return -EINVAL;

	while (tegra_hv_ivc_channel_notified(ictxt->ivck) != 0) {
		dev_err(ictxt->dev, "channel notified returns non zero\n");
		dcnt--;
		udelay(100);
		if (!dcnt)
			return -EIO;
	}

	spin_lock_irqsave(&ictxt->ivck_tx_lock, flags);

	if (!tegra_hv_ivc_can_write(ictxt->ivck)) {
		err = -EBUSY;
		goto fail;
	}

	len = tegra_hv_ivc_write(ictxt->ivck, msg, size);
	if (len != size) {
		pr_err("%s: write Error\n", __func__);
		err = -EIO;
		goto fail;
	}

	err = len;

fail:
	spin_unlock_irqrestore(&ictxt->ivck_tx_lock, flags);
	return err;
}
EXPORT_SYMBOL_GPL(nvaudio_ivc_send);

int nvaudio_ivc_send_receive(struct nvaudio_ivc_ctxt *ictxt,
			struct nvaudio_ivc_msg *rx_msg, int size)
{

	int len = 0;
	unsigned long flags = 0, flags1 = 0;
	int err = 0;
	int dcnt = 50;
	int status = -1;
	struct nvaudio_ivc_msg *msg = rx_msg;

	if (!ictxt || !ictxt->ivck || !msg || !size)
		return -EINVAL;

	while (tegra_hv_ivc_channel_notified(ictxt->ivck) != 0) {
		dev_err(ictxt->dev, "channel notified returns non zero\n");
		dcnt--;
		udelay(100);
		if (!dcnt)
			return -EIO;
	}

	spin_lock_irqsave(&ictxt->ivck_rx_lock, flags);
	spin_lock_irqsave(&ictxt->ivck_tx_lock, flags1);

	if (!tegra_hv_ivc_can_write(ictxt->ivck)) {
		err = -EBUSY;
		spin_unlock_irqrestore(&ictxt->ivck_tx_lock, flags1);
		goto fail;
	}

	len = tegra_hv_ivc_write(ictxt->ivck, msg, size);
	if (len != size) {
		pr_err("%s: write Error\n", __func__);
		err = -EIO;
		spin_unlock_irqrestore(&ictxt->ivck_tx_lock, flags1);
		goto fail;
	}
	spin_unlock_irqrestore(&ictxt->ivck_tx_lock, flags1);

	err = readx_poll_timeout_atomic(tegra_hv_ivc_can_read, ictxt->ivck,
			status, status, 10, NVAUDIO_IVC_WAIT_TIMEOUT);

	if (err == -ETIMEDOUT) {
		pr_err("%s: Waited too long for msg reply\n", __func__);
		goto fail;
	}

	/* Message available */
	memset(rx_msg, 0, sizeof(struct nvaudio_ivc_msg));
	len = tegra_hv_ivc_read(ictxt->ivck,
				rx_msg,
				sizeof(struct nvaudio_ivc_msg));
	if (len != sizeof(struct nvaudio_ivc_msg)) {
		dev_err(ictxt->dev, "IVC read failure (msg size error)\n");
		err = -1;
		goto fail;
	}
	err = len;

fail:
	spin_unlock_irqrestore(&ictxt->ivck_rx_lock, flags);

	return err;

}
EXPORT_SYMBOL_GPL(nvaudio_ivc_send_receive);

/* Every communication with the server is identified
 * with this ivc context.
 * There can be one outstanding request to the server per
 * ivc context.
 */
struct nvaudio_ivc_ctxt *nvaudio_ivc_alloc_ctxt(struct device *dev)
{
	struct nvaudio_ivc_ctxt *ictxt = NULL;

	if (saved_ivc_ctxt)
		return saved_ivc_ctxt;

	ictxt = devm_kzalloc(dev, sizeof(struct nvaudio_ivc_ctxt),
			GFP_KERNEL);
	if (ictxt == NULL) {
		dev_err(dev, "unable to allocate mem for ivc context\n");
		return NULL;
	}

	saved_ivc_ctxt = ictxt;
	ictxt->dev = dev;
	ictxt->timeout = 250; /* Not used in polling */
	if (nvaudio_ivc_init(ictxt) != 0) {
		dev_err(dev, "nvaudio_ivc_init failed\n");
		goto fail;
	}

	spin_lock_init(&ictxt->ivck_rx_lock);
	spin_lock_init(&ictxt->ivck_tx_lock);

	tegra_hv_ivc_channel_reset(ictxt->ivck);

	return ictxt;
fail:
	nvaudio_ivc_free_ctxt(dev);
	return NULL;
}
EXPORT_SYMBOL_GPL(nvaudio_ivc_alloc_ctxt);

/* hook to domain destroy */
void nvaudio_ivc_free_ctxt(struct device *dev)
{
	if (saved_ivc_ctxt) {
		nvaudio_ivc_deinit(saved_ivc_ctxt);
		devm_kfree(dev, saved_ivc_ctxt);
		saved_ivc_ctxt = NULL;
	}
}
EXPORT_SYMBOL_GPL(nvaudio_ivc_free_ctxt);

static void nvaudio_ivc_deinit(struct nvaudio_ivc_ctxt *ictxt)
{
	if (ictxt)
		tegra_hv_ivc_unreserve(ictxt->ivck);
}

static int nvaudio_ivc_init(struct nvaudio_ivc_ctxt *ictxt)
{
	int err, ivc_queue;
	struct device_node *dn, *hv_dn;
	struct device *dev = ictxt->dev;
	struct tegra_hv_ivc_cookie *ivck = NULL;

	dn = dev->of_node;
	if (dn == NULL) {
		dev_err(dev, "No OF data\n");
		return -EINVAL;
	}

	hv_dn = of_parse_phandle(dn, "ivc_queue", 0);
	if (hv_dn == NULL) {
		dev_err(dev, "Failed to parse phandle of ivc prop\n");
		return -EINVAL;
	}

	err = of_property_read_u32_index(dn, "ivc_queue", 1, &ivc_queue);
	if (err != 0) {
		dev_err(dev, "Failed to read IVC property ID\n");
		of_node_put(hv_dn);
		return -EINVAL;
	}

	ivck = tegra_hv_ivc_reserve(hv_dn, ivc_queue, NULL);

	if (IS_ERR_OR_NULL(ivck)) {
		dev_err(dev, "Failed to reserve ivc queue %d\n", ivc_queue);
		if (ivck == ERR_PTR(-EPROBE_DEFER))
			return -EPROBE_DEFER;
		else
			return -EINVAL;
	}

	of_node_put(hv_dn);

	ictxt->ivc_queue = ivc_queue;
	ictxt->ivck = ivck;

	return 0;
}

MODULE_LICENSE("GPL");
