/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-instance.h>
#include <linux/tegra-ivc.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/chip-id.h>
#include "bpmp.h"

#define HSP_SHRD_SEM_1_STA	0x1b0000

static struct ivc *ivc_channels[NR_MAX_CHANNELS];
static int hv_bpmp_first_queue = -1;
static uint32_t num_ivc_queues;
static struct tegra_hv_ivc_cookie **hv_bpmp_ivc_cookies;

static int hv_bpmp_get_cookie_index(uint32_t queue_id)
{
	WARN_ON(hv_bpmp_first_queue == -1);
	if ((hv_bpmp_first_queue == -1) || (queue_id < hv_bpmp_first_queue))
		return -1;
	return (queue_id - hv_bpmp_first_queue);
}

static irqreturn_t hv_bpmp_irq_handler(int irq, void *dev_id)
{
	bpmp_handle_irq(0);

	return IRQ_HANDLED;
}

static int virt_channel_init(const struct channel_cfg *cfg,
		struct device_node *of_node)
{
	struct device_node *hv_of_node;
	int err;
	uint32_t ivc_queue;
	int index;
	struct tegra_hv_ivc_cookie *cookie;

	/* Read ivc queue numbers */
	hv_of_node = of_parse_phandle(of_node, "ivc_queue", 0);
	if (!hv_of_node) {
		pr_err("%s: Unable to find hypervisor node\n", __func__);
		return -EINVAL;
	}

	err = of_property_read_u32_index(of_node, "ivc_queue", 1,
			&ivc_queue);
	if (err != 0) {
		pr_err("%s: Failed to read start IVC queue\n",
				__func__);
		of_node_put(hv_of_node);
		return -EINVAL;
	}

	err = of_property_read_u32_index(of_node, "ivc_queue", 2,
			&num_ivc_queues);
	if (err != 0) {
		pr_err("%s: Failed to read range of IVC queues\n",
				__func__);
		of_node_put(hv_of_node);
		return -EINVAL;
	}

	hv_bpmp_first_queue = ivc_queue;

	hv_bpmp_ivc_cookies = kzalloc(sizeof(struct tegra_hv_ivc_cookie *) *
			num_ivc_queues, GFP_KERNEL);

	if (!hv_bpmp_ivc_cookies) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		of_node_put(hv_of_node);
		return -ENOMEM;
	}

	for (index = hv_bpmp_get_cookie_index(ivc_queue);
			(index >= 0) && (index < num_ivc_queues);
			index = hv_bpmp_get_cookie_index(++ivc_queue)) {

		cookie = tegra_hv_ivc_reserve(hv_of_node, ivc_queue, NULL);

		if (IS_ERR_OR_NULL(cookie)) {
			pr_err("%s: Failed to reserve ivc queue %d @index %d\n",
					__func__, index, ivc_queue);
			goto cleanup;
		}

		/* There is no compile time check for this and it's not really
		 * safe to proceed
		 */
		if (cookie->frame_size < MSG_DATA_MIN_SZ) {
			pr_err("%s: Frame size is too small\n", __func__);
			goto cleanup;
		}

		if (index >= cfg->thread_ch_0) {
			err = request_threaded_irq(
					cookie->irq,
					hv_bpmp_irq_handler, NULL,
					IRQF_NO_SUSPEND,
					"bpmp_irq_handler", &cookie);
		} else
			err = 0;

		if (err) {
			pr_err("%s: Failed to request irq %d for queue %d (index %d)\n",
					__func__, cookie->irq, ivc_queue,
					index);
			goto cleanup;
		}
		/* set ivc channel to invalid state */
		tegra_hv_ivc_channel_reset(cookie);
		hv_bpmp_ivc_cookies[index] = cookie;
	}

	if (index < 0) {
		pr_err("%s: Unable to translate ivc_queue %d\n", __func__,
				ivc_queue);
		goto cleanup;
	}

	of_node_put(hv_of_node);

	return 0;

cleanup:
	for (index = 0; index < num_ivc_queues; index++) {
		if (hv_bpmp_ivc_cookies[index]) {
			tegra_hv_ivc_unreserve(
					hv_bpmp_ivc_cookies[index]);
			hv_bpmp_ivc_cookies[index] = NULL;
		}
	}
	kfree(hv_bpmp_ivc_cookies);
	of_node_put(hv_of_node);

	return -ENOMEM;
}

static struct ivc *virt_ivc_obj(int ch)
{
	struct tegra_hv_ivc_cookie *cookie = hv_bpmp_ivc_cookies[ch];

	return tegra_hv_ivc_convert_cookie(cookie);
}

static void virt_synchronize(void)
{
	int index;
	struct tegra_hv_ivc_cookie *cookie;

	/* Ideally notified should be called in an non-interrupt message handler
	 * context. But this module does not have such a context. It only has
	 * handlers called in IRQ context and initialization code. This seems
	 * like the only option
	 */

	for (index = 0; index < num_ivc_queues; index++) {
		/* The server must be up at this point or we will get stuck */
		/* This is pretty bad, need some way to parallelize this */
		cookie = hv_bpmp_ivc_cookies[index];
		while (tegra_hv_ivc_channel_notified(cookie)) {
			cpu_relax();
			udelay(1000);
		}

		pr_debug("%s: cookie %d, channel notified\n", __func__, index);
	}
}

static int virt_connect(const struct channel_cfg *cfg,
		const struct mail_ops *ops, struct device_node *of_node)
{
	int r;

	r = virt_channel_init(cfg, of_node);
	if (r)
		return r;

	virt_synchronize();

	return 0;
}

static void tegra_hv_ivc_disconnect(void)
{
	int index;
	struct tegra_hv_ivc_cookie *cookie;

	for (index = 0; index < num_ivc_queues; index++) {
		cookie = hv_bpmp_ivc_cookies[index];
		disable_irq(cookie->irq);
		tegra_hv_ivc_channel_reset(cookie);
		pr_debug("%s: cookie %d, channel reset\n", __func__, index);
	}
}

static void tegra_hv_ivc_connect(void)
{
	int index;
	struct tegra_hv_ivc_cookie *cookie;

	for (index = 0; index < num_ivc_queues; index++) {
		cookie = hv_bpmp_ivc_cookies[index];
		enable_irq(cookie->irq);
		tegra_hv_ivc_channel_reset(cookie);
		while (tegra_hv_ivc_channel_notified(cookie)) {
			cpu_relax();
			udelay(1000);
		}
	}
}

static int native_init_prepare(void)
{
	return tegra_hsp_init();
}

static void native_inbox_irq(void *data)
{
	bpmp_handle_irq(0);
}

static int native_init_irq(unsigned int cnt)
{
	tegra_hsp_db_add_handler(HSP_MASTER_BPMP, native_inbox_irq, NULL);

	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);

	return 0;
}

static void native_ring_doorbell(int ch)
{
	tegra_hsp_db_ring(HSP_DB_BPMP);
}

static void native_synchronize(int relax)
{
	struct ivc *ivc;
	int i;

	pr_info("bpmp: synchronizing channels\n");

	for (i = 0; i < NR_MAX_CHANNELS; i++) {
		ivc = ivc_channels[i];
		if (!ivc)
			continue;

		while (tegra_ivc_channel_notified(ivc)) {
			native_ring_doorbell(i);
			if (tegra_platform_is_vdk() && relax)
				msleep(100);
		}
	}

	pr_info("bpmp: channels synchronized\n");
}

static int native_handshake(void __iomem *bpmp_base)
{
	if (tegra_platform_is_silicon())
		goto dbwait;

	pr_info("bpmp: waiting for signs of life\n");

	while (!__raw_readl(bpmp_base + HSP_SHRD_SEM_1_STA))
		msleep(500);

dbwait:
	pr_info("bpmp: waiting for handshake\n");
	while (!tegra_hsp_db_can_ring(HSP_DB_BPMP)) {
		if (tegra_platform_is_vdk())
			msleep(100);
	}

	pr_info("bpmp: handshake completed\n");

	return 0;
}

static void tegra_virt_suspend(void)
{
	tegra_hv_ivc_disconnect();
}

static void tegra_virt_resume(void)
{
	tegra_hv_ivc_connect();
}

static void native_resume(void)
{
	struct ivc *ivc;
	int i;

	tegra_hsp_db_enable_master(HSP_MASTER_BPMP);

	pr_info("bpmp: waiting for handshake\n");
	while (!tegra_hsp_db_can_ring(HSP_DB_BPMP))
		;

	for (i = 0; i < NR_MAX_CHANNELS; i++) {
		ivc = ivc_channels[i];
		if (ivc)
			tegra_ivc_channel_reset(ivc);
	}

	native_synchronize(0);
}

static void native_notify(struct ivc *ivc)
{
}

static int native_single_init(int ch,
		void __iomem *ma_page, void __iomem *sl_page)
{
	struct ivc *ivc;
	uintptr_t rx_base;
	uintptr_t tx_base;
	size_t msg_sz;
	size_t que_sz;
	size_t hdr_sz;
	int r;

	msg_sz = tegra_ivc_align(MSG_DATA_MIN_SZ);
	hdr_sz = tegra_ivc_total_queue_size(0);
	que_sz = tegra_ivc_total_queue_size(msg_sz);

	rx_base = (uintptr_t)(sl_page + ch * que_sz);
	tx_base = (uintptr_t)(ma_page + ch * que_sz);

	ivc = kzalloc(sizeof(*ivc), GFP_KERNEL);
	if (!ivc)
		return -ENOMEM;

	r = tegra_ivc_init(ivc, rx_base, tx_base,
			1, msg_sz, NULL, native_notify);
	if (r) {
		pr_err("tegra_ivc_init() ch %d returned %d\n", ch, r);
		WARN_ON(1);
		return r;
	}

	ivc_channels[ch] = ivc;

	tegra_ivc_channel_reset(ivc);
	native_ring_doorbell(ch);

	return 0;
}

static int native_channel_init(unsigned int chmask,
		void __iomem *ma_page, void __iomem *sl_page)
{
	unsigned int i;
	int r;

	while (chmask) {
		i = __ffs(chmask);
		chmask &= ~(1 << i);

		r = native_single_init(i, ma_page, sl_page);
		if (r)
			return r;
	}

	native_synchronize(1);

	return 0;
}

static struct ivc *native_ivc_obj(int ch)
{
	return ivc_channels[ch];
}

static int native_connect(const struct channel_cfg *cfg,
		const struct mail_ops *ops, struct device_node *of_node)
{
	void __iomem *bpmp_base;
	void __iomem *ma_page;
	void __iomem *sl_page;
	int r;

	bpmp_base = of_iomap(of_node, 0);
	if (!bpmp_base)
		return -ENODEV;

	ma_page = of_iomap(of_node, 1);
	if (!ma_page)
		return -ENODEV;

	sl_page = of_iomap(of_node, 2);
	if (!sl_page)
		return -ENODEV;

	r = native_handshake(bpmp_base);
	if (r)
		return r;

	return native_channel_init(cfg->channel_mask, ma_page, sl_page);
}

static bool ivc_rx_ready(const struct mail_ops *ops, int ch)
{
	struct ivc *ivc;
	void *frame;
	bool ready;

	ivc = ops->ivc_obj(ch);
	frame = tegra_ivc_read_get_next_frame(ivc);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ib = ready ? frame : NULL;

	return ready;
}

static bool bpmp_master_acked(const struct mail_ops *ops, int ch)
{
	return ivc_rx_ready(ops, ch);
}

static bool bpmp_slave_signalled(const struct mail_ops *ops, int ch)
{
	return ivc_rx_ready(ops, ch);
}

static void bpmp_free_master(const struct mail_ops *ops, int ch)
{
	struct ivc *ivc;

	ivc = ops->ivc_obj(ch);
	if (tegra_ivc_read_advance(ivc))
		WARN_ON(1);
}

static void bpmp_signal_slave(const struct mail_ops *ops, int ch)
{
	struct ivc *ivc;

	ivc = ops->ivc_obj(ch);
	if (tegra_ivc_write_advance(ivc))
		WARN_ON(1);
}

static bool bpmp_master_free(const struct mail_ops *ops, int ch)
{
	struct ivc *ivc;
	void *frame;
	bool ready;

	ivc = ops->ivc_obj(ch);
	frame = tegra_ivc_write_get_next_frame(ivc);
	ready = !IS_ERR_OR_NULL(frame);
	channel_area[ch].ob = ready ? frame : NULL;

	return ready;
}

static void bpmp_return_data(const struct mail_ops *ops,
		int ch, int code, void *data, int sz)
{
	const int flags = channel_area[ch].ib->flags;
	void __iomem *addr;
	struct ivc *ivc;
	struct mb_data *frame;
	int r;

	if (sz > MSG_DATA_MIN_SZ) {
		WARN_ON(1);
		return;
	}

	ivc = ops->ivc_obj(ch);
	r = tegra_ivc_read_advance(ivc);
	WARN_ON(r);

	if (!(flags & DO_ACK))
		return;

	frame = tegra_ivc_write_get_next_frame(ivc);
	if (IS_ERR_OR_NULL(frame)) {
		WARN_ON(1);
		return;
	}

	frame->code = code;
	addr = (void __iomem*) frame->data;
	memcpy_toio(addr, data, sz);
	r = tegra_ivc_write_advance(ivc);
	WARN_ON(r);

	if ((flags & RING_DOORBELL) && ops->ring_doorbell)
		ops->ring_doorbell(ch);
}

const struct mail_ops t186_hv_mail_ops = {
	.connect = virt_connect,
	.ivc_obj = virt_ivc_obj,
	.suspend = tegra_virt_suspend,
	.resume = tegra_virt_resume,
	.master_free = bpmp_master_free,
	.free_master = bpmp_free_master,
	.master_acked = bpmp_master_acked,
	.signal_slave = bpmp_signal_slave,
	.slave_signalled = bpmp_slave_signalled,
	.return_data = bpmp_return_data
};

const struct mail_ops t186_native_mail_ops = {
	.init_prepare = native_init_prepare,
	.init_irq = native_init_irq,
	.connect = native_connect,
	.resume = native_resume,
	.ivc_obj = native_ivc_obj,
	.master_free = bpmp_master_free,
	.free_master = bpmp_free_master,
	.master_acked = bpmp_master_acked,
	.signal_slave = bpmp_signal_slave,
	.slave_signalled = bpmp_slave_signalled,
	.ring_doorbell = native_ring_doorbell,
	.return_data = bpmp_return_data
};
