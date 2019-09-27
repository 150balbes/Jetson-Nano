/*
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/io.h>
#include <linux/of_address.h>
#include <soc/tegra/doorbell.h>
#include "bpmp.h"

static void __iomem *arb_sema;

#define CPU_OB_DOORBELL		4

#define TRIGGER_OFFSET		0x000
#define RESULT_OFFSET(id)	(0xc00 + id * 4)
#define TRIGGER_ID_SHIFT	16
#define TRIGGER_CMD_GET		4

#define STA_OFFSET		0
#define SET_OFFSET		4
#define CLR_OFFSET		8

/*
 * How the token bits are interpretted
 *
 * SL_SIGL (b00): slave ch in signalled state
 * SL_QUED (b01): slave ch is in queue
 * MA_FREE (b10): master ch is free
 * MA_ACKD (b11): master ch is acked
 *
 * Ideally, the slave should only set bits while the
 * master do only clear them. But there is an exception -
 * see bpmp_ack_master()
 */
#define CH_MASK(ch)	(0x3 << ((ch) * 2))
#define SL_SIGL(ch)	(0x0 << ((ch) * 2))
#define SL_QUED(ch)	(0x1 << ((ch) * 2))
#define MA_FREE(ch)	(0x2 << ((ch) * 2))
#define MA_ACKD(ch)	(0x3 << ((ch) * 2))

static u32 bpmp_ch_sta(int ch)
{
	return __raw_readl(arb_sema + STA_OFFSET) & CH_MASK(ch);
}

static bool bpmp_master_free(const struct mail_ops *ops, int ch)
{
	return bpmp_ch_sta(ch) == MA_FREE(ch);
}

static bool bpmp_slave_signalled(const struct mail_ops *ops, int ch)
{
	return bpmp_ch_sta(ch) == SL_SIGL(ch);
}

static bool bpmp_master_acked(const struct mail_ops *ops, int ch)
{
	return bpmp_ch_sta(ch) == MA_ACKD(ch);
}

static void bpmp_signal_slave(const struct mail_ops *ops, int ch)
{
	__raw_writel(CH_MASK(ch), arb_sema + CLR_OFFSET);
}

static void bpmp_ack_master(int ch, int flags)
{
	__raw_writel(MA_ACKD(ch), arb_sema + SET_OFFSET);

	if (flags & DO_ACK)
		return;

	/*
	 * We have to violate the bit modification rule while
	 * moving from SL_QUED to MA_FREE (DO_ACK not set) so that
	 * the channel won't be in ACKD state forever.
	 */
	__raw_writel(MA_ACKD(ch) ^ MA_FREE(ch), arb_sema + CLR_OFFSET);
}

/* MA_ACKD to MA_FREE */
static void bpmp_free_master(const struct mail_ops *ops, int ch)
{
	__raw_writel(MA_ACKD(ch) ^ MA_FREE(ch), arb_sema + CLR_OFFSET);
}

static void bpmp_ring_doorbell(int ch)
{
	tegra_ring_doorbell(CPU_OB_DOORBELL);
}

static void bpmp_return_data(const struct mail_ops *ops,
		int ch, int code, void *data, int sz)
{
	struct mb_data *p;
	int flags;

	if (sz > MSG_DATA_MIN_SZ) {
		WARN_ON(1);
		return;
	}

	p = channel_area[ch].ob;
	p->code = code;
	memcpy(p->data, data, sz);

	flags = channel_area[ch].ib->flags;
	bpmp_ack_master(ch, flags);
	if (flags & RING_DOORBELL)
		bpmp_ring_doorbell(ch);
}

static void bpmp_doorbell_handler(void *data)
{
	unsigned int chidx = (long)data;

	bpmp_handle_irq(chidx);
}

static int bpmp_init_irq(unsigned int cnt)
{
	unsigned int i;
	long chidx;
	int r;

	for (i = 0; i < cnt; i++) {
		chidx = i;
		r = tegra_register_doorbell_handler(i, bpmp_doorbell_handler,
						   (void *)chidx);
		if (r)
			return r;
	}

	return 0;
}

/* Channel area is setup by BPMP before signalling handshake */
static u32 bpmp_channel_area(void __iomem *atomics, int ch)
{
	u32 a;

	writel(ch << TRIGGER_ID_SHIFT | TRIGGER_CMD_GET,
			atomics + TRIGGER_OFFSET);
	a = readl(atomics + RESULT_OFFSET(ch));

	return a;
}

static int bpmp_connect(const struct channel_cfg *cfg,
		const struct mail_ops *ops, struct device_node *of_node)
{
	unsigned int chmask;
	void __iomem *atomics;
	uint32_t area;
	void *p;
	int i;

	atomics = of_iomap(of_node, 0);
	if (!atomics)
		return -ENODEV;

	arb_sema = of_iomap(of_node, 1);
	if (!arb_sema)
		return -ENODEV;

	/* handshake */
	if (!readl(arb_sema + STA_OFFSET))
		return -ENODEV;

	chmask = cfg->channel_mask;

	while (chmask) {
		i = __ffs(chmask);
		chmask &= ~(1 << i);

		area = bpmp_channel_area(atomics, i);
		if (!area)
			return -EFAULT;

		p = ioremap(area, 0x80);
		if (!p)
			return -ENOMEM;

		channel_area[i].ib = p;
		channel_area[i].ob = p;
	}

	return 0;
}

const struct mail_ops t210_mail_ops = {
	.init_irq = bpmp_init_irq,
	.connect = bpmp_connect,
	.master_free = bpmp_master_free,
	.free_master = bpmp_free_master,
	.master_acked = bpmp_master_acked,
	.signal_slave = bpmp_signal_slave,
	.ring_doorbell = bpmp_ring_doorbell,
	.slave_signalled = bpmp_slave_signalled,
	.return_data = bpmp_return_data
};
