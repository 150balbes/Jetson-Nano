/*
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/tegra_bpmp.h>
#include "bpmp.h"

#define CHANNEL_TIMEOUT		(timeout_mul * USEC_PER_SEC)
#define THREAD_CH_TIMEOUT	(timeout_mul * USEC_PER_SEC)

static unsigned int timeout_mul = 4;
struct channel_data channel_area[NR_MAX_CHANNELS];
static struct completion *completion;
static DEFINE_SPINLOCK(lock);
static DEFINE_RAW_SPINLOCK(ach_lock);
static const struct channel_cfg *channel_cfg;
static const struct mail_ops *mail_ops;

uint32_t tegra_bpmp_mail_readl(int ch, int offset)
{
	u32 *data = (u32 *)(channel_area[ch].ib->data + offset);
	return *data;
}
EXPORT_SYMBOL(tegra_bpmp_mail_readl);

int tegra_bpmp_read_data(unsigned int ch, void *data, size_t sz)
{
	void __iomem *addr;
	unsigned int m;

	if (!data || sz > MSG_DATA_MIN_SZ)
		return -EINVAL;

	m = (1u << ch) & channel_cfg->channel_mask;
	if (!m)
		return -EINVAL;

	addr = (void __iomem*)channel_area[ch].ib->data;

	memcpy_fromio(data, addr, sz);

	return 0;
}
EXPORT_SYMBOL(tegra_bpmp_read_data);

void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz)
{
	if (mail_ops && mail_ops->return_data)
		mail_ops->return_data(mail_ops, ch, code, data, sz);
}
EXPORT_SYMBOL(tegra_bpmp_mail_return_data);

void tegra_bpmp_mail_return(int ch, int code, int v)
{
	tegra_bpmp_mail_return_data(ch, code, &v, sizeof(v));
}
EXPORT_SYMBOL(tegra_bpmp_mail_return);

static unsigned int to_reclaim;
static unsigned int to_complete;
static unsigned int tch_free;
static struct semaphore tch_sem;

static struct completion *bpmp_completion_obj(int ch)
{
	unsigned int i;

	i = ch - channel_cfg->thread_ch_0;

	if (i >= channel_cfg->thread_ch_cnt)
		return NULL;

	return completion + i;
}

static void bpmp_signal_thread(int ch)
{
	struct mb_data *p = channel_area[ch].ob;
	struct completion *w;
	unsigned int m = 1u << ch;

	if (!(p->flags & RING_DOORBELL))
		return;

	if ((to_reclaim & m) != 0) {
		mail_ops->free_master(mail_ops, ch);
		to_reclaim &= ~m;
		tch_free |= m;
		WARN_ON(tch_sem.count >= channel_cfg->thread_ch_cnt);
		up(&tch_sem);
		return;
	}

	w = bpmp_completion_obj(ch);
	if (!w) {
		WARN_ON(1);
		return;
	}

	complete(w);
}

void bpmp_handle_irq(unsigned int chidx)
{
	unsigned int m;
	int ch;
	int i;

	if (!mail_ops)
		return;

	if (WARN_ON_ONCE(chidx >= channel_cfg->ib_ch_cnt))
		return;

	ch = channel_cfg->ib_ch_0 + chidx;

	if (mail_ops->slave_signalled(mail_ops, ch))
		bpmp_handle_mail(channel_area[ch].ib->code, ch);

	spin_lock(&lock);

	for (i = 0; i < channel_cfg->thread_ch_cnt && to_complete; i++) {
		ch = channel_cfg->thread_ch_0 + i;
		m = 1u << ch;
		if (mail_ops->master_acked(mail_ops, ch) && (to_complete & m)) {
			to_complete &= ~m;
			bpmp_signal_thread(ch);
		}
	}

	spin_unlock(&lock);
}

static int bpmp_wait_master_free(int ch)
{
	ktime_t start;

	if (mail_ops->master_free(mail_ops, ch))
		return 0;

	start = ktime_get();

	while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT) {
		if (mail_ops->master_free(mail_ops, ch))
			return 0;
	}

	return -ETIMEDOUT;
}

static void __bpmp_write_ch(int ch, int mrq, int flags, void *data, int sz)
{
	struct mb_data *p = channel_area[ch].ob;
	void __iomem *addr;

	p->code = mrq;
	p->flags = flags;

	if (data) {
		addr = (void __iomem *)p->data;
		memcpy_toio(addr, data, sz);
	}

	mail_ops->signal_slave(mail_ops, ch);
}

static int bpmp_write_ch(int ch, int mrq, int flags, void *data, int sz)
{
	int r;

	r = bpmp_wait_master_free(ch);
	if (r)
		return r;

	__bpmp_write_ch(ch, mrq, flags, data, sz);
	return 0;
}

static int bpmp_write_threaded_ch(int *ch, int mrq, void *data, int sz)
{
	unsigned long flags;
	unsigned int m;
	int ret;

	spin_lock_irqsave(&lock, flags);

	*ch = __ffs(tch_free);

	ret = mail_ops->master_free(mail_ops, *ch) ? 0 : -EFAULT;

	if (ret) {
		pr_err("bpmp: channel %d is not free to write\n", *ch);
	} else {
		m = 1u << *ch;
		tch_free &= ~m;
		__bpmp_write_ch(*ch, mrq, DO_ACK | RING_DOORBELL, data, sz);
		to_complete |= m;
	}

	spin_unlock_irqrestore(&lock, flags);

	return ret;
}

static int __bpmp_read_ch(int ch, void *data, int sz)
{
	struct mb_data *p = channel_area[ch].ib;
	void __iomem *addr;

	if (data) {
		addr = (void __iomem*) p->data;
		memcpy_fromio(data, addr, sz);
	}

	mail_ops->free_master(mail_ops, ch);

	return p->code;
}

static int bpmp_read_ch(int ch, void *data, int sz)
{
	unsigned long flags;
	int r;

	spin_lock_irqsave(&lock, flags);
	r = __bpmp_read_ch(ch, data, sz);
	tch_free |= (1u << ch);
	spin_unlock_irqrestore(&lock, flags);

	return r;
}

static int bpmp_wait_ack(int ch)
{
	ktime_t start;

	if (mail_ops->master_acked(mail_ops, ch))
		return 0;

	start = ktime_get();

	while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT) {
		if (mail_ops->master_acked(mail_ops, ch))
			return 0;
	}

	return -ETIMEDOUT;
}

static int bpmp_valid_txfer(void *ob_data, int ob_sz, void *ib_data, int ib_sz)
{
	return ob_sz >= 0 &&
			ob_sz <= MSG_DATA_MIN_SZ &&
			ib_sz >= 0 &&
			ib_sz <= MSG_DATA_MIN_SZ &&
			(!ob_sz || ob_data) &&
			(!ib_sz || ib_data);
}

static void bpmp_format_req(char *fmt, size_t len, uint8_t *data, size_t sz)
{
	size_t ne;
	size_t i;
	int n;

	for (i = 0; i < sz; i++) {
		ne = sz - i - 1;
		n = snprintf(fmt, len, ne ? "0x%02x " : "0x%02x", data[i]);
		if (n >= len)
			break;
		fmt += n;
		len -= n;
	}
}

static void bpmp_dump_req(int ch)
{
	char fmt[MSG_DATA_MIN_SZ * 5] = "";
	struct mb_data d;
	struct mb_data *p = channel_area[ch].ob;

	memcpy_fromio(&d, p, sizeof(d));
	bpmp_format_req(fmt, sizeof(fmt), d.data, sizeof(d.data));
	pr_err("ch %d mrq %d data <%s>\n", ch, d.code, fmt);
}

static int bpmp_send_receive_atomic(int ch, int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	char fmt[MSG_DATA_MIN_SZ * 5] = "";
	int r;

	r = bpmp_write_ch(ch, mrq, DO_ACK, ob_data, ob_sz);
	if (r)
		return r;

	if (mail_ops->ring_doorbell)
		mail_ops->ring_doorbell(ch);

	r = bpmp_wait_ack(ch);
	if (r) {
		bpmp_format_req(fmt, sizeof(fmt), ob_data, ob_sz);
		pr_err("bpmp_wait_ack() returned %d (ch %d mrq %d data <%s>)\n",
				r, mrq, ch, fmt);
		WARN_ON(1);
		return r;
	}

	return __bpmp_read_ch(ch, ib_data, ib_sz);
}

/* should be called with local irqs disabled */
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	unsigned int cpu;
	int ch;
	int r;

	if (WARN_ON(!irqs_disabled()))
		return -EPERM;

	if (!bpmp_valid_txfer(ob_data, ob_sz, ib_data, ib_sz))
		return -EINVAL;

	if (!mail_ops)
		return -ENODEV;

	if (channel_cfg->per_cpu_ch_cnt == 1) {
		raw_spin_lock(&ach_lock);
		ch = channel_cfg->per_cpu_ch_0;
	} else {
		cpu = smp_processor_id();
		if (cpu >= channel_cfg->per_cpu_ch_cnt)
			return -ENODEV;
		ch = channel_cfg->per_cpu_ch_0 + cpu;
	}

	r = bpmp_send_receive_atomic(ch, mrq, ob_data, ob_sz, ib_data, ib_sz);

	if (channel_cfg->per_cpu_ch_cnt == 1)
		raw_spin_unlock(&ach_lock);

	return r;
}
EXPORT_SYMBOL(tegra_bpmp_send_receive_atomic);

static int bpmp_trywait(int ch, int mrq, void *ob_data, int ob_sz)
{
	char fmt[MSG_DATA_MIN_SZ * 5] = "";
	struct completion *w;
	unsigned long timeout;
	unsigned long rt;
	unsigned long et;
	unsigned long flags;

	w = bpmp_completion_obj(ch);
	timeout = usecs_to_jiffies(THREAD_CH_TIMEOUT);
	rt = wait_for_completion_timeout(w, timeout);
	if (rt) {
		et = timeout - rt;
		if (et > timeout / 4)
			pr_warn("bpmp: mrq %d took %u us\n",
					mrq, jiffies_to_usecs(et));
		return 0;
	}

	spin_lock_irqsave(&lock, flags);

	if (mail_ops->master_acked(mail_ops, ch)) {
		spin_unlock_irqrestore(&lock, flags);
		return 0;
	}

	to_reclaim |= 1u << ch;

	spin_unlock_irqrestore(&lock, flags);

	bpmp_format_req(fmt, sizeof(fmt), ob_data, ob_sz);
	pr_err("%s() timed out (ch %d mrq %d data <%s>)\n",
			__func__, ch, mrq, fmt);
	WARN_ON(1);

	return -ETIMEDOUT;
}

int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz)
{
	int ch;
	int r;

	if (WARN_ON(irqs_disabled()))
		return -EPERM;

	if (!bpmp_valid_txfer(ob_data, ob_sz, ib_data, ib_sz))
		return -EINVAL;

	if (!mail_ops)
		return -ENODEV;

	r = down_timeout(&tch_sem, usecs_to_jiffies(THREAD_CH_TIMEOUT));
	if (r) {
		pr_err("%s() down_timeout return %d\n", __func__, r);
		pr_err("tch_free 0x%x to_complete 0x%x\n",
				tch_free, to_complete);
		return r;
	}

	r = bpmp_write_threaded_ch(&ch, mrq, ob_data, ob_sz);
	if (r)
		goto out;

	if (mail_ops->ring_doorbell)
		mail_ops->ring_doorbell(ch);

	r = bpmp_trywait(ch, mrq, ob_data, ob_sz);
	if (r)
		return r;

	r = bpmp_read_ch(ch, ib_data, ib_sz);

out:
	up(&tch_sem);

	return r;
}
EXPORT_SYMBOL(tegra_bpmp_send_receive);

int tegra_bpmp_running(void)
{
	return mail_ops ? 1 : 0;
}
EXPORT_SYMBOL(tegra_bpmp_running);

void tegra_bpmp_resume(void)
{
	if (mail_ops->resume)
		mail_ops->resume();
}

int tegra_bpmp_suspend(void)
{
	if (to_complete) {
		unsigned int i;
		pr_err("%s() channels waiting (to_complete 0x%x)\n",
				__func__, to_complete);
		for (i = 0; i < NR_MAX_CHANNELS; ++i) {
			if (to_complete & (1 << i)) {
				bpmp_dump_req(i);
			}
		}
		return -EBUSY;
	}

	if (mail_ops->suspend)
		mail_ops->suspend();

	return 0;
}

static int bpmp_init_completion(int cnt)
{
	int i;

	completion = kcalloc(cnt, sizeof(*completion), GFP_KERNEL);
	if (!completion)
		return -ENOMEM;

	for (i = 0; i < cnt; i++)
		init_completion(completion + i);

	return 0;
}

int bpmp_mail_init(const struct channel_cfg *cfg, const struct mail_ops *ops,
		struct device_node *of_node)
{
	int r;

	r = ops->init_prepare ? ops->init_prepare() : 0;
	if (r) {
		pr_err("bpmp: mail init prepare failed (%d)\n", r);
		return r;
	}

	tch_free = (1 << cfg->thread_ch_cnt) - 1;
	tch_free <<= cfg->thread_ch_0;

	sema_init(&tch_sem, cfg->thread_ch_cnt);

	r = bpmp_init_completion(cfg->thread_ch_cnt);
	if (r)
		return r;

	r = ops->init_irq ? ops->init_irq(cfg->ib_ch_cnt) : 0;
	if (r) {
		pr_err("bpmp: irq init failed (%d)\n", r);
		return r;
	}

	r = bpmp_mailman_init();
	if (r) {
		pr_err("bpmp: mailman init failed (%d)\n", r);
		return r;
	}

	r = ops->connect(cfg, ops, of_node);
	if (r) {
		pr_err("bpmp: connect failed (%d)\n", r);
		return r;
	}

	if (!tegra_platform_is_silicon())
		timeout_mul = 600;

	channel_cfg = cfg;

	mail_ops = ops;

	pr_info("bpmp: mail init ok\n");

	return 0;
}
