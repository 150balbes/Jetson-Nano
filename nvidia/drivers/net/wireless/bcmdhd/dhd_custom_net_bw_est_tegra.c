/*
 * drivers/net/wireless/bcmdhd/dhd_custom_net_bw_est_tegra.c
 *
 * NVIDIA Tegra Network Bandwidth Estimator for BCMDHD driver
 *
 * Copyright (C) 2015 NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "dhd_custom_net_bw_est_tegra.h"
#ifdef CONFIG_BCMDHD_CUSTOM_NET_PERF_TEGRA
#include "dhd_custom_net_perf_tegra.h"
#endif
#include <linux_osl.h>

static int tegra_net_bw_est_debug;

#define USE_JIFFIES
#define USE_KTIME

/* network bandwidth estimate (bps) between src/dst mac addr(s) */

#ifndef TEGRA_NET_BW_EST_GUESS
#define TEGRA_NET_BW_EST_GUESS				450000000UL /* bps */
#endif

#ifndef TEGRA_NET_BW_EST_GUESS_MIN
#define TEGRA_NET_BW_EST_GUESS_MIN			1000000UL /* bps */
#endif

#ifndef TEGRA_NET_BW_EST_GUESS_MAX
#define TEGRA_NET_BW_EST_GUESS_MAX			1000000000UL /* bps */
#endif

#ifndef TEGRA_NET_BW_EST_SAMPLE_TIME
#define TEGRA_NET_BW_EST_SAMPLE_TIME			100 /* ms */
#endif

#ifndef TEGRA_NET_BW_EST_SAMPLE_TIME_MIN
#define TEGRA_NET_BW_EST_SAMPLE_TIME_MIN		10 /* ms */
#endif

#ifndef TEGRA_NET_BW_EST_SAMPLE_TIME_MAX
#define TEGRA_NET_BW_EST_SAMPLE_TIME_MAX		10000 /* ms */
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER		5
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MIN
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MIN	1
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MAX
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MAX	100
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM		10
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MIN
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MIN	1
#endif

#ifndef TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MAX
#define TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MAX	100
#endif

static unsigned long tegra_net_bw_est_guess
	= TEGRA_NET_BW_EST_GUESS;
static unsigned int tegra_net_bw_est_sample_time
	= TEGRA_NET_BW_EST_SAMPLE_TIME;
static unsigned int tegra_net_bw_est_prefill_tx_queue_numer
	= TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER;
static unsigned int tegra_net_bw_est_prefill_tx_queue_denom
	= TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM;

module_param(tegra_net_bw_est_guess, ulong, 0644);
module_param(tegra_net_bw_est_sample_time, uint, 0644);
module_param(tegra_net_bw_est_prefill_tx_queue_numer, uint, 0644);
module_param(tegra_net_bw_est_prefill_tx_queue_denom, uint, 0644);

static unsigned long tegra_net_bw_est_value;
static unsigned char tegra_net_bw_est_src_macaddr[ETH_ALEN];
static unsigned char tegra_net_bw_est_dst_macaddr[ETH_ALEN];

/* network bandwidth estimator packet generator */

#ifndef TEGRA_NET_BW_EST_PKTGEN_MINSIZ
#define TEGRA_NET_BW_EST_PKTGEN_MINSIZ			64
#endif

#ifndef TEGRA_NET_BW_EST_PKTGEN_MAXSIZ
#define TEGRA_NET_BW_EST_PKTGEN_MAXSIZ			1512
#endif

#ifndef TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_SRC_ADDR
#define TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_SRC_ADDR	0x01234567
#endif

#ifndef TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_DST_ADDR
#define TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_DST_ADDR	0x89abcdef
#endif

#ifndef TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_SRC_PORT
#define TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_SRC_PORT	32768
#endif

#ifndef TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_DST_PORT
#define TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_DST_PORT	5001
#endif

atomic_t ip_ident = ATOMIC_INIT(0);

static void tegra_net_bw_est_pktgen(struct net_device *net,
	unsigned int minsiz, unsigned int maxsiz)
{
	unsigned int size;
	unsigned int ethhdr_len;
	unsigned int udphdr_len;
	unsigned int iphdr_len;
	unsigned int data_len;
	struct sk_buff *skb;
	unsigned char *data;
	struct udphdr *udphdr;
	struct iphdr *iphdr;
	struct ethhdr *ethhdr;

	/* check input */
	if (!net)
		return;
	if (minsiz < TEGRA_NET_BW_EST_PKTGEN_MINSIZ)
		minsiz = TEGRA_NET_BW_EST_PKTGEN_MINSIZ;
	if (minsiz > TEGRA_NET_BW_EST_PKTGEN_MAXSIZ)
		minsiz = TEGRA_NET_BW_EST_PKTGEN_MAXSIZ;
	if (maxsiz < TEGRA_NET_BW_EST_PKTGEN_MINSIZ)
		maxsiz = TEGRA_NET_BW_EST_PKTGEN_MINSIZ;
	if (maxsiz > TEGRA_NET_BW_EST_PKTGEN_MAXSIZ)
		maxsiz = TEGRA_NET_BW_EST_PKTGEN_MAXSIZ;
	if (maxsiz < minsiz)
		maxsiz = minsiz;

	/* pick random packet size */
	get_random_bytes(&size, sizeof(size));
	size %= maxsiz - minsiz + 1;
	size += minsiz;

	/* calculate header / data length(s) */
	ethhdr_len = ETH_HLEN;
	iphdr_len = sizeof(*iphdr);
	udphdr_len = sizeof(*udphdr);
	data_len = size - ethhdr_len - iphdr_len - udphdr_len;

	/* allocate packet */
	skb = alloc_skb(size, GFP_KERNEL);
	if (!skb) {
		TEGRA_NET_BW_EST_DEBUG("%s: alloc skb (%d bytes) failed\n",
			__func__,  size);
		return;
	}
	skb_reserve(skb, ethhdr_len + iphdr_len + udphdr_len);
	skb->dev = net;
	skb->protocol = htons(ETH_P_IP);

	/* put data */
	data = skb_put(skb, data_len);
#if 0
	get_random_bytes(data, data_len);
#endif

	/* push udp header */
	skb_push(skb, udphdr_len);
	skb_reset_transport_header(skb);
	udphdr = udp_hdr(skb);
	udphdr->source = htons(TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_SRC_PORT);
	udphdr->dest = htons(TEGRA_NET_BW_EST_PKTGEN_FAKE_UDP_DST_PORT);
	udphdr->len = htons(udphdr_len + data_len);
	udphdr->check = 0;

	/* push ip header */
	skb_push(skb, iphdr_len);
	skb_reset_network_header(skb);
	iphdr = ip_hdr(skb);
	iphdr->version = 4;
	iphdr->ihl = iphdr_len / 4;
	iphdr->tos = 0;
	iphdr->tot_len = htons(iphdr_len + udphdr_len + data_len);
	iphdr->id = htons(atomic_inc_return(&ip_ident));
	iphdr->frag_off = htons(0);
	iphdr->ttl = 64;
	iphdr->protocol = IPPROTO_UDP;
	iphdr->check = 0;	/* 0 = to force bad IP header checksum */
	iphdr->saddr = TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_SRC_ADDR;
	iphdr->daddr = TEGRA_NET_BW_EST_PKTGEN_FAKE_IP_DST_ADDR;

	/* push ethernet header */
	skb_push(skb, ethhdr_len);
	skb_reset_mac_header(skb);
	ethhdr = eth_hdr(skb);
	ethhdr->h_proto = htons(ETH_P_IP);
	memcpy(ethhdr->h_source, net->dev_addr, ETH_ALEN);
	memcpy(ethhdr->h_dest, tegra_net_bw_est_dst_macaddr, ETH_ALEN);

	/* queue packet */
	dev_queue_xmit(skb);

}

/* network bandwidth estimator work */

static unsigned int tegra_net_bw_est_work_rate;

static struct workqueue_struct *tegra_net_bw_est_wq;

static void tegra_net_bw_est_work_func(struct work_struct *work);

static DECLARE_DELAYED_WORK(tegra_net_bw_est_work, tegra_net_bw_est_work_func);

static void tegra_net_bw_est_work_func(struct work_struct *work)
{
#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
	extern struct net_device *dhd_custom_sysfs_tegra_histogram_stat_netdev;
	struct net_device *net
		= dhd_custom_sysfs_tegra_histogram_stat_netdev;
#else
	struct net_device *net
		= NULL;
#endif
	unsigned long guess;
	unsigned int sample_time;
	unsigned int numer;
	unsigned int denom;
	wl_cnt_t *cnt0, *cnt1;
#if defined(USE_JIFFIES)
	unsigned long jiffies0, jiffies1;
#endif
#if defined(USE_KTIME)
	ktime_t ktime0, ktime1;
#endif
	unsigned long bps;
	unsigned int minsiz, maxsiz, pktsiz, pktcnt, i, m, n;

	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	tegra_net_bw_est_value = 0;

	/* check network device / parameters */
	if (!net) {
		TEGRA_NET_BW_EST_DEBUG("%s: !net\n", __func__);
		goto done;
	}
	guess = tegra_net_bw_est_guess;
	sample_time = tegra_net_bw_est_sample_time;
	numer = tegra_net_bw_est_prefill_tx_queue_numer;
	denom = tegra_net_bw_est_prefill_tx_queue_denom;
	if (guess < TEGRA_NET_BW_EST_GUESS_MIN)
		guess = TEGRA_NET_BW_EST_GUESS_MIN;
	if (guess > TEGRA_NET_BW_EST_GUESS_MAX)
		guess = TEGRA_NET_BW_EST_GUESS_MAX;
	if (sample_time < TEGRA_NET_BW_EST_SAMPLE_TIME_MIN)
		sample_time = TEGRA_NET_BW_EST_SAMPLE_TIME_MIN;
	if (sample_time > TEGRA_NET_BW_EST_SAMPLE_TIME_MAX)
		sample_time = TEGRA_NET_BW_EST_SAMPLE_TIME_MAX;
	if (numer < TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MIN)
		numer = TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MIN;
	if (numer > TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MAX)
		numer = TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_NUMER_MAX;
	if (denom < TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MIN)
		denom = TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MIN;
	if (denom > TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MAX)
		denom = TEGRA_NET_BW_EST_PREFILL_TX_QUEUE_DENOM_MAX;

	/* boost net perf while running b/w estimator */
#ifdef CONFIG_BCMDHD_CUSTOM_NET_PERF_TEGRA
	wifi_sclk_enable();
#endif
	/* wait for boost to take into effect */
	OSL_SLEEP(2);

	/* create starting / ending counters request */
	cnt0 = kmalloc(2 * sizeof(wl_cnt_t), GFP_KERNEL);
	if (!cnt0) {
		TEGRA_NET_BW_EST_DEBUG("%s: kmalloc(2 * wl_cnt_t) failed\n",
			__func__);
		goto done2;
	}
	cnt1 = cnt0 + 1;

	/* get starting tx byte count / time */
	if (wldev_iovar_getbuf(net, "counters", NULL, 0,
		(void *) cnt0, sizeof(wl_cnt_t), NULL) != BCME_OK) {
		TEGRA_NET_BW_EST_DEBUG("%s: wldev_iovar_getbuf(cnt0) failed\n",
			__func__);
		kfree(cnt0);
		goto done2;
	}
#if defined(USE_JIFFIES)
	jiffies0 = jiffies;
#endif
#if defined(USE_KTIME)
	ktime0 = ktime_get();
#endif

	/* send enough packets to saturate network bandwidth for sample time */
	bps = guess;
	minsiz = 1500;
	maxsiz = 1500;
	pktsiz = (minsiz + maxsiz) / 2;
	pktcnt = bps / (pktsiz * 8) * sample_time / 1000;
	TEGRA_NET_BW_EST_DEBUG("%s: bps %lu pktsiz %u %u %u pktcnt %u\n",
		__func__, bps, minsiz, maxsiz, pktsiz, pktcnt);

	/* prefill tx queue */
	n = pktcnt * numer / denom;
	for (i = 0; i < n; i++)
		tegra_net_bw_est_pktgen(net, minsiz, maxsiz);

	/* wait for sample time */
	m = 0;
	n = pktcnt / denom;
	while (time_before(jiffies, jiffies0 +
		msecs_to_jiffies(sample_time))) {
		/* queue more tx packets
		 * - pktcnt = calculated number of packets required to saturate
		 *   network bandwidth for entire sample time
		 * - already queued numer/denom of pktcnt before this loop
		 * - for first (denom - numer) loop iterations, queue 1/denom
		 *   of pktcnt
		 * - for remaining numer loop iterations, no need to queue any
		 *   more packets (but can calculate bw est accurately)
		 */
		if (m++ < denom - numer)
			for (i = 0; i < n; i++)
				tegra_net_bw_est_pktgen(net, minsiz, maxsiz);
		/* get intermediate tx byte count / time */
#if 0
		if (wldev_iovar_getbuf(net, "counters", NULL, 0,
			(void *) cnt1, sizeof(wl_cnt_t), NULL) != BCME_OK) {
			TEGRA_NET_BW_EST_DEBUG("%s: wldev_iovar_getbuf(cnt1)"
				" failed\n",
				__func__);
			kfree(cnt0);
			goto done2;
		}
		jiffies1 = jiffies;
		/* calculate intermediate bandwidth */
		tegra_net_bw_est_value
			= ((cnt1->txbyte - cnt0->txbyte) * 8) /
				(jiffies1 - jiffies0 + 1) * HZ;
		TEGRA_NET_BW_EST_DEBUG("%s: bw est = %lu bps\n",
			__func__, tegra_net_bw_est_value);
#endif
		/* wait for next sample window */
		OSL_SLEEP(sample_time / denom);
	}

	/* get ending tx byte count / time */
	if (wldev_iovar_getbuf(net, "counters", NULL, 0,
		(void *) cnt1, sizeof(wl_cnt_t), NULL) != BCME_OK) {
		TEGRA_NET_BW_EST_DEBUG("%s: wldev_iovar_getbuf(cnt1) failed\n",
			__func__);
		kfree(cnt0);
		goto done2;
	}
#if defined(USE_JIFFIES)
	jiffies1 = jiffies;
#endif
#if defined(USE_KTIME)
	ktime1 = ktime_get();
#endif

	/* calculate bandwidth */
#if defined(USE_JIFFIES)
	tegra_net_bw_est_value
		= ((cnt1->txbyte - cnt0->txbyte) * 8) /
			(jiffies1 - jiffies0 + 1) * HZ;
	TEGRA_NET_BW_EST_DEBUG("%s: jiff: bw est = %lu bps (final)\n",
		__func__, tegra_net_bw_est_value);
#endif
#if defined(USE_KTIME)
	tegra_net_bw_est_value
		= ((cnt1->txbyte - cnt0->txbyte) * 8) /
			((unsigned long)
			ktime_to_ms(ktime_sub(ktime1, ktime0))) * 1000UL;
	TEGRA_NET_BW_EST_DEBUG("%s: ktime: bw est = %lu bps (final)\n",
		__func__, tegra_net_bw_est_value);
#endif

	/* free starting / ending counters request */
	kfree(cnt0);

	/* unboost net perf after running b/w estimator */
done2:
#ifdef CONFIG_BCMDHD_CUSTOM_NET_PERF_TEGRA
	wifi_sclk_disable();
#endif

	/* schedule next bandwidth estimator work */
done:	if (tegra_net_bw_est_work_rate > 0)
		queue_delayed_work(tegra_net_bw_est_wq,
			&tegra_net_bw_est_work,
			msecs_to_jiffies(tegra_net_bw_est_work_rate));
}

static void tegra_net_bw_est_work_start(void)
{
	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* check work queue */
	if (!tegra_net_bw_est_wq) {
		TEGRA_NET_BW_EST_DEBUG("%s: !workqueue\n", __func__);
		return;
	}

	/* schedule bandwidth estimator work */
	queue_delayed_work(tegra_net_bw_est_wq,
		&tegra_net_bw_est_work,
		msecs_to_jiffies(0));
}

static void tegra_net_bw_est_work_stop(void)
{
	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* check work queue */
	if (!tegra_net_bw_est_wq) {
		TEGRA_NET_BW_EST_DEBUG("%s: !workqueue\n", __func__);
		return;
	}

	/* cancel bandwidth estimator work */
	tegra_net_bw_est_work_rate = 0;
	cancel_delayed_work_sync(&tegra_net_bw_est_work);
}

/* network bandwidth estimator sysfs */

static ssize_t
tegra_net_bw_est_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* show network bandwidth estimate (bps) */
	snprintf(buf, PAGE_SIZE, "%ld\n", tegra_net_bw_est_value);

	return strlen(buf);
}

static ssize_t
tegra_net_bw_est_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int err;
	unsigned int uint;
	unsigned long ul;

	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	if (strncmp(buf, "debug", 5) == 0) {
		tegra_net_bw_est_debug = !tegra_net_bw_est_debug;
	} else if (strncmp(buf, "enable", 6) == 0) {
		TEGRA_NET_BW_EST_DEBUG("%s: starting bw est delayed work...\n",
			__func__);
		tegra_net_bw_est_work_start();
	} else if (strncmp(buf, "disable", 7) == 0) {
		TEGRA_NET_BW_EST_DEBUG("%s: stopping bw est delayed work...\n",
			__func__);
		tegra_net_bw_est_work_stop();
	} else if (strncmp(buf, "rate ", 5) == 0) {
		err = kstrtouint(buf + 5, 0, &uint);
		if (err < 0) {
			TEGRA_NET_BW_EST_DEBUG("%s: invalid bw est rate"
				" (ms)\n",
				__func__);
			return count;
		}
		TEGRA_NET_BW_EST_DEBUG("%s: set bw est rate (ms) %u\n",
			__func__, uint);
		tegra_net_bw_est_work_rate = uint;
	} else if (strncmp(buf, "guess ", 6) == 0) {
		err = kstrtoul(buf + 6, 0, &ul);
		if (err < 0) {
			TEGRA_NET_BW_EST_DEBUG("%s: invalid bw est guess"
				" (bps)\n",
				__func__);
			return count;
		}
		TEGRA_NET_BW_EST_DEBUG("%s: set bw est guess (bps) %lu\n",
			__func__, ul);
		tegra_net_bw_est_guess = ul;
	} else if (strncmp(buf, "sample_time ", 12) == 0) {
		err = kstrtouint(buf + 12, 0, &uint);
		if (err < 0) {
			TEGRA_NET_BW_EST_DEBUG("%s: invalid bw est sample time"
				" (ms)\n",
				__func__);
			return count;
		}
		TEGRA_NET_BW_EST_DEBUG("%s: set bw est sample time (ms) %u\n",
			__func__, uint);
		tegra_net_bw_est_sample_time = uint;
	} else if (strncmp(buf, "prefill_numer ", 14) == 0) {
		err = kstrtouint(buf + 14, 0, &uint);
		if (err < 0) {
			TEGRA_NET_BW_EST_DEBUG("%s: invalid bw est prefill"
				" numer\n",
				__func__);
			return count;
		}
		TEGRA_NET_BW_EST_DEBUG("%s: set bw est prefill numer %u\n",
			__func__, uint);
		tegra_net_bw_est_prefill_tx_queue_numer = uint;
	} else if (strncmp(buf, "prefill_denom ", 14) == 0) {
		err = kstrtouint(buf + 14, 0, &uint);
		if (err < 0) {
			TEGRA_NET_BW_EST_DEBUG("%s: invalid bw est prefill"
				" denom\n",
				__func__);
			return count;
		}
		TEGRA_NET_BW_EST_DEBUG("%s: set bw est prefill denom %u\n",
			__func__, uint);
		tegra_net_bw_est_prefill_tx_queue_denom = uint;
	} else {
		TEGRA_NET_BW_EST_DEBUG("%s: unknown command\n", __func__);
	}
	return count;
}

static DEVICE_ATTR(net_bw_est, S_IRUGO | S_IWUSR,
	tegra_net_bw_est_show,
	tegra_net_bw_est_store);

/* network bandwidth estimator debugfs */

ssize_t
tegra_debugfs_net_bw_est_read(struct file *filp,
	char __user *buff, size_t count, loff_t *offp)
{
	char buf[512];
	ssize_t len;

	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* check for read eof */
	if (offp && (*offp != 0))
		return 0;

	/* show network bandwidth estimate (bps) */
	snprintf(buf, sizeof(buf), "%ld\n", tegra_net_bw_est_value);
	len = strlen(buf);
	if (copy_to_user(buff, buf, len) != 0) {
		TEGRA_NET_BW_EST_DEBUG("%s: copy_to_user() failed!\n",
			__func__);
		return 0;
	}
	if (offp)
		*offp += len;

	return len;
}

ssize_t
tegra_debugfs_net_bw_est_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *offp)
{
	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);
	return count;
}

static struct file_operations tegra_debugfs_net_bw_est_fops = {
	.read = tegra_debugfs_net_bw_est_read,
	.write = tegra_debugfs_net_bw_est_write,
};

static struct dentry *tegra_debugfs_net_bw_est;

/* network bandwidth estimator initialization */

int tegra_net_bw_est_register(struct device *dev)
{
	int err;

	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* create sysfs */
	err = sysfs_create_file(&dev->kobj, &dev_attr_net_bw_est.attr);
	if (err) {
		TEGRA_NET_BW_EST_DEBUG("%s: failed to create sysfs file"
			" - %d\n",
			__func__, err);
		return err;
	}

	/* create debugfs */
	tegra_debugfs_net_bw_est = debugfs_create_file("bcmdhd_net_bw_est",
		S_IRUGO | S_IWUGO, NULL, (void *) 0,
		&tegra_debugfs_net_bw_est_fops);
	if (!tegra_debugfs_net_bw_est) {
		TEGRA_NET_BW_EST_DEBUG("%s: failed to create debugfs file\n",
			__func__);
		/* remove sysfs */
		sysfs_remove_file(&dev->kobj, &dev_attr_net_bw_est.attr);
		return -EPERM;
	}

	/* create work queue */
	tegra_net_bw_est_wq = create_workqueue("tegra_net_bw_est_wq");
	if (!tegra_net_bw_est_wq) {
		TEGRA_NET_BW_EST_DEBUG("%s: failed to create workqueue\n",
			__func__);
		/* remove debugfs */
		debugfs_remove(tegra_debugfs_net_bw_est);
		tegra_debugfs_net_bw_est = NULL;
		/* remove sysfs */
		sysfs_remove_file(&dev->kobj, &dev_attr_net_bw_est.attr);
		return -EPERM;
	}

	/* start work */
	tegra_net_bw_est_work_start();

	return 0;
}

void tegra_net_bw_est_unregister(struct device *dev)
{
	TEGRA_NET_BW_EST_DEBUG("%s\n", __func__);

	/* stop work */
	tegra_net_bw_est_work_stop();

	/* destroy workqueue */
	if (tegra_net_bw_est_wq) {
		destroy_workqueue(tegra_net_bw_est_wq);
		tegra_net_bw_est_wq = NULL;
	}

	/* remove debugfs */
	if (tegra_debugfs_net_bw_est) {
		debugfs_remove(tegra_debugfs_net_bw_est);
		tegra_debugfs_net_bw_est = NULL;
	}

	/* remove sysfs */
	sysfs_remove_file(&dev->kobj, &dev_attr_net_bw_est.attr);
}

void tegra_net_bw_est_set_src_macaddr(unsigned char *macaddr)
{
	TEGRA_NET_BW_EST_DEBUG("%s: %02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__,
		macaddr[0], macaddr[1], macaddr[2],
		macaddr[3], macaddr[4], macaddr[5]);
	memcpy(tegra_net_bw_est_src_macaddr, macaddr, ETH_ALEN);
}

void tegra_net_bw_est_set_dst_macaddr(unsigned char *macaddr)
{
	TEGRA_NET_BW_EST_DEBUG("%s: %02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__,
		macaddr[0], macaddr[1], macaddr[2],
		macaddr[3], macaddr[4], macaddr[5]);
	memcpy(tegra_net_bw_est_dst_macaddr, macaddr, ETH_ALEN);
}

/* private interface for network diagnostics */

unsigned long tegra_net_bw_est_get_value(void)
{

#ifdef CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA
	if (!tegra_sysfs_wifi_on) {
		return 0;
	}
#endif /* CONFIG_BCMDHD_CUSTOM_SYSFS_TEGRA */

	/* start work */
	tegra_net_bw_est_work_start();

	/* wait for work to complete */
	flush_delayed_work(&tegra_net_bw_est_work);

	/* return bw est */
	return tegra_net_bw_est_value;
}
