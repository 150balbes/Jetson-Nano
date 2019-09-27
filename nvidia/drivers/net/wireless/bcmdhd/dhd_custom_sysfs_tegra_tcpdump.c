/*
 * drivers/net/wireless/bcmdhd/dhd_custom_sysfs_tegra_tcpdump.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2014-2016 NVIDIA Corporation. All rights reserved.
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

#include "dhd_custom_sysfs_tegra.h"
#include <linux/jiffies.h>
#include <linux/spinlock_types.h>

#ifndef TCPDUMP_NETIF_MAXSIZ
#define TCPDUMP_NETIF_MAXSIZ	16
#endif

#ifndef TCPDUMP_DATA_MAXSIZ
#define TCPDUMP_DATA_MAXSIZ	64
#endif

#ifndef PWRDUMP_DATA_MAXSIZ
#define PWRDUMP_DATA_MAXSIZ	128
#endif

#ifndef STATDUMP_DATA_MAXSIZ
#define STATDUMP_DATA_MAXSIZ	2560
#endif

#if defined(CONFIG_MACH_ARDBEG)
#define TCPDUMP_MAXSIZ		(1 * 1024 * 1024)
#elif defined(CONFIG_MACH_T132REF)
#define TCPDUMP_MAXSIZ		(3 * 1024 * 1024)
#endif

#ifndef TCPDUMP_MAXSIZ
#define TCPDUMP_MAXSIZ		(3 * 1024 * 1024)
#endif

/* delay between rx packet and running statistics work function
 * - ensures that statistics updated more frequently if rx is active
 */

#ifndef TCPDUMP_RX_STAT_DELAY
#define TCPDUMP_RX_STAT_DELAY	5 /* ms */
#endif

/* packet logs */

typedef struct {
	unsigned long serial_no;
	unsigned long time;
	char tag;
	char netif[TCPDUMP_NETIF_MAXSIZ];
	const char *func;
	int line;
	unsigned int data_nonpaged_len;
	unsigned int data_paged_len;
} multidump_pkt_t;

typedef struct {
	int head;
	int tail;
	unsigned long serial_no;
	multidump_pkt_t *pkt;
	int maxpkt;
} multidump_pkt_log_t;

static multidump_pkt_t tcpdump_pkt
	[(TCPDUMP_MAXSIZ * 8 / 10) /
		(sizeof(multidump_pkt_t) + TCPDUMP_DATA_MAXSIZ)];

static multidump_pkt_t pwrdump_pkt
	[(TCPDUMP_MAXSIZ * 1 / 10) /
		(sizeof(multidump_pkt_t) + PWRDUMP_DATA_MAXSIZ)];

static multidump_pkt_t statdump_pkt
	[(TCPDUMP_MAXSIZ * 1 / 10) /
		(sizeof(multidump_pkt_t) + STATDUMP_DATA_MAXSIZ)];

static unsigned char tcpdump_pkt_data
	[sizeof(tcpdump_pkt) / sizeof(tcpdump_pkt[0])]
	[TCPDUMP_DATA_MAXSIZ];

static unsigned char pwrdump_pkt_data
	[sizeof(pwrdump_pkt) / sizeof(pwrdump_pkt[0])]
	[PWRDUMP_DATA_MAXSIZ];

static unsigned char statdump_pkt_data
	[sizeof(statdump_pkt) / sizeof(statdump_pkt[0])]
	[STATDUMP_DATA_MAXSIZ];

static multidump_pkt_log_t multidump_pkt_log[3] = {
	/* first log is for tcpdump */
	{
		.pkt = tcpdump_pkt,
		.maxpkt = sizeof(tcpdump_pkt) / sizeof(tcpdump_pkt[0]),
	},
	/* second log is for pwrdump */
	{
		.pkt = pwrdump_pkt,
		.maxpkt = sizeof(pwrdump_pkt) / sizeof(pwrdump_pkt[0]),
	},
	/* third log is for statdump */
	{
		.pkt = statdump_pkt,
		.maxpkt = sizeof(statdump_pkt) / sizeof(statdump_pkt[0]),
	},
};

/* macros for accessing tcpdump log */

#define tcpdump_head		(multidump_pkt_log[0].head)
#define tcpdump_tail		(multidump_pkt_log[0].tail)
#define tcpdump_serial_no	(multidump_pkt_log[0].serial_no)
#define tcpdump_pkt(i)		(multidump_pkt_log[0].pkt + i)
#define tcpdump_maxpkt		(multidump_pkt_log[0].maxpkt)

/* macros for accessing pwrdump log */

#define pwrdump_head		(multidump_pkt_log[1].head)
#define pwrdump_tail		(multidump_pkt_log[1].tail)
#define pwrdump_serial_no	(multidump_pkt_log[1].serial_no)
#define pwrdump_pkt(i)		(multidump_pkt_log[1].pkt + i)
#define pwrdump_maxpkt		(multidump_pkt_log[1].maxpkt)

/* macros for accessing statdump log */

#define statdump_head		(multidump_pkt_log[2].head)
#define statdump_tail		(multidump_pkt_log[2].tail)
#define statdump_serial_no	(multidump_pkt_log[2].serial_no)
#define statdump_pkt(i)		(multidump_pkt_log[2].pkt + i)
#define statdump_maxpkt		(multidump_pkt_log[2].maxpkt)

/* macros for accessing *dump log */

#define multidump_head		(multidump_pkt_log[multi].head)
#define multidump_tail		(multidump_pkt_log[multi].tail)
#define multidump_serial_no	(multidump_pkt_log[multi].serial_no)
#define multidump_pkt(i)	(multidump_pkt_log[multi].pkt + i)
#define multidump_maxpkt	(multidump_pkt_log[multi].maxpkt)
/* If multi is not set default multi is 0 do replacing null with
 &(tcpdump_pkt_data[i][0]) but not updating max length. it will
 be 0 which means no memcpy will happen */
#define multidump_pkt_data(i)	(\
				(multi == 0)\
				? &(tcpdump_pkt_data[i][0]) \
				: (multi == 1)\
				? &(pwrdump_pkt_data[i][0]) \
				: (multi == 2)\
				? &(statdump_pkt_data[i][0]) \
				: &(tcpdump_pkt_data[i][0])\
				)
#define multidump_pkt_data_max_len(i)\
				(\
				(multi == 0)\
				? sizeof(tcpdump_pkt_data[i]) \
				: (multi == 1)\
				? sizeof(pwrdump_pkt_data[i]) \
				: (multi == 2)\
				? sizeof(statdump_pkt_data[i]) \
				: 0\
				)

/* */

static int multidump_get(void)
{
	int selected_pkt_log;
	int selected_pkt_head;
	int multi;

	selected_pkt_log = -1;
	selected_pkt_head = -1;
	for (multi = 0; multi < 3; multi++) {
		if (multidump_maxpkt <= 0)
			continue;
		if (multidump_head == multidump_tail)
			continue;
		if ((selected_pkt_log == -1)
		 || time_before(multidump_pkt(multidump_head)->time,
			multidump_pkt_log[selected_pkt_log].pkt
				[selected_pkt_head].time)) {
			selected_pkt_log = multi;
			selected_pkt_head = multidump_head;
		}
	}

	return selected_pkt_log;

}

/* */

DEFINE_SPINLOCK(tcpdump_lock);

static DEFINE_SEMAPHORE(tcpdump_read_lock);

extern int lp0_logs_enable;
static int pkt_save;
static int pkt_rx_save = 1;
static int pkt_tx_save = 1;
static atomic_t insert_dummy_timestamp = ATOMIC_INIT(1);
static struct timespec time_stamp;

struct dummy_time {
	unsigned long time_sec;
	unsigned long jiffies;
	unsigned long time_nsec;
} dt;

static void
tcpdump_set_maxpkt(int maxpkt)
{
	unsigned long flags;

	pr_info("%s: maxpkt %d\n", __func__, maxpkt);

	/* set max packet */
	spin_lock_irqsave(&tcpdump_lock, flags);
	tcpdump_head = 0;
	tcpdump_tail = 0;
	tcpdump_maxpkt = maxpkt;
	spin_unlock_irqrestore(&tcpdump_lock, flags);

}

void
tcpdump_pkt_save(char tag, const char *netif, const char *func, int line,
	const unsigned char *data,
	unsigned int data_nonpaged_len,
	unsigned int data_paged_len)
{
	int multi;
	multidump_pkt_t pkt;
	unsigned int pkt_data_max_len;
	unsigned long flags;
	int i;

	/* pick which log to save packet in - (tcp/pwr/stat/...)dump */
	multi = 0;
	if (tag == TCPDUMP_TAG_PWR)
		multi = 1;
	else if (tag == TCPDUMP_TAG_STAT)
		multi = 2;

	/* check if (tcp/pwr/stat/...)dump enabled */
	if (multidump_maxpkt <= 0)
		return;

	/* check if (tcp/pwr/stat/...)dump packet save enable */
	if ((multi == 0) && (pkt_save == 0))
		return;

	/* copy (tcp/pwr/stat/...)dump pkt */
	pkt.serial_no = 0;
	pkt.time = 0;
	pkt.tag = tag;
	strcpy(pkt.netif, netif);
	pkt.func = func;
	pkt.line = line;
	pkt.data_nonpaged_len = data_nonpaged_len;
	pkt.data_paged_len = data_paged_len;

	/* save (tcp/pwr/stat/...)dump pkt */
	spin_lock_irqsave(&tcpdump_lock, flags);
	if (multidump_maxpkt <= 0) {
		spin_unlock_irqrestore(&tcpdump_lock, flags);
		return;
	} else {
		i = multidump_tail;
		multidump_tail = (multidump_tail + 1) % multidump_maxpkt;
		if (multidump_tail == multidump_head)
			multidump_head = (multidump_head + 1)
				% multidump_maxpkt;
	}
	pkt.serial_no = tcpdump_serial_no++;
	pkt.time = jiffies;
	*(multidump_pkt(i)) = pkt;
	pkt_data_max_len = multidump_pkt_data_max_len(i);
	if (data_nonpaged_len > pkt_data_max_len)
		memcpy(multidump_pkt_data(i), data, pkt_data_max_len);
	else
		memcpy(multidump_pkt_data(i), data, data_nonpaged_len);
	spin_unlock_irqrestore(&tcpdump_lock, flags);

	/* TODO - analyze packet jitter / etc. */

}

void
tegra_sysfs_histogram_tcpdump_rx(struct sk_buff *skb,
	const char *func, int line)
{
	struct net_device *netdev = skb ? skb->dev : NULL;
	char *netif = netdev ? netdev->name : "";

	/* check if rx packet logging enabled */
	if (skb->protocol == ETHER_TYPE_BRCM_REV && lp0_logs_enable == 0)
		return;
	if (!pkt_rx_save)
		return;
	pr_debug_ratelimited("%s: %s(%d): %s\n", __func__, func, line, netif);

	/* save rx packet */
	tcpdump_pkt_save(TCPDUMP_TAG_RX, netif, func, line,
		skb->data, skb_headlen(skb), skb->data_len);

	/* kick off a stat work so we can get counters report */
#if TCPDUMP_RX_STAT_DELAY > 0
	tegra_sysfs_histogram_stat_work_run(TCPDUMP_RX_STAT_DELAY);
#endif

}

void
tegra_sysfs_histogram_tcpdump_tx(struct sk_buff *skb,
	const char *func, int line)
{
	struct net_device *netdev = skb ? skb->dev : NULL;
	char *netif = netdev ? netdev->name : "";

	/* check if tx packet logging enabled */
	if (!pkt_tx_save)
		return;
	pr_debug_ratelimited("%s: %s(%d): %s\n", __func__, func, line, netif);

	/* save tx packet */
	tcpdump_pkt_save(TCPDUMP_TAG_TX, netif, func, line,
		skb->data, skb_headlen(skb), skb->data_len);
}

void
tegra_sysfs_histogram_tcpdump_work_start(void)
{
//	pr_info("%s\n", __func__);

	/* placeholder for tcpdump work */

}

void
tegra_sysfs_histogram_tcpdump_work_stop(void)
{
//	pr_info("%s\n", __func__);

	/* placeholder for tcpdump work */

}

char *
multidump_format_pkt(int multi, char *buf, multidump_pkt_t *pkt,
	unsigned char *pkt_data)
{
	char *s = buf;
	unsigned int pkt_data_max_len = multidump_pkt_data_max_len(0);
	int m, n;
	sprintf(s,
		"[%08lx|%08lx] %c %s: %s(%d): %u+%u\n",
		pkt->serial_no,
		pkt->time,
		pkt->tag,
		pkt->netif,
		pkt->func,
		pkt->line,
		pkt->data_nonpaged_len,
		pkt->data_paged_len);
	s += strlen(s);
	for (m = 0;
		(m < pkt_data_max_len) && (m < pkt->data_nonpaged_len);
		m += n) {
		for (n = 0; n < 16; n++) {
			if (m + n >= pkt_data_max_len)
				break;
			if (m + n >= pkt->data_nonpaged_len)
				break;
			sprintf(s,
				" %02x",
				pkt_data[m + n]);
			s += 3;
		}
		sprintf(s, "\n");
		s++;
	}
	return s;
}

extern const char dummy_inf[];
char *
tcpdump_insert_dummy_timestamp(char *buf)
{
	multidump_pkt_t pkt;
	dt.jiffies = jiffies;
	getnstimeofday(&time_stamp);
	dt.time_sec = time_stamp.tv_sec;
	dt.time_nsec = time_stamp.tv_nsec;

	/* Update the pkt structure for dummy timestamp.
	 * tcpdump_lock not needed as it will be already locked.
	 */
	pkt.time = jiffies;
	pkt.tag = TCPDUMP_TAG_TIME;
	memcpy(pkt.netif, dummy_inf, strlen(dummy_inf)+1); /* +1 for null */
	pkt.func = "dummy_time";
	pkt.line = 0;
	pkt.data_nonpaged_len = sizeof(struct dummy_time);
	pkt.data_paged_len = 0;
	pkt.serial_no = tcpdump_serial_no++;

	atomic_set(&insert_dummy_timestamp, 0);
	return multidump_format_pkt(0 /* 0 = tcpdump pkt log */, buf, &pkt,
		(unsigned char *) &dt);
}

#define SHOW_BUF_DATA_MAXSIZ	/* must be max of *DUMP_DATA_MAXSIZ */\
	STATDUMP_DATA_MAXSIZ

#if	(SHOW_BUF_DATA_MAXSIZ < TCPDUMP_DATA_MAXSIZ) || \
	(SHOW_BUF_DATA_MAXSIZ < PWRDUMP_DATA_MAXSIZ) || \
	(SHOW_BUF_DATA_MAXSIZ < STATDUMP_DATA_MAXSIZ)
#error "SHOW_BUF_DATA_MAXSIZ too small"
#endif

#define SHOW_BUF_MAXSTRLEN\
	(\
	/* 1st line */ \
	80 + \
	/* number of rows (with 16 hexadecimal numbers per row) */ \
	(((SHOW_BUF_DATA_MAXSIZ - 1) / 16) + 1) * \
	/* number of characters per row (of 16 hexadecimal numbers) */ \
	(3 * 16 + 1) \
	)\

static char show_buf[SHOW_BUF_MAXSTRLEN];
static char *show_buf_head;
static char *show_buf_tail;

ssize_t
tegra_sysfs_histogram_tcpdump_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	int multi;
	char *s;
	unsigned int len;
	multidump_pkt_t pkt;
	unsigned long flags;
	int i;

	/* pr_info("%s\n", __func__); */

	/* get/show (tcp/pwr/...)dump pkt(s) */
	for (s = buf; (s - buf) < PAGE_SIZE; ) {
		/* show tail of previous (tcp/pwr/...)dump pkt */
		if ((show_buf_head != NULL)
			&& (show_buf_tail != NULL)
			&& (show_buf_tail != show_buf_head)) {
			len = show_buf_tail - show_buf_head;
			if (len > PAGE_SIZE - (s - buf))
				len = PAGE_SIZE - (s - buf);
			memcpy(s, show_buf_head, len);
			s += len;
			show_buf_head += len;
			if (show_buf_head == show_buf_tail) {
				show_buf_head = NULL;
				show_buf_tail = NULL;
			}
			continue;
		}
		/* get (tcp/pwr/...)dump pkt */
		spin_lock_irqsave(&tcpdump_lock, flags);
		multi = multidump_get();
		if (multi < 0) {
			atomic_set(&insert_dummy_timestamp, 1);
			spin_unlock_irqrestore(&tcpdump_lock, flags);
			return s - buf;
		}
		if (multidump_maxpkt <= 0) {
			spin_unlock_irqrestore(&tcpdump_lock, flags);
			return (s - buf);
		} else if (multidump_head == multidump_tail) {
			spin_unlock_irqrestore(&tcpdump_lock, flags);
//			pr_info("%s: no more tcpdump pkt(s)!\n", __func__);
			return (s - buf);
		} else {
			if (atomic_read(&insert_dummy_timestamp))
				s = tcpdump_insert_dummy_timestamp(s);
			i = multidump_head++;
			if (multidump_head >= multidump_maxpkt)
				multidump_head = 0;
		}
		pkt = *(multidump_pkt(i));
		spin_unlock_irqrestore(&tcpdump_lock, flags);
		/* show (tcp/pwr/...)dump pkt */
		show_buf_head = show_buf;
		show_buf_tail = multidump_format_pkt(multi, show_buf_head,
			&pkt, multidump_pkt_data(i));
	}
	return (s - buf);
}

ssize_t
tegra_sysfs_histogram_tcpdump_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	int maxpkt;
	int err;

//	pr_info("%s\n", __func__);

	if (strncmp(buf, "enable", 6) == 0) {
		pr_info("%s: tcpdump enabled\n", __func__);
		pkt_save = 1;
		maxpkt = sizeof(tcpdump_pkt) / sizeof(tcpdump_pkt[0]);
	} else if (strncmp(buf, "disable", 7) == 0) {
		pr_info("%s: tcpdump disabled\n", __func__);
		pkt_save = 0;
		maxpkt = 0;
	} else if (strncmp(buf, "stop", 4) == 0) {
		pr_info("%s: tcpdump stopped\n", __func__);
		pkt_save = 0;
		return count;
	} else if (strncmp(buf, "start", 5) == 0) {
		pr_info("%s: tcpdump started\n", __func__);
		pkt_save = 1;
		return count;
	} else if (strncmp(buf, "rxstop", 6) == 0) {
		pr_info("%s: tcpdump rxstopped\n", __func__);
		pkt_rx_save = 0;
		return count;
	} else if (strncmp(buf, "rxstart", 7) == 0) {
		pr_info("%s: tcpdump rxstarted\n", __func__);
		pkt_rx_save = 1;
		return count;
	} else if (strncmp(buf, "txstop", 6) == 0) {
		pr_info("%s: tcpdump txstopped\n", __func__);
		pkt_tx_save = 0;
		return count;
	} else if (strncmp(buf, "txstart", 7) == 0) {
		pr_info("%s: tcpdump txstarted\n", __func__);
		pkt_tx_save = 1;
		return count;
	} else if (strncmp(buf, "lp0_logs_start", 14) == 0) {
		pr_info("%s: lp0 logs started\n", __func__);
		lp0_logs_enable = 1;
		return count;
	} else if (strncmp(buf, "lp0_logs_stop", 13) == 0) {
		pr_info("%s: lp0 logs stopped\n", __func__);
		lp0_logs_enable = 0;
		return count;
	} else if (strncmp(buf, "test_pwrdump", 12) == 0) {
		pr_info("%s: test pwrdump - add phony record\n", __func__);
		tcpdump_pkt_save(TCPDUMP_TAG_PWR, "multi0", __func__, __LINE__,
			(unsigned char *) buf + 12, count - 12, 0);
		return count;
	} else if (strncmp(buf, "test_statdump", 13) == 0) {
		pr_info("%s: test statdump - add phony record\n", __func__);
		tcpdump_pkt_save(TCPDUMP_TAG_STAT, "multi0", __func__, __LINE__,
			(unsigned char *) buf + 13, count - 13, 0);
		return count;
	} else {
		maxpkt = -1;
		err = kstrtoint(buf, 0, &maxpkt);
		if (maxpkt < 0) {
			pr_err("%s: ignore invalid maxpkt %d\n",
				__func__, maxpkt);
			return count;
		} else if (maxpkt > sizeof(tcpdump_pkt)
			/ sizeof(tcpdump_pkt[0])) {
			pr_info("%s: limit maxpkt from %d to %ld\n",
				__func__,
				maxpkt,
				sizeof(tcpdump_pkt) / sizeof(tcpdump_pkt[0]));
			maxpkt = sizeof(tcpdump_pkt) / sizeof(tcpdump_pkt[0]);
		}
	}
	tcpdump_set_maxpkt(maxpkt);

	return count;
}

ssize_t
tegra_debugfs_histogram_tcpdump_read(struct file *filp,
	char __user *buff, size_t count, loff_t *offp)
{
	static char buf[PAGE_SIZE];
	struct device *dev = NULL;
	struct device_attribute *attr = NULL;
	ssize_t size, chunk;

//	pr_info("%s\n", __func__);

	/* lock read semaphore */
	if (down_interruptible(&tcpdump_read_lock) < 0) {
		pr_err("%s: cannot lock read semaphore\n",
			__func__);
		return 0;
	}

	for (size = 0; size + PAGE_SIZE <= count; size += chunk) {
		chunk = tegra_sysfs_histogram_tcpdump_show(dev, attr, buf);
		if (chunk <= 0)
			break;
		if (copy_to_user(buff + size, buf, chunk) != 0) {
			pr_err("%s: copy_to_user() failed!\n", __func__);
			break;
		}
	}

	/* unlock read semaphore */
	up(&tcpdump_read_lock);

	return size;
}

ssize_t
tegra_debugfs_histogram_tcpdump_write(struct file *filp,
	const char __user *buff, size_t count, loff_t *offp)
{
//	pr_info("%s\n", __func__);
	return count;
}
