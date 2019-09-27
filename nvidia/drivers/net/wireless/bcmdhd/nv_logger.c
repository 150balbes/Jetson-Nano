/*
 * drivers/net/wireless/bcmdhd/nv_logger.c
 *
 * NVIDIA Tegra Sysfs for BCMDHD driver
 *
 * Copyright (C) 2016 NVIDIA Corporation. All rights reserved.
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

#include <nv_logger.h>

atomic_t list1_val = ATOMIC_INIT(1);
atomic_t list2_val = ATOMIC_INIT(1);
char logbuf[MAX_LOGLIMIT + 128];
char nv_error_buffer[MAX_ERROR_SIZE];
bool enable_file_logging;
struct list_head list1;
struct list_head list2;
bool select_list;

struct workqueue_struct *logger_wqueue;
struct log_buffer {
	char tmstmp[TIMESTAMPSIZE];
	char *buf;
	char *info;
	int event;
	int size;
};

struct log_node {
	struct list_head list;
	struct log_buffer *log;
};

struct work_struct enqueue_work;
static void dhd_log_netlink_deinit(void);
static int dhd_log_netlink_init(void);

void write_log_init()
{
	logger_wqueue = create_workqueue("dhd_log");
	if (logger_wqueue == NULL) {
		pr_err("write_log_init: failed to allocate workqueue\n");
		return;
	}
	INIT_WORK(&enqueue_work, write_queue_work);
	INIT_LIST_HEAD(&list1);
	INIT_LIST_HEAD(&list2);

	if (dhd_log_netlink_init())
		goto init_fail;

	dhd_log_netlink_send_msg(0, 0, 0, NULL, 0);
	enable_file_logging = true;
	select_list = true;
	return;

init_fail:
	destroy_workqueue(logger_wqueue);
	enable_file_logging = false;
}

void write_log_uninit()
{
	pr_info("write_log_uninit\n");

	if (logger_wqueue == NULL)
		return;

	flush_workqueue(logger_wqueue);
	destroy_workqueue(logger_wqueue);
	dhd_log_netlink_deinit();
}

int write_log(int event, const char *buf, const char *info)
{
	struct log_node *temp;
	int buf_len = 0;
	int info_len = 0;
	int time_len = 0;
	struct timeval now;
	struct tm date_time;
	static int count = 0;

	if (!enable_file_logging) {
		return -1;
	}

	if (buf == NULL)
		return -1;

	switch (event) {

	case WLC_E_ESCAN_RESULT:
		break;
	default:
		temp = kmalloc(sizeof(struct log_node), GFP_ATOMIC);
		if (temp == NULL) {
			pr_err("write_log: temp memory allocation failed");
			return -1;
		}
		memset(temp, 0, sizeof(struct log_node));

		temp->log = kmalloc(sizeof(struct log_buffer), GFP_ATOMIC);
		if (temp->log == NULL) {
			pr_err("write_log: log memory allocation failed");
			kfree(temp);
			return -1;
		}

		buf_len = strlen(buf) + 1;
		temp->log->buf = kmalloc(buf_len, GFP_ATOMIC);
		if (temp->log->buf == NULL) {
			pr_err("write_log_buf: log memory allocation failed");
			kfree(temp->log);
			kfree(temp);
			return -1;
		}

		strncpy(temp->log->buf, buf, buf_len);

		do_gettimeofday(&now);
		time_to_tm(now.tv_sec, -sys_tz.tz_minuteswest * 60, &date_time);
		time_len = sprintf(temp->log->tmstmp,
					"[%.2d-%.2d %.2d:%.2d:%.2d.%u]",
					date_time.tm_mon+1,
					date_time.tm_mday,
					date_time.tm_hour,
					date_time.tm_min,
					date_time.tm_sec ,
					(unsigned int)(now.tv_usec/1000));
		if (info != NULL) {
			info_len = strlen(info) + 1;
			temp->log->info = kmalloc(info_len, GFP_ATOMIC);
			if (temp->log->info != NULL)
				strncpy(temp->log->info, info, info_len);
		} else {
			temp->log->info = NULL;
		}
		temp->log->event = event;
		temp->log->size = time_len + buf_len + info_len;
		/* whichever list is not busy, dump data in that list.
		   Make sure we fill the last active list with MAX_LOG_NUM
		   before switching the lists
		*/
		if (select_list && (1 == atomic_read(&list1_val))) {
			count++;
			list_add_tail(&(temp->list), &(list1));
		} else if (!select_list && (1 == atomic_read(&list2_val))) {
			count++;
			list_add_tail(&(temp->list), &(list2));
		} else {
		/* send data directly over netlink because both lists are busy*/
			pr_err("Message dropped due to busy queues");
		}

		if (count == MAX_LOG_NUM) {
			count = 0;
			if (select_list)
				atomic_set(&list1_val, 0);
			else
				atomic_set(&list2_val, 0);
			queue_work(logger_wqueue, &enqueue_work);
			select_list = (select_list == false);
		}
	}
	return buf_len + info_len;
}

void write_queue_work(struct work_struct *work)
{
	struct log_node *temp = NULL;
	struct list_head *pos = NULL, *n = NULL;
	int list1_size = 0;
	int list2_size = 0;

	/* iterate over the listi until list_for_each_safe empties the list.
	   The list is empty is deduced if pos == head, where for eg &(list1)
	   is the head for list1.
	*/

	/* queuing in list1 is blocked, so can dequeue list1*/
	if (atomic_read(&list1_val) == 0) {
		while (pos != &list1) {
			list_for_each_safe(pos, n, &(list1)) {
				if (list1_size > MAX_LOGLIMIT)
					break;
				temp = list_entry(pos, struct log_node, list);
				/* for the correct string of the event */
				strcat(logbuf, temp->log->tmstmp);

				if (temp->log->buf != NULL)
					strcat(logbuf, temp->log->buf);
				strcat(logbuf, " ");
				if (temp->log->info != NULL)
					strcat(logbuf, temp->log->info);
				strcat(logbuf, "\n");
				list1_size += temp->log->size;

				list_del(pos);
				kfree(temp->log->info);
				kfree(temp->log->buf);
				kfree(temp->log);
				kfree(temp);
			}
			write_log_file(logbuf);
			memset(logbuf, '\0', sizeof(logbuf));
			list1_size = 0;
		}
		/* make this list available for writing now */
		atomic_set(&list1_val, 1);

	}

	/* queuing in list1 is blocked, so can dequeue list1*/
	if (atomic_read(&list2_val) == 0) {
		while (pos != &list2) {
			list_for_each_safe(pos, n, &(list2)) {
				if (list1_size > MAX_LOGLIMIT)
					break;
				temp = list_entry(pos, struct log_node, list);
			/* for the correct string of the event */
				strcat(logbuf, temp->log->tmstmp);

				if (temp->log->buf != NULL)
					strcat(logbuf, temp->log->buf);
				strcat(logbuf, " ");
				if (temp->log->info != NULL)
					strcat(logbuf, temp->log->info);
				strcat(logbuf, "\n");
				list2_size += temp->log->size;

				list_del(pos);
				kfree(temp->log->info);
				kfree(temp->log->buf);
				kfree(temp->log);
				kfree(temp);
			}
			write_log_file(logbuf);
			memset(logbuf, '\0', sizeof(logbuf));
			list2_size = 0;
		}
		/* make this list available for writing now */
		atomic_set(&list2_val, 1);
	}

}

void write_log_file(const char *log)
{
	static int seq;
	dhd_log_netlink_send_msg(0, 0, seq++, (void*) log, strlen(log) + 1);
	if (seq % 1024)
		seq = 0;
}

void nvlogger_suspend_work()
{
	enable_file_logging = false;
	pr_info("nvlogger_suspend_work\n");
	cancel_work_sync(&enqueue_work);
}

void nvlogger_resume_work()
{
	pr_info("nvlogger_resume_work\n");
	if (logger_wqueue == NULL)
		return;

	enable_file_logging = true;
}

#define NETLINK_CARBON     29

static struct sock *nl_sk;

static int g_pid;
static void dhd_log_netlink_recv(struct sk_buff *skb)
{

	struct nlmsghdr *nlh;
	nlh = (struct nlmsghdr *)skb->data;

	if (nlh == NULL) {
		pr_err("ids received messaged with null data\n");
		return;
	}

	g_pid = nlh->nlmsg_pid;

	if (g_pid > 0)
		dhd_log_netlink_send_msg(0, 0, 0, "Firmware logs\n", 15);
}

static int dhd_log_netlink_init(void)
{
	struct netlink_kernel_cfg cfg = {
		.input	= dhd_log_netlink_recv,
	};

	pr_info("ids dhd_log_netlink_init\n");
	if (nl_sk != NULL) {
		pr_err("ids nl_sk already assigned\n");
		return 0;
	}

	nl_sk = netlink_kernel_create(&init_net, NETLINK_CARBON, &cfg);

	if (nl_sk == NULL) {
		pr_err("ids netlink create failed\n");
		return -ENOMEM;
	}

	return 0;
}

static void dhd_log_netlink_deinit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

s32
dhd_log_netlink_send_msg(int pid, int type, int seq, void *data, size_t size)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	int ret = -1;

	if (nl_sk == NULL)
		goto nlmsg_failure;

	skb = alloc_skb(NLMSG_SPACE(size), GFP_ATOMIC);
	if (skb == NULL)
		goto nlmsg_failure;

	nlh = nlmsg_put(skb, 0, 0, 0, size, 0);
	if (nlh == NULL) {
		dev_kfree_skb(skb);
		goto nlmsg_failure;
	}

	memcpy(nlmsg_data(nlh), data, size);
	nlh->nlmsg_seq = seq;
	nlh->nlmsg_type = type;

	/* netlink_unicast() takes ownership of the skb and frees it itself. */
	ret = netlink_unicast(nl_sk, skb, g_pid, 0);

nlmsg_failure:
	return ret;
}

void dumplogs(void)
{
	/* try sleeping blocking two queues to avoid blocking both queues */
	pr_info("dumplogs from nv_logger\n");
	atomic_set(&list1_val, 0);
	atomic_set(&list2_val, 0);
	queue_work(logger_wqueue, &enqueue_work);
}

static ssize_t dhdlog_sysfs_enablelog_store(struct device *dev,
			struct device_attribute *attr,
				const char *buf, size_t count)
{
	pr_info("dhdlog_sysfs_enablelog_store = %s", buf);
	if (strncmp(buf, "0", 1) == 0 || strncmp(buf, "false", 5) == 0
		|| strncmp(buf, "no", 2) == 0) {
		enable_file_logging = false;
	} else if (strncmp(buf, "dump", 4) == 0) {
		dumplogs();
	} else if (strncmp(buf, "1", 1) == 0 || strncmp(buf, "true", 4) == 0
		|| strncmp(buf, "yes", 3) == 0) {
		if (logger_wqueue != NULL)
			enable_file_logging = true;
	}

	return count;
}

static ssize_t dhdlog_sysfs_enablelog_show(struct device *dev,
			struct device_attribute *attr,
				char *buf) {

	pr_info("dhdlog_sysfs_enablelog_show");
	return sprintf(buf, "%d\n", enable_file_logging);
}

static DEVICE_ATTR(enablelog, S_IRUGO | S_IWUSR,
		dhdlog_sysfs_enablelog_show,
		dhdlog_sysfs_enablelog_store);

static struct attribute *dhdlog_sysfs_attrs[] = {
	&dev_attr_enablelog.attr,
	NULL,
};

static struct attribute_group dhdlog_sysfs_attr_group = {
	.name = "enablelog",
	.attrs = dhdlog_sysfs_attrs,
};

int dhdlog_sysfs_deinit(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &dhdlog_sysfs_attr_group);
	return 0;
}

int dhdlog_sysfs_init(struct device *dev)
{

	int ret = 0;

	ret = sysfs_create_group(&dev->kobj, &dhdlog_sysfs_attr_group);
	if (ret) {
		pr_err("%s: create_group failed\n", __func__);
	}

	return ret;
}
