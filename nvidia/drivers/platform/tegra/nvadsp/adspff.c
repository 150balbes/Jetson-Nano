/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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

#define pr_fmt(fmt) "adspff: " fmt

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>

#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/semaphore.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/list.h>

#include <linux/tegra_nvadsp.h>

#include "adspff.h"
#include "dev.h"


#define ADSPFF_MAX_OPEN_FILES	(32)

struct file_struct {
	struct file *fp;
	uint8_t file_name[ADSPFF_MAX_FILENAME_SIZE];
	unsigned int flags;
	unsigned long long wr_offset;
	unsigned long long rd_offset;
	struct list_head list;
};

static struct list_head file_list;
static spinlock_t adspff_lock;
static int open_count;

/******************************************************************************
* Kernel file functions
******************************************************************************/

struct file *file_open(const char *path, int flags, int rights)
{
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(get_ds());
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if (IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}
	return filp;
}

void file_close(struct file *file)
{
	filp_close(file, NULL);
}

int file_write(struct file *file, unsigned long long *offset,
				unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	int ret = 0;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_write(file, data, size, offset);

	set_fs(oldfs);
	return ret;
}

uint32_t file_read(struct file *file, unsigned long long *offset,
				unsigned char *data, unsigned int size)
{
	mm_segment_t oldfs;
	uint32_t ret = 0;

	oldfs = get_fs();
	set_fs(get_ds());

	ret = vfs_read(file, data, size, offset);

	set_fs(oldfs);

	return ret;
}

uint32_t file_size(struct file *file)
{
	mm_segment_t oldfs;
	uint32_t size = 0;

	oldfs = get_fs();
	set_fs(get_ds());

	size = vfs_llseek(file, 0, SEEK_END);

	vfs_llseek(file, 0, SEEK_SET);

	set_fs(oldfs);

	return size;
}

/******************************************************************************
* ADSPFF file functions
******************************************************************************/

static struct adspff_shared_state_t *adspff;
static struct nvadsp_mbox rx_mbox;

/**																*
 * w  - open for writing (file need not exist)					*
 * a  - open for appending (file need not exist)				*
 * r+ - open for reading and writing, start at beginning		*
 * w+ - open for reading and writing (overwrite file)			*
 * a+ - open for reading and writing (append if file exists)	*/

void set_flags(union adspff_message_t *m, unsigned int *flags)
{
	if (0 == strcmp(m->msg.payload.fopen_msg.modes, "r+"))
		*flags = O_RDWR;

	else if (0 == strcmp(m->msg.payload.fopen_msg.modes, "w+"))
		*flags = O_CREAT | O_RDWR | O_TRUNC;

	else if (0 == strcmp(m->msg.payload.fopen_msg.modes, "a+"))
		*flags = O_APPEND | O_RDWR;

	else if (0 == strcmp(m->msg.payload.fopen_msg.modes, "r"))
		*flags = O_RDONLY;

	else if (0 == strcmp(m->msg.payload.fopen_msg.modes, "w"))
		*flags = O_CREAT | O_WRONLY | O_TRUNC;

	else if (0 == strcmp(m->msg.payload.fopen_msg.modes, "a"))
		*flags = O_CREAT | O_APPEND | O_WRONLY;

	else
		*flags = O_CREAT | O_RDWR;
}

/*
 *	checks if file is already opened
 *	if yes, then returns the struct file_struct for the file
 *	if no, then allocates a file_struct and adds to the list
 *	and returns the pointer to the newly allocated file_struct
 *	if ADSPFF_MAX_OPEN_FILES already open, returns NULL
 */
static struct file_struct *check_file_opened(const char *path)
{
	struct file_struct *file = NULL;
	struct list_head *pos;

	/* assuming files opened by ADSP will
	 * never be actually closed in kernel
	 */
	list_for_each(pos, &file_list) {
		file = list_entry(pos, struct file_struct, list);
		if (!file->fp)
			break;
		if (!strncmp(path, file->file_name,
					ADSPFF_MAX_FILENAME_SIZE)) {
			break;
		}
		file = NULL;
	}

	if (file != NULL)
		return file;

	if (open_count == ADSPFF_MAX_OPEN_FILES) {
		pr_err("adspff: %d files already opened\n",
			ADSPFF_MAX_OPEN_FILES);
		file = NULL;
	} else {
		file = kzalloc(sizeof(*file), GFP_KERNEL);
		open_count++;
		list_add_tail(&file->list, &file_list);
	}
	return file;
}

void adspff_fopen(void)
{
	union adspff_message_t *message;
	union adspff_message_t *msg_recv;
	unsigned int flags = 0;
	int ret = 0;
	struct file_struct *file;


	message = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!message)
		return;


	msg_recv = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!msg_recv) {
		kfree(message);
		return;
	}

	message->msgq_msg.size = MSGQ_MSG_SIZE(struct fopen_msg_t);

	ret = msgq_dequeue_message(&adspff->msgq_send.msgq,
			(msgq_message_t *)message);

	if (ret < 0) {
		pr_err("fopen Dequeue failed %d.", ret);
		kfree(message);
		kfree(msg_recv);
		return;
	}

	file = check_file_opened(message->msg.payload.fopen_msg.fname);
	if (file && !file->fp) {
		/* open a new file */
		set_flags(message, &flags);
		pr_info("adspff: opening file %s\n",
			message->msg.payload.fopen_msg.fname);

		file->fp = file_open(
			(const char *)message->msg.payload.fopen_msg.fname,
			flags, S_IRWXU | S_IRWXG | S_IRWXO);

		file->wr_offset = 0;
		file->rd_offset = 0;
		memcpy(file->file_name,
				message->msg.payload.fopen_msg.fname,
				ADSPFF_MAX_FILENAME_SIZE);
		file->flags = flags;
	}

	if (file && !file->fp) {
		file = NULL;
		pr_err("File not found - %s\n",
			(const char *) message->msg.payload.fopen_msg.fname);
	}

	msg_recv->msgq_msg.size = MSGQ_MSG_SIZE(struct fopen_recv_msg_t);
	msg_recv->msg.payload.fopen_recv_msg.file = (int64_t)file;

	ret = msgq_queue_message(&adspff->msgq_recv.msgq,
			(msgq_message_t *)msg_recv);
	if (ret < 0) {
		pr_err("fopen Enqueue failed %d.", ret);

		if (file) {
			file_close(file->fp);
			file->fp = NULL;
		}

		kfree(message);
		kfree(msg_recv);
		return;
	}

	nvadsp_mbox_send(&rx_mbox, adspff_cmd_fopen_recv,
				NVADSP_MBOX_SMSG, 0, 0);
	kfree(message);
	kfree(msg_recv);
}

static inline unsigned int is_read_file(struct file_struct *file)
{
	return ((!file->flags) || (file->flags & O_RDWR));
}

static inline unsigned int is_write_file(struct file_struct *file)
{
	return file->flags & (O_WRONLY | O_RDWR);
}

void adspff_fclose(void)
{
	union adspff_message_t *message;
	struct file_struct *file = NULL;
	int32_t ret = 0;

	message = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!message)
		return;

	message->msgq_msg.size = MSGQ_MSG_SIZE(struct fclose_msg_t);

	ret = msgq_dequeue_message(&adspff->msgq_send.msgq,
				(msgq_message_t *)message);

	if (ret < 0) {
		pr_err("fclose Dequeue failed %d.", ret);
		kfree(message);
		return;
	}

	file = (struct file_struct *)message->msg.payload.fclose_msg.file;
	if (file) {
		if ((file->flags & O_APPEND) == 0) {
			if (is_read_file(file))
				file->rd_offset = 0;
			if (is_write_file(file))
				file->wr_offset = 0;
		}
	}
	kfree(message);
}

void adspff_fsize(void)
{
	union adspff_message_t *msg_recv;
	union adspff_message_t message;
	struct file_struct *file = NULL;
	int32_t ret = 0;
	uint32_t size = 0;

	msg_recv = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	msg_recv->msgq_msg.size = MSGQ_MSG_SIZE(struct ack_msg_t);

	message.msgq_msg.size = MSGQ_MSG_SIZE(struct fsize_msg_t);
	ret = msgq_dequeue_message(&adspff->msgq_send.msgq,
				(msgq_message_t *)&message);

	if (ret < 0) {
		pr_err("fsize Dequeue failed %d.", ret);
		kfree(msg_recv);
		return;
	}
	file = (struct file_struct *)message.msg.payload.fsize_msg.file;
	if (file) {
		size = file_size(file->fp);
	}

	/* send ack */
	msg_recv->msg.payload.ack_msg.size = size;
	ret = msgq_queue_message(&adspff->msgq_recv.msgq,
			(msgq_message_t *)msg_recv);

	if (ret < 0) {
		pr_err("fsize Enqueue failed %d.", ret);
		kfree(msg_recv);
		return;
	}
	nvadsp_mbox_send(&rx_mbox, adspff_cmd_ack,
			NVADSP_MBOX_SMSG, 0, 0);
	kfree(msg_recv);
}

void adspff_fwrite(void)
{
	union adspff_message_t message;
	union adspff_message_t *msg_recv;
	struct file_struct *file = NULL;
	int ret = 0;
	uint32_t size = 0;
	uint32_t bytes_to_write = 0;
	uint32_t bytes_written = 0;

	msg_recv = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!msg_recv)
		return;

	msg_recv->msgq_msg.size = MSGQ_MSG_SIZE(struct ack_msg_t);

	message.msgq_msg.size = MSGQ_MSG_SIZE(struct fwrite_msg_t);
	ret = msgq_dequeue_message(&adspff->msgq_send.msgq,
				(msgq_message_t *)&message);
	if (ret < 0) {
		pr_err("fwrite Dequeue failed %d.", ret);
		return;
	}

	file = (struct file_struct *)message.msg.payload.fwrite_msg.file;
	size = message.msg.payload.fwrite_msg.size;

	bytes_to_write = ((adspff->write_buf.read_index + size) < ADSPFF_SHARED_BUFFER_SIZE) ?
		size : (ADSPFF_SHARED_BUFFER_SIZE - adspff->write_buf.read_index);
	ret = file_write(file->fp, &file->wr_offset,
			adspff->write_buf.data + adspff->write_buf.read_index, bytes_to_write);
	bytes_written += ret;

	if ((size - bytes_to_write) > 0) {
		ret = file_write(file->fp, &file->wr_offset,
				adspff->write_buf.data, size - bytes_to_write);
		bytes_written += ret;
	}

	adspff->write_buf.read_index =
		(adspff->write_buf.read_index + size) % ADSPFF_SHARED_BUFFER_SIZE;

	/* send ack */
	msg_recv->msg.payload.ack_msg.size = bytes_written;
	ret = msgq_queue_message(&adspff->msgq_recv.msgq,
			(msgq_message_t *)msg_recv);

	if (ret < 0) {
		pr_err("adspff: fwrite Enqueue failed %d.", ret);
		kfree(msg_recv);
		return;
	}
	nvadsp_mbox_send(&rx_mbox, adspff_cmd_ack,
			NVADSP_MBOX_SMSG, 0, 0);
	kfree(msg_recv);
}

void adspff_fread(void)
{
	union adspff_message_t *message;
	union adspff_message_t *msg_recv;
	struct file_struct *file = NULL;
	uint32_t bytes_free;
	uint32_t wi = adspff->read_buf.write_index;
	uint32_t ri = adspff->read_buf.read_index;
	uint8_t can_wrap = 0;
	uint32_t size = 0, size_read = 0;
	int32_t ret = 0;

	if (ri <= wi) {
		bytes_free = ADSPFF_SHARED_BUFFER_SIZE - wi + ri - 1;
		can_wrap = 1;
	} else {
		bytes_free = ri - wi - 1;
		can_wrap = 0;
	}
	message = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!message)
		return;

	msg_recv = kzalloc(sizeof(union adspff_message_t), GFP_KERNEL);
	if (!msg_recv) {
		kfree(message);
		return;
	}

	msg_recv->msgq_msg.size = MSGQ_MSG_SIZE(struct ack_msg_t);
	message->msgq_msg.size = MSGQ_MSG_SIZE(struct fread_msg_t);

	ret = msgq_dequeue_message(&adspff->msgq_send.msgq,
				(msgq_message_t *)message);

	if (ret < 0) {
		pr_err("fread Dequeue failed %d.", ret);
		kfree(message);
		kfree(msg_recv);
		return;
	}

	file = (struct file_struct *)message->msg.payload.fread_msg.file;
	size = message->msg.payload.fread_msg.size;
	if (bytes_free < size) {
		size_read = 0;
		goto send_ack;
	}

	if (can_wrap) {
		uint32_t bytes_to_read = (size < (ADSPFF_SHARED_BUFFER_SIZE - wi)) ?
			size : (ADSPFF_SHARED_BUFFER_SIZE - wi);
		ret = file_read(file->fp, &file->rd_offset,
				adspff->read_buf.data + wi, bytes_to_read);
		size_read = ret;
		if (ret < bytes_to_read)
			goto send_ack;
		if ((size - bytes_to_read) > 0) {
			ret = file_read(file->fp, &file->rd_offset,
					adspff->read_buf.data, size - bytes_to_read);
			size_read += ret;
			goto send_ack;
		}
	} else {
		ret = file_read(file->fp, &file->rd_offset,
				adspff->read_buf.data + wi, size);
		size_read = ret;
		goto send_ack;
	}
send_ack:
	msg_recv->msg.payload.ack_msg.size = size_read;
	ret = msgq_queue_message(&adspff->msgq_recv.msgq,
			(msgq_message_t *)msg_recv);

	if (ret < 0) {
		pr_err("fread Enqueue failed %d.", ret);
		kfree(message);
		kfree(msg_recv);
		return;
	}
	adspff->read_buf.write_index =
		(adspff->read_buf.write_index + size_read) % ADSPFF_SHARED_BUFFER_SIZE;

	nvadsp_mbox_send(&rx_mbox, adspff_cmd_ack,
			NVADSP_MBOX_SMSG, 0, 0);
	kfree(message);
	kfree(msg_recv);
}


static const struct sched_param param = {
	.sched_priority = MAX_RT_PRIO - 1,
};
static struct task_struct *adspff_kthread;
static struct list_head adspff_kthread_msgq_head;
static wait_queue_head_t  wait_queue;

struct adspff_kthread_msg {
	uint32_t msg_id;
	struct list_head list;
};


static int adspff_kthread_fn(void *data)
{
	int ret = 0;
	struct adspff_kthread_msg *kmsg;
	unsigned long flags;

	while (1) {

		ret = wait_event_interruptible(wait_queue, kthread_should_stop()
				 || !list_empty(&adspff_kthread_msgq_head));

		if (kthread_should_stop())
			do_exit(0);

		if (!list_empty(&adspff_kthread_msgq_head)) {
			kmsg = list_first_entry(&adspff_kthread_msgq_head,
					struct adspff_kthread_msg, list);
			switch (kmsg->msg_id) {
			case adspff_cmd_fopen:
				adspff_fopen();
				break;
			case adspff_cmd_fclose:
				adspff_fclose();
				break;
			case adspff_cmd_fwrite:
				adspff_fwrite();
				break;
			case adspff_cmd_fread:
				adspff_fread();
				break;
			case adspff_cmd_fsize:
				adspff_fsize();
				break;
			default:
				pr_warn("adspff: kthread unsupported msg %d\n",
					kmsg->msg_id);
			}
			spin_lock_irqsave(&adspff_lock, flags);
			list_del(&kmsg->list);
			spin_unlock_irqrestore(&adspff_lock, flags);
			kfree(kmsg);
		}
	}

	do_exit(ret);
}

/******************************************************************************
* ADSP mailbox message handler
******************************************************************************/


static int adspff_msg_handler(uint32_t msg, void *data)
{
	unsigned long flags;
	struct adspff_kthread_msg *kmsg;

	spin_lock_irqsave(&adspff_lock, flags);
	kmsg = kzalloc(sizeof(*kmsg), GFP_ATOMIC);
	if (!kmsg) {
		spin_unlock_irqrestore(&adspff_lock, flags);
		return -ENOMEM;
	}

	kmsg->msg_id = msg;
	list_add_tail(&kmsg->list, &adspff_kthread_msgq_head);

	wake_up(&wait_queue);
	spin_unlock_irqrestore(&adspff_lock, flags);

	return 0;
}

static int adspff_set(void *data, u64 val)
{
	struct file_struct *file;
	struct list_head *pos, *n;

	if (val != 1)
		return 0;
	list_for_each_safe(pos, n, &file_list) {
		file = list_entry(pos, struct file_struct, list);
		list_del(pos);
		if (file->fp)
			file_close(file->fp);
		kfree(file);
	}

	open_count = 0;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adspff_fops, NULL, adspff_set, "%llu\n");

static int adspff_debugfs_init(struct nvadsp_drv_data *drv)
{
	int ret = -ENOMEM;
	struct dentry *d, *dir;

	if (!drv->adsp_debugfs_root)
		return ret;
	dir = debugfs_create_dir("adspff", drv->adsp_debugfs_root);
	if (!dir)
		return ret;

	d = debugfs_create_file(
			"close_files", S_IWUGO, dir, NULL, &adspff_fops);
	if (!d)
		return ret;

	return 0;
}

int adspff_init(struct platform_device *pdev)
{
	int ret = 0;
	nvadsp_app_handle_t handle;
	nvadsp_app_info_t *app_info;
	struct nvadsp_drv_data *drv = platform_get_drvdata(pdev);

	handle = nvadsp_app_load("adspff", "adspff.elf");
	if (!handle)
		return -1;

	app_info = nvadsp_app_init(handle, NULL);
	if (!app_info) {
		pr_err("unable to init app adspff\n");
		return -1;
	}

	adspff = ADSPFF_SHARED_STATE(app_info->mem.shared);

	ret = nvadsp_mbox_open(&rx_mbox, &adspff->mbox_id,
			"adspff", adspff_msg_handler, NULL);

	if (ret < 0) {
		pr_err("Failed to open mbox %d", adspff->mbox_id);
		return -1;
	}

	spin_lock_init(&adspff_lock);

	ret = adspff_debugfs_init(drv);
	if (ret)
		pr_warn("adspff: failed to create debugfs entry\n");

	INIT_LIST_HEAD(&adspff_kthread_msgq_head);
	INIT_LIST_HEAD(&file_list);

	// kthread inIt
	init_waitqueue_head(&wait_queue);
	adspff_kthread = kthread_create(adspff_kthread_fn,
		NULL, "adspp_kthread");
	sched_setscheduler(adspff_kthread, SCHED_FIFO, &param);
	get_task_struct(adspff_kthread);
	wake_up_process(adspff_kthread);

	return ret;
}

void adspff_exit(void)
{
	nvadsp_mbox_close(&rx_mbox);
	kthread_stop(adspff_kthread);
	put_task_struct(adspff_kthread);
}
