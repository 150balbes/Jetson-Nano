/*
 *
 * Touch Screen USB Driver for EETI Controller
 *
 * Copyright (C) 2000-2018  eGalax_eMPIA Technology Inc. All rights reserved.
 * Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define RELEASE_DATE "2018/04/27"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/input/mt.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#define EETI_USB_DEVICE_ID	0x0EEF
#define EETI_USB_PRODUCT_ID	0xCC01
#define EETI_USB_BL_PRODUCT_ID	0x79FD

#define MAX_EVENTS		600
#define MAX_USB_LEN		64U
#define FIFO_SIZE		8192
#define MAX_SUPPORT_POINT	16
#define REPORTID_VENDOR		0x03
#define REPORTID_MTOUCH		0x06
#define MAX_RESOLUTION		4095
#define MAX_Z_RESOLUTION	1023

struct tag_mt_contacts {
	unsigned char id;
	signed char status;
	unsigned short x;
	unsigned short y;
	unsigned short z;
};

struct _egalax_usb {
	unsigned char *data;
	dma_addr_t data_dma;
	unsigned char *buffer;
	int buf_len;
	struct urb *irq;
	struct usb_device *udev;
	struct usb_interface *interface;
	unsigned char work_state;
	unsigned char down_cnt;
	char phys[128];
	wait_queue_head_t sysfs_query_queue;
	bool sysfs_query_wait;
	unsigned char sysfs_hook_cmd[3];
	unsigned char sysfs_cmd_result[MAX_USB_LEN];
};

struct egalax_char_dev {
	int open_cnts;
	struct kfifo data_kfifo;
	unsigned char *p_fifo_buf;
	spinlock_t fifo_lock;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

static struct _egalax_usb *p_egalax_usb_dev;
static struct egalax_char_dev *p_char_dev;
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static atomic_t wait_command_ack = ATOMIC_INIT(0);
static struct input_dev *input_dev;
static struct tag_mt_contacts p_contact_buf[MAX_SUPPORT_POINT];
static char fifo_read_buf[MAX_USB_LEN];
static int total_pts_cnt, recv_pts_cnt;
static bool misc_registered;
static bool sysfs_created;

/* DT for platform */
int g_reset_gpio;
bool g_enable_high;
struct regulator *g_regulator_hv;
struct regulator *g_regulator_5v0;
struct regulator *g_regulator_3v3;
struct regulator *g_regulator_1v8;
bool g_flip_x;
bool g_flip_y;

#define DBG_MODULE	0x00000001U
#define DBG_CDEV	0x00000002U
#define DBG_PROC	0x00000004U
#define DBG_POINT	0x00000008U
#define DBG_INT		0x00000010U
#define DBG_USB		0x00000020U
#define DBG_SUSP	0x00000040U
#define DBG_INPUT	0x00000080U
#define DBG_CONST	0x00000100U
#define DBG_IDLE	0x00000200U
#define DBG_WAKEUP	0x00000400U
#define DBG_BUTTON	0x00000800U
static unsigned int dbg_level = DBG_MODULE|DBG_SUSP;

#define PROC_FS_NAME	"egalax_dbg"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *p_dbg_proc_file;

#define EGALAX_DBG(level, fmt, args...) \
do { if ((level & dbg_level) > 0U) { \
pr_debug("egalax_usb: " fmt, ## args); } \
} while (false)

#define EETI_HID_SET_REPORT_VALUE	0x203
static int send_usb_data(struct _egalax_usb *egalax_usb, unsigned char *buf,
			 int len)
{
	int ret;
	struct usb_interface *intf = egalax_usb->interface;

	ret = usb_control_msg(egalax_usb->udev,
		usb_sndctrlpipe(egalax_usb->udev, 0),
		HID_REQ_SET_REPORT,
		USB_DIR_OUT|USB_TYPE_CLASS|USB_RECIP_INTERFACE,
		EETI_HID_SET_REPORT_VALUE,
		intf->cur_altsetting->desc.bInterfaceNumber,
		buf, len, USB_CTRL_SET_TIMEOUT);

	if (ret < 0) {
		EGALAX_DBG(DBG_MODULE,
		"Can't send data to device with err:%d\n", ret);
	}
	return ret;
}

static void run_fw_init(struct _egalax_usb *egalax_usb)
{
	unsigned char *buf;
	char retry = 3, ret = 0;

	buf = kzalloc(3, GFP_NOIO);
	if (buf == NULL) {
		EGALAX_DBG(DBG_MODULE, " Can't alloc buffer to do FW init\n");
		return;
	}

	while (retry--) {
		buf[0] = 0x05; buf[1] = 0x02; buf[2] = 0x00;
		ret = send_usb_data(egalax_usb, buf, 3);
		if (ret < 0) {
			EGALAX_DBG(DBG_MODULE,
			" Send FW init command failed!\n");
		} else
			break;
	}

	if (ret < 0)
		EGALAX_DBG(DBG_MODULE, " %s faiiled!\n", __func__);
	else
		EGALAX_DBG(DBG_MODULE, " %s done!\n", __func__);

	kfree(buf);
}

static int egalax_cdev_open(struct inode *inode, struct file *filp)
{
	if (!atomic_dec_and_test(&egalax_char_available)) {
		atomic_inc(&egalax_char_available);
		return -EBUSY;
	}

	p_char_dev->open_cnts++;
	filp->private_data = p_char_dev;

	EGALAX_DBG(DBG_CDEV, " CDev open done!\n");
	try_module_get(THIS_MODULE);
	return 0;
}

static int egalax_cdev_release(struct inode *inode, struct file *filp)
{
	struct egalax_char_dev *cdev = filp->private_data;

	atomic_inc(&egalax_char_available);

	cdev->open_cnts--;

	kfifo_reset(&cdev->data_kfifo);

	EGALAX_DBG(DBG_CDEV, " CDev release done!\n");
	module_put(THIS_MODULE);
	return 0;
}

static ssize_t egalax_cdev_read(struct file *file, char __user *buf,
				size_t count, loff_t *offset)
{
	int read_cnt, ret, fifo_len;
	struct egalax_char_dev *cdev = file->private_data;

	if (down_interruptible(&cdev->sem))
		return -ERESTARTSYS;

	fifo_len = kfifo_len(&cdev->data_kfifo);

	while (fifo_len < 1) {
		/* release the lock */
		up(&cdev->sem);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		if (wait_event_interruptible(cdev->fifo_inq,
					kfifo_len(&cdev->data_kfifo) > 0)) {
			/* signal: tell the fs layer to handle it */
			return -ERESTARTSYS;
		}

		if (down_interruptible(&cdev->sem))
			return -ERESTARTSYS;
	}

	if (count > MAX_USB_LEN)
		count = MAX_USB_LEN;

	read_cnt = kfifo_out_locked(&cdev->data_kfifo, fifo_read_buf, count,
				    &cdev->fifo_lock);

	EGALAX_DBG(DBG_CDEV, " \"%s\" reading fifo data count=%d\n",
		   current->comm, read_cnt);

	ret = copy_to_user(buf, fifo_read_buf, read_cnt) ? -EFAULT : read_cnt;

	up(&cdev->sem);

	return ret;
}

static ssize_t egalax_cdev_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct egalax_char_dev *cdev = file->private_data;
	int ret = 0;
	char *tmp;

	if (p_egalax_usb_dev == NULL)
		return -ENODEV;

	if (down_interruptible(&cdev->sem))
		return -ERESTARTSYS;

	if (count > MAX_USB_LEN)
		count = MAX_USB_LEN;

	tmp = kzalloc(MAX_USB_LEN, GFP_NOIO);
	if (tmp == NULL) {
		up(&cdev->sem);
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		up(&cdev->sem);
		kfree(tmp);
		return -EFAULT;
	}

	ret = send_usb_data(p_egalax_usb_dev, tmp, MAX_USB_LEN);

	up(&cdev->sem);
	EGALAX_DBG(DBG_CDEV, " USB writing %d bytes.\n", ret);
	kfree(tmp);

	return (ret < 0 ? -1 : count);
}

static unsigned int egalax_cdev_poll(struct file *filp,
				     struct poll_table_struct *wait)
{
	struct egalax_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifo_len;

	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq,  wait);

	fifo_len = kfifo_len(&cdev->data_kfifo);

	if (fifo_len > 0)
		mask |= POLLIN | POLLRDNORM;

	if ((FIFO_SIZE - fifo_len) > MAX_USB_LEN)
		mask |= POLLOUT | POLLWRNORM;

	up(&cdev->sem);
	return mask;
}

static int egalax_proc_show(struct seq_file *seqfilp, void *v)
{
	seq_printf(seqfilp,
	"EETI USB for All Points.\nDebug Level: 0x%08X\nRelease Date: %s\n",
	dbg_level, RELEASE_DATE);

	return 0;
}

static int egalax_proc_open(struct inode *inode, struct file *filp)
{
	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_open\n", current->comm);
	return single_open(filp, egalax_proc_show, NULL);
}

static ssize_t egalax_proc_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *offset)
{
	char procfs_buffer_size = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int new_level = 0;

	EGALAX_DBG(DBG_PROC, " \"%s\" call proc_write\n", current->comm);

	procfs_buffer_size = count;
	if (procfs_buffer_size > PROC_FS_MAX_LEN)
		procfs_buffer_size = PROC_FS_MAX_LEN+1;

	if (copy_from_user(procfs_buf, buf, procfs_buffer_size)) {
		EGALAX_DBG(DBG_PROC, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	if (!kstrtouint(procfs_buf, 16, &new_level))
		dbg_level = new_level;

	EGALAX_DBG(DBG_PROC, " Switch Debug Level to 0x%08X\n", dbg_level);

	return procfs_buffer_size;
}

static bool sys_sendcmd_wait(unsigned char *by_send_cmd, int n_send_cmd_len,
			     unsigned char *by_hook_cmd, int n_hook_cmd_len,
			     int nTimeOut)
{
	int i;
	bool b_ret = true;

	if (p_egalax_usb_dev == NULL)
		return false;

	memset(p_egalax_usb_dev->sysfs_cmd_result, 0,
		sizeof(p_egalax_usb_dev->sysfs_cmd_result));

	for (i = 0; i < 3; i++) {
		if (i < n_hook_cmd_len)
			p_egalax_usb_dev->sysfs_hook_cmd[i] = by_hook_cmd[i];
		else
			p_egalax_usb_dev->sysfs_hook_cmd[i] = 0xFF;
	}
	p_egalax_usb_dev->sysfs_query_wait = true;

	if (send_usb_data(p_egalax_usb_dev, by_send_cmd, n_send_cmd_len) < 0) {
		b_ret = false;
	} else {
		wait_event_interruptible_timeout(
			p_egalax_usb_dev->sysfs_query_queue,
			!p_egalax_usb_dev->sysfs_query_wait,
			nTimeOut);

		if (p_egalax_usb_dev->sysfs_query_wait)
			b_ret = false;
		else
			b_ret = true;
	}
	p_egalax_usb_dev->sysfs_query_wait = false;
	return b_ret;
}

#define OP_MODE_GET     0x00
#define OP_MODE_SET     0x01
static ssize_t sys_show_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x91, 0x01, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: %s\n",
			RELEASE_DATE, p_egalax_usb_dev->sysfs_cmd_result+6);
	else
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: Invalid\n",
			RELEASE_DATE);
}

static ssize_t sys_show_touchevent(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x91, 0x02, OP_MODE_GET};
	bool b_ret = true;
	int code = 0;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret) {
		code = p_egalax_usb_dev->sysfs_cmd_result[6];
		code += (p_egalax_usb_dev->sysfs_cmd_result[7]<<8);
		code += (p_egalax_usb_dev->sysfs_cmd_result[8]<<16);
		code += (p_egalax_usb_dev->sysfs_cmd_result[9]<<24);
		return snprintf(buf, PAGE_SIZE, "0x%08X\n", code);
	} else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

static ssize_t sys_show_reportmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x91, 0x04, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			p_egalax_usb_dev->sysfs_cmd_result[6]);
	else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_REPORTMODE_MAX  0x06
static ssize_t sys_store_reportmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x05, 0x36, 0x91, 0x04, OP_MODE_SET};
	bool b_ret = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0] - '0';
	if (mode > NV_REPORTMODE_MAX || mode < 0)
		return -EINVAL;

	send_cmd_buf[6] = mode;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return count;
	else
		return -EIO;
}

static ssize_t sys_show_bypassmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x91, 0x05, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			p_egalax_usb_dev->sysfs_cmd_result[6]);
	else
	return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_BYPASSMODE_MAX  0x02
static ssize_t sys_store_bypassmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x05, 0x36, 0x91, 0x05, OP_MODE_SET};
	bool b_ret = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0]-'0';
	if (mode > NV_BYPASSMODE_MAX || mode < 0)
	return -EINVAL;

	send_cmd_buf[6] = mode;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return count;
	else
		return -EIO;
}

static ssize_t sys_show_calibration(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
			0x03, 0x02, 0x3F, 0x4E};
	bool b_ret = true;
	unsigned int status = 0;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret) {
		status = p_egalax_usb_dev->sysfs_cmd_result[4];
		status += p_egalax_usb_dev->sysfs_cmd_result[5]<<8;
		if (status&0x00000100)
			return sprintf(buf, "0xff\n");
		else
			return sprintf(buf, "0x00\n");
	} else
		return sprintf(buf, "0x00\n");
}

static DEVICE_ATTR(version, 0440, sys_show_version, NULL);
static DEVICE_ATTR(touch_event, 0440, sys_show_touchevent, NULL);
static DEVICE_ATTR(report_mode, 0640, sys_show_reportmode,
		sys_store_reportmode);
static DEVICE_ATTR(bypass_mode, 0640, sys_show_bypassmode,
		sys_store_bypassmode);
static DEVICE_ATTR(calibration, 0440, sys_show_calibration, NULL);

static struct attribute *egalax_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_touch_event.attr,
	&dev_attr_report_mode.attr,
	&dev_attr_bypass_mode.attr,
	&dev_attr_calibration.attr,
	NULL,
};

static const struct attribute_group egalax_attr_group = {
	.attrs = egalax_attributes,
};

#define STYLUS_MASK 0x10
#define MAX_POINT_PER_PACKET    5U
#define POINT_STRUCT_SIZE   10U
static void ProcessParallelReport(unsigned char *buf,
				  struct _egalax_usb *p_egalax_usb)
{
	unsigned char i, index = 0, cnt_down = 0, cnt_up = 0, shift = 0;
	unsigned char status = 0;
	unsigned short contact_id = 0, x = 0, y = 0, z = 0;

	if (total_pts_cnt <= 0) {
		if (buf[1] == 0 || buf[1] > MAX_SUPPORT_POINT) {
			EGALAX_DBG(DBG_POINT,
				" NumsofContacts mismatch, skip packet\n");
			return;
		}

		total_pts_cnt = buf[1];
		recv_pts_cnt = 0;
	} else if (buf[1] > 0) {
		total_pts_cnt = 0;
		recv_pts_cnt = 0;
		EGALAX_DBG(DBG_POINT,
			" NumsofContacts mismatch, skip packet\n");
		return;
	}

	while (index < MAX_POINT_PER_PACKET) {
		shift = index * POINT_STRUCT_SIZE + 2;
		status = buf[shift];
		contact_id = buf[shift+1];
		x = ((buf[shift+3]<<8) + buf[shift+2]);
		y = ((buf[shift+5]<<8) + buf[shift+4]);
		z = ((buf[shift+7]<<8) + buf[shift+6]);

		if (contact_id >= MAX_SUPPORT_POINT) {
			total_pts_cnt = 0;
			recv_pts_cnt = 0;
			EGALAX_DBG(DBG_POINT, " Get error ContactID.\n");
			return;
		}

		EGALAX_DBG(DBG_POINT,
			"Get Point[%d] Update: Status=%d X=%d Y=%d\n",
			contact_id, status, x, y);

    #ifdef _SWITCH_XY
		short tmp = x;

		x = y;
		y = tmp;
    #endif

		if (g_flip_x)
			x = MAX_RESOLUTION - x;


		if (g_flip_y)
			y = MAX_RESOLUTION - y;

		p_contact_buf[recv_pts_cnt].id = contact_id;
		p_contact_buf[recv_pts_cnt].status = status;
		p_contact_buf[recv_pts_cnt].x = x;
		p_contact_buf[recv_pts_cnt].y = y;
		p_contact_buf[recv_pts_cnt].z = z;

		recv_pts_cnt++;
		index++;

		/* Recv all points, send input report */
		if (recv_pts_cnt == total_pts_cnt) {
			for (i = 0; i < recv_pts_cnt; i++) {
				input_mt_slot(input_dev, p_contact_buf[i].id);
				if ((p_contact_buf[i].status &
					STYLUS_MASK) != 0) {
					input_mt_report_slot_state(input_dev,
						MT_TOOL_PEN,
						((p_contact_buf[i].status&0x01)
						!= 0));
				} else {
					input_mt_report_slot_state(input_dev,
						MT_TOOL_FINGER,
						((p_contact_buf[i].status&0x01)
						!= 0));
				}

				if ((p_contact_buf[i].status & 0x01) != 0) {
					input_report_abs(input_dev,
							ABS_MT_POSITION_X,
							p_contact_buf[i].x);
					input_report_abs(input_dev,
							ABS_MT_POSITION_Y,
							p_contact_buf[i].y);
					input_report_abs(input_dev,
							ABS_MT_PRESSURE,
							p_contact_buf[i].z);
				}

				if (p_contact_buf[i].status)
					cnt_down++;
				else
					cnt_up++;
			}

			input_sync(input_dev);

			EGALAX_DBG(DBG_POINT,
			"Input sync point data done! (Down:%d Up:%d)\n",
			cnt_down, cnt_up);

			total_pts_cnt = 0;
			recv_pts_cnt = 0;
			return;
		}
	}
}

static void egalax_usb_measure(struct _egalax_usb *egalax_usb,
			       unsigned char *buf, int len)
{
	int loop = 3, ret;

	EGALAX_DBG(DBG_USB, " %s\n", __func__);

	if (buf[0] != REPORTID_MTOUCH && buf[0] != REPORTID_VENDOR) {
		EGALAX_DBG(DBG_USB,
		"USB read error data with Len=%d hedaer=%d\n", len, buf[0]);
		return;
	}

	switch (buf[0]) {
	case REPORTID_VENDOR:
		EGALAX_DBG(DBG_USB, " USB get vendor command packet\n");
		atomic_set(&wait_command_ack, 1);
		if (egalax_usb->sysfs_query_wait &&
			egalax_usb->sysfs_hook_cmd[0] == buf[2] &&
			((egalax_usb->sysfs_hook_cmd[1] == 0xFF) ||
			egalax_usb->sysfs_hook_cmd[1] == buf[3]) &&
			((egalax_usb->sysfs_hook_cmd[2] == 0xFF) ||
			egalax_usb->sysfs_hook_cmd[2] == buf[4])) {
			memcpy(egalax_usb->sysfs_cmd_result,
				buf, buf[1]+2);
			egalax_usb->sysfs_query_wait = false;
			wake_up_interruptible(
				&egalax_usb->sysfs_query_queue);
			break;
		}

		/* If someone reading now! put the data into the buffer! */
		if (p_char_dev->open_cnts > 0) {
			loop = 3;
			do {
				ret = wait_event_timeout(p_char_dev->fifo_inq,
				kfifo_avail(
					&p_char_dev->data_kfifo) >= len, HZ);
			} while (ret <= 0 && --loop);

			/* fifo size is ready */
			if (ret > 0) {
				ret = kfifo_in_locked(&p_char_dev->data_kfifo,
					buf, len, &p_char_dev->fifo_lock);
				wake_up_interruptible(&p_char_dev->fifo_inq);
			} else {
				EGALAX_DBG(DBG_CDEV,
				" [Warning] fifo size is overflow.\n");
			}
		}
		break;
	case REPORTID_MTOUCH:
		ProcessParallelReport(buf, egalax_usb);
		break;
	default:
		break;
	}
}

static void egalax_usb_irq(struct urb *urb)
{
	struct _egalax_usb *egalax_usb = urb->context;
	int retval;

	EGALAX_DBG(DBG_INT, " %s\n", __func__);

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ETIME:
		/* this urb is timing out */
		EGALAX_DBG(DBG_INT, " urb timed out\n");
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		EGALAX_DBG(DBG_INT,
		" urb shutting down with status: %d\n", urb->status);
		return;
	default:
		EGALAX_DBG(DBG_INT,
		" urb nonzero urb status received: %d\n", urb->status);
		goto exit;
	}

	egalax_usb_measure(egalax_usb, egalax_usb->data, urb->actual_length);

exit:
	usb_mark_last_busy(egalax_usb->udev);
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		EGALAX_DBG(DBG_INT,
	    " usb_submit_urb failed with result: %d\n", retval);
}

static int exc80_input_open(struct input_dev *dev)
{
	return 0;
}

static void exc80_input_close(struct input_dev *dev)
{
}

static int exc80_input_disable(struct input_dev *dev)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x67, 0x02, 0x00};

	if (sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN, send_cmd_buf+2, 3, HZ))
		return 0;

	return -EIO;
}

static int exc80_input_enable(struct input_dev *dev)
{
	unsigned char send_cmd_buf[MAX_USB_LEN] = {
		0x03, 0x04, 0x36, 0x67, 0x02, 0x01};

	if (sys_sendcmd_wait(send_cmd_buf, MAX_USB_LEN, send_cmd_buf+2, 3, HZ))
		return 0;

	return -EIO;
}

static struct input_dev *allocate_Input_Dev(struct _egalax_usb *egalax_usb,
					    struct usb_interface *intf)
{
	int ret;
	struct input_dev *p_input_dev = NULL;

	p_input_dev = input_allocate_device();
	if (p_input_dev == NULL) {
		EGALAX_DBG(DBG_MODULE, " Failed to allocate input device\n");
		return NULL;/* -ENOMEM; */
	}

	p_input_dev->name = "eGalax_Touch_Screen";
	p_input_dev->phys = egalax_usb->phys;
	usb_to_input_id(interface_to_usbdev(intf), &p_input_dev->id);
	p_input_dev->dev.parent = &intf->dev;
	p_input_dev->open = exc80_input_open;
	p_input_dev->close = exc80_input_close;
	p_input_dev->enable = exc80_input_enable;
	p_input_dev->disable = exc80_input_disable;
	p_input_dev->enabled = true;
	input_set_drvdata(p_input_dev, egalax_usb);

	set_bit(EV_ABS, p_input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, p_input_dev->propbit);
	input_mt_init_slots(p_input_dev, MAX_SUPPORT_POINT, 0);
	input_set_abs_params(p_input_dev, ABS_MT_POSITION_X, 0,
		MAX_RESOLUTION, 0, 0);
	input_set_abs_params(p_input_dev, ABS_MT_POSITION_Y, 0,
		MAX_RESOLUTION, 0, 0);
	input_set_abs_params(p_input_dev, ABS_MT_PRESSURE, 0,
		MAX_Z_RESOLUTION, 0, 0);
	input_set_abs_params(p_input_dev, ABS_MT_TOOL_TYPE, 0,
		MT_TOOL_MAX, 0, 0);

	input_set_events_per_packet(p_input_dev, MAX_EVENTS);

	ret = input_register_device(p_input_dev);
	if (ret) {
		EGALAX_DBG(DBG_MODULE,
		" Unable to register input device, err: %d\n", ret);
		input_free_device(p_input_dev);
		p_input_dev = NULL;
	}

	return p_input_dev;
}

static void egalax_usb_free_buffers(struct usb_device *udev,
				    struct _egalax_usb *egalax_usb)
{
	usb_free_coherent(udev, MAX_USB_LEN, egalax_usb->data,
		egalax_usb->data_dma);
	kfree(egalax_usb->buffer);
}

static const struct file_operations egalax_cdev_fops = {
	.owner  = THIS_MODULE,
	.read   = egalax_cdev_read,
	.write  = egalax_cdev_write,
	.open   = egalax_cdev_open,
	.release = egalax_cdev_release,
	.poll   = egalax_cdev_poll,
};

static struct miscdevice egalax_misc_dev = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = "touch",
	.fops   = &egalax_cdev_fops,
};

static const struct file_operations egalax_proc_fops = {
	.owner      = THIS_MODULE,
	.open       = egalax_proc_open,
	.read       = seq_read,
	.write      = egalax_proc_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static struct egalax_char_dev *setup_chardev(void)
{
	struct egalax_char_dev *p_char_dev;

	p_char_dev = kzalloc(1*sizeof(struct egalax_char_dev), GFP_KERNEL);
	if (p_char_dev == NULL)
		goto fail_cdev;

	spin_lock_init(&p_char_dev->fifo_lock);
	p_char_dev->p_fifo_buf = kzalloc(sizeof(unsigned char)*FIFO_SIZE,
				GFP_KERNEL);
	if (p_char_dev->p_fifo_buf == NULL)
		goto fail_fifobuf;

	kfifo_init(&p_char_dev->data_kfifo, p_char_dev->p_fifo_buf, FIFO_SIZE);
	if (!kfifo_initialized(&p_char_dev->data_kfifo))
		goto fail_kfifo;

	p_char_dev->open_cnts = 0;
	sema_init(&p_char_dev->sem, 1);
	init_waitqueue_head(&p_char_dev->fifo_inq);

	return p_char_dev;

fail_kfifo:
	kfree(p_char_dev->p_fifo_buf);
fail_fifobuf:
	kfree(p_char_dev);
fail_cdev:
	return NULL;
}

static void egalax_usb_add_ncb(struct usb_device *udev)
{
	int result;
	struct usb_interface *intf;
	struct _egalax_usb *egalax_usb;
	struct usb_endpoint_descriptor *endpoint;

	EGALAX_DBG(DBG_MODULE,
		" USB device probe (VID=%04X PID=%04X)\n",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct));

	if (udev->descriptor.idVendor != EETI_USB_DEVICE_ID ||
	    (udev->descriptor.idProduct != EETI_USB_PRODUCT_ID &&
	     udev->descriptor.idProduct != EETI_USB_BL_PRODUCT_ID)) {
		EGALAX_DBG(DBG_MODULE, " No support this device.\n");
		return;
	}

	intf = usb_ifnum_to_if(udev, 0);
	if (intf == NULL)
		return;

	endpoint = &intf->cur_altsetting->endpoint[0].desc;
	egalax_usb = kzalloc(sizeof(struct _egalax_usb), GFP_KERNEL);
	if (egalax_usb == NULL)
		goto out_free;

	p_egalax_usb_dev = egalax_usb;

	egalax_usb->data = usb_alloc_coherent(udev,
			MAX_USB_LEN, GFP_KERNEL,
			&egalax_usb->data_dma);

	if (egalax_usb->data == NULL) {
		EGALAX_DBG(DBG_MODULE, " usb_buffer_alloc failed!\n");
		goto out_free;
	}

	egalax_usb->buffer = kzalloc(MAX_USB_LEN, GFP_KERNEL);
	if (egalax_usb->buffer == NULL) {
		EGALAX_DBG(DBG_MODULE, " Can't alloc USB buffer!\n");
		goto out_free_buffers;
	}

	egalax_usb->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (egalax_usb->irq == NULL) {
		EGALAX_DBG(DBG_MODULE, " usb_alloc_urb failed!\n");
		goto out_free_buffers;
	}

	egalax_usb->udev = udev;
	egalax_usb->interface = intf;

	usb_make_path(udev, egalax_usb->phys, sizeof(egalax_usb->phys));
	strlcat(egalax_usb->phys, "/input0", sizeof(egalax_usb->phys));

	if (input_dev == NULL &&
	    udev->descriptor.idProduct == EETI_USB_PRODUCT_ID) {
		input_dev = allocate_Input_Dev(egalax_usb, intf);
		if (input_dev == NULL) {
			EGALAX_DBG(DBG_MODULE, " allocate_Input_Dev failed\n");
			goto out_free_buffers;
		}
		EGALAX_DBG(DBG_MODULE, " Register input device done\n");
	} else
		EGALAX_DBG(DBG_MODULE, " Input device already created!\n");

	usb_fill_int_urb(egalax_usb->irq, egalax_usb->udev,
			usb_rcvintpipe(egalax_usb->udev,
			endpoint->bEndpointAddress),
			egalax_usb->data, MAX_USB_LEN, egalax_usb_irq,
			egalax_usb, endpoint->bInterval);

	egalax_usb->irq->dev = egalax_usb->udev;
	egalax_usb->irq->transfer_dma = egalax_usb->data_dma;
	egalax_usb->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	usb_set_intfdata(intf, egalax_usb);

	/* start USB function */
	if (usb_submit_urb(egalax_usb->irq, GFP_KERNEL)) {
		EGALAX_DBG(DBG_MODULE, " usb_submit_urb failed\n");
		goto out_unregister_input;
	}

	result = misc_register(&egalax_misc_dev);
	if (result) {
		EGALAX_DBG(DBG_MODULE,
				" misc device register failed\n");
		goto out_unregister_input;
	}
	misc_registered = true;

	result = sysfs_create_group(&egalax_misc_dev.this_device->kobj,
		&egalax_attr_group);
	if (result) {
		EGALAX_DBG(DBG_MODULE,
		" Failed to create sysfs attributes:%d\n", result);
		goto out_misc_deregister;
	}
	sysfs_created = true;

	/* allocate the character device */
	p_char_dev = setup_chardev();
	if (p_char_dev == NULL) {
		result = -ENOMEM;
		goto out_sysfs_remove_group;
	}

	init_waitqueue_head(&p_egalax_usb_dev->sysfs_query_queue);
	p_egalax_usb_dev->sysfs_query_wait = false;
	p_egalax_usb_dev->sysfs_hook_cmd[0] = 0xFF;
	p_egalax_usb_dev->sysfs_hook_cmd[1] = 0xFF;
	p_egalax_usb_dev->sysfs_hook_cmd[2] = 0xFF;

	if (udev->descriptor.idProduct != EETI_USB_BL_PRODUCT_ID)
		run_fw_init(egalax_usb);

	EGALAX_DBG(DBG_MODULE, " USB device probe done\n");

	return;

out_sysfs_remove_group:
	sysfs_remove_group(&egalax_misc_dev.this_device->kobj,
			   &egalax_attr_group);
out_misc_deregister:
	misc_deregister(&egalax_misc_dev);
out_unregister_input:
	input_unregister_device(input_dev);
	input_dev = NULL;
out_free_buffers:
	p_egalax_usb_dev = NULL;
	egalax_usb_free_buffers(udev, egalax_usb);
out_free:
	kfree(egalax_usb);
}

static void egalax_usb_rm_ncb(struct usb_device *udev)
{
	if (udev->descriptor.idVendor != EETI_USB_DEVICE_ID ||
	    (udev->descriptor.idProduct != EETI_USB_PRODUCT_ID &&
	     udev->descriptor.idProduct != EETI_USB_BL_PRODUCT_ID))
		return;

	if (p_egalax_usb_dev == NULL || (p_egalax_usb_dev->udev != udev))
		return;

	EGALAX_DBG(DBG_MODULE, " USB device disconnect!\n");

	if (sysfs_created)
		sysfs_remove_group(&egalax_misc_dev.this_device->kobj,
				&egalax_attr_group);
	sysfs_created = false;

	if (misc_registered)
		misc_deregister(&egalax_misc_dev);
	misc_registered = false;

	if (p_char_dev != NULL) {
		kfree(p_char_dev->p_fifo_buf);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	usb_kill_urb(p_egalax_usb_dev->irq);
	usb_free_urb(p_egalax_usb_dev->irq);
	egalax_usb_free_buffers(udev, p_egalax_usb_dev);
	kfree(p_egalax_usb_dev);
	p_egalax_usb_dev = NULL;
	input_dev = NULL;
}

int egalax_usb_ncb(struct notifier_block *nb, unsigned long usb_event,
		   void *udev)
{
	int result = NOTIFY_OK;

	switch (usb_event) {
	case USB_DEVICE_ADD:
		egalax_usb_add_ncb(udev);
		break;
	case USB_DEVICE_REMOVE:
		egalax_usb_rm_ncb(udev);
		break;
	case USB_BUS_ADD:
	case USB_BUS_REMOVE:
		break;
	default:
		result = NOTIFY_BAD;
	}
	return result;
}

static struct notifier_block egalax_usb_notifier = {
	.notifier_call = egalax_usb_ncb,
	.priority = INT_MAX	/* Need to be called first of all */
};

static int request_dt(struct device *dev)
{
	int result, val;
	struct device_node *devnode;

	g_reset_gpio = 0;
	g_enable_high = false;
	g_flip_x = false;
	g_flip_y = false;
	g_regulator_hv = NULL;
	g_regulator_5v0 = NULL;
	g_regulator_3v3 = NULL;
	g_regulator_1v8 = NULL;

	devnode = dev->of_node;
	if (devnode) {
		/* Touch orientation */
		result = of_property_read_u32(devnode, "flip-x", &val);
		if (result < 0)
			val = 0;
		g_flip_x = val != 0 ? true : false;
		result = of_property_read_u32(devnode, "flip-y", &val);
		if (result < 0)
			val = 0;
		g_flip_y = val != 0 ? true : false;

		if (of_property_read_bool(devnode, "enable-active-high"))
			g_enable_high = true;

		g_reset_gpio = of_get_named_gpio(devnode,
							"reset-gpio", 0);
		/* regulator */
		g_regulator_hv = devm_regulator_get(
						dev, "vdd-ts-hv");
		if (IS_ERR(g_regulator_hv)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-12v regulator_get failed: %ld\n",
				PTR_ERR(g_regulator_hv));
			return -EINVAL;
		}
		g_regulator_5v0 = devm_regulator_get(
						dev, "vdd-ts-5v0");
		if (IS_ERR(g_regulator_5v0)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-5v regulator_get failed: %ld\n",
				PTR_ERR(g_regulator_5v0));
			return -EINVAL;
		}
		g_regulator_3v3 = devm_regulator_get(
						dev, "vdd-ts-3v3");
		if (IS_ERR(g_regulator_3v3)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 3v3 regulator_get failed: %ld\n",
				PTR_ERR(g_regulator_3v3));
			return -EINVAL;
		}
		g_regulator_1v8 = devm_regulator_get(
						dev, "vdd-ts-1v8");
		if (IS_ERR(g_regulator_1v8)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 18v regulator_get failed: %ld\n",
				PTR_ERR(g_regulator_1v8));
			return -EINVAL;
		}
	}

	if (!gpio_is_valid(g_reset_gpio)) {
		EGALAX_DBG(DBG_MODULE, " gpio[%d] is not valid\n",
			   g_reset_gpio);
		return -EINVAL;
	}

	result = gpio_request(g_reset_gpio, "rest-gpio");
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " gpio_request[%d] failed: %d\n",
			   g_reset_gpio, result);
		return -EINVAL;
	}

	return 0;
}

static void egalax_power_on(void)
{
	int error = 0;

	if (g_enable_high)
		gpio_direction_output(g_reset_gpio, 1);
	else
		gpio_direction_output(g_reset_gpio, 0);

	error = regulator_enable(g_regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(g_regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(g_regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(g_regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	usleep_range(1000, 5000);
	if (g_enable_high)
		gpio_direction_output(g_reset_gpio, 0);
	else
		gpio_direction_output(g_reset_gpio, 1);
	EGALAX_DBG(DBG_MODULE, " Device power on!\n");

}

static void egalax_power_off(void)
{
	int error = 0;

	if (g_enable_high)
		gpio_direction_output(g_reset_gpio, 0);
	else
		gpio_direction_output(g_reset_gpio, 1);

	error = regulator_disable(g_regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(g_regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(g_regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(g_regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	usleep_range(1000, 5000);
	if (g_enable_high)
		gpio_direction_output(g_reset_gpio, 1);
	else
		gpio_direction_output(g_reset_gpio, 0);

	EGALAX_DBG(DBG_MODULE, " Device power on!\n");
}

static int egalax_platform_probe(struct platform_device *pdev)
{
	int i, result;

	p_egalax_usb_dev = NULL;
	p_char_dev = NULL;
	input_dev = NULL;
	total_pts_cnt = 0;
	recv_pts_cnt = 0;

	for (i = 0; i < MAX_SUPPORT_POINT; i++) {
		p_contact_buf[i].status = -1;
		p_contact_buf[i].id = 0;
		p_contact_buf[i].x = p_contact_buf[i].y =
				     p_contact_buf[i].z = 0;
	}

	if (request_dt(&(pdev->dev)) != 0)
		return -EINVAL;

	egalax_power_on();

	p_dbg_proc_file = proc_create(PROC_FS_NAME, 0660, NULL,
			&egalax_proc_fops);
	if (p_dbg_proc_file == NULL) {
		remove_proc_entry(PROC_FS_NAME, NULL);
		EGALAX_DBG(DBG_MODULE, " Could not initialize /proc/%s\n",
		PROC_FS_NAME);
		result = -EINVAL;
		goto out_fail;
	}

	usb_register_notify(&egalax_usb_notifier);
	return 0;

out_fail:
	return result;
}

static int egalax_platform_remove(struct platform_device *pdev)
{
	if (p_dbg_proc_file != NULL)
		remove_proc_entry(PROC_FS_NAME, NULL);

	if (sysfs_created)
		sysfs_remove_group(&egalax_misc_dev.this_device->kobj,
				&egalax_attr_group);
	sysfs_created = false;

	if (misc_registered)
		misc_deregister(&egalax_misc_dev);
	misc_registered = false;

	if (p_char_dev != NULL) {
		kfree(p_char_dev->p_fifo_buf);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	egalax_power_off();

	gpio_free(g_reset_gpio);

	usb_unregister_notify(&egalax_usb_notifier);
	return 0;
}

static void egalax_platform_shutdown(struct platform_device *pdev)
{
	egalax_power_off();
}

static const struct of_device_id egalax_usb_idtable[] = {
	{ .compatible = "eeti,exc80_ts_usb" },
	{ },
};
MODULE_DEVICE_TABLE(of, egalax_usb_idtable);

static struct platform_driver egalax_platform_driver = {
	.driver = {
		.name = "exc80_ts_usb",
		.owner = THIS_MODULE,
		.of_match_table = egalax_usb_idtable,
	},
	.probe = egalax_platform_probe,
	.remove = egalax_platform_remove,
	.shutdown = egalax_platform_shutdown,
};

static void egalax_usb_ts_exit(void)
{
	EGALAX_DBG(DBG_MODULE, " Driver exit!\n");
	platform_driver_unregister(&egalax_platform_driver);
}

static int egalax_usb_ts_init(void)
{
	EGALAX_DBG(DBG_MODULE, " Driver init!\n");
	return platform_driver_register(&egalax_platform_driver);
}

module_init(egalax_usb_ts_init);
module_exit(egalax_usb_ts_exit);

MODULE_AUTHOR("EETI <touch_fae@eeti.com>");
MODULE_DESCRIPTION("egalax touch screen usb driver");
MODULE_LICENSE("GPL");
