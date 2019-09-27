/*
 *
 * Touch Screen I2C Driver for EETI Controller
 *
 * Copyright (C) 2000-2017  eGalax_eMPIA Technology Inc. All rights reserved.
 * Copyright (c) 2017-2018 NVIDIA CORPORATION. All rights reserved.
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

#define RELEASE_DATE "2018/05/18"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

/* Global define to enable function */
/* #define _SWITCH_XY */
/* #define _CONVERT_Y */

#define MAX_EVENTS		600
#define MAX_I2C_LEN		64U
#define FIFO_SIZE		8192
#define MAX_SUPPORT_POINT	16
#define REPORTID_VENDOR		0x03
#define REPORTID_MTOUCH		0x06
#define MAX_RESOLUTION		4095
#define MAX_Z_RESOLUTION	1023

/* running mode */
#define MODE_STOP	0
#define MODE_WORKING	1
#define MODE_IDLE	2
#define MODE_SUSPEND	3

struct tag_mt_contacts {
	unsigned char id;
	signed char status;
	unsigned short x;
	unsigned short y;
	unsigned short z;
};

struct _egalax_i2c {
	struct workqueue_struct *ktouch_wq;
	struct work_struct work_irq;
	struct delayed_work delay_work_ioctl;
	struct mutex mutex_wq;
	struct i2c_client *client;
	unsigned char work_state;
	bool is_disabled;
	unsigned char skip_packet;
	unsigned int ioctl_cmd;
	int interrupt_gpio;
	int reset_gpio;
	bool enable_high;
	wait_queue_head_t sysfs_query_queue;
	bool sysfs_query_wait;
	unsigned char sysfs_hook_cmd[3];
	unsigned char sysfs_cmd_result[MAX_I2C_LEN];
	struct regulator        *regulator_hv;
	struct regulator        *regulator_5v0;
	struct regulator        *regulator_3v3;
	struct regulator        *regulator_1v8;
	bool                    flip_x;
	bool                    flip_y;
};

struct egalax_char_dev {
	int open_cnts;
	struct kfifo data_kfifo;
	unsigned char *p_fifo_buf;
	spinlock_t fifo_lock;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};

static struct _egalax_i2c *p_egalax_i2c_dev;
static struct egalax_char_dev *p_char_dev;
static atomic_t egalax_char_available = ATOMIC_INIT(1);
static atomic_t wait_command_ack = ATOMIC_INIT(0);
static struct input_dev *input_dev;
static struct tag_mt_contacts p_contact_buf[MAX_SUPPORT_POINT];
static unsigned char input_report_buf[MAX_I2C_LEN+2];
static char fifo_read_buf[MAX_I2C_LEN];
static int total_pts_cnt, recv_pts_cnt;

#define DBG_MODULE	0x00000001U
#define DBG_CDEV	0x00000002U
#define DBG_PROC	0x00000004U
#define DBG_POINT	0x00000008U
#define DBG_INT		0x00000010U
#define DBG_I2C		0x00000020U
#define DBG_SUSP	0x00000040U
#define DBG_INPUT	0x00000080U
#define DBG_CONST	0x00000100U
#define DBG_IDLE	0x00000200U
#define DBG_WAKEUP	0x00000400U
#define DBG_BUTTON	0x00000800U
static unsigned int dbg_level = DBG_MODULE|DBG_SUSP;

#define PROC_FS_NAME	"egalax_dbg"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *dbg_proc_file;

#define EGALAX_DBG(level, fmt, args...) \
do { if ((level & dbg_level) > 0U) { \
pr_debug("egalax_i2c: " fmt, ## args); } \
} while (false)

static int egalax_i2c_read(unsigned char *p_buf, unsigned short len)
{
	struct i2c_msg xfer;

	if (p_buf == NULL)
		return -1;

	/* Read device data */
	xfer.addr = p_egalax_i2c_dev->client->addr;
	xfer.flags = I2C_M_RD;
	xfer.len = len;
	xfer.buf = p_buf;

	if (i2c_transfer(p_egalax_i2c_dev->client->adapter, &xfer, 1) != 1) {
		EGALAX_DBG(DBG_I2C, " %s: i2c transfer fail\n", __func__);
		return -EIO;
	}

	EGALAX_DBG(DBG_I2C, " %s: i2c transfer success\n", __func__);

	return 0;
}

static int egalax_i2c_write(unsigned short reg, unsigned char *p_buf,
			    unsigned short len)
{
	unsigned char cmdbuf[4+len];
	struct i2c_msg xfer;

	if (p_buf == NULL)
		return -1;

	cmdbuf[0] = reg & 0x00FFU;
	cmdbuf[1] = (reg >> 8) & 0x00FFU;
	cmdbuf[2] = (len+2) & 0x00FFU;
	cmdbuf[3] = ((len+2) >> 8) & 0x00FFU;
	memcpy(cmdbuf+4, p_buf, len);

	/* Write data to device */
	xfer.addr = p_egalax_i2c_dev->client->addr;
	xfer.flags = 0;
	xfer.len = sizeof(cmdbuf);
	xfer.buf = cmdbuf;

	if (i2c_transfer(p_egalax_i2c_dev->client->adapter, &xfer, 1) != 1) {
		EGALAX_DBG(DBG_I2C, " %s: i2c transfer fail\n", __func__);
		return -EIO;
	}

	EGALAX_DBG(DBG_I2C, " %s: i2c transfer success\n", __func__);

	return 0;
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

	if (count > MAX_I2C_LEN)
		count = MAX_I2C_LEN;

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

	if (down_interruptible(&cdev->sem))
		return -ERESTARTSYS;

	if (count > MAX_I2C_LEN)
		count = MAX_I2C_LEN;

	tmp = kzalloc(MAX_I2C_LEN, GFP_KERNEL);
	if (tmp == NULL) {
		up(&cdev->sem);
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		up(&cdev->sem);
		kfree(tmp);
		return -EFAULT;
	}

	ret = egalax_i2c_write(0x0067, tmp, MAX_I2C_LEN);

	up(&cdev->sem);
	EGALAX_DBG(DBG_CDEV, " I2C writing %zu bytes.\n", count);
	kfree(tmp);

	return (ret == 0 ? count : -1);
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

	if ((FIFO_SIZE - fifo_len) > MAX_I2C_LEN)
		mask |= POLLOUT | POLLWRNORM;

	up(&cdev->sem);
	return mask;
}

static int egalax_proc_show(struct seq_file *seqfilp, void *v)
{
	seq_printf(seqfilp,
	"EETI I2C for All Points.\nDebug Level: 0x%08X\nRelease Date: %s\n",
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
			     int n_time_out)
{
	int i;
	bool b_ret = true;

	memset(p_egalax_i2c_dev->sysfs_cmd_result, 0,
	       sizeof(p_egalax_i2c_dev->sysfs_cmd_result));

	for (i = 0; i < 3; i++) {
		if (i < n_hook_cmd_len)
			p_egalax_i2c_dev->sysfs_hook_cmd[i] = by_hook_cmd[i];
		else
			p_egalax_i2c_dev->sysfs_hook_cmd[i] = 0xFF;
	}
	p_egalax_i2c_dev->sysfs_query_wait = true;

	if (egalax_i2c_write(0x0067, by_send_cmd, n_send_cmd_len) != 0) {
		b_ret = false;
	} else {
		wait_event_interruptible_timeout(
				p_egalax_i2c_dev->sysfs_query_queue,
				!p_egalax_i2c_dev->sysfs_query_wait,
				n_time_out);

		if (p_egalax_i2c_dev->sysfs_query_wait)
			b_ret = false;
		else
			b_ret = true;
	}
	p_egalax_i2c_dev->sysfs_query_wait = false;
	return b_ret;
}

#define OP_MODE_GET		0x00
#define OP_MODE_SET		0x01
static ssize_t sys_show_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x01, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: %s\n",
			RELEASE_DATE, p_egalax_i2c_dev->sysfs_cmd_result+6);
	else
		return snprintf(buf, PAGE_SIZE, "Driver: %s  FW: Invalid\n",
				RELEASE_DATE);

}

static ssize_t sys_show_touchevent(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x02, OP_MODE_GET};
	bool b_ret = true;
	int code = 0;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret) {
		code = p_egalax_i2c_dev->sysfs_cmd_result[6];
		code += (p_egalax_i2c_dev->sysfs_cmd_result[7]<<8);
		code += (p_egalax_i2c_dev->sysfs_cmd_result[8]<<16);
		code += (p_egalax_i2c_dev->sysfs_cmd_result[9]<<24);
		return snprintf(buf, PAGE_SIZE, "0x%08X\n", code);
	} else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

static ssize_t sys_show_reportmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x04, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			       p_egalax_i2c_dev->sysfs_cmd_result[6]);
	else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_REPORTMODE_MAX  0x06
static ssize_t sys_store_reportmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x05, 0x36, 0x91, 0x04, OP_MODE_SET};
	bool b_ret = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0]-'0';
	if (mode > NV_REPORTMODE_MAX || mode < 0)
		return -EINVAL;

	send_cmd_buf[6] = mode;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return count;
	else
		return -EIO;
}

static ssize_t sys_show_bypassmode(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x04, 0x36, 0x91, 0x05, OP_MODE_GET};
	bool b_ret = true;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return snprintf(buf, PAGE_SIZE, "%02X\n",
			       p_egalax_i2c_dev->sysfs_cmd_result[6]);
	else
		return snprintf(buf, PAGE_SIZE, "Invalid\n");
}

#define NV_BYPASSMODE_MAX  0x02
static ssize_t sys_store_bypassmode(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x05, 0x36, 0x91, 0x05, OP_MODE_SET};
	bool b_ret = true;
	char mode;

	if (count != 2)
		return -EINVAL;

	mode = buf[0]-'0';
	if (mode > NV_BYPASSMODE_MAX || mode < 0)
		return -EINVAL;

	send_cmd_buf[6] = mode;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret)
		return count;
	else
		return -EIO;
}

static ssize_t sys_show_calibration(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x02, 0x3F, 0x4E};
	bool b_ret = true;
	unsigned int status = 0;

	b_ret = sys_sendcmd_wait(send_cmd_buf, MAX_I2C_LEN,
				 send_cmd_buf+2, 3, HZ);
	if (b_ret) {
		status = p_egalax_i2c_dev->sysfs_cmd_result[4];
		status += p_egalax_i2c_dev->sysfs_cmd_result[5]<<8;
		if (status & 0x00000100)
			return sprintf(buf, "0xff\n");
		else
			return sprintf(buf, "0x00\n");
	} else {
		/* Don't show the status if communication fail */
		return sprintf(buf, "0x00\n");
	}
}

static DEVICE_ATTR(version, 0640, sys_show_version, NULL);
static DEVICE_ATTR(touch_event, 0640, sys_show_touchevent, NULL);
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

#define STYLUS_MASK	0x10
#define MAX_POINT_PER_PACKET	5U
#define POINT_STRUCT_SIZE	10U
static void process_report(unsigned char *buf,
			  struct _egalax_i2c *p_egalax_i2c)
{
	unsigned char i, index = 0, cnt_down = 0, cnt_up = 0, shift = 0;
	unsigned char status = 0;
	unsigned short contact_id = 0, x = 0, y = 0, z = 0;

	if (total_pts_cnt <= 0) {
		if ((buf[1] == 0) || (buf[1] > MAX_SUPPORT_POINT)) {
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
			   " Get Point[%d] Update: Status=%d X=%d Y=%d\n",
			   contact_id, status, x, y);

	#ifdef _SWITCH_XY
		short tmp = x;

		x = y;
		y = tmp;
	#endif

		if (p_egalax_i2c_dev->flip_x)
			x = MAX_RESOLUTION-x;

		if (p_egalax_i2c_dev->flip_y)
			y = MAX_RESOLUTION-y;

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

				if ((p_contact_buf[i].status & 0x01) != 0)
					cnt_down++;
				else
					cnt_up++;
			}

			input_sync(input_dev);
			EGALAX_DBG(DBG_POINT,
			" Input sync point data done! (Down:%d Up:%d)\n",
				cnt_down, cnt_up);

			total_pts_cnt = 0;
			recv_pts_cnt = 0;
			return;
		}
	}
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
	unsigned char send_cmd_buf[MAX_I2C_LEN] = {
			0x03, 0x03, 0x36, 0x3F, 0x02, 0x00};

	p_egalax_i2c_dev->is_disabled = true;
	if (egalax_i2c_write(0x0067, send_cmd_buf, MAX_I2C_LEN) < 0)
		return -EIO;

	return 0;
}

static int exc80_input_enable(struct input_dev *dev)
{
	disable_irq(p_egalax_i2c_dev->client->irq);

	/* pull the int pin to low level */
	gpio_direction_output(p_egalax_i2c_dev->interrupt_gpio, 0);
	udelay(200);
	/* return to high level */
	gpio_direction_input(p_egalax_i2c_dev->interrupt_gpio);

	enable_irq(p_egalax_i2c_dev->client->irq);
	p_egalax_i2c_dev->is_disabled = false;

	return 0;
}

static struct input_dev *allocate_input_dev(void)
{
	int ret;
	struct input_dev *p_input_dev = NULL;

	p_input_dev = input_allocate_device();
	if (p_input_dev == NULL) {
		EGALAX_DBG(DBG_MODULE, " Failed to allocate input device\n");
		return NULL;
	}

	p_input_dev->name = "eGalax_Touch_Screen";
	p_input_dev->phys = "I2C";
	p_input_dev->id.bustype = BUS_I2C;
	p_input_dev->id.vendor = 0x0EEF;
	p_input_dev->id.product = 0x0020;
	p_input_dev->id.version = 0x0001;
	/* Input control */
	p_input_dev->open = exc80_input_open;
	p_input_dev->close = exc80_input_close;
	p_input_dev->enable = exc80_input_enable;
	p_input_dev->disable = exc80_input_disable;
	p_input_dev->enabled = true;
	p_input_dev->dev.parent = &p_egalax_i2c_dev->client->dev;
	input_set_drvdata(p_input_dev, p_egalax_i2c_dev);

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
		EGALAX_DBG(DBG_MODULE, " Unable to register input device.\n");
		input_free_device(p_input_dev);
		p_input_dev = NULL;
	}

	return p_input_dev;
}

static int egalax_i2c_measure(struct _egalax_i2c *egalax_i2c)
{
	int ret = 0, frame_len = 0, loop = 3, i;

	EGALAX_DBG(DBG_INT, " egalax_i2c_measure\n");

	if (egalax_i2c_read(input_report_buf, MAX_I2C_LEN+2) < 0) {
		EGALAX_DBG(DBG_I2C, " I2C read input report fail!\n");
		return -1;
	}

	if ((dbg_level & DBG_I2C) != 0U) {
		char dbgmsg[(MAX_I2C_LEN+2)*4];

		for (i = 0; i < (MAX_I2C_LEN+2); i++)
			snprintf(dbgmsg+(i*4), 4, "[%02X]",
				input_report_buf[i]);

		EGALAX_DBG(DBG_I2C, " Buf=%s\n", dbgmsg);
	}

	frame_len = input_report_buf[0] + (input_report_buf[1]<<8);
	EGALAX_DBG(DBG_I2C, " I2C read data with Len=%d\n", frame_len);

	if (frame_len == 0) {
		EGALAX_DBG(DBG_MODULE, " Device reset\n");
		return -1;
	}

	switch (input_report_buf[2]) {
	case REPORTID_MTOUCH:
		if (!egalax_i2c->skip_packet &&
		    egalax_i2c->work_state == MODE_WORKING) {
			process_report(input_report_buf+2, egalax_i2c);
		}
		ret = 0;
		break;
	case REPORTID_VENDOR:
		atomic_set(&wait_command_ack, 1);
		EGALAX_DBG(DBG_I2C, " I2C get vendor command packet\n");

		if (egalax_i2c->sysfs_query_wait &&
			egalax_i2c->sysfs_hook_cmd[0] == input_report_buf[2+2]
			 && ((egalax_i2c->sysfs_hook_cmd[1] == 0xFF) ||
			egalax_i2c->sysfs_hook_cmd[1] == input_report_buf[2+3])
			&& ((egalax_i2c->sysfs_hook_cmd[2] == 0xFF) ||
			egalax_i2c->sysfs_hook_cmd[2] == input_report_buf[2+4])
			) {
			memcpy(egalax_i2c->sysfs_cmd_result,
			       input_report_buf+2, input_report_buf[2+1]+2);
			egalax_i2c->sysfs_query_wait = false;
			wake_up_interruptible(&egalax_i2c->sysfs_query_queue);
			break;
		}

		/* If someone reading now! put the data into the buffer! */
		if (p_char_dev->open_cnts > 0) {
			loop = 3;
			do {
				ret = wait_event_timeout(p_char_dev->fifo_inq,
					(kfifo_avail(&p_char_dev->data_kfifo) >=
					MAX_I2C_LEN), HZ);
			} while (ret <= 0 && --loop);

			/* fifo size is ready */
			if (ret > 0) {
				ret = kfifo_in_locked(&p_char_dev->data_kfifo,
					(input_report_buf+2), MAX_I2C_LEN,
					&p_char_dev->fifo_lock);
				wake_up_interruptible(&p_char_dev->fifo_inq);
			} else {
				EGALAX_DBG(DBG_CDEV,
					" [Warning] fifo size is overflow.\n");
			}
		}

		break;
	default:
		EGALAX_DBG(DBG_I2C, " I2C read error data with hedaer=%d\n",
			   input_report_buf[2]);
		ret = -1;
		break;
	}

	return ret;
}

static void egalax_i2c_wq_irq(struct work_struct *work)
{
	struct _egalax_i2c *egalax_i2c =
			container_of(work, struct _egalax_i2c, work_irq);
	struct i2c_client *client = egalax_i2c->client;

	EGALAX_DBG(DBG_INT, " egalax_i2c_wq run\n");

	/* continue recv data */
	while (gpio_get_value(egalax_i2c->interrupt_gpio) == 0) {
		egalax_i2c_measure(egalax_i2c);
		schedule();
	}

	if (egalax_i2c->skip_packet > 0U)
		egalax_i2c->skip_packet = 0U;

	enable_irq(client->irq);

	EGALAX_DBG(DBG_INT, " egalax_i2c_wq leave\n");
}

static irqreturn_t egalax_i2c_interrupt(int irq, void *dev_id)
{
	struct _egalax_i2c *egalax_i2c = (struct _egalax_i2c *)dev_id;

	EGALAX_DBG(DBG_INT, " INT with irq:%d\n", irq);

	disable_irq_nosync(irq);

	queue_work(egalax_i2c->ktouch_wq, &egalax_i2c->work_irq);

	return IRQ_HANDLED;
}

static void egalax_i2c_senduppoint(void)
{
	int i = 0;

	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);

	for (i = 0; i < MAX_SUPPORT_POINT; i++) {
		input_mt_slot(input_dev, p_contact_buf[i].id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
		p_contact_buf[i].status = 0;
	}

	input_sync(input_dev);
	EGALAX_DBG(DBG_POINT, " Sent up point data done!\n");
}

static int egalax_power_off(void)
{
	int error;

	if (p_egalax_i2c_dev->enable_high)
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 1);
	else
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 0);
	error = regulator_disable(p_egalax_i2c_dev->regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(p_egalax_i2c_dev->regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(p_egalax_i2c_dev->regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_disable(p_egalax_i2c_dev->regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);

	return 0;
}

static int egalax_power_on(void)
{
	int error;

	error = regulator_enable(p_egalax_i2c_dev->regulator_hv);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_5v0);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_1v8);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	error = regulator_enable(p_egalax_i2c_dev->regulator_3v3);
	if (error < 0)
		EGALAX_DBG(DBG_MODULE, " regulator enable failed: %d\n",
			error);
	usleep_range(1000, 5000);
	if (p_egalax_i2c_dev->enable_high)
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 0);
	else
		gpio_direction_output(p_egalax_i2c_dev->reset_gpio, 1);

	return 0;
}

static int egalax_i2c_pm_suspend(struct i2c_client *client, pm_message_t mesg)
{
	EGALAX_DBG(DBG_SUSP, " Enter pm_suspend state:%d\n",
		   p_egalax_i2c_dev->work_state);

	if (!p_egalax_i2c_dev)
		goto fail_suspend;

	if (!p_egalax_i2c_dev->is_disabled) {
		/* only called when input device is not disabled/enabled via
		 * /sys/class/input/input0/enabled interface.
		 * Android uses sysfs by default and will not run into here
		 */
		egalax_power_off();
		p_egalax_i2c_dev->work_state = MODE_SUSPEND;
	}

	EGALAX_DBG(DBG_SUSP, " pm_suspend done!!\n");
	return 0;

fail_suspend:
	EGALAX_DBG(DBG_SUSP, " pm_suspend failed!!\n");
	return -1;
}

static int egalax_i2c_pm_resume(struct i2c_client *client)
{
	EGALAX_DBG(DBG_SUSP, " Enter pm_resume state:%d\n",
			p_egalax_i2c_dev->work_state);

	if (!p_egalax_i2c_dev)
		goto fail_resume;

	if (!p_egalax_i2c_dev->is_disabled) {
		/* only called when input device is not disabled/enabled via
		 * /sys/class/input/input0/enabled interface.
		 * Android uses sysfs by default and will not run into here
		 */
		p_egalax_i2c_dev->work_state = MODE_WORKING;
		egalax_power_on();
		egalax_i2c_senduppoint();

		EGALAX_DBG(DBG_SUSP, " pm_resume done!!\n");
	}
	return 0;

fail_resume:
	EGALAX_DBG(DBG_SUSP, " pm_resume failed!!\n");
	return -1;
}

static int egalax_i2c_ops_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	pm_message_t state;

	state.event = PM_EVENT_SUSPEND;
	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);
	return egalax_i2c_pm_suspend(client, state);
}

static int egalax_i2c_ops_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	EGALAX_DBG(DBG_SUSP, " %s\n", __func__);
	return egalax_i2c_pm_resume(client);
}

static int request_dt(struct i2c_client *client)
{
	int result = 0, val;
	struct device_node *devnode;

	devnode = client->dev.of_node;
	if (devnode) {
		p_egalax_i2c_dev->interrupt_gpio = of_get_named_gpio(devnode,
							"irq-gpio", 0);
		p_egalax_i2c_dev->reset_gpio = of_get_named_gpio(devnode,
							"reset-gpio", 0);
		if (of_property_read_bool(devnode, "enable-active-high"))
			p_egalax_i2c_dev->enable_high = true;

		/* Touch orientation */
		result = of_property_read_u32(devnode, "flip-x", &val);
		if (result < 0)
			val = 0;
		p_egalax_i2c_dev->flip_x = val != 0 ? true : false;
		result = of_property_read_u32(devnode, "flip-y", &val);
		if (result < 0)
			val = 0;
		p_egalax_i2c_dev->flip_y = val != 0 ? true : false;

		/* regulator */
		p_egalax_i2c_dev->regulator_hv = devm_regulator_get(
						&client->dev, "vdd-ts-hv");
		if (IS_ERR(p_egalax_i2c_dev->regulator_hv)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-12v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_hv));
			return -EINVAL;
		}
		p_egalax_i2c_dev->regulator_5v0 = devm_regulator_get(
						&client->dev, "vdd-ts-5v0");
		if (IS_ERR(p_egalax_i2c_dev->regulator_5v0)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd-5v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_5v0));
			return -EINVAL;
		}

		p_egalax_i2c_dev->regulator_3v3 = devm_regulator_get(
						&client->dev, "vdd-ts-3v3");
		if (IS_ERR(p_egalax_i2c_dev->regulator_3v3)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 3v3 regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_3v3));
			return -EINVAL;
		}
		p_egalax_i2c_dev->regulator_1v8 = devm_regulator_get(
						&client->dev, "vdd-ts-1v8");
		if (IS_ERR(p_egalax_i2c_dev->regulator_1v8)) {
			EGALAX_DBG(DBG_MODULE,
				"vdd 18v regulator_get failed: %ld\n",
				PTR_ERR(p_egalax_i2c_dev->regulator_1v8));
			return -EINVAL;
		}
	}

	if (!gpio_is_valid(p_egalax_i2c_dev->interrupt_gpio)) {
		EGALAX_DBG(DBG_MODULE, " gpio[%d] is not valid\n",
			   p_egalax_i2c_dev->interrupt_gpio);
		return -EINVAL;
	}
	result = gpio_request(p_egalax_i2c_dev->interrupt_gpio, "irq-gpio");
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " gpio_request[%d] failed: %d\n",
			   p_egalax_i2c_dev->interrupt_gpio, result);
		return -EINVAL;
	}
	gpio_direction_input(p_egalax_i2c_dev->interrupt_gpio);
	client->irq = gpio_to_irq(p_egalax_i2c_dev->interrupt_gpio);

	if (!gpio_is_valid(p_egalax_i2c_dev->reset_gpio)) {
		EGALAX_DBG(DBG_MODULE, " gpio[%d] is not valid\n",
			   p_egalax_i2c_dev->reset_gpio);
		return -EINVAL;
	}
	result = gpio_request(p_egalax_i2c_dev->reset_gpio, "rest-gpio");
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " gpio_request[%d] failed: %d\n",
			   p_egalax_i2c_dev->reset_gpio, result);
		return -EINVAL;
	}

	return result;
}

static SIMPLE_DEV_PM_OPS((egalax_i2c_pm_ops), (egalax_i2c_ops_suspend),
			 (egalax_i2c_ops_resume));

static const struct file_operations egalax_cdev_fops = {
	.owner		= THIS_MODULE,
	.read		= egalax_cdev_read,
	.write		= egalax_cdev_write,
	.open		= egalax_cdev_open,
	.release	= egalax_cdev_release,
	.poll		= egalax_cdev_poll,
};

static struct miscdevice egalax_misc_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "touch",
	.fops	= &egalax_cdev_fops,
};

static const struct file_operations egalax_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= egalax_proc_open,
	.read		= seq_read,
	.write		= egalax_proc_write,
	.llseek		= seq_lseek,
	.release	= single_release,
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

static int egalax_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *idp)
{
	int result;

	p_egalax_i2c_dev = NULL;
	p_char_dev = NULL;
	input_dev = NULL;
	total_pts_cnt = 0;
	recv_pts_cnt = 0;
	EGALAX_DBG(DBG_MODULE, " Start probe\n");

	p_egalax_i2c_dev = kzalloc(sizeof(struct _egalax_i2c), GFP_KERNEL);
	if (!p_egalax_i2c_dev) {
		EGALAX_DBG(DBG_MODULE, " Request memory failed\n");
		result = -ENOMEM;
		goto fail1;
	}
	p_egalax_i2c_dev->client = client;

	result = request_dt(client);
	if (result < 0) {
		EGALAX_DBG(DBG_MODULE, " Request DT failed\n");
		result = -ENODEV;
		goto fail1;
	}

	egalax_power_on();

	input_dev = allocate_input_dev();
	if (input_dev == NULL) {
		EGALAX_DBG(DBG_MODULE, " allocate_Input_Dev failed\n");
		result = -EINVAL;
		goto fail2;
	}
	EGALAX_DBG(DBG_MODULE, " Register input device done\n");

	mutex_init(&p_egalax_i2c_dev->mutex_wq);

	p_egalax_i2c_dev->ktouch_wq =
			create_singlethread_workqueue("egalax_touch_wq");
	INIT_WORK(&p_egalax_i2c_dev->work_irq, egalax_i2c_wq_irq);

	i2c_set_clientdata(client, p_egalax_i2c_dev);

	if (gpio_get_value(p_egalax_i2c_dev->interrupt_gpio))
		p_egalax_i2c_dev->skip_packet = 0;
	else
		p_egalax_i2c_dev->skip_packet = 1;

	p_egalax_i2c_dev->work_state = MODE_WORKING;
	p_egalax_i2c_dev->is_disabled = false;

	result = request_irq(client->irq, egalax_i2c_interrupt,
			     IRQF_TRIGGER_LOW, client->name, p_egalax_i2c_dev);
	if (result) {
		EGALAX_DBG(DBG_MODULE, " Request irq(%d) failed\n",
			   client->irq);
		goto fail3;
	}
	EGALAX_DBG(DBG_MODULE, " Request irq(%d) gpio(%d) with result:%d\n",
		   client->irq, p_egalax_i2c_dev->interrupt_gpio, result);

	p_char_dev = setup_chardev();
	if (p_char_dev == NULL)
		goto fail4;

	result = misc_register(&egalax_misc_dev);
	if (result) {
		EGALAX_DBG(DBG_MODULE, " misc device register failed\n");
		goto fail5;
	}

	result = sysfs_create_group(&egalax_misc_dev.this_device->kobj,
				    &egalax_attr_group);
	if (result) {
		EGALAX_DBG(DBG_MODULE,
			   " Failed to create sysfs attributes:%d\n", result);
		goto fail6;
	}

	dbg_proc_file = proc_create(PROC_FS_NAME, 0660, NULL,
				  &egalax_proc_fops);
	if (dbg_proc_file == NULL) {
		remove_proc_entry(PROC_FS_NAME, NULL);
		EGALAX_DBG(DBG_MODULE, " Could not initialize /proc/%s\n",
			   PROC_FS_NAME);
	}

	init_waitqueue_head(&p_egalax_i2c_dev->sysfs_query_queue);
	p_egalax_i2c_dev->sysfs_query_wait = false;
	p_egalax_i2c_dev->sysfs_hook_cmd[0] = 0xFF;
	p_egalax_i2c_dev->sysfs_hook_cmd[1] = 0xFF;
	p_egalax_i2c_dev->sysfs_hook_cmd[2] = 0xFF;

	EGALAX_DBG(DBG_MODULE, " I2C probe done\n");
	return 0;
fail6:
	misc_deregister(&egalax_misc_dev);
fail5:
	kfree(p_char_dev->p_fifo_buf);
	kfree(p_char_dev);
	p_char_dev = NULL;
fail4:
	free_irq(client->irq, p_egalax_i2c_dev);
fail3:
	i2c_set_clientdata(client, NULL);
	destroy_workqueue(p_egalax_i2c_dev->ktouch_wq);
	input_unregister_device(input_dev);
	input_dev = NULL;
fail2:
	gpio_free(p_egalax_i2c_dev->interrupt_gpio);
fail1:
	kfree(p_egalax_i2c_dev);
	p_egalax_i2c_dev = NULL;

	EGALAX_DBG(DBG_MODULE, " I2C probe failed\n");
	return result;
}

static int egalax_i2c_remove(struct i2c_client *client)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);

	if (p_char_dev != NULL) {
		kfree(p_char_dev->p_fifo_buf);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	egalax_i2c->work_state = MODE_STOP;

	cancel_work_sync(&egalax_i2c->work_irq);

	if (client->irq) {
		disable_irq(client->irq);
		free_irq(client->irq, egalax_i2c);
	}

	/* turn off power */
	egalax_power_off();

	gpio_free(egalax_i2c->interrupt_gpio);
	gpio_free(egalax_i2c->reset_gpio);

	if (egalax_i2c->ktouch_wq)
		destroy_workqueue(egalax_i2c->ktouch_wq);

	if (input_dev) {
		sysfs_remove_group(
				   &egalax_misc_dev.this_device->kobj,
				   &egalax_attr_group);
		misc_deregister(&egalax_misc_dev);
		EGALAX_DBG(DBG_MODULE,  " Unregister input device\n");
		input_unregister_device(input_dev);
		input_dev = NULL;
	}

	if (dbg_proc_file != NULL)
		remove_proc_entry(PROC_FS_NAME, NULL);

	i2c_set_clientdata(client, NULL);
	kfree(egalax_i2c);
	p_egalax_i2c_dev = NULL;

	return 0;
}

static void egalax_i2c_shutdown(struct i2c_client *client)
{
	struct _egalax_i2c *egalax_i2c = i2c_get_clientdata(client);

	if (client->irq) {
		disable_irq(client->irq);
		free_irq(client->irq, egalax_i2c);
	}
	egalax_power_off();
}

static const struct i2c_device_id egalax_i2c_idtable[] = {
	{ "egalax_i2c", 0 },
	{ }
};

static const struct of_device_id egalax_i2c_dt_ids[] = {
	{ .compatible = "eeti,exc80_ts" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, egalax_i2c_idtable);

static struct i2c_driver egalax_i2c_driver = {
	.driver = {
		.name	= "egalax_i2c",
		.owner	= THIS_MODULE,
		.of_match_table = egalax_i2c_dt_ids,
		.pm		= &egalax_i2c_pm_ops,
	},
	.id_table	= egalax_i2c_idtable,
	.probe		= egalax_i2c_probe,
	.remove		= egalax_i2c_remove,
	.shutdown	= egalax_i2c_shutdown,
};

static void egalax_i2c_ts_exit(void)
{

	i2c_del_driver(&egalax_i2c_driver);
	EGALAX_DBG(DBG_MODULE, " Exit driver done!\n");
}

static int egalax_i2c_ts_init(void)
{

	EGALAX_DBG(DBG_MODULE, " Driver init done!\n");
	return i2c_add_driver(&egalax_i2c_driver);

}

module_init(egalax_i2c_ts_init);
module_exit(egalax_i2c_ts_exit);

MODULE_AUTHOR("EETI <touch_fae@eeti.com>");
MODULE_DESCRIPTION("egalax all points controller i2c driver");
MODULE_LICENSE("GPL");
