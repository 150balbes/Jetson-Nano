/*
 * lirc_odroid.c
 *
 * lirc_odroid - Modified version from lirc_gpioblaster.c of OpenWrt.
 *               taken from https://wiki.openwrt.org/doc/howto/lirc-gpioblaster
 *
 * Copyright (C) 2014 Qball Cow <qball@gmpclient.org>,
 * Copyright (C) 2012 Aron Robert Szabo <aron@reon.hu>,
 *                    Michael Bishop <cleverca22@gmail.com>
 *
 *  This driver has been modified to support ODROID-C1/C2
 *  and only supports IR transmitter.
 *      Modified by Joy Cho <joy.cho@hardkernel.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/spinlock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>
#include <linux/gpio.h>

#define LIRC_DRIVER_NAME "lirc_odroid"

#define RBUF_LEN 256
#define LIRC_TRANSMITTER_LATENCY 50

#ifndef MAX_UDELAY_MS
#define MAX_UDELAY_US 5000
#else
#define MAX_UDELAY_US (MAX_UDELAY_MS*1000)
#endif

/* module parameters */
/* set the default GPIO output pin */
static int gpio_out_pin;
/* enable debugging messages */
static bool debug;

/* softcarrier option : default on */
static bool softcarrier = 1;
/* 0 = do not invert output, 1 = invert output */
static bool invert;

/* forward declarations */
static long send_pulse(unsigned long length);
static void send_space(long length);
static void lirc_odroid_exit(void);

static struct platform_device *lirc_odroid_dev;
static struct lirc_buffer rbuf;
static spinlock_t lock;

/* initialized/set in init_timing_params() */
static unsigned int freq = 38000;
static unsigned int duty_cycle = 50;
static unsigned long period;
static unsigned long pulse_width;
static unsigned long space_width;

static void safe_udelay(unsigned long usecs)
{
	while (usecs > MAX_UDELAY_US) {
		udelay(MAX_UDELAY_US);
		usecs -= MAX_UDELAY_US;
	}
	udelay(usecs);
}

static unsigned long read_current_us(void)
{
	struct timespec now;
	getnstimeofday(&now);
	return (now.tv_sec * 1000000) + (now.tv_nsec/1000);
}

static int init_timing_params(unsigned int new_duty_cycle,
	unsigned int new_freq)
{
	/* Time unit of all parameters is microseconds unit.
	 * It's available for the case when base frequency is smaller
	 * than 1MHz.
	 */
	duty_cycle = new_duty_cycle;
	freq = new_freq;
	period = 1000000L / freq;

	/* duty cycle in percentage unit, duty_cycle 50 means 50% */
	pulse_width = period * duty_cycle / 100;
	space_width = period - pulse_width;

	pr_info("in init_timing_params, freq=%d pulse=%ld, space=%ld\n",
		freq, pulse_width, space_width);

	return 0;
}

static long send_pulse_softcarrier(unsigned long length)
{
	int flag;
	unsigned long actual, target;
	unsigned long actual_us, initial_us, target_us;

	actual = 0; target = 0; flag = 0;

	actual_us = read_current_us();
	/* length from lircd.conf is in us unit */
	while (actual < length) {
		if (flag) {
			gpio_set_value(gpio_out_pin, invert);
			target += space_width;
		} else {
			gpio_set_value(gpio_out_pin, !invert);
			target += pulse_width;
		}

		initial_us = actual_us;
		target_us = actual_us + (target - actual);

		/*
		 * Note - we've checked in ioctl that the pulse/space
		 * widths are big enough so that d is > 0
		 */
		if  ((int)(target_us - actual_us) > 0)
			udelay(target_us - actual_us);

		actual_us = read_current_us();
		actual += (actual_us - initial_us);
		flag = !flag;
	}

	return actual-length;
}

static long send_pulse(unsigned long length)
{
	if (length <= 0)
		return 0;

	if (softcarrier) {
		return send_pulse_softcarrier(length);
	} else {
		gpio_set_value(gpio_out_pin, !invert);
		safe_udelay(length);
		return 0;
	}
}

static void send_space(long length)
{
	gpio_set_value(gpio_out_pin, invert);
	if (length <= 0)
		return;
	safe_udelay(length);
}

static int init_port(void)
{
	int ret;

	if (gpio_request(gpio_out_pin, LIRC_DRIVER_NAME " ir/out")) {
		pr_info(LIRC_DRIVER_NAME ": cant claim gpio pin %d\n",
			gpio_out_pin);
		ret = -ENODEV;
		goto exit_init_port;
	}

	gpio_direction_output(gpio_out_pin, invert);
	gpio_set_value(gpio_out_pin, invert);

	return 0;

exit_init_port:
	gpio_free(gpio_out_pin);
	return ret;
}

/* called when the character device is opened */
static int set_use_inc(void *data)
{
	pr_info(LIRC_DRIVER_NAME " is opened\n");
	return 0;
}

static void set_use_dec(void *data)
{
	pr_info(LIRC_DRIVER_NAME " is closed\n");
}

/*
 * Header space pulse
 * + pre_data space pulse pairs
 * + key_codes space pulse pairs
 */
static ssize_t lirc_write(struct file *file, const char *buf,
	size_t n, loff_t *ppos)
{
	int i, count;
	unsigned long flags;
	long delta = 0;
	int *wbuf;

	count = n / sizeof(int);
	if (n % sizeof(int) || count % 2 == 0)
		return -EINVAL;
	wbuf = memdup_user(buf, n);
	if (IS_ERR(wbuf))
		return PTR_ERR(wbuf);
	spin_lock_irqsave(&lock, flags);

	/* refer to space and pulse duration for 'one','zero'
	   space first and then pulse */
	for (i = 0; i < count; i++) {
		if (i%2)
			send_space(wbuf[i] - delta);
		else
			delta = send_pulse(wbuf[i]);
	}
	/* set default level after transmission */
	gpio_set_value(gpio_out_pin, invert);

	spin_unlock_irqrestore(&lock, flags);
	kfree(wbuf);

	return n;
}

static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int result;
	__u32 value;

	switch (cmd) {
	case LIRC_GET_SEND_MODE:
		return -ENOIOCTLCMD;
		break;

	case LIRC_SET_SEND_MODE:
		result = get_user(value, (__u32 *) arg);
		if (result)
			return result;
		/* only LIRC_MODE_PULSE supported */
		if (value != LIRC_MODE_PULSE)
			return -ENOSYS;
		break;

	case LIRC_GET_LENGTH:
		return -ENOSYS;
		break;

	case LIRC_SET_SEND_DUTY_CYCLE:
		result = get_user(value, (__u32 *) arg);
		pr_info(LIRC_DRIVER_NAME " - SET_SEND_DUTY_CYCLE %d\n", value);
		if (result)
			return result;
		if (value <= 0 || value > 100)
			return -EINVAL;
		return init_timing_params(value, freq);
		break;

	case LIRC_SET_SEND_CARRIER:
		result = get_user(value, (__u32 *) arg);
		pr_info(LIRC_DRIVER_NAME " - SET_SEND_CARRIER %d\n", value);
		if (result)
			return result;
		if (value > 500000 || value < 20000)
			return -EINVAL;
		return init_timing_params(duty_cycle, value);
		break;

	default:
		return lirc_dev_fop_ioctl(filep, cmd, arg);
	}
	return 0;
}

static const struct file_operations lirc_fops = {
	.owner		= THIS_MODULE,
	.write		= lirc_write,
	.unlocked_ioctl	= lirc_ioctl,
	.read		= lirc_dev_fop_read,
	.poll		= lirc_dev_fop_poll,
	.open		= lirc_dev_fop_open,
	.release	= lirc_dev_fop_close,
	.llseek		= no_llseek,
};

static struct lirc_driver driver = {
	.name		= LIRC_DRIVER_NAME,
	.minor		= -1,
	.code_length	= 1,
	.sample_rate	= 0,
	.data		= NULL,
	.add_to_buf	= NULL,
	.rbuf		= &rbuf,
	.set_use_inc	= set_use_inc,
	.set_use_dec	= set_use_dec,
	.fops		= &lirc_fops,
	.dev		= NULL,
	.owner		= THIS_MODULE,
};

static struct platform_driver lirc_odroid_driver = {
	.driver = {
		.name   = LIRC_DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init lirc_odroid_init(void)
{
	int result;

	/* Init read buffer. */
	result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (result < 0)
		return -ENOMEM;

	result = platform_driver_register(&lirc_odroid_driver);
	if (result) {
		pr_info("lirc register returned %d\n", result);
		goto exit_buffer_free;
	}

	lirc_odroid_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
	if (!lirc_odroid_dev) {
		result = -ENOMEM;
		goto exit_driver_unregister;
	}

	result = platform_device_add(lirc_odroid_dev);
	if (result)
		goto exit_device_put;

	return 0;

exit_device_put:
	platform_device_put(lirc_odroid_dev);

exit_driver_unregister:
	platform_driver_unregister(&lirc_odroid_driver);

exit_buffer_free:
	lirc_buffer_free(&rbuf);

	return result;
}

static void lirc_odroid_exit(void)
{
	pr_info(LIRC_DRIVER_NAME "[%s]\n", __func__);
	gpio_free(gpio_out_pin);
	platform_device_unregister(lirc_odroid_dev);
	platform_driver_unregister(&lirc_odroid_driver);
	lirc_buffer_free(&rbuf);
}

static int __init lirc_odroid_init_module(void)
{
	int result;

	result = lirc_odroid_init();
	if (result)
		return result;

	result = init_port();
	if (result < 0) {
		pr_info(LIRC_DRIVER_NAME ": init port fail!\n");
		goto exit_odroid;
	}

	/* check if the module received valid gpio pin numbers */
	driver.features = LIRC_CAN_SET_SEND_DUTY_CYCLE |
			  LIRC_CAN_SET_SEND_CARRIER |
			  LIRC_CAN_SEND_PULSE;

	driver.dev = &lirc_odroid_dev->dev;
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		pr_info(LIRC_DRIVER_NAME ": device registration failed with %d\n",
			result);
		result = -EIO;
		goto exit_odroid;
	}

	pr_info(LIRC_DRIVER_NAME ": driver registered!\n");

	return 0;

exit_odroid:
	lirc_odroid_exit();

	return result;
}

static void __exit lirc_odroid_exit_module(void)
{
	lirc_odroid_exit();

	lirc_unregister_driver(driver.minor);
	pr_info(LIRC_DRIVER_NAME ": cleaned up module\n");
}

module_init(lirc_odroid_init_module);
module_exit(lirc_odroid_exit_module);

MODULE_DESCRIPTION("GPIO based IR Transmitter driver for Odroid C/C2");
MODULE_AUTHOR("Joy Cho <joy.cho@hardkernel.com>");
MODULE_LICENSE("GPL");

module_param(gpio_out_pin, int, S_IRUGO);
MODULE_PARM_DESC(gpio_out_pin, "GPIO output/transmitter pin number");

module_param(softcarrier, bool, S_IRUGO);
MODULE_PARM_DESC(softcarrier, "Software carrier (0 = off, 1 = on, default on)");

module_param(invert, bool, S_IRUGO);
MODULE_PARM_DESC(invert, "Invert output (0 = off, 1 = on, default off");

module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");
