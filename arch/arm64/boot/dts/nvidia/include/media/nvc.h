/* Copyright (c) 2012-2019, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __NVC_H__
#define __NVC_H__

#include <linux/regulator/consumer.h>
#include <uapi/media/nvc.h>

#define MAKE_CONSTUSER_PTR(p)   (const void __user *)((unsigned long)(p))
#define MAKE_USER_PTR(p)        (void __user *)((unsigned long)(p))

/* The NVC_CFG_ defines are for the .cfg entry in the
 * platform data structure.
 */
/* Device not registered if not found */
#define NVC_CFG_NODEV			(1 << 0)
/* Don't return errors */
#define NVC_CFG_NOERR			(1 << 1)
/* Always go to _PWR_STDBY instead of _PWR_OFF */
#define NVC_CFG_OFF2STDBY		(1 << 2)
/* Init device at sys boot */
#define NVC_CFG_BOOT_INIT		(1 << 3)
/* Sync mode uses an I2C MUX to send at same time */
#define NVC_CFG_SYNC_I2C_MUX		(1 << 4)

struct nvc_regulator_init {
	unsigned vreg_num;
	const char *vreg_name;
};

struct nvc_regulator {
	bool vreg_flag;
	struct regulator *vreg;
	const char *vreg_name;
};

/* The GPIO mechanism uses the _gpio_type in the device's header file as a key
 * to define all the possible GPIO's the device will need.  The key is used to
 * combine the GPIO's defined in the platform board file using the
 * nvc_gpio_pdata structure with the nvc_gpio structure in the nvc kernel
 * driver.
 */
struct nvc_gpio_pdata {
	/* use a _gpio_type enum from the device's header file */
	unsigned gpio_type;
	/* the GPIO system number */
	unsigned gpio;
	/* init_en is typically set to true for all GPIO's used by the driver.
	 * However, some GPIO's are used by multiple drivers (CSI MUX, reset,
	 * etc.).  In this case, this is set true for only one of the drivers
	 * that uses the GPIO and false for the others.  If the platform board
	 * file initializes the GPIO, then this is false for all of the drivers
	 * using the GPIO.
	 */
	bool init_en;
	/* this defines the assert level for the general purpose GPIO's
	 * (_GPIO_TYPE_GPx, etc.).  The _GPIO_TYPE_GPx can be used for a GPIO
	 * that the driver doesn't know about but is needed in order for the
	 * device to work (CSI select, regulator, etc.).  The driver will
	 * blindly assert the GPIO when the device is operational and deassert
	 * when the device is turned off.
	 */
	bool active_high;
};

struct nvc_gpio_init {
	/* key to match in nvc_gpio_pdata */
	unsigned gpio_type;
	/* same as in gpio.h */
	unsigned long flags;
	/* same as in gpio.h */
	const char *label;
	/* used instead of nvc_gpio_pdata.active_high if use_flags true */
	bool active_high;
	/* false if nvc_gpio_pdata.active_high used else flags is used */
	bool use_flags;
};

struct nvc_gpio {
	unsigned gpio; /* system GPIO number */
	bool own; /* gets set if driver initializes */
	bool active_high; /* used for GP GPIOs */
	bool valid; /* set if struct data is valid */
	bool flag; /* scratch flag for driver implementation */
};

#endif /* __NVC_H__ */
