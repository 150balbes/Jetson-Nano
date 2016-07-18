/*
 * pps-gpio.c -- PPS client driver using GPIO
 *
 *
 * Copyright (C) 2010 Ricardo Martins <rasm@fe.up.pt>
 * Copyright (C) 2011 James Nuss <jamesnuss@nanometrics.ca>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define PPS_GPIO_NAME "pps-gpio"
#define pr_fmt(fmt) PPS_GPIO_NAME ": " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pps_kernel.h>
#include <linux/pps-gpio.h>
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/pinctrl_amlogic.h>
/* AMLogic GPIO irq bank start offset */
#define	AMLGPIO_IRQ_BASE	96

/* Info for each registered platform device */
static struct pps_gpio_device_data {
	int irq;			/* IRQ used as PPS source */
	struct pps_device *pps;		/* PPS source device */
	struct pps_source_info info;	/* PPS source information */
	bool capture_clear;
	unsigned int gpio_pin;
} pps_data;

// Default GPIO.  Use parameters during module load to override.
// note: this only handles one instance of pps-gpio.  If you want more, use the devicetree version
static int gpio_pin = 238; // GPIOX_10 / pin 12
module_param(gpio_pin, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(gpio_pin, "GPIO pin number (\"Export GPIO\" type), default=238 (GPIOX_10 / pin 12)");

static int assert_falling_edge = 0;
module_param(assert_falling_edge, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(assert_falling_edge, "use the falling edge instead of the rising edge, default=0 (rising)");

/*
 * Report the PPS event
 */

static irqreturn_t pps_gpio_irq_handler(int irq, void *data)
{
	struct pps_event_time ts;
	int rising_edge;

	/* Get the time stamp first */
	pps_get_ts(&ts);

	rising_edge = gpio_get_value(pps_data.gpio_pin);
	if ((rising_edge && !assert_falling_edge) ||
			(!rising_edge && assert_falling_edge))
		pps_event(pps_data.pps, &ts, PPS_CAPTUREASSERT, NULL);
	else if (pps_data.capture_clear &&
			((rising_edge && assert_falling_edge) ||
			 (!rising_edge && !assert_falling_edge)))
		pps_event(pps_data.pps, &ts, PPS_CAPTURECLEAR, NULL);

	return IRQ_HANDLED;
}

static unsigned long
get_irqf_trigger_flags(const struct pps_gpio_device_data *data)
{
	unsigned long flags = assert_falling_edge ?
		IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;

	if (data->capture_clear) {
		flags |= ((flags & IRQF_TRIGGER_RISING) ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	}

	return flags;
}

static int pps_gpio_irq_setup(unsigned int gpio_pin)
{
	struct gpio_chip *chip;
	int ret;
	unsigned long irq_flags;
	int irq_banks[2] = {0, 0};

	chip = gpio_to_chip(gpio_pin);
	if(!chip)
		return -EINVAL;

	// gpio_to_irq translates from global GPIO # to chip offset, which meson_setup_irq wants
	gpio_pin = gpio_to_irq(gpio_pin);

	irq_flags = assert_falling_edge ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
	ret = meson_setup_irq(chip, gpio_pin, irq_flags, &irq_banks[0]);
	if(ret < 0)
		return -EINVAL;

	if(irq_banks[0] != -1)
		return irq_banks[0] + AMLGPIO_IRQ_BASE;
	if(irq_banks[1] != -1)
		return irq_banks[1] + AMLGPIO_IRQ_BASE;

	return -EINVAL;
}

static void pps_gpio_release(struct device *dev) {
  // does nothing, everything is allocated as part of the module
}

static struct device pps_gpio_dev = {
  .id = 0,
  .parent = &platform_bus,
  .release = &pps_gpio_release
};

static int __init pps_gpio_init(void)
{
	int ret;
	int pps_default_params;

	/* register the device */
	ret = dev_set_name(&pps_gpio_dev, "%s%d", PPS_GPIO_NAME, pps_gpio_dev.id);	
	if(ret < 0) {
		pr_err("dev_set_name failed with %d\n", ret);
		return ret;
	}
	ret = device_register(&pps_gpio_dev);
	if(ret < 0) {
		pr_err("device_register failed with %d\n", ret);
		return ret;
	}

	pps_data.gpio_pin = gpio_pin;

	/* GPIO setup */
	ret = devm_gpio_request(&pps_gpio_dev, pps_data.gpio_pin, "PPS-GPIO");
	if (ret) {
		dev_err(&pps_gpio_dev, "failed to request GPIO %u\n",
			pps_data.gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(pps_data.gpio_pin);
	if (ret) {
		dev_err(&pps_gpio_dev, "failed to set pin direction\n");
		return -EINVAL;
	}

	/* IRQ setup */
        ret = pps_gpio_irq_setup(pps_data.gpio_pin);
	if (ret < 0) {
		dev_err(&pps_gpio_dev, "failed to map GPIO to IRQ: %d\n", ret);
		return -EINVAL;
	}
	pps_data.irq = ret;

	/* initialize PPS specific parts of the bookkeeping data structure. */
	pps_data.info.mode = PPS_CAPTUREASSERT | PPS_OFFSETASSERT |
		PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
	if (pps_data.capture_clear)
		pps_data.info.mode |= PPS_CAPTURECLEAR | PPS_OFFSETCLEAR |
			PPS_ECHOCLEAR;
	pps_data.info.owner = THIS_MODULE;
	snprintf(pps_data.info.name, PPS_MAX_NAME_LEN - 1, "%s.%d",
		 PPS_GPIO_NAME, pps_gpio_dev.id);

	/* register PPS source */
	pps_default_params = PPS_CAPTUREASSERT | PPS_OFFSETASSERT;
	if (pps_data.capture_clear)
		pps_default_params |= PPS_CAPTURECLEAR | PPS_OFFSETCLEAR;
	pps_data.pps = pps_register_source(&pps_data.info, pps_default_params);
	if (pps_data.pps == NULL) {
		dev_err(&pps_gpio_dev, "failed to register IRQ %d as PPS source\n",
			pps_data.irq);
		return -EINVAL;
	}

	/* register IRQ interrupt handler */
	ret = devm_request_irq(&pps_gpio_dev, pps_data.irq, pps_gpio_irq_handler,
			get_irqf_trigger_flags(&pps_data), pps_data.info.name, NULL);
	if (ret) {
		pps_unregister_source(pps_data.pps);
		dev_err(&pps_gpio_dev, "failed to acquire IRQ %d\n", pps_data.irq);
		return -EINVAL;
	}

	dev_info(pps_data.pps->dev, "Registered IRQ %d as PPS source\n",
		 pps_data.irq);

	return 0;
}

static void __exit pps_gpio_exit(void)
{
	unsigned int gpio_pin;
	int irq_banks[2] = {0, 0};

	// gpio_to_irq translates from global GPIO # to chip offset, which meson_free_irq wants
	gpio_pin = gpio_to_irq(pps_data.gpio_pin);
	meson_free_irq(gpio_pin, irq_banks);

	pps_unregister_source(pps_data.pps);
	dev_info(&pps_gpio_dev, "removed IRQ %d as PPS source\n", pps_data.irq);

	device_unregister(&pps_gpio_dev);
}

module_init(pps_gpio_init);
module_exit(pps_gpio_exit);

MODULE_AUTHOR("Ricardo Martins <rasm@fe.up.pt>");
MODULE_AUTHOR("James Nuss <jamesnuss@nanometrics.ca>");
MODULE_DESCRIPTION("Use GPIO pin as PPS source");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
