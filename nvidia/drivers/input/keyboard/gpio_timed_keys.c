/*
 * Timed gpio keys driver based on the gpio keys driver.
 * Key presses are timed to generate different key codes
 * based on the config in the dt node.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * Copyright (c) 2010-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
#include <linux/system-wakeup.h>
#include <linux/tegra-pm.h>
#include <linux/ktime.h>

#define TIME_IN_USEC	1000000

struct device;

struct gpio_timed_keys_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
	int irq;		/* Irq number in case of interrupt keys */
	int *press_times;	/* Amount of time the key needs to be pressed to return code */
	int *key_codes;
	int num_vals;
	ktime_t down_time;
};

struct gpio_timed_keys_platform_data {
	struct gpio_timed_keys_button *buttons;
	int nbuttons;
	unsigned int poll_interval;	/* polling interval in msecs -
					   for polling driver only */
	unsigned int rep:1;		/* enable input subsystem auto repeat */
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
	int (*wakeup_key)(void);
};

struct gpio_timed_button_data {
	const struct gpio_timed_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
	ktime_t down_time;
};

struct gpio_timed_keys_drvdata {
	struct notifier_block pm_nb;
	const struct gpio_timed_keys_platform_data *pdata;
	struct input_dev *input;
	struct mutex disable_lock;
	struct gpio_timed_button_data data[0];
};

static void gpio_timed_keys_gpio_report_event(struct gpio_timed_button_data *bdata)
{
	const struct gpio_timed_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;
	u64 press_time;
	int i;

	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		if (state == 1) {
			bdata->down_time = ktime_get();
		} else {
			press_time =
				ktime_us_delta(ktime_get(), bdata->down_time);

			for (i = 0; i < button->num_vals; i++) {
				if (press_time <=
					button->press_times[i]*TIME_IN_USEC) {
					input_event(input, type,
						button->key_codes[i], 1);
					input_sync(input);
					input_event(input, type,
						button->key_codes[i], 0);
					input_sync(input);
					break;
				}
			}
		}
	}
}

static void gpio_timed_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_timed_button_data *bdata =
		container_of(work, struct gpio_timed_button_data, work);

	gpio_timed_keys_gpio_report_event(bdata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void gpio_timed_keys_gpio_timer(unsigned long _data)
{
	struct gpio_timed_button_data *bdata =
				(struct gpio_timed_button_data *)_data;

	schedule_work(&bdata->work);
}

static irqreturn_t gpio_timed_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_timed_button_data *bdata = dev_id;

	BUG_ON(irq != bdata->irq);

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);
	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static int gpio_timed_keys_setup_key(struct platform_device *pdev,
				struct input_dev *input,
				struct gpio_timed_button_data *bdata,
				const struct gpio_timed_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_timed_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error, i;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio) && (button->irq <= 0)) {
		error = devm_gpio_request_one(dev, button->gpio, GPIOF_IN, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->timer_debounce =
						button->debounce_interval;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_timed_keys_gpio_work_func);
		setup_timer(&bdata->timer,
			    gpio_timed_keys_gpio_timer, (unsigned long)bdata);

		isr = gpio_timed_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (button->irq <= 0) {
			dev_err(dev, "No IRQ specified \n");
			return -EINVAL;
		}

		if (!gpio_is_valid(button->gpio)) {
			dev_err(dev, "Invalid GPIO \n");
			return -EINVAL;
		}
	}

	for (i = 0; i < button->num_vals; i++) {
		input_set_capability(input, button->type ?: EV_KEY, button->key_codes[i]);
	}

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		return error;
	}

	return 0;
}

static void gpio_timed_keys_report_state(struct gpio_timed_keys_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_timed_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_timed_keys_gpio_report_event(bdata);
	}
	input_sync(input);
}

static int gpio_timed_keys_open(struct input_dev *input)
{
	struct gpio_timed_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_timed_keys_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	/* Report current state of buttons that are connected to GPIOs */
	gpio_timed_keys_report_state(ddata);

	return 0;
}

static void gpio_timed_keys_close(struct input_dev *input)
{
	struct gpio_timed_keys_drvdata *ddata = input_get_drvdata(input);
	const struct gpio_timed_keys_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_timed_keys_platform_data *
gpio_timed_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct gpio_timed_keys_platform_data *pdata;
	struct gpio_timed_keys_button *button;
	int *time_data;
	int *code_data;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata) + nbuttons * (sizeof *button),
			GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->buttons = (struct gpio_timed_keys_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);

	of_property_read_string(node, "gpio-timed-keys,name", &pdata->name);
	if (!pdata->name)
		pdata->name = node->name;

	i = 0;
	for_each_child_of_node(node, pp) {
		int gpio = -1;
		unsigned int irq = 0;
		enum of_gpio_flags flags;
		bool gpio_props = false;
		bool irq_prop = false;

		/* Ignore the button if it is disabled. */
		error = of_device_is_available(pp);
		if (!error) {
			dev_info(dev, "Button %s is ignored\n", pp->name);
			pdata->nbuttons--;
			continue;
		}

		if (of_find_property(pp, "gpios", NULL))
			gpio_props = true;

		irq = irq_of_parse_and_map(pp, 0);
		if (irq > 0)
			irq_prop = true;

		if (!gpio_props && !irq_prop) {
			dev_warn(dev, "Found button without gpios/irq\n");
			pdata->nbuttons--;
			continue;
		}

		if (!gpio_props)
			goto gpio_get;

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,
					"Failed to get gpio flags, error: %d\n",
					error);
			goto err_out;
		}

gpio_get:
		button = &pdata->buttons[i++];

		button->gpio = gpio;
		button->irq = irq;
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,num_codes",
						&button->num_vals))
			button->num_vals = 1;

		time_data = devm_kzalloc(dev, sizeof(int) * button->num_vals,
					 GFP_KERNEL);
		if (!time_data) {
			error = -ENOMEM;
			goto err_out;
		}

		code_data = devm_kzalloc(dev, sizeof(int) * button->num_vals,
						GFP_KERNEL);
		if (!code_data) {
			error = -ENOMEM;
			goto err_out;
		}

		if (of_property_read_u32_array(pp, "linux,press-time-secs",
						time_data,
						(size_t)button->num_vals)) {

			dev_err(dev, "Button press times not defined \n");
			error = -EINVAL;
			goto err_out;
		}
		button->press_times = time_data;

		if (of_property_read_u32_array(pp, "linux,key-codes",
						code_data,
						(size_t)button->num_vals)) {

			dev_err(dev, "Button codes not defined \n");
			error = -EINVAL;
			goto err_out;
		}
		button->key_codes = code_data;

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &button->type))
			button->type = EV_KEY;

		button->wakeup = !!of_get_property(pp, "gpio-timed-key,wakeup",
							NULL);

		if (of_property_read_u32(pp, "debounce-interval",
					 &button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0) {
		error = -EINVAL;
		goto err_out;
	}

	return pdata;

err_out:
	return ERR_PTR(error);
}

static struct of_device_id gpio_timed_keys_of_match[] = {
	{ .compatible = "gpio-timed-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_timed_keys_of_match);

static void gpio_remove_key(struct gpio_timed_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int gpio_timed_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct gpio_timed_keys_platform_data *pdata = dev_get_platdata(dev);
	struct gpio_timed_keys_drvdata *ddata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = gpio_timed_keys_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = devm_kzalloc(dev, sizeof(struct gpio_timed_keys_drvdata) +
		 pdata->nbuttons * sizeof(struct gpio_timed_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-timed-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_timed_keys_open;
	input->close = gpio_timed_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct gpio_timed_keys_button *button = &pdata->buttons[i];
		struct gpio_timed_button_data *bdata = &ddata->data[i];

		error = gpio_timed_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int gpio_timed_keys_remove(struct platform_device *pdev)
{
	struct gpio_timed_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

static struct platform_driver gpio_timed_keys_device_driver = {
	.probe		= gpio_timed_keys_probe,
	.remove		= gpio_timed_keys_remove,
	.driver		= {
		.name	= "gpio-timed-keys",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(gpio_timed_keys_of_match),
	}
};

module_platform_driver(gpio_timed_keys_device_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ankita Garg <ankitag@nvidia.com>");
MODULE_DESCRIPTION("Timed Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-timed-keys");
