/* Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* NVS = NVidia Sensor framework */
/* See nvs_iio.c and nvs.h for documentation */


#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/nvs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define DEFAULT_ON_MS 1000

struct nvs_led_test_context {
	struct device *dev;
	struct nvs_fn_if *nvs;
	void *nvs_st;
	struct sensor_cfg cfg;
	unsigned int sts;
	unsigned int errs;
	unsigned int enabled;
	int in_irq;
	int out_gpio;
	int enable_gpio;
	enum of_gpio_flags enable_gpio_flags;
	struct delayed_work work;
};

static void nvs_led_test_generate_event(struct nvs_led_test_context *ctx,
					int on_ms)
{
	s64 ts;
	u32 data = 0xcafebabe;

	/*
	 * If a valid ms count is provided, and we have an out-gpio, write 1
	 * to the out-gpio (presumably, to turn on an LED), and schedule
	 * work to turn it back off.
	 */
	if (on_ms > 0) {
		dev_info(ctx->dev, "Setting out-gpio to 1\n");
		schedule_delayed_work(&ctx->work, (on_ms * HZ) / 1000);
		gpio_set_value_cansleep(ctx->out_gpio, 1);
	}

	/*
	 * Regardless, generate the test sensor event.  For some reason,
	 * we need at least one channel, or the (Android) app doesn't appear
	 * to see the event, so we just insert some bogus data.
	 */
	ts = nvs_timestamp();
#ifdef CONFIG_NVS_LED_TRACE_PRINTK
	trace_printk(
		"Sensor event occurred at %llu nsec (-500usec(hw/irq latency) -sampling_period)\n",
		ts);
#endif
	ctx->nvs->handler(ctx->nvs_st, &data, ts);
}

static void nvs_led_test_work(struct work_struct *work)
{
	struct nvs_led_test_context *ctx = container_of(work,
						    struct nvs_led_test_context,
						    work.work);

	dev_info(ctx->dev, "Setting out-gpio to 0\n");
	gpio_set_value_cansleep(ctx->out_gpio, 0);
}

static irqreturn_t nvs_led_test_threaded_isr(int irq, void *data)
{
	struct nvs_led_test_context *ctx = data;

	WARN_ON(!ctx);

	if (ctx)
		nvs_led_test_generate_event(ctx, DEFAULT_ON_MS);

	return IRQ_HANDLED;
}

static ssize_t nvs_led_test_store_on_ms(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct nvs_led_test_context *ctx = dev_get_drvdata(dev);
	int on_ms;
	int err;

	WARN_ON(!ctx);

	if (!ctx)
		return 0;

	err = kstrtouint(buf, 0, &on_ms);
	if (err < 0)
		dev_warn(dev, "Invalid ms %s\n", buf);

	nvs_led_test_generate_event(ctx, on_ms);

	return count;
}

static DEVICE_ATTR(on_ms, S_IWUSR | S_IWGRP, NULL, nvs_led_test_store_on_ms);

static int nvs_led_test_enable(void *client, int snsr_id, int enable)
{
	struct nvs_led_test_context *ctx =
				(struct nvs_led_test_context *)client;

	if (enable < 0)
		return ctx->enabled;

	ctx->enabled = enable;

	return 0;
}

static struct sensor_cfg nvs_led_test_cfg_dflt = {
	.part			= "nvs",
	.name			= "test",
	.vendor			= "NVIDIA",
	.version		= 1,
	.ch_n			= 1,
	.ch_sz			= 4,
	.flags			= SENSOR_FLAG_ON_CHANGE_MODE,
};

static struct nvs_fn_dev nvs_led_test_fn_dev = {
	.enable			= nvs_led_test_enable,
};

static int nvs_led_test_parse_dt(struct nvs_led_test_context *ctx,
			     struct device_node *np)
{
	ctx->in_irq = irq_of_parse_and_map(np, 0);
	ctx->out_gpio = of_get_named_gpio(np, "out-gpios", 0);
	ctx->enable_gpio = of_get_named_gpio_flags(np, "enable-gpio", 0,
						   &ctx->enable_gpio_flags);

	return 0;
}

static int nvs_led_test_probe(struct platform_device *pdev)
{
	struct nvs_led_test_context *ctx;
	int err;

	/* Create and set up our driver-specific context. */
	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	memcpy(&ctx->cfg, &nvs_led_test_cfg_dflt, sizeof(ctx->cfg));
	INIT_DELAYED_WORK(&ctx->work, nvs_led_test_work);
	dev_set_drvdata(&pdev->dev, ctx);
	ctx->dev = &pdev->dev;

	err = nvs_led_test_parse_dt(ctx, pdev->dev.of_node);
	if (err)
		return err;

	/*
	 * If we are given an irq to trigger the generation of test sensor
	 * events (like, from a button wired to a GPIO), set up the ISR.
	 */
	if (ctx->in_irq > 0) {
		err = devm_request_threaded_irq(&pdev->dev, ctx->in_irq,
			NULL, nvs_led_test_threaded_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			pdev->name, ctx);
		if (err) {
			dev_err(&pdev->dev,
				"devm_request_irq() failed with err %d\n", err);
			ctx->in_irq = 0;
			return err;
		}
		dev_info(&pdev->dev, "Set up input gpio\n");
	}

	/*
	 * If there is a required enable-gpio set, then enable that first.
	 */
	if (gpio_is_valid(ctx->enable_gpio)) {
		err = devm_gpio_request(&pdev->dev, ctx->enable_gpio,
					"NVS LED enable GPIO");
		if (err) {
			dev_err(&pdev->dev, "Failed to request Enable GPIO");
			return err;
		}

		if (ctx->enable_gpio_flags & OF_GPIO_ACTIVE_LOW)
			err = gpio_direction_output(ctx->enable_gpio, 0);
		else
			err = gpio_direction_output(ctx->enable_gpio, 1);

		if (err) {
			dev_err(&pdev->dev, "Failed setting output of gpio");
			return err;
		}
	}

	/*
	 * If we are given an output GPIO (like, an LED wired to a GPIO),
	 * set that output GPIO up.
	 */
	if (!gpio_is_valid(ctx->out_gpio)) {
		dev_err(&pdev->dev, "Invalid GPIO set");
		return -EINVAL;
	}
	err = devm_gpio_request(&pdev->dev, ctx->out_gpio,
				"NVS LED test Sensor out");
	if (err) {
		dev_err(&pdev->dev, "Failed requesting gpio");
		return err;
	}

	err = gpio_direction_output(ctx->out_gpio, 0);
	if (err) {
		dev_err(&pdev->dev, "Failed setting output gpio");
		return err;
	}

	dev_info(&pdev->dev, "Set up output gpio\n");

	/* Register with NVS */
	err = nvs_of_dt(pdev->dev.of_node, &ctx->cfg, NULL);
	if (err < 0)
		return err;

	nvs_led_test_fn_dev.errs = &ctx->errs;
	nvs_led_test_fn_dev.sts = &ctx->sts;

	ctx->nvs = nvs_iio();
	if (!ctx->nvs)
		return -ENODEV;

	err = ctx->nvs->probe(&ctx->nvs_st, ctx, &pdev->dev,
			      &nvs_led_test_fn_dev, &ctx->cfg);
	if (err)
		return err;

	/* Create our sysfs file for triggering test sensor event generation. */
	err = device_create_file(&pdev->dev, &dev_attr_on_ms);
	if (err)
		return err;

	dev_info(&pdev->dev, "NVS LED test sensor driver initialized.\n");

	return 0;
}

static int nvs_led_test_remove(struct platform_device *pdev)
{
	struct nvs_led_test_context *ctx =
		(struct nvs_led_test_context *)dev_get_drvdata(&pdev->dev);

	WARN_ON(!ctx);

	if (!ctx)
		return 0;

	dev_info(&pdev->dev, "NVS LED test sensor driver removed.\n");

	device_remove_file(&pdev->dev, &dev_attr_on_ms);

	flush_delayed_work(&ctx->work);

	ctx->nvs->remove(&ctx->nvs_st);

	return 0;
}

static const struct of_device_id nvs_led_test_of_match[] = {
	{ .compatible = "nvidia,nvs-led-test", },
	{}
};

MODULE_DEVICE_TABLE(of, nvs_led_test_of_match);

static struct platform_driver nvs_led_test_driver = {
	.driver				= {
		.name			= "nvs-led-test",
		.of_match_table		= of_match_ptr(nvs_led_test_of_match),
	},
	.probe				= nvs_led_test_probe,
	.remove				= nvs_led_test_remove,
};

module_platform_driver(nvs_led_test_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NVidia NVS LED Test driver");
MODULE_AUTHOR("NVIDIA Corporation");

