/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/tegra-gte.h>
#include <linux/gpio.h>
#include <linux/timer.h>

/*
 * Sample GTE test driver demonstrating GTE API usage.
 *
 * Sample drivers monitors LIC IRQ provided by lic_irq module parameter and
 * GPIO provided by gpio_in parameter.
 *
 * Note: gpio_out and gpio_in need to be shorted externally using some wire
 * in order for this test driver to work for the GPIO monitoring.
 */

/*
 * This represents global ID of the GPIO since global ID or logical
 * partitioning of the GPIOs across various GPIO controller happens
 * at the run time, user has to provide this parameter in order to
 * request.
 *
 * gpio_in will be used in GTE to monitor the event and will be configured
 * as input.
 */
static int gpio_in = -EINVAL;
module_param(gpio_in, int, 0660);

/*
 * Same comment as gpio_in but will be used as output and to toggle gpio_in
 * state.
 */
static int gpio_out = -EINVAL;
module_param(gpio_out, int, 0660);

/* IRQ number to monitor */
static int lic_irq = -EINVAL;
module_param(lic_irq, int, 0660);

static struct tegra_gte_test {
	struct tegra_gte_ev_desc *data_lic;
	struct tegra_gte_ev_desc *data_gpio;
	int gpio_in_irq;
	struct timer_list timer;
	struct kobject *kobj;
} gte;

/*
 * Sysfs attribute to register/unregister GTE gpio event for 1 and 0 values
 */
static ssize_t store_gpio_en_dis(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int ret = count;
	unsigned long val = 0;
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "nvidia,tegra194-gte-aon");

	if (!np) {
		pr_err("Could not locate aon gte node\n");
		return -EINVAL;
	}

	if (kstrtoul(buf, 10, &val) < 0) {
		ret = -EINVAL;
		goto error;
	}

	if (val == 1) {
		if (gte.data_gpio) {
			pr_info("gpio_in is already registered\n");
			ret = -EEXIST;
			goto error;
		}
		gte.data_gpio = tegra_gte_register_event(np, gpio_in);
		if (IS_ERR(gte.data_gpio)) {
			pr_err("Could not register gpio\n");
			ret = PTR_ERR(gte.data_gpio);
			gte.data_gpio = NULL;
			goto error;
		}
	} else if (val == 0) {
		if (!gte.data_gpio) {
			pr_info("gpio_in is not registered\n");
			ret = -EINVAL;
			goto error;
		}
		ret = tegra_gte_unregister_event(gte.data_gpio);
		if (ret == -EBUSY) {
			/* User should retry */
			pr_err("failed to unregister gpio in\n");
			goto error;
		} else { /* For anything else set data to null */
			gte.data_gpio = NULL;
			if (ret == 0)
				ret = count;
		}
	} else {
		ret = -EINVAL;
	}

error:
	of_node_put(np);
	return ret;
}

/*
 * Sysfs attribute to register/unregister GTE LIC IRQ event for 1 and 0 values
 */
static ssize_t store_lic_irq_en_dis(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	int ret = count;
	unsigned long val = 0;
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, "nvidia,tegra194-gte-lic");

	if (!np) {
		pr_err("Could not locate lic gte node\n");
		return -EINVAL;
	}

	if (kstrtoul(buf, 10, &val) < 0) {
		ret = -EINVAL;
		goto error;
	}

	if (val == 1) {
		if (gte.data_lic) {
			pr_info("lic_irq is already registered\n");
			ret = -EEXIST;
			goto error;
		}
		gte.data_lic = tegra_gte_register_event(np, lic_irq);
		if (IS_ERR(gte.data_lic)) {
			pr_err("Could not register lic irq\n");
			ret = PTR_ERR(gte.data_lic);
			gte.data_lic = NULL;
			goto error;
		}
	} else if (val == 0) {
		if (!gte.data_lic) {
			pr_info("lic_irq is not registered\n");
			ret = -EINVAL;
			goto error;
		}
		ret = tegra_gte_unregister_event(gte.data_lic);
		if (ret == -EBUSY) {
			/* User should retry */
			pr_err("failed to unregister lic irq\n");
			goto error;
		} else { /* For anything else set data to null */
			gte.data_lic = NULL;
			if (ret == 0)
				ret = count;
		}
	} else {
		ret = -EINVAL;
	}

error:
	of_node_put(np);
	return ret;
}

/* Shows LIC event timestamp information */
static ssize_t show_lic_irq_ts(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   char *buf)
{
	struct tegra_gte_ev_detail hts;

	if (tegra_gte_retrieve_event((gte.data_lic), &hts) != 0)
		return -EINVAL;

	pr_debug("Retrieved lic event ts_raw: %llu, ts_ns %llu\n",
		 hts.ts_raw, hts.ts_ns);
	return scnprintf(buf, PAGE_SIZE, "ts_raw: %llu, ts_ns: %llu\n",
			 hts.ts_raw, hts.ts_ns);
}

struct kobj_attribute gpio_en_dis_attr =
		__ATTR(gpio_en_dis, 0220, NULL, store_gpio_en_dis);
struct kobj_attribute lic_irq_en_dis_attr =
		__ATTR(lic_irq_en_dis, 0220, NULL, store_lic_irq_en_dis);
struct kobj_attribute lic_irq_ts_attr =
		__ATTR(lic_irq_ts, 0440, show_lic_irq_ts, NULL);

static struct attribute *attrs[] = {
	&gpio_en_dis_attr.attr,
	&lic_irq_en_dis_attr.attr,
	&lic_irq_ts_attr.attr,
	NULL,
};

static struct attribute_group tegra_gte_test_attr_group = {
	.attrs = attrs,
};

static int tegra_gte_test_sysfs_create(void)
{
	int ret;

	/* Creates object under /sys/kernel/ */
	gte.kobj = kobject_create_and_add("tegra_gte_test", kernel_kobj);
	if (!gte.kobj)
		return -ENOMEM;

	ret = sysfs_create_group(gte.kobj, &tegra_gte_test_attr_group);
	if (ret)
		kobject_put(gte.kobj);
	return ret;
}

static void gpio_timer_cb(unsigned long data)
{
	gpio_set_value(gpio_out, !gpio_get_value(gpio_out));
	mod_timer(&gte.timer, jiffies + msecs_to_jiffies(5000));
}

static irqreturn_t tegra_gte_test_gpio_isr(int irq, void *data)
{
	struct tegra_gte_ev_detail hts;
	struct tegra_gte_test *gte = data;

	if (tegra_gte_retrieve_event((gte->data_gpio), &hts) != 0) {
		pr_info("No timestamp available\n");
		return IRQ_HANDLED;
	}

	pr_info("GPIO HW Timestamp: raw %llu, ns %llu\n",
		 hts.ts_raw, hts.ts_ns);
	return IRQ_HANDLED;
}

static int __init tegra_gte_test_init(void)
{
	int ret = 0;

	if (gpio_out == -EINVAL || gpio_in == -EINVAL || lic_irq == EINVAL) {
		pr_err("Invalid gpio_out, gpio_in and irq\n");
		return -EINVAL;
	}

	gte.data_lic = NULL;
	gte.data_gpio = NULL;

	ret = gpio_request(gpio_out, "gte_test_gpio_out");
	if (ret) {
		pr_err("failed request gpio out\n");
		return -EINVAL;
	}

	ret = gpio_direction_output(gpio_out, 0);
	if (ret) {
		pr_err("failed to set pin direction\n");
		ret = -EINVAL;
		goto free_gpio_out;
	}

	ret = gpio_request(gpio_in, "gte_test_gpio_in");
	if (ret) {
		pr_err("failed to request gpio in\n");
		ret = -EINVAL;
		goto free_gpio_out;
	}

	ret = gpio_direction_input(gpio_in);
	if (ret) {
		pr_err("failed to set pin direction\n");
		ret = -EINVAL;
		goto free_gpio_in;

	}

	/* IRQ setup */
	ret = gpio_to_irq(gpio_in);
	if (ret < 0) {
		pr_err("failed to map GPIO to IRQ: %d\n", ret);
		ret = -EINVAL;
		goto free_gpio_in;
	}

	gte.gpio_in_irq = ret;

	ret = request_irq(ret, tegra_gte_test_gpio_isr,
			 IRQF_TRIGGER_RISING | IRQF_NO_THREAD,
			 "tegra_gte_test_isr", &gte);
	if (ret) {
		pr_err("failed to acquire IRQ\n");
		ret = -EINVAL;
		goto free_gpio_in;
	}

	ret = tegra_gte_test_sysfs_create();
	if (ret != 0) {
		pr_err("sysfs creation failed\n");
		ret = -EINVAL;
		goto free_irq;
	}

	setup_timer(&gte.timer, gpio_timer_cb, 0);
	mod_timer(&gte.timer, jiffies + msecs_to_jiffies(5000));

	return 0;

free_irq:
	free_irq(gte.gpio_in_irq, &gte);
free_gpio_in:
	gpio_free(gpio_in);
free_gpio_out:
	gpio_free(gpio_out);

	return ret;
}

static void __exit tegra_gte_test_exit(void)
{
	free_irq(gte.gpio_in_irq, &gte);
	gpio_free(gpio_in);
	gpio_free(gpio_out);
	tegra_gte_unregister_event(gte.data_gpio);
	tegra_gte_unregister_event(gte.data_lic);
	kobject_put(gte.kobj);
	del_timer(&gte.timer);
}

module_init(tegra_gte_test_init);
module_exit(tegra_gte_test_exit);
MODULE_AUTHOR("Dipen Patel <dipenp@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra GTE driver test");
MODULE_LICENSE("GPL v2");
