/*
 * drivers/gpio/gpio-tmpm32xi2c.c
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#ifdef CONFIG_OF_GPIO
#include <linux/of_platform.h>
#endif
#include <linux/mfd/tmpm32xi2c.h>

#define TMPM_MAX_PORT		112
#define TMPM_MAX_INTR_PORT	8
#define TMPM_BANK_SZ		8

#define TMPM_MAX_BANK		(TMPM_MAX_PORT / TMPM_BANK_SZ)
#define TMPM_MAX_INTR_BANK	(TMPM_MAX_INTR_PORT / TMPM_BANK_SZ)

#if ((TMPM_MAX_PORT % TMPM_BANK_SZ) || (TMPM_MAX_INTR_PORT % TMPM_BANK_SZ))
#error "Please set TMPM_MAX_PORT with BANK size as a unit!"
#endif

#define TMPM_GET_BANK(_A, _pos)		(_A[(_pos) / TMPM_BANK_SZ])
#define TMPM_GET_BANKOFFSET(_pos)	((_pos) % TMPM_BANK_SZ)

#define TMPM_SET_BIT(_A, _pos) \
	(_A[(_pos) / TMPM_BANK_SZ] |= (1 << TMPM_GET_BANKOFFSET(_pos)))
#define TMPM_CLR_BIT(_A, _pos) \
	(_A[(_pos) / TMPM_BANK_SZ] &= ~(1 << TMPM_GET_BANKOFFSET(_pos)))
#define TMPM_TEST_BIT(_A, _pos) \
	(_A[(_pos) / TMPM_BANK_SZ] & (1 << TMPM_GET_BANKOFFSET(_pos)))

struct tmpm32xi2c_intr_map {
	/* index: intr-num, data: gpio-num */
	const uint32_t iidg[TMPM_MAX_INTR_PORT];
	/* index: gpio-num, data: intr-num */
	uint32_t igdi[TMPM_MAX_PORT];
};

/*
 * Currently MCU firmware supports only 8 interrupt sources including
 * one reserved.
 *
 * *---------------------------------------*
 * | intr source |      GPIO offset        |
 * *---------------------------------------*
 * |     0       | 24 - TMPM32X_GPIO(D, 0) |
 * |     1       | 25 - TMPM32X_GPIO(D, 1) |
 * |     2       | 35 - TMPM32X_GPIO(H, 6) |
 * |     3       | 27 - TMPM32X_GPIO(D, 3) |
 * |     4       | 28 - TMPM32X_GPIO(D, 4) |
 * |     5       | 29 - TMPM32X_GPIO(D, 5) |
 * |     6       | 81 - TMPM32X_GPIO(K, 1) |
 * |     7       | N/A                     |
 * *---------------------------------------*
 */
static struct tmpm32xi2c_intr_map intr_map = {
	{ 24, 25, 35, 27, 28, 29, 81, ~0 }, { ~0, }
};

#define TMPM_GET_GPIO_NUM(index) (intr_map.iidg[(index)])
#define TMPM_GET_INTR_NUM(index) (intr_map.igdi[(index)])

struct tmpm32xi2c_gpio_data {
	struct device *dev;
	struct gpio_chip gc;

	struct mutex lock;
	struct mutex irq_lock;
	int irq;
	unsigned long irq_flags;

	/* for interrupt */
	uint8_t irq_available[TMPM_MAX_BANK];
	uint8_t reg_direction_intr[TMPM_MAX_INTR_BANK];
	uint8_t irq_mask_cache[TMPM_MAX_INTR_BANK];
	uint8_t irq_mask[TMPM_MAX_INTR_BANK];
	uint8_t irq_trig_raise[TMPM_MAX_INTR_BANK];
	uint8_t irq_trig_fall[TMPM_MAX_INTR_BANK];

	uint8_t irq_pending[TMPM_MAX_BANK];
	uint8_t irq_is_pending;

	/* for direction output */
	uint8_t dir_output[TMPM_MAX_BANK];
	uint8_t dir_output_init[TMPM_MAX_BANK];
	uint8_t output_val[TMPM_MAX_BANK];
	uint8_t output_val_init[TMPM_MAX_BANK];
};

static void tmpm32xi2c_gpio_set_value(struct gpio_chip *gc, unsigned int offset,
				      int val);

static inline
struct tmpm32xi2c_gpio_data *gc_to_tmpm32xi2c_gpio(struct gpio_chip *gpio_chip)
{
	return container_of(gpio_chip, struct tmpm32xi2c_gpio_data, gc);
}

static int tmpm32xi2c_gpio_to_irq(struct gpio_chip *gc, unsigned int offset)
{
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);

	if (TMPM_TEST_BIT(data->irq_available, offset))
		return irq_find_mapping(gc->irqdomain, offset);

	dev_dbg(data->dev, "%s: offset[%u] is not available for irq\n",
		 __func__, offset);
	return 0;
}

static int __tmpm32xi2c_gpio_direction_input(struct tmpm32xi2c_gpio_data *data,
					     unsigned int offset)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint32_t intr_num;
	u8 tx_buf[] = { CMD_PIN_IN, 0 /*pin*/ };
	int update = 0;
	int ret;

	dev_dbg(data->dev, "%s: offset[%u]\n", __func__, offset);

	if (!TMPM_TEST_BIT(data->dir_output_init, offset))
		update = 1;
	else if (TMPM_TEST_BIT(data->dir_output, offset))
		update = 1;

	if (!update)
		return 0;

	tx_buf[1] = (u8)offset;
	ret = chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
	if (ret < 0)
		goto exit;

	TMPM_CLR_BIT(data->dir_output, offset);
	intr_num = TMPM_GET_INTR_NUM(offset);
	if (intr_num != ~0)
		TMPM_SET_BIT(data->reg_direction_intr, intr_num);
	TMPM_SET_BIT(data->dir_output_init, offset);

	ret = 0;

exit:
	return ret;
}

static int tmpm32xi2c_gpio_direction_input(struct gpio_chip *gc,
					   unsigned int offset)
{
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	int ret;

	mutex_lock(&data->lock);
	ret = __tmpm32xi2c_gpio_direction_input(data, offset);
	mutex_unlock(&data->lock);
	return ret;
}

static int __tmpm32xi2c_gpio_direction_output(struct tmpm32xi2c_gpio_data *data,
					      unsigned int offset, int val)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint32_t intr_num;
	u8 tx_buf[] = { CMD_PIN_OUT, 0 /*pin*/, 0 /*val*/ };
	int update = 0;
	int ret;

	dev_dbg(data->dev, "%s: offset[%u], val[%d]\n", __func__, offset, val);

	if (!TMPM_TEST_BIT(data->dir_output_init, offset))
		update = 1;
	else if (!TMPM_TEST_BIT(data->dir_output, offset))
		update = 1;

	if (!update)
		return 0;

	tx_buf[1] = (u8)offset;
	tx_buf[2] = val ? 1 : 0;

	/* set output level */
	ret = chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0);
	if (ret < 0)
		goto exit;

	TMPM_SET_BIT(data->dir_output, offset);
	intr_num = TMPM_GET_INTR_NUM(offset);
	if (intr_num != ~0)
		TMPM_CLR_BIT(data->reg_direction_intr, intr_num);
	TMPM_SET_BIT(data->dir_output_init, offset);
	ret = 0;

exit:
	return ret;
}

static int tmpm32xi2c_gpio_direction_output(struct gpio_chip *gc,
					    unsigned int offset, int val)
{
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	int ret;

	mutex_lock(&data->lock);
	ret = __tmpm32xi2c_gpio_direction_output(data, offset, val);
	mutex_unlock(&data->lock);

	/* WAR: CMD_PIN_OUT doesn't set the output value with latest F/W */
	tmpm32xi2c_gpio_set_value(gc, offset, val);

	return ret;
}

static int __tmpm32xi2c_gpio_get_value(struct tmpm32xi2c_gpio_data *data,
				       unsigned int offset)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	int ret;

	dev_dbg(data->dev, "%s: offset[%u]\n", __func__, offset);

	if (TMPM_TEST_BIT(data->dir_output, offset)) {
		ret = TMPM_TEST_BIT(data->output_val, offset) ? 1 : 0;
	} else {
		u8 tx_buf[] = { CMD_PIN_RD, 0 /*pin*/ };
		u8 rx_buf[] = { 0 /*val*/ };

		tx_buf[1] = (u8)offset;
		ret = chip->write_read(chip, tx_buf, sizeof(tx_buf),
				       rx_buf, sizeof(rx_buf));
		if (ret < 0)
			return 0;

		ret = rx_buf[0] ? 1 : 0;
	}

	return ret;
}

static int tmpm32xi2c_gpio_get_value(struct gpio_chip *gc, unsigned int offset)
{
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	int ret;

	mutex_lock(&data->lock);
	ret = __tmpm32xi2c_gpio_get_value(data, offset);
	mutex_unlock(&data->lock);

	return ret;
}

static void __tmpm32xi2c_gpio_set_value(struct tmpm32xi2c_gpio_data *data,
					unsigned int offset, int val)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	u8 tx_buf[] = { CMD_PIN_WR, 0 /*pin*/, 0 /*val*/};
	int update = 0;

	dev_dbg(data->dev, "%s: offset[%u], val[%d]\n", __func__, offset, val);

	if (!TMPM_TEST_BIT(data->output_val_init, offset))
		update = 1;
	else if (TMPM_TEST_BIT(data->output_val, offset) && !val)
		update = 1;
	else if (!TMPM_TEST_BIT(data->output_val, offset) && val)
		update = 1;

	if (!update)
		return;

	tx_buf[1] = (u8)offset;
	tx_buf[2] = val ? 1 : 0;

	if (chip->write_read(chip, tx_buf, sizeof(tx_buf), NULL, 0) < 0)
		return;

	if (val)
		TMPM_SET_BIT(data->output_val, offset);
	else
		TMPM_CLR_BIT(data->output_val, offset);
	TMPM_SET_BIT(data->output_val_init, offset);
}

static void tmpm32xi2c_gpio_set_value(struct gpio_chip *gc, unsigned int offset,
				      int val)
{
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);

	mutex_lock(&data->lock);
	__tmpm32xi2c_gpio_set_value(data, offset, val);
	mutex_unlock(&data->lock);
}

static int tmpm32xi2c_gpio_irq_pending(struct tmpm32xi2c_gpio_data *data)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint8_t *pending = &data->irq_pending[0];
	uint8_t tx_buf[] = { CMD_INT_REG, 0 /* dummy */ };
	uint8_t rx_buf[TMPM_MAX_INTR_BANK * 2] = { 0, };
	int pendings = 0;
	int i;
	int ret = -1;

	ret = chip->write_read(chip, tx_buf, sizeof(tx_buf),
			       rx_buf, sizeof(rx_buf));
	if (ret < 0)
		return 0;

	mutex_lock(&data->irq_lock);
	for (i = 0; i < TMPM_MAX_INTR_BANK; i++) {
		uint8_t cur_stat;
		uint8_t trig_raise, trig_fall;
		uint8_t irq_trig_raise, irq_trig_fall;

		cur_stat = (rx_buf[i * 2] & data->reg_direction_intr[i]) &
			   data->irq_mask_cache[i];
		trig_raise = cur_stat & rx_buf[i * 2 + 1];
		trig_fall = cur_stat & ~(rx_buf[i * 2 + 1]);
		irq_trig_raise = cur_stat & data->irq_trig_raise[i];
		irq_trig_fall = cur_stat & data->irq_trig_fall[i];

		if (!trig_raise && !trig_fall)
			continue;

		if (irq_trig_raise || irq_trig_fall) {
			/* Check pending irq based on configured irq type. */
			pending[i] = (trig_raise & irq_trig_raise) |
				     (trig_fall & irq_trig_fall);
		} else {
			/*
			 * If there was no configured irq type, consider any
			 * triggered irq types as pending irq.
			 */
			pending[i] = (trig_raise | trig_fall);
		}
		pendings += (pending[i] ? 1 : 0);
	}
	mutex_unlock(&data->irq_lock);

	return pendings;
}

static irqreturn_t tmpm32xi2c_gpio_irq_handler(int irq, void *devid)
{
	struct tmpm32xi2c_gpio_data *data =
				(struct tmpm32xi2c_gpio_data *)devid;
	unsigned int nhandled = 0;
	unsigned int gpio_irq = 0;
	uint8_t level;
	int pendings;
	int i;

	pendings = tmpm32xi2c_gpio_irq_pending(data);
	if (!data->irq_is_pending && !pendings)
		return IRQ_NONE;

	for (i = 0; i < TMPM_MAX_INTR_BANK; i++) {
		while (data->irq_pending[i]) {
			level = __ffs(data->irq_pending[i]);
			gpio_irq = irq_find_mapping(data->gc.irqdomain,
						    TMPM_GET_GPIO_NUM(level));
			handle_nested_irq(gpio_irq);
			data->irq_pending[i] &= ~(1 << level);
			nhandled++;
		}
	}
	data->irq_is_pending = 0;

	return (nhandled > 0) ? IRQ_HANDLED : IRQ_NONE;
}

static void
tmpm32xi2c_gpio_update_irq_mask_reg(struct tmpm32xi2c_gpio_data *data)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint8_t tx_mask[] = { CMD_INT_MASK, 0x0 /*value*/ };
	uint8_t irq_mask_cache;
	uint8_t level;
	uint8_t new_irqs;
	int i;

	for (i = 0; i < TMPM_MAX_INTR_BANK; i++) {
		irq_mask_cache = data->irq_mask_cache[i];
		new_irqs = ~irq_mask_cache & data->reg_direction_intr[i];

		while (new_irqs) {
			level = __ffs(new_irqs);
			mutex_lock(&data->lock);
			__tmpm32xi2c_gpio_direction_input(
					data, TMPM_GET_GPIO_NUM(level));
			mutex_unlock(&data->lock);
			new_irqs &= ~(1 << level);
		}

		if (irq_mask_cache != data->irq_mask[i]) {
			tx_mask[1] = data->irq_mask[i] = irq_mask_cache;
			chip->write_read(chip, tx_mask, sizeof(tx_mask),
					 NULL, 0);
		}
	}
}

static void tmpm32xi2c_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	uint32_t intr_num = TMPM_GET_INTR_NUM(d->hwirq);

	TMPM_CLR_BIT(data->irq_mask_cache, intr_num);
}

static void tmpm32xi2c_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	uint32_t intr_num = TMPM_GET_INTR_NUM(d->hwirq);

	TMPM_SET_BIT(data->irq_mask_cache, intr_num);
}

static void tmpm32xi2c_gpio_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);

	mutex_lock(&data->irq_lock);
}

static void tmpm32xi2c_gpio_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);

	tmpm32xi2c_gpio_update_irq_mask_reg(data);
	mutex_unlock(&data->irq_lock);
}

static int tmpm32xi2c_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct tmpm32xi2c_gpio_data *data = gc_to_tmpm32xi2c_gpio(gc);
	uint32_t intr_num;

	dev_dbg(data->dev, "%s: irq=%u, hwirq=%lu, type=%u\n",
		__func__, d->irq, d->hwirq, type);

	if (!TMPM_TEST_BIT(data->irq_available, d->hwirq))
		return 0;

	intr_num = TMPM_GET_INTR_NUM(d->hwirq);
	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	/* FIXME: LEVEL type is not supported yet in current MCU firmware. */
	if (!(type & IRQ_TYPE_EDGE_BOTH) && (type & IRQ_TYPE_LEVEL_MASK))
		return 0;

	if (type & IRQ_TYPE_EDGE_FALLING)
		TMPM_SET_BIT(data->irq_trig_fall, intr_num);
	else
		TMPM_CLR_BIT(data->irq_trig_fall, intr_num);

	if (type & IRQ_TYPE_EDGE_RISING)
		TMPM_SET_BIT(data->irq_trig_raise, intr_num);
	else
		TMPM_CLR_BIT(data->irq_trig_raise, intr_num);

	return 0;
}

static struct irq_chip tmpm32xi2c_gpio_irq_chip = {
	.name			= "tmpm32xi2c",
	.irq_mask		= tmpm32xi2c_gpio_irq_mask,
	.irq_unmask		= tmpm32xi2c_gpio_irq_unmask,
	.irq_bus_lock		= tmpm32xi2c_gpio_irq_bus_lock,
	.irq_bus_sync_unlock	= tmpm32xi2c_gpio_irq_bus_sync_unlock,
	.irq_set_type		= tmpm32xi2c_gpio_irq_set_type,
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};

static int tmpm32xi2c_gpio_irq_setup(struct tmpm32xi2c_gpio_data *data,
				     const struct platform_device_id *id)
{
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(data->dev->parent);
	uint32_t gpio_num = 0;
	unsigned long irq_flags;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(intr_map.iidg); i++) {
		gpio_num = TMPM_GET_GPIO_NUM(i);
		if (gpio_num != ~0) {
			TMPM_SET_BIT(data->irq_available, gpio_num);
			intr_map.igdi[gpio_num] = i;
		}
	}

	for (i = 0; i < TMPM_MAX_INTR_BANK; i++) {
		uint8_t tx_mask[] = { CMD_INT_MASK, 0x0 /*value*/ };

		data->irq_mask[i] = 0;
		ret = chip->write_read(chip, tx_mask, sizeof(tx_mask), NULL, 0);
		if (ret < 0) {
			dev_err(data->dev,
				"Failed to init interrupt mask, %d\n", ret);
			return ret;
		}
	}

	irq_flags = IRQF_ONESHOT | IRQF_SHARED | IRQF_EARLY_RESUME;
	ret = devm_request_threaded_irq(data->dev, data->irq, NULL,
					tmpm32xi2c_gpio_irq_handler,
					data->irq_flags | irq_flags,
					dev_name(data->dev), data);
	if (ret < 0) {
		dev_err(data->dev,
			"Failed to request irq%d, %d\n", data->irq, ret);
		return ret;
	}

	ret = gpiochip_irqchip_add(&data->gc, &tmpm32xi2c_gpio_irq_chip,
				   0, handle_simple_irq,  IRQ_TYPE_NONE);
	if (ret < 0) {
		dev_err(data->dev,
			"Failed to add irqchip to gpiochip, %d\n", ret);
		return ret;
	}

	return 0;
}

static int tmpm32xi2c_gpio_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id = platform_get_device_id(pdev);
	struct tmpm32xi2c_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct tmpm32xi2c_gpio_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct tmpm32xi2c_gpio_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	data->irq = chip->irq;
	data->irq_flags = chip->irq_flags;
	if (data->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ\n");
		return -EINVAL;
	}

	mutex_init(&data->lock);
	mutex_init(&data->irq_lock);

	data->gc.direction_input = tmpm32xi2c_gpio_direction_input;
	data->gc.direction_output = tmpm32xi2c_gpio_direction_output;
	data->gc.get = tmpm32xi2c_gpio_get_value;
	data->gc.set = tmpm32xi2c_gpio_set_value;
	data->gc.can_sleep = true;
	data->gc.base = -1;
	data->gc.ngpio = id->driver_data;
	data->gc.label = pdev->name;
	data->gc.parent = &pdev->dev;
	data->gc.owner = THIS_MODULE;
	data->gc.can_sleep = true;
#ifdef CONFIG_OF_GPIO
	data->gc.of_node = data->dev->parent->of_node;
#endif

	ret = devm_gpiochip_add_data(&pdev->dev, &data->gc, data);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add gpiochip data, %d\n", ret);
		return ret;
	}

	ret = tmpm32xi2c_gpio_irq_setup(data, id);
	if (ret < 0) {
		gpiochip_remove(&data->gc);
		return ret;
	}

	data->gc.to_irq = tmpm32xi2c_gpio_to_irq;

	device_init_wakeup(data->dev, true);

	return 0;
}

static int tmpm32xi2c_gpio_remove(struct platform_device *pdev)
{
	struct tmpm32xi2c_gpio_data *data = platform_get_drvdata(pdev);

	device_init_wakeup(data->dev, false);
	gpiochip_remove(&data->gc);
	mutex_destroy(&data->lock);
	mutex_destroy(&data->irq_lock);

	return 0;
}

#ifdef CONFIG_PM
static int tmpm32xi2c_gpio_suspend(struct device *dev)
{
	struct tmpm32xi2c_gpio_data *data = dev_get_drvdata(dev);
	int ret = 0;

	disable_irq(data->irq);

	if (device_may_wakeup(dev)) {
		ret = enable_irq_wake(data->irq);
		if (ret < 0)
			dev_err(dev,
				"Failed to enable irq wake for irq%d, %d\n",
				data->irq, ret);
	}

	return ret;
}

static int tmpm32xi2c_gpio_resume(struct device *dev)
{
	struct tmpm32xi2c_gpio_data *data = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(data->irq);

	enable_irq(data->irq);

	/* If there is any pending irq, invoke the irq handler. */
	if (tmpm32xi2c_gpio_irq_pending(data)) {
		local_irq_disable();
		data->irq_is_pending = 1;
		generic_handle_irq(data->irq);
		local_irq_enable();
	}

	return 0;
}

static const struct dev_pm_ops tmpm32xi2c_gpio_pm = {
	.suspend_late = tmpm32xi2c_gpio_suspend,
	.resume_early = tmpm32xi2c_gpio_resume,
};
#endif

static const struct platform_device_id tmpm32xi2c_gpio_id[] = {
	{ "tmpm32xi2c-gpio", TMPM_MAX_PORT, },
	{ }
};
MODULE_DEVICE_TABLE(platform, tmpm32xi2c_gpio_id);

static struct platform_driver tmpm32xi2c_gpio_driver = {
	.driver = {
		.name	= "tmpm32xi2c-gpio",
#ifdef CONFIG_PM
		.pm	= &tmpm32xi2c_gpio_pm,
#endif
	},
	.probe		= tmpm32xi2c_gpio_probe,
	.remove		= tmpm32xi2c_gpio_remove,
	.id_table	= tmpm32xi2c_gpio_id,
};

static int __init tmpm32xi2c_gpio_init(void)
{
	return platform_driver_register(&tmpm32xi2c_gpio_driver);
}
/*
 * register after postcore initcall and subsys initcall that may rely on
 * these GPIOs and I2C.
 */
subsys_initcall_sync(tmpm32xi2c_gpio_init);

static void __exit tmpm32xi2c_gpio_exit(void)
{
	platform_driver_unregister(&tmpm32xi2c_gpio_driver);
}
module_exit(tmpm32xi2c_gpio_exit);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("GPIO expander driver for TMPM32x I2C");
MODULE_LICENSE("GPL");
