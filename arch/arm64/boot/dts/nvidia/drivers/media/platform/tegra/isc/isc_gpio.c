/*
 * ISC GPIO driver
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/nospec.h>

#include "isc-gpio-priv.h"

#define MAX_STR_SIZE 255

static int of_isc_gpio_pdata(struct platform_device *pdev,
				struct isc_gpio_plat_data *pdata) {
	struct device_node *np = pdev->dev.of_node;
	int err;

	err = of_property_read_string(np, "parent-gpio-chip",
					&pdata->gpio_prnt_chip);
	if (err < 0)
		return err;

	err = of_property_read_u32(pdev->dev.of_node,
					"max-gpios", &pdata->max_gpio);

	return err;
}

static int isc_gpio_chip_match(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static struct gpio_chip *isc_gpio_get_chip(struct platform_device *pdev,
						struct isc_gpio_plat_data *pd)
{
	struct gpio_chip *gc = NULL;
	char name[MAX_STR_SIZE];

	if (strlen(pd->gpio_prnt_chip) > MAX_STR_SIZE) {
		dev_err(&pdev->dev, "%s: gpio chip name is too long: %s\n",
			__func__, pd->gpio_prnt_chip);
		return NULL;
	}
	strcpy(name, pd->gpio_prnt_chip);

	gc = gpiochip_find(name, isc_gpio_chip_match);
	if (!gc) {
		dev_err(&pdev->dev, "%s: unable to find gpio parent chip %s\n",
			__func__, pd->gpio_prnt_chip);
		return NULL;
	}

	return gc;
}

static int isc_gpio_init_desc(struct platform_device *pdev,
				struct isc_gpio_priv *isc_gpio)
{
	struct isc_gpio_desc *desc = NULL;
	u32 i;

	desc = devm_kzalloc(&pdev->dev,
			(sizeof(struct isc_gpio_desc) *
				isc_gpio->pdata.max_gpio),
			GFP_KERNEL);
	if (!desc) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	for (i = 0; i < isc_gpio->pdata.max_gpio; i++) {
		desc[i].gpio = 0;
		atomic_set(&desc[i].ref_cnt, 0);
	}

	isc_gpio->gpios = desc;
	return 0;
}

static int isc_gpio_get_index(struct device *dev,
				struct isc_gpio_priv *isc_gpio, u32 gpio)
{
	u32 i;
	int idx = -1;

	/* find gpio in array */
	for (i = 0; i < isc_gpio->num_gpio; i++) {
		if (isc_gpio->gpios[i].gpio == gpio) {
			idx = i;
			break;
		}
	}

	/* gpio exists, return idx */
	if (idx >= 0)
		return idx;

	/* add gpio if it doesn't exist and there is memory available */
	if (isc_gpio->num_gpio < isc_gpio->pdata.max_gpio) {
		idx = isc_gpio->num_gpio;
		isc_gpio->gpios[idx].gpio = gpio;
		isc_gpio->num_gpio++;

		return idx;
	}

	dev_err(dev, "%s: Unable to add gpio to desc\n", __func__);
	return -EFAULT;
}

static int isc_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct gpio_chip *tgc = NULL;
	struct isc_gpio_priv *isc_gpio = NULL;
	int err;

	isc_gpio = gpiochip_get_data(gc);
	if (!isc_gpio)
		return -EFAULT;

	mutex_lock(&isc_gpio->mutex);

	tgc = isc_gpio->tgc;
	err = tgc->direction_input(tgc, off);

	mutex_unlock(&isc_gpio->mutex);

	return err;
}

static int isc_gpio_direction_output(struct gpio_chip *gc, unsigned off,
					int val)
{
	struct gpio_chip *tgc = NULL;
	struct isc_gpio_priv *isc_gpio = NULL;
	int err;

	isc_gpio = gpiochip_get_data(gc);
	if (!isc_gpio)
		return -EFAULT;

	mutex_lock(&isc_gpio->mutex);

	tgc = isc_gpio->tgc;
	err = tgc->direction_output(tgc, off, val);

	mutex_unlock(&isc_gpio->mutex);

	return err;
}

static int isc_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	int gpio_val;
	struct gpio_chip *tgc = NULL;
	struct isc_gpio_priv *isc_gpio = NULL;

	isc_gpio = gpiochip_get_data(gc);
	if (!isc_gpio)
		return -EFAULT;

	mutex_lock(&isc_gpio->mutex);

	tgc = isc_gpio->tgc;
	gpio_val = tgc->get(tgc, off);

	mutex_unlock(&isc_gpio->mutex);

	return gpio_val;
}

static void isc_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	int idx;
	struct gpio_chip *tgc = NULL;
	struct isc_gpio_priv *isc_gpio = NULL;
	atomic_t *ref_cnt;
	struct device *dev = NULL;

	isc_gpio = gpiochip_get_data(gc);
	if (!isc_gpio)
		return;

	mutex_lock(&isc_gpio->mutex);

	dev = isc_gpio->pdev;
	tgc = isc_gpio->tgc;

	idx = isc_gpio_get_index(dev, isc_gpio, off);
	if (idx < 0) {
		mutex_unlock(&isc_gpio->mutex);
		return;
	}
	idx = array_index_nospec(idx, 0);

	/* set gpio value based on refcount */
	ref_cnt = &isc_gpio->gpios[idx].ref_cnt;
	switch (val) {
	case 0:
		if ((atomic_read(ref_cnt) > 0) &&
			atomic_dec_and_test(ref_cnt)) {
			tgc->set(tgc, off, val);
		}
		dev_info(dev, "%s: gpio idx: %d, val to set: %d, refcount: %d\n",
			__func__, idx, val, atomic_read(ref_cnt));
		break;
	case 1:
		if (!atomic_inc_and_test(ref_cnt))
			tgc->set(tgc, off, val);

		dev_info(dev, "%s: gpio idx: %d, val to set: %d, refcount: %d\n",
			__func__, idx, val, atomic_read(ref_cnt));
		break;
	default:
		dev_err(dev, "%s: Invalid gpio value provided\n",
		__func__);
		break;
	}

	mutex_unlock(&isc_gpio->mutex);
}

static int isc_gpio_probe(struct platform_device *pdev)
{
	struct isc_gpio_priv *isc_gpio;
	struct isc_gpio_plat_data *pd = NULL;
	struct gpio_chip *tgc, *gc;
	int err;

	dev_info(&pdev->dev, "probing %s...\n", __func__);

	isc_gpio = devm_kzalloc(&pdev->dev,
				sizeof(struct isc_gpio_priv),
				GFP_KERNEL);
	if (!isc_gpio) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	/* get platform data from device tree */
	err = of_isc_gpio_pdata(pdev, &isc_gpio->pdata);
	if (err < 0)
		return err;

	pd = &isc_gpio->pdata;

	/* get tegra gpio chip */
	tgc = isc_gpio_get_chip(pdev, pd);
	if (!tgc)
		return -ENXIO;

	isc_gpio->tgc = tgc;

	/* initialize gpio desc */
	err = isc_gpio_init_desc(pdev, isc_gpio);
	if (err < 0)
		return err;

	isc_gpio->num_gpio = 0;

	/* setup gpio chip */
	gc = &isc_gpio->gpio_chip;
	gc->direction_input  = isc_gpio_direction_input;
	gc->direction_output = isc_gpio_direction_output;
	gc->get = isc_gpio_get_value;
	gc->set = isc_gpio_set_value;

	gc->can_sleep = false;
	gc->base = -1;
	gc->ngpio = pd->max_gpio;
	gc->label = "isc-gpio";
	gc->of_node = pdev->dev.of_node;
	gc->owner = THIS_MODULE;

	err = gpiochip_add_data(gc, isc_gpio);
	if (err) {
		dev_err(&pdev->dev, "failed to add GPIO controller\n");
		return err;
	}

	mutex_init(&isc_gpio->mutex);
	isc_gpio->pdev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, isc_gpio);

	dev_info(&pdev->dev, "%s: successfully registered gpio device\n",
			__func__);
	return 0;
}

static int isc_gpio_remove(struct platform_device *pdev)
{
	struct isc_gpio_priv *isc_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&isc_gpio->gpio_chip);

	return 0;
}

static const struct of_device_id isc_gpio_dt_ids[] = {
	{ .compatible = "nvidia,isc-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, isc_gpio_dt_ids);

static struct platform_driver isc_gpio_driver = {
	.probe = isc_gpio_probe,
	.remove = isc_gpio_remove,
	.driver = {
		.name = "isc-gpio",
		.of_match_table = isc_gpio_dt_ids,
		.owner = THIS_MODULE,
	}
};

static int __init isc_gpio_init(void)
{
	return platform_driver_register(&isc_gpio_driver);
}

static void __exit isc_gpio_exit(void)
{
	platform_driver_unregister(&isc_gpio_driver);
}

/* call in subsys so that this module loads before isc-mgr driver */
subsys_initcall(isc_gpio_init);
module_exit(isc_gpio_exit);

MODULE_DESCRIPTION("Tegra Auto ISC GPIO Driver");
MODULE_AUTHOR("Anurag Dosapati <adosapati@nvidia.com>");
MODULE_LICENSE("GPL v2");
