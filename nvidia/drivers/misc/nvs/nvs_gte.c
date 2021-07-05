/* Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVS_GTE
#define NVS_GTE				(0)
#endif

#define NVS_GTE_VERSION			(1)

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/nvs.h>
#include <linux/nvs_gte.h>


static void nvs_gte_exit_irq(struct nvs_gte_irq *ngi, unsigned int n)
{
	unsigned int i;
	unsigned int irq;
	unsigned int j;

	for (i = 0; i < n; i++) {
		if (ngi[i].irq > 0 && ngi[i].gpio >= 0) {
			for (j = 0; j < i; j++) {
				if (ngi[j].irq == ngi[i].irq)
					break;
			}
			if (j < i)
				/* IRQ already done */
				continue;

			irq = ngi[i].irq;
			free_irq(irq, &ngi[i]);
		}
	}

	for (i = 0; i < n; i++) {
		if (ngi[i].gpio >= 0)
			ngi[i].irq = -1;
	}

	return;
}

static void nvs_gte_exit_gpio(struct nvs_gte_irq *ngi, unsigned int n)
{
	unsigned int gpio;
	unsigned int i;
	unsigned int j;

	for (i = 0; i < n; i++) {
		if (ngi[i].gpio >= 0) {
			for (j = 0; j < i; j++) {
				if (ngi[j].gpio == ngi[i].gpio)
					break;
			}
			if (j < i)
				/* GPIO already done */
				continue;

			gpio = ngi[i].gpio;
			gpio_free(gpio);
		}
	}

	for (i = 0; i < n; i++)
		ngi[i].gpio = -1;

	return;
}

static int nvs_gte_init_gpio2irq(struct device *dev, struct nvs_gte_irq *ngi,
				 unsigned int n)
{
	unsigned int i;
	unsigned int j;
	int ret = -EINVAL;

	for (i = 0; i < n; i++) {
		if (ngi[i].irq > 0) {
			ret = 0;
			continue;
		}

		if (ngi[i].gpio < 0)
			continue;

		for (j = 0; j < i; j++) {
			if (ngi[j].gpio == ngi[i].gpio)
				break;
		}
		if (j < i) {
			/* GPIO already done */
			ngi[i].irq = ngi[j].irq;
			continue;
		}

		if (!gpio_is_valid(ngi[i].gpio)) {
			ret = -EPROBE_DEFER;
			goto nvs_gte_init_gpio_fie;
		}

		ret = gpio_request(ngi[i].gpio, ngi[i].dev_name);
		if (ret) {
			ret = -EPROBE_DEFER;
			goto nvs_gte_init_gpio_fie;
		}

		ret = gpio_direction_input(ngi[i].gpio);
		if (ret < 0) {
			dev_err(dev, "%s %s gpio_dir_input(%d) ERR:%d\n",
				__func__, ngi[i].dev_name, ngi[i].gpio, ret);
			nvs_gte_exit_irq(ngi, n);
			nvs_gte_exit_gpio(ngi, n);
			ret = -ENODEV;
			goto nvs_gte_init_gpio_fie;
		}

		ret = gpio_to_irq(ngi[i].gpio);
		if (ret <= 0) {
			dev_err(dev, "%s %s gpio_to_irq(%d) ERR:%d\n",
				__func__, ngi[i].dev_name, ngi[i].gpio, ret);
			nvs_gte_exit_irq(ngi, n);
			nvs_gte_exit_gpio(ngi, n);
			ret = -ENODEV;
			goto nvs_gte_init_gpio_fie;
		}

		ngi[i].irq = ret;
		ret = 0;
	}

nvs_gte_init_gpio_fie:
	return ret;
}


#if NVS_GTE


#include <linux/tegra-gte.h>

static char *nvs_gte_hw_str = "nvidia,tegra194-gte-aon";

int nvs_gte_ts(struct nvs_gte_irq *ngi)
{
	struct tegra_gte_ev_desc *desc = (struct tegra_gte_ev_desc *)ngi->gte;
	struct tegra_gte_ev_detail dtl;
	int ret;

	ret = tegra_gte_retrieve_event(desc, &dtl);
	if (ret)
		ngi->err_n++;
	else
		ngi->irq_ts = dtl.ts_ns;
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_gte_ts);

int nvs_gte_exit(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n)
{
	struct tegra_gte_ev_desc *desc;
	unsigned int i;
	int ret;
	int ret_t = 0;

	for (i = 0; i < n; i++) {
		if (ngi[i].gte) {
			desc = (struct tegra_gte_ev_desc *)ngi[i].gte;
			ret = tegra_gte_unregister_event(desc);
			if (ret) {
				ngi[i].gte = NULL;
				dev_err(dev,
					"%s gte_unregister_event ERR: %d\n",
					__func__, ret);
			} else {
				ret_t |= ret;
			}
		}
	}

	nvs_gte_exit_irq(ngi, n);
	nvs_gte_exit_gpio(ngi, n);
	return ret_t;
}
EXPORT_SYMBOL_GPL(nvs_gte_exit);

int nvs_gte_init(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n)
{
	struct tegra_gte_ev_desc *desc;
	unsigned int gpio;
	unsigned int i;
	int ret;
	/* Until the GTE API is improved we have to do this */
	struct device_node *np;
	np = of_find_compatible_node(NULL, NULL, nvs_gte_hw_str);

	ret = nvs_gte_init_gpio2irq(dev, ngi, n);
	if (ret == -EPROBE_DEFER)
		goto nvs_gte_init_fie;

	if (ret < 0) {
		nvs_gte_exit(dev, ngi, n);
		goto nvs_gte_init_fie;
	}

	ret = -ENODEV;
	for (i = 0; i < n; i++) {
		if (ngi[i].gpio >= 0 && !ngi[i].gte) {
			gpio = ngi[i].gpio;
			desc = tegra_gte_register_event(np, gpio);
			if (desc) {
				ngi[i].gte = (void *)desc;
				ret = 0;
			}
		}
	}

nvs_gte_init_fie:
	of_node_put(np);
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_gte_init);

int nvs_gte_sts(struct nvs_gte_irq *ngi, struct nvs_gte_sts *ngs)
{
	struct tegra_gte_ev_desc *desc = (struct tegra_gte_ev_desc *)ngi->gte;
	struct tegra_gte_ev_detail dtl;
	int ret;

	ngs->ver = NVS_GTE_VERSION;
	ngs->gte = nvs_gte_hw_str;
	ngs->ts_ns = 0;
	ngs->ts_raw = 0;
	if (desc) {
		ret = tegra_gte_retrieve_event(desc, &dtl);
		if (ret) {
			ngs->err = "HW read err";
		} else {
			ngs->err = NULL;
			ngs->ts_ns = dtl.ts_ns;
			ngs->ts_raw = dtl.ts_raw;
		}
	} else {
		ngs->err = "not initialized";
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_gte_sts);


#else /* !NVS_GTE ========================================================== */


int nvs_gte_ts(struct nvs_gte_irq *ngi)
{
	ngi->irq_ts = nvs_timestamp();
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_gte_ts);

int nvs_gte_exit(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n)
{
	nvs_gte_exit_irq(ngi, n);
	nvs_gte_exit_gpio(ngi, n);
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_gte_exit);

int nvs_gte_init(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n)
{
	int ret;

	ret = nvs_gte_init_gpio2irq(dev, ngi, n);
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_gte_init);

int nvs_gte_sts(struct nvs_gte_irq *ngi, struct nvs_gte_sts *ngs)
{
	ngs->ver = NVS_GTE_VERSION;
	ngs->gte = "disabled";
	ngs->err = NULL;
	ngs->ts_ns = 0;
	ngs->ts_raw = 0;
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_gte_sts);


#endif /* ! NVS_GTE */


MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor Generic Timestamp Engine module");
MODULE_AUTHOR("NVIDIA Corporation");

