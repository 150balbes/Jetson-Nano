/*
 * Tegra Wakeups for NVIDIA SoCs Tegra
 *
 * Copyright (c) 2013-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
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

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/irqchip/tegra.h>
#include <linux/system-wakeup.h>

int *tegra_gpio_wake_table;
EXPORT_SYMBOL_GPL(tegra_gpio_wake_table);

int *tegra_irq_wake_table;
EXPORT_SYMBOL_GPL(tegra_irq_wake_table);

int tegra_wake_table_len;
EXPORT_SYMBOL_GPL(tegra_wake_table_len);

void tegra_pm_update_gpio_wakeup_table(int base, int *gpio_wakeup_list,
				       int nlist)
{
	int i;

	for (i = 0; i < nlist; ++i) {
		if (gpio_wakeup_list[i] == -ENOENT)
			continue;

		if (gpio_wakeup_list[i] < 0) {
			tegra_gpio_wake_table[i] = gpio_wakeup_list[i];
		} else {
			tegra_gpio_wake_table[i] = base + gpio_wakeup_list[i];
			tegra_irq_wake_table[i] = -EAGAIN;
		}
	}
}

void tegra_irq_to_wake(int irq, int *wak_list, int *wak_size)
{
	int i;

	*wak_size = 0;
	for (i = 0; i < tegra_wake_table_len; i++) {
		if (tegra_irq_wake_table[i] == irq) {
			pr_info("Wake%d for irq=%d\n", i, irq);
			wak_list[*wak_size] = i;
			*wak_size = *wak_size + 1;
		}
	}
}

int tegra_wake_to_gpio(int wake)
{
	if (wake < 0 || wake >= tegra_wake_table_len)
		return -EINVAL;

	return tegra_gpio_wake_table[wake];
}

int tegra_wake_to_irq(int wake)
{
	int ret;

	if (wake < 0)
		return -EINVAL;

	if (wake >= tegra_wake_table_len)
		return -EINVAL;

	ret = tegra_irq_wake_table[wake];
	if (ret == -EAGAIN) {
		ret = tegra_gpio_wake_table[wake];
		if (ret != -EINVAL)
			ret = gpio_to_irq(ret);
	}

	return ret;
}

int get_wakeup_reason_irq(void)
{
	int irq;
	struct irq_desc *desc;
	int i, j;
	u32 wake_status[3];
	int len;

	len = tegra_read_wake_status(wake_status);

	for (i = 0; i < len; i++) {
		unsigned long temp = wake_status[i];
		for_each_set_bit(j, &temp, 32) {
			irq = tegra_wake_to_irq(j + (i * 32));
			if (!irq)
				continue;
			desc = irq_to_desc(irq);
			if (!desc || !desc->action || !desc->action->name)
				continue;
			return irq;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(get_wakeup_reason_irq);
