/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irqchip/tegra.h>

#include "iomap.h"

#define MAX_WAKE_ENTRIES	96

#define INT_OFFSET	32

#define INT_AOTAG2PMC  279 + INT_OFFSET
#define INT_RTC        10 + INT_OFFSET
#define INT_AOVC       215 + INT_OFFSET
#define INT_AOWDT      18 + INT_OFFSET
#define INT_XUSB       167 + INT_OFFSET
#define INT_SW_WAKE_TRIGGER 19 + INT_OFFSET
#define INT_AOVIC_FIQ	21 + INT_OFFSET
#define INT_AOVIC_IRQ   22 + INT_OFFSET
#define INT_AON_GPIO_0 60 + INT_OFFSET
#define INT_AON_GPIO_1 61 + INT_OFFSET
#define INT_VFMON      280 + INT_OFFSET
#define INT_AOPM       26 + INT_OFFSET
#define INT_PMC2LIC    211 + INT_OFFSET
#define INT_AO_DEBUG_WAKE 29 + INT_OFFSET
#define INT_AOPM2LIC   30 + INT_OFFSET
#define INT_AON_CAR    226 + INT_OFFSET
#define INT_SPE_WDT_EXPIRY 15 + INT_OFFSET
#define INT_EXTERNAL_PMU 209 + INT_OFFSET

#define T19X_INT_XUSB			(167 + INT_OFFSET)
#define T19X_INT_VFMON			(280 + INT_OFFSET)
#define T19X_INT_AOPM2LIC		(216 + INT_OFFSET)
#define T19X_INT_SPE_WAKEON_IPC		(133 + INT_OFFSET)

static int tegra_gpio_wakes[MAX_WAKE_ENTRIES];

static int tegra19_wake_event_irq[] = {
	-EAGAIN,			/* wake0 */
	-EAGAIN,			/* wake1 */
	-EAGAIN,			/* wake2 */
	-EAGAIN,			/* wake3 */
	-EAGAIN,			/* wake4 */
	-EAGAIN,			/* wake5 */
	-EINVAL,			/* wake6 */
	-EAGAIN,			/* wake7 */
	-EAGAIN,			/* wake8 */
	-EAGAIN,			/* wake9 */
	-EAGAIN,			/* wake10 */
	-EAGAIN,			/* wake11 */
	-EAGAIN,			/* wake12 */
	-EAGAIN,			/* wake13 */
	-EAGAIN,			/* wake14 */
	-EAGAIN,			/* wake15 */
	-EAGAIN,			/* wake16 */
	-EAGAIN,			/* wake17 */
	-EAGAIN,			/* wake18 */
	-EAGAIN,			/* wake19 */
	-EAGAIN,			/* wake20 */
	-EAGAIN,			/* wake21 */
	-EAGAIN,			/* wake22 */
	-EAGAIN,			/* wake23 */
	INT_EXTERNAL_PMU,		/* wake24 */
	-EAGAIN,			/* wake25 */
	-EAGAIN,			/* wake26 */
	-EAGAIN,			/* wake27 */
	-EAGAIN,			/* wake28 */
	-EAGAIN,			/* wake29 */
	-EAGAIN,			/* wake30 */
	-EAGAIN,			/* wake31 */
	-EAGAIN,			/* wake32 */
	-EAGAIN,			/* wake33 */
	-EAGAIN,			/* wake34 */
	-EAGAIN,			/* wake35 */
	-EAGAIN,			/* wake36 */
	-EAGAIN,			/* wake37 */
	-EAGAIN,			/* wake38 */
	-EAGAIN,			/* wake39 */
	-EAGAIN,			/* wake40 */
	-EAGAIN,			/* wake41 */
	-EAGAIN,			/* wake42 */
	-EAGAIN,			/* wake43 */
	-EAGAIN,			/* wake44 */
	-EAGAIN,			/* wake45 */
	-EAGAIN,			/* wake46 */
	-EAGAIN,			/* wake47 */
	-EAGAIN,			/* wake48 */
	-EAGAIN,			/* wake49 */
	-EAGAIN,			/* wake50 */
	-EAGAIN,			/* wake51 */
	-EAGAIN,			/* wake52 */
	-EAGAIN,			/* wake53 */
	-EAGAIN,			/* wake54 */
	-EAGAIN,			/* wake55 */
	-EAGAIN,			/* wake56 */
	-EAGAIN,			/* wake57 */
	-EAGAIN,			/* wake58 */
	-EAGAIN,			/* wake59 */
	-EAGAIN,			/* wake60 */
	-EAGAIN,			/* wake61 */
	-EAGAIN,			/* wake62 */
	-EAGAIN,			/* wake63 */
	-EAGAIN,			/* wake64 */
	-EAGAIN,			/* wake65 */
	-EAGAIN,			/* wake66 */
	-EAGAIN,			/* wake67 */
	-EAGAIN,			/* wake68 */
	-EAGAIN,			/* wake69 */
	-EAGAIN,			/* wake70 */
	-EAGAIN,			/* wake71 */
	INT_AOTAG2PMC,			/* wake72 */
	INT_RTC,			/* wake73 */
	INT_AOVC,			/* wake74 */
	-EAGAIN,			/* wake75 */
	T19X_INT_XUSB,			/* wake76 */
	T19X_INT_XUSB,			/* wake77 */
	T19X_INT_XUSB,			/* wake78 */
	T19X_INT_XUSB,			/* wake79 */
	T19X_INT_XUSB,			/* wake80 */
	T19X_INT_XUSB,			/* wake81 */
	T19X_INT_XUSB,			/* wake82 */
	-EAGAIN,			/* wake83 */
	INT_SPE_WDT_EXPIRY,		/* wake84 */
	-EAGAIN,			/* wake85 */
	-EAGAIN,			/* wake86 */
	-EAGAIN,			/* wake87 */
	-EAGAIN,			/* wake88 */
	T19X_INT_VFMON,			/* wake89 */
	-EAGAIN,			/* wake90 */
	T19X_INT_SPE_WAKEON_IPC,	/* wake91 */
	INT_PMC2LIC,			/* wake92 */
	-EAGAIN,			/* wake93 */
	T19X_INT_AOPM2LIC,		/* wake94 */
	INT_AON_CAR,			/* wake95 */
};

static int tegra_wake_event_irq[] = {
	-EAGAIN,		/* wake0 */
	-EAGAIN,		/* wake1 */
	-EAGAIN,		/* wake2 */
	-EAGAIN,		/* wake3 */
	-EAGAIN,		/* wake4 */
	-EAGAIN,		/* wake5 */
	-EINVAL,		/* wake6 */
	-EAGAIN,		/* wake7 */
	-EAGAIN,		/* wake8 */
	-EAGAIN,		/* wake9 */
	-EAGAIN,		/* wake10 */
	-EAGAIN,		/* wake11 */
	-EAGAIN,		/* wake12 */
	-EAGAIN,		/* wake13 */
	-EAGAIN,		/* wake14 */
	-EAGAIN,		/* wake15 */
	-EAGAIN,		/* wake16 */
	-EAGAIN,		/* wake17 */
	-EAGAIN,		/* wake18 */
	-EAGAIN,		/* wake19 */
	-EAGAIN,		/* wake20 */
	-EAGAIN,		/* wake21 */
	-EAGAIN,		/* wake22 */
	-EAGAIN,		/* wake23 */
	INT_EXTERNAL_PMU,	/* wake24 */
	-EAGAIN,		/* wake25 */
	-EAGAIN,		/* wake26 */
	-EAGAIN,		/* wake27 */
	-EAGAIN,		/* wake28 */
	-EAGAIN,		/* wake29 */
	-EAGAIN,		/* wake30 */
	-EAGAIN,		/* wake31 */
	-EAGAIN,		/* wake32 */
	-EAGAIN,		/* wake33 */
	-EAGAIN,		/* wake34 */
	-EAGAIN,		/* wake35 */
	-EAGAIN,		/* wake36 */
	-EAGAIN,		/* wake37 */
	-EAGAIN,		/* wake38 */
	-EAGAIN,		/* wake39 */
	-EAGAIN,		/* wake40 */
	-EAGAIN,		/* wake41 */
	-EAGAIN,		/* wake42 */
	-EAGAIN,		/* wake43 */
	-EAGAIN,		/* wake44 */
	-EAGAIN,		/* wake45 */
	-EAGAIN,		/* wake46 */
	-EAGAIN,		/* wake47 */
	-EAGAIN,		/* wake48 */
	-EAGAIN,		/* wake49 */
	-EAGAIN,		/* wake50 */
	-EAGAIN,		/* wake51 */
	-EAGAIN,		/* wake52 */
	-EAGAIN,		/* wake53 */
	-EAGAIN,		/* wake54 */
	-EAGAIN,		/* wake55 */
	-EAGAIN,		/* wake56 */
	-EAGAIN,		/* wake57 */
	-EAGAIN,		/* wake58 */
	-EAGAIN,		/* wake59 */
	-EAGAIN,		/* wake60 */
	-EAGAIN,		/* wake61 */
	-EAGAIN,		/* wake62 */
	-EAGAIN,		/* wake63 */
	-EAGAIN,		/* wake64 */
	-EAGAIN,		/* wake65 */
	-EAGAIN,		/* wake66 */
	-EAGAIN,		/* wake67 */
	-EAGAIN,		/* wake68 */
	-EAGAIN,		/* wake69 */
	-EAGAIN,		/* wake70 */
	-EAGAIN,		/* wake71 */
	INT_AOTAG2PMC,		/* wake72 */
	INT_RTC,		/* wake73 */
	INT_AOVC,		/* wake74 */
	INT_AOWDT,		/* wake75 */
	INT_XUSB,		/* wake76 */
	INT_XUSB,		/* wake77 */
	INT_XUSB,		/* wake78 */
	INT_XUSB,		/* wake79 */
	INT_XUSB,		/* wake80 */
	INT_XUSB,		/* wake81 */
	INT_XUSB,		/* wake82 */
	INT_SW_WAKE_TRIGGER,	/* wake83 */
	INT_SPE_WDT_EXPIRY,	/* wake84 */
	INT_AOVIC_FIQ,		/* wake85 */
	INT_AOVIC_IRQ,		/* wake86 */
	INT_AON_GPIO_0,		/* wake87 */
	INT_AON_GPIO_1,		/* wake88 */
	INT_VFMON,		/* wake89 */
	INT_AOPM,		/* wake90 */
	-EINVAL,		/* wake91 */
	INT_PMC2LIC,		/* wake92 */
	INT_AO_DEBUG_WAKE,	/* wake93 */
	INT_AOPM2LIC,		/* wake94 */
	INT_AON_CAR,		/* wake95 */
};

struct tegra_wakeup_soc {
	void (*init_func)(void);
};

static void tegra18x_wakeup_table_init(void)
{
	tegra_gpio_wake_table = tegra_gpio_wakes;
	tegra_irq_wake_table = tegra_wake_event_irq;
	tegra_wake_table_len = ARRAY_SIZE(tegra_gpio_wakes);
}

static void tegra19x_wakeup_table_init(void)
{
	tegra_gpio_wake_table = tegra_gpio_wakes;
	tegra_irq_wake_table = tegra19_wake_event_irq;
	tegra_wake_table_len = ARRAY_SIZE(tegra_gpio_wakes);
}

static const struct tegra_wakeup_soc t18x_wakeup_soc = {
	.init_func = tegra18x_wakeup_table_init,
};

static const struct tegra_wakeup_soc t19x_wakeup_soc = {
	.init_func = tegra19x_wakeup_table_init,
};

static const struct of_device_id tegra_wakeup_match[] = {
	{ .compatible = "nvidia,tegra186", .data = &t18x_wakeup_soc},
	{ .compatible = "nvidia,tegra194", .data = &t19x_wakeup_soc},
	{ }
};

int __init tegra_wakeup_table_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct tegra_wakeup_soc *soc_data;
	int i;

	for (i = 0; i < MAX_WAKE_ENTRIES; i++)
		tegra_gpio_wakes[i] = -EINVAL;

	np = of_find_matching_node_and_match(NULL, tegra_wakeup_match, &match);
	if (np) {
		soc_data = (struct tegra_wakeup_soc *)match->data;
		soc_data->init_func();
	}

	return 0;
}
