/*
 * drivers/video/tegra/dc/nvdisplay/nvdisp_stub.c
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
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

/* Define stub functions to allow compilation */

#include <linux/types.h>
#include <linux/clk.h>
#include <linux/ioport.h>
#include <linux/clk/tegra.h>
#include <linux/of.h>
#include <soc/tegra/chip-id.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>
#include <linux/iommu.h>
#include <linux/memblock.h>
#include <linux/bootmem.h>

#include <linux/platform/tegra/latency_allowance.h>

#include "dc.h"
#include "board-panel.h"
#include "dc_priv.h"
#include <linux/platform_data/lp855x.h>
#include <soc/tegra/common.h>

__weak const struct disp_client *tegra_la_disp_clients_info;

int tegra_is_clk_enabled(struct clk *c)
{
	dump_stack();
	pr_info(" WARNING!!! OBSOLETE FUNCTION CALL!!! \
			DON'T USE %s FUNCTION \n", __func__);
	return 0;
}
EXPORT_SYMBOL(tegra_is_clk_enabled);

int nvdisp_register_backlight_notifier(struct tegra_dc *dc)
{
	return 0;
}

static int disp_fb_linear_set(void)
{
#if defined(CONFIG_OF_TEGRA_IOMMU_SMMU)
	tegra_fb_linear_set(NULL);
#endif
	return 0;
}
arch_initcall(disp_fb_linear_set);

