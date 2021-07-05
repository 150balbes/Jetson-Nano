/**
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/debugfs.h>
#include <linux/io.h>

#include "../../../../arch/arm/mach-tegra/iomap.h"

static u32 bwmgr_t210_iso_bw_table[] = { /* MHz */
	  5,  10,  20,  30,  40,  60,  80, 100, 120, 140,
	160, 180, 200, 250, 300, 350, 360, 370, 380, 400,
	450, 500, 550, 600, 650, 700
};

/* value of 1 indicates the range over max ISO Bw allowed for the DRAM type */
static u32 bwmgr_t210_lpddr4_iso_eff[] = {
	56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
	56,  56,  56,  56,  56,  56,  56,  56,  56,  56,
	56,  56,  56,  56,  49,  45
};

static u32 bwmgr_t210_lpddr3_iso_eff[] = {
	64,  64,  64,  64,  64,  64,  64,  64,  64,  64,
	64,  64,  64,  63,  60,  54,  45,   1,   1,   1,
	 1,   1,   1,   1,   1,   1
};

static u32 bwmgr_t210_ddr3_iso_eff[] = {
	65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
	65,  65,  65,  65,  65,  65,  65,  65,  65,  65,
	65,  65,  65,  65,  65,   1
};

#define EMC_FBIO_CFG5_0 0x104

#define DRAM_MASK 0x3
#define DRAM_DDR3 0
#define DRAM_LPDDR4 1
#define DRAM_LPDDR3 2 /* On T210 this value is LPDDR3 */

static int get_iso_bw_table_idx(unsigned long iso_bw)
{
	int i = ARRAY_SIZE(bwmgr_t210_iso_bw_table) - 1;

	/* Input is in Hz, iso_bw table's unit is MHz */
	iso_bw /= 1000000;

	while (i > 0 && bwmgr_t210_iso_bw_table[i] > iso_bw)
		i--;

	return i;
}

static unsigned long freq_to_bw(unsigned long freq)
{
	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR3_2CH ||
			bwmgr_dram_type == DRAM_TYPE_DDR3_2CH)
		return freq * 32;

	return freq * 16;
}

static unsigned long bw_to_freq(unsigned long bw)
{
	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR3_2CH ||
			bwmgr_dram_type == DRAM_TYPE_DDR3_2CH)
		return (bw + 32 - 1) / 32;

	return (bw + 16 - 1) / 16;
}

static u32 dvfs_latency(u32 ufreq)
{
	return 4;
}

static unsigned long t21x_bwmgr_apply_efficiency(
		unsigned long total_bw, unsigned long iso_bw,
		unsigned long max_rate, u64 usage_flags,
		unsigned long *iso_bw_min, unsigned long iso_bw_nvdis,
		unsigned long iso_bw_vi)
{
	u8 efficiency = bwmgr_dram_efficiency;

	if (total_bw && efficiency && (efficiency < 100)) {
		total_bw = total_bw / efficiency;
		total_bw = (total_bw < max_rate / 100) ?
			(total_bw * 100) : max_rate;
	}

	efficiency = bwmgr_dram_iso_eff_table[get_iso_bw_table_idx(iso_bw)];
	WARN_ON(efficiency == 1);
	if (iso_bw && efficiency && (efficiency < 100)) {
		iso_bw /= efficiency;
		iso_bw = (iso_bw < max_rate / 100) ?
		(iso_bw * 100) : max_rate;
	}

	if (iso_bw_min)
		*iso_bw_min = iso_bw;

	return max(total_bw, iso_bw);
}

static struct bwmgr_ops bwmgr_ops_t21x = {
	.freq_to_bw = freq_to_bw,
	.bw_to_freq = bw_to_freq,
	.dvfs_latency = dvfs_latency,
	.bwmgr_apply_efficiency = t21x_bwmgr_apply_efficiency,
};

struct bwmgr_ops *bwmgr_eff_init_t21x(void)
{
	int i;
	u32 dram;
	void __iomem *emc_base;

	emc_base = ioremap(TEGRA_T210_EMC_BASE, TEGRA_T210_EMC_SIZE);

	dram = readl(emc_base + EMC_FBIO_CFG5_0) & DRAM_MASK;

	iounmap(emc_base);

	switch (dram) {
	case DRAM_LPDDR4:
		bwmgr_dram_type = DRAM_TYPE_LPDDR4_2CH;
		bwmgr_dram_iso_eff_table = bwmgr_t210_lpddr4_iso_eff;
		bwmgr_dram_efficiency = 70;
		emc_to_dram_freq_factor = 2;
		/* valid ddr configuration */
		bwmgr_dram_config_supported = 1;

		break;

	case DRAM_LPDDR3:
		bwmgr_dram_type = DRAM_TYPE_LPDDR3_2CH;
		bwmgr_dram_efficiency = 80;
		bwmgr_dram_iso_eff_table = bwmgr_t210_lpddr3_iso_eff;
		emc_to_dram_freq_factor = 1;
		/* valid ddr configuration */
		bwmgr_dram_config_supported = 1;
		break;

	case DRAM_DDR3:
		bwmgr_dram_type = DRAM_TYPE_DDR3_2CH;
		bwmgr_dram_efficiency = 80;
		bwmgr_dram_iso_eff_table = bwmgr_t210_ddr3_iso_eff;
		emc_to_dram_freq_factor = 1;
		/* valid ddr configuration */
		bwmgr_dram_config_supported = 1;
		break;

	default:
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
	}
	bwmgr_dram_num_channels = 2;

	if (bwmgr_dram_config_supported) {
		for (i = ARRAY_SIZE(bwmgr_t210_iso_bw_table) - 1; i >= 0; i--) {
			if (bwmgr_dram_iso_eff_table[i] > 1) {
				bwmgr_iso_bw_percentage =
					bwmgr_dram_iso_eff_table[i];
				break;
			}
		}
	}

	return &bwmgr_ops_t21x;
}
