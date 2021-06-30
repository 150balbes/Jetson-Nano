/*
 * Copyright (C) 2010-2012 Google, Inc.
 * Copyright (C) 2013-2018, NVIDIA Corporation.  All rights reserved.
 *
 * Author:
 *	Erik Gilling <konkers@google.com>
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

#ifndef __MACH_TEGRA_MC_H
#define __MACH_TEGRA_MC_H

#include <linux/platform/tegra/tegra_mc.h>

#define MC_MAX_INTR_COUNT	32
#define MC_MAX_CHANNELS		16
#define MC_MAX_MSSNVLINK_HUBS	4

#define MC_BROADCAST_CHANNEL	-1

extern int mc_channels;
extern int mc_err_channel;
extern int mc_intstatus_reg;
extern unsigned int mssnvlink_hubs;

struct mc_client {
	const char *name;
	const char *swgroup;
	const int swgid;
	unsigned int intr_counts[MC_MAX_INTR_COUNT];
};

extern void __iomem *mc;
extern void __iomem *mc_regs[MC_MAX_CHANNELS];
extern void __iomem *mssnvlink_regs[MC_MAX_MSSNVLINK_HUBS];

#include <linux/io.h>
#include <linux/debugfs.h>

#include <soc/tegra/chip-id.h>

/**
 * Check if the MC has more than 1 channel.
 */
static inline int mc_multi_channel(void)
{
	return mc_channels > 1;
}

/**
 * Read from the MC.
 *
 * @idx The MC channel to read from.
 * @reg The offset of the register to read.
 *
 * Read from the specified MC channel: 0 -> MC0, 1 -> MC1, etc. If @idx
 * corresponds to a non-existent channel then 0 is returned.
 */
static inline u32 __mc_readl(int idx, u32 reg)
{
	if (WARN(!mc, "Read before MC init'ed"))
		return 0;

	if ((idx != MC_BROADCAST_CHANNEL && idx < 0) || idx > mc_channels)
		return 0;

	if (idx == MC_BROADCAST_CHANNEL)
		return readl(mc + reg);
	else
		return readl(mc_regs[idx] + reg);
}

/**
 * Write to the MC.
 *
 * @idx The MC channel to write to.
 * @val Value to write.
 * @reg The offset of the register to write.
 *
 * Write to the specified MC channel: 0 -> MC0, 1 -> MC1, etc. For writes there
 * is a special channel, %MC_BROADCAST_CHANNEL, which writes to all channels. If
 * @idx corresponds to a non-existent channel then the write is dropped.
 */
static inline void __mc_writel(int idx, u32 val, u32 reg)
{
	if (WARN(!mc, "Write before MC init'ed"))
		return;

	if ((idx != MC_BROADCAST_CHANNEL && idx < 0) ||
	    idx > mc_channels)
		return;

	if (idx == MC_BROADCAST_CHANNEL)
		writel(val, mc + reg);
	else
		writel(val, mc_regs[idx] + reg);
}

static inline u32 __mc_raw_readl(int idx, u32 reg)
{
	if (WARN(!mc, "Read before MC init'ed"))
		return 0;

	if ((idx != MC_BROADCAST_CHANNEL && idx < 0) || idx > mc_channels)
		return 0;

	if (idx == MC_BROADCAST_CHANNEL)
		return __raw_readl(mc + reg);
	else
		return __raw_readl(mc_regs[idx] + reg);
}

static inline void __mc_raw_writel(int idx, u32 val, u32 reg)
{
	if (WARN(!mc, "Write before MC init'ed"))
		return;

	if ((idx != MC_BROADCAST_CHANNEL && idx < 0) ||
	    idx > mc_channels)
		return;

	if (idx == MC_BROADCAST_CHANNEL)
		__raw_writel(val, mc + reg);
	else
		__raw_writel(val, mc_regs[idx] + reg);
}

#define mc_readl(reg)       __mc_readl(MC_BROADCAST_CHANNEL, reg)
#define mc_writel(val, reg) __mc_writel(MC_BROADCAST_CHANNEL, val, reg)

/**
 * Read from the MSSNVLINK hub.
 *
 * @idx The mssnvlink hub ID
 * @reg The register offset
 *
 * Read from the specified MSSNVLINK hub. Reads from
 * non-existent hubs return 0.
 */
static inline u32 mssnvlink_readl(unsigned int idx, u32 reg)
{
	if (mssnvlink_hubs == UINT_MAX)
		return 0;

	if (idx >= mssnvlink_hubs) {
		WARN(1, "mssnvlink read: invalid hub ID - %u\n", idx);
		return 0;
	}

	if (WARN(!mssnvlink_regs[idx], "Read before MSSNVLINK is initialized."))
		return 0;

	return readl(mssnvlink_regs[idx] + reg);
}

/**
 * Write to the MSSNVLINK hub.
 *
 * @idx ID of MSSNVLINK hub.
 * @val Value to write.
 * @reg Register offset to write.
 *
 * Write to the specified MSSNVLINK hub. Writes to
 * non-existent hubs are dropped.
 */

static inline void mssnvlink_writel(unsigned int idx, u32 val, u32 reg)
{
	if (mssnvlink_hubs == UINT_MAX)
		return;

	if (idx >= mssnvlink_hubs) {
		WARN(1, "mssnvlink write: invalid hub ID - %u\n", idx);
		return;
	}

	if (WARN(!mssnvlink_regs[idx], "Write before MSSNVLINK is initialized."))
		return;

	writel(val, mssnvlink_regs[idx] + reg);
}

unsigned long tegra_emc_bw_to_freq_req(unsigned long bw);
unsigned long tegra_emc_freq_req_to_bw(unsigned long freq);

/* API to get freqency switch latency at given MC freq.
 * freq_khz: Frequncy in KHz.
 * retruns latency in microseconds.
 */
static inline unsigned tegra_emc_dvfs_latency(unsigned int freq_khz)
{
	/* The latency data is not available based on freq.
	 * H/W expects it to be around 3 to 4us.
	 */
	return 4;
}

/*
 * On very old chips (T30) this was not always one.
 */
static inline int tegra_mc_get_tiled_memory_bandwidth_multiplier(void)
{
	return 1;
}


#define TEGRA_MC_CLIENT_AFI		0
#define TEGRA_MC_CLIENT_DC		2
#define TEGRA_MC_CLIENT_DCB		3
#define TEGRA_MC_CLIENT_EPP		4
#define TEGRA_MC_CLIENT_G2		5
#define TEGRA_MC_CLIENT_ISP		8
#define TEGRA_MC_CLIENT_MSENC		11
#define TEGRA_MC_CLIENT_MPE		11
#define TEGRA_MC_CLIENT_NV		12
#define TEGRA_MC_CLIENT_SATA		15
#define TEGRA_MC_CLIENT_VDE		16
#define TEGRA_MC_CLIENT_VI		17
#define TEGRA_MC_CLIENT_VIC		18
#define TEGRA_MC_CLIENT_XUSB_HOST	19
#define TEGRA_MC_CLIENT_XUSB_DEV	20
#define TEGRA_MC_CLIENT_TSEC		22
#define TEGRA_MC_CLIENT_ISPB		33
#define TEGRA_MC_CLIENT_GPU		34
#define TEGRA_MC_CLIENT_NVDEC		37
#define TEGRA_MC_CLIENT_NVJPG		40
#define TEGRA_MC_CLIENT_TSECB		45

enum {
	DRAM_TYPE_DDR3   = 0,
	DRAM_TYPE_LPDDR4 = 1,
	DRAM_TYPE_LPDDR2 = 2,
	DRAM_TYPE_DDR2   = 3,
};

int tegra_mc_flush(int id);
int tegra_mc_flush_done(int id);

/*
 * Necessary bit fields for various MC registers. Add to these as
 * necessary.
 */
#define MC_EMEM_ARB_MISC0_MC_EMC_SAME_FREQ_BIT			(1 << 27)

u32 tegra_get_dvfs_clk_change_latency_nsec(unsigned long emc_freq_khz);

#endif
