/*
 * Copyright (C) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __TEGRA_MC_H
#define __TEGRA_MC_H

/*
 * API for reading carveout info.
 */
enum carveout_desc {
	MC_SECURITY_CARVEOUT1 = 0,
	MC_SECURITY_CARVEOUT2,
	MC_SECURITY_CARVEOUT3,
	MC_SECURITY_CARVEOUT4,
	MC_NR_CARVEOUTS
};

struct mc_carveout_info {
	enum carveout_desc desc;

	u64 base;
	u64 size;
};

#if defined(CONFIG_TEGRA_MC)

/**
 * Read from the MC.
 *
 * @idx The MC channel to read from.
 * @reg The offset of the register to read.
 *
 * Read from the specified MC channel: 0 -> MC0, 1 -> MC1, etc. If @idx
 * corresponds to a non-existent channel then 0 is returned.
 */
extern u32 tegra_mc_readl(u32 reg);

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
extern void tegra_mc_writel(u32 val, u32 reg);

extern int mc_get_carveout_info(struct mc_carveout_info *inf, int *nr,
			 enum carveout_desc co);

#else

static inline u32 tegra_mc_readl(u32 reg)
{
	return 0xffffffff;
}

static inline void tegra_mc_writel(u32 val, u32 reg)
{
}

static inline int mc_get_carveout_info(struct mc_carveout_info *inf, int *nr,
			 enum carveout_desc co)
{
	return -ENODEV;
}

#endif

#endif
