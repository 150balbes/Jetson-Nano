/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __SOC_TEGRA_KFUSE_H__
#define __SOC_TEGRA_KFUSE_H__

/* there are 144 32-bit values in total */
#define KFUSE_DATA_SZ (144 * 4)

#ifdef CONFIG_TEGRA_KFUSE
int tegra_kfuse_read(void *dest, size_t len);
void tegra_kfuse_disable_sensing(void);
int tegra_kfuse_enable_sensing(void);
#else
static inline int tegra_kfuse_read(void *dest, size_t len)
{
	return -EOPNOTSUPP;
}

static inline void tegra_kfuse_disable_sensing(void)
{
}

static inline int tegra_kfuse_enable_sensing(void)
{
	return -EOPNOTSUPP;
}
#endif

#endif /* __SOC_TEGRA_KFUSE_H__ */
