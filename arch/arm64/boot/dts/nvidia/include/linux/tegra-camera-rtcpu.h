/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_TEGRA_CAMERA_RTCPU_H_
#define _LINUX_TEGRA_CAMERA_RTCPU_H_

#include <linux/types.h>

struct device;

int tegra_camrtc_iovm_setup(struct device *dev, dma_addr_t iova);
ssize_t tegra_camrtc_print_version(struct device *dev, char *buf, size_t size);
int tegra_camrtc_reboot(struct device *dev);
int tegra_camrtc_restore(struct device *dev);
bool tegra_camrtc_is_rtcpu_alive(struct device *dev);
void tegra_camrtc_flush_trace(struct device *dev);

bool tegra_camrtc_is_rtcpu_powered(void);

#define TEGRA_CAMRTC_VERSION_LEN 128

int tegra_camrtc_command(struct device *dev, u32 command, long timeout);
int tegra_camrtc_prefix_command(struct device *dev,
				u32 prefix, u32 command, long timeout);

#endif
