/*
 * GK20A Platform (SoC) Interface
 *
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _NVGPU_PLATFORM_GK20A_TEGRA_H_
#define _NVGPU_PLATFORM_GK20A_TEGRA_H_

struct gk20a_platform;

int gk20a_tegra_init_secure_alloc(struct gk20a_platform *platform);

#endif
