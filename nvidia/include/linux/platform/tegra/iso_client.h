/*
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _INCLUDE_MACH_ISO_CLIENT_H
#define _INCLUDE_MACH_ISO_CLIENT_H

enum tegra_iso_client {
	TEGRA_ISO_CLIENT_DISP_0,
	TEGRA_ISO_CLIENT_DISP_1,
	TEGRA_ISO_CLIENT_DISP_2,
	TEGRA_ISO_CLIENT_VI_0,
	TEGRA_ISO_CLIENT_VI_1,
	TEGRA_ISO_CLIENT_ISP_A,
	TEGRA_ISO_CLIENT_ISP_B,
	TEGRA_ISO_CLIENT_BBC_0,
	TEGRA_ISO_CLIENT_TEGRA_CAMERA,
	TEGRA_ISO_CLIENT_APE_ADMA,
	TEGRA_ISO_CLIENT_EQOS,
	TEGRA_ISO_CLIENT_COUNT
};

#endif /* _INCLUDE_MACH_ISO_CLIENT_H */
