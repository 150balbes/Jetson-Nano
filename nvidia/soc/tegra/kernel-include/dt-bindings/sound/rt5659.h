/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
 * linux/sound/rt5659.h -- Platform data for #define RT5659
 *
 */

#ifndef __LINUX_SND_RT5659_H
#define __LINUX_SND_RT5659_H

/* dmic1 pin configuration */
#define RT5659_DMIC1_NULL		0
#define RT5659_DMIC1_DATA_IN2N		1
#define RT5659_DMIC1_DATA_GPIO5		2
#define RT5659_DMIC1_DATA_GPIO9		3
#define RT5659_DMIC1_DATA_GPIO11	4

/* dmic2 pin configuration */
#define RT5659_DMIC2_NULL		0
#define RT5659_DMIC2_DATA_IN2P		1
#define RT5659_DMIC2_DATA_GPIO6		2
#define RT5659_DMIC2_DATA_GPIO10	3
#define RT5659_DMIC2_DATA_GPIO12	4

/* jack detection source */
#define RT5659_JD_NULL			0
#define RT5659_JD3			1

#endif

