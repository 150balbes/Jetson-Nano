/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This header provides Tegra124-specific constants for binding
 * nvidia,tegra124-car.
 */

#include <dt-bindings/clock/tegra124-car-common.h>

#ifndef _DT_BINDINGS_CLOCK_TEGRA124_CAR_H
#define _DT_BINDINGS_CLOCK_TEGRA124_CAR_H

#define TEGRA124_CLK_PLL_X		227
#define TEGRA124_CLK_PLL_X_OUT0		228

#define TEGRA124_CLK_CCLK_G		262
#define TEGRA124_CLK_CCLK_LP		263

#define TEGRA124_CLK_CLK_MAX		315

#endif	/* _DT_BINDINGS_CLOCK_TEGRA124_CAR_H */
