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
 * This header provides Tegra210-specific constants for binding
 * nvidia,tegra210-car.
 */

#ifndef _DT_BINDINGS_RESET_TEGRA210_CAR_H
#define _DT_BINDINGS_RESET_TEGRA210_CAR_H

#define TEGRA210_RESET(x)		(7 * 32 + (x))
#define TEGRA210_RST_DFLL_DVCO		TEGRA210_RESET(0)
#define TEGRA210_RST_ADSP		TEGRA210_RESET(1)
#define TEGRA210_RST_UARTB		7
#define TEGRA210_RST_VI			20
#define TEGRA210_RST_XUSB_DEV		95

#endif	/* _DT_BINDINGS_RESET_TEGRA210_CAR_H */
