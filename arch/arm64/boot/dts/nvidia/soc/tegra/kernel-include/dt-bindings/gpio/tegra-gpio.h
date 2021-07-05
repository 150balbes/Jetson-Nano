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
 * This header provides constants for binding nvidia,tegra*-gpio.
 *
 * The first cell in Tegra's GPIO specifier is the GPIO ID. The macros below
 * provide names for this.
 *
 * The second cell contains standard flag values specified in gpio.h.
 *
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _DT_BINDINGS_GPIO_TEGRA_GPIO_H
#define _DT_BINDINGS_GPIO_TEGRA_GPIO_H

#include <dt-bindings/gpio/gpio.h>

#define TEGRA_GPIO_BANK_ID_A 0
#define TEGRA_GPIO_BANK_ID_B 1
#define TEGRA_GPIO_BANK_ID_C 2
#define TEGRA_GPIO_BANK_ID_D 3
#define TEGRA_GPIO_BANK_ID_E 4
#define TEGRA_GPIO_BANK_ID_F 5
#define TEGRA_GPIO_BANK_ID_G 6
#define TEGRA_GPIO_BANK_ID_H 7
#define TEGRA_GPIO_BANK_ID_I 8
#define TEGRA_GPIO_BANK_ID_J 9
#define TEGRA_GPIO_BANK_ID_K 10
#define TEGRA_GPIO_BANK_ID_L 11
#define TEGRA_GPIO_BANK_ID_M 12
#define TEGRA_GPIO_BANK_ID_N 13
#define TEGRA_GPIO_BANK_ID_O 14
#define TEGRA_GPIO_BANK_ID_P 15
#define TEGRA_GPIO_BANK_ID_Q 16
#define TEGRA_GPIO_BANK_ID_R 17
#define TEGRA_GPIO_BANK_ID_S 18
#define TEGRA_GPIO_BANK_ID_T 19
#define TEGRA_GPIO_BANK_ID_U 20
#define TEGRA_GPIO_BANK_ID_V 21
#define TEGRA_GPIO_BANK_ID_W 22
#define TEGRA_GPIO_BANK_ID_X 23
#define TEGRA_GPIO_BANK_ID_Y 24
#define TEGRA_GPIO_BANK_ID_Z 25
#define TEGRA_GPIO_BANK_ID_AA 26
#define TEGRA_GPIO_BANK_ID_BB 27
#define TEGRA_GPIO_BANK_ID_CC 28
#define TEGRA_GPIO_BANK_ID_DD 29
#define TEGRA_GPIO_BANK_ID_EE 30
#define TEGRA_GPIO_BANK_ID_FF 31

#define TEGRA_GPIO(bank, offset) \
	((TEGRA_GPIO_BANK_ID_##bank * 8) + offset)

#endif
