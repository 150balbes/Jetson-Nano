/*
 * This header provides constants for Tegra pinctrl bindings.
 *
 * Copyright (c) 2013-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _DT_BINDINGS_PINCTRL_TEGRA_H
#define _DT_BINDINGS_PINCTRL_TEGRA_H

/*
 * Enable/disable for diffeent dt properties. This is applicable for
 * properties nvidia,enable-input, nvidia,tristate, nvidia,open-drain,
 * nvidia,lock, nvidia,rcv-sel, nvidia,high-speed-mode, nvidia,schmitt.
 */
#define TEGRA_PIN_DISABLE				0
#define TEGRA_PIN_ENABLE				1

#define TEGRA_PIN_PULL_NONE				0
#define TEGRA_PIN_PULL_DOWN				1
#define TEGRA_PIN_PULL_UP				2

/* Low power mode driver */
#define TEGRA_PIN_LP_DRIVE_DIV_8			0
#define TEGRA_PIN_LP_DRIVE_DIV_4			1
#define TEGRA_PIN_LP_DRIVE_DIV_2			2
#define TEGRA_PIN_LP_DRIVE_DIV_1			3

/* Rising/Falling slew rate */
#define TEGRA_PIN_SLEW_RATE_FASTEST			0
#define TEGRA_PIN_SLEW_RATE_FAST			1
#define TEGRA_PIN_SLEW_RATE_SLOW			2
#define TEGRA_PIN_SLEW_RATE_SLOWEST			3

#endif
