/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved.
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

#ifndef _DT_BINDINGS_PINCTRL_TEGRA_PADCTL_PHY_H
#define _DT_BINDINGS_PINCTRL_TEGRA_PADCTL_PHY_H 1

/* 0-15 for USB3 */
#define TEGRA_PADCTL_PHY_USB3_BASE	(0)
#define TEGRA_PADCTL_PHY_USB3_P(x)	((x) + TEGRA_PADCTL_PHY_USB3_BASE)
/* 16-31 for UTMI */
#define TEGRA_PADCTL_PHY_UTMI_BASE	(16)
#define TEGRA_PADCTL_PHY_UTMI_P(x)	((x) + TEGRA_PADCTL_PHY_UTMI_BASE)
/* 32-47 for HSIC */
#define TEGRA_PADCTL_PHY_HSIC_BASE	(32)
#define TEGRA_PADCTL_PHY_HSIC_P(x)	((x) + TEGRA_PADCTL_PHY_HSIC_BASE)
/* 48-63 for Tegra built-in CDP phy for UTMI */
#define TEGRA_PADCTL_PHY_CDP_BASE	(48)
#define TEGRA_PADCTL_PHY_CDP_P(x)	((x) + TEGRA_PADCTL_PHY_CDP_BASE)

#define TEGRA_PADCTL_PORT_DISABLED	(0)
#define TEGRA_PADCTL_PORT_HOST_ONLY	(1)
#define TEGRA_PADCTL_PORT_DEVICE_ONLY	(2)
#define TEGRA_PADCTL_PORT_OTG_CAP	(3)
#endif /* _DT_BINDINGS_PINCTRL_TEGRA_PADCTL_PHY_H */
