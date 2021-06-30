/*
 * This header provides macros for nvidia,xhci controller
 * device bindings.
 *
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

#ifndef __DT_TEGRA_XHCI_H__
#define __DT_TEGRA_XHCI_H__

#define TEGRA_XHCI_SS_P0	0
#define TEGRA_XHCI_SS_P1	1
#define TEGRA_XHCI_SS_P2	2
#define TEGRA_XHCI_SS_P3	3

#define TEGRA_XHCI_USB2_P0	0
#define TEGRA_XHCI_USB2_P1	1
#define TEGRA_XHCI_USB2_P2	2
#define TEGRA_XHCI_USB2_P3	3

#define TEGRA_XHCI_LANE_0	0
#define TEGRA_XHCI_LANE_1	1
#define TEGRA_XHCI_LANE_2	2
#define TEGRA_XHCI_LANE_3	3
#define TEGRA_XHCI_LANE_4	4
#define TEGRA_XHCI_LANE_5	5
#define TEGRA_XHCI_LANE_6	6

#define TEGRA_XHCI_PORT_OTG	1
#define TEGRA_XHCI_PORT_STD	0

#define TEGRA_XHCI_UNUSED_PORT	7
#define TEGRA_XHCI_UNUSED_LANE	0xF

#endif
