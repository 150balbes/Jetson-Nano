/*
 * include/dt-bindings/display/tegra-dc.h
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

#ifndef __TEGRA_DC_H
#define __TEGRA_DC_H

/* tegra_dc_out.type */
#define TEGRA_DC_OUT_RGB		0
#define TEGRA_DC_OUT_HDMI		1
#define TEGRA_DC_OUT_DSI		2
#define TEGRA_DC_OUT_DP			3
#define TEGRA_DC_OUT_LVDS		4
#define TEGRA_DC_OUT_NVSR_DP		5
#define TEGRA_DC_OUT_FAKE_DP		6
#define TEGRA_DC_OUT_FAKE_DSIA		7
#define TEGRA_DC_OUT_FAKE_DSIB		8
#define TEGRA_DC_OUT_FAKE_DSI_GANGED	9
#define TEGRA_DC_OUT_NULL		10
/* Keep as the last dc_out_define*/
#define TEGRA_DC_OUT_MAX		11

/* tegra_dc_out.dither */
#define TEGRA_DC_UNDEFINED_DITHER	0
#define TEGRA_DC_DISABLE_DITHER		1
#define TEGRA_DC_ORDERED_DITHER		2
#define TEGRA_DC_ERRDIFF_DITHER		3
#define TEGRA_DC_TEMPORAL_DITHER	4

/* bits for tegra_dc_out.flags */
#define TEGRA_DC_OUT_HOTPLUG_HIGH		(0 << 1)
#define TEGRA_DC_OUT_HOTPLUG_LOW		(1 << 1)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ALWAYS_ON	(0 << 2)
#define TEGRA_DC_OUT_NVHDCP_POLICY_ON_DEMAND	(1 << 2)
#define TEGRA_DC_OUT_CONTINUOUS_MODE		(0 << 3)
#define TEGRA_DC_OUT_ONE_SHOT_MODE		(1 << 3)
#define TEGRA_DC_OUT_N_SHOT_MODE		(1 << 4)
#define TEGRA_DC_OUT_ONE_SHOT_LP_MODE		(1 << 5)
#define TEGRA_DC_OUT_INITIALIZED_MODE		(1 << 6)
/* Makes hotplug GPIO a LP0 wakeup source */
#define TEGRA_DC_OUT_HOTPLUG_WAKE_LP0		(1 << 7)

/* bots for tegra_dc_out.hdcp_policy */
#define TEGRA_DC_HDCP_POLICY_ALWAYS_ON	0
#define TEGRA_DC_HDCP_POLICY_ON_DEMAND	1
#define TEGRA_DC_HDCP_POLICY_ALWAYS_OFF	2

/* tegra_dc_out.align */
#define TEGRA_DC_ALIGN_MSB		0
#define TEGRA_DC_ALIGN_LSB		1

/* tegra_dc_out.order */
#define TEGRA_DC_ORDER_RED_BLUE		0
#define TEGRA_DC_ORDER_BLUE_RED		1

/* tegra_fb_data.flags */
#define TEGRA_FB_FLIP_ON_PROBE		(1 << 0)

/* tegra_dc_platform_data.flags */
#define TEGRA_DC_FLAG_ENABLED		(1 << 0)
#define TEGRA_DC_FLAG_SET_EARLY_MODE		(1 << 1)
#define TEGRA_DC_FLAG_FBCON_DISABLED	(1 << 2)

/* tegra_dc_out_pin.name */
#define TEGRA_DC_OUT_PIN_DATA_ENABLE	0
#define TEGRA_DC_OUT_PIN_H_SYNC		1
#define TEGRA_DC_OUT_PIN_V_SYNC		2
#define TEGRA_DC_OUT_PIN_PIXEL_CLOCK	3

/* tegra_dc_out_pin.pol */
#define TEGRA_DC_OUT_PIN_POL_LOW	0
#define TEGRA_DC_OUT_PIN_POL_HIGH	1

/* tegra_hdmi_out.generic_infoframe_type */
#define HDMI_INFOFRAME_TYPE_SPD		0x83
#define HDMI_INFOFRAME_TYPE_HDR		0x87

#endif /* __TEGRA_DC_H */

