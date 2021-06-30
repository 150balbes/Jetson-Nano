/*
* Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_CLK_FREQ_CONTROLLER_H
#define NVGPU_CLK_FREQ_CONTROLLER_H

#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_ALL  0xFF
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_SYS  0x00
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_LTC  0x01
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_XBAR 0x02
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0 0x03
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC1 0x04
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC2 0x05
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC3 0x06
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC4 0x07
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC5 0x08
#define CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPCS 0x09

#define CTRL_CLK_CLK_FREQ_CONTROLLER_MASK_UNICAST_GPC     \
			(BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC0) | \
			BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC1) | \
			BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC2) | \
			BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC3) | \
			BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC4) | \
			BIT(CTRL_CLK_CLK_FREQ_CONTROLLER_ID_GPC5))

#define CTRL_CLK_CLK_FREQ_CONTROLLER_TYPE_DISABLED  0x00
#define CTRL_CLK_CLK_FREQ_CONTROLLER_TYPE_PI        0x01


struct clk_freq_controller {
	struct boardobj    super;
	u8   controller_id;
	u8   parts_freq_mode;
	bool bdisable;
	u32  clk_domain;
	s16  freq_cap_noise_unaware_vmin_above;
	s16  freq_cap_noise_unaware_vmin_below;
	s16  freq_hyst_pos_mhz;
	s16  freq_hyst_neg_mhz;
};

struct clk_freq_controller_pi {
	struct clk_freq_controller super;
	s32 prop_gain;
	s32 integ_gain;
	s32 integ_decay;
	s32 volt_delta_min;
	s32 volt_delta_max;
	u8  slowdown_pct_min;
	bool bpoison;
};

struct clk_freq_controllers {
	struct boardobjgrp_e32 super;
	u32 sampling_period_ms;
	struct boardobjgrpmask_e32 freq_ctrl_load_mask;
	u8 volt_policy_idx;
	void *pprereq_load;
};

int clk_freq_controller_sw_setup(struct gk20a *g);
int clk_freq_controller_pmu_setup(struct gk20a *g);

#endif /* NVGPU_CLK_FREQ_CONTROLLER_H */
