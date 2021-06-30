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

#ifndef NVGPU_CLK_FLL_H
#define NVGPU_CLK_FLL_H

#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "boardobj/boardobjgrp_e32.h"
#include "boardobj/boardobjgrpmask.h"

/*data and function definition to talk to driver*/
int clk_fll_sw_setup(struct gk20a *g);
int clk_fll_pmu_setup(struct gk20a *g);

struct avfsfllobjs {
	struct boardobjgrp_e32 super;
	struct boardobjgrpmask_e32 lut_prog_master_mask;
	u32 lut_step_size_uv;
	u32 lut_min_voltage_uv;
	u8 lut_num_entries;
	u16 max_min_freq_mhz;
};

struct fll_device;

typedef u32 fll_lut_broadcast_slave_register(struct gk20a *g,
	struct avfsfllobjs *pfllobjs,
	struct fll_device *pfll,
	struct fll_device *pfll_slave);

struct fll_device {
	struct boardobj super;
	u8 id;
	u8 mdiv;
	u16 input_freq_mhz;
	u32 clk_domain;
	u8 vin_idx_logic;
	u8 vin_idx_sram;
	u8 rail_idx_for_lut;
	struct nv_pmu_clk_lut_device_desc lut_device;
	struct nv_pmu_clk_regime_desc regime_desc;
	u8 min_freq_vfe_idx;
	u8 freq_ctrl_idx;
	u8 target_regime_id_override;
	bool b_skip_pldiv_below_dvco_min;
	bool b_dvco_1x;
	struct boardobjgrpmask_e32 lut_prog_broadcast_slave_mask;
	fll_lut_broadcast_slave_register *lut_broadcast_slave_register;
};

u32 nvgpu_clk_get_vbios_clk_domain_gv10x( u32 vbios_domain);
u32 nvgpu_clk_get_vbios_clk_domain_gp10x( u32 vbios_domain);

#define CLK_FLL_LUT_VF_NUM_ENTRIES(pclk) \
	(pclk->avfs_fllobjs.lut_num_entries)

#define CLK_FLL_LUT_MIN_VOLTAGE_UV(pclk) \
	(pclk->avfs_fllobjs.lut_min_voltage_uv)
#define CLK_FLL_LUT_STEP_SIZE_UV(pclk) \
	(pclk->avfs_fllobjs.lut_step_size_uv)

#endif /* NVGPU_CLK_FLL_H */
