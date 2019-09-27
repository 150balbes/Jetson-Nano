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

#ifndef NVGPU_CLK_VIN_H
#define NVGPU_CLK_VIN_H

#include "boardobj/boardobj.h"
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobjgrp_e32.h"

struct vin_device;
struct clk_pmupstate;

struct avfsvinobjs {
	struct boardobjgrp_e32 super;
	u8 calibration_rev_vbios;
	u8 calibration_rev_fused;
	bool vin_is_disable_allowed;
};
typedef u32 vin_device_state_load(struct gk20a *g,
			struct clk_pmupstate *clk, struct vin_device *pdev);

struct vin_device {
	struct boardobj super;
	u8 id;
	u8 volt_domain;
	u8 volt_domain_vbios;
	u32 flls_shared_mask;

	vin_device_state_load  *state_load;
};

struct vin_device_v10 {
	struct vin_device super;
	struct ctrl_clk_vin_device_info_data_v10 data;
};

struct vin_device_v20 {
	struct vin_device super;
	struct ctrl_clk_vin_device_info_data_v20 data;
};

/* get vin device object from descriptor table index*/
#define CLK_GET_VIN_DEVICE(pvinobjs, dev_index)                               \
	((struct vin_device *)BOARDOBJGRP_OBJ_GET_BY_IDX(                       \
	((struct boardobjgrp *)&(pvinobjs->super.super)), (dev_index)))

boardobj_construct construct_vindevice;
boardobj_pmudatainit vindeviceinit_pmudata_super;

int clk_vin_sw_setup(struct gk20a *g);
int clk_vin_pmu_setup(struct gk20a *g);
u32 clk_avfs_get_vin_cal_fuse_v10(struct gk20a *g,
					struct avfsvinobjs *pvinobjs,
					struct vin_device_v20 *pvindev);
u32 clk_avfs_get_vin_cal_fuse_v20(struct gk20a *g,
					struct avfsvinobjs *pvinobjs,
					struct vin_device_v20 *pvindev);

#endif /* NVGPU_CLK_VIN_H */
