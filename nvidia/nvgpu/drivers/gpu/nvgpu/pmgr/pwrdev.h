/*
 * general power device structures & definitions
 *
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
#ifndef NVGPU_PMGR_PWRDEV_H
#define NVGPU_PMGR_PWRDEV_H

#include "boardobj/boardobj.h"
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "ctrl/ctrlpmgr.h"

#define  PWRDEV_I2CDEV_DEVICE_INDEX_NONE  (0xFF)

#define  PWR_DEVICE_PROV_NUM_DEFAULT                                           1

struct pwr_device {
	struct boardobj super;
	u8 power_rail;
	u8 i2c_dev_idx;
	bool bIs_inforom_config;
	u32 power_corr_factor;
};

struct pwr_devices {
	struct boardobjgrp_e32 super;
};

struct pwr_device_ina3221 {
	struct pwr_device super;
	struct ctrl_pmgr_pwr_device_info_rshunt
		r_shuntm_ohm[NV_PMU_PMGR_PWR_DEVICE_INA3221_CH_NUM];
	u16 configuration;
	u16 mask_enable;
	u8 gpio_function;
	u16 curr_correct_m;
	s16 curr_correct_b;
} ;

int pmgr_device_sw_setup(struct gk20a *g);

#endif /* NVGPU_PMGR_PWRDEV_H */
