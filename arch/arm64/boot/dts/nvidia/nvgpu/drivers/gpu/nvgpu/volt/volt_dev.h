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

#ifndef NVGPU_VOLT_DEV_H
#define NVGPU_VOLT_DEV_H

#include "boardobj/boardobj.h"
#include "boardobj/boardobjgrp.h"
#include "ctrl/ctrlvolt.h"

#define VOLTAGE_TABLE_MAX_ENTRIES_ONE	1U
#define VOLTAGE_TABLE_MAX_ENTRIES	256U

struct voltage_device {
	struct boardobj super;
	u8 volt_domain;
	u8 i2c_dev_idx;
	u32 switch_delay_us;
	u32 num_entries;
	struct voltage_device_entry *pentry[VOLTAGE_TABLE_MAX_ENTRIES];
	struct voltage_device_entry *pcurr_entry;
	u8 rsvd_0;
	u8 rsvd_1;
	u8 operation_type;
	u32 voltage_min_uv;
	u32 voltage_max_uv;
	u32 volt_step_uv;
};

struct voltage_device_entry {
	u32  voltage_uv;
};

struct voltage_device_metadata {
	struct boardobjgrp_e32 volt_devices;
};

/*!
 * Extends VOLTAGE_DEVICE providing attributes specific to PWM controllers.
 */
struct voltage_device_pwm {
	struct voltage_device super;
	s32 voltage_base_uv;
	s32 voltage_offset_scale_uv;
	enum nv_pmu_pmgr_pwm_source source;
	u32 raw_period;
};

struct voltage_device_pwm_entry {
	struct voltage_device_entry  super;
	u32 duty_cycle;
};
/* PWM end */

u32 volt_dev_sw_setup(struct gk20a *g);
int volt_dev_pmu_setup(struct gk20a *g);

#endif /* NVGPU_VOLT_DEV_H */
