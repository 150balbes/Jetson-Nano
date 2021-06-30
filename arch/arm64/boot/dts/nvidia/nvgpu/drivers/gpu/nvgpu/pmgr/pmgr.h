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
#ifndef NVGPU_PMGR_H
#define NVGPU_PMGR_H

#include "pwrdev.h"
#include "pwrmonitor.h"
#include "pwrpolicy.h"

struct pmgr_pmupstate {
	struct pwr_devices pmgr_deviceobjs;
	struct pmgr_pwr_monitor pmgr_monitorobjs;
	struct pmgr_pwr_policy pmgr_policyobjs;
};

u32 pmgr_domain_sw_setup(struct gk20a *g);
int pmgr_domain_pmu_setup(struct gk20a *g);
int pmgr_pwr_devices_get_current(struct gk20a *g, u32 *val);
int pmgr_pwr_devices_get_voltage(struct gk20a *g, u32 *val);
int pmgr_pwr_devices_get_power(struct gk20a *g, u32 *val);

#endif /* NVGPU_PMGR_H */
