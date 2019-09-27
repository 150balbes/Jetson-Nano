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

#include <nvgpu/gk20a.h>

#include "pwrdev.h"
#include "pmgrpmu.h"

int pmgr_pwr_devices_get_power(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status) {
		nvgpu_err(g, "pmgr_pwr_devices_get_current_power failed %x",
			status);
	}

	*val = payload.devices[0].powerm_w;

	return status;
}

int pmgr_pwr_devices_get_current(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status) {
		nvgpu_err(g, "pmgr_pwr_devices_get_current failed %x",
			status);
	}

	*val = payload.devices[0].currentm_a;

	return status;
}

int pmgr_pwr_devices_get_voltage(struct gk20a *g, u32 *val)
{
	struct nv_pmu_pmgr_pwr_devices_query_payload payload;
	int status;

	status = pmgr_pmu_pwr_devices_query_blocking(g, 1, &payload);
	if (status) {
		nvgpu_err(g, "pmgr_pwr_devices_get_current_voltage failed %x",
			status);
	}

	*val = payload.devices[0].voltageu_v;

	return status;
}

u32 pmgr_domain_sw_setup(struct gk20a *g)
{
	u32 status;

	status = pmgr_device_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr devices, status - 0x%x",
			status);
		goto exit;
	}

	status = pmgr_monitor_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr monitor, status - 0x%x",
			status);
		goto exit;
	}

	status = pmgr_policy_sw_setup(g);
	if (status) {
		nvgpu_err(g,
			"error creating boardobjgrp for pmgr policy, status - 0x%x",
			status);
		goto exit;
	}

exit:
	return status;
}

int pmgr_domain_pmu_setup(struct gk20a *g)
{
	return pmgr_send_pmgr_tables_to_pmu(g);
}
