/*
 * general power channel structures & definitions
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
#ifndef NVGPU_PMGR_PWRMONITOR_H
#define NVGPU_PMGR_PWRMONITOR_H

#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include "boardobj/boardobjgrp.h"
#include "boardobj/boardobj.h"
#include "ctrl/ctrlpmgr.h"

struct pwr_channel {
	struct boardobj super;
	u8 pwr_rail;
	u32 volt_fixed_uv;
	u32 pwr_corr_slope;
	s32 pwr_corr_offset_mw;
	u32 curr_corr_slope;
	s32 curr_corr_offset_ma;
	u32 dependent_ch_mask;
};

struct pwr_chrelationship {
	struct boardobj super;
	u8 chIdx;
};

struct pwr_channel_sensor {
	struct pwr_channel super;
	u8 pwr_dev_idx;
	u8 pwr_dev_prov_idx;
};

struct pmgr_pwr_monitor {
	bool b_is_topology_tbl_ver_1x;
	struct boardobjgrp_e32 pwr_channels;
	struct boardobjgrp_e32 pwr_ch_rels;
	u8 total_gpu_channel_idx;
	u32 physical_channel_mask;
	struct nv_pmu_pmgr_pwr_monitor_pack pmu_data;
};

#define PMGR_PWR_MONITOR_GET_PWR_CHANNEL(g, channel_idx)                    \
	((struct pwr_channel *)BOARDOBJGRP_OBJ_GET_BY_IDX(                                 \
		&(g->pmgr_pmu.pmgr_monitorobjs.pwr_channels.super), (channel_idx)))

int pmgr_monitor_sw_setup(struct gk20a *g);

#endif /* NVGPU_PMGR_PWRMONITOR_H */
