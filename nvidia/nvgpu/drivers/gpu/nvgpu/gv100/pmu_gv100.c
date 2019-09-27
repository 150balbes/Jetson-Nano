/*
 * GV100 PMU
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/pmu.h>
#include <nvgpu/gk20a.h>

#include "gv100/pmu_gv100.h"

int gv100_pmu_init_acr(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_acr_init_wpr_region rpc;
	int status = 0;

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_acr_init_wpr_region));
	rpc.wpr_regionId = 0x1;
	rpc.wpr_offset = 0x0;
	PMU_RPC_EXECUTE(status, pmu, ACR, INIT_WPR_REGION, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x",
			status);
	}

	return status;
}

int gv100_load_falcon_ucode(struct gk20a *g, u32 falconidmask)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct nv_pmu_rpc_struct_acr_bootstrap_gr_falcons rpc;
	u32 flags = PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES;
	int status = 0;

	if (falconidmask == 0)
		return -EINVAL;

	if (falconidmask & ~((1 << LSF_FALCON_ID_FECS) |
		(1 << LSF_FALCON_ID_GPCCS)))
		return -EINVAL;

	g->pmu_lsf_loaded_falcon_id = 0;
	/* check whether pmu is ready to bootstrap lsf if not wait for it */
	if (!g->pmu_lsf_pmu_wpr_init_done) {
		pmu_wait_message_cond(&g->pmu,
				gk20a_get_gr_idle_timeout(g),
				&g->pmu_lsf_pmu_wpr_init_done, 1);
		/* check again if it still not ready indicate an error */
		if (!g->pmu_lsf_pmu_wpr_init_done) {
			nvgpu_err(g, "PMU not ready to load LSF");
			status = -ETIMEDOUT;
			goto exit;
		}
	}

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_acr_bootstrap_gr_falcons));
	rpc.falcon_id_mask = falconidmask;
	rpc.flags = flags;
	rpc.falcon_va_mask = 0;
	rpc.wpr_base_virtual.lo = 0;
	rpc.wpr_base_virtual.hi = 0;
	PMU_RPC_EXECUTE(status, pmu, ACR, BOOTSTRAP_GR_FALCONS, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC, status=0x%x", status);
		goto exit;
	}

	pmu_wait_message_cond(&g->pmu, gk20a_get_gr_idle_timeout(g),
		&g->pmu_lsf_loaded_falcon_id, 1);

	if (g->pmu_lsf_loaded_falcon_id != 1)
		status =  -ETIMEDOUT;

exit:
	return status;
}
