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

#include <nvgpu/pmu.h>
#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>

void nvgpu_pmu_dump_elpg_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = pmu->g;

	/* Print PG stats */
	nvgpu_err(g, "Print PG stats");
	nvgpu_flcn_print_dmem(pmu->flcn,
		pmu->stat_dmem_offset[PMU_PG_ELPG_ENGINE_ID_GRAPHICS],
		sizeof(struct pmu_pg_stats_v2));

	g->ops.pmu.pmu_dump_elpg_stats(pmu);
}

void nvgpu_pmu_dump_falcon_stats(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = pmu->g;

	nvgpu_flcn_dump_stats(pmu->flcn);
	g->ops.pmu.pmu_dump_falcon_stats(pmu);

	nvgpu_err(g, "pmu state: %d", pmu->pmu_state);
	nvgpu_err(g, "elpg state: %d", pmu->elpg_stat);

	/* PMU may crash due to FECS crash. Dump FECS status */
	gk20a_fecs_dump_falcon_stats(g);
}
