/*
 * drivers/video/tegra/host/gk20a/pmu_gk20a.h
 *
 * GK20A PMU (aka. gPMU outside gk20a context)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_GK20A_PMU_GK20A_H
#define NVGPU_GK20A_PMU_GK20A_H

#include <nvgpu/flcnif_cmn.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/pmu.h>

struct nvgpu_firmware;

#define ZBC_MASK(i)			(~(~(0) << ((i)+1)) & 0xfffe)

bool gk20a_pmu_is_interrupted(struct nvgpu_pmu *pmu);
void gk20a_pmu_isr(struct gk20a *g);

u32 gk20a_pmu_pg_engines_list(struct gk20a *g);
u32 gk20a_pmu_pg_feature_list(struct gk20a *g, u32 pg_engine_id);

void gk20a_pmu_save_zbc(struct gk20a *g, u32 entries);

void gk20a_pmu_init_perfmon_counter(struct gk20a *g);

void gk20a_pmu_pg_idle_counter_config(struct gk20a *g, u32 pg_engine_id);

int gk20a_pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token);
int gk20a_pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token);

int gk20a_pmu_queue_head(struct gk20a *g, struct nvgpu_falcon_queue *queue,
			u32 *head, bool set);
int gk20a_pmu_queue_tail(struct gk20a *g, struct nvgpu_falcon_queue *queue,
			u32 *tail, bool set);
void gk20a_pmu_msgq_tail(struct nvgpu_pmu *pmu, u32 *tail, bool set);

u32 gk20a_pmu_read_idle_counter(struct gk20a *g, u32 counter_id);
void gk20a_pmu_reset_idle_counter(struct gk20a *g, u32 counter_id);

u32 gk20a_pmu_read_idle_intr_status(struct gk20a *g);
void gk20a_pmu_clear_idle_intr_status(struct gk20a *g);

void gk20a_write_dmatrfbase(struct gk20a *g, u32 addr);
bool gk20a_is_pmu_supported(struct gk20a *g);

int pmu_bootstrap(struct nvgpu_pmu *pmu);

void gk20a_pmu_dump_elpg_stats(struct nvgpu_pmu *pmu);
void gk20a_pmu_dump_falcon_stats(struct nvgpu_pmu *pmu);

void gk20a_pmu_enable_irq(struct nvgpu_pmu *pmu, bool enable);
void pmu_handle_fecs_boot_acr_msg(struct gk20a *g, struct pmu_msg *msg,
				void *param, u32 handle, u32 status);
void gk20a_pmu_elpg_statistics(struct gk20a *g, u32 pg_engine_id,
		struct pmu_pg_stats_data *pg_stat_data);
bool gk20a_pmu_is_engine_in_reset(struct gk20a *g);
int gk20a_pmu_engine_reset(struct gk20a *g, bool do_reset);
u32 gk20a_pmu_get_irqdest(struct gk20a *g);
#endif /*NVGPU_GK20A_PMU_GK20A_H*/
