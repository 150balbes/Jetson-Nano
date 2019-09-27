/*
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

#ifndef NVGPU_ECC_H
#define NVGPU_ECC_H

#include <nvgpu/types.h>
#include <nvgpu/list.h>

#define NVGPU_ECC_STAT_NAME_MAX_SIZE	100

struct gk20a;

struct nvgpu_ecc_stat {
	char name[NVGPU_ECC_STAT_NAME_MAX_SIZE];
	u32 counter;
	struct nvgpu_list_node node;
};

static inline struct nvgpu_ecc_stat *nvgpu_ecc_stat_from_node(
		struct nvgpu_list_node *node)
{
	return (struct nvgpu_ecc_stat *)(
			(uintptr_t)node - offsetof(struct nvgpu_ecc_stat, node)
		);
}

struct nvgpu_ecc {
	struct {
		/* stats per tpc */

		struct nvgpu_ecc_stat **sm_lrf_ecc_single_err_count;
		struct nvgpu_ecc_stat **sm_lrf_ecc_double_err_count;

		struct nvgpu_ecc_stat **sm_shm_ecc_sec_count;
		struct nvgpu_ecc_stat **sm_shm_ecc_sed_count;
		struct nvgpu_ecc_stat **sm_shm_ecc_ded_count;

		struct nvgpu_ecc_stat **tex_ecc_total_sec_pipe0_count;
		struct nvgpu_ecc_stat **tex_ecc_total_ded_pipe0_count;
		struct nvgpu_ecc_stat **tex_unique_ecc_sec_pipe0_count;
		struct nvgpu_ecc_stat **tex_unique_ecc_ded_pipe0_count;
		struct nvgpu_ecc_stat **tex_ecc_total_sec_pipe1_count;
		struct nvgpu_ecc_stat **tex_ecc_total_ded_pipe1_count;
		struct nvgpu_ecc_stat **tex_unique_ecc_sec_pipe1_count;
		struct nvgpu_ecc_stat **tex_unique_ecc_ded_pipe1_count;

		struct nvgpu_ecc_stat **sm_l1_tag_ecc_corrected_err_count;
		struct nvgpu_ecc_stat **sm_l1_tag_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat **sm_cbu_ecc_corrected_err_count;
		struct nvgpu_ecc_stat **sm_cbu_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat **sm_l1_data_ecc_corrected_err_count;
		struct nvgpu_ecc_stat **sm_l1_data_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat **sm_icache_ecc_corrected_err_count;
		struct nvgpu_ecc_stat **sm_icache_ecc_uncorrected_err_count;

		/* stats per gpc */

		struct nvgpu_ecc_stat *gcc_l15_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *gcc_l15_ecc_uncorrected_err_count;

		struct nvgpu_ecc_stat *gpccs_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *gpccs_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat *mmu_l1tlb_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *mmu_l1tlb_ecc_uncorrected_err_count;

		/* stats per device */
		struct nvgpu_ecc_stat *fecs_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *fecs_ecc_uncorrected_err_count;
	} gr;

	struct {
		/* stats per lts */
		struct nvgpu_ecc_stat **ecc_sec_count;
		struct nvgpu_ecc_stat **ecc_ded_count;
	} ltc;

	struct {
		/* stats per device */
		struct nvgpu_ecc_stat *mmu_l2tlb_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *mmu_l2tlb_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat *mmu_hubtlb_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *mmu_hubtlb_ecc_uncorrected_err_count;
		struct nvgpu_ecc_stat *mmu_fillunit_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *mmu_fillunit_ecc_uncorrected_err_count;
	} fb;

	struct {
		/* stats per device */
		struct nvgpu_ecc_stat *pmu_ecc_corrected_err_count;
		struct nvgpu_ecc_stat *pmu_ecc_uncorrected_err_count;
	} pmu;

	struct {
		/* stats per fbpa */
		struct nvgpu_ecc_stat *fbpa_ecc_sec_err_count;
		struct nvgpu_ecc_stat *fbpa_ecc_ded_err_count;
	} fbpa;

	struct nvgpu_list_node stats_list;
	int stats_count;
};

int nvgpu_ecc_counter_init_per_tpc(struct gk20a *g,
		struct nvgpu_ecc_stat ***stat, const char *name);
#define NVGPU_ECC_COUNTER_INIT_PER_TPC(stat) \
	nvgpu_ecc_counter_init_per_tpc(g, &g->ecc.gr.stat, #stat)

int nvgpu_ecc_counter_init_per_gpc(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name);
#define NVGPU_ECC_COUNTER_INIT_PER_GPC(stat) \
	nvgpu_ecc_counter_init_per_gpc(g, &g->ecc.gr.stat, #stat)

int nvgpu_ecc_counter_init(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name);
#define NVGPU_ECC_COUNTER_INIT_GR(stat) \
	nvgpu_ecc_counter_init(g, &g->ecc.gr.stat, #stat)
#define NVGPU_ECC_COUNTER_INIT_FB(stat) \
	nvgpu_ecc_counter_init(g, &g->ecc.fb.stat, #stat)
#define NVGPU_ECC_COUNTER_INIT_PMU(stat) \
	nvgpu_ecc_counter_init(g, &g->ecc.pmu.stat, #stat)

int nvgpu_ecc_counter_init_per_lts(struct gk20a *g,
		struct nvgpu_ecc_stat ***stat, const char *name);
#define NVGPU_ECC_COUNTER_INIT_PER_LTS(stat) \
	nvgpu_ecc_counter_init_per_lts(g, &g->ecc.ltc.stat, #stat)

int nvgpu_ecc_counter_init_per_fbpa(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name);
#define NVGPU_ECC_COUNTER_INIT_PER_FBPA(stat) \
	nvgpu_ecc_counter_init_per_fbpa(g, &g->ecc.fbpa.stat, #stat)

void nvgpu_ecc_free(struct gk20a *g);

int nvgpu_ecc_init_support(struct gk20a *g);
void nvgpu_ecc_remove_support(struct gk20a *g);

/* OSes to implement */

int nvgpu_ecc_sysfs_init(struct gk20a *g);
void nvgpu_ecc_sysfs_remove(struct gk20a *g);

#endif
