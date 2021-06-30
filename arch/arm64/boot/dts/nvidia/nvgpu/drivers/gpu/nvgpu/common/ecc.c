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

#include <nvgpu/gk20a.h>

static void nvgpu_ecc_stat_add(struct gk20a *g, struct nvgpu_ecc_stat *stat)
{
	struct nvgpu_ecc *ecc = &g->ecc;

	nvgpu_init_list_node(&stat->node);

	nvgpu_list_add_tail(&stat->node, &ecc->stats_list);
	ecc->stats_count++;
}

static void nvgpu_ecc_init(struct gk20a *g)
{
	struct nvgpu_ecc *ecc = &g->ecc;

	nvgpu_init_list_node(&ecc->stats_list);
}

int nvgpu_ecc_counter_init_per_tpc(struct gk20a *g,
		struct nvgpu_ecc_stat ***stat, const char *name)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_ecc_stat **stats;
	u32 gpc, tpc;
	int err = 0;

	stats = nvgpu_kzalloc(g, sizeof(*stats) * gr->gpc_count);
	if (stats == NULL) {
		return -ENOMEM;
	}
	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		stats[gpc] = nvgpu_kzalloc(g,
				sizeof(*stats[gpc]) * gr->gpc_tpc_count[gpc]);
		if (stats[gpc] == NULL) {
			err = -ENOMEM;
			break;
		}
	}

	if (err != 0) {
		while (gpc-- != 0u) {
			nvgpu_kfree(g, stats[gpc]);
		}

		nvgpu_kfree(g, stats);
		return err;
	}

	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		for (tpc = 0; tpc < gr->gpc_tpc_count[gpc]; tpc++) {
			snprintf(stats[gpc][tpc].name,
					NVGPU_ECC_STAT_NAME_MAX_SIZE,
					"gpc%d_tpc%d_%s", gpc, tpc, name);
			nvgpu_ecc_stat_add(g, &stats[gpc][tpc]);
		}
	}

	*stat = stats;
	return 0;
}

int nvgpu_ecc_counter_init_per_gpc(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_ecc_stat *stats;
	u32 gpc;

	stats = nvgpu_kzalloc(g, sizeof(*stats) * gr->gpc_count);
	if (stats == NULL) {
		return -ENOMEM;
	}
	for (gpc = 0; gpc < gr->gpc_count; gpc++) {
		snprintf(stats[gpc].name, NVGPU_ECC_STAT_NAME_MAX_SIZE,
				"gpc%d_%s", gpc, name);
		nvgpu_ecc_stat_add(g, &stats[gpc]);
	}

	*stat = stats;
	return 0;
}

int nvgpu_ecc_counter_init(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name)
{
	struct nvgpu_ecc_stat *stats;

	stats = nvgpu_kzalloc(g, sizeof(*stats));
	if (stats == NULL) {
		return -ENOMEM;
	}

	(void)strncpy(stats->name, name, NVGPU_ECC_STAT_NAME_MAX_SIZE - 1);
	nvgpu_ecc_stat_add(g, stats);
	*stat = stats;
	return 0;
}

int nvgpu_ecc_counter_init_per_lts(struct gk20a *g,
		struct nvgpu_ecc_stat ***stat, const char *name)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_ecc_stat **stats;
	u32 ltc, lts;
	int err = 0;

	stats = nvgpu_kzalloc(g, sizeof(*stats) * g->ltc_count);
	if (stats == NULL) {
		return -ENOMEM;
	}
	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		stats[ltc] = nvgpu_kzalloc(g,
				sizeof(*stats[ltc]) * gr->slices_per_ltc);
		if (stats[ltc] == NULL) {
			err = -ENOMEM;
			break;
		}
	}

	if (err != 0) {
		while (ltc-- > 0u) {
			nvgpu_kfree(g, stats[ltc]);
		}

		nvgpu_kfree(g, stats);
		return err;
	}

	for (ltc = 0; ltc < g->ltc_count; ltc++) {
		for (lts = 0; lts < gr->slices_per_ltc; lts++) {
			snprintf(stats[ltc][lts].name,
					NVGPU_ECC_STAT_NAME_MAX_SIZE,
					"ltc%d_lts%d_%s", ltc, lts, name);
			nvgpu_ecc_stat_add(g, &stats[ltc][lts]);
		}
	}

	*stat = stats;
	return 0;
}

int nvgpu_ecc_counter_init_per_fbpa(struct gk20a *g,
		struct nvgpu_ecc_stat **stat, const char *name)
{
	int i;
	int num_fbpa = nvgpu_get_litter_value(g, GPU_LIT_NUM_FBPAS);
	struct nvgpu_ecc_stat *stats;

	stats = nvgpu_kzalloc(g, sizeof(*stats) * num_fbpa);
	if (stats == NULL) {
		return -ENOMEM;
	}

	for (i = 0; i < num_fbpa; i++) {
		snprintf(stats[i].name, NVGPU_ECC_STAT_NAME_MAX_SIZE,
				"fbpa%d_%s", i, name);
		nvgpu_ecc_stat_add(g, &stats[i]);
	}

	*stat = stats;
	return 0;
}

/* release all ecc_stat */
void nvgpu_ecc_free(struct gk20a *g)
{
	struct nvgpu_ecc *ecc = &g->ecc;
	struct gr_gk20a *gr = &g->gr;
	u32 i;

	for (i = 0; i < gr->gpc_count; i++) {
		if (ecc->gr.sm_lrf_ecc_single_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_lrf_ecc_single_err_count[i]);
		}

		if (ecc->gr.sm_lrf_ecc_double_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_lrf_ecc_double_err_count[i]);
		}

		if (ecc->gr.sm_shm_ecc_sec_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_shm_ecc_sec_count[i]);
		}

		if (ecc->gr.sm_shm_ecc_sed_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_shm_ecc_sed_count[i]);
		}

		if (ecc->gr.sm_shm_ecc_ded_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_shm_ecc_ded_count[i]);
		}

		if (ecc->gr.tex_ecc_total_sec_pipe0_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_ecc_total_sec_pipe0_count[i]);
		}

		if (ecc->gr.tex_ecc_total_ded_pipe0_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_ecc_total_ded_pipe0_count[i]);
		}

		if (ecc->gr.tex_unique_ecc_sec_pipe0_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_unique_ecc_sec_pipe0_count[i]);
		}

		if (ecc->gr.tex_unique_ecc_ded_pipe0_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_unique_ecc_ded_pipe0_count[i]);
		}

		if (ecc->gr.tex_ecc_total_sec_pipe1_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_ecc_total_sec_pipe1_count[i]);
		}

		if (ecc->gr.tex_ecc_total_ded_pipe1_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_ecc_total_ded_pipe1_count[i]);
		}

		if (ecc->gr.tex_unique_ecc_sec_pipe1_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_unique_ecc_sec_pipe1_count[i]);
		}

		if (ecc->gr.tex_unique_ecc_ded_pipe1_count != NULL) {
			nvgpu_kfree(g, ecc->gr.tex_unique_ecc_ded_pipe1_count[i]);
		}

		if (ecc->gr.sm_l1_tag_ecc_corrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_l1_tag_ecc_corrected_err_count[i]);
		}

		if (ecc->gr.sm_l1_tag_ecc_uncorrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_l1_tag_ecc_uncorrected_err_count[i]);
		}

		if (ecc->gr.sm_cbu_ecc_corrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_cbu_ecc_corrected_err_count[i]);
		}

		if (ecc->gr.sm_cbu_ecc_uncorrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_cbu_ecc_uncorrected_err_count[i]);
		}

		if (ecc->gr.sm_l1_data_ecc_corrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_l1_data_ecc_corrected_err_count[i]);
		}

		if (ecc->gr.sm_l1_data_ecc_uncorrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_l1_data_ecc_uncorrected_err_count[i]);
		}

		if (ecc->gr.sm_icache_ecc_corrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_icache_ecc_corrected_err_count[i]);
		}

		if (ecc->gr.sm_icache_ecc_uncorrected_err_count != NULL) {
			nvgpu_kfree(g, ecc->gr.sm_icache_ecc_uncorrected_err_count[i]);
		}
	}
	nvgpu_kfree(g, ecc->gr.sm_lrf_ecc_single_err_count);
	nvgpu_kfree(g, ecc->gr.sm_lrf_ecc_double_err_count);
	nvgpu_kfree(g, ecc->gr.sm_shm_ecc_sec_count);
	nvgpu_kfree(g, ecc->gr.sm_shm_ecc_sed_count);
	nvgpu_kfree(g, ecc->gr.sm_shm_ecc_ded_count);
	nvgpu_kfree(g, ecc->gr.tex_ecc_total_sec_pipe0_count);
	nvgpu_kfree(g, ecc->gr.tex_ecc_total_ded_pipe0_count);
	nvgpu_kfree(g, ecc->gr.tex_unique_ecc_sec_pipe0_count);
	nvgpu_kfree(g, ecc->gr.tex_unique_ecc_ded_pipe0_count);
	nvgpu_kfree(g, ecc->gr.tex_ecc_total_sec_pipe1_count);
	nvgpu_kfree(g, ecc->gr.tex_ecc_total_ded_pipe1_count);
	nvgpu_kfree(g, ecc->gr.tex_unique_ecc_sec_pipe1_count);
	nvgpu_kfree(g, ecc->gr.tex_unique_ecc_ded_pipe1_count);
	nvgpu_kfree(g, ecc->gr.sm_l1_tag_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_l1_tag_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_cbu_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_cbu_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_l1_data_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_l1_data_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_icache_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.sm_icache_ecc_uncorrected_err_count);

	nvgpu_kfree(g, ecc->gr.gcc_l15_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.gcc_l15_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.gpccs_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.gpccs_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.mmu_l1tlb_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.mmu_l1tlb_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->gr.fecs_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->gr.fecs_ecc_uncorrected_err_count);

	for (i = 0; i < g->ltc_count; i++) {
		if (ecc->ltc.ecc_sec_count != NULL) {
			nvgpu_kfree(g, ecc->ltc.ecc_sec_count[i]);
		}

		if (ecc->ltc.ecc_ded_count != NULL) {
			nvgpu_kfree(g, ecc->ltc.ecc_ded_count[i]);
		}
	}
	nvgpu_kfree(g, ecc->ltc.ecc_sec_count);
	nvgpu_kfree(g, ecc->ltc.ecc_ded_count);

	nvgpu_kfree(g, ecc->fb.mmu_l2tlb_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->fb.mmu_l2tlb_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->fb.mmu_hubtlb_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->fb.mmu_hubtlb_ecc_uncorrected_err_count);
	nvgpu_kfree(g, ecc->fb.mmu_fillunit_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->fb.mmu_fillunit_ecc_uncorrected_err_count);

	nvgpu_kfree(g, ecc->pmu.pmu_ecc_corrected_err_count);
	nvgpu_kfree(g, ecc->pmu.pmu_ecc_uncorrected_err_count);

	nvgpu_kfree(g, ecc->fbpa.fbpa_ecc_sec_err_count);
	nvgpu_kfree(g, ecc->fbpa.fbpa_ecc_ded_err_count);

	(void)memset(ecc, 0, sizeof(*ecc));
}

int nvgpu_ecc_init_support(struct gk20a *g)
{
	int err;

	if (g->ops.gr.init_ecc == NULL) {
		return 0;
	}

	nvgpu_ecc_init(g);
	err = g->ops.gr.init_ecc(g);
	if (err != 0) {
		return err;
	}

	err = nvgpu_ecc_sysfs_init(g);
	if (err != 0) {
		nvgpu_ecc_free(g);
		return err;
	}

	return 0;
}

void nvgpu_ecc_remove_support(struct gk20a *g)
{
	if (g->ops.gr.init_ecc == NULL) {
		return;
	}

	nvgpu_ecc_sysfs_remove(g);
	nvgpu_ecc_free(g);
}
