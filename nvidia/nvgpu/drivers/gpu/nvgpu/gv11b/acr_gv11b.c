/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/timers.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/firmware.h>
#include <nvgpu/mm.h>
#include <nvgpu/acr/nvgpu_acr.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include "acr_gv11b.h"
#include "pmu_gv11b.h"
#include "gm20b/mm_gm20b.h"
#include "gm20b/pmu_gm20b.h"
#include "gm20b/acr_gm20b.h"
#include "gp106/acr_gp106.h"

#include <nvgpu/hw/gv11b/hw_pwr_gv11b.h>

/*Defines*/
#define gv11b_dbg_pmu(g, fmt, arg...) \
	nvgpu_log(g, gpu_dbg_pmu, fmt, ##arg)

/*Externs*/

/*Forwards*/

int gv11b_alloc_blob_space(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem)
{
	int err;

	gv11b_dbg_pmu(g, "alloc blob space: NVGPU_DMA_FORCE_CONTIGUOUS");
	err = nvgpu_dma_alloc_flags_sys(g, NVGPU_DMA_FORCE_CONTIGUOUS,
						size, mem);

	return err;
}

void gv11b_setup_apertures(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;
	struct nvgpu_mem *inst_block = &mm->pmu.inst_block;

	nvgpu_log_fn(g, " ");

	/* setup apertures - virtual */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_UCODE),
			pwr_fbif_transcfg_mem_type_physical_f() |
			nvgpu_aperture_mask(g, inst_block,
			pwr_fbif_transcfg_target_noncoherent_sysmem_f(),
			pwr_fbif_transcfg_target_coherent_sysmem_f(),
			pwr_fbif_transcfg_target_local_fb_f()));
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_VIRT),
			pwr_fbif_transcfg_mem_type_virtual_f());
	/* setup apertures - physical */
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_VID),
			pwr_fbif_transcfg_mem_type_physical_f() |
			nvgpu_aperture_mask(g, inst_block,
			pwr_fbif_transcfg_target_noncoherent_sysmem_f(),
			pwr_fbif_transcfg_target_coherent_sysmem_f(),
			pwr_fbif_transcfg_target_local_fb_f()));
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_COH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_coherent_sysmem_f());
	gk20a_writel(g, pwr_fbif_transcfg_r(GK20A_PMU_DMAIDX_PHYS_SYS_NCOH),
			pwr_fbif_transcfg_mem_type_physical_f() |
			pwr_fbif_transcfg_target_noncoherent_sysmem_f());
}

int gv11b_acr_patch_wpr_info_to_ucode(struct gk20a *g, struct nvgpu_acr *acr,
	struct hs_acr *acr_desc, bool is_recovery)
{
	struct nvgpu_firmware *acr_fw = acr_desc->acr_fw;
	struct acr_fw_header *acr_fw_hdr = NULL;
	struct bin_hdr *acr_fw_bin_hdr = NULL;
	struct flcn_acr_desc_v1 *acr_dmem_desc;
	u32 *acr_ucode_header = NULL;
	u32 *acr_ucode_data = NULL;

	nvgpu_log_fn(g, " ");

	if (is_recovery) {
		acr_desc->acr_dmem_desc_v1->nonwpr_ucode_blob_size = 0U;
	} else {
		acr_fw_bin_hdr = (struct bin_hdr *)acr_fw->data;
		acr_fw_hdr = (struct acr_fw_header *)
			(acr_fw->data + acr_fw_bin_hdr->header_offset);

		acr_ucode_data = (u32 *)(acr_fw->data +
			acr_fw_bin_hdr->data_offset);
		acr_ucode_header = (u32 *)(acr_fw->data +
			acr_fw_hdr->hdr_offset);

		/* During recovery need to update blob size as 0x0*/
		acr_desc->acr_dmem_desc_v1 = (struct flcn_acr_desc_v1 *)
			((u8 *)(acr_desc->acr_ucode.cpu_va) +
			acr_ucode_header[2U]);

		/* Patch WPR info to ucode */
		acr_dmem_desc = (struct flcn_acr_desc_v1 *)
			&(((u8 *)acr_ucode_data)[acr_ucode_header[2U]]);

		acr_dmem_desc->nonwpr_ucode_blob_start =
			nvgpu_mem_get_addr(g, &g->acr.ucode_blob);
		acr_dmem_desc->nonwpr_ucode_blob_size =
			g->acr.ucode_blob.size;
		acr_dmem_desc->regions.no_regions = 1U;
		acr_dmem_desc->wpr_offset = 0U;
	}

	return 0;
}

static void gv11b_acr_default_sw_init(struct gk20a *g, struct hs_acr *hs_acr)
{
	struct hs_flcn_bl *hs_bl = &hs_acr->acr_hs_bl;

	nvgpu_log_fn(g, " ");

	hs_bl->bl_fw_name = HSBIN_ACR_BL_UCODE_IMAGE;

	hs_acr->acr_type = ACR_DEFAULT;
	hs_acr->acr_fw_name = HSBIN_ACR_UCODE_IMAGE;

	hs_acr->ptr_bl_dmem_desc = &hs_acr->bl_dmem_desc_v1;
	hs_acr->bl_dmem_desc_size = sizeof(struct flcn_bl_dmem_desc_v1);

	hs_acr->acr_flcn = &g->pmu_flcn;
	hs_acr->acr_flcn_setup_hw_and_bl_bootstrap =
		gm20b_pmu_setup_hw_and_bl_bootstrap;
}

void nvgpu_gv11b_acr_sw_init(struct gk20a *g, struct nvgpu_acr *acr)
{
	nvgpu_log_fn(g, " ");

	acr->g = g;

	acr->bootstrap_owner = LSF_FALCON_ID_PMU;
	acr->max_supported_lsfm = MAX_SUPPORTED_LSFM;

	gv11b_acr_default_sw_init(g, &acr->acr);

	acr->get_wpr_info = gm20b_wpr_info;
	acr->alloc_blob_space = gv11b_alloc_blob_space;
	acr->bootstrap_hs_acr = gm20b_bootstrap_hs_acr;
	acr->patch_wpr_info_to_ucode = gv11b_acr_patch_wpr_info_to_ucode;
	acr->acr_fill_bl_dmem_desc =
		gp106_acr_fill_bl_dmem_desc;

	acr->remove_support = gm20b_remove_acr_support;
}
