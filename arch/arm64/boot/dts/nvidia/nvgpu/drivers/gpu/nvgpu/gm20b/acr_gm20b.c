/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/acr/nvgpu_acr.h>
#include <nvgpu/firmware.h>
#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>
#include <nvgpu/enabled.h>
#include <nvgpu/mm.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include "mm_gm20b.h"
#include "pmu_gm20b.h"
#include "acr_gm20b.h"

#include <nvgpu/hw/gm20b/hw_falcon_gm20b.h>
#include <nvgpu/hw/gm20b/hw_pwr_gm20b.h>

typedef int (*get_ucode_details)(struct gk20a *g, struct flcn_ucode_img *udata);

/*Externs*/

/*Forwards*/
static int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int fecs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int gpccs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img);
static int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr *plsfm);
static int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct flcn_ucode_img *ucode_image, u32 falcon_id);
static void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img *p_img);
static void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img *p_img);
static int lsf_gen_wpr_requirements(struct gk20a *g, struct ls_flcn_mgr *plsfm);
static void lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct nvgpu_mem *nonwpr);
static void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr *plsfm);

/*Globals*/
static get_ucode_details pmu_acr_supp_ucode_list[] = {
	pmu_ucode_details,
	fecs_ucode_details,
	gpccs_ucode_details,
};

void gm20b_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf)
{
	g->ops.fb.read_wpr_info(g, inf);
}

bool gm20b_is_pmu_supported(struct gk20a *g)
{
	return true;
}

static int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct nvgpu_firmware *pmu_fw, *pmu_desc, *pmu_sig;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct lsf_ucode_desc *lsf_desc;
	int err;
	nvgpu_pmu_dbg(g, "requesting PMU ucode in GM20B\n");
	pmu_fw = nvgpu_request_firmware(g, GM20B_PMU_UCODE_IMAGE, 0);
	if (!pmu_fw) {
		nvgpu_err(g, "failed to load pmu ucode!!");
		return -ENOENT;
	}
	g->acr.pmu_fw = pmu_fw;
	nvgpu_pmu_dbg(g, "Loaded PMU ucode in for blob preparation");

	nvgpu_pmu_dbg(g, "requesting PMU ucode desc in GM20B\n");
	pmu_desc = nvgpu_request_firmware(g, GM20B_PMU_UCODE_DESC, 0);
	if (!pmu_desc) {
		nvgpu_err(g, "failed to load pmu ucode desc!!");
		err = -ENOENT;
		goto release_img_fw;
	}
	pmu_sig = nvgpu_request_firmware(g, GM20B_PMU_UCODE_SIG, 0);
	if (!pmu_sig) {
		nvgpu_err(g, "failed to load pmu sig!!");
		err = -ENOENT;
		goto release_desc;
	}
	pmu->desc = (struct pmu_ucode_desc *)pmu_desc->data;
	pmu->ucode_image = (u32 *)pmu_fw->data;
	g->acr.pmu_desc = pmu_desc;

	err = nvgpu_init_pmu_fw_support(pmu);
	if (err) {
		nvgpu_pmu_dbg(g, "failed to set function pointers\n");
		goto release_sig;
	}

	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
	if (!lsf_desc) {
		err = -ENOMEM;
		goto release_sig;
	}
	memcpy(lsf_desc, (void *)pmu_sig->data,
			min_t(size_t, sizeof(*lsf_desc), pmu_sig->size));
	lsf_desc->falcon_id = LSF_FALCON_ID_PMU;

	p_img->desc = pmu->desc;
	p_img->data = pmu->ucode_image;
	p_img->data_size = pmu->desc->image_size;
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	nvgpu_pmu_dbg(g, "requesting PMU ucode in GM20B exit\n");
	nvgpu_release_firmware(g, pmu_sig);
	return 0;
release_sig:
	nvgpu_release_firmware(g, pmu_sig);
release_desc:
	nvgpu_release_firmware(g, pmu_desc);
	g->acr.pmu_desc = NULL;
release_img_fw:
	nvgpu_release_firmware(g, pmu_fw);
	g->acr.pmu_fw = NULL;
	return err;
}

static int fecs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	struct nvgpu_firmware *fecs_sig;
	int err;

	fecs_sig = nvgpu_request_firmware(g, GM20B_FECS_UCODE_SIG, 0);
	if (!fecs_sig) {
		nvgpu_err(g, "failed to load fecs sig");
		return -ENOENT;
	}
	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)fecs_sig->data,
			min_t(size_t, sizeof(*lsf_desc), fecs_sig->size));
	lsf_desc->falcon_id = LSF_FALCON_ID_FECS;

	p_img->desc = nvgpu_kzalloc(g, sizeof(struct pmu_ucode_desc));
	if (p_img->desc == NULL) {
		err = -ENOMEM;
		goto free_lsf_desc;
	}

	p_img->desc->bootloader_start_offset =
		g->ctxsw_ucode_info.fecs.boot.offset;
	p_img->desc->bootloader_size =
		ALIGN(g->ctxsw_ucode_info.fecs.boot.size, 256);
	p_img->desc->bootloader_imem_offset =
		g->ctxsw_ucode_info.fecs.boot_imem_offset;
	p_img->desc->bootloader_entry_point =
		g->ctxsw_ucode_info.fecs.boot_entry;

	p_img->desc->image_size =
		ALIGN(g->ctxsw_ucode_info.fecs.boot.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.data.size, 256);
	p_img->desc->app_size = ALIGN(g->ctxsw_ucode_info.fecs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.fecs.data.size, 256);
	p_img->desc->app_start_offset = g->ctxsw_ucode_info.fecs.code.offset;
	p_img->desc->app_imem_offset = 0;
	p_img->desc->app_imem_entry = 0;
	p_img->desc->app_dmem_offset = 0;
	p_img->desc->app_resident_code_offset = 0;
	p_img->desc->app_resident_code_size =
		g->ctxsw_ucode_info.fecs.code.size;
	p_img->desc->app_resident_data_offset =
		g->ctxsw_ucode_info.fecs.data.offset -
		g->ctxsw_ucode_info.fecs.code.offset;
	p_img->desc->app_resident_data_size =
		g->ctxsw_ucode_info.fecs.data.size;
	p_img->data = g->ctxsw_ucode_info.surface_desc.cpu_va;
	p_img->data_size = p_img->desc->image_size;

	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	nvgpu_pmu_dbg(g, "fecs fw loaded\n");
	nvgpu_release_firmware(g, fecs_sig);
	return 0;
free_lsf_desc:
	nvgpu_kfree(g, lsf_desc);
rel_sig:
	nvgpu_release_firmware(g, fecs_sig);
	return err;
}
static int gpccs_ucode_details(struct gk20a *g, struct flcn_ucode_img *p_img)
{
	struct lsf_ucode_desc *lsf_desc;
	struct nvgpu_firmware *gpccs_sig;
	int err;

	if (!nvgpu_is_enabled(g, NVGPU_SEC_SECUREGPCCS)) {
		return -ENOENT;
	}

	gpccs_sig = nvgpu_request_firmware(g, T18x_GPCCS_UCODE_SIG, 0);
	if (!gpccs_sig) {
		nvgpu_err(g, "failed to load gpccs sig");
		return -ENOENT;
	}
	lsf_desc = nvgpu_kzalloc(g, sizeof(struct lsf_ucode_desc));
	if (!lsf_desc) {
		err = -ENOMEM;
		goto rel_sig;
	}
	memcpy(lsf_desc, (void *)gpccs_sig->data,
			min_t(size_t, sizeof(*lsf_desc), gpccs_sig->size));
	lsf_desc->falcon_id = LSF_FALCON_ID_GPCCS;

	p_img->desc = nvgpu_kzalloc(g, sizeof(struct pmu_ucode_desc));
	if (p_img->desc == NULL) {
		err = -ENOMEM;
		goto free_lsf_desc;
	}

	p_img->desc->bootloader_start_offset =
		0;
	p_img->desc->bootloader_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.boot.size, 256);
	p_img->desc->bootloader_imem_offset =
		g->ctxsw_ucode_info.gpccs.boot_imem_offset;
	p_img->desc->bootloader_entry_point =
		g->ctxsw_ucode_info.gpccs.boot_entry;

	p_img->desc->image_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.boot.size, 256) +
		ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256) +
		ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->desc->app_size = ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256)
		+ ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->desc->app_start_offset = p_img->desc->bootloader_size;
	p_img->desc->app_imem_offset = 0;
	p_img->desc->app_imem_entry = 0;
	p_img->desc->app_dmem_offset = 0;
	p_img->desc->app_resident_code_offset = 0;
	p_img->desc->app_resident_code_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.code.size, 256);
	p_img->desc->app_resident_data_offset =
		ALIGN(g->ctxsw_ucode_info.gpccs.data.offset, 256) -
		ALIGN(g->ctxsw_ucode_info.gpccs.code.offset, 256);
	p_img->desc->app_resident_data_size =
		ALIGN(g->ctxsw_ucode_info.gpccs.data.size, 256);
	p_img->data = (u32 *)((u8 *)g->ctxsw_ucode_info.surface_desc.cpu_va +
		g->ctxsw_ucode_info.gpccs.boot.offset);
	p_img->data_size = ALIGN(p_img->desc->image_size, 256);
	p_img->fw_ver = NULL;
	p_img->header = NULL;
	p_img->lsf_desc = (struct lsf_ucode_desc *)lsf_desc;
	nvgpu_pmu_dbg(g, "gpccs fw loaded\n");
	nvgpu_release_firmware(g, gpccs_sig);
	return 0;
free_lsf_desc:
	nvgpu_kfree(g, lsf_desc);
rel_sig:
	nvgpu_release_firmware(g, gpccs_sig);
	return err;
}

bool gm20b_is_lazy_bootstrap(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = false;
		break;
	default:
		break;
	}

	return enable_status;
}

bool gm20b_is_priv_load(u32 falcon_id)
{
	bool enable_status = false;

	switch (falcon_id) {
	case LSF_FALCON_ID_FECS:
		enable_status = false;
		break;
	case LSF_FALCON_ID_GPCCS:
		enable_status = false;
		break;
	default:
		break;
	}

	return enable_status;
}

int gm20b_alloc_blob_space(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem)
{
	int err;

	err = nvgpu_dma_alloc_sys(g, size, mem);

	return err;
}

int prepare_ucode_blob(struct gk20a *g)
{

	int err;
	struct ls_flcn_mgr lsfm_l, *plsfm;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct wpr_carveout_info wpr_inf;

	if (g->acr.ucode_blob.cpu_va) {
		/*Recovery case, we do not need to form
		non WPR blob of ucodes*/
		err = nvgpu_init_pmu_fw_support(pmu);
		if (err) {
			nvgpu_pmu_dbg(g, "failed to set function pointers\n");
			return err;
		}
		return 0;
	}
	plsfm = &lsfm_l;
	memset((void *)plsfm, 0, sizeof(struct ls_flcn_mgr));
	nvgpu_pmu_dbg(g, "fetching GMMU regs\n");
	g->ops.fb.vpr_info_fetch(g);
	gr_gk20a_init_ctxsw_ucode(g);

	g->acr.get_wpr_info(g, &wpr_inf);
	nvgpu_pmu_dbg(g, "wpr carveout base:%llx\n", wpr_inf.wpr_base);
	nvgpu_pmu_dbg(g, "wpr carveout size :%llx\n", wpr_inf.size);

	/* Discover all managed falcons*/
	err = lsfm_discover_ucode_images(g, plsfm);
	nvgpu_pmu_dbg(g, " Managed Falcon cnt %d\n", plsfm->managed_flcn_cnt);
	if (err) {
		goto free_sgt;
	}

	if (plsfm->managed_flcn_cnt && !g->acr.ucode_blob.cpu_va) {
		/* Generate WPR requirements*/
		err = lsf_gen_wpr_requirements(g, plsfm);
		if (err) {
			goto free_sgt;
		}

		/*Alloc memory to hold ucode blob contents*/
		err = g->acr.alloc_blob_space(g, plsfm->wpr_size
				, &g->acr.ucode_blob);
		if (err) {
			goto free_sgt;
		}

		nvgpu_pmu_dbg(g, "managed LS falcon %d, WPR size %d bytes.\n",
			plsfm->managed_flcn_cnt, plsfm->wpr_size);
		lsfm_init_wpr_contents(g, plsfm, &g->acr.ucode_blob);
	} else {
		nvgpu_pmu_dbg(g, "LSFM is managing no falcons.\n");
	}
	nvgpu_pmu_dbg(g, "prepare ucode blob return 0\n");
	free_acr_resources(g, plsfm);
free_sgt:
	return err;
}

static u8 lsfm_falcon_disabled(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	u32 falcon_id)
{
	return (plsfm->disable_mask >> falcon_id) & 0x1;
}

/* Discover all managed falcon ucode images */
static int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr *plsfm)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct flcn_ucode_img ucode_img;
	u32 falcon_id;
	u32 i;
	int status;

	/* LSFM requires a secure PMU, discover it first.*/
	/* Obtain the PMU ucode image and add it to the list if required*/
	memset(&ucode_img, 0, sizeof(ucode_img));
	status = pmu_ucode_details(g, &ucode_img);
	if (status) {
		return status;
	}

	/* The falon_id is formed by grabbing the static base
	 * falon_id from the image and adding the
	 * engine-designated falcon instance.*/
	pmu->pmu_mode |= PMU_SECURE_MODE;
	falcon_id = ucode_img.lsf_desc->falcon_id +
		ucode_img.flcn_inst;

	if (!lsfm_falcon_disabled(g, plsfm, falcon_id)) {
		pmu->falcon_id = falcon_id;
		if (lsfm_add_ucode_img(g, plsfm, &ucode_img,
				pmu->falcon_id) == 0) {
			pmu->pmu_mode |= PMU_LSFM_MANAGED;
		}

		plsfm->managed_flcn_cnt++;
	} else {
		nvgpu_pmu_dbg(g, "id not managed %d\n",
			ucode_img.lsf_desc->falcon_id);
	}

	/*Free any ucode image resources if not managing this falcon*/
	if (!(pmu->pmu_mode & PMU_LSFM_MANAGED)) {
		nvgpu_pmu_dbg(g, "pmu is not LSFM managed\n");
		lsfm_free_ucode_img_res(g, &ucode_img);
	}

	/* Enumerate all constructed falcon objects,
	 as we need the ucode image info and total falcon count.*/

	/*0th index is always PMU which is already handled in earlier
	if condition*/
	for (i = 1; i < g->acr.max_supported_lsfm; i++) {
		memset(&ucode_img, 0, sizeof(ucode_img));
		if (pmu_acr_supp_ucode_list[i](g, &ucode_img) == 0) {
			if (ucode_img.lsf_desc != NULL) {
				/* We have engine sigs, ensure that this falcon
				is aware of the secure mode expectations
				(ACR status)*/

				/* falon_id is formed by grabbing the static
				base falonId from the image and adding the
				engine-designated falcon instance. */
				falcon_id = ucode_img.lsf_desc->falcon_id +
					ucode_img.flcn_inst;

				if (!lsfm_falcon_disabled(g, plsfm,
					falcon_id)) {
					/* Do not manage non-FB ucode*/
					if (lsfm_add_ucode_img(g,
						plsfm, &ucode_img, falcon_id)
						== 0) {
						plsfm->managed_flcn_cnt++;
					}
				} else {
					nvgpu_pmu_dbg(g, "not managed %d\n",
						ucode_img.lsf_desc->falcon_id);
					lsfm_free_nonpmu_ucode_img_res(g,
						&ucode_img);
				}
			}
		} else {
			/* Consumed all available falcon objects */
			nvgpu_pmu_dbg(g, "Done checking for ucodes %d\n", i);
			break;
		}
	}
	return 0;
}


int gm20b_pmu_populate_loader_cfg(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size)
{
	struct wpr_carveout_info wpr_inf;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct lsfm_managed_ucode_img *p_lsfm =
			(struct lsfm_managed_ucode_img *)lsfm;
	struct flcn_ucode_img *p_img = &(p_lsfm->ucode_img);
	struct loader_config *ldr_cfg = &(p_lsfm->bl_gen_desc.loader_cfg);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;
	u32 addr_args;

	if (p_img->desc == NULL) {
		/*
		 * This means its a header based ucode,
		 * and so we do not fill BL gen desc structure
		 */
		return -EINVAL;
	}
	desc = p_img->desc;
	/*
	 * Calculate physical and virtual addresses for various portions of
	 * the PMU ucode image
	 * Calculate the 32-bit addresses for the application code, application
	 * data, and bootloader code. These values are all based on IM_BASE.
	 * The 32-bit addresses will be the upper 32-bits of the virtual or
	 * physical addresses of each respective segment.
	 */
	addr_base = p_lsfm->lsb_header.ucode_off;
	g->acr.get_wpr_info(g, &wpr_inf);
	addr_base += wpr_inf.wpr_base;
	nvgpu_pmu_dbg(g, "pmu loader cfg u32 addrbase %x\n", (u32)addr_base);
	/*From linux*/
	addr_code = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	nvgpu_pmu_dbg(g, "app start %d app res code off %d\n",
		desc->app_start_offset, desc->app_resident_code_offset);
	addr_data = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);
	nvgpu_pmu_dbg(g, "app res data offset%d\n",
		desc->app_resident_data_offset);
	nvgpu_pmu_dbg(g, "bl start off %d\n", desc->bootloader_start_offset);

	addr_args = ((pwr_falcon_hwcfg_dmem_size_v(
			gk20a_readl(g, pwr_falcon_hwcfg_r())))
			<< GK20A_PMU_DMEM_BLKSIZE2);
	addr_args -= g->ops.pmu_ver.get_pmu_cmdline_args_size(pmu);

	nvgpu_pmu_dbg(g, "addr_args %x\n", addr_args);

	/* Populate the loader_config state*/
	ldr_cfg->dma_idx = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->code_dma_base1 = 0x0;
	ldr_cfg->code_size_total = desc->app_size;
	ldr_cfg->code_size_to_load = desc->app_resident_code_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_dma_base1 = 0;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->overlay_dma_base = addr_code;
	ldr_cfg->overlay_dma_base1 = 0x0;

	/* Update the argc/argv members*/
	ldr_cfg->argc = 1;
	ldr_cfg->argv = addr_args;

	*p_bl_gen_desc_size = sizeof(struct loader_config);
	g->acr.pmu_args = addr_args;
	return 0;
}

int gm20b_flcn_populate_bl_dmem_desc(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size, u32 falconid)
{
	struct wpr_carveout_info wpr_inf;
	struct lsfm_managed_ucode_img *p_lsfm =
			(struct lsfm_managed_ucode_img *)lsfm;
	struct flcn_ucode_img *p_img = &(p_lsfm->ucode_img);
	struct flcn_bl_dmem_desc *ldr_cfg =
			&(p_lsfm->bl_gen_desc.bl_dmem_desc);
	u64 addr_base;
	struct pmu_ucode_desc *desc;
	u64 addr_code, addr_data;

	if (p_img->desc == NULL) {
		/*
		 * This means its a header based ucode,
		 * and so we do not fill BL gen desc structure
		 */
		return -EINVAL;
	}
	desc = p_img->desc;

	/*
	 * Calculate physical and virtual addresses for various portions of
	 * the PMU ucode image
	 * Calculate the 32-bit addresses for the application code, application
	 * data, and bootloader code. These values are all based on IM_BASE.
	 * The 32-bit addresses will be the upper 32-bits of the virtual or
	 * physical addresses of each respective segment.
	 */
	addr_base = p_lsfm->lsb_header.ucode_off;
	g->acr.get_wpr_info(g, &wpr_inf);
	addr_base += wpr_inf.wpr_base;

	nvgpu_pmu_dbg(g, "gen loader cfg %x u32 addrbase %x ID\n", (u32)addr_base,
			p_lsfm->wpr_header.falcon_id);
	addr_code = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_code_offset) >> 8);
	addr_data = u64_lo32((addr_base +
				desc->app_start_offset +
				desc->app_resident_data_offset) >> 8);

	nvgpu_pmu_dbg(g, "gen cfg %x u32 addrcode %x & data %x load offset %xID\n",
		(u32)addr_code, (u32)addr_data, desc->bootloader_start_offset,
		p_lsfm->wpr_header.falcon_id);

	/* Populate the LOADER_CONFIG state */
	memset((void *) ldr_cfg, 0, sizeof(struct flcn_bl_dmem_desc));
	ldr_cfg->ctx_dma = GK20A_PMU_DMAIDX_UCODE;
	ldr_cfg->code_dma_base = addr_code;
	ldr_cfg->non_sec_code_size = desc->app_resident_code_size;
	ldr_cfg->data_dma_base = addr_data;
	ldr_cfg->data_size = desc->app_resident_data_size;
	ldr_cfg->code_entry_point = desc->app_imem_entry;
	*p_bl_gen_desc_size = sizeof(struct flcn_bl_dmem_desc);
	return 0;
}

/* Populate falcon boot loader generic desc.*/
static int lsfm_fill_flcn_bl_gen_desc(struct gk20a *g,
		struct lsfm_managed_ucode_img *pnode)
{

	struct nvgpu_pmu *pmu = &g->pmu;
	if (pnode->wpr_header.falcon_id != pmu->falcon_id) {
		nvgpu_pmu_dbg(g, "non pmu. write flcn bl gen desc\n");
		g->ops.pmu.flcn_populate_bl_dmem_desc(g,
				pnode, &pnode->bl_gen_desc_size,
				pnode->wpr_header.falcon_id);
		return 0;
	}

	if (pmu->pmu_mode & PMU_LSFM_MANAGED) {
		nvgpu_pmu_dbg(g, "pmu write flcn bl gen desc\n");
		if (pnode->wpr_header.falcon_id == pmu->falcon_id) {
			return g->ops.pmu.pmu_populate_loader_cfg(g, pnode,
				&pnode->bl_gen_desc_size);
		}
	}

	/* Failed to find the falcon requested. */
	return -ENOENT;
}

/* Initialize WPR contents */
static void lsfm_init_wpr_contents(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct nvgpu_mem *ucode)
{
	struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
	struct lsf_wpr_header last_wpr_hdr;
	u32 i;

	/* The WPR array is at the base of the WPR */
	pnode = plsfm->ucode_img_list;
	memset(&last_wpr_hdr, 0, sizeof(struct lsf_wpr_header));
	i = 0;

	/*
	 * Walk the managed falcons, flush WPR and LSB headers to FB.
	 * flush any bl args to the storage area relative to the
	 * ucode image (appended on the end as a DMEM area).
	 */
	while (pnode) {
		/* Flush WPR header to memory*/
		nvgpu_mem_wr_n(g, ucode, i * sizeof(pnode->wpr_header),
				&pnode->wpr_header, sizeof(pnode->wpr_header));

		nvgpu_pmu_dbg(g, "wpr header");
		nvgpu_pmu_dbg(g, "falconid :%d",
				pnode->wpr_header.falcon_id);
		nvgpu_pmu_dbg(g, "lsb_offset :%x",
				pnode->wpr_header.lsb_offset);
		nvgpu_pmu_dbg(g, "bootstrap_owner :%d",
			pnode->wpr_header.bootstrap_owner);
		nvgpu_pmu_dbg(g, "lazy_bootstrap :%d",
				pnode->wpr_header.lazy_bootstrap);
		nvgpu_pmu_dbg(g, "status :%d",
				pnode->wpr_header.status);

		/*Flush LSB header to memory*/
		nvgpu_mem_wr_n(g, ucode, pnode->wpr_header.lsb_offset,
				&pnode->lsb_header, sizeof(pnode->lsb_header));

		nvgpu_pmu_dbg(g, "lsb header");
		nvgpu_pmu_dbg(g, "ucode_off :%x",
				pnode->lsb_header.ucode_off);
		nvgpu_pmu_dbg(g, "ucode_size :%x",
				pnode->lsb_header.ucode_size);
		nvgpu_pmu_dbg(g, "data_size :%x",
				pnode->lsb_header.data_size);
		nvgpu_pmu_dbg(g, "bl_code_size :%x",
				pnode->lsb_header.bl_code_size);
		nvgpu_pmu_dbg(g, "bl_imem_off :%x",
				pnode->lsb_header.bl_imem_off);
		nvgpu_pmu_dbg(g, "bl_data_off :%x",
				pnode->lsb_header.bl_data_off);
		nvgpu_pmu_dbg(g, "bl_data_size :%x",
				pnode->lsb_header.bl_data_size);
		nvgpu_pmu_dbg(g, "app_code_off :%x",
				pnode->lsb_header.app_code_off);
		nvgpu_pmu_dbg(g, "app_code_size :%x",
				pnode->lsb_header.app_code_size);
		nvgpu_pmu_dbg(g, "app_data_off :%x",
				pnode->lsb_header.app_data_off);
		nvgpu_pmu_dbg(g, "app_data_size :%x",
				pnode->lsb_header.app_data_size);
		nvgpu_pmu_dbg(g, "flags :%x",
				pnode->lsb_header.flags);

		/*If this falcon has a boot loader and related args,
		 * flush them.*/
		if (!pnode->ucode_img.header) {
			/*Populate gen bl and flush to memory*/
			lsfm_fill_flcn_bl_gen_desc(g, pnode);
			nvgpu_mem_wr_n(g, ucode,
					pnode->lsb_header.bl_data_off,
					&pnode->bl_gen_desc,
					pnode->bl_gen_desc_size);
		}
		/*Copying of ucode*/
		nvgpu_mem_wr_n(g, ucode, pnode->lsb_header.ucode_off,
				pnode->ucode_img.data,
				pnode->ucode_img.data_size);
		pnode = pnode->next;
		i++;
	}

	/* Tag the terminator WPR header with an invalid falcon ID. */
	last_wpr_hdr.falcon_id = LSF_FALCON_ID_INVALID;
	nvgpu_mem_wr_n(g, ucode,
			plsfm->managed_flcn_cnt * sizeof(struct lsf_wpr_header),
			&last_wpr_hdr,
			sizeof(struct lsf_wpr_header));
}

/*!
 * lsfm_parse_no_loader_ucode: parses UCODE header of falcon
 *
 * @param[in] p_ucodehdr : UCODE header
 * @param[out] lsb_hdr : updates values in LSB header
 *
 * @return 0
 */
static int lsfm_parse_no_loader_ucode(u32 *p_ucodehdr,
	struct lsf_lsb_header *lsb_hdr)
{

	u32 code_size = 0;
	u32 data_size = 0;
	u32 i = 0;
	u32 total_apps = p_ucodehdr[FLCN_NL_UCODE_HDR_NUM_APPS_IND];

	/* Lets calculate code size*/
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_CODE_SIZE_IND
			(total_apps, i)];
	}
	code_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_OVL_SIZE_IND(total_apps)];

	/* Calculate data size*/
	data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	for (i = 0; i < total_apps; i++) {
		data_size += p_ucodehdr[FLCN_NL_UCODE_HDR_APP_DATA_SIZE_IND
			(total_apps, i)];
	}

	lsb_hdr->ucode_size = code_size;
	lsb_hdr->data_size = data_size;
	lsb_hdr->bl_code_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND];
	lsb_hdr->bl_imem_off = 0;
	lsb_hdr->bl_data_off = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_OFF_IND];
	lsb_hdr->bl_data_size = p_ucodehdr[FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND];
	return 0;
}

/*!
 * @brief lsfm_fill_static_lsb_hdr_info
 * Populate static LSB header infomation using the provided ucode image
 */
static void lsfm_fill_static_lsb_hdr_info(struct gk20a *g,
	u32 falcon_id, struct lsfm_managed_ucode_img *pnode)
{

	struct nvgpu_pmu *pmu = &g->pmu;
	u32 full_app_size = 0;
	u32 data = 0;

	if (pnode->ucode_img.lsf_desc) {
		memcpy(&pnode->lsb_header.signature, pnode->ucode_img.lsf_desc,
			sizeof(struct lsf_ucode_desc));
	}
	pnode->lsb_header.ucode_size = pnode->ucode_img.data_size;

	/* The remainder of the LSB depends on the loader usage */
	if (pnode->ucode_img.header) {
		/* Does not use a loader */
		pnode->lsb_header.data_size = 0;
		pnode->lsb_header.bl_code_size = 0;
		pnode->lsb_header.bl_data_off = 0;
		pnode->lsb_header.bl_data_size = 0;

		lsfm_parse_no_loader_ucode(pnode->ucode_img.header,
			&(pnode->lsb_header));

		/* Load the first 256 bytes of IMEM. */
		/* Set LOAD_CODE_AT_0 and DMACTL_REQ_CTX.
		True for all method based falcons */
		data = NV_FLCN_ACR_LSF_FLAG_LOAD_CODE_AT_0_TRUE |
			NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
		pnode->lsb_header.flags = data;
	} else {
		/* Uses a loader. that is has a desc */
		pnode->lsb_header.data_size = 0;

		/* The loader code size is already aligned (padded) such that
		the code following it is aligned, but the size in the image
		desc is not, bloat it up to be on a 256 byte alignment. */
		pnode->lsb_header.bl_code_size = ALIGN(
			pnode->ucode_img.desc->bootloader_size,
			LSF_BL_CODE_SIZE_ALIGNMENT);
		full_app_size = ALIGN(pnode->ucode_img.desc->app_size,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.ucode_size = ALIGN(
			pnode->ucode_img.desc->app_resident_data_offset,
			LSF_BL_CODE_SIZE_ALIGNMENT) +
			pnode->lsb_header.bl_code_size;
		pnode->lsb_header.data_size = full_app_size -
			pnode->lsb_header.ucode_size;
		/* Though the BL is located at 0th offset of the image, the VA
		is different to make sure that it doesnt collide the actual OS
		VA range */
		pnode->lsb_header.bl_imem_off =
			pnode->ucode_img.desc->bootloader_imem_offset;

		/* TODO: OBJFLCN should export properties using which the below
			flags should be populated.*/
		pnode->lsb_header.flags = 0;

		if (falcon_id == pmu->falcon_id) {
			data = NV_FLCN_ACR_LSF_FLAG_DMACTL_REQ_CTX_TRUE;
			pnode->lsb_header.flags = data;
		}

		if (g->ops.pmu.is_priv_load(falcon_id)) {
			pnode->lsb_header.flags |=
				NV_FLCN_ACR_LSF_FLAG_FORCE_PRIV_LOAD_TRUE;
		}
	}
}

/* Adds a ucode image to the list of managed ucode images managed. */
static int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr *plsfm,
	struct flcn_ucode_img *ucode_image, u32 falcon_id)
{

	struct lsfm_managed_ucode_img *pnode;
	pnode = nvgpu_kzalloc(g, sizeof(struct lsfm_managed_ucode_img));
	if (pnode == NULL) {
		return -ENOMEM;
	}

	/* Keep a copy of the ucode image info locally */
	memcpy(&pnode->ucode_img, ucode_image, sizeof(struct flcn_ucode_img));

	/* Fill in static WPR header info*/
	pnode->wpr_header.falcon_id = falcon_id;
	pnode->wpr_header.bootstrap_owner = g->acr.bootstrap_owner;
	pnode->wpr_header.status = LSF_IMAGE_STATUS_COPY;

	pnode->wpr_header.lazy_bootstrap =
			g->ops.pmu.is_lazy_bootstrap(falcon_id);

	/*TODO to check if PDB_PROP_FLCN_LAZY_BOOTSTRAP is to be supported by
	Android */
	/* Fill in static LSB header info elsewhere */
	lsfm_fill_static_lsb_hdr_info(g, falcon_id, pnode);
	pnode->next = plsfm->ucode_img_list;
	plsfm->ucode_img_list = pnode;
	return 0;
}

/* Free any ucode image structure resources. */
static void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		nvgpu_kfree(g, p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
}

/* Free any ucode image structure resources. */
static void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img *p_img)
{
	if (p_img->lsf_desc != NULL) {
		nvgpu_kfree(g, p_img->lsf_desc);
		p_img->lsf_desc = NULL;
	}
	if (p_img->desc != NULL) {
		nvgpu_kfree(g, p_img->desc);
		p_img->desc = NULL;
	}
}

static void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr *plsfm)
{
	u32 cnt = plsfm->managed_flcn_cnt;
	struct lsfm_managed_ucode_img *mg_ucode_img;
	while (cnt) {
		mg_ucode_img = plsfm->ucode_img_list;
		if (mg_ucode_img->ucode_img.lsf_desc->falcon_id ==
				LSF_FALCON_ID_PMU) {
			lsfm_free_ucode_img_res(g, &mg_ucode_img->ucode_img);
		} else {
			lsfm_free_nonpmu_ucode_img_res(g,
				&mg_ucode_img->ucode_img);
		}
		plsfm->ucode_img_list = mg_ucode_img->next;
		nvgpu_kfree(g, mg_ucode_img);
		cnt--;
	}
}

/* Generate WPR requirements for ACR allocation request */
static int lsf_gen_wpr_requirements(struct gk20a *g, struct ls_flcn_mgr *plsfm)
{
	struct lsfm_managed_ucode_img *pnode = plsfm->ucode_img_list;
	u32 wpr_offset;

	/* Calculate WPR size required */

	/* Start with an array of WPR headers at the base of the WPR.
	 The expectation here is that the secure falcon will do a single DMA
	 read of this array and cache it internally so it's OK to pack these.
	 Also, we add 1 to the falcon count to indicate the end of the array.*/
	wpr_offset = sizeof(struct lsf_wpr_header) *
		(plsfm->managed_flcn_cnt+1);

	/* Walk the managed falcons, accounting for the LSB structs
	as well as the ucode images. */
	while (pnode) {
		/* Align, save off, and include an LSB header size */
		wpr_offset = ALIGN(wpr_offset,
			LSF_LSB_HEADER_ALIGNMENT);
		pnode->wpr_header.lsb_offset = wpr_offset;
		wpr_offset += sizeof(struct lsf_lsb_header);

		/* Align, save off, and include the original (static)
		ucode image size */
		wpr_offset = ALIGN(wpr_offset,
			LSF_UCODE_DATA_ALIGNMENT);
		pnode->lsb_header.ucode_off = wpr_offset;
		wpr_offset += pnode->ucode_img.data_size;

		/* For falcons that use a boot loader (BL), we append a loader
		desc structure on the end of the ucode image and consider this
		the boot loader data. The host will then copy the loader desc
		args to this space within the WPR region (before locking down)
		and the HS bin will then copy them to DMEM 0 for the loader. */
		if (!pnode->ucode_img.header) {
			/* Track the size for LSB details filled in later
			 Note that at this point we don't know what kind of i
			boot loader desc, so we just take the size of the
			generic one, which is the largest it will will ever be.
			*/
			/* Align (size bloat) and save off generic
			descriptor size*/
			pnode->lsb_header.bl_data_size = ALIGN(
				sizeof(pnode->bl_gen_desc),
				LSF_BL_DATA_SIZE_ALIGNMENT);

			/*Align, save off, and include the additional BL data*/
			wpr_offset = ALIGN(wpr_offset,
				LSF_BL_DATA_ALIGNMENT);
			pnode->lsb_header.bl_data_off = wpr_offset;
			wpr_offset += pnode->lsb_header.bl_data_size;
		} else {
			/* bl_data_off is already assigned in static
			information. But that is from start of the image */
			pnode->lsb_header.bl_data_off +=
				(wpr_offset - pnode->ucode_img.data_size);
		}

		/* Finally, update ucode surface size to include updates */
		pnode->full_ucode_size = wpr_offset -
			pnode->lsb_header.ucode_off;
		if (pnode->wpr_header.falcon_id != LSF_FALCON_ID_PMU) {
			pnode->lsb_header.app_code_off =
				pnode->lsb_header.bl_code_size;
			pnode->lsb_header.app_code_size =
				pnode->lsb_header.ucode_size -
				pnode->lsb_header.bl_code_size;
			pnode->lsb_header.app_data_off =
				pnode->lsb_header.ucode_size;
			pnode->lsb_header.app_data_size =
				pnode->lsb_header.data_size;
		}
		pnode = pnode->next;
	}
	plsfm->wpr_size = wpr_offset;
	return 0;
}

/*
 * @brief Patch signatures into ucode image
 */
int acr_ucode_patch_sig(struct gk20a *g,
		unsigned int *p_img,
		unsigned int *p_prod_sig,
		unsigned int *p_dbg_sig,
		unsigned int *p_patch_loc,
		unsigned int *p_patch_ind)
{
	unsigned int i, *p_sig;
	nvgpu_pmu_dbg(g, " ");

	if (!g->ops.pmu.is_debug_mode_enabled(g)) {
		p_sig = p_prod_sig;
		nvgpu_pmu_dbg(g, "PRODUCTION MODE\n");
	} else {
		p_sig = p_dbg_sig;
		nvgpu_pmu_dbg(g, "DEBUG MODE\n");
	}

	/* Patching logic:*/
	for (i = 0; i < sizeof(*p_patch_loc)>>2; i++) {
		p_img[(p_patch_loc[i]>>2)] = p_sig[(p_patch_ind[i]<<2)];
		p_img[(p_patch_loc[i]>>2)+1] = p_sig[(p_patch_ind[i]<<2)+1];
		p_img[(p_patch_loc[i]>>2)+2] = p_sig[(p_patch_ind[i]<<2)+2];
		p_img[(p_patch_loc[i]>>2)+3] = p_sig[(p_patch_ind[i]<<2)+3];
	}
	return 0;
}

static int nvgpu_gm20b_acr_wait_for_completion(struct gk20a *g,
	struct nvgpu_falcon *flcn, unsigned int timeout)
{
	u32 base_addr = flcn->flcn_base;
	int completion = 0;
	u32 data = 0;

	nvgpu_log_fn(g, " ");

	completion = nvgpu_flcn_wait_for_halt(flcn, timeout);
	if (completion != 0U) {
		nvgpu_err(g, "flcn-%d: ACR boot timed out", flcn->flcn_id);
		goto exit;
	}

	nvgpu_pmu_dbg(g, "flcn-%d: ACR capabilities %x\n", flcn->flcn_id,
		nvgpu_flcn_mailbox_read(flcn, FALCON_MAILBOX_1));

	data = nvgpu_flcn_mailbox_read(flcn, FALCON_MAILBOX_0);
	if (data != 0U) {
		nvgpu_err(g, "flcn-%d: ACR boot failed, err %x", flcn->flcn_id,
			data);
		completion = -EAGAIN;
		goto exit;
	}

	nvgpu_pmu_dbg(g, "flcn-%d: sctl reg %x", flcn->flcn_id,
		gk20a_readl(g, base_addr + falcon_falcon_sctl_r()));
	nvgpu_pmu_dbg(g, "flcn-%d: cpuctl reg %x", flcn->flcn_id,
		gk20a_readl(g, base_addr + falcon_falcon_cpuctl_r()));

exit:
	return completion;
}

static int gm20b_acr_hs_bl_exec(struct gk20a *g, struct nvgpu_acr *acr,
	struct hs_acr *acr_desc, bool b_wait_for_halt)
{
	struct nvgpu_firmware *hs_bl_fw = acr_desc->acr_hs_bl.hs_bl_fw;
	struct hsflcn_bl_desc *hs_bl_desc;
	struct nvgpu_falcon_bl_info bl_info;
	struct hs_flcn_bl *hs_bl = &acr_desc->acr_hs_bl;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	u32 *hs_bl_code = NULL;
	int err = 0;
	u32 bl_sz;

	nvgpu_pmu_dbg(g, "Executing ACR HS Bootloader %s on Falcon-ID - %d",
		hs_bl->bl_fw_name, acr_desc->acr_flcn->flcn_id);

	if (hs_bl_fw == NULL) {
		hs_bl_fw = nvgpu_request_firmware(g, hs_bl->bl_fw_name, 0U);
		if (hs_bl_fw == NULL) {
			nvgpu_err(g, "ACR HS BL ucode load fail");
			return -ENOENT;
		}

		hs_bl->hs_bl_fw = hs_bl_fw;
		hs_bl->hs_bl_bin_hdr = (struct bin_hdr *)hs_bl_fw->data;
		hs_bl->hs_bl_desc = (struct hsflcn_bl_desc *)(hs_bl_fw->data +
			hs_bl->hs_bl_bin_hdr->header_offset);

		hs_bl_desc = hs_bl->hs_bl_desc;
		hs_bl_code = (u32 *)(hs_bl_fw->data +
			hs_bl->hs_bl_bin_hdr->data_offset);

		bl_sz = ALIGN(hs_bl_desc->bl_img_hdr.bl_code_size, 256U);

		hs_bl->hs_bl_ucode.size = bl_sz;

		err = nvgpu_dma_alloc_sys(g, bl_sz, &hs_bl->hs_bl_ucode);
		if (err) {
			nvgpu_err(g, "ACR HS BL failed to allocate memory");
			goto err_done;
		}

		hs_bl->hs_bl_ucode.gpu_va = nvgpu_gmmu_map(vm,
			&hs_bl->hs_bl_ucode,
			bl_sz,
			0U, /* flags */
			gk20a_mem_flag_read_only, false,
			hs_bl->hs_bl_ucode.aperture);
		if (hs_bl->hs_bl_ucode.gpu_va == 0U) {
			nvgpu_err(g, "ACR HS BL failed to map ucode memory!!");
			goto err_free_ucode;
		}

		nvgpu_mem_wr_n(g, &hs_bl->hs_bl_ucode, 0U, hs_bl_code, bl_sz);

		nvgpu_pmu_dbg(g, "Copied BL ucode to bl_cpuva");
	}

	/* Fill HS BL info */
	bl_info.bl_src = hs_bl->hs_bl_ucode.cpu_va;
	bl_info.bl_desc = acr_desc->ptr_bl_dmem_desc;
	bl_info.bl_desc_size = acr_desc->bl_dmem_desc_size;
	bl_info.bl_size = hs_bl->hs_bl_ucode.size;
	bl_info.bl_start_tag = hs_bl->hs_bl_desc->bl_start_tag;

	/*
	 * 1. Dose falcon reset
	 * 2. setup falcon apertures
	 * 3. bootstrap falcon
	 */
	acr_desc->acr_flcn_setup_hw_and_bl_bootstrap(g, acr_desc, &bl_info);

	if (b_wait_for_halt) {
		/* wait for ACR halt*/
		err = nvgpu_gm20b_acr_wait_for_completion(g, acr_desc->acr_flcn,
			ACR_COMPLETION_TIMEOUT_MS);
		if (err != 0U) {
			goto err_unmap_bl;
		}
	}

	return 0U;
err_unmap_bl:
	nvgpu_gmmu_unmap(vm, &hs_bl->hs_bl_ucode, hs_bl->hs_bl_ucode.gpu_va);
err_free_ucode:
	nvgpu_dma_free(g, &hs_bl->hs_bl_ucode);
err_done:
	nvgpu_release_firmware(g, hs_bl_fw);
	acr_desc->acr_hs_bl.hs_bl_fw = NULL;

	return err;
}

int gm20b_acr_patch_wpr_info_to_ucode(struct gk20a *g,
	struct nvgpu_acr *acr, struct hs_acr *acr_desc, bool is_recovery)
{
	struct nvgpu_firmware *acr_fw = acr_desc->acr_fw;
	struct acr_fw_header *acr_fw_hdr = NULL;
	struct bin_hdr *acr_fw_bin_hdr = NULL;
	struct flcn_acr_desc *acr_dmem_desc;
	u32 *acr_ucode_header = NULL;
	u32 *acr_ucode_data = NULL;

	nvgpu_log_fn(g, " ");

	if (is_recovery) {
		acr_desc->acr_dmem_desc->nonwpr_ucode_blob_size = 0U;
	} else {
		acr_fw_bin_hdr = (struct bin_hdr *)acr_fw->data;
		acr_fw_hdr = (struct acr_fw_header *)
			(acr_fw->data + acr_fw_bin_hdr->header_offset);

		acr_ucode_data = (u32 *)(acr_fw->data +
			acr_fw_bin_hdr->data_offset);

		acr_ucode_header = (u32 *)(acr_fw->data +
			acr_fw_hdr->hdr_offset);

		/* During recovery need to update blob size as 0x0*/
		acr_desc->acr_dmem_desc = (struct flcn_acr_desc *)((u8 *)(
			acr_desc->acr_ucode.cpu_va) + acr_ucode_header[2U]);

		/* Patch WPR info to ucode */
		acr_dmem_desc = (struct flcn_acr_desc *)
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

int gm20b_acr_fill_bl_dmem_desc(struct gk20a *g,
	struct nvgpu_acr *acr, struct hs_acr *acr_desc,
	u32 *acr_ucode_header)
{
	struct flcn_bl_dmem_desc *bl_dmem_desc = &acr_desc->bl_dmem_desc;

	nvgpu_log_fn(g, " ");

	memset(bl_dmem_desc, 0U, sizeof(struct flcn_bl_dmem_desc));

	bl_dmem_desc->signature[0] = 0U;
	bl_dmem_desc->signature[1] = 0U;
	bl_dmem_desc->signature[2] = 0U;
	bl_dmem_desc->signature[3] = 0U;
	bl_dmem_desc->ctx_dma = GK20A_PMU_DMAIDX_VIRT;
	bl_dmem_desc->code_dma_base =
		(unsigned int)(((u64)acr_desc->acr_ucode.gpu_va >> 8U));
	bl_dmem_desc->code_dma_base1 = 0x0U;
	bl_dmem_desc->non_sec_code_off  = acr_ucode_header[0U];
	bl_dmem_desc->non_sec_code_size = acr_ucode_header[1U];
	bl_dmem_desc->sec_code_off = acr_ucode_header[5U];
	bl_dmem_desc->sec_code_size = acr_ucode_header[6U];
	bl_dmem_desc->code_entry_point = 0U; /* Start at 0th offset */
	bl_dmem_desc->data_dma_base =
		bl_dmem_desc->code_dma_base +
		((acr_ucode_header[2U]) >> 8U);
	bl_dmem_desc->data_dma_base1 = 0x0U;
	bl_dmem_desc->data_size = acr_ucode_header[3U];

	return 0;
}

/*
 * Loads ACR bin to SYSMEM/FB and bootstraps ACR with bootloader code
 * start and end are addresses of ucode blob in non-WPR region
 */
int gm20b_bootstrap_hs_acr(struct gk20a *g, struct nvgpu_acr *acr,
	struct hs_acr *acr_desc)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;
	struct nvgpu_firmware *acr_fw = acr_desc->acr_fw;
	struct bin_hdr *acr_fw_bin_hdr = NULL;
	struct acr_fw_header *acr_fw_hdr = NULL;
	struct nvgpu_mem *acr_ucode_mem = &acr_desc->acr_ucode;
	u32 img_size_in_bytes = 0;
	u32 *acr_ucode_data;
	u32 *acr_ucode_header;
	u32 status = 0U;

	nvgpu_pmu_dbg(g, "ACR TYPE %x ", acr_desc->acr_type);

	if (acr_fw != NULL) {
		acr->patch_wpr_info_to_ucode(g, acr, acr_desc, true);
	} else {
		acr_fw = nvgpu_request_firmware(g, acr_desc->acr_fw_name,
				NVGPU_REQUEST_FIRMWARE_NO_SOC);
		if (acr_fw == NULL) {
			nvgpu_err(g, "%s ucode get fail for %s",
				acr_desc->acr_fw_name, g->name);
			return -ENOENT;
		}

		acr_desc->acr_fw = acr_fw;

		acr_fw_bin_hdr = (struct bin_hdr *)acr_fw->data;

		acr_fw_hdr = (struct acr_fw_header *)
			(acr_fw->data + acr_fw_bin_hdr->header_offset);

		acr_ucode_header = (u32 *)(acr_fw->data +
			acr_fw_hdr->hdr_offset);

		acr_ucode_data = (u32 *)(acr_fw->data +
			acr_fw_bin_hdr->data_offset);

		img_size_in_bytes = ALIGN((acr_fw_bin_hdr->data_size), 256U);

		/* Lets patch the signatures first.. */
		if (acr_ucode_patch_sig(g, acr_ucode_data,
			(u32 *)(acr_fw->data + acr_fw_hdr->sig_prod_offset),
			(u32 *)(acr_fw->data + acr_fw_hdr->sig_dbg_offset),
			(u32 *)(acr_fw->data + acr_fw_hdr->patch_loc),
			(u32 *)(acr_fw->data + acr_fw_hdr->patch_sig)) < 0) {
			nvgpu_err(g, "patch signatures fail");
			status = -1;
			goto err_release_acr_fw;
		}

		status = nvgpu_dma_alloc_map_sys(vm, img_size_in_bytes,
			acr_ucode_mem);
		if (status != 0U) {
			status = -ENOMEM;
			goto err_release_acr_fw;
		}

		acr->patch_wpr_info_to_ucode(g, acr, acr_desc, false);

		nvgpu_mem_wr_n(g, acr_ucode_mem, 0U, acr_ucode_data,
			img_size_in_bytes);

		/*
		 * In order to execute this binary, we will be using
		 * a bootloader which will load this image into
		 * FALCON IMEM/DMEM.
		 * Fill up the bootloader descriptor to use..
		 * TODO: Use standard descriptor which the generic bootloader is
		 * checked in.
		 */
		acr->acr_fill_bl_dmem_desc(g, acr, acr_desc, acr_ucode_header);
	}

	status = gm20b_acr_hs_bl_exec(g, acr, acr_desc, true);
	if (status != 0U) {
		goto err_free_ucode_map;
	}

	return 0;
err_free_ucode_map:
	nvgpu_dma_unmap_free(vm, acr_ucode_mem);
err_release_acr_fw:
	nvgpu_release_firmware(g, acr_fw);
	acr_desc->acr_fw = NULL;
	return status;
}

void gm20b_remove_acr_support(struct nvgpu_acr *acr)
{
	struct gk20a *g = acr->g;
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->pmu.vm;

	if (acr->acr.acr_fw != NULL) {
		nvgpu_release_firmware(g, acr->acr.acr_fw);
	}

	if (acr->acr.acr_hs_bl.hs_bl_fw != NULL) {
		nvgpu_release_firmware(g, acr->acr.acr_hs_bl.hs_bl_fw);
	}

	nvgpu_dma_unmap_free(vm, &acr->acr.acr_ucode);
	nvgpu_dma_unmap_free(vm, &acr->acr.acr_hs_bl.hs_bl_ucode);
}

static void gm20b_acr_default_sw_init(struct gk20a *g, struct hs_acr *hs_acr)
{
	struct hs_flcn_bl *hs_bl = &hs_acr->acr_hs_bl;

	nvgpu_log_fn(g, " ");

	/* ACR HS bootloader ucode name */
	hs_bl->bl_fw_name = HSBIN_ACR_BL_UCODE_IMAGE;

	/* ACR HS ucode type & f/w name*/
	hs_acr->acr_type = ACR_DEFAULT;
	hs_acr->acr_fw_name = HSBIN_ACR_UCODE_IMAGE;

	/* bootlader interface used by ACR HS bootloader*/
	hs_acr->ptr_bl_dmem_desc = &hs_acr->bl_dmem_desc;
	hs_acr->bl_dmem_desc_size = sizeof(struct flcn_bl_dmem_desc);

	/* set on which falcon ACR need to execute*/
	hs_acr->acr_flcn = &g->pmu_flcn;
	hs_acr->acr_flcn_setup_hw_and_bl_bootstrap =
		gm20b_pmu_setup_hw_and_bl_bootstrap;
}

void nvgpu_gm20b_acr_sw_init(struct gk20a *g, struct nvgpu_acr *acr)
{
	nvgpu_log_fn(g, " ");

	acr->g = g;

	acr->bootstrap_owner = LSF_FALCON_ID_PMU;
	acr->max_supported_lsfm = MAX_SUPPORTED_LSFM;

	gm20b_acr_default_sw_init(g, &acr->acr);

	acr->get_wpr_info = gm20b_wpr_info;
	acr->alloc_blob_space = gm20b_alloc_blob_space;
	acr->bootstrap_hs_acr = gm20b_bootstrap_hs_acr;
	acr->patch_wpr_info_to_ucode =
		gm20b_acr_patch_wpr_info_to_ucode;
	acr->acr_fill_bl_dmem_desc =
		gm20b_acr_fill_bl_dmem_desc;

	acr->remove_support = gm20b_remove_acr_support;
}
