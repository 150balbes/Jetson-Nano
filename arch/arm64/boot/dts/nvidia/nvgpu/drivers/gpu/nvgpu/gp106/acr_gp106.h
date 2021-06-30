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

#ifndef NVGPU_ACR_GP106_H
#define NVGPU_ACR_GP106_H

#define GP106_FECS_UCODE_SIG "gp106/fecs_sig.bin"
#define GP106_GPCCS_UCODE_SIG "gp106/gpccs_sig.bin"
#define GP104_FECS_UCODE_SIG "gp104/fecs_sig.bin"
#define GP104_GPCCS_UCODE_SIG "gp104/gpccs_sig.bin"


int gp106_bootstrap_hs_flcn(struct gk20a *g);
int gp106_prepare_ucode_blob(struct gk20a *g);
int gp106_alloc_blob_space(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem);

void gp106_wpr_info(struct gk20a *g, struct wpr_carveout_info *inf);

void lsfm_free_ucode_img_res(struct gk20a *g,
				    struct flcn_ucode_img_v1 *p_img);
void lsfm_free_nonpmu_ucode_img_res(struct gk20a *g,
					   struct flcn_ucode_img_v1 *p_img);
int lsf_gen_wpr_requirements(struct gk20a *g,
		struct ls_flcn_mgr_v1 *plsfm);
void free_acr_resources(struct gk20a *g, struct ls_flcn_mgr_v1 *plsfm);
void lsfm_fill_static_lsb_hdr_info(struct gk20a *g,
	u32 falcon_id, struct lsfm_managed_ucode_img_v2 *pnode);
int gp106_pmu_populate_loader_cfg(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size);

int pmu_ucode_details(struct gk20a *g, struct flcn_ucode_img_v1 *p_img);
int fecs_ucode_details(struct gk20a *g,
		struct flcn_ucode_img_v1 *p_img);
int gpccs_ucode_details(struct gk20a *g,
		struct flcn_ucode_img_v1 *p_img);
int sec2_ucode_details(struct gk20a *g, struct flcn_ucode_img_v1 *p_img);
int lsfm_add_ucode_img(struct gk20a *g, struct ls_flcn_mgr_v1 *plsfm,
	struct flcn_ucode_img_v1 *ucode_image, u32 falcon_id);
int lsfm_discover_ucode_images(struct gk20a *g,
	struct ls_flcn_mgr_v1 *plsfm);
void lsfm_init_wpr_contents(struct gk20a *g,
		struct ls_flcn_mgr_v1 *plsfm, struct nvgpu_mem *nonwpr);
int gp106_flcn_populate_bl_dmem_desc(struct gk20a *g,
	void *lsfm, u32 *p_bl_gen_desc_size, u32 falconid);
int lsfm_fill_flcn_bl_gen_desc(struct gk20a *g,
		struct lsfm_managed_ucode_img_v2 *pnode);
int gp106_acr_fill_bl_dmem_desc(struct gk20a *g,
	struct nvgpu_acr *acr, struct hs_acr *acr_desc,
	u32 *acr_ucode_header);
int gp106_acr_patch_wpr_info_to_ucode(struct gk20a *g, struct nvgpu_acr *acr,
	struct hs_acr *acr_desc, bool is_recovery);
void nvgpu_gp106_acr_sw_init(struct gk20a *g, struct nvgpu_acr *acr);

#endif /* NVGPU_ACR_GP106_H */

