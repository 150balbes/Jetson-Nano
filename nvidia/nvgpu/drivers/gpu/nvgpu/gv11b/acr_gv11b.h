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

#ifndef NVGPU_ACR_GV11B_H
#define NVGPU_ACR_GV11B_H


int gv11b_bootstrap_hs_flcn(struct gk20a *g);
int gv11b_init_pmu_setup_hw1(struct gk20a *g,
		void *desc, u32 bl_sz);
void gv11b_setup_apertures(struct gk20a *g);
int gv11b_alloc_blob_space(struct gk20a *g, size_t size,
				struct nvgpu_mem *mem);

void nvgpu_gv11b_acr_sw_init(struct gk20a *g, struct nvgpu_acr *acr);
int gv11b_acr_patch_wpr_info_to_ucode(struct gk20a *g, struct nvgpu_acr *acr,
	struct hs_acr *acr_desc, bool is_recovery);
#endif /* NVGPU_ACR_GV11B_H */

