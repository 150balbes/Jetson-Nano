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
#ifndef NVGPU_ACR_OBJLSFM_H
#define NVGPU_ACR_OBJLSFM_H

#ifndef NVGPU_ACR_H
#warning "acr_objlsfm.h not included from nvgpu_acr.h!" \
	"Include nvgpu_acr.h instead of acr_xxx.h to get access to ACR interfaces"
#endif

#include "acr_flcnbl.h"
#include "acr_objflcn.h"

/*
 * LSFM Managed Ucode Image
 * next             : Next image the list, NULL if last.
 * wpr_header         : WPR header for this ucode image
 * lsb_header         : LSB header for this ucode image
 * bl_gen_desc     : Bootloader generic desc structure for this ucode image
 * bl_gen_desc_size : Sizeof bootloader desc structure for this ucode image
 * full_ucode_size  : Surface size required for final ucode image
 * ucode_img        : Ucode image info
 */
struct lsfm_managed_ucode_img {
	struct lsfm_managed_ucode_img *next;
	struct lsf_wpr_header wpr_header;
	struct lsf_lsb_header lsb_header;
	union flcn_bl_generic_desc bl_gen_desc;
	u32 bl_gen_desc_size;
	u32 full_ucode_size;
	struct flcn_ucode_img ucode_img;
};

struct lsfm_managed_ucode_img_v2 {
	struct lsfm_managed_ucode_img_v2 *next;
	struct lsf_wpr_header_v1 wpr_header;
	struct lsf_lsb_header_v1 lsb_header;
	union flcn_bl_generic_desc_v1 bl_gen_desc;
	u32 bl_gen_desc_size;
	u32 full_ucode_size;
	struct flcn_ucode_img_v1 ucode_img;
};

/*
 * Defines the structure used to contain all generic information related to
 * the LSFM.
 * Contains the Light Secure Falcon Manager (LSFM) feature related data.
 */
struct ls_flcn_mgr {
	u16 managed_flcn_cnt;
	u32 wpr_size;
	u32 disable_mask;
	struct lsfm_managed_ucode_img *ucode_img_list;
	void *wpr_client_req_state;/*PACR_CLIENT_REQUEST_STATE originally*/
};

/*
 * LSFM SUB WPRs struct
 * pnext        : Next entry in the list, NULL if last
 * sub_wpr_header : SubWpr Header struct
 */
struct lsfm_sub_wpr {
	struct lsfm_sub_wpr *pnext;
	struct lsf_shared_sub_wpr_header sub_wpr_header;
};

struct ls_flcn_mgr_v1 {
	u16 managed_flcn_cnt;
	u32 wpr_size;
	u32 disable_mask;
	struct lsfm_managed_ucode_img_v2 *ucode_img_list;
	void *wpr_client_req_state;/*PACR_CLIENT_REQUEST_STATE originally*/
	u16 managed_sub_wpr_count;
	struct lsfm_sub_wpr *psub_wpr_list;
};


#endif /* NVGPU_ACR_OBJLSFM_H */
