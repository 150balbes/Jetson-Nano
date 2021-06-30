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
#ifndef NVGPU_ACR_OBJFLCN_H
#define NVGPU_ACR_OBJFLCN_H

#ifndef NVGPU_ACR_H
#warning "acr_objflcn.h not included from nvgpu_acr.h!" \
	"Include nvgpu_acr.h instead of acr_xxx.h to get access to ACR interfaces"
#endif

struct flcn_ucode_img {
	u32 *header; /* only some falcons have header */
	u32 *data;
	struct pmu_ucode_desc *desc; /* only some falcons have descriptor */
	u32 data_size;
	void *fw_ver; /* CTRL_GPU_GET_FIRMWARE_VERSION_PARAMS struct */
	u8 load_entire_os_data; /* load the whole osData section at boot time.*/
	/* NULL if not a light secure falcon.*/
	struct lsf_ucode_desc *lsf_desc;
	/* True if there a resources to freed by the client. */
	u8 free_res_allocs;
	u32 flcn_inst;
};

struct flcn_ucode_img_v1 {
	u32 *header;
	u32 *data;
	struct pmu_ucode_desc_v1 *desc;
	u32 data_size;
	void *fw_ver;
	u8 load_entire_os_data;
	struct lsf_ucode_desc_v1 *lsf_desc;
	u8 free_res_allocs;
	u32 flcn_inst;
};

/*
 * Falcon UCODE header index.
 */
#define FLCN_NL_UCODE_HDR_OS_CODE_OFF_IND              (0)
#define FLCN_NL_UCODE_HDR_OS_CODE_SIZE_IND             (1)
#define FLCN_NL_UCODE_HDR_OS_DATA_OFF_IND              (2)
#define FLCN_NL_UCODE_HDR_OS_DATA_SIZE_IND             (3)
#define FLCN_NL_UCODE_HDR_NUM_APPS_IND                 (4)

/*
 * There are total N number of Apps with code and offset defined in UCODE header
 * This macro provides the CODE and DATA offset and size of Ath application.
 */
#define FLCN_NL_UCODE_HDR_APP_CODE_START_IND           (5)
#define FLCN_NL_UCODE_HDR_APP_CODE_OFF_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (A*2))
#define FLCN_NL_UCODE_HDR_APP_CODE_SIZE_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (A*2) + 1)
#define FLCN_NL_UCODE_HDR_APP_CODE_END_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_CODE_START_IND + (N*2) - 1)

#define FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_CODE_END_IND(N) + 1)
#define FLCN_NL_UCODE_HDR_APP_DATA_OFF_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (A*2))
#define FLCN_NL_UCODE_HDR_APP_DATA_SIZE_IND(N, A) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (A*2) + 1)
#define FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_START_IND(N) + (N*2) - 1)

#define FLCN_NL_UCODE_HDR_OS_OVL_OFF_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) + 1)
#define FLCN_NL_UCODE_HDR_OS_OVL_SIZE_IND(N) \
	(FLCN_NL_UCODE_HDR_APP_DATA_END_IND(N) + 2)

#endif /* NVGPU_ACR_OBJFLCN_H */
