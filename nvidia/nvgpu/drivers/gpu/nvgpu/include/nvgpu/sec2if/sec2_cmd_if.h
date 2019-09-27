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

#ifndef NVGPU_SEC2_CMD_IF_H
#define NVGPU_SEC2_CMD_IF_H

#include <nvgpu/sec2if/sec2_if_sec2.h>
#include <nvgpu/sec2if/sec2_if_acr.h>

struct nv_flcn_cmd_sec2 {
	struct pmu_hdr hdr;
	union {
		union nv_sec2_acr_cmd acr;
	} cmd;
};

struct nv_flcn_msg_sec2 {
	struct pmu_hdr hdr;

	union {
		union nv_flcn_msg_sec2_init init;
		union nv_sec2_acr_msg acr;
	} msg;
};

#define  NV_SEC2_UNIT_REWIND          NV_FLCN_UNIT_ID_REWIND
#define  NV_SEC2_UNIT_INIT            (0x01U)
#define  NV_SEC2_UNIT_ACR             (0x07U)
#define  NV_SEC2_UNIT_END             (0x0AU)

#endif /* NVGPU_SEC2_CMD_IF_H */
