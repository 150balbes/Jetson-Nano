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

#ifndef NVGPU_SEC2_IF_ACR_H
#define NVGPU_SEC2_IF_ACR_H

#include <nvgpu/types.h>

/*
 * ACR Command Types
 * _BOOT_FALCON
 * NVGPU sends a Falcon ID and LSB offset to SEC2 to boot
 * the falcon in LS mode.
 * SEC2 needs to hanlde the case since UCODE of falcons are
 * stored in secured location on FB.
 */
#define NV_SEC2_ACR_CMD_ID_BOOTSTRAP_FALCON		0U

/* nvgpu provides the Falcon ID to bootstrap */
struct nv_sec2_acr_cmd_bootstrap_falcon {
	/* Command must be first as this struct is the part of union */
	u8 cmd_type;

	/* Additional bootstrapping flags */
	u32 flags;

	/* ID to identify Falcon, ref LSF_FALCON_ID_<XYZ> */
	u32 falcon_id;
};

#define NV_SEC2_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET     0U
#define NV_SEC2_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_NO  1U
#define NV_SEC2_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES 0U

/* A union of all ACR Commands */
union nv_sec2_acr_cmd {
	/* Command type */
	u8 cmd_type;

	/* Bootstrap Falcon */
	struct nv_sec2_acr_cmd_bootstrap_falcon bootstrap_falcon;
};

/* ACR Message Status */

/* Returns the Bootstrapped falcon ID to RM */
#define NV_SEC2_ACR_MSG_ID_BOOTSTRAP_FALCON    0U

/* Returns the Error Status for Invalid Command */
#define NV_SEC2_ACR_MSG_ID_INVALID_COMMAND     2U

/*
 * SEC2 notifies nvgpu about bootstrap status of falcon
 */
struct nv_sec2_acr_msg_bootstrap_falcon {
	/* Message must be at start */
	u8 msg_type;

	/* Falcon Error Code returned by message */
	u32 error_code;

	/* Bootstrapped falcon ID by ACR */
	u32 falcon_id;
} ;

/*
 * A union of all ACR Messages.
 */
union nv_sec2_acr_msg {
	/* Message type */
	u8 msg_type;

	/* Bootstrap details of falcon and status code */
	struct nv_sec2_acr_msg_bootstrap_falcon msg_flcn;
};

#endif /* NVGPU_SEC2_IF_ACR_H */
