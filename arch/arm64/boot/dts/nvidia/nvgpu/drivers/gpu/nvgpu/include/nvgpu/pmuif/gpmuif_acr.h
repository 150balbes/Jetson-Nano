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
#ifndef NVGPU_PMUIF_GPMUIF_ACR_H
#define NVGPU_PMUIF_GPMUIF_ACR_H

/* ACR Commands/Message structures */

enum {
	PMU_ACR_CMD_ID_INIT_WPR_REGION = 0x0,
	PMU_ACR_CMD_ID_BOOTSTRAP_FALCON,
	PMU_ACR_CMD_ID_RESERVED,
	PMU_ACR_CMD_ID_BOOTSTRAP_MULTIPLE_FALCONS,
};

/*
 * Initializes the WPR region details
 */
struct pmu_acr_cmd_init_wpr_details {
	u8  cmd_type;
	u32 regionid;
	u32 wproffset;

};

/*
 * falcon ID to bootstrap
 */
struct pmu_acr_cmd_bootstrap_falcon {
	u8 cmd_type;
	u32 flags;
	u32 falconid;
};

/*
 * falcon ID to bootstrap
 */
struct pmu_acr_cmd_bootstrap_multiple_falcons {
	u8 cmd_type;
	u32 flags;
	u32 falconidmask;
	u32 usevamask;
	struct falc_u64 wprvirtualbase;
};

#define PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_NO  1
#define PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_RESET_YES 0


struct pmu_acr_cmd {
	union {
		u8 cmd_type;
		struct pmu_acr_cmd_bootstrap_falcon bootstrap_falcon;
		struct pmu_acr_cmd_init_wpr_details init_wpr;
		struct pmu_acr_cmd_bootstrap_multiple_falcons boot_falcons;
	};
};

/* acr messages */

/*
 * returns the WPR region init information
 */
#define PMU_ACR_MSG_ID_INIT_WPR_REGION   0

/*
 * Returns the Bootstrapped falcon ID to RM
 */
#define PMU_ACR_MSG_ID_BOOTSTRAP_FALCON  1

/*
 * Returns the WPR init status
 */
#define PMU_ACR_SUCCESS                  0
#define PMU_ACR_ERROR                    1

/*
 * PMU notifies about bootstrap status of falcon
 */
struct pmu_acr_msg_bootstrap_falcon {
	u8 msg_type;
	union {
		u32 errorcode;
		u32 falconid;
	};
};

struct pmu_acr_msg {
	union {
		u8 msg_type;
		struct pmu_acr_msg_bootstrap_falcon acrmsg;
	};
};

/* ACR RPC */
#define NV_PMU_RPC_ID_ACR_INIT_WPR_REGION      0x00
#define NV_PMU_RPC_ID_ACR_WRITE_CBC_BASE       0x01
#define NV_PMU_RPC_ID_ACR_BOOTSTRAP_FALCON     0x02
#define NV_PMU_RPC_ID_ACR_BOOTSTRAP_GR_FALCONS 0x03
#define NV_PMU_RPC_ID_ACR__COUNT               0x04

/*
 * structure that holds data used
 * to execute INIT_WPR_REGION RPC.
 */
struct nv_pmu_rpc_struct_acr_init_wpr_region {
	/*[IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
	/*[IN] ACR region ID of WPR region */
	u32 wpr_regionId;
	/* [IN] WPR offset from startAddress */
	u32 wpr_offset;
	u32 scratch[1];
};

/*
 * structure that holds data used to
 * execute BOOTSTRAP_GR_FALCONS RPC.
 */
struct nv_pmu_rpc_struct_acr_bootstrap_gr_falcons {
	/*[IN/OUT] Must be first field in RPC structure */
	struct nv_pmu_rpc_header hdr;
	/* [IN] Mask of falcon IDs @ref LSF_FALCON_ID_<XYZ> */
	u32  falcon_id_mask;
	/*
	 * [IN] Boostrapping flags @ref
	 * PMU_ACR_CMD_BOOTSTRAP_FALCON_FLAGS_<XYZ>
	 */
	u32 flags;
	/* [IN] Indicate whether the particular falon uses VA */
	u32  falcon_va_mask;
	/*
	 * [IN] WPR Base Address in VA. The Inst Block containing
	 * this VA should be bound to both PMU and GR falcons
	 * during the falcon boot
	 */
	struct falc_u64  wpr_base_virtual;
	u32  scratch[1];
};

#endif /* NVGPU_PMUIF_GPMUIF_ACR_H */
