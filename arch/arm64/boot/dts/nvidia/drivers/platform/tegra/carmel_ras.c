/*
 * RAS driver for T194
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <asm/traps.h>
#include <linux/platform/tegra/tegra18_cpu_map.h>
#include <linux/platform/tegra/carmel_ras.h>
#include <linux/platform/tegra/tegra-cpu.h>
#include <linux/of_device.h>
#include <linux/debugfs.h>
#include <linux/cpuhotplug.h>

static LIST_HEAD(core_ras_list);
static DEFINE_RAW_SPINLOCK(core_ras_lock);
static LIST_HEAD(corecluster_ras_list);
static DEFINE_RAW_SPINLOCK(corecluster_ras_lock);
static LIST_HEAD(ccplex_ras_list);
static DEFINE_RAW_SPINLOCK(ccplex_ras_lock);

static struct dentry *debugfs_dir;
static struct dentry *debugfs_node;
static int is_debug;

/* saved hotplug state */
static enum cpuhp_state hp_state;

/* Error Records per CORE - IFU errors
 * error_code = value of ARM_ERR_STATUS:IERR[15:8]
 */
static struct ras_error ifu_errors[] = {
	{.name = "IMQ Data Parity", .error_code = 0x08},
	{.name = "L2 I$ Fetch Uncorrectable", .error_code = 0x07},
	{.name = "I$ Tag Parity Snoop", .error_code = 0x06},
	{.name = "I$ Multi-Hit Snoop", .error_code = 0x05},
	{.name = "ITLB Parity", .error_code = 0x04},
	{.name = "Trace Hash Error", .error_code = 0x03},
	{.name = "I$ Data Parity", .error_code = 0x02},
	{.name = "I$ Tag Parity", .error_code = 0x01},
	{.name = "I$ Multi-Hit", .error_code = 0x0F},
	{}
};

/* Error Records per CORE - RET JSR errors */
static struct ras_error ret_jsr_errors[] = {
	{.name = "FRF Parity", .error_code = 0x13},
	{.name = "IRF Parity", .error_code = 0x12},
	{.name = "Garbage Bundle", .error_code = 0x11},
	{.name = "Bundle Completion Timeout", .error_code = 0x10},
	{}
};

/* Error Records per CORE - MTS JSR errors */
static struct ras_error mts_jsr_errors[] = {
	{.name = "CTU MMIO Region", .error_code = 0x25},
	{.name = "MTS MMCRAB Region Access", .error_code = 0x24},
	{.name = "MTS_CARVEOUT Access from ARM SW", .error_code = 0x23},
	{.name = "NAFLL PLL Failure to Lock", .error_code = 0x22},
	{.name = "Internal Correctable MTS Error", .error_code = 0x21},
	{.name = "Internal Uncorrectable MTS Error", .error_code = 0x20},
	{}
};

/* Error Records per CORE - LSD_1/LSD_STQ errors */
static struct ras_error lsd_1_errors[] = {
	{.name = "Coherent Cache Data Store Multi-Line ECC",
						.error_code = 0x39},
	{.name = "Coherent Cache Data Store Uncorrectable ECC",
						.error_code = 0x38},
	{.name = "Coherent Cache Data Store Correctable ECC",
						.error_code = 0x37},
	{.name = "Coherent Cache Data Load Uncorrectable ECC",
						.error_code = 0x36},
	{.name = "Coherent Cache Data Load Correctable ECC",
						.error_code = 0x35},
	{.name = "Coherent Cache Multi-Hit", .error_code = 0x33},
	{.name = "Coherent Cache Tag Store Parity", .error_code = 0x31},
	{.name = "Coherent Cache Tag Load Parity", .error_code = 0x30},
	{}
};

/* Error Records per CORE - LSD_2/LSD_ECC errors */
static struct ras_error lsd_2_errors[] = {
	{.name = "BTU Copy Mini-Cache PPN Multi-Hit Error", .error_code = 0x49},
	{.name = "Coherent Cache Data Uncorrectable ECC", .error_code = 0x47},
	{.name = "Coherent Cache Data Correctable ECC", .error_code = 0x46},
	{.name = "Version Cache Byte-Enable Parity", .error_code = 0x45},
	{.name = "Version Cache Data Uncorrectable ECC", .error_code = 0x44},
	{.name = "Version Cache Data Correctable ECC", .error_code = 0x43},
	{.name = "BTU Copy Coherent Cache PPN Parity", .error_code = 0x41},
	{.name = "BTU Copy Coherent Cache VPN Parity", .error_code = 0x40},
	{}
};

/* Error Records per CORE - LSD_3/LSD_L1HPF errors */
static struct ras_error lsd_3_errors[] = {
	{.name = "L2 TLB Parity Error", .error_code = 0xE0},
	{}
};

/* Error Records per CORE */
static struct error_record core_ers[] = {
	{.name = "IFU", .errx = 0,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_IFU_ICMH_ERR | ERR_CTL_IFU_ICTP_ERR |
		ERR_CTL_IFU_ICDP_ERR |
		ERR_CTL_IFU_THERR_ERR | ERR_CTL_IFU_ITLBP_ERR |
		ERR_CTL_IFU_ICMHSNP_ERR | ERR_CTL_IFU_ICTPSNP_ERR |
		ERR_CTL_IFU_L2UC_ERR | ERR_CTL_IFU_IMQDP_ERR,
	 .errors = ifu_errors},
	{.name = "RET_JSR", .errx = 1,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE |
		ERR_CTL_RET_JSR_TO_ERR | ERR_CTL_RET_JSR_GB_ERR |
		ERR_CTL_RET_JSR_IRFP_ERR | ERR_CTL_RET_JSR_FRFP_ERR,
	 .errors = ret_jsr_errors},
	{.name = "MTS_JSR", .errx = 2,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_MTS_JSR_ERRUC_ERR | ERR_CTL_MTS_JSR_ERRC_ERR |
		ERR_CTL_MTS_JSR_NAFLL_ERR | ERR_CTL_MTS_JSR_CARVE_ERR |
		ERR_CTL_MTS_JSR_CRAB_ERR | ERR_CTL_MTS_JSR_MMIO_ERR,
	 .errors = mts_jsr_errors},
	{.name = "LSD_STQ", .errx = 3,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_LSD1_CCTLP_ERR | ERR_CTL_LSD1_CCTSP_ERR |
		ERR_CTL_LSD1_CCMH_ERR |
		ERR_CTL_LSD1_CCDLECC_S_ERR | ERR_CTL_LSD1_CCDLECC_D_ERR |
		ERR_CTL_LSD1_CCDSECC_S_ERR | ERR_CTL_LSD1_CCDSECC_D_ERR |
		ERR_CTL_LSD1_CCDSMLECC_ERR,
	 .errors = lsd_1_errors},
	{.name = "LSD_DCC", .errx = 4,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_LSD2_BTCCVPP_ERR | ERR_CTL_LSD2_BTCCPPP_ERR |
		ERR_CTL_LSD2_VRCDECC_S_ERR | ERR_CTL_LSD2_VRCDECC_D_ERR |
		ERR_CTL_LSD2_BTMCMH_ERR | ERR_CTL_LSD2_VRCBP_ERR |
		ERR_CTL_LSD2_CCDEECC_S_ERR | ERR_CTL_LSD2_CCDEECC_D_ERR,
	 .errors = lsd_2_errors},
	{.name = "LSD_L1HPF", .errx = 5,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_LSD3_L2TLBP_ERR,
	 .errors = lsd_3_errors},
	{}
};

/* Error Records per CORE CLUSTER - L2 errors
 * error_code = value of ARM_ERR_STATUS:IERR[15:8]
 */
static struct ras_error l2_errors[] = {
	{.name = "URT Timeout", .error_code = 0x68},
	{.name = "L2 Protocol Violation", .error_code = 0x67},
	{.name = "SCF to L2 Slave Error Read", .error_code = 0x66},
	{.name = "SCF to L2 Slave Error Write", .error_code = 0x65},
	{.name = "SCF to L2 Decode Error Read", .error_code = 0x64},
	{.name = "SCF to L2 Decode Error Write", .error_code = 0x63},
	{.name = "SCF to L2 Request Response Interface Parity Errors",
		.error_code = 0x62},
	{.name = "SCF to L2 Advance notice interface parity errors",
		.error_code = 0x61},
	{.name = "SCF to L2 Filldata Parity Errors", .error_code = 0x60},
	{.name = "SCF to L2 UnCorrectable ECC Data Error on interface",
		.error_code = 0x5F},
	{.name = "SCF to L2 Correctable ECC Data Error on interface",
		.error_code = 0x5E},
	{.name = "Core 1 to L2 Parity Error", .error_code = 0x5D},
	{.name = "Core 0 to L2 Parity Error", .error_code = 0x5C},
	{.name = "L2 Multi-Hit", .error_code = 0x5B},
	{.name = "L2 URT Tag Parity Error", .error_code = 0x5A},
	{.name = "L2 NTT Tag Parity Error", .error_code = 0x59},
	{.name = "L2 MLT Tag Parity Error", .error_code = 0x58},
	{.name = "L2 URD Data", .error_code = 0x57},
	{.name = "L2 NTP Data", .error_code = 0x56},
	{.name = "L2 MLC Uncorrectable Clean", .error_code = 0x54},
	{.name = "L2 URD Uncorrectable", .error_code = 0x53},
	{.name = "L2 MLC Uncorrectable Dirty", .error_code = 0x52},
	{.name = "L2 URD Correctable Error", .error_code = 0x51},
	{.name = "L2 MLC Correctable Error", .error_code = 0x50},
	{}
};

/* Error Records per CORE CLUSTER - MMU errors */
static struct ras_error mmu_errors[] = {
	{.name = "Walker Cache Parity Error", .error_code = 0xE9},
	{.name = "A$ Parity Error", .error_code = 0xE8},
	{}
};

/* Error Records per CORE CLUSTER - Cluster Clocks errors */
static struct ras_error cluster_clocks_errors[] = {
	{.name = "Frequency Monitor Error", .error_code = 0xE4},
	{}
};

/* Error Records per CORE CLUSTER */
static struct error_record corecluster_ers[] = {
	{.name = "L2", .errx = 0,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_L2_MLD_ECCC_ERR | ERR_CTL_L2_URD_ECCC_ERR |
		ERR_CTL_L2_MLD_ECCUD_ERR | ERR_CTL_L2_URD_ECCU_ERR |
		ERR_CTL_L2_MLD_ECCUC_ERR | ERR_CTL_L2_NTDP_ERR |
		ERR_CTL_L2_URDP | ERR_CTL_L2_MLTP_ERR | ERR_CTL_L2_NTTP_ERR |
		ERR_CTL_L2_URTP_ERR | ERR_CTL_L2_L2MH_ERR |
		ERR_CTL_L2_CORE02L2CP_ERR | ERR_CTL_L2_CORE12L2CP_ERR |
		ERR_CTL_L2_SCF2L2C_ECCC_ERR | ERR_CTL_L2_SCF2L2C_ECCU_ERR |
		ERR_CTL_L2_SCF2L2C_FILLDATAP_ERR |
		ERR_CTL_L2_SCF2L2C_ADVNOTP_ERR |
		ERR_CTL_L2_SCF2L2C_REQRSPP_ERR |
		ERR_CTL_L2_SCF2L2C_DECWTERR_ERR |
		ERR_CTL_L2_SCF2L2C_DECRDERR_ERR |
		ERR_CTL_L2_SCF2L2C_SLVWTERR_ERR |
		ERR_CTL_L2_SCF2L2C_SLVRDERR_ERR | ERR_CTL_L2_L2PCL_ERR |
		ERR_CTL_L2_URTTO_ERR,
	.errors = l2_errors},
	{.name = "CLUSTER_CLOCKS", .errx = 1,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | ERR_CTL_CC_FREQ_MON_ERR,
	 .errors = cluster_clocks_errors},
	{.name = "MMU", .errx = 2,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_CFI |
		ERR_CTL_MMU_ACPERR_ERR | ERR_CTL_MMU_WCPERR_ERR,
	 .errors = mmu_errors},
	{}
};

/* Error Records per CCPLEX - CMU:CCPMU errors
 * error_code = value of ARM_ERR_STATUS:IERR[15:8]
 */
static struct ras_error cmu_ccpmu_errors[] = {
	{.name = "MCE Ucode Error", .error_code = 0x84},
	{.name = "MCE IL1 Parity Error", .error_code = 0x83},
	{.name = "MCE Timeout Error", .error_code = 0x82},
	{.name = "CRAB Access Error", .error_code = 0x81},
	{.name = "MCE Memory Access Error", .error_code = 0x80},
	{}
};

/* Error Records per CCPLEX - SCF:IOB errors */
static struct ras_error scf_iob_errors[] = {
	{.name = "Request parity error", .error_code = 0x99},
	{.name = "Putdata parity error", .error_code = 0x98},
	{.name = "Uncorrectable ECC on Putdata", .error_code = 0x97},
	{.name = "CBB Interface Error", .error_code = 0x96},
	{.name = "MMCRAB Error", .error_code = 0x95},
	{.name = "IHI Interface Error", .error_code = 0x94},
	{.name = "CRI Error", .error_code = 0x93},
	{.name = "TBX Interface Error", .error_code = 0x92},
	{.name = "EVP Interface Error", .error_code = 0x91},
	{.name = "Correctable ECC on Putdata", .error_code = 0x90},
	{}
};

/* Error Records per CCPLEX - SCF:SNOC errors */
static struct ras_error scf_snoc_errors[] = {
	{.name = "Misc Client Parity  Error", .error_code = 0xAA},
	{.name = "Misc Filldata Parity  Error", .error_code = 0xA9},
	{.name = "Uncorrectable ECC Misc Client", .error_code = 0xA8},
	{.name = "DVMU Interface Parity  Error", .error_code = 0xA7},
	{.name = "DVMU Interface Timeout  Error", .error_code = 0xA6},
	{.name = "CPE Request  Error", .error_code = 0xA5},
	{.name = "CPE Response  Error", .error_code = 0xA4},
	{.name = "CPE Timeout  Error", .error_code = 0xA3},
	{.name = "Uncorrectable Carveout  Error", .error_code = 0xA2},
	{.name = "Correctable ECC Misc Client", .error_code = 0xA1},
	{.name = "Correctable Carveout  Error", .error_code = 0xA0},
	{}
};

/* Error Records per CCPLEX - SCF:CTU errors */
static struct ras_error cmu_ctu_errors[] = {
	{.name = "Timeout error for TRC_DMA request", .error_code = 0xB7},
	{.name = "Timeout error for CTU Snp", .error_code = 0xB6},
	{.name = "Parity error in CTU TAG RAM", .error_code = 0xB5},
	{.name = "Parity error in CTU DATA RAM", .error_code = 0xB3},
	{.name = "Parity error for Cluster Rsp", .error_code = 0xB4},
	{.name = "Parity error for TRL requests from 9 agents",
		.error_code = 0xB2},
	{.name = "Parity error for MCF request", .error_code =	0xB1},
	{.name = "TRC DMA fillsnoop parity error", .error_code = 0xB0},
	{}
};

/* Error Records per CCPLEX - SCF:L3_* errors */
static struct ras_error scf_l3_errors[] = {
	{.name = "L3 Correctable ECC Error", .error_code = 0x7C},
	{.name = "SNOC Interface Parity Error", .error_code = 0x7B},
	{.name = "MCF Interface Parity Error", .error_code = 0x7A},
	{.name = "L3 Tag Parity Error", .error_code = 0x79},
	{.name = "L3 Dir Parity Error", .error_code = 0x78},
	{.name = "L3 Uncorrectable ECC Error", .error_code = 0x77},
	{.name = "Multi-Hit CAM Error", .error_code = 0x75},
	{.name = "Multi-Hit Tag Error", .error_code = 0x74},
	{.name = "Unrecognized Command Error", .error_code = 0x73},
	{.name = "L3 Protocol Error", .error_code = 0x72},
	{}
};

/* Error Records per CCPLEX - CMU_Clocks errors */
static struct ras_error scfcmu_clocks_errors[] = {
	{.name = "Cluster 3 frequency monitor error", .error_code = 0xC7},
	{.name = "Cluster 2 frequency monitor error", .error_code = 0xC6},
	{.name = "Cluster 1 frequency monitor error", .error_code = 0xC5},
	{.name = "Cluster 0 frequency monitor error", .error_code = 0xC3},
	{.name = "Voltage error on ADC1 Monitored Logic", .error_code = 0xC4},
	{.name = "Voltage error on ADC0 Monitored Logic", .error_code = 0xC2},
	{.name = "Lookup Table 1 Parity Error", .error_code = 0xC1},
	{.name = "Lookup Table 0 Parity Error", .error_code = 0xC0},
	{}
};

/* Error Records per CCPLEX */
static struct error_record ccplex_ers[] = {
	{.name = "CMU:CCPMU", .errx = 1024,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE |
		ERR_CTL_DPMU_DMCE_CRAB_ACC_ERR | ERR_CTL_DPMU_CRAB_ACC_ERR |
		ERR_CTL_DPMU_DMCE_IL1_PAR_ERR | ERR_CTL_DPMU_DMCE_TIMEOUT_ERR |
		ERR_CTL_DPMU_DMCE_UCODE_ERR,
	 .errors = cmu_ccpmu_errors},
	{.name = "SCF:IOB", .errx = 1025,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFIOB_REQ_PAR_ERR | ERR_CTL_SCFIOB_PUT_PAR_ERR |
		ERR_CTL_SCFIOB_PUT_CECC_ERR | ERR_CTL_SCFIOB_PUT_UECC_ERR |
		ERR_CTL_SCFIOB_EVP_ERR | ERR_CTL_SCFIOB_TBX_ERR |
		ERR_CTL_SCFIOB_CRI_ERR | ERR_CTL_SCFIOB_MMCRAB_ERR |
		ERR_CTL_SCFIOB_IHI_ERR | ERR_CTL_SCFIOB_CBB_ERR,
	 .errors = scf_iob_errors},
	{.name = "SCF:SNOC", .errx = 1026,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFSNOC_CPE_TO_ERR | ERR_CTL_SCFSNOC_CPE_RSP_ERR |
		ERR_CTL_SCFSNOC_CPE_REQ_ERR | ERR_CTL_SCFSNOC_DVMU_TO_ERR |
		ERR_CTL_SCFSNOC_DVMU_PAR_ERR | ERR_CTL_SCFSNOC_MISC_CECC_ERR |
		ERR_CTL_SCFSNOC_MISC_UECC_ERR | ERR_CTL_SCFSNOC_MISC_PAR_ERR |
		ERR_CTL_SCFSNOC_MISC_RSP_ERR | ERR_CTL_SCFSNOC_CARVEOUT_ERR |
		ERR_CTL_SCFSNOC_CARVEOUT_CECC_ERR,
	 .errors = scf_snoc_errors},
	{.name = "CMU:CTU", .errx = 1027,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE |
		ERR_CTL_CMUCTU_TRCDMA_PAR_ERR | ERR_CTL_CMUCTU_MCF_PAR_ERR |
		ERR_CTL_CMUCTU_TRL_PAR_ERR | ERR_CTL_CMUCTU_CTU_DATA_PAR_ERR |
		ERR_CTL_CMUCTU_TAG_PAR_ERR | ERR_CTL_CMUCTU_CTU_SNP_ERR |
		ERR_CTL_CMUCTU_TRCDMA_REQ_ERR | ERR_CTL_CMUCTU_RSP_PAR_ERR,
	.errors = cmu_ctu_errors},
	{.name = "SCF:L3_0", .errx = 768,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFL3_CECC_ERR | ERR_CTL_SCFL3_SNOC_INTFC_ERR |
		ERR_CTL_SCFL3_MCF_INTFC_ERR | ERR_CTL_SCFL3_TAG_ERR |
		ERR_CTL_SCFL3_L2DIR_ERR | ERR_CTL_SCFL3_UECC_ERR |
		ERR_CTL_SCFL3_MH_CAM_ERR | ERR_CTL_SCFL3_MH_TAG_ERR |
		ERR_CTL_SCFL3_UNSUPP_REQ_ERR | ERR_CTL_SCFL3_PROT_ERR,
	 .errors = scf_l3_errors},
	{.name = "SCF:L3_1", .errx = 769,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFL3_CECC_ERR | ERR_CTL_SCFL3_SNOC_INTFC_ERR |
		ERR_CTL_SCFL3_MCF_INTFC_ERR | ERR_CTL_SCFL3_TAG_ERR |
		ERR_CTL_SCFL3_L2DIR_ERR | ERR_CTL_SCFL3_UECC_ERR |
		ERR_CTL_SCFL3_MH_CAM_ERR | ERR_CTL_SCFL3_MH_TAG_ERR |
		ERR_CTL_SCFL3_UNSUPP_REQ_ERR | ERR_CTL_SCFL3_PROT_ERR,
	 .errors = scf_l3_errors},
	{.name = "SCF:L3_2", .errx = 770,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFL3_CECC_ERR | ERR_CTL_SCFL3_SNOC_INTFC_ERR |
		ERR_CTL_SCFL3_MCF_INTFC_ERR | ERR_CTL_SCFL3_TAG_ERR |
		ERR_CTL_SCFL3_L2DIR_ERR | ERR_CTL_SCFL3_UECC_ERR |
		ERR_CTL_SCFL3_MH_CAM_ERR | ERR_CTL_SCFL3_MH_TAG_ERR |
		ERR_CTL_SCFL3_UNSUPP_REQ_ERR | ERR_CTL_SCFL3_PROT_ERR,
	 .errors = scf_l3_errors},
	{.name = "SCF:L3_3", .errx = 771,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE | RAS_CTL_CFI |
		ERR_CTL_SCFL3_CECC_ERR | ERR_CTL_SCFL3_SNOC_INTFC_ERR |
		ERR_CTL_SCFL3_MCF_INTFC_ERR | ERR_CTL_SCFL3_TAG_ERR |
		ERR_CTL_SCFL3_L2DIR_ERR | ERR_CTL_SCFL3_UECC_ERR |
		ERR_CTL_SCFL3_MH_CAM_ERR | ERR_CTL_SCFL3_MH_TAG_ERR |
		ERR_CTL_SCFL3_UNSUPP_REQ_ERR | ERR_CTL_SCFL3_PROT_ERR,
	 .errors = scf_l3_errors},
	{.name = "SCFCMU_CLOCKS", .errx = 1028,
	 .err_ctrl = RAS_CTL_ED | RAS_CTL_UE |
		ERR_CTL_SCFCMU_LUT0_PAR_ERR | ERR_CTL_SCFCMU_LUT1_PAR_ERR |
		ERR_CTL_SCFCMU_ADC0_MON_ERR | ERR_CTL_SCFCMU_ADC1_MON_ERR |
		ERR_CTL_SCFCMU_FREQ0_MON_ERR | ERR_CTL_SCFCMU_FREQ1_MON_ERR |
		ERR_CTL_SCFCMU_FREQ2_MON_ERR | ERR_CTL_SCFCMU_FREQ3_MON_ERR,
	 .errors = scfcmu_clocks_errors},
	{}
};

static struct tegra_ras_impl_err_bit t194_ras_impl_err_bit[] = {
	{0xFF,			     ERR_CTL_IFU_ICDP_ERR},	  /*IFU*/
	{ERR_CTL_RET_JSR_GB_ERR,     0xFF},			  /*JSR_RET*/
	{ERR_CTL_MTS_JSR_CARVE_ERR,  ERR_CTL_MTS_JSR_ERRC_ERR},	  /*JSR_MTS*/
	{ERR_CTL_LSD1_CCDSECC_D_ERR, ERR_CTL_LSD1_CCDSECC_S_ERR}, /*LSD_STQ*/
	{ERR_CTL_LSD2_CCDEECC_D_ERR, ERR_CTL_LSD2_CCDEECC_S_ERR}, /*LSD_DCC*/
	{0xFF,			     ERR_CTL_LSD3_L2TLBP_ERR},	  /*LSD_L1HPF*/
	{ERR_CTL_L2_L2PCL_ERR,	     ERR_CTL_L2_SCF2L2C_ECCC_ERR},/*L2*/
	{ERR_CTL_CC_FREQ_MON_ERR,    0xFF},		     /*Cluster_Clocks*/
	{0xFF,			     ERR_CTL_MMU_WCPERR_ERR},	  /*MMU*/
	{ERR_CTL_SCFL3_PROT_ERR,     ERR_CTL_SCFL3_CECC_ERR},	  /*L3*/
	{ERR_CTL_DPMU_DMCE_CRAB_ACC_ERR, 0xFF},			  /*CCPMU*/
	{ERR_CTL_SCFIOB_CBB_ERR,     ERR_CTL_SCFIOB_PUT_CECC_ERR},/*SCF_IOB*/
	{ERR_CTL_SCFSNOC_CPE_TO_ERR, ERR_CTL_SCFSNOC_MISC_CECC_ERR},/*SCFSNOC*/
	{ERR_CTL_CMUCTU_MCF_PAR_ERR, 0xFF},			  /*SCF_CTU*/
	{ERR_CTL_SCFCMU_FREQ0_MON_ERR,	0xFF}			  /*CMU_Clocks*/
};


/* This is called for each online CPU during probe and is also used
 * as hotplug callback to enable RAS every time a core comes online
 */
static void carmel_ras_enable(void *info)
{
	u64 errx;
	int i;
	u8 cpu = smp_processor_id();

	/* Enable Core Error Records */
	for (i = 0; core_ers[i].name; i++) {
		errx = (tegra18_logical_to_cluster(cpu) << 5) +
			(tegra18_logical_to_cpu(cpu) << 4) +
			core_ers[i].errx;

		ras_write_errselr(errx);
		ras_write_error_control(core_ers[i].err_ctrl);
		ras_read_error_control();
	}

	/* Enable Core Cluster Error Records */
	for (i = 0; corecluster_ers[i].name; i++) {
		errx = 512 + (tegra18_logical_to_cluster(cpu) << 4) +
		       corecluster_ers[i].errx;

		ras_write_errselr(errx);
		ras_write_error_control(corecluster_ers[i].err_ctrl);
		ras_read_error_control();
	}

	/* Enable CCPLEX Error Records */
	for (i = 0; ccplex_ers[i].name; i++) {
		ras_write_errselr(ccplex_ers[i].errx);
		ras_write_error_control(ccplex_ers[i].err_ctrl);
		ras_read_error_control();
	}

	pr_info("%s: RAS enabled on cpu%d\n", __func__, cpu);
}

static int carmel_ras_enable_callback(unsigned int cpu)
{

	if (is_this_ras_cpu())
		smp_call_function_single(cpu, carmel_ras_enable, NULL, 1);

	return 0;
}

/* SERROR is triggered for Uncorrectable errors.
 * This is SERR Callback for error records per core.
 * A core will scan all other core's per core error records
 */
static int ras_core_serr_callback(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	u64 err_status;
	int cpu, errx;
	unsigned long flags;
	int retval = 1;
	struct error_record *record;

	if (!is_this_ras_cpu())
		return retval;

	pr_info("%s: Scanning Core Error Records for Uncorrectable Errors\n",
		__func__);
	raw_spin_lock_irqsave(&core_ras_lock, flags);
	/* scan all CPU's per core error records */
	for_each_online_cpu(cpu) {
		if (!tegra_is_cpu_carmel(cpu))
			continue;

		list_for_each_entry(record, &core_ras_list, node) {
			errx = (tegra18_logical_to_cluster(cpu) << 5) +
				(tegra18_logical_to_cpu(cpu) << 4) +
				record->errx;

			ras_write_errselr(errx);
			err_status = ras_read_error_status();
			if ((err_status & ERRi_STATUS_UE) &&
				(err_status & ERRi_STATUS_VALID)) {
				print_error_record(record, err_status, errx);
				retval = 0;
			}
		}
	}
	raw_spin_unlock_irqrestore(&core_ras_lock, flags);
	return retval;
}

static struct serr_hook core_serr_callback = {
	.fn = ras_core_serr_callback
};

static void register_core_er(struct error_record *record)
{
	list_add(&record->node, &core_ras_list);
}

static void unregister_core_er(struct error_record *record)
{
	list_del(&record->node);
}

static void ras_register_core_ers(void)
{
	int i;

	for (i = 0; core_ers[i].name; i++)
		register_core_er(&core_ers[i]);
}

static void ras_unregister_core_ers(void)
{
	int i;

	for (i = 0; core_ers[i].name; i++)
		unregister_core_er(&core_ers[i]);
}

/*
 * This is used to handle FHI or Correctable Errors triggered from
 * error records per core.
 */
static void handle_fhi_core(void)
{
	u64 err_status;
	int cpu, errx;
	struct error_record *record;

	pr_info("%s: Scanning Core Error Records for Correctable Errors\n",
		__func__);
	/* scan all CPU's per core error records */
	for_each_online_cpu(cpu) {
		if (!tegra_is_cpu_carmel(cpu))
			continue;

		list_for_each_entry(record, &core_ras_list, node) {
			errx = (tegra18_logical_to_cluster(cpu) << 5) +
				(tegra18_logical_to_cpu(cpu) << 4) +
				record->errx;

			ras_write_errselr(errx);
			err_status = ras_read_error_status();
			if (get_error_status_ce(err_status) &&
				(err_status & ERRi_STATUS_VALID))
				print_error_record(record, err_status, errx);
		}
	}
}

/* SERROR is triggered for Uncorrectable errors.
 * This is SERR Callback for error records per Core Cluster.
 */
static int ras_corecluster_serr_callback(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	u64 err_status;
	int cpu, errx;
	unsigned long flags;
	int retval = 1;
	struct error_record *record;

	if (!is_this_ras_cpu())
		return retval;

	pr_info("%s:Scanning CoreCluster Error Records for Uncorrectable "
		"Errors\n", __func__);
	raw_spin_lock_irqsave(&corecluster_ras_lock, flags);
	/* scan all CPU's per core error records */
	for_each_online_cpu(cpu) {
		if (!tegra_is_cpu_carmel(cpu))
			continue;

		list_for_each_entry(record, &corecluster_ras_list, node) {
			errx = 512 + (tegra18_logical_to_cluster(cpu) << 4) +
				record->errx;
			ras_write_errselr(errx);
			err_status = ras_read_error_status();

			if ((err_status & ERRi_STATUS_UE) &&
				(err_status & ERRi_STATUS_VALID)) {
				print_error_record(record, err_status, errx);
				retval = 0;
			}
		}
	}
	raw_spin_unlock_irqrestore(&corecluster_ras_lock, flags);
	return retval;
}

static struct serr_hook corecluster_serr_callback = {
	.fn = ras_corecluster_serr_callback
};

static void register_corecluster_er(struct error_record *record)
{
	list_add(&record->node, &corecluster_ras_list);
}

static void unregister_corecluster_er(struct error_record *record)
{
	list_del(&record->node);
}

static void ras_register_corecluster_ers(void)
{
	int i;

	for (i = 0; corecluster_ers[i].name; i++)
		register_corecluster_er(&corecluster_ers[i]);
}

static void ras_unregister_corecluster_ers(void)
{
	int i;

	for (i = 0; corecluster_ers[i].name; i++)
		unregister_corecluster_er(&corecluster_ers[i]);
}

/* This is used to handle FHI or Correctable Errors
 * triggered from error records per Core Cluster
 */
static void handle_fhi_corecluster(void)
{
	u64 err_status;
	int cpu, errx;
	struct error_record *record;

	pr_info("%s:Scanning CoreCluster Error Records for Correctable Errors\n",
		__func__);
	for_each_online_cpu(cpu) {
		if (!tegra_is_cpu_carmel(cpu))
			continue;

		list_for_each_entry(record, &corecluster_ras_list, node) {
			errx = 512 + (tegra18_logical_to_cluster(cpu) << 4) +
				record->errx;
			ras_write_errselr(errx);
			err_status = ras_read_error_status();

			if (get_error_status_ce(err_status) &&
				(err_status & ERRi_STATUS_VALID))
				print_error_record(record, err_status, errx);
		}
	}
}

/* SERROR is triggered for Uncorrectable errors.
 * This is SERR Callback for error records per CCPLEX.
 */
static int ras_ccplex_serr_callback(struct pt_regs *regs, int reason,
			unsigned int esr, void *priv)
{
	u64 err_status;
	unsigned long flags;
	int retval = 1;
	struct error_record *record;

	/* Return if this CPU doesn't support RAS */
	if (!is_this_ras_cpu())
		return retval;

	pr_info("%s: Scanning CCPLEX Error Records for Uncorrectable Errors\n",
		__func__);

	raw_spin_lock_irqsave(&ccplex_ras_lock, flags);
	list_for_each_entry(record, &ccplex_ras_list, node) {
		ras_write_errselr(record->errx);
		err_status = ras_read_error_status();
		if ((err_status & ERRi_STATUS_UE) &&
			(err_status & ERRi_STATUS_VALID)) {
			print_error_record(record, err_status, record->errx);
			retval = 0;
		}
	}
	raw_spin_unlock_irqrestore(&ccplex_ras_lock, flags);
	return is_debug?1 : retval;
}

static struct serr_hook ccplex_serr_callback = {
	.fn = ras_ccplex_serr_callback
};

static void register_ccplex_er(struct error_record *record)
{
	list_add(&record->node, &ccplex_ras_list);
}

static void unregister_ccplex_er(struct error_record *record)
{
	list_del(&record->node);
}

static void ras_register_ccplex_ers(void)
{
	int i;

	for (i = 0; ccplex_ers[i].name; i++)
		register_ccplex_er(&ccplex_ers[i]);
}

static void ras_unregister_ccplex_ers(void)
{
	int i;

	for (i = 0; ccplex_ers[i].name; i++)
		unregister_ccplex_er(&ccplex_ers[i]);
}

/* This is used to handle FHI or Correctable Errors
 * triggered from error records per CCPLEX.
 */
static void handle_fhi_ccplex(void)
{
	u64 err_status;
	struct error_record *record;

	/* Return if  RAS is not supported on this CPU */
	if (!is_this_ras_cpu())
		return;

	pr_info("%s: Scanning CCPLEX Error Records for Correctable Errors\n",
		__func__);

	list_for_each_entry(record, &ccplex_ras_list, node) {
		ras_write_errselr(record->errx);
		err_status = ras_read_error_status();

		if (get_error_status_ce(err_status) &&
			(err_status & ERRi_STATUS_VALID))
			print_error_record(record, err_status, record->errx);
	}
}

/* FHI is triggered for Correctable errors.
 * This is FHI Callback for handling error records per core,
 * per core cluster and per CCPLEX
 */
static void carmel_fhi_callback(void)
{
	handle_fhi_core();
	handle_fhi_corecluster();
	handle_fhi_ccplex();
}

static struct ras_fhi_callback fhi_callback = {
	.fn = carmel_fhi_callback
};

/* This function is used to trigger RAS Errors
 * depending upon the error record and error enabled
 * in the pfgctl passed to it
 */
static int ras_trip(u64 errx, u64 pfgctl)
{
	unsigned long flags, err_ctl;

	flags = arch_local_save_flags();

	/* Print some debug information */
	pr_crit("%s: DAIF = 0x%lx\n", __func__, flags);
	if (flags & 0x4) {
		pr_crit("%s: \"A\" not set", __func__);
		return 0;
	}

	ras_write_errselr(errx);
	pr_info("%s: Error Record Selected = %lld\n",
		__func__, ras_read_errselr());

	err_ctl = ras_read_error_control();
	pr_crit("%s: Error Record ERRCTL = 0x%lx\n", __func__, err_ctl);
	if (!(err_ctl & RAS_CTL_ED)) {
		pr_crit("%s: Error Detection is not enabled", __func__);
		return 0;
	}

	/* Write some value to MISC0 */
	ras_write_error_misc0(ERRi_MISC0_CONST);
	/* Write some value to MISC1 */
	ras_write_error_misc1(ERRi_MISC1_CONST);
	/* Write some value to ADDR */
	ras_write_error_addr(ERRi_ADDR_CONST);
	is_debug = 1;
	/* Set coundown value */
	ras_write_pfg_cdn(ERRi_PFGCDN_CDN_1);
	/* Write to ERR<X>PFGCTL */
	pr_info("%s: Writing 0x%llx to ERRXPFGCTL\n", __func__, pfgctl);
	ras_write_pfg_control(pfgctl);
	return 0;
}

static int l3_cecc_put(void *data, u64 val)
{
	return ras_trip(ERRX_SCFL3, val);
}

/* This will return the special value to be written to debugfs node
 * L3_0_CECC_ERR-trip to trigger L3_0_CECC Error
 * Value is written to PFGCTL register.
 * Enables bits CECC_ERR|CDNEN|MV|AV|CE|UC
 */
static int l3_cecc_get(void *data, u64 *val)
{
	*val = ERRi_PFGCTL_UC | ERRi_PFGCTL_CE | ERRi_PFGCTL_CDNEN |
		ERR_CTL_SCFL3_CECC_ERR;
	return 0;
}

static int scf_iob_cecc_put(void *data, u64 val)
{
	return ras_trip(ERRX_SCFIOB, val);
}

/* This will return the special value to be written to debugfs node
 * SCF_IOB-PUTDATA_CECC_ERR-trip to trigger SCF IOB PUTDATA_CECC Error
 */
static int scf_iob_cecc_get(void *data, u64 *val)
{
	*val = ERRi_PFGCTL_UC | ERRi_PFGCTL_CE | ERRi_PFGCTL_CDNEN |
		ERR_CTL_SCFIOB_PUT_CECC_ERR;
	return 0;
}

static int scf_iob_cbb_put(void *data, u64 val)
{
	return ras_trip(ERRX_SCFIOB, val);
}

/* This will return the special value to be written to debugfs node
 * SCF_IOB-CBB_ERR-trip to trigger SCF IOB CBB Error
 */
static int scf_iob_cbb_get(void *data, u64 *val)
{
	*val = ERRi_PFGCTL_UC | ERRi_PFGCTL_CE | ERRi_PFGCTL_CDNEN |
		ERR_CTL_SCFIOB_CBB_ERR;
	return 0;
}


/*
 * Parse fields from input to use further for injecting RAS error.
 * These fields are used to get error record number which will be
 * used to select specific error record using ERRSELR_EL1 for
 * injecting error.
 * i/p field "val" format is "EEDDCCBBAA", where:
 *		AA[00-07] - Unit
 *		BB[08-15] - Error type(Corr is 0, UnCorr is 1)
 *		CC[16-23] - Logical_CPU_ID
 *		DD[24-31] - Logical_Cluster_ID
 *		EE[32-39] - L3_Bank_ID
 */
static int ras_mca_get_record_errselr(u64 val, u64 *err_inject)
{
	int unit = RAS_EXTRACT(val, 7, 0);
	int uncorr_err = RAS_EXTRACT(val, 15, 8);
	int Logical_CPU_ID = RAS_EXTRACT(val, 23, 16);
	int Logical_Cluster_ID = RAS_EXTRACT(val, 31, 24);
	int L3_Bank_ID = RAS_EXTRACT(val, 39, 32);

	*err_inject = ERRi_PFGCTL_UC | ERRi_PFGCTL_CE | ERRi_PFGCTL_CDNEN;

	pr_info("Unit:0x%x Err_type:%s Logical_CPUID:0x%x Logical_ClusterID:"
		"0x%x L3_BankID:0x%x\n", unit, uncorr_err?"UnCorr":"Corr",
		Logical_CPU_ID, Logical_Cluster_ID, L3_Bank_ID);

	if (uncorr_err)
		*err_inject |= t194_ras_impl_err_bit[unit].uncorr_bit;
	else
		*err_inject |= t194_ras_impl_err_bit[unit].corr_bit;

	switch (unit) {
	case IFU:
		return 0*256 + Logical_CPU_ID*16 + 0;
	case JSR_RET:
		return 0*256 + Logical_CPU_ID*16 + 1;
	case JSR_MTS:
		return 0*256 + Logical_CPU_ID*16 + 2;
	case LSD_STQ:
		return 0*256 + Logical_CPU_ID*16 + 3;
	case LSD_DCC:
		return 0*256 + Logical_CPU_ID*16 + 4;
	case LSD_L1HPF:
		return 0*256 + Logical_CPU_ID*16 + 5;
	case L2:
		return 2*256 + Logical_Cluster_ID*16 + 0;
	case Cluster_Clocks:
		return 2*256 + Logical_Cluster_ID*16 + 1;
	case MMU:
		return 2*256 + Logical_Cluster_ID*16 + 2;
	case L3:
		return 3*256 + L3_Bank_ID;
	case CCPMU:
		return 4*256 + 0;
	case SCF_IOB:
		return 4*256 + 1;
	case SCF_SNOC:
		return 4*256 + 2;
	case SCF_CTU:
		return 4*256 + 3;
	case CMU_Clocks:
		return 4*256 + 4;
	default:
		return 0xFF;
	}
}

/*
 * Print help for error injection and basic register info.
 */
static int ras_mca_get(void *data, u64 *val)
{
	unsigned long errctl = ras_read_error_control();
	*val = ras_read_pfg_control();

	pr_info("ERXPFGCTL_EL1:0x%llx ERR<n>CTLR:0x%lx\n", *val, errctl);
	pr_info("Please write data in below format to this node for "
		"injecting RAS error.\n\techo EEDDCCBBAA > RAS_MCA_ERR-trip\n"
		"where:\n\t"
		"  EE[32-39] - L3_Bank_ID\n\t"
		"  DD[24-31] - Logical_Cluster_ID\n\t"
		"  CC[16-23] - Logical_CPU_ID\n\t"
		"  BB[08-15] - Error type(Corr is 0, UnCorr is 1)\n\t"
		"  AA[00-07] - Unit\n\t"
		"    Unit values are:\n\t\t"
		"IFU:00\n\t\tJSR_RET:01\n\t\tJSR_MTS:02\n\t\tLSD_STQ:03\n\t\t"
		"LSD_DCC:04\n\t\tLSD_L1HPF:05\n\t\tL2:06\n\t\t"
		"Cluster_Clocks:07\n\t\tMMU:08\n\t\tL3:09\n\t\tCCPMU:0A\n\t\t"
		"SCF_IOB:0B\n\t\tSCF_SNOC:0C\n\t\tSCF_CTU:0D\n\t\t"
		"CMU_Clocks:0E\n\n"
	       );

	return 0;
}

/*
 * Read input(i/p) value and inject error based on value.
 */
static int ras_mca_put(void *data, u64 val)
{
	int err_record_no = 0;
	u64 err_inject = 0;

	err_record_no = ras_mca_get_record_errselr(val, &err_inject);

	pr_info("Errx(ERRSELR_EL1):0x%x ERXPFGCTL_EL1:0x%llx PFGCTL_bits:"
		"0x%llx\n", err_record_no, ras_read_pfg_control(), err_inject);

	if (err_inject == 0xFF || err_record_no == 0xFF)
		pr_info("Invalid input.\n");
	else
		return ras_trip(err_record_no, err_inject);

	return 0;
}

static int ras_mca_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, ras_mca_get, ras_mca_put,
				"0x%08lx");
}

static int scf_iob_cbb_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, scf_iob_cbb_get, scf_iob_cbb_put,
				"0x%08lx");
}

static int scf_iob_cecc_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, scf_iob_cecc_get, scf_iob_cecc_put,
				"0x%08lx");
}

static int l3_cecc_open(struct inode *inode, struct file *file)
{
	return simple_attr_open(inode, file, l3_cecc_get, l3_cecc_put,
				"0x%08lx");
}

static const struct file_operations fops_scf_iob_cbb = {
	.read =		simple_attr_read,
	.write =	simple_attr_write,
	.open =		scf_iob_cbb_open,
	.llseek =	noop_llseek,
};

static const struct file_operations fops_scf_iob_cecc = {
	.read =		simple_attr_read,
	.write =	simple_attr_write,
	.open =		scf_iob_cecc_open,
	.llseek =	noop_llseek,
};

static const struct file_operations fops_l3_cecc = {
	.read =		simple_attr_read,
	.write =	simple_attr_write,
	.open =		l3_cecc_open,
	.llseek =	noop_llseek,
};

static const struct file_operations fops_ras_mca = {
	.read =		simple_attr_read,
	.write =	simple_attr_write,
	.open =		ras_mca_open,
	.llseek =	noop_llseek,
};

static int ras_carmel_dbgfs_init(void)
{

	/* Install debugfs nodes to test RAS */
	debugfs_dir = debugfs_create_dir("carmel_ras", NULL);
	if (!debugfs_dir) {
		pr_err("Error creating carmel_ras debugfs dir.\n");
		return -ENODEV;
	}

	debugfs_node = debugfs_create_file("SCF_IOB-CBB_ERR-trip", 0600,
			 debugfs_dir, NULL, &fops_scf_iob_cbb);
	if (!debugfs_node) {
		pr_err("Error creating SCF_IOB-CBB_ERR-trip debugfs node.\n");
		return -ENODEV;
	}

	debugfs_node = debugfs_create_file("SCF_IOB-PUTDATA_CECC_ERR-trip",
			 0600, debugfs_dir, NULL, &fops_scf_iob_cecc);
	if (!debugfs_node) {
		pr_err("Error creating SCF_IOB-PUTDATA_CECC_ERR-trip debugfs node.\n");
		return -ENODEV;
	}

	debugfs_node = debugfs_create_file("L3_0_CECC_ERR-trip", 0600,
					debugfs_dir, NULL, &fops_l3_cecc);
	if (!debugfs_node) {
		pr_err("Error creating L3_0_CECC_ERR-trip debugfs node.\n");
		return -ENODEV;
	}

	debugfs_node = debugfs_create_file("RAS_MCA_ERR-trip", 0600,
					debugfs_dir, NULL, &fops_ras_mca);
	if (!debugfs_node) {
		pr_err("Error creating L3_0_CECC_ERR-trip debugfs node.\n");
		return -ENODEV;
	}
	return 0;
}

static int ras_carmel_probe(struct platform_device *pdev)
{
	int cpu, do_init = 0, ret = -1;
	struct device *dev = &pdev->dev;

	if (!is_ras_ready()) {
		dev_info(dev, "Deferring probe, arm64_ras hasnt been probed yet");
		return -EPROBE_DEFER;
	}

	/* probe only if RAS is supported on any of the online CPUs */
	for_each_online_cpu(cpu) {
		if (tegra_is_cpu_carmel(cpu) && is_ras_cpu(cpu))
			do_init = 1;
	}

	if (!do_init) {
		dev_info(dev, "None of the CPUs support RAS");
		return 0;
	}

	ras_register_core_ers();
	ras_register_corecluster_ers();
	ras_register_ccplex_ers();

	/* register FHI callback for Correctable Errors */
	ret = register_fhi_callback(&fhi_callback, pdev);
	if (ret) {
		dev_err(dev, "Failed to register FHI callback\n");
		return -ENOENT;
	}

	/* Ensure that any CPU brought online sets up RAS */
	ret = cpuhp_setup_state(CPUHP_AP_ONLINE_DYN,
				  "ras_carmel:online",
				  carmel_ras_enable_callback,
				  NULL);
	if (ret < 0) {
		dev_err(dev, "unable to register cpu hotplug state\n");
		return ret;
	}

	hp_state = ret;

	/* register SERR for Uncorrectable Errors */
	register_serr_hook(&core_serr_callback);
	register_serr_hook(&corecluster_serr_callback);
	register_serr_hook(&ccplex_serr_callback);

	ret = ras_carmel_dbgfs_init();
	if (ret)
		return ret;

	dev_info(dev, "probed");
	return 0;
}

static int ras_carmel_remove(struct platform_device *pdev)
{
	unregister_fhi_callback(&fhi_callback);

	unregister_serr_hook(&core_serr_callback);
	unregister_serr_hook(&corecluster_serr_callback);
	unregister_serr_hook(&ccplex_serr_callback);

	cpuhp_remove_state(hp_state);

	ras_unregister_core_ers();
	ras_unregister_corecluster_ers();
	ras_unregister_ccplex_ers();

	return 0;
}

static const struct of_device_id ras_carmel_of_match[] = {
	{
		.name = "carmel_ras",
		.compatible = "nvidia,carmel-ras",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ras_carmel_of_match);

static struct platform_driver ras_carmel_driver = {
	.probe = ras_carmel_probe,
	.remove = ras_carmel_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "carmel_ras",
		.of_match_table = of_match_ptr(ras_carmel_of_match),
	},
};

static int __init ras_carmel_init(void)
{
	return platform_driver_register(&ras_carmel_driver);
}

static void __exit ras_carmel_exit(void)
{
	platform_driver_unregister(&ras_carmel_driver);
}

arch_initcall(ras_carmel_init);
module_exit(ras_carmel_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Carmel RAS handler");
