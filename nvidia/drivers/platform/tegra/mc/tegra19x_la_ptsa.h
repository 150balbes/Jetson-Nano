/*
 * Copyright (C) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MACH_TEGRA_19X_LA_PTSA_H
#define _MACH_TEGRA_19X_LA_PTSA_H

#include <asm/io.h>
#include <linux/types.h>
#include <linux/platform/tegra/tegra_emc.h>
#include <linux/platform/tegra/mc.h>
#include <linux/platform/tegra/latency_allowance.h>
#include <linux/t194_nvg.h>
#include "mc-regs-t19x.h"
#include "la_priv.h"
#include "fixed_point.h"
#include "nvrm_drf.h"


#define T19X_MC_LA_MAX_VALUE                    2047U
#define DRAM_TYPE_MASK                          0x3
#define DRAM_LPDDR4                             1
#define DRAM_CH_MASK                            0xffff
#define MEM_MODE_SHIFT                          16
#define MEM_MODE_MASK                           0x3

#define MAX_TEGRA_DDA_NAME_SIZE 128
#define MAX_TEGRA_LA_CLIENT_NAME_SIZE 128
#define MAX_TEGRA_MC_REG_NAME_SIZE 128

enum tegra_t19x_la_id {
	TEGRA_T19X_LA_AONDMAR_ID,
	TEGRA_T19X_LA_AONDMAW_ID,
	TEGRA_T19X_LA_AONR_ID,
	TEGRA_T19X_LA_AONW_ID,
	TEGRA_T19X_LA_APEDMAR_ID,
	TEGRA_T19X_LA_APEDMAW_ID,
	TEGRA_T19X_LA_APER_ID,
	TEGRA_T19X_LA_APEW_ID,
	TEGRA_T19X_LA_AXIAPR_ID,
	TEGRA_T19X_LA_AXIAPW_ID,
	TEGRA_T19X_LA_AXISR_ID,
	TEGRA_T19X_LA_AXISW_ID,
	TEGRA_T19X_LA_BPMPDMAR_ID,
	TEGRA_T19X_LA_BPMPDMAW_ID,
	TEGRA_T19X_LA_BPMPR_ID,
	TEGRA_T19X_LA_BPMPW_ID,
	TEGRA_T19X_LA_CIFLL_WR_ID,
	TEGRA_T19X_LA_DLA0FALRDB_ID,
	TEGRA_T19X_LA_DLA0RDA_ID,
	TEGRA_T19X_LA_DLA0FALWRB_ID,
	TEGRA_T19X_LA_DLA0WRA_ID,
	TEGRA_T19X_LA_DLA0RDA1_ID,
	TEGRA_T19X_LA_DLA1RDA1_ID,
	TEGRA_T19X_LA_DLA1FALRDB_ID,
	TEGRA_T19X_LA_DLA1RDA_ID,
	TEGRA_T19X_LA_DLA1FALWRB_ID,
	TEGRA_T19X_LA_DLA1WRA_ID,
	TEGRA_T19X_LA_EQOSR_ID,
	TEGRA_T19X_LA_EQOSW_ID,
	TEGRA_T19X_LA_ETRR_ID,
	TEGRA_T19X_LA_ETRW_ID,
	TEGRA_T19X_LA_HOST1XDMAR_ID,
	TEGRA_T19X_LA_HDAR_ID,
	TEGRA_T19X_LA_HDAW_ID,
	TEGRA_T19X_LA_ISPFALR_ID,
	TEGRA_T19X_LA_ISPRA_ID,
	TEGRA_T19X_LA_ISPWA_ID,
	TEGRA_T19X_LA_ISPWB_ID,
	TEGRA_T19X_LA_ISPFALW_ID,
	TEGRA_T19X_LA_ISPRA1_ID,
	TEGRA_T19X_LA_MIU0R_ID,
	TEGRA_T19X_LA_MIU0W_ID,
	TEGRA_T19X_LA_MIU1R_ID,
	TEGRA_T19X_LA_MIU1W_ID,
	TEGRA_T19X_LA_MIU2R_ID,
	TEGRA_T19X_LA_MIU2W_ID,
	TEGRA_T19X_LA_MIU3R_ID,
	TEGRA_T19X_LA_MIU3W_ID,
	TEGRA_T19X_LA_MIU4R_ID,
	TEGRA_T19X_LA_MIU4W_ID,
	TEGRA_T19X_LA_MIU5R_ID,
	TEGRA_T19X_LA_MIU5W_ID,
	TEGRA_T19X_LA_MIU6R_ID,
	TEGRA_T19X_LA_MIU6W_ID,
	TEGRA_T19X_LA_MIU7R_ID,
	TEGRA_T19X_LA_MIU7W_ID,
	TEGRA_T19X_LA_MPCORER_ID,
	TEGRA_T19X_LA_MPCOREW_ID,
	TEGRA_T19X_LA_NVDECSRD_ID,
	TEGRA_T19X_LA_NVDECSWR_ID,
	TEGRA_T19X_LA_NVDEC1SRD_ID,
	TEGRA_T19X_LA_NVDECSRD1_ID,
	TEGRA_T19X_LA_NVDEC1SRD1_ID,
	TEGRA_T19X_LA_NVDEC1SWR_ID,
	TEGRA_T19X_LA_NVDISPLAYR_ID,
	TEGRA_T19X_LA_NVENCSRD_ID,
	TEGRA_T19X_LA_NVENCSWR_ID,
	TEGRA_T19X_LA_NVENC1SRD_ID,
	TEGRA_T19X_LA_NVENC1SWR_ID,
	TEGRA_T19X_LA_NVENC1SRD1_ID,
	TEGRA_T19X_LA_NVENCSRD1_ID,
	TEGRA_T19X_LA_NVJPGSRD_ID,
	TEGRA_T19X_LA_NVJPGSWR_ID,
	TEGRA_T19X_LA_PCIE0R_ID,
	TEGRA_T19X_LA_PCIE0W_ID,
	TEGRA_T19X_LA_PCIE1R_ID,
	TEGRA_T19X_LA_PCIE1W_ID,
	TEGRA_T19X_LA_PCIE2AR_ID,
	TEGRA_T19X_LA_PCIE2AW_ID,
	TEGRA_T19X_LA_PCIE3R_ID,
	TEGRA_T19X_LA_PCIE3W_ID,
	TEGRA_T19X_LA_PCIE4R_ID,
	TEGRA_T19X_LA_PCIE4W_ID,
	TEGRA_T19X_LA_PCIE5R_ID,
	TEGRA_T19X_LA_PCIE5W_ID,
	TEGRA_T19X_LA_PCIE0R1_ID,
	TEGRA_T19X_LA_PCIE5R1_ID,
	TEGRA_T19X_LA_PVA0RDA_ID,
	TEGRA_T19X_LA_PVA0RDB_ID,
	TEGRA_T19X_LA_PVA0RDC_ID,
	TEGRA_T19X_LA_PVA0WRA_ID,
	TEGRA_T19X_LA_PVA0WRB_ID,
	TEGRA_T19X_LA_PVA0WRC_ID,
	TEGRA_T19X_LA_PVA0RDA1_ID,
	TEGRA_T19X_LA_PVA0RDB1_ID,
	TEGRA_T19X_LA_PVA1RDA_ID,
	TEGRA_T19X_LA_PVA1RDB_ID,
	TEGRA_T19X_LA_PVA1RDC_ID,
	TEGRA_T19X_LA_PVA1WRA_ID,
	TEGRA_T19X_LA_PVA1WRB_ID,
	TEGRA_T19X_LA_PVA1WRC_ID,
	TEGRA_T19X_LA_PVA1RDA1_ID,
	TEGRA_T19X_LA_PVA1RDB1_ID,
	TEGRA_T19X_LA_RCEDMAR_ID,
	TEGRA_T19X_LA_RCEDMAW_ID,
	TEGRA_T19X_LA_RCER_ID,
	TEGRA_T19X_LA_RCEW_ID,
	TEGRA_T19X_LA_SATAR_ID,
	TEGRA_T19X_LA_SATAW_ID,
	TEGRA_T19X_LA_SCEDMAR_ID,
	TEGRA_T19X_LA_SCEDMAW_ID,
	TEGRA_T19X_LA_SCER_ID,
	TEGRA_T19X_LA_SCEW_ID,
	TEGRA_T19X_LA_SDMMCRAB_ID,
	TEGRA_T19X_LA_SDMMCWAB_ID,
	TEGRA_T19X_LA_SDMMCRA_ID,
	TEGRA_T19X_LA_SDMMCWA_ID,
	TEGRA_T19X_LA_SDMMCR_ID,
	TEGRA_T19X_LA_SDMMCW_ID,
	TEGRA_T19X_LA_SESRD_ID,
	TEGRA_T19X_LA_SESWR_ID,
	TEGRA_T19X_LA_TSECSRDB_ID,
	TEGRA_T19X_LA_TSECSWRB_ID,
	TEGRA_T19X_LA_TSECSRD_ID,
	TEGRA_T19X_LA_TSECSWR_ID,
	TEGRA_T19X_LA_UFSHCR_ID,
	TEGRA_T19X_LA_UFSHCW_ID,
	TEGRA_T19X_LA_VIW_ID,
	TEGRA_T19X_LA_VICSRD_ID,
	TEGRA_T19X_LA_VICSWR_ID,
	TEGRA_T19X_LA_VICSRD1_ID,
	TEGRA_T19X_LA_VIFALR_ID,
	TEGRA_T19X_LA_VIFALW_ID,
	TEGRA_T19X_LA_WCAM_ID,
	TEGRA_T19X_LA_XUSB_HOSTR_ID,
	TEGRA_T19X_LA_XUSB_HOSTW_ID,
	TEGRA_T19X_LA_XUSB_DEVR_ID,
	TEGRA_T19X_LA_XUSB_DEVW_ID,
	TEGRA_T19X_LA_NVLRHP_ID,
	TEGRA_T19X_LA_DGPU_ID,
	TEGRA_T19X_LA_IGPU_ID,
	TEGRA_T19X_LA_MAX_ID
};

/*
 * T19X only supports LP4 and LP4X with 8 and 16 channels.
 * TODO: Check and remove others if not needed.
 */
enum tegra_dram_t {
	TEGRA_DDR3,
	TEGRA_LP3,
	TEGRA_LP4,
	TEGRA_DDR3_1CH,
	TEGRA_DDR3_2CH,
	TEGRA_LP3_1CH,
	TEGRA_LP3_2CH,
	TEGRA_LP4_2CH,
	TEGRA_LP4_4CH,
	TEGRA_LP4_8CH,
	TEGRA_LP4X_8CH,
	TEGRA_LP4_16CH,
	TEGRA_LP4X_16CH
};

#define MAX_TEGRA_LA_CLIENT_NAME_SIZE 128

/* Frequency range used to determine different DDA parameters */
struct dda_freq_range {
	struct fixed_point lo_freq;
	struct fixed_point hi_freq;
	struct fixed_point lo_gd; /* Grantdec at lowest freq in this range */
	struct fixed_point hi_gd; /* Grantdec at highest freq in this range */
	unsigned int emc_mc_ratio;
	unsigned int valid;
};

struct mc_settings_info {
	enum tegra_dram_t dram_type;
	int num_channels;
	int mccif_buf_sz_bytes;
	int stat_lat_minus_snaparb2rs;
	int exp_time;
	int dram_width_bits;
	struct fixed_point cons_mem_eff;
	int stat_lat_snaparb_rs;
	int row_sorter_sz_bytes;
	struct fixed_point max_drain_time_usec;
	struct fixed_point ns_per_tick;
	struct fixed_point max_lat_all_usec;
	int ring2_dda_rate;
	int ring2_dda_en;
	int siso_hp_en;
	int vi_always_hp;
	int bytes_per_dram_clk;
	struct fixed_point hub_dda_div;
	struct fixed_point ring0_dda_div;
	int dram_to_emc_freq_ratio;
	struct fixed_point disp_catchup_factor;
	struct fixed_point dda_bw_margin;
	struct fixed_point two_stge_ecc_iso_dda_bw_margin;
	struct fixed_point mc_emc_same_freq_thr;
	struct fixed_point lowest_dram_freq;
	struct fixed_point highest_dram_freq;
	int ptsa_reg_length_bits;
	struct fixed_point grant_dec_multiplier;
	struct fixed_point max_gd;
	int set_perf_regs;
	int hub2mcf_dda;
	int igpu_mcf_dda;
	int tsa_arb_fix;
	int iso_holdoff_override;
	int pcfifo_interlock;
	int en_ordering;
	int set_order_id;
	int hp_cpu_throttle_en;
	int override_isoptc_hub_mapping;
	int override_hub_vcarb_type;
	int override_hub_vcarb_wt;
	int override_iso_tbu_cchk_en_ctrl;
	int hub2mcf_dda_rate;
	int hub2mcf_dda_max;
	int mssnvlink_mcf_igpu_dda_rate;
	int mssnvlink_mcf_igpu_dda_max;
	int isoptc_hub_num;
	int hub_vcarb_type;
	int hub_vcarb_niso_wt;
	int hub_vcarb_siso_wt;
	int hub_vcarb_iso_wt;
	int iso_tbu_cchk_en_ctrl;
	struct dda_freq_range freq_range;
};

enum tegra_dda_id {
	TEGRA_DDA_AONPC_ID,
	TEGRA_DDA_APB_ID,
	TEGRA_DDA_AUD_ID,
	TEGRA_DDA_BPMPPC_ID,
	TEGRA_DDA_CIFLL_ISO_ID,
	TEGRA_DDA_CIFLL_SISO_ID,
	TEGRA_DDA_CIFLL_NISO_ID,
	TEGRA_DDA_CIFLL_RING0X_ID,
	TEGRA_DDA_DIS_ID,
	TEGRA_DDA_DLA0FALPC_ID,
	TEGRA_DDA_DLA0XA_ID,
	TEGRA_DDA_DLA0XA2_ID,
	TEGRA_DDA_DLA0XA3_ID,
	TEGRA_DDA_DLA1FALPC_ID,
	TEGRA_DDA_DLA1XA_ID,
	TEGRA_DDA_DLA1XA2_ID,
	TEGRA_DDA_DLA1XA3_ID,
	TEGRA_DDA_EQOSPC_ID,
	TEGRA_DDA_HDAPC_ID,
	TEGRA_DDA_HOST_ID,
	TEGRA_DDA_ISP_ID,
	TEGRA_DDA_ISP2PC_ID,
	TEGRA_DDA_ISPPC_ID,
	TEGRA_DDA_JPG_ID,
	TEGRA_DDA_MIU0_ID,
	TEGRA_DDA_MIU1_ID,
	TEGRA_DDA_MIU2_ID,
	TEGRA_DDA_MIU3_ID,
	TEGRA_DDA_MIU4_ID,
	TEGRA_DDA_MIU5_ID,
	TEGRA_DDA_MIU6_ID,
	TEGRA_DDA_MIU7_ID,
	TEGRA_DDA_MLL_MPCORER_ID,
	TEGRA_DDA_MSE_ID,
	TEGRA_DDA_MSE2_ID,
	TEGRA_DDA_MSE3_ID,
	TEGRA_DDA_MSEA_ID,
	TEGRA_DDA_MSEB_ID,
	TEGRA_DDA_MSEB1_ID,
	TEGRA_DDA_NIC_ID,
	TEGRA_DDA_NVD_ID,
	TEGRA_DDA_NVD2_ID,
	TEGRA_DDA_NVD3_ID,
	TEGRA_DDA_NVD4_ID,
	TEGRA_DDA_NVD5_ID,
	TEGRA_DDA_NVD6_ID,
	TEGRA_DDA_PCIE0X_ID,
	TEGRA_DDA_PCIE0X2_ID,
	TEGRA_DDA_PCIE0XA_ID,
	TEGRA_DDA_PCIE1X_ID,
	TEGRA_DDA_PCIE1XA_ID,
	TEGRA_DDA_PCIE4X_ID,
	TEGRA_DDA_PCIE4XA_ID,
	TEGRA_DDA_PCIE5X_ID,
	TEGRA_DDA_PCIE5X2_ID,
	TEGRA_DDA_PCIE5XA_ID,
	TEGRA_DDA_PVA0XA_ID,
	TEGRA_DDA_PVA0XA2_ID,
	TEGRA_DDA_PVA0XA3_ID,
	TEGRA_DDA_PVA0XB_ID,
	TEGRA_DDA_PVA0XB2_ID,
	TEGRA_DDA_PVA0XB3_ID,
	TEGRA_DDA_PVA0XC_ID,
	TEGRA_DDA_PVA1XA_ID,
	TEGRA_DDA_PVA1XA2_ID,
	TEGRA_DDA_PVA1XA3_ID,
	TEGRA_DDA_PVA1XB_ID,
	TEGRA_DDA_PVA1XB2_ID,
	TEGRA_DDA_PVA1XB3_ID,
	TEGRA_DDA_PVA1XC_ID,
	TEGRA_DDA_RCEPC_ID,
	TEGRA_DDA_RING2_ID,
	TEGRA_DDA_SAX_ID,
	TEGRA_DDA_SCEPC_ID,
	TEGRA_DDA_SD_ID,
	TEGRA_DDA_SDM_ID,
	TEGRA_DDA_SMMU_SMMU_ID,
	TEGRA_DDA_UFSHCPC_ID,
	TEGRA_DDA_UFSHCPC2_ID,
	TEGRA_DDA_USBD_ID,
	TEGRA_DDA_USBD2_ID,
	TEGRA_DDA_USBX_ID,
	TEGRA_DDA_USBX2_ID,
	TEGRA_DDA_VE_ID,
	TEGRA_DDA_VICPC_ID,
	TEGRA_DDA_VICPC2_ID,
	TEGRA_DDA_VICPC3_ID,
	TEGRA_DDA_MAX_ID
};

enum tegra_iso_t {
	TEGRA_HISO,
	TEGRA_SISO,
	TEGRA_NISO
};

#define MAX_TEGRA_DDA_NAME_SIZE 128

struct dda_info {
	char name[MAX_TEGRA_DDA_NAME_SIZE];
	unsigned int rate; /* BW rate expressed as 0.rate */
	struct fixed_point frac; /* fixed point fraction for the BW rate */
	int frac_valid; /* frac is valid. */
	int max; /* Maximum value of accum allowed. Expressed as max.0 */
	int min; /* Minimum value of accum allowed. Expressed as min.0 */
	struct fixed_point bw; /* actual BW requested */
	unsigned int ring; /* what ring this client feeds into. */
	enum tegra_iso_t iso_type; /* HISO, SISO, or NISO*/
	unsigned int rate_reg_addr;
	unsigned long mask;
	struct fixed_point dda_div;
};

enum tegra_kern_init_mc_id {
	TEGRA_MC_HUB2MCF_REQ_DDA_ENABLE_ID,
	TEGRA_MC_HUB_HUB2MCF_REQ_DDA_RATE_ID,
	TEGRA_MC_HUBORD_HUB2MCF_REQ_DDA_RATE_ID,
	TEGRA_MC_HUBINT_HUB2MCF_REQ_DDA_RATE_ID,
	TEGRA_MC_HUB_HUB2MCF_REQ_DDA_MAX_ID,
	TEGRA_MC_HUBORD_HUB2MCF_REQ_DDA_MAX_ID,
	TEGRA_MC_HUBINT_HUB2MCF_REQ_DDA_MAX_ID,
	TEGRA_MC_CIFLL_NVLRHP_LATENCY_ALLOWANCE_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_APER_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_APEW_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_APEDMAR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_APEDMAW_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_EQOSR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_EQOSW_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_HDAR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_HDAW_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_NVDISPLAYR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_NVDISPLAYR1_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_PTCR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_VIFALR_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_VIFALW_ID,
	TEGRA_MC_TXN_OVERRIDE_CONFIG_VIW_ID,
	TEGRA_MC_CONFIG_TSA_SINGLE_ARB_ENABLE_ID,
	TEGRA_MC_EMEM_ARB_OVERRIDE_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG0_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG1_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG2_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG3_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG4_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG5_ID,
	TEGRA_MC_PCFIFO_CLIENT_CONFIG7_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE5W_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE0W_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE4W_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_XUSB_HOSTW_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_XUSB_DEVW_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_SATAW_ID,
	TEGRA_MC_CLIENT_ORDER_ID_9_ID,
	TEGRA_MC_CLIENT_ORDER_ID_28_ID,
	TEGRA_MC_FREE_BANK_QUEUES_ID,
	TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_OVERRIDE_ID,
	TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_ID,
	TEGRA_MC_HUB_VC_ARB_SEL_ID,
	TEGRA_MC_MC_SMMU_ISO_TBU_CCHK_REQ_PRI_CTRL_ID,
	TEGRA_KERN_INIT_MC_MAX_ID
};

enum tegra_kern_init_mssnvlink_id {
	TEGRA_MSSNVLINK_MASTER_MCF_DDA_ID,
	TEGRA_KERN_INIT_MSSNVLINK_MAX_ID
};

enum tegra_kern_init_mcpcie_id {
	TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE1W_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE2AW_ID,
	TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE3W_ID,
	TEGRA_MC_CLIENT_ORDER_ID_27_ID,
	TEGRA_KERN_INIT_MCPCIE_MAX_ID
};

#define MAX_TEGRA_MC_REG_NAME_SIZE 128

struct reg_info {
	char name[MAX_TEGRA_MC_REG_NAME_SIZE];
	unsigned int val;
	unsigned int offset;
	int dirty;
};

struct la_ptsa_core {
	/* Gets the initial la value given client type and mc settings */
	unsigned int (*get_init_la)(
		enum la_client_type client_type,
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Init la_client_info_array for all clients */
	void (*la_info_array_init)(
		struct la_client_info *info_array,
		int *gen_to_t19x_la_id,
		int *t19x_to_gen_la_id,
		int *t19x_la_kern_init,
		struct mc_settings_info *mc_set,
		unsigned int *error);

	/* Updates mc parameters given dram type */
	void (*mc_settings_init)(
		enum tegra_dram_t dram_type,
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Overrides a mc_settings_info type */
	void (*mc_settings_override)(
		struct mc_settings_info info,
		struct mc_settings_info *mc_settings_ptr);

	/* Calculates the display read latency allowance given the bw */
	void (*get_disp_rd_lat_allow_given_disp_bw)(
		struct mc_settings_info *mc_settings_ptr,
		struct fixed_point emc_freq_mhz,
		struct fixed_point dis_bw, /* MBps */
		int *disp_la,
		struct fixed_point *drain_time_usec,
		struct fixed_point *la_bw_up_bnd_usec,
		unsigned int *error);

	/* Init dda_info_array for all clients */
	void (*dda_info_array_init)(
		struct dda_info *inf_arr,
		int info_array_size,
		struct mc_settings_info *mc_set,
		unsigned int *error);

	/* Updates DDA MIN/MAX values for kernel init */
	void (*update_new_dda_minmax_kern_init)(
		struct dda_info *dda_info_array,
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Updates DDA RATE values for kernel init */
	void (*update_new_dda_rate_frac_kern_init)(
		struct dda_info *dda_info_array,
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Maps ISO LA to DDA */
	enum tegra_dda_id (*convert_la2dda_id_for_dyn_ptsa)(
		enum tegra_la_id la_id,
		unsigned int *error);

	/* Init max grant decrement */
	void (*init_max_gd)(
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Init mc/emc same freq threshold */
	void (*init_mcemc_same_freq_thr)(
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Sets up frequency ranges for grant decrement */
	void (*setup_freq_ranges)(
		struct mc_settings_info *mc_settings_ptr,
		unsigned int *error);

	/* Gets bytes per DRAM clock */
	int (*get_bytes_per_dram_clk)(
		enum tegra_dram_t dram_type,
		unsigned int *error);

	/* Converts bw to fraction of DRAM bw */
	struct fixed_point (*bw2fraction)(
		struct mc_settings_info *mc_settings_ptr,
		struct fixed_point bw_mbps,
		unsigned int *error);

	/* Converts fraction of DRAM bw to bw */
	unsigned int (*fraction2dda)(
		struct fixed_point fraction,
		struct fixed_point div,
		unsigned int mask,
		int round_up_or_to_nearest,
		unsigned int *error);

	/* Update DDA rate based on use case */
	void (*update_new_dda_rate_frac_use_case)(
		struct dda_info *dda_info_array,
		struct mc_settings_info *mc_settings_ptr,
		int clientid,
		struct fixed_point bw_mbps,
		unsigned int *error);

	/* Init non freq dep kernel init registers */
	void (*all_reg_info_array_init)(
		struct reg_info *mc_inf_arr,
		struct reg_info *mssnvl1_inf_arr,
		struct reg_info *mssnvl2_inf_arr,
		struct reg_info *mssnvl3_inf_arr,
		struct reg_info *mssnvl4_inf_arr);

	/* Write non freq dep kernel init registers */
	void (*write_perf_regs_kern_init)(
		struct mc_settings_info *mc_settings_ptr,
		struct reg_info *mc_inf_arr,
		struct reg_info *mssnvl1_inf_arr,
		struct reg_info *mssnvl2_inf_arr,
		struct reg_info *mssnvl3_inf_arr,
		struct reg_info *mssnvl4_inf_arr);

	/* Init non freq dep kernel init registers */
	void (*mcpcie_reg_info_array_init)(
		struct reg_info *inf_arr);

	void (*update_ord_ids)(
		struct reg_info *mcpcie_inf_arr,
		struct mc_settings_info *mc_settings_ptr,
		unsigned int pcie_xbar_cfg,
		unsigned int *error);
};

void init_la_ptsa_core(struct la_ptsa_core *lp);

void tegra_la_get_t19x_specific(struct la_chip_specific *cs_la);

#endif /* _MACH_TEGRA_19X_LA_PTSA_H */
