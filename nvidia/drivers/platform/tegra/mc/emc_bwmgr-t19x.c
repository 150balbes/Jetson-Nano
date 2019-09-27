/**
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/platform/tegra/bwmgr_mc.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/io.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/chip-id.h>

/* T194 dram freq table */
static u32 bwmgr_t194_dram_freq_table[] = { /* MHz */
	  68,   102,   204,   408,   600,
	 792,   924,  1056,  1200,  1600,
	1866,  2133
};

/* ALL VALUES BELOW CORRESPOND TO FREQ in bwmgr_t194_dram_freq_table */

/* efficiency percentage 8ch ecc 4X 1-rank*/
static u32 bwmgr_t194_8ch_ecc_4X_1rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 4ch ecc 4X 1-rank*/
static u32 bwmgr_t194_4ch_ecc_4X_1rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 16ch ecc 4X 1-rank*/
static u32 bwmgr_t194_16ch_ecc_4X_1rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 8ch 4X 1-rank*/
static u32 bwmgr_t194_8ch_4X_1rank_eff[] = { /* % */
	  30,    30,    30,    30,    30,
	  30,    40,    40,    40,    70,
	  70,    70
};

/* efficiency percentage 4ch 4X 1-rank*/
static u32 bwmgr_t194_4ch_4X_1rank_eff[] = { /* % */
	  30,    30,    30,    30,    30,
	  30,    40,    40,    40,    70,
	  70,    70
};

/* efficiency percentage 16ch 4X 1-rank*/
static u32 bwmgr_t194_16ch_4X_1rank_eff[] = { /* % */
	  30,    30,    30,    30,    30,
	  30,    40,    40,    40,    70,
	  70,    70
};

/* efficiency percentage 8ch ecc 4X 2-rank*/
static u32 bwmgr_t194_8ch_ecc_4X_2rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 4ch ecc 4X 2-rank*/
static u32 bwmgr_t194_4ch_ecc_4X_2rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 16ch ecc 4X 2-rank*/
static u32 bwmgr_t194_16ch_ecc_4X_2rank_eff[] = { /* % */
	  20,    20,    20,    20,    20,
	  20,    30,    30,    30,    55,
	  55,    60
};

/* efficiency percentage 8ch 4X 2-rank*/
static u32 bwmgr_t194_8ch_4X_2rank_eff[] = { /* % */
	  25,    25,    25,    25,    25,
	  25,    40,    40,    40,    65,
	  65,    65
};

/* efficiency percentage 4ch 4X 2-rank*/
static u32 bwmgr_t194_4ch_4X_2rank_eff[] = { /* % */
	  25,    25,    25,    25,    25,
	  25,    40,    40,    40,    65,
	  65,    65
};

/* efficiency percentage 16ch 4X 2-rank*/
static u32 bwmgr_t194_16ch_4X_2rank_eff[] = { /* % */
	  30,    30,    30,    30,    30,
	  30,    35,    35,    35,    65,
	  65,    65
};

/* efficiency percentage 8ch ecc 1X 1-rank*/
static u32 bwmgr_t194_8ch_ecc_1X_1rank_eff[] = { /* % */
	  45,    45,    45,    45,    45,
	  45,    50,    50,    50,    65,
	  65,    70
};

/* efficiency percentage 4ch ecc 1X 1-rank*/
static u32 bwmgr_t194_4ch_ecc_1X_1rank_eff[] = { /* % */
	  45,    45,    45,    45,    45,
	  45,    50,    50,    50,    65,
	  65,    70
};

/* efficiency percentage 16ch ecc 1X 1-rank*/
static u32 bwmgr_t194_16ch_ecc_1X_1rank_eff[] = { /* % */
	  40,    40,    40,    40,    40,
	  40,    50,    50,    50,    65,
	  65,    70
};

/* efficiency percentage 8ch 1X 1-rank*/
static u32 bwmgr_t194_8ch_1X_1rank_eff[] = { /* % */
	  60,    60,    60,    60,    60,
	  60,    60,    60,    60,    80,
	  80,    80
};

/* efficiency percentage 4ch 1X 1-rank*/
static u32 bwmgr_t194_4ch_1X_1rank_eff[] = { /* % */
	  60,    60,    60,    60,    60,
	  60,    60,    60,    60,    80,
	  80,    80
};

/* efficiency percentage 16ch 1X 1-rank*/
static u32 bwmgr_t194_16ch_1X_1rank_eff[] = { /* % */
	  60,    60,    60,    60,    60,
	  60,    60,    60,    60,    80,
	  80,    80
};

/* efficiency percentage 8ch ecc 1X 2-rank*/
static u32 bwmgr_t194_8ch_ecc_1X_2rank_eff[] = { /* % */
	  40,    40,    40,    40,    40,
	  40,    45,    45,    45,    60,
	  55,    55
};

/* efficiency percentage 4ch ecc 1X 2-rank*/
static u32 bwmgr_t194_4ch_ecc_1X_2rank_eff[] = { /* % */
	  40,    40,    40,    40,    40,
	  40,    45,    45,    45,    60,
	  55,    55
};

/* efficiency percentage 16ch ecc 1X 2-rank*/
static u32 bwmgr_t194_16ch_ecc_1X_2rank_eff[] = { /* % */
	  40,    40,    40,    40,    40,
	  40,    50,    50,    50,    60,
	  55,    55
};

/* efficiency percentage 8ch 1X 2-rank*/
static u32 bwmgr_t194_8ch_1X_2rank_eff[] = { /* % */
	  55,    55,    55,    55,    55,
	  55,    55,    55,    55,    70,
	  70,    70
};

/* efficiency percentage 4ch 1X 2-rank*/
static u32 bwmgr_t194_4ch_1X_2rank_eff[] = { /* % */
	  55,    55,    55,    55,    55,
	  55,    55,    55,    55,    70,
	  70,    70
};

/* efficiency percentage 16ch 1X 2-rank*/
static u32 bwmgr_t194_16ch_1X_2rank_eff[] = { /* % */
	  55,    55,    55,    55,    55,
	  55,    55,    55,    55,    70,
	  70,    70
};

/* max nvdis bw req in MHz
 * The value below reflects the maximum display bandwidth supported
 * at the frequency corresponding to the same index in
 * dram freq table.
 */
static u32 bwmgr_t194_lpddr4_8ch_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    22,    66,   107,
	 148,   177,   205,   237,   437,
	 656,   684
};

static u32 bwmgr_t194_lpddr4_4ch_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    22,    66,   107,
	 148,   177,   205,   237,   437,
	 656,   684
};

static u32 bwmgr_t194_lpddr4_16ch_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    11,    33,    53,
	  74,    88,   102,   167,   347,
	 361,   375
};

static u32 bwmgr_t194_lpddr4_8ch_ecc_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    26,    80,   131,
	 169,   196,   222,   251,   397,
	 494,   591
};

static u32 bwmgr_t194_lpddr4_4ch_ecc_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    26,    80,   131,
	 169,   196,   222,   251,   397,
	 494,   591
};

static u32 bwmgr_t194_lpddr4_16ch_ecc_iso_max_nvdis_bw_reqd[] = { /* MHz */
	   0,     0,    14,    43,    70,
	  98,   116,   135,   156,   250,
	 312,   375
};

/* max vi bw req in MHz
 * The value below reflects the maximum vi bandwidth supported
 * at the frequency corresponding to the same index in
 * dram freq table.
 */
static u32 bwmgr_t194_lpddr4_8ch_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,    14,    42,    68,
	  95,   113,   131,   151,   279,
	 279,   279
};

static u32 bwmgr_t194_lpddr4_4ch_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,    14,    42,    68,
	  95,   113,   131,   151,   279,
	 279,   279
};

static u32 bwmgr_t194_lpddr4_16ch_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,     8,    24,    40,
	  55,    66,    76,    88,   139,
	 139,   139
};

static u32 bwmgr_t194_lpddr4_8ch_ecc_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,    11,    34,    56,
	  78,    93,   108,   124,   279,
	 279,   279
};

static u32 bwmgr_t194_lpddr4_4ch_ecc_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,    11,    34,    56,
	  78,    93,   108,   124,   279,
	 279,   279
};

static u32 bwmgr_t194_lpddr4_16ch_ecc_iso_max_vi_bw_reqd[] = { /* MHz */
	   0,     0,     6,    19,    31,
	  43,    52,    60,    69,   139,
	 139,   139
};

/* slope for calculating eff
 * The value below reflects the slope
 * at the frequency corresponding to the same index in
 * dram freq table.
 */
static int bwmgr_t194_lpddr4_8ch_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1
};

static int bwmgr_t194_lpddr4_4ch_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1
};

static int bwmgr_t194_lpddr4_16ch_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1
};

static int bwmgr_t194_lpddr4_8ch_ecc_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -2
};

static int bwmgr_t194_lpddr4_4ch_ecc_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -2
};

static int bwmgr_t194_lpddr4_16ch_ecc_iso_slope[] = {
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1,    -1,    -1,    -1,
	  -1,    -1
};

/* vi offset bw req in MHz
 * The value below reflects the vi offset
 * at the frequency corresponding to the same index in
 * dram freq table.
 */
static u32 bwmgr_t194_lpddr4_8ch_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,     0
};

static u32 bwmgr_t194_lpddr4_4ch_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,     0
};

static u32 bwmgr_t194_lpddr4_16ch_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,    37
};

static u32 bwmgr_t194_lpddr4_8ch_ecc_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,     0
};

static u32 bwmgr_t194_lpddr4_4ch_ecc_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,     0
};

static u32 bwmgr_t194_lpddr4_16ch_ecc_iso_vi_bw_reqd_offset[] = { /* MHz */
	   0,     0,     0,     0,     0,
	   0,     0,     0,     0,     0,
	   0,    37
};

static struct mrq_emc_dvfs_latency_response bwmgr_emc_dvfs;
static int dram_rank;
static int dram_freq_count;

#define MC_BASE 0x02c10000
#define EMC_BASE 0x02c60000

#define MC_EMEM_ADR_CFG_CHANNEL_ENABLE_0 0xdf8
#define MC_EMEM_ADR_CFG_0 0x54
#define MC_ECC_CONTROL_0 0x1880
#define EMC_FBIO_CFG5_0 0x104

#define CH_MASK 0xFFFF /* Change bit counting if this mask changes */
#define CH4 0xf
#define CH2 0x3

#define ECC_MASK 0x1 /* 1 = enabled, 0 = disabled */
#define RANK_MASK 0x1 /* 1 = 2-RANK, 0 = 1-RANK */

#define DRAM_MASK 0x3
#define DRAM_DDR3 0
#define DRAM_LPDDR4 1
#define DRAM_LPDDR3 2 /* On T186 this value is LPDDR3 */
#define DRAM_DDR2 3

#define DRAM_REFRESH_1X 0
#define DRAM_REFRESH_4X 1

static unsigned long get_best_iso_freq(long total_iso_bw, long iso_bw_nvdis,
						long iso_bw_vi)
{
	long max_nvdis_bw_reqd_mhz = 0;
	long max_vi_bw_reqd_mhz = 0;
	long slope = -1;
	long vi_bw_reqd_offset_mhz = 0;
	int i;
	unsigned long dram_freq_mhz = 0;

	if (!bwmgr_dram_config_supported)
		return dram_freq_mhz;

	/*Input is in Hz, converting into MHz*/
	total_iso_bw /= 1000000;
	iso_bw_nvdis /= 1000000;
	iso_bw_vi /= 1000000;

	for (i = 0; i < dram_freq_count; i++) {
		max_nvdis_bw_reqd_mhz = bwmgr_max_nvdis_bw_reqd[i];
		max_vi_bw_reqd_mhz = bwmgr_max_vi_bw_reqd[i];
		slope = bwmgr_slope[i];
		vi_bw_reqd_offset_mhz = bwmgr_vi_bw_reqd_offset[i];

		if (!(iso_bw_nvdis <= max_nvdis_bw_reqd_mhz))
			continue;

		if (!(iso_bw_vi <= max_vi_bw_reqd_mhz))
			continue;

		if (!(iso_bw_vi <= (slope * (iso_bw_nvdis -
			max_nvdis_bw_reqd_mhz) + vi_bw_reqd_offset_mhz)))
			continue;

		if (!(total_iso_bw <= (slope *
				(iso_bw_nvdis - max_nvdis_bw_reqd_mhz)
				+ vi_bw_reqd_offset_mhz + iso_bw_nvdis)))
			continue;

		dram_freq_mhz = bwmgr_t194_dram_freq_table[i] * 1000000;
		break;
	}

	return dram_freq_mhz;
}

static unsigned long freq_to_bw(unsigned long freq)
{
	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH)
		return freq * 64;

	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH)
		return freq * 32;

	/*4CH and 4CH_ECC*/
	return freq * 16;
}

static unsigned long bw_to_freq(unsigned long bw)
{
	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH)
		return (bw + 64 - 1) / 64;

	if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH_ECC ||
			bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH)
		return (bw + 32 - 1) / 32;

	/*4CH and 4CH_ECC*/
	return (bw + 16 - 1) / 16;
}

static u32 dvfs_latency(u32 ufreq)
{
	u32 lt = 80000; /* default value of 80000 nsec */
	int i;

	if (bwmgr_emc_dvfs.num_pairs <= 0)
		return lt / 1000; /* convert nsec to usec, Bug 1697424 */

	for (i = 0; i < bwmgr_emc_dvfs.num_pairs; i++) {
		if (ufreq <= bwmgr_emc_dvfs.pairs[i].freq) {
			lt = bwmgr_emc_dvfs.pairs[i].latency;
			break;
		}
	}

	if (i >= bwmgr_emc_dvfs.num_pairs)
		lt =
		bwmgr_emc_dvfs.pairs[bwmgr_emc_dvfs.num_pairs - 1].latency;

	return lt / 1000; /* convert nsec to usec, Bug 1697424 */
}

static unsigned long t19x_bwmgr_apply_efficiency(
		unsigned long total_bw, unsigned long iso_bw,
		unsigned long max_rate, u64 usage_flags,
		unsigned long *iso_bw_min, unsigned long iso_bw_nvdis,
		unsigned long iso_bw_vi)
{
	int i;
	unsigned long total_bw_mhz, freq_after_eff_mhz;
	u8 efficiency;
	bool bw_req_satisfied = 1;

	if (total_bw) {
		total_bw_mhz = total_bw / 1000000;

		/* loop over all dram freq and its corresponding
		 * efficiency and check if total bw request can
		 * satisfied at that freq
		 */
		for (i = 0; i < dram_freq_count; i++) {
			efficiency = bwmgr_dram_noniso_eff_table[i];
			freq_after_eff_mhz = bwmgr_t194_dram_freq_table[i] *
						efficiency;
			freq_after_eff_mhz /= 100;
			if (total_bw_mhz < freq_after_eff_mhz) {
				total_bw = bwmgr_t194_dram_freq_table[i] *
						1000000;
				bw_req_satisfied = 0;
				break;
			}
		}
		/*defaults to max emc rate if reqd rate is not satisfied*/
		if (bw_req_satisfied)
			total_bw = max_rate;
	}

	// This functions loops over the available dram freq and returns the
	//lowest freq which can satisfy the *iso* bw requirement
	iso_bw = get_best_iso_freq(iso_bw, iso_bw_nvdis, iso_bw_vi);

	if (iso_bw_min)
		*iso_bw_min = iso_bw;

	return max(total_bw, iso_bw);
}

static void t19x_update_efficiency(unsigned long dram_refresh_rate)
{
	if ((dram_refresh_rate != DRAM_REFRESH_4X) &&
		(dram_refresh_rate != DRAM_REFRESH_1X)) {
		pr_err("bwmgr:Unknown cooling state. Set 4X refresh co-eff\n");
		dram_refresh_rate = DRAM_REFRESH_4X;
	}

	if (dram_refresh_rate == DRAM_REFRESH_4X) {
		if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_4X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_4X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_4X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_4X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_4X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_4X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_4X_1rank_eff;
		}
	} else if (dram_refresh_rate == DRAM_REFRESH_1X) {
		if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_1X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_1X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH_ECC) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_1X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_16CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_1X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_8CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_1X_1rank_eff;
		} else if (bwmgr_dram_type == DRAM_TYPE_LPDDR4_4CH) {
			if (dram_rank)
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_1X_2rank_eff;
			else
				bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_1X_1rank_eff;
		}
	}
}

static int get_dram_freq_table_idx(unsigned long emc_rate)
{
	int i = ARRAY_SIZE(bwmgr_t194_dram_freq_table) - 1;

	/* emc_rate is in Hz, dram_freq table's unit is MHz */
	emc_rate /= 1000000;

	while (i > 0 && bwmgr_t194_dram_freq_table[i] > emc_rate)
		i--;

	return i;
}

static u32 t19x_get_max_iso_bw(enum tegra_iso_client client)
{
	u8 idx = 0;
	unsigned long emc_max_rate = tegra_bwmgr_get_max_emc_rate();

	idx = get_dram_freq_table_idx(emc_max_rate);

	if ((client == TEGRA_ISO_CLIENT_DISP_0) ||
		(client == TEGRA_ISO_CLIENT_DISP_1) ||
		(client == TEGRA_ISO_CLIENT_DISP_2))
		return bwmgr_freq_to_bw(bwmgr_max_nvdis_bw_reqd[idx] * 1000);
	else if (client == TEGRA_ISO_CLIENT_TEGRA_CAMERA)
		return bwmgr_freq_to_bw(bwmgr_max_vi_bw_reqd[idx] * 1000);
	else
		return bwmgr_freq_to_bw((((bwmgr_slope[idx] *
		bwmgr_max_nvdis_bw_reqd[idx]) +
		bwmgr_vi_bw_reqd_offset[idx]) - 1) * 1000);
}

static struct bwmgr_ops bwmgr_ops_t19x = {
	.get_best_iso_freq = get_best_iso_freq,
	.freq_to_bw = freq_to_bw,
	.bw_to_freq = bw_to_freq,
	.dvfs_latency = dvfs_latency,
	.bwmgr_apply_efficiency = t19x_bwmgr_apply_efficiency,
	.update_efficiency = t19x_update_efficiency,
	.get_max_iso_bw = t19x_get_max_iso_bw,
};

struct bwmgr_ops *bwmgr_eff_init_t19x(void)
{
	int ch_num = 0;
	u32 dram, ch, ecc;
	void __iomem *mc_base, *emc_base;

	mc_base = ioremap(MC_BASE, 0x00010000);
	emc_base = ioremap(EMC_BASE, 0x00010000);

	dram = readl(emc_base + EMC_FBIO_CFG5_0) & DRAM_MASK;
	ch = readl(mc_base + MC_EMEM_ADR_CFG_CHANNEL_ENABLE_0) & CH_MASK;
	ecc = readl(mc_base + MC_ECC_CONTROL_0) & ECC_MASK;
	dram_rank = readl(mc_base + MC_EMEM_ADR_CFG_0) & RANK_MASK;

	iounmap(emc_base);
	iounmap(mc_base);

	while (ch) {
		if (ch & 1)
			ch_num++;
		ch >>= 1;
	}

	/* pre silicon use LPDDR4 16ch no ecc 1-rank config*/
	if (tegra_platform_is_sim() || tegra_platform_is_fpga()) {
		dram = DRAM_LPDDR4;
		ch_num = 16;
	}

	bwmgr_dram_num_channels = ch_num;

	/* T194 platforms should only have LPDDR4 */
	switch (dram) {
	case DRAM_LPDDR4:
		if (ecc) {
			if (ch_num == 16) {
				bwmgr_dram_type =
					DRAM_TYPE_LPDDR4_16CH_ECC;

				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_ecc_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
			bwmgr_t194_lpddr4_16ch_ecc_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_16ch_ecc_iso_max_vi_bw_reqd;

				bwmgr_slope =
					bwmgr_t194_lpddr4_16ch_ecc_iso_slope;

				bwmgr_vi_bw_reqd_offset =
			bwmgr_t194_lpddr4_16ch_ecc_iso_vi_bw_reqd_offset;

			} else if (ch_num == 8) {
				bwmgr_dram_type =
					DRAM_TYPE_LPDDR4_8CH_ECC;
				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_ecc_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
				bwmgr_t194_lpddr4_8ch_ecc_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_8ch_ecc_iso_max_vi_bw_reqd;

				bwmgr_slope =
				bwmgr_t194_lpddr4_8ch_ecc_iso_slope;

				bwmgr_vi_bw_reqd_offset =
				bwmgr_t194_lpddr4_8ch_ecc_iso_vi_bw_reqd_offset;

			} else if (ch_num == 4) {
				bwmgr_dram_type =
					DRAM_TYPE_LPDDR4_4CH_ECC;
				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_ecc_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
				bwmgr_t194_lpddr4_4ch_ecc_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_4ch_ecc_iso_max_vi_bw_reqd;

				bwmgr_slope =
				bwmgr_t194_lpddr4_4ch_ecc_iso_slope;

				bwmgr_vi_bw_reqd_offset =
				bwmgr_t194_lpddr4_4ch_ecc_iso_vi_bw_reqd_offset;

			}
		} else {
			if (ch_num == 16) {
				bwmgr_dram_type = DRAM_TYPE_LPDDR4_16CH;
				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_16ch_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
				bwmgr_t194_lpddr4_16ch_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_16ch_iso_max_vi_bw_reqd;

				bwmgr_slope = bwmgr_t194_lpddr4_16ch_iso_slope;

				bwmgr_vi_bw_reqd_offset =
				bwmgr_t194_lpddr4_16ch_iso_vi_bw_reqd_offset;

			} else if (ch_num == 8) {
				bwmgr_dram_type = DRAM_TYPE_LPDDR4_8CH;
				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_8ch_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
				bwmgr_t194_lpddr4_8ch_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_8ch_iso_max_vi_bw_reqd;

				bwmgr_slope = bwmgr_t194_lpddr4_8ch_iso_slope;

				bwmgr_vi_bw_reqd_offset =
				bwmgr_t194_lpddr4_8ch_iso_vi_bw_reqd_offset;

			} else if (ch_num == 4) {
				bwmgr_dram_type = DRAM_TYPE_LPDDR4_4CH;
				if (dram_rank)
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_4X_2rank_eff;
				else
					bwmgr_dram_noniso_eff_table =
					bwmgr_t194_4ch_4X_1rank_eff;

				bwmgr_max_nvdis_bw_reqd =
				bwmgr_t194_lpddr4_4ch_iso_max_nvdis_bw_reqd;

				bwmgr_max_vi_bw_reqd =
				bwmgr_t194_lpddr4_4ch_iso_max_vi_bw_reqd;

				bwmgr_slope = bwmgr_t194_lpddr4_4ch_iso_slope;

				bwmgr_vi_bw_reqd_offset =
				bwmgr_t194_lpddr4_4ch_iso_vi_bw_reqd_offset;

			}
		}

		if (ch_num < 4) {
			pr_err("bwmgr: Unknown memory channel configuration\n");
			pr_err("bwmgr: ddr config not supported\n");
			WARN_ON(true);
		} else {
			emc_to_dram_freq_factor = 2;
			/* valid ddr configuration */
			bwmgr_dram_config_supported = 1;
		}

		break;

	case DRAM_LPDDR3:
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		break;

	case DRAM_DDR3:
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		break;

	case DRAM_DDR2:
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
		break;

	default:
		pr_err("bwmgr: ddr config not supported\n");
		WARN_ON(true);
	}

	dram_freq_count = ARRAY_SIZE(bwmgr_t194_dram_freq_table);

	tegra_bpmp_send_receive(MRQ_EMC_DVFS_LATENCY, NULL, 0,
			&bwmgr_emc_dvfs, sizeof(bwmgr_emc_dvfs));

	/* isomgr uses this value to calculate max_iso_bw.
	 * We need this until the isomgr framework changes are checked in.
	 */
	bwmgr_iso_bw_percentage = 28;

	return &bwmgr_ops_t19x;
}
