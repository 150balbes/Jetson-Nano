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

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "tegra19x_la_ptsa.h"

#define __stringify_1(x...) #x
#define __stringify(x...) __stringify_1(x)
#define FIX_PT(x, y, err) fixed_point_init(x, y, 32, 32, err)
#define MASK(x) \
	((0xFFFFFFFFUL >> (31 - (1 ? x) + (0 ? x))) << (0 ? x))
#define SHIFT(x) \
	(0 ? x)
#define GEN_ID(id) \
	TEGRA_LA_##id
#define T19X_ID(id) \
	TEGRA_T19X_LA_##id##_ID

#define LA_T19X(a, r, i, ct, k)			\
	do { \
		gen_to_t19x_la_id[GEN_ID(i)] = T19X_ID(i); \
		t19x_to_gen_la_id[T19X_ID(i)] = GEN_ID(i); \
		t19x_la_kern_init[T19X_ID(i)] = k; \
		la_client_info_init( \
			&info_array[T19X_ID(i)], \
			mc_set, \
			MC_LATENCY_ALLOWANCE_ ## a ## _0, \
			MASK(r), \
			SHIFT(r), \
			GEN_ID(i), \
			__stringify(i), \
			TEGRA_LA_ ## ct ## _CLIENT, \
			error);   \
	} while (0)

#define GPU_LA_T19X(a, r, i, ct, k)		\
	do { \
		gen_to_t19x_la_id[GEN_ID(i)] = T19X_ID(i); \
		t19x_to_gen_la_id[T19X_ID(i)] = GEN_ID(i); \
		t19x_la_kern_init[T19X_ID(i)] = k; \
		la_client_info_init( \
			&info_array[T19X_ID(i)], \
			mc_set, \
			MC_ ## a ## _LATENCY_ALLOWANCE ## _0, \
			MASK(r), \
			SHIFT(r), \
			GEN_ID(i), \
			__stringify(i), \
			TEGRA_LA_ ## ct ## _CLIENT, \
			error);   \
	} while (0)

static struct fixed_point bw2fraction(
	struct mc_settings_info *mc_settings_ptr,
	struct fixed_point bw_mbps,
	unsigned int *error);

static unsigned int fraction2dda(
	struct fixed_point fraction,
	struct fixed_point div,
	unsigned int mask,
	int round_up_or_to_nearest,
	unsigned int *error);

static unsigned int calc_eff_rowsorter_sz(
	struct fixed_point emc_clk_mhz,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error)
{
	struct fixed_point term1;
	struct fixed_point term2;
	struct fixed_point term3;
	unsigned int eff_rs_size_bytes;

	term1 = FIX_PT(mc_settings_ptr->row_sorter_sz_bytes, 0, error);
	term2 = fixed_point_mult(
		fixed_point_div(
			fixed_point_mult(
				FIX_PT(2, 0, error),
				FIX_PT(mc_settings_ptr->dram_width_bits,
					0,
					error),
				error),
			FIX_PT(8, 0, error),
			error),
		fixed_point_add(
			emc_clk_mhz,
			FIX_PT(50, 0, error),
			error),
		error);
	term3 = fixed_point_mult(
		fixed_point_mult(
			fixed_point_sub(
				fixed_point_mult(
					mc_settings_ptr->max_drain_time_usec,
					emc_clk_mhz,
					error),
				FIX_PT(mc_settings_ptr->stat_lat_snaparb_rs,
					0,
					error),
				error),
			FIX_PT(2, 0, error),
			error),
		fixed_point_mult(
			fixed_point_div(
				FIX_PT(mc_settings_ptr->dram_width_bits,
					0,
					error),
				FIX_PT(8, 0, error),
				error),
			mc_settings_ptr->cons_mem_eff,
			error),
		error);
	eff_rs_size_bytes = (unsigned int)
		fixed_point_to_int(
			fixed_point_min(
				fixed_point_min(term1, term2, error),
				term3,
				error),
			error);

	return eff_rs_size_bytes;
}

static struct fixed_point calc_drain_time(
	struct fixed_point emc_clk_mhz,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error)
{
	unsigned int eff_rs_size_bytes;
	struct fixed_point term1;
	struct fixed_point drain_time_usec;

	eff_rs_size_bytes =
		calc_eff_rowsorter_sz(emc_clk_mhz, mc_settings_ptr, error);

	term1 = fixed_point_div(
		FIX_PT(mc_settings_ptr->dram_width_bits, 0, error),
		FIX_PT(4, 0, error),
		error);
	drain_time_usec =
		fixed_point_add(
			fixed_point_div(
				FIX_PT(eff_rs_size_bytes, 0, error),
				fixed_point_mult(
					fixed_point_mult(
						emc_clk_mhz,
						term1,
						error
					),
					mc_settings_ptr->cons_mem_eff,
					error
				),
			error
			),
			fixed_point_div(
				FIX_PT(mc_settings_ptr->stat_lat_snaparb_rs,
					0,
					error),
				emc_clk_mhz,
				error),
			error
		);

	return drain_time_usec;
}

static unsigned int get_init_la(
	enum la_client_type client_type,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error)
{
	unsigned int ret_la = 0;

	switch (client_type) {
	case TEGRA_LA_HUB_READ_CLIENT:
		ret_la = 33; /*(min((1000/1066.5) * 1059, 7650) / 30)*/
		break;
	case TEGRA_LA_HUB_WRITE_CLIENT:
		ret_la = 255;
		break;
	case TEGRA_LA_WCAM_WRITE_CLIENT:
		ret_la = 40;
		break;
	case TEGRA_LA_CPU_READ_CLIENT:
		ret_la = 4;
		break;
	case TEGRA_LA_CIFLL_WRITE_CLIENT:
		ret_la = 1023;
		break;
	case TEGRA_LA_DISPLAY_READ_CLIENT:
	{
		struct fixed_point term1;
		struct fixed_point term2;
		struct fixed_point max_drain_time_usec;

		max_drain_time_usec =
			calc_drain_time(
				FIX_PT(1066, 0x80000000, error) /* 1066.5 */,
				mc_settings_ptr,
				error);
		term1 = fixed_point_min(
			mc_settings_ptr->max_lat_all_usec,
			max_drain_time_usec,
			error);
		term2 = fixed_point_div(
			mc_settings_ptr->ns_per_tick,
			FIX_PT(1000, 0, error),
			error);
		ret_la = (unsigned int)
			fixed_point_ceil(
				fixed_point_div(
					term1,
					term2,
					error),
				error); /* 18 */
		break;
	}
	case TEGRA_LA_NVLRHP_READ_CLIENT:
		ret_la = 4;
		break;
	case TEGRA_LA_GPU_READ_CLIENT:
		ret_la = 31; /*(min((1000/1066.5) * 1019, 7650) / 30)*/
		break;
	case TEGRA_LA_NUM_CLIENT_TYPES:
		ret_la = 0;
		break;
	default:
		pr_err("%s: la_client_type %d not handled\n",
			__func__, client_type);
		(*error) |= 1;
		WARN_ON(1);
	}

	return ret_la;
}

static void la_client_info_init(
	struct la_client_info *entry,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int reg_addr,
	unsigned long mask,
	unsigned long shift,
	enum tegra_la_id id,
	const char *name,
	enum la_client_type client_type,
	unsigned int *error)
{
	entry->reg_addr = reg_addr;
	entry->mask = mask;
	entry->shift = shift;
	entry->id = id;
	if (name != NULL) {
		entry->name = (char *) kmalloc(strlen(name) + 1, GFP_KERNEL);
		strcpy(entry->name, name);
	}
	entry->client_type = client_type;
	entry->min_scaling_ratio = 0;
	entry->la_ref_clk_mhz = 0;
}

static void la_info_array_init(
	struct la_client_info *info_array,
	int *gen_to_t19x_la_id,
	int *t19x_to_gen_la_id,
	int *t19x_la_kern_init,
	struct mc_settings_info *mc_set,
	unsigned int *error)
{
	int i;

	for (i = 0; i < TEGRA_LA_MAX_ID; i++)
		gen_to_t19x_la_id[i] = TEGRA_T19X_LA_MAX_ID;

	for (i = 0; i < TEGRA_T19X_LA_MAX_ID; i++) {
		t19x_to_gen_la_id[i] = TEGRA_LA_MAX_ID;
		la_client_info_init(
			&info_array[i],
			mc_set,
			0,
			0,
			0,
			TEGRA_LA_MAX_ID,
			NULL,
			TEGRA_LA_NUM_CLIENT_TYPES,
			error);
	}

	LA_T19X(AONDMA_0, 10 : 0, AONDMAR, HUB_READ, 0);
	LA_T19X(AONDMA_0, 26 : 16, AONDMAW, HUB_WRITE, 0);
	LA_T19X(AON_0, 10 : 0, AONR, HUB_READ, 0);
	LA_T19X(AON_0, 26 : 16, AONW, HUB_WRITE, 0);
	LA_T19X(APEDMA_0, 10 : 0, APEDMAR, HUB_READ, 0);
	LA_T19X(APEDMA_0, 26 : 16, APEDMAW, HUB_WRITE, 0);
	LA_T19X(APE_0, 10 : 0, APER, HUB_READ, 0);
	LA_T19X(APE_0, 26 : 16, APEW, HUB_WRITE, 0);
	LA_T19X(AXIAP_0, 10 : 0, AXIAPR, HUB_READ, 0);
	LA_T19X(AXIAP_0, 26 : 16, AXIAPW, HUB_WRITE, 0);
	LA_T19X(AXIS_0, 10 : 0, AXISR, HUB_READ, 0);
	LA_T19X(AXIS_0, 26 : 16, AXISW, HUB_WRITE, 0);
	LA_T19X(BPMPDMA_0, 10 : 0, BPMPDMAR, HUB_READ, 0);
	LA_T19X(BPMPDMA_0, 26 : 16, BPMPDMAW, HUB_WRITE, 0);
	LA_T19X(BPMP_0, 10 : 0, BPMPR, HUB_READ, 0);
	LA_T19X(BPMP_0, 26 : 16, BPMPW, HUB_WRITE, 0);
	LA_T19X(CIFLL_WR_0, 10 : 0, CIFLL_WR, CIFLL_WRITE, 1);
	LA_T19X(DLA0_0, 26 : 16, DLA0FALRDB, HUB_READ, 0);
	LA_T19X(DLA0_0, 10 : 0, DLA0RDA, HUB_READ, 0);
	LA_T19X(DLA0_1, 26 : 16, DLA0FALWRB, HUB_WRITE, 0);
	LA_T19X(DLA0_1, 10 : 0, DLA0WRA, HUB_WRITE, 0);
	LA_T19X(DLA0_2, 10 : 0, DLA0RDA1, HUB_READ, 0);
	LA_T19X(DLA0_2, 26 : 16, DLA1RDA1, HUB_READ, 0);
	LA_T19X(DLA1_0, 26 : 16, DLA1FALRDB, HUB_READ, 0);
	LA_T19X(DLA1_0, 10 : 0, DLA1RDA, HUB_READ, 0);
	LA_T19X(DLA1_1, 26 : 16, DLA1FALWRB, HUB_WRITE, 0);
	LA_T19X(DLA1_1, 10 : 0, DLA1WRA, HUB_WRITE, 0);
	LA_T19X(EQOS_0, 10 : 0, EQOSR, HUB_READ, 0);
	LA_T19X(EQOS_0, 26 : 16, EQOSW, HUB_WRITE, 0);
	LA_T19X(ETR_0, 10 : 0, ETRR, HUB_READ, 0);
	LA_T19X(ETR_0, 26 : 16, ETRW, HUB_WRITE, 0);
	LA_T19X(HC_0, 10 : 0, HOST1XDMAR, HUB_READ, 0);
	LA_T19X(HDA_0, 10 : 0, HDAR, HUB_READ, 0);
	LA_T19X(HDA_0, 26 : 16, HDAW, HUB_WRITE, 0);
	LA_T19X(ISP2_0, 26 : 16, ISPFALR, HUB_READ, 0);
	LA_T19X(ISP2_0, 10 : 0, ISPRA, HUB_READ, 0);
	LA_T19X(ISP2_1, 10 : 0, ISPWA, HUB_WRITE, 0);
	LA_T19X(ISP2_1, 26 : 16, ISPWB, HUB_WRITE, 0);
	LA_T19X(ISP3_0, 10 : 0, ISPFALW, HUB_WRITE, 0);
	LA_T19X(ISP3_0, 26 : 16, ISPRA1, HUB_READ, 0);
	LA_T19X(MIU0_0, 10 : 0, MIU0R, HUB_READ, 0);
	LA_T19X(MIU0_0, 26 : 16, MIU0W, HUB_WRITE, 0);
	LA_T19X(MIU1_0, 10 : 0, MIU1R, HUB_READ, 0);
	LA_T19X(MIU1_0, 26 : 16, MIU1W, HUB_WRITE, 0);
	LA_T19X(MIU2_0, 10 : 0, MIU2R, HUB_READ, 0);
	LA_T19X(MIU2_0, 26 : 16, MIU2W, HUB_WRITE, 0);
	LA_T19X(MIU3_0, 10 : 0, MIU3R, HUB_READ, 0);
	LA_T19X(MIU3_0, 26 : 16, MIU3W, HUB_WRITE, 0);
	LA_T19X(MIU4_0, 10 : 0, MIU4R, HUB_READ, 0);
	LA_T19X(MIU4_0, 26 : 16, MIU4W, HUB_WRITE, 0);
	LA_T19X(MIU5_0, 10 : 0, MIU5R, HUB_READ, 0);
	LA_T19X(MIU5_0, 26 : 16, MIU5W, HUB_WRITE, 0);
	LA_T19X(MIU6_0, 10 : 0, MIU6R, HUB_READ, 0);
	LA_T19X(MIU6_0, 26 : 16, MIU6W, HUB_WRITE, 0);
	LA_T19X(MIU7_0, 10 : 0, MIU7R, HUB_READ, 0);
	LA_T19X(MIU7_0, 26 : 16, MIU7W, HUB_WRITE, 0);
	LA_T19X(MPCORE_0, 10 : 0, MPCORER, CPU_READ, 0);
	LA_T19X(MPCORE_0, 26 : 16, MPCOREW, HUB_WRITE, 0);
	LA_T19X(NVDEC_0, 10 : 0, NVDECSRD, HUB_READ, 0);
	LA_T19X(NVDEC_0, 26 : 16, NVDECSWR, HUB_WRITE, 0);
	LA_T19X(NVDEC_1, 26 : 16, NVDEC1SRD, HUB_READ, 0);
	LA_T19X(NVDEC_1, 10 : 0, NVDECSRD1, HUB_READ, 0);
	LA_T19X(NVDEC_2, 10 : 0, NVDEC1SRD1, HUB_READ, 0);
	LA_T19X(NVDEC_2, 26 : 16, NVDEC1SWR, HUB_WRITE, 0);
	LA_T19X(NVDISPLAY_0, 10 : 0, NVDISPLAYR, DISPLAY_READ, 1);
	LA_T19X(NVENC_0, 10 : 0, NVENCSRD, HUB_READ, 0);
	LA_T19X(NVENC_0, 26 : 16, NVENCSWR, HUB_WRITE, 0);
	LA_T19X(NVENC_1, 10 : 0, NVENC1SRD, HUB_READ, 0);
	LA_T19X(NVENC_1, 26 : 16, NVENC1SWR, HUB_WRITE, 0);
	LA_T19X(NVENC_2, 26 : 16, NVENC1SRD1, HUB_READ, 0);
	LA_T19X(NVENC_2, 10 : 0, NVENCSRD1, HUB_READ, 0);
	LA_T19X(NVJPG_0, 10 : 0, NVJPGSRD, HUB_READ, 0);
	LA_T19X(NVJPG_0, 26 : 16, NVJPGSWR, HUB_WRITE, 0);
	LA_T19X(PCIE0_0, 10 : 0, PCIE0R, HUB_READ, 0);
	LA_T19X(PCIE0_0, 26 : 16, PCIE0W, HUB_WRITE, 0);
	LA_T19X(PCIE1_0, 10 : 0, PCIE1R, HUB_READ, 0);
	LA_T19X(PCIE1_0, 26 : 16, PCIE1W, HUB_WRITE, 0);
	LA_T19X(PCIE2_0, 10 : 0, PCIE2AR, HUB_READ, 0);
	LA_T19X(PCIE2_0, 26 : 16, PCIE2AW, HUB_WRITE, 0);
	LA_T19X(PCIE3_0, 10 : 0, PCIE3R, HUB_READ, 0);
	LA_T19X(PCIE3_0, 26 : 16, PCIE3W, HUB_WRITE, 0);
	LA_T19X(PCIE4_0, 10 : 0, PCIE4R, HUB_READ, 0);
	LA_T19X(PCIE4_0, 26 : 16, PCIE4W, HUB_WRITE, 0);
	LA_T19X(PCIE5_0, 10 : 0, PCIE5R, HUB_READ, 0);
	LA_T19X(PCIE5_0, 26 : 16, PCIE5W, HUB_WRITE, 0);
	LA_T19X(PCIE5_1, 26 : 16, PCIE0R1, HUB_READ, 0);
	LA_T19X(PCIE5_1, 10 : 0, PCIE5R1, HUB_READ, 0);
	LA_T19X(PVA0_0, 10 : 0, PVA0RDA, HUB_READ, 0);
	LA_T19X(PVA0_0, 26 : 16, PVA0RDB, HUB_READ, 0);
	LA_T19X(PVA0_1, 10 : 0, PVA0RDC, HUB_READ, 0);
	LA_T19X(PVA0_1, 26 : 16, PVA0WRA, HUB_WRITE, 0);
	LA_T19X(PVA0_2, 10 : 0, PVA0WRB, HUB_WRITE, 0);
	LA_T19X(PVA0_2, 26 : 16, PVA0WRC, HUB_WRITE, 0);
	LA_T19X(PVA0_3, 10 : 0, PVA0RDA1, HUB_READ, 0);
	LA_T19X(PVA0_3, 26 : 16, PVA0RDB1, HUB_READ, 0);
	LA_T19X(PVA1_0, 10 : 0, PVA1RDA, HUB_READ, 0);
	LA_T19X(PVA1_0, 26 : 16, PVA1RDB, HUB_READ, 0);
	LA_T19X(PVA1_1, 10 : 0, PVA1RDC, HUB_READ, 0);
	LA_T19X(PVA1_1, 26 : 16, PVA1WRA, HUB_WRITE, 0);
	LA_T19X(PVA1_2, 10 : 0, PVA1WRB, HUB_WRITE, 0);
	LA_T19X(PVA1_2, 26 : 16, PVA1WRC, HUB_WRITE, 0);
	LA_T19X(PVA1_3, 10 : 0, PVA1RDA1, HUB_READ, 0);
	LA_T19X(PVA1_3, 26 : 16, PVA1RDB1, HUB_READ, 0);
	LA_T19X(RCEDMA_0, 10 : 0, RCEDMAR, HUB_READ, 0);
	LA_T19X(RCEDMA_0, 26 : 16, RCEDMAW, HUB_WRITE, 0);
	LA_T19X(RCE_0, 10 : 0, RCER, HUB_READ, 0);
	LA_T19X(RCE_0, 26 : 16, RCEW, HUB_WRITE, 0);
	LA_T19X(SATA_0, 10 : 0, SATAR, HUB_READ, 0);
	LA_T19X(SATA_0, 26 : 16, SATAW, HUB_WRITE, 0);
	LA_T19X(SCEDMA_0, 10 : 0, SCEDMAR, HUB_READ, 0);
	LA_T19X(SCEDMA_0, 26 : 16, SCEDMAW, HUB_WRITE, 0);
	LA_T19X(SCE_0, 10 : 0, SCER, HUB_READ, 0);
	LA_T19X(SCE_0, 26 : 16, SCEW, HUB_WRITE, 0);
	LA_T19X(SDMMCAB_0, 10 : 0, SDMMCRAB, HUB_READ, 0);
	LA_T19X(SDMMCAB_0, 26 : 16, SDMMCWAB, HUB_WRITE, 0);
	LA_T19X(SDMMCA_0, 10 : 0, SDMMCRA, HUB_READ, 0);
	LA_T19X(SDMMCA_0, 26 : 16, SDMMCWA, HUB_WRITE, 0);
	LA_T19X(SDMMC_0, 10 : 0, SDMMCR, HUB_READ, 0);
	LA_T19X(SDMMC_0, 26 : 16, SDMMCW, HUB_WRITE, 0);
	LA_T19X(SE_0, 10 : 0, SESRD, HUB_READ, 0);
	LA_T19X(SE_0, 26 : 16, SESWR, HUB_WRITE, 0);
	LA_T19X(TSECB_0, 10 : 0, TSECSRDB, HUB_READ, 0);
	LA_T19X(TSECB_0, 26 : 16, TSECSWRB, HUB_WRITE, 0);
	LA_T19X(TSEC_0, 10 : 0, TSECSRD, HUB_READ, 0);
	LA_T19X(TSEC_0, 26 : 16, TSECSWR, HUB_WRITE, 0);
	LA_T19X(UFSHC_0, 10 : 0, UFSHCR, HUB_READ, 0);
	LA_T19X(UFSHC_0, 26 : 16, UFSHCW, HUB_WRITE, 0);
	LA_T19X(VI2_0, 10 : 0, VIW, HUB_WRITE, 0);
	LA_T19X(VIC_0, 10 : 0, VICSRD, HUB_READ, 0);
	LA_T19X(VIC_0, 26 : 16, VICSWR, HUB_WRITE, 0);
	LA_T19X(VIC_1, 10 : 0, VICSRD1, HUB_READ, 0);
	LA_T19X(VIFAL_0, 10 : 0, VIFALR, HUB_READ, 0);
	LA_T19X(VIFAL_0, 26 : 16, VIFALW, HUB_WRITE, 0);
	LA_T19X(WCAM, 10 : 0, WCAM, WCAM_WRITE, 0);
	LA_T19X(XUSB_0, 10 : 0, XUSB_HOSTR, HUB_READ, 0);
	LA_T19X(XUSB_0, 26 : 16, XUSB_HOSTW, HUB_WRITE, 0);
	LA_T19X(XUSB_1, 10 : 0, XUSB_DEVR, HUB_READ, 0);
	LA_T19X(XUSB_1, 26 : 16, XUSB_DEVW, HUB_WRITE, 0);
	GPU_LA_T19X(CIFLL_NVLRHP, 10 : 0, NVLRHP, NVLRHP_READ, 1);
	GPU_LA_T19X(MSSNVLINK_DGPU, 10 : 0, DGPU, GPU_READ, 0);
	GPU_LA_T19X(MSSNVLINK_IGPU, 10 : 0, IGPU, GPU_READ, 0);
}

static void init_max_gd(
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error
)
{
	struct fixed_point max_gd = FIX_PT(0, 0, error);

	if (mc_settings_ptr->dram_to_emc_freq_ratio == 2) {
		/* 1.5 - pow(2.0, -1.0 * PTSA_reg_length_bits) */
		switch (mc_settings_ptr->ptsa_reg_length_bits) {
		case 8:
			max_gd = fixed_point_sub(
				FIX_PT(1, 0x80000000, error),
				FIX_PT(0, 0x01000000, error),
				error); /* 1.49609375 */
			break;
		case 12:
			max_gd = fixed_point_sub(
				FIX_PT(1, 0x80000000, error),
				FIX_PT(0, 0x00100000, error),
				error); /* 1.499755859*/
			break;
		default:
			pr_err("%s: ptsa_reg_length_bits %d not handled\n",
				__func__,
				mc_settings_ptr->ptsa_reg_length_bits);
			(*error) |= 1;
			WARN_ON(1);
		}
	} else {
		/* 2 - pow(2.0, -1.0 * PTSA_reg_length_bits) */
		switch (mc_settings_ptr->ptsa_reg_length_bits) {
		case 8:
			max_gd = fixed_point_sub(
				FIX_PT(2, 0, error),
				FIX_PT(0, 0x01000000, error),
				error); /* 1.99609375 */
			break;
		case 12:
			max_gd = fixed_point_sub(
				FIX_PT(2, 0, error),
				FIX_PT(0, 0x00100000, error),
				error); /* 1.999755859 */
			break;
		default:
			pr_err("%s: ptsa_reg_length_bits %d not handled\n",
				__func__,
				mc_settings_ptr->ptsa_reg_length_bits);
			(*error) |= 1;
			WARN_ON(1);
		}
	}

	mc_settings_ptr->max_gd =
		fixed_point_mult(
			mc_settings_ptr->grant_dec_multiplier,
			max_gd,
			error);
}

static void init_mcemc_same_freq_thr(
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error
)
{
	struct fixed_point lowest_emc_freq;
	lowest_emc_freq = fixed_point_div(
		mc_settings_ptr->lowest_dram_freq,
		FIX_PT(mc_settings_ptr->dram_to_emc_freq_ratio, 0, error),
		error);

	/* Want 2:1 all throughout, so set mc_emc_same_freq_thr to something */
	/* below lowest_emc_freq, but don't make it zero. */
	mc_settings_ptr->mc_emc_same_freq_thr =
		fixed_point_max(
			fixed_point_sub(
				lowest_emc_freq,
				FIX_PT(1, 0, error),
				error
				),
			FIX_PT(0, 0x1999999A, error), /* 0.1 */
			error
			);
}

static void mc_settings_init(
	enum tegra_dram_t dram_type,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error)
{
	switch (dram_type) {
	case TEGRA_LP4_4CH:
		mc_settings_ptr->num_channels = 4;
		mc_settings_ptr->bytes_per_dram_clk = 16;
		mc_settings_ptr->hub_dda_div = FIX_PT(1, 0, error);
		mc_settings_ptr->ring0_dda_div = FIX_PT(4, 0, error);
		mc_settings_ptr->dram_to_emc_freq_ratio = 2;
		mc_settings_ptr->highest_dram_freq = FIX_PT(2132, 0, error);
		mc_settings_ptr->lowest_dram_freq = FIX_PT(25, 0, error);
		break;
	case TEGRA_LP4_8CH:
	case TEGRA_LP4X_8CH:
		mc_settings_ptr->num_channels = 8;
		mc_settings_ptr->bytes_per_dram_clk = 32;
		mc_settings_ptr->hub_dda_div = FIX_PT(1, 0, error);
		mc_settings_ptr->ring0_dda_div = FIX_PT(4, 0, error);
		mc_settings_ptr->dram_to_emc_freq_ratio = 2;
		mc_settings_ptr->highest_dram_freq = FIX_PT(2132, 0, error);
		mc_settings_ptr->lowest_dram_freq = FIX_PT(25, 0, error);
		break;
	case TEGRA_LP4_16CH:
	case TEGRA_LP4X_16CH:
		mc_settings_ptr->num_channels = 16;
		mc_settings_ptr->bytes_per_dram_clk = 64;
		mc_settings_ptr->hub_dda_div = FIX_PT(1, 0, error);
		mc_settings_ptr->ring0_dda_div = FIX_PT(4, 0, error);
		mc_settings_ptr->dram_to_emc_freq_ratio = 2;
		mc_settings_ptr->highest_dram_freq = FIX_PT(2132, 0, error);
		mc_settings_ptr->lowest_dram_freq = FIX_PT(25, 0, error);
		break;
	default:
		pr_err("%s: tegra_dram_t %d not handled\n",
			__func__, dram_type);
		(*error) |= 1;
		WARN_ON(1);
	}

	mc_settings_ptr->dram_type = dram_type;
	mc_settings_ptr->mccif_buf_sz_bytes = 64 * 484;
	mc_settings_ptr->stat_lat_minus_snaparb2rs = 230;
	mc_settings_ptr->exp_time = 206;
	mc_settings_ptr->dram_width_bits = EMC_FBIO_DATA_WIDTH *
		mc_settings_ptr->num_channels;
	mc_settings_ptr->cons_mem_eff = FIX_PT(0, 0x80000000, error); /* 0.5 */
	mc_settings_ptr->stat_lat_snaparb_rs = 54;
	mc_settings_ptr->row_sorter_sz_bytes =
		mc_settings_ptr->num_channels *
		64 * (NV_MC_EMEM_NUM_SLOTS + 1);
	mc_settings_ptr->max_drain_time_usec = FIX_PT(10, 0, error);
	mc_settings_ptr->ns_per_tick = FIX_PT(30, 0, error);
	mc_settings_ptr->max_lat_all_usec =
		FIX_PT(7, 0xA6666666, error); /* 7.65 */
	mc_settings_ptr->ring2_dda_rate = 1;
	mc_settings_ptr->ring2_dda_en = 1;
	mc_settings_ptr->siso_hp_en = 1;
	mc_settings_ptr->vi_always_hp = 1;
	mc_settings_ptr->disp_catchup_factor =
		FIX_PT(1, 0x1999999A, error); /* 1.1 */
	mc_settings_ptr->dda_bw_margin = FIX_PT(1, 0x33333333, error); /* 1.2 */
	mc_settings_ptr->two_stge_ecc_iso_dda_bw_margin =
		FIX_PT(1, 0x66666666, error); /* 1.4 */
	mc_settings_ptr->ptsa_reg_length_bits = NV_MC_EMEM_PTSA_RATE_WIDTH;
	mc_settings_ptr->grant_dec_multiplier = FIX_PT(1, 0, error);
	mc_settings_ptr->set_perf_regs = 1;
	mc_settings_ptr->hub2mcf_dda = 2; /* AUTO */
	mc_settings_ptr->igpu_mcf_dda = 2; /* AUTO */
	mc_settings_ptr->tsa_arb_fix = 1;
	mc_settings_ptr->iso_holdoff_override = 1;
	mc_settings_ptr->pcfifo_interlock = 1;
	mc_settings_ptr->en_ordering = 1;
	mc_settings_ptr->set_order_id = 1;
	mc_settings_ptr->hp_cpu_throttle_en = 0;
	mc_settings_ptr->override_isoptc_hub_mapping = 1;
	mc_settings_ptr->override_hub_vcarb_type = 1;
	mc_settings_ptr->override_hub_vcarb_wt = 1;
	mc_settings_ptr->override_iso_tbu_cchk_en_ctrl = 1;
	mc_settings_ptr->hub2mcf_dda_rate = 1638; /* 80% */
	mc_settings_ptr->hub2mcf_dda_max = 32;
	mc_settings_ptr->mssnvlink_mcf_igpu_dda_rate = 1740; /* 85% */
	mc_settings_ptr->mssnvlink_mcf_igpu_dda_max = 32;
	mc_settings_ptr->isoptc_hub_num = 0;
	mc_settings_ptr->hub_vcarb_type = 3;
	mc_settings_ptr->hub_vcarb_niso_wt = 1;
	mc_settings_ptr->hub_vcarb_siso_wt = 4;
	mc_settings_ptr->hub_vcarb_iso_wt = 31;
	mc_settings_ptr->iso_tbu_cchk_en_ctrl = 1; /* disable hp iso tbu chk */
	mc_settings_ptr->freq_range.lo_freq = FIX_PT(0, 0, error);
	mc_settings_ptr->freq_range.hi_freq = FIX_PT(0, 0, error);
	mc_settings_ptr->freq_range.lo_gd = FIX_PT(0, 0, error);
	mc_settings_ptr->freq_range.hi_gd = FIX_PT(0, 0, error);
	mc_settings_ptr->freq_range.emc_mc_ratio = 0;
	mc_settings_ptr->freq_range.valid = 0;
	init_max_gd(mc_settings_ptr, error);
	init_mcemc_same_freq_thr(mc_settings_ptr, error);
}

static void mc_settings_override(
	struct mc_settings_info info,
	struct mc_settings_info *mc_settings_ptr)
{
	(*mc_settings_ptr) = info;
}

static void get_disp_rd_lat_allow_given_disp_bw(
	struct mc_settings_info *mc_settings_ptr,
	struct fixed_point emc_freq_mhz,
	struct fixed_point dis_bw, /* MBps */
	int *disp_la,
	struct fixed_point *drain_time_usec,
	struct fixed_point *la_bw_up_bnd_usec,
	unsigned int *error)
{
	struct fixed_point mccif_buf_sz_bytes;
	struct fixed_point lat_allow_usec;
	struct fixed_point lat_allow_ticks;

	struct fixed_point term1;

	mccif_buf_sz_bytes =
		FIX_PT(mc_settings_ptr->mccif_buf_sz_bytes, 0, error);
	term1 = fixed_point_add(
		FIX_PT(mc_settings_ptr->stat_lat_minus_snaparb2rs, 0, error),
		FIX_PT(mc_settings_ptr->exp_time, 0, error),
		error);
	(*la_bw_up_bnd_usec) =
		fixed_point_sub(
			fixed_point_div(
				mccif_buf_sz_bytes,
				dis_bw,
				error
			),
			fixed_point_div(
				term1,
				emc_freq_mhz,
				error
			),
			error
		);
	lat_allow_usec = fixed_point_min((*la_bw_up_bnd_usec),
		mc_settings_ptr->max_lat_all_usec,
		error);

	lat_allow_ticks =
		fixed_point_div(lat_allow_usec,
			fixed_point_div(
				mc_settings_ptr->ns_per_tick,
				FIX_PT(1000, 0, error),
				error),
			error
		);

	if (fixed_point_gt(lat_allow_ticks, FIX_PT(255, 0, error), error))
		lat_allow_ticks = FIX_PT(255, 0, error);

	(*disp_la) = fixed_point_ceil(lat_allow_ticks, error);
	(*drain_time_usec) =
		calc_drain_time(emc_freq_mhz, mc_settings_ptr, error);
}

static void dda_info_init(
	struct dda_info *entry,
	const char *name,
	int ring,
	enum tegra_iso_t iso_type,
	unsigned int rate_reg_addr,
	unsigned long mask,
	struct fixed_point dda_div,
	unsigned int *error
)
{
	strcpy(entry->name, name);
	entry->iso_type = iso_type;
	entry->ring = ring;
	entry->rate_reg_addr = rate_reg_addr;
	entry->mask = mask;
	entry->dda_div = dda_div;

	entry->min = -1;
	entry->max = -1;
	entry->rate = 0;
	entry->frac = FIX_PT(0, 0, error);
	entry->frac_valid = 0;
	entry->bw = FIX_PT(0, 0, error);
}

#define INIT_DDA(info_array, NAME, RING, ISO_TYPE, DDA_DIV) \
	dda_info_init( \
		&info_array[TEGRA_DDA_##NAME##_ID], \
		__stringify(TEGRA_DDA_##NAME##_ID), \
		RING, \
		ISO_TYPE, \
		MC_##NAME##_PTSA_RATE_0, \
		MC_##NAME##_PTSA_RATE_0_PTSA_RATE_##NAME##_DEFAULT_MASK, \
		DDA_DIV, error)


static void dda_info_array_init(
	struct dda_info *inf_arr,
	int info_array_size,
	struct mc_settings_info *mc_set,
	unsigned int *error
)
{
	int i;

	for (i = 0; i < info_array_size; i++) {
		dda_info_init(
			&inf_arr[i],
			"",
			-1,
			TEGRA_NISO,
			0,
			0xffff,
			FIX_PT(0, 0, error),
			error);
	}

	INIT_DDA(inf_arr, AONPC,        1, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, APB,          2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, AUD,          1, TEGRA_HISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, BPMPPC,       1, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, CIFLL_ISO,    0, TEGRA_HISO, mc_set->ring0_dda_div);
	INIT_DDA(inf_arr, CIFLL_SISO,   0, TEGRA_SISO, mc_set->ring0_dda_div);
	INIT_DDA(inf_arr, CIFLL_NISO,   0, TEGRA_NISO, mc_set->ring0_dda_div);
	INIT_DDA(inf_arr, CIFLL_RING0X, 0, TEGRA_NISO, mc_set->ring0_dda_div);
	INIT_DDA(inf_arr, DIS,          1, TEGRA_HISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA0FALPC,    2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA0XA,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA0XA2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA0XA3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA1FALPC,    2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA1XA,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA1XA2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, DLA1XA3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, EQOSPC,       1, TEGRA_HISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, HDAPC,        1, TEGRA_HISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, HOST,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, ISP,          2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, ISP2PC,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, ISPPC,        2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, JPG,          2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU0,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU1,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU2,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU3,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU4,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU5,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU6,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MIU7,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MLL_MPCORER,  0, TEGRA_NISO, mc_set->ring0_dda_div);
	INIT_DDA(inf_arr, MSE,          2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MSE2,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MSE3,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MSEA,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MSEB,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, MSEB1,        2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NIC,          2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD,          2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD2,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD3,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD4,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD5,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, NVD6,         2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE0X,       2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE0X2,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE0XA,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE1X,       2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE1XA,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE4X,       2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE4XA,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE5X,       2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE5X2,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PCIE5XA,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XA,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XA2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XA3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XB,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XB2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XB3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA0XC,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XA,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XA2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XA3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XB,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XB2,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XB3,      2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, PVA1XC,       2, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, RCEPC,        1, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, RING2,        1, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, SAX,          2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, SCEPC,        1, TEGRA_SISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, SD,           2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, SDM,          2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, SMMU_SMMU,    0, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, UFSHCPC,      2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, UFSHCPC2,     2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, USBD,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, USBD2,        2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, USBX,         2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, USBX2,        2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, VE,           1, TEGRA_HISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, VICPC,        2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, VICPC2,       2, TEGRA_NISO, mc_set->hub_dda_div);
	INIT_DDA(inf_arr, VICPC3,       2, TEGRA_NISO, mc_set->hub_dda_div);
}

#undef INIT_DDA

static void update_new_dda_minmax_kern_init(
	struct dda_info *dda_info_array,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error
) {
	int clientid;

	for (clientid = 0; clientid < TEGRA_DDA_MAX_ID; clientid++) {
		if (!mc_settings_ptr->ring2_dda_en &&
			dda_info_array[clientid].ring == 2) {
			dda_info_array[clientid].min =
				dda_info_array[clientid].max = -1;
		} else if (((mc_settings_ptr->vi_always_hp) &&
					(clientid == TEGRA_DDA_VE_ID)) ||
				 (clientid == TEGRA_DDA_CIFLL_SISO_ID) ||
				 (clientid == TEGRA_DDA_SMMU_SMMU_ID)
			) { /* VI always high priority since self limiting */
			dda_info_array[clientid].min = 1;
			dda_info_array[clientid].max = 1;
		} else if (dda_info_array[clientid].iso_type == TEGRA_HISO ||
			(dda_info_array[clientid].iso_type == TEGRA_SISO &&
				!mc_settings_ptr->siso_hp_en &&
				dda_info_array[clientid].ring == 2) ||
			(clientid == TEGRA_DDA_CIFLL_ISO_ID)
			){
			int max_max = (1 << NV_MC_EMEM_PTSA_MINMAX_WIDTH) - 1;
			dda_info_array[clientid].min = -5;
			dda_info_array[clientid].max = max_max;
		} else if (dda_info_array[clientid].iso_type == TEGRA_SISO &&
			mc_settings_ptr->siso_hp_en &&
			dda_info_array[clientid].ring == 2) {
			dda_info_array[clientid].min =
				dda_info_array[clientid].max = 1;
		} else if ((
			(dda_info_array[clientid].iso_type == TEGRA_NISO) ||
			(dda_info_array[clientid].iso_type == TEGRA_SISO &&
			dda_info_array[clientid].ring == 1) ||
			(clientid == TEGRA_DDA_RING2_ID) ||
			(clientid == TEGRA_DDA_CIFLL_NISO_ID) ||
			(clientid == TEGRA_DDA_CIFLL_RING0X_ID)
			)
			&& (clientid != TEGRA_DDA_MLL_MPCORER_ID)) {
			dda_info_array[clientid].min = -2;
			dda_info_array[clientid].max = 0;
		} else if (clientid != TEGRA_DDA_MLL_MPCORER_ID) {
			pr_err("%s: ", __func__);
			pr_err("clientid != TEGRA_DDA_MLL_MPCORER_ID\n");
			(*error) |= 1;
			WARN_ON(1);
		}
	}
}

static void update_new_dda_rate_frac_kern_init(
	struct dda_info *dda_info_array,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error
)
{
	int clientid;

	for (clientid = 0; clientid < TEGRA_DDA_MAX_ID; clientid++) {
		enum tegra_iso_t iso_type = dda_info_array[clientid].iso_type;
		unsigned int ring = dda_info_array[clientid].ring;
		int iso_or_ring2_siso;
		int ring2_hp_siso;

		iso_or_ring2_siso = ((iso_type == TEGRA_HISO) ||
			((iso_type == TEGRA_SISO) &&
			!mc_settings_ptr->siso_hp_en &&
			(ring == 2))) ? 1 : 0;
		ring2_hp_siso = ((iso_type == TEGRA_SISO) &&
			mc_settings_ptr->siso_hp_en &&
			(ring == 2)) ? 1 : 0;

		if (!mc_settings_ptr->ring2_dda_en && (ring == 2)) {
			dda_info_array[clientid].rate = 0;
			dda_info_array[clientid].frac_valid = 0;
		} else if (!iso_or_ring2_siso) {
			if (ring2_hp_siso) {
				/*ring2 SISO client and always hp SISO */
				dda_info_array[clientid].rate = 0;
				dda_info_array[clientid].frac_valid = 0;
			} else if (clientid != TEGRA_DDA_MLL_MPCORER_ID) {
				if (clientid != TEGRA_DDA_SMMU_SMMU_ID) {
					/* all other DDAs */
					dda_info_array[clientid].rate = 1;
					dda_info_array[clientid].frac_valid = 0;
				}
			}
		} else if (clientid == TEGRA_DDA_EQOSPC_ID) {
			struct fixed_point iso_adj_bw;

			iso_adj_bw = fixed_point_mult(
				FIX_PT(250, 0, error),
				mc_settings_ptr->two_stge_ecc_iso_dda_bw_margin,
				error);
			dda_info_array[clientid].frac =
				bw2fraction(mc_settings_ptr, iso_adj_bw, error);
			dda_info_array[clientid].frac_valid = 1;
			dda_info_array[clientid].rate =
			fraction2dda(
				dda_info_array[clientid].frac,
				dda_info_array[clientid].dda_div,
				dda_info_array[clientid].mask,
				(dda_info_array[clientid].iso_type !=
					TEGRA_NISO),
				error);
		}
	}

	/* Set the DDA value of ring2 to epsilon*/
	dda_info_array[TEGRA_DDA_RING2_ID].rate =
		mc_settings_ptr->ring2_dda_rate;

	/* Ring1 DDA */
	dda_info_array[TEGRA_DDA_CIFLL_SISO_ID].rate = 0;
	dda_info_array[TEGRA_DDA_CIFLL_NISO_ID].rate = 1;
	dda_info_array[TEGRA_DDA_CIFLL_RING0X_ID].rate = 1;
}

static enum tegra_dda_id convert_la2dda_id_for_dyn_ptsa(
	enum tegra_la_id la_id,
	unsigned int *error)
{
	enum tegra_dda_id ret_dda_id = TEGRA_DDA_MAX_ID;

	switch (la_id) {
	case TEGRA_LA_APEDMAR:
	case TEGRA_LA_APEDMAW:
	case TEGRA_LA_APER:
	case TEGRA_LA_APEW:
		ret_dda_id = TEGRA_DDA_AUD_ID;
		break;
	case TEGRA_LA_EQOSR:
	case TEGRA_LA_EQOSW:
		ret_dda_id = TEGRA_DDA_EQOSPC_ID;
		break;
	case TEGRA_LA_HDAR:
	case TEGRA_LA_HDAW:
		ret_dda_id = TEGRA_DDA_HDAPC_ID;
		break;
	case TEGRA_LA_NVDISPLAYR:
		ret_dda_id = TEGRA_DDA_DIS_ID;
		break;
	case TEGRA_LA_VIW:
	case TEGRA_LA_VIFALR:
	case TEGRA_LA_VIFALW:
		ret_dda_id = TEGRA_DDA_VE_ID;
		break;
	default:
	{
		pr_err("%s: tegra_la_id %d not handled\n",
			__func__, la_id);
		(*error) |= 1;
		WARN_ON(1);
	}
	}

	return ret_dda_id;
}

/*
  setupFreqRanges()
  ====================================================================
  This function is used to initialize the frequency ranges based on which DDA
  programming is done.
  ====================================================================
*/
static void setup_freq_ranges(
	struct mc_settings_info *mc_settings_ptr,
	unsigned int *error
)
{
	struct fixed_point comparison_freq_thr_to_use;
	struct fixed_point lo_freq;

	/*if we are in LP4, then DRAM:EMC is 2:1 so to make the comparison */
	/* below we need to use double MCEMCsameFreqThr */
	comparison_freq_thr_to_use =
		fixed_point_mult(
			mc_settings_ptr->mc_emc_same_freq_thr,
			FIX_PT(mc_settings_ptr->dram_to_emc_freq_ratio,
				0,
				error),
			error
			);

	mc_settings_ptr->freq_range.lo_freq =
		mc_settings_ptr->lowest_dram_freq;
	mc_settings_ptr->freq_range.hi_freq =
		mc_settings_ptr->highest_dram_freq;
	mc_settings_ptr->freq_range.hi_gd = mc_settings_ptr->max_gd;
	lo_freq = fixed_point_lt(
		comparison_freq_thr_to_use,
		mc_settings_ptr->lowest_dram_freq,
		error
		) ? fixed_point_div(
			mc_settings_ptr->lowest_dram_freq,
			FIX_PT(2, 0, error),
			error
			) : mc_settings_ptr->lowest_dram_freq;
	mc_settings_ptr->freq_range.lo_gd =
		fixed_point_mult(
			mc_settings_ptr->max_gd,
			fixed_point_div(
				lo_freq,
				fixed_point_div(
					mc_settings_ptr->highest_dram_freq,
					FIX_PT(2, 0, error),
					error
					),
				error
				),
			error
			);
	mc_settings_ptr->freq_range.valid = 1;
}

static int get_bytes_per_dram_clk(
	enum tegra_dram_t dram_type,
	unsigned int *error)
{
	int bytes_per_dram_clk = 0;

	switch (dram_type) {
	case TEGRA_DDR3_1CH:
	case TEGRA_LP3_1CH:
	case TEGRA_LP4_2CH:
		bytes_per_dram_clk = 16;
		break;
	case TEGRA_DDR3_2CH:
	case TEGRA_LP3_2CH:
	case TEGRA_LP4_4CH:
		bytes_per_dram_clk = 16;
		break;
	case TEGRA_LP4_8CH:
	case TEGRA_LP4X_8CH:
		bytes_per_dram_clk = 32;
		break;
	case TEGRA_LP4_16CH:
	case TEGRA_LP4X_16CH:
		bytes_per_dram_clk = 64;
		break;
	default:
	{
		pr_err("%s: tegra_dram_t %d not handled\n",
			__func__, dram_type);
		(*error) |= 1;
		WARN_ON(1);
	}
	}

	return bytes_per_dram_clk;
}

static struct fixed_point bw2fraction(
	struct mc_settings_info *mc_settings_ptr,
	struct fixed_point bw_mbps,
	unsigned int *error
)
{
	struct fixed_point bw_at_lo_freq_mbps;
	if (mc_settings_ptr->freq_range.valid != 1) {
		pr_err("%s: freq_range.valid not 1, but %d\n",
			__func__, mc_settings_ptr->freq_range.valid);
		(*error) |= 1;
		WARN_ON(1);
	}

	bw_at_lo_freq_mbps =
		fixed_point_mult(
			mc_settings_ptr->freq_range.lo_freq,
			FIX_PT(
				get_bytes_per_dram_clk(
					mc_settings_ptr->dram_type,
					error),
			0,
			error),
			error
			);
	return fixed_point_mult(
		fixed_point_div(
			bw_mbps,
			bw_at_lo_freq_mbps,
			error),
		mc_settings_ptr->freq_range.lo_gd,
		error
		);
}

/*
  Method fraction2dda()
  =================================================================
  Convert a floating point fraction into a DDA value
  =================================================================
*/
static unsigned int fraction2dda(
	struct fixed_point fraction,
	struct fixed_point div,
	unsigned int mask,
	int round_up_or_to_nearest,
	unsigned int *error)
{
	/* round_up_or_to_nearest determines whether the final calculated
	 * DDA rate is to be rounded up or to nearest. Using this input
	 * to the function we can enable rounding to nearest for NISO client
	 * DDA rates. Rounding up for all other cases to be conservative.
	 */
	int i;
	unsigned int dda = 0;
	struct fixed_point f = fixed_point_div(fraction, div, error);

	for (i = 0; i < NV_MC_EMEM_PTSA_RATE_WIDTH; i++) {
		struct fixed_point r;
		f = fixed_point_mult(f, FIX_PT(2, 0, error), error);
		r = FIX_PT(fixed_point_to_int(f, error), 0, error);
		dda = (dda << 1) | ((unsigned int)fixed_point_to_int(r, error));
		f = fixed_point_sub(f, r, error);
	}
	if (fixed_point_gt(f, FIX_PT(0, 0, error), error)) {
		/* Do not round up if the calculated dda is at the mask
		 * value already, it will overflow
		 */
		if (dda != mask) {
			if (round_up_or_to_nearest == 1 ||
				fixed_point_goet(f,
					FIX_PT(0, 0x80000000, error) /* 0.5 */,
					error)
				|| dda == 0) {
				dda++; /* to round up dda value */
			}
		}
	}

	return dda;
}

static void update_new_dda_rate_frac_use_case(
	struct dda_info *dda_info_array,
	struct mc_settings_info *mc_settings_ptr,
	int clientid,
	struct fixed_point bw_mbps,
	unsigned int *error
)
{
	dda_info_array[clientid].frac =
		bw2fraction(mc_settings_ptr, bw_mbps, error);
	dda_info_array[clientid].frac_valid = 1;
	dda_info_array[clientid].rate =
		fraction2dda(
			dda_info_array[clientid].frac,
			dda_info_array[clientid].dda_div,
			dda_info_array[clientid].mask,
			(dda_info_array[clientid].iso_type != TEGRA_NISO),
			error);
}

static void reg_info_init(
	struct reg_info *entry,
	const char *name,
	unsigned int offset,
	int dirty
)
{
	strcpy(entry->name, name);
	entry->offset = offset;
	entry->dirty = dirty;

	entry->val = 0;
}

#define INIT_REG_INFO(info_array, NAME) \
	reg_info_init( \
		&info_array[TEGRA_##NAME##_ID], \
		__stringify(NAME), \
		NAME##_0, \
		0)

static void mc_reg_info_array_init(
	struct reg_info *inf_arr
)
{
	int i;

	for (i = 0; i < TEGRA_KERN_INIT_MC_MAX_ID; i++) {
		reg_info_init(
			&inf_arr[i],
			"",
			0,
			0);
	}

	INIT_REG_INFO(inf_arr, MC_HUB2MCF_REQ_DDA_ENABLE);
	INIT_REG_INFO(inf_arr, MC_HUB_HUB2MCF_REQ_DDA_RATE);
	INIT_REG_INFO(inf_arr, MC_HUBORD_HUB2MCF_REQ_DDA_RATE);
	INIT_REG_INFO(inf_arr, MC_HUBINT_HUB2MCF_REQ_DDA_RATE);
	INIT_REG_INFO(inf_arr, MC_HUB_HUB2MCF_REQ_DDA_MAX);
	INIT_REG_INFO(inf_arr, MC_HUBORD_HUB2MCF_REQ_DDA_MAX);
	INIT_REG_INFO(inf_arr, MC_HUBINT_HUB2MCF_REQ_DDA_MAX);
	INIT_REG_INFO(inf_arr, MC_CIFLL_NVLRHP_LATENCY_ALLOWANCE);
	INIT_REG_INFO(inf_arr, MC_CONFIG_TSA_SINGLE_ARB_ENABLE);
	INIT_REG_INFO(inf_arr, MC_EMEM_ARB_OVERRIDE);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG0);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG1);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG2);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG3);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG4);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG5);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG7);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG2);
	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG1);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE5W);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE0W);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE4W);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_XUSB_HOSTW);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_XUSB_DEVW);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_SATAW);
	INIT_REG_INFO(inf_arr, MC_CLIENT_ORDER_ID_9);
	INIT_REG_INFO(inf_arr, MC_CLIENT_ORDER_ID_28);
	INIT_REG_INFO(inf_arr, MC_FREE_BANK_QUEUES);
	INIT_REG_INFO(inf_arr, MC_MC_SMMU_PTC2H_REQ_MAPPING_OVERRIDE);
	INIT_REG_INFO(inf_arr, MC_MC_SMMU_PTC2H_REQ_MAPPING);
	INIT_REG_INFO(inf_arr, MC_HUB_VC_ARB_SEL);
	INIT_REG_INFO(inf_arr, MC_MC_SMMU_ISO_TBU_CCHK_REQ_PRI_CTRL);
}

#define INIT_MSSNVLINK_REG_INFO(info_array, NAME) \
	do { \
		strcpy(reg_name, apert_name); \
		strcat(reg_name, __stringify(_##NAME)); \
		reg_info_init( \
			&info_array[TEGRA_##NAME##_ID], \
			reg_name, \
			NAME##_0, \
			0); \
	} while (0)

static void mssnvlink_reg_info_array_init(
	struct reg_info *inf_arr,
	const char *apert_name
)
{
	int i;
	char reg_name[MAX_TEGRA_MC_REG_NAME_SIZE];

	for (i = 0; i < TEGRA_KERN_INIT_MSSNVLINK_MAX_ID; i++) {
		reg_info_init(
			&inf_arr[i],
			"",
			0,
			0);
	}

	INIT_MSSNVLINK_REG_INFO(inf_arr, MSSNVLINK_MASTER_MCF_DDA);
}

static void all_reg_info_array_init(
	struct reg_info *mc_inf_arr,
	struct reg_info *mssnvl1_inf_arr,
	struct reg_info *mssnvl2_inf_arr,
	struct reg_info *mssnvl3_inf_arr,
	struct reg_info *mssnvl4_inf_arr
)
{
	mc_reg_info_array_init(
	mc_inf_arr);
	mssnvlink_reg_info_array_init(
	mssnvl1_inf_arr,
		"NV_ADDRESS_MAP_MSS_NVLINK_1_BASE");
	mssnvlink_reg_info_array_init(
	mssnvl2_inf_arr,
		"NV_ADDRESS_MAP_MSS_NVLINK_2_BASE");
	mssnvlink_reg_info_array_init(
	mssnvl3_inf_arr,
		"NV_ADDRESS_MAP_MSS_NVLINK_3_BASE");
	mssnvlink_reg_info_array_init(
	mssnvl4_inf_arr,
		"NV_ADDRESS_MAP_MSS_NVLINK_4_BASE");
}

static void mcpcie_reg_info_array_init(
	struct reg_info *inf_arr
)
{
	int i;

	for (i = 0; i < TEGRA_KERN_INIT_MCPCIE_MAX_ID; i++) {
		reg_info_init(
			&inf_arr[i],
			"",
			0,
			0);
	}

	INIT_REG_INFO(inf_arr, MC_PCFIFO_CLIENT_CONFIG6);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE1W);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE2AW);
	INIT_REG_INFO(inf_arr, MC_TBU_CLIENT_STEERING_CONFIG_PCIE3W);
	INIT_REG_INFO(inf_arr, MC_CLIENT_ORDER_ID_27);
}
#undef INIT_REG_INFO

static unsigned int reg_info_reg_rd(
	struct reg_info *inf_arr,
	unsigned int addr
)
{
	return inf_arr[addr].val;
}

static void reg_info_reg_wr(
	struct reg_info *inf_arr,
	unsigned int addr,
	unsigned int data
)
{
	inf_arr[addr].val = data;
	inf_arr[addr].dirty = 1;
}

static void write_mcf_dda_perf_regs_kern_init(
	struct mc_settings_info *mc_settings_ptr,
	struct reg_info *mc_inf_arr,
	struct reg_info *mssnvl1_inf_arr,
	struct reg_info *mssnvl2_inf_arr,
	struct reg_info *mssnvl3_inf_arr,
	struct reg_info *mssnvl4_inf_arr
)
{
	unsigned int data;

	if ((mc_settings_ptr->hub2mcf_dda == 1) ||
	   ((mc_settings_ptr->hub2mcf_dda == 2) &&
		((mc_settings_ptr->dram_type == TEGRA_LP4_16CH) ||
		(mc_settings_ptr->dram_type == TEGRA_LP4X_16CH)))) {
		data = reg_info_reg_rd(mc_inf_arr,
			TEGRA_MC_HUB2MCF_REQ_DDA_ENABLE_ID);
		data = NV_FLD_SET_DRF_DEF(MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUB_DDA_ENABLE,
			ENABLED,
			data);
		data = NV_FLD_SET_DRF_DEF(MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUBORD_DDA_ENABLE,
			ENABLED,
			data);
		data = NV_FLD_SET_DRF_DEF(MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUBINT_DDA_ENABLE,
			ENABLED,
			data);
		reg_info_reg_wr(mc_inf_arr,
			TEGRA_MC_HUB2MCF_REQ_DDA_ENABLE_ID,
			data);

#define WRITE_HUB2MCF_DDA(VAR_NAME, REG_NAME) \
	do { \
		reg_info_reg_wr(mc_inf_arr, \
			TEGRA_MC_##REG_NAME##_HUB2MCF_REQ_DDA_RATE_ID, \
			mc_settings_ptr->hub2mcf_dda_rate & \
			MC_##REG_NAME##_HUB2MCF_REQ_DDA_RATE_0_## \
			REG_NAME##_DDA_RATE_DEFAULT_MASK); \
		reg_info_reg_wr(mc_inf_arr, \
			TEGRA_MC_##REG_NAME##_HUB2MCF_REQ_DDA_MAX_ID, \
			mc_settings_ptr->hub2mcf_dda_max & \
			MC_##REG_NAME##_HUB2MCF_REQ_DDA_MAX_0_## \
			REG_NAME##_DDA_MAX_DEFAULT_MASK); \
	} while (0)

		WRITE_HUB2MCF_DDA(hub, HUB);
		WRITE_HUB2MCF_DDA(hubord, HUBORD);
		WRITE_HUB2MCF_DDA(hubint, HUBINT);

#undef WRITE_HUB2MCF_DDA
	} else {
		data = reg_info_reg_rd(
			mc_inf_arr,
			TEGRA_MC_HUB2MCF_REQ_DDA_ENABLE_ID);
		data = NV_FLD_SET_DRF_DEF(
			MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUB_DDA_ENABLE,
			DISABLED,
			data);
		data = NV_FLD_SET_DRF_DEF(
			MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUBORD_DDA_ENABLE,
			DISABLED,
			data);
		data = NV_FLD_SET_DRF_DEF(
			MC,
			HUB2MCF_REQ_DDA_ENABLE,
			HUBINT_DDA_ENABLE,
			DISABLED,
			data);
		reg_info_reg_wr(
			mc_inf_arr,
			TEGRA_MC_HUB2MCF_REQ_DDA_ENABLE_ID,
			data);
	}

	if ((mc_settings_ptr->igpu_mcf_dda == 1) ||
	   ((mc_settings_ptr->igpu_mcf_dda == 2) &&
		((mc_settings_ptr->dram_type == TEGRA_LP4_16CH) ||
		(mc_settings_ptr->dram_type == TEGRA_LP4X_16CH)))) {
#define WRITE_MSSNVLINK_MCF_DDA(NVL_NUM) \
	do { \
		data = reg_info_reg_rd( \
			mssnvl##NVL_NUM##_inf_arr, \
			TEGRA_MSSNVLINK_MASTER_MCF_DDA_ID); \
		data = NV_FLD_SET_DRF_DEF( \
			MSSNVLINK, \
			MASTER_MCF_DDA, \
			ENBL, \
			ENABLE, \
			data); \
		data = NV_FLD_SET_DRF_NUM( \
			MSSNVLINK, \
			MASTER_MCF_DDA, \
			RATE, \
			mc_settings_ptr->mssnvlink_mcf_igpu_dda_rate, \
			data); \
		data = NV_FLD_SET_DRF_NUM( \
			MSSNVLINK, \
			MASTER_MCF_DDA, \
			MAX, \
			mc_settings_ptr->mssnvlink_mcf_igpu_dda_max, \
			data); \
		reg_info_reg_wr( \
			mssnvl##NVL_NUM##_inf_arr, \
			TEGRA_MSSNVLINK_MASTER_MCF_DDA_ID, \
			data); \
	} while (0)

		WRITE_MSSNVLINK_MCF_DDA(1);
		WRITE_MSSNVLINK_MCF_DDA(2);
		WRITE_MSSNVLINK_MCF_DDA(3);
		WRITE_MSSNVLINK_MCF_DDA(4);
#undef WRITE_MSSNVLINK_MCF_DDA
	} else {
#define DISABLE_MSSNVLINK_MCF_DDA(NVL_NUM) \
	do { \
		data = reg_info_reg_rd( \
			mssnvl##NVL_NUM##_inf_arr, \
			TEGRA_MSSNVLINK_MASTER_MCF_DDA_ID); \
		data = NV_FLD_SET_DRF_DEF( \
			MSSNVLINK, \
			MASTER_MCF_DDA, \
			ENBL, \
			DISABLE, \
			data); \
		reg_info_reg_wr( \
			mssnvl##NVL_NUM##_inf_arr, \
			TEGRA_MSSNVLINK_MASTER_MCF_DDA_ID, \
			data); \
	} while (0)

		DISABLE_MSSNVLINK_MCF_DDA(1);
		DISABLE_MSSNVLINK_MCF_DDA(2);
		DISABLE_MSSNVLINK_MCF_DDA(3);
		DISABLE_MSSNVLINK_MCF_DDA(4);
#undef DISABLE_MSSNVLINK_MCF_DDA
	}
}

static void disable_pcfifo_interlock(
	struct reg_info *mc_inf_arr
)
{
	unsigned int data = 0x0;
#define DIS_PCFIFO_INT(cfg_num) \
	reg_info_reg_wr(mc_inf_arr, \
		TEGRA_MC_PCFIFO_CLIENT_CONFIG##cfg_num##_ID, \
		data);

	DIS_PCFIFO_INT(0);
	DIS_PCFIFO_INT(1);
	DIS_PCFIFO_INT(2);
	DIS_PCFIFO_INT(3);
	DIS_PCFIFO_INT(4);
	DIS_PCFIFO_INT(5);
	DIS_PCFIFO_INT(7);
#undef DIS_PCFIFO_INT
}

static void wr_pcfifo_interlock_perf_regs(
	struct reg_info *mc_inf_arr
)
{
	unsigned int data;

	data = 0;
	data = NV_FLD_SET_DRF_DEF(
		MC,
		PCFIFO_CLIENT_CONFIG2,
		PCFIFO_XUSB_DEVW_ORDERED_CLIENT,
		ORDERED,
		data);
	reg_info_reg_wr(
		mc_inf_arr,
		TEGRA_MC_PCFIFO_CLIENT_CONFIG2_ID,
		data);

	data = 0;
	data = NV_FLD_SET_DRF_DEF(
		MC,
		PCFIFO_CLIENT_CONFIG1,
		PCFIFO_HDAW_ORDERED_CLIENT,
		ORDERED,
		data);
	data = NV_FLD_SET_DRF_DEF(
		MC,
		PCFIFO_CLIENT_CONFIG1,
		PCFIFO_SATAW_ORDERED_CLIENT,
		ORDERED,
		data);
	reg_info_reg_wr(
		mc_inf_arr,
		TEGRA_MC_PCFIFO_CLIENT_CONFIG1_ID,
		data);
}

static void wr_ord_perf_regs(
	struct reg_info *mc_inf_arr
)
{
	unsigned int data = 0;

	/* Mapping ordered clients to different TBUs */

	#define WRITE_TBU_CLNT_STEER(client_name, tbu_num) \
	do { \
		data = reg_info_reg_rd( \
			mc_inf_arr, \
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_## \
			client_name##_ID); \
		data = NV_FLD_SET_DRF_NUM( \
			MC, \
			TBU_CLIENT_STEERING_CONFIG_##client_name, \
			client_name##_SO_DEV_TBUID, \
			tbu_num, \
			data); \
		reg_info_reg_wr( \
			mc_inf_arr, \
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_## \
				client_name##_ID, \
			data); \
	} while (0)

	WRITE_TBU_CLNT_STEER(PCIE5W, 0x1);
	WRITE_TBU_CLNT_STEER(PCIE0W, 0x2);
	WRITE_TBU_CLNT_STEER(PCIE4W, 0x3);
	WRITE_TBU_CLNT_STEER(XUSB_HOSTW, 0x3);
	WRITE_TBU_CLNT_STEER(XUSB_DEVW, 0x0);
	WRITE_TBU_CLNT_STEER(SATAW, 0x0);
#undef WRITE_TBU_CLNT_STEER
}

static void wr_ord_id_perf_regs(
	struct reg_info *mc_inf_arr
)
{
	unsigned int data;

	data = reg_info_reg_rd(mc_inf_arr,
		TEGRA_MC_CLIENT_ORDER_ID_9_ID);
	data = NV_FLD_SET_DRF_DEF(MC, CLIENT_ORDER_ID_9,
		XUSB_HOSTW_ORDER_ID, ORDER_ID3, data);
	reg_info_reg_wr(mc_inf_arr,
		TEGRA_MC_CLIENT_ORDER_ID_9_ID, data);

	data = reg_info_reg_rd(mc_inf_arr,
		TEGRA_MC_CLIENT_ORDER_ID_28_ID);
	data = NV_FLD_SET_DRF_DEF(MC, CLIENT_ORDER_ID_28,
		PCIE4W_ORDER_ID, ORDER_ID3, data);
	data = NV_FLD_SET_DRF_DEF(MC, CLIENT_ORDER_ID_28,
		PCIE5W_ORDER_ID, ORDER_ID1, data);
	reg_info_reg_wr(mc_inf_arr,
		TEGRA_MC_CLIENT_ORDER_ID_28_ID, data);
}

static void write_perf_regs_kern_init(
	struct mc_settings_info *mc_settings_ptr,
	struct reg_info *mc_inf_arr,
	struct reg_info *mssnvl1_inf_arr,
	struct reg_info *mssnvl2_inf_arr,
	struct reg_info *mssnvl3_inf_arr,
	struct reg_info *mssnvl4_inf_arr
)
{
	unsigned int data;

	if (!mc_settings_ptr->set_perf_regs)
		return;

	write_mcf_dda_perf_regs_kern_init(
		mc_settings_ptr,
	mc_inf_arr,
	mssnvl1_inf_arr,
	mssnvl2_inf_arr,
	mssnvl3_inf_arr,
	mssnvl4_inf_arr
		);

	/* Setting GMMU misses to high priority [Bug#200288764] */
	data = 0x4;
	reg_info_reg_wr(mc_inf_arr,
		TEGRA_MC_CIFLL_NVLRHP_LATENCY_ALLOWANCE_ID, data);

	if (mc_settings_ptr->tsa_arb_fix) {
		data = 0;
		data = NV_FLD_SET_DRF_DEF(MC,
			CONFIG_TSA_SINGLE_ARB_ENABLE,
			SINGLE_ARB_ENABLE, ENABLE, data);
		reg_info_reg_wr(mc_inf_arr,
			TEGRA_MC_CONFIG_TSA_SINGLE_ARB_ENABLE_ID, data);
	}

	/* Setup ISO holdoff clients.
	 * By default, ISO holdoff for DISPLAY is always "on".
	 * ISO holdoff can be turned off completely by setting
	 * the "override" bit.
	 */
	data = reg_info_reg_rd(mc_inf_arr, TEGRA_MC_EMEM_ARB_OVERRIDE_ID);
	if (mc_settings_ptr->iso_holdoff_override) {
		data = NV_FLD_SET_DRF_DEF(MC, EMEM_ARB_OVERRIDE,
			TS2AA_HOLDOFF_OVERRIDE, ENABLE, data);
	} else {
		data = NV_FLD_SET_DRF_DEF(MC, EMEM_ARB_OVERRIDE,
			TS2AA_HOLDOFF_OVERRIDE, DISABLE, data);
	}
	reg_info_reg_wr(mc_inf_arr, TEGRA_MC_EMEM_ARB_OVERRIDE_ID, data);

	disable_pcfifo_interlock(mc_inf_arr);
	if (mc_settings_ptr->pcfifo_interlock)
		wr_pcfifo_interlock_perf_regs(mc_inf_arr);

	if (mc_settings_ptr->en_ordering)
		wr_ord_perf_regs(mc_inf_arr);

	if (mc_settings_ptr->set_order_id)
		wr_ord_id_perf_regs(mc_inf_arr);

	data = reg_info_reg_rd(mc_inf_arr, TEGRA_MC_FREE_BANK_QUEUES_ID);
	if (mc_settings_ptr->hp_cpu_throttle_en) {
		data = NV_FLD_SET_DRF_DEF(MC, FREE_BANK_QUEUES,
			HP_CPU_THROTTLE_EN, ENABLE, data);
	} else {
		data = NV_FLD_SET_DRF_DEF(MC, FREE_BANK_QUEUES,
			HP_CPU_THROTTLE_EN, DISABLE, data);
	}
	reg_info_reg_wr(mc_inf_arr, TEGRA_MC_FREE_BANK_QUEUES_ID, data);

	if (mc_settings_ptr->override_isoptc_hub_mapping) {
		data = reg_info_reg_rd(mc_inf_arr,
			TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_OVERRIDE_ID);
		data = NV_FLD_SET_DRF_DEF(MC,
			MC_SMMU_PTC2H_REQ_MAPPING_OVERRIDE,
			PTC22H_REQ_MAPPING_OVERRIDE, ENABLE, data);
		reg_info_reg_wr(mc_inf_arr,
			TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_OVERRIDE_ID, data);

		data = reg_info_reg_rd(mc_inf_arr,
			TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_ID);
		data = NV_FLD_SET_DRF_NUM(MC, MC_SMMU_PTC2H_REQ_MAPPING,
			PTC22H_REQ_MAPPING,
			mc_settings_ptr->isoptc_hub_num, data);
		reg_info_reg_wr(mc_inf_arr,
			TEGRA_MC_MC_SMMU_PTC2H_REQ_MAPPING_ID, data);
	}

	if (mc_settings_ptr->override_hub_vcarb_type) {
		data = reg_info_reg_rd(mc_inf_arr, TEGRA_MC_HUB_VC_ARB_SEL_ID);
		data = NV_FLD_SET_DRF_NUM(MC, HUB_VC_ARB_SEL, VC_ARB_TYPE,
			mc_settings_ptr->hub_vcarb_type, data);
		reg_info_reg_wr(mc_inf_arr, TEGRA_MC_HUB_VC_ARB_SEL_ID, data);
	}

	if (mc_settings_ptr->override_hub_vcarb_wt) {
		data = reg_info_reg_rd(mc_inf_arr, TEGRA_MC_HUB_VC_ARB_SEL_ID);
		data = NV_FLD_SET_DRF_NUM(MC, HUB_VC_ARB_SEL, NISO_WT,
			mc_settings_ptr->hub_vcarb_niso_wt, data);
		data = NV_FLD_SET_DRF_NUM(MC, HUB_VC_ARB_SEL, SISO_WT,
			mc_settings_ptr->hub_vcarb_siso_wt, data);
		data = NV_FLD_SET_DRF_NUM(MC, HUB_VC_ARB_SEL, ISO_WT,
			mc_settings_ptr->hub_vcarb_iso_wt, data);
		reg_info_reg_wr(mc_inf_arr, TEGRA_MC_HUB_VC_ARB_SEL_ID, data);
	}

	if (mc_settings_ptr->override_iso_tbu_cchk_en_ctrl) {
		data = reg_info_reg_rd(mc_inf_arr,
			TEGRA_MC_MC_SMMU_ISO_TBU_CCHK_REQ_PRI_CTRL_ID);
		data = NV_FLD_SET_DRF_NUM(MC,
			MC_SMMU_ISO_TBU_CCHK_REQ_PRI_CTRL,
			ISO_TBU_CCHK_EN_CTRL,
			mc_settings_ptr->iso_tbu_cchk_en_ctrl, data);
		reg_info_reg_wr(mc_inf_arr,
			TEGRA_MC_MC_SMMU_ISO_TBU_CCHK_REQ_PRI_CTRL_ID, data);
	}

}

#undef INIT_MSSNVLINK_REG_INFO

static void set_pcie1_ord_id(
	struct reg_info *mcpcie_inf_arr,
	struct mc_settings_info *mc_settings_ptr,
	int ordered,
	int id)
{
	unsigned int data;

	if (mc_settings_ptr->pcfifo_interlock) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID);
		if (ordered) {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE1W_ORDERED_CLIENT,
				ORDERED, data);
		} else {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE1W_ORDERED_CLIENT,
				UNORDERED, data);
		}
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID, data);
	}

	if (mc_settings_ptr->set_order_id) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID);
		data = NV_FLD_SET_DRF_NUM(MC, CLIENT_ORDER_ID_27,
			PCIE1W_ORDER_ID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID, data);
	}

	if (mc_settings_ptr->en_ordering) {
		data = reg_info_reg_rd(
			mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE1W_ID);
		data = NV_FLD_SET_DRF_NUM(MC, TBU_CLIENT_STEERING_CONFIG_PCIE1W,
			PCIE1W_SO_DEV_TBUID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE1W_ID, data);
	}
}

static void set_pcie2_ord_id(
	struct reg_info *mcpcie_inf_arr,
	struct mc_settings_info *mc_settings_ptr,
	int ordered,
	int id)
{
	unsigned int data;

	if (mc_settings_ptr->pcfifo_interlock) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID);
		if (ordered) {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE2AW_ORDERED_CLIENT,
				ORDERED, data);
		} else {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE2AW_ORDERED_CLIENT,
				UNORDERED, data);
		}
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID, data);
	}

	if (mc_settings_ptr->set_order_id) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID);
		data = NV_FLD_SET_DRF_NUM(MC, CLIENT_ORDER_ID_27,
			PCIE2AW_ORDER_ID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID, data);
	}

	if (mc_settings_ptr->en_ordering) {
		data = reg_info_reg_rd(
			mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE2AW_ID);
		data = NV_FLD_SET_DRF_NUM(MC,
			TBU_CLIENT_STEERING_CONFIG_PCIE2AW,
			PCIE2AW_SO_DEV_TBUID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE2AW_ID, data);
	}
}

static void set_pcie3_ord_id(
	struct reg_info *mcpcie_inf_arr,
	struct mc_settings_info *mc_settings_ptr,
	int ordered,
	int id)
{
	unsigned int data;

	if (mc_settings_ptr->pcfifo_interlock) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID);
		if (ordered) {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE3W_ORDERED_CLIENT,
				ORDERED, data);
		} else {
			data = NV_FLD_SET_DRF_DEF(MC, PCFIFO_CLIENT_CONFIG6,
				PCFIFO_PCIE3W_ORDERED_CLIENT,
				UNORDERED, data);
		}
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID, data);
	}

	if (mc_settings_ptr->set_order_id) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID);
		data = NV_FLD_SET_DRF_NUM(MC, CLIENT_ORDER_ID_27,
			PCIE3W_ORDER_ID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID, data);
	}

	if (mc_settings_ptr->en_ordering) {
		data = reg_info_reg_rd(
			mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE3W_ID);
		data = NV_FLD_SET_DRF_NUM(MC, TBU_CLIENT_STEERING_CONFIG_PCIE3W,
			PCIE3W_SO_DEV_TBUID,
			id, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_TBU_CLIENT_STEERING_CONFIG_PCIE3W_ID, data);
	}
}

static void update_ord_ids(
	struct reg_info *mcpcie_inf_arr,
	struct mc_settings_info *mc_settings_ptr,
	unsigned int pcie_xbar_cfg,
	unsigned int *error)
{
	unsigned int data;

	data = 0;
	reg_info_reg_wr(mcpcie_inf_arr,
		TEGRA_MC_PCFIFO_CLIENT_CONFIG6_ID,
		data);

	if (mc_settings_ptr->set_order_id) {
		data = reg_info_reg_rd(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID);
		data = NV_FLD_SET_DRF_DEF(MC, CLIENT_ORDER_ID_27,
			PCIE0W_ORDER_ID, ORDER_ID2, data);
		reg_info_reg_wr(mcpcie_inf_arr,
			TEGRA_MC_CLIENT_ORDER_ID_27_ID, data);
	}

	switch (pcie_xbar_cfg) { /* PCIE_COMMON_APPL_COMMON_CONTROL_0 */
	case 0:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 1:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 2:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 3:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 4:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 5:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 6:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 7:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 8:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 9:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 10:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 11:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 12:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 2);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 13:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 14:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 15:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 16:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 17:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 18:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 19:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 20:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 21:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 22:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	case 23:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		break;
	case 24:
		set_pcie1_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie2_ord_id(mcpcie_inf_arr, mc_settings_ptr, 1, 0);
		set_pcie3_ord_id(mcpcie_inf_arr, mc_settings_ptr, 0, 3);
		break;
	default:
	{
		pr_err("%s: pcie_xbar_cfg %d not handled\n",
			__func__, pcie_xbar_cfg);
		(*error) |= 1;
		WARN_ON(1);
	}
	}
}

void init_la_ptsa_core(struct la_ptsa_core *lp)
{
	lp->get_init_la = get_init_la;
	lp->la_info_array_init = la_info_array_init;
	lp->mc_settings_init = mc_settings_init;
	lp->mc_settings_override = mc_settings_override;
	lp->get_disp_rd_lat_allow_given_disp_bw =
		get_disp_rd_lat_allow_given_disp_bw;
	lp->dda_info_array_init = dda_info_array_init;
	lp->update_new_dda_minmax_kern_init = update_new_dda_minmax_kern_init;
	lp->update_new_dda_rate_frac_kern_init =
		update_new_dda_rate_frac_kern_init;
	lp->convert_la2dda_id_for_dyn_ptsa = convert_la2dda_id_for_dyn_ptsa;
	lp->init_max_gd = init_max_gd;
	lp->init_mcemc_same_freq_thr = init_mcemc_same_freq_thr;
	lp->setup_freq_ranges = setup_freq_ranges;
	lp->get_bytes_per_dram_clk = get_bytes_per_dram_clk;
	lp->bw2fraction = bw2fraction;
	lp->fraction2dda = fraction2dda;
	lp->update_new_dda_rate_frac_use_case =
		update_new_dda_rate_frac_use_case;
	lp->all_reg_info_array_init = all_reg_info_array_init;
	lp->write_perf_regs_kern_init = write_perf_regs_kern_init;
	lp->mcpcie_reg_info_array_init = mcpcie_reg_info_array_init;
	lp->update_ord_ids = update_ord_ids;
}
