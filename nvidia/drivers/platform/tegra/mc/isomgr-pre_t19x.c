/*
 * arch/arm/mach-tegra/isomgr-pre_t19x.c
 *
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
 *
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#ifdef CONFIG_COMMON_CLK
#include <linux/platform/tegra/bwmgr_mc.h>
#else
#include <linux/platform/tegra/mc.h>
#include <linux/clk.h>
#include <linux/platform/tegra/clock.h>
#endif
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/io.h>
#include <soc/tegra/chip-id.h>

/* This file contains platform specific isomgr
 * code for all chips prior to T194.
 */

#define ISOMGR_DEBUG 1

#if ISOMGR_DEBUG
#define SANITY_CHECK_AVAIL_BW() { \
	int t = 0; \
	int idx = 0; \
	for (idx = 0; idx < TEGRA_ISO_CLIENT_COUNT; idx++) { \
		if (isomgr_clients[idx].real_bw >= \
		   isomgr_clients[idx].margin_bw) \
			t += isomgr_clients[idx].real_bw; \
		else \
			t += isomgr_clients[idx].margin_bw; \
	} \
	if (t + isomgr.avail_bw != isomgr.max_iso_bw) { \
		pr_err("bw mismatch, line=%d\n", __LINE__); \
		pr_err("t+isomgr.avail_bw=%d, isomgr.max_iso_bw=%d\n", \
			t + isomgr.avail_bw, isomgr.max_iso_bw); \
		return false; \
	} \
}
#else
#define SANITY_CHECK_AVAIL_BW()
#endif

static int iso_bw_percentage = 100;

static struct isoclient_info tegra_null_isoclients[] = {
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info tegra11x_isoclients[] = {
	{
		.client = TEGRA_ISO_CLIENT_DISP_0,
		.name = "disp_0",
		.dev_name = "tegradc.0",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_1,
		.name = "disp_1",
		.dev_name = "tegradc.1",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_VI_0,
		.name = "vi_0",
		.dev_name = "vi",
		.emc_clk_name = "emc",
	},
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info tegra14x_isoclients[] = {
	{
		.client = TEGRA_ISO_CLIENT_DISP_0,
		.name = "disp_0",
		.dev_name = "tegradc.0",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_1,
		.name = "disp_1",
		.dev_name = "tegradc.1",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_VI_0,
		.name = "vi_0",
		.dev_name = "vi",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_BBC_0,
		.name = "bbc_0",
		.dev_name = "tegra_bb.0",
		.emc_clk_name = "emc_bw",
	},
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info tegra12x_isoclients[] = {
	{
		.client = TEGRA_ISO_CLIENT_DISP_0,
		.name = "disp_0",
		.dev_name = "tegradc.0",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_1,
		.name = "disp_1",
		.dev_name = "tegradc.1",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_VI_0,
		.name = "vi_0",
		.dev_name = "tegra_vi",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_ISP_A,
		.name = "isp_a",
		.dev_name = "tegra_isp.0",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_ISP_B,
		.name = "isp_b",
		.dev_name = "tegra_isp.1",
		.emc_clk_name = "emc",
	},
	{
		.client = TEGRA_ISO_CLIENT_TEGRA_CAMERA,
		.name = "tegra_camera",
		.dev_name = "tegra_camera_ctrl",
		.emc_clk_name = "iso.emc",
	},
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info tegra21x_isoclients[] = {
	{
		.client = TEGRA_ISO_CLIENT_DISP_0,
		.name = "disp_0",
		.dev_name = "tegradc.0",
		.emc_clk_name = "emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_DISP0,
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_1,
		.name = "disp_1",
		.dev_name = "tegradc.1",
		.emc_clk_name = "emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_DISP1,
	},
	{
		.client = TEGRA_ISO_CLIENT_VI_0,
		.name = "vi_0",
		.dev_name = "tegra_vi",
		.emc_clk_name = "emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_VI,
	},
	{
		.client = TEGRA_ISO_CLIENT_ISP_A,
		.name = "isp_a",
		.dev_name = "tegra_isp.0",
		.emc_clk_name = "emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_ISPA,
	},
	{
		.client = TEGRA_ISO_CLIENT_ISP_B,
		.name = "isp_b",
		.dev_name = "tegra_isp.1",
		.emc_clk_name = "emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_ISPB,
	},
	{
		.client = TEGRA_ISO_CLIENT_TEGRA_CAMERA,
		.name = "tegra_camera",
		.dev_name = "tegra_camera_ctrl",
		.emc_clk_name = "iso.emc",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_CAMERA,
	},
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info tegra18x_isoclients[] = {
	{
		.client = TEGRA_ISO_CLIENT_DISP_0,
		.name = "disp_0",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_DISP0,
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_1,
		.name = "disp_1",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_DISP1,
	},
	{
		.client = TEGRA_ISO_CLIENT_DISP_2,
		.name = "disp_2",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_DISP2,
	},
	{
		.client = TEGRA_ISO_CLIENT_ISP_A,
		.name = "isp_a",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_ISPA,
	},
	{
		.client = TEGRA_ISO_CLIENT_TEGRA_CAMERA,
		.name = "camera",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_CAMERA,
	},
	{
		.client = TEGRA_ISO_CLIENT_APE_ADMA,
		.name = "ape_adma",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_APE_ADMA,
	},
	{
		.client = TEGRA_ISO_CLIENT_EQOS,
		.name = "eqos",
		.bwmgr_id = TEGRA_BWMGR_CLIENT_EQOS,
	},
	/* This must be last entry*/
	{
		.client = TEGRA_ISO_CLIENT_COUNT,
		.name = NULL,
	},
};

static struct isoclient_info *get_iso_client_info(int *length)
{
	enum tegra_chipid cid;
	struct isoclient_info *cinfo;
	int i, len;

	cid = tegra_get_chip_id();
	switch (cid) {
	case TEGRA114:
		cinfo = tegra11x_isoclients;
		len = ARRAY_SIZE(tegra11x_isoclients);
		iso_bw_percentage = 50;
		for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
			isomgr_clients[i].limit_bw_percentage = 100;
		break;
	case TEGRA148:
		cinfo = tegra14x_isoclients;
		len = ARRAY_SIZE(tegra14x_isoclients);
		iso_bw_percentage = 50;
		for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
			isomgr_clients[i].limit_bw_percentage = 100;
		break;
	case TEGRA124:
	case TEGRA132:
		cinfo = tegra12x_isoclients;
		len = ARRAY_SIZE(tegra12x_isoclients);
		iso_bw_percentage = 50;
		for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
			isomgr_clients[i].limit_bw_percentage = 100;
		break;
	case TEGRA210:
		cinfo = tegra21x_isoclients;
		iso_bw_percentage = 45; /* Hack: Should be determined based on
					 * DRAM type
					 */
		len = ARRAY_SIZE(tegra21x_isoclients);
		for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
			isomgr_clients[i].limit_bw_percentage = 100;
		break;
	case TEGRA186:
		cinfo = tegra18x_isoclients;
		len = ARRAY_SIZE(tegra18x_isoclients);
		for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++) {
			if (i == TEGRA_ISO_CLIENT_TEGRA_CAMERA)
				isomgr_clients[i].limit_bw_percentage = 10;
			else
				isomgr_clients[i].limit_bw_percentage = 100;
		}
		break;
	default:
		cinfo = tegra_null_isoclients;
		len = 0;
		break;
	}

	if (length)
		*length = len;

	return cinfo;
}

static void pre_t19x_iso_plat_init(void)
{
	unsigned int max_emc_clk;
	unsigned int max_emc_bw;

	if (!isomgr.max_iso_bw) {
#ifdef CONFIG_COMMON_CLK
		max_emc_clk = tegra_bwmgr_get_max_emc_rate() / 1000;
		max_emc_bw = bwmgr_freq_to_bw(max_emc_clk);
		isomgr.max_iso_bw = max_emc_bw *
			bwmgr_iso_bw_percentage_max() / 100;
#else
		/* With DVFS disabled, bus children cannot get real
		 * max emc freq supported Only the root parent EMC
		 * node is set to max possible rate
		 */
		max_emc_clk = clk_round_rate(clk_get_parent(isomgr.emc_clk),
						ULONG_MAX) / 1000;
		max_emc_bw = tegra_emc_freq_req_to_bw(max_emc_clk);
		/* ISO clients can use iso_bw_percentage of max emc bw. */
		isomgr.max_iso_bw = max_emc_bw * iso_bw_percentage / 100;
#endif
		pr_info("iso emc max clk=%dKHz\n", max_emc_clk);
		pr_info("max_iso_bw=%dKB\n", isomgr.max_iso_bw);
		isomgr.avail_bw = isomgr.max_iso_bw;
	}
}

static void pre_t19x_iso_plat_unregister(struct isomgr_client *cp)
{
	if (cp->real_bw > cp->margin_bw)
		isomgr.avail_bw += cp->real_bw;
	else
		isomgr.avail_bw += cp->margin_bw;
}

static bool pre_t19x_iso_plat_reserve(struct isomgr_client *cp, u32 bw,
					enum tegra_iso_client client)
{
	u64 bw_check;
	u32 max_emc_bw;

	if (bw <= cp->margin_bw)
		goto bw_limit_check;

	if (bw > cp->dedi_bw &&
	  bw > isomgr.avail_bw + cp->real_bw - isomgr.sleep_bw)
		return false;

bw_limit_check:
	/* During reserve, check if BW request is within limit_bw_percentage%
	 * of max emc bw. Using max emc bw to find if the request is possible
	 * when we raise the freq to max possible value. If not, the reserve
	 * call will fail
	 */
#ifdef CONFIG_COMMON_CLK
	max_emc_bw = bwmgr_freq_to_bw(tegra_bwmgr_get_max_emc_rate() / 1000);
#else
	max_emc_bw = tegra_emc_freq_req_to_bw(
		clk_round_rate(clk_get_parent(isomgr.emc_clk), ULONG_MAX)
		/ 1000);
#endif
	bw_check = ((u64)max_emc_bw * (u64)(cp->limit_bw_percentage) / 100);
	if (bw > bw_check)
		return false;
	else
		return true;

}

static bool pre_t19x_iso_plat_realize(struct isomgr_client *cp)
{
	s32 delta_bw = 0;

	if (cp->margin_bw < cp->real_bw)
		isomgr.avail_bw += cp->real_bw - cp->margin_bw;

	cp->real_bw = 0;
	cp->realize = true;

	if (unlikely(isomgr.avail_bw > isomgr.max_iso_bw)) {
		pr_err("isomgr: iso_plat_realize: avail_bw > max_iso_bw\n");
		return false;
	}

	if (cp->rsvd_bw <= cp->margin_bw) {
		if (unlikely(cp->sleep_bw)) {
			pr_err
			("isomgr_realize: rsvd_bw < margin_bw, sleep_bw = 1\n");
			return false;
		}
		cp->real_bw = cp->rsvd_bw; /* reservation has been realized */
		cp->real_mf = cp->rsvd_mf; /* minimum frequency realized */
	} else if (cp->rsvd_bw <= isomgr.avail_bw + cp->margin_bw) {
		delta_bw = cp->rsvd_bw - cp->margin_bw;
		isomgr.avail_bw -= delta_bw;
		cp->real_bw = cp->rsvd_bw; /* reservation has been realized */
		cp->real_mf = cp->rsvd_mf; /* minimum frequency realized */
		if (cp->sleep_bw) {
			isomgr.sleep_bw -= delta_bw;
			cp->sleep_bw -= delta_bw;
			if (unlikely(cp->sleep_bw)) {
				pr_err
				("isomgr:rsvd_bw < margin_bw, sleep_bw = 1\n");
				return false;
			}
		}
		if (unlikely(isomgr.avail_bw < 0)) {
			pr_err("isomgr: iso_plat_realize: avail_bw < 0\n");
			return false;
		}
		SANITY_CHECK_AVAIL_BW();
	} else {
		return false;
	}

	return true;
}

static bool pre_t19x_iso_plat_register(u32 dedi_bw, enum tegra_iso_client client)
{
	if (unlikely(dedi_bw > isomgr.max_iso_bw - isomgr.dedi_bw)) {
		pr_err("iso bandwidth %uKB is not available, client %s\n",
			dedi_bw, cname[client]);
		return false;
	}
	return true;
}

static struct isomgr_ops isomgr_ops_pre_t19x = {
	.isomgr_plat_init = pre_t19x_iso_plat_init,
	.isomgr_plat_register = pre_t19x_iso_plat_register,
	.isomgr_plat_unregister = pre_t19x_iso_plat_unregister,
	.isomgr_plat_reserve = pre_t19x_iso_plat_reserve,
	.isomgr_plat_realize = pre_t19x_iso_plat_realize,
};

struct isomgr_ops *pre_t19x_isomgr_init(void)
{
	isoclient_info = get_iso_client_info(&isoclients);
	return &isomgr_ops_pre_t19x;
}
