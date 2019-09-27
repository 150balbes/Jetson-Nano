/*
 * arch/arm/mach-tegra/isomgr-t19x.c
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

static struct isoclient_info tegra19x_isoclients[] = {
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

static struct isoclient_info *t19x_get_iso_client_info(int *length)
{
	struct isoclient_info *cinfo;
	int i, len;

	cinfo = tegra19x_isoclients;
	len = ARRAY_SIZE(tegra19x_isoclients);
	for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
		isomgr_clients[i].limit_bw_percentage = 100;

	if (length)
		*length = len;

	return cinfo;
}

/* This function is used to find out if a isomgr request is possible
 * This is basically a wrapper for bwmgr_get_lowest_iso_emc_freq, which walks
 * all the supported DRAM clock frequencies and returns the lowest freq which
 * satisfies the bw requirement. It returns 0 if request cannot be supported
 *
 * This function is called from both isomgr_register and isomgr_reserve
 *
 * @ reserve - call made from tegra_isomgr_reserve
 * @ register_bw - bw requested by isomgr client during register KB/s
 * @ reserve_bw - bw requested by isomgr client during reserve KB/s
 * @ regclient - iso client passed to register
 * @ resclient - iso client which is calling reserve
 *
 * returns true if request can be satisfied or else false
 */
static bool is_isomgr_request_possible(bool reserve, u32 register_bw,
				u32 reserve_bw, enum tegra_iso_client regclient,
				enum tegra_iso_client resclient)
{
	u32 min_freq = 0; //KHz
	unsigned long iso_bw_other = 0; //Hz
	unsigned long iso_bw_total = 0; //Hz
	unsigned long iso_bw_nvdis = 0; //Hz
	unsigned long iso_bw_vi = 0; //Hz
	unsigned long ret = 0;
	int i;
	enum tegra_iso_client curr_res_cli = resclient;
	u32 cur_req_bw = 0;
	enum tegra_iso_client cur_iso_client;

	if (!reserve) { //tegra_isomgr_register
		cur_req_bw = register_bw;
		cur_iso_client = regclient;
	} else { //tegra_isomgr_reserve
		cur_req_bw = reserve_bw;
		cur_iso_client = resclient;
	}

	min_freq = bwmgr_bw_to_freq(cur_req_bw);

	if ((cur_iso_client == TEGRA_ISO_CLIENT_DISP_0) ||
	   (cur_iso_client == TEGRA_ISO_CLIENT_DISP_1) ||
	   (cur_iso_client == TEGRA_ISO_CLIENT_DISP_2))
		iso_bw_nvdis = min_freq * 1000;

	else if (cur_iso_client == TEGRA_ISO_CLIENT_TEGRA_CAMERA)
		iso_bw_vi = min_freq * 1000;

	else
		iso_bw_other = min_freq * 1000;

	if (reserve) {
		//bw requests reserved and realized by other ISO clients
		for (i = 0; i < isoclients; i++) {
			if (isoclient_info[i].name) {
				resclient = isoclient_info[i].client;

				/* bw req of client calling is considered,
				 * skip rest of loop.
				 */
				if (curr_res_cli == resclient)
					continue;

				if ((resclient == TEGRA_ISO_CLIENT_DISP_0) ||
				   (resclient == TEGRA_ISO_CLIENT_DISP_1) ||
				   (resclient == TEGRA_ISO_CLIENT_DISP_2))
					iso_bw_nvdis +=
					(max(isomgr_clients[resclient].rsvd_mf,
					isomgr_clients[resclient].real_mf)
					* 1000);

				else if ((resclient ==
					TEGRA_ISO_CLIENT_TEGRA_CAMERA))
					iso_bw_vi +=
					(max(isomgr_clients[resclient].rsvd_mf,
					isomgr_clients[resclient].real_mf)
					* 1000);

				else
					iso_bw_other +=
					(max(isomgr_clients[resclient].rsvd_mf,
					isomgr_clients[resclient].real_mf)
					* 1000);
			}
		}
	}

	iso_bw_total = iso_bw_nvdis + iso_bw_vi + iso_bw_other;

	ret = bwmgr_get_lowest_iso_emc_freq(iso_bw_total, iso_bw_nvdis,
						iso_bw_vi); //inputs in Hz
	if (!ret)
		return false;

	return true;
}

static void t19x_iso_plat_init(void)
{
}

static void t19x_iso_plat_unregister(struct isomgr_client *cp)
{
}

static bool t19x_iso_plat_reserve(struct isomgr_client *cp, u32 bw,
					enum tegra_iso_client client)
{
	if (is_isomgr_request_possible(1, 0, bw, 0, client))
		return true;
	else
		return false;
}

static bool t19x_iso_plat_realize(struct isomgr_client *cp)
{
	/* Nothing specific to do in realize, update internal variables */
	cp->real_bw = 0;
	cp->realize = true;

	cp->real_bw = cp->rsvd_bw; /* reservation has been realized */
	cp->real_mf = cp->rsvd_mf; /* minimum frequency realized */

	return true;
}

static bool t19x_iso_plat_register(u32 dedi_bw, enum tegra_iso_client client)
{
	if (is_isomgr_request_possible(0, dedi_bw, 0, client, 0))
		return true;
	else
		return false;
}

static u32 t19x_iso_max_bw(enum tegra_iso_client client)
{
	return tegra_bwmgr_get_max_iso_bw(client);
}

static struct isomgr_ops isomgr_ops_t19x = {
	.isomgr_plat_init = t19x_iso_plat_init,
	.isomgr_plat_register = t19x_iso_plat_register,
	.isomgr_plat_unregister = t19x_iso_plat_unregister,
	.isomgr_plat_reserve = t19x_iso_plat_reserve,
	.isomgr_plat_realize = t19x_iso_plat_realize,
	.isomgr_max_iso_bw = t19x_iso_max_bw,
};

struct isomgr_ops *t19x_isomgr_init(void)
{
	isoclient_info = t19x_get_iso_client_info(&isoclients);

	/* On T194, camera cannot tolerate emc frequency changes when active.
	 * Set emc floor to max when camera is active to avoid
	 * frequency switching
	 */
	isomgr_camera_max_floor_req = 1;

	return &isomgr_ops_t19x;
}
