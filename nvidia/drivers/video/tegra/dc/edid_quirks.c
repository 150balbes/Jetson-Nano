/*
 * edid_quirks.c: edid specific exceptions.
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION, All rights reserved.
 * Author: Anshuman Nath Kar <anshumank@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "edid.h"

static const struct hdmi_blacklist {
	char manufacturer[4];
	u32 model;
	char monitor[14];
	u32 quirks;
} edid_blacklist[] = {
	/* Bauhn ATVS65-815 65" 4K TV */
	{ "CTV", 48, "Tempo 4K TV", TEGRA_EDID_QUIRK_NO_YUV },
	/* Vizio SmartCast P-Series 4K TV */
	{ "VIZ", 4120, "P50-C1",    TEGRA_EDID_QUIRK_DELAY_HDCP },
	{ "VIZ", 4120, "P55-C1",    TEGRA_EDID_QUIRK_DELAY_HDCP },
	{ "VIZ", 4120, "P65-C1",    TEGRA_EDID_QUIRK_DELAY_HDCP },
	{ "VIZ", 4120, "P75-C1",    TEGRA_EDID_QUIRK_DELAY_HDCP },
	/* BlackMagic 12G SDI */
	{ "BMD", 0,    "BMD HDMI",  TEGRA_EDID_QUIRK_NO_HDCP    },
	/* Denon 2313 doesn't support YUV422, but declares support for it */
	{ "DON", 48, "DENON-AVR",   TEGRA_EDID_QUIRK_NO_YUV_422 },
	/* AVerMedia External Capture */
	{ "AVX", 4,    "AVT GC510", TEGRA_EDID_QUIRK_NO_HDCP    },
	/* Few TVs causing HPD bounce for 1-4 seconds */
	{ "YMH", 12774,"RX-A1070",  TEGRA_EDID_QUIRK_HPD_BOUNCE },
	{ "SAM", 3387, "SAMSUNG",   TEGRA_EDID_QUIRK_HPD_BOUNCE },
	/* WAR for bug 2408317 to skip non-CEA modes */
	{ "KL@", 18508, "LCD HDTV", TEGRA_EDID_QUIRK_ONLY_CEA   },
};

u32 tegra_edid_lookup_quirks(const char *manufacturer, u32 model,
	const char *monitor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(edid_blacklist); i++)
		if (!strcmp(edid_blacklist[i].manufacturer, manufacturer) &&
			edid_blacklist[i].model == model &&
			!strcmp(edid_blacklist[i].monitor, monitor))
			return edid_blacklist[i].quirks;

	return TEGRA_EDID_QUIRK_NONE;
}
