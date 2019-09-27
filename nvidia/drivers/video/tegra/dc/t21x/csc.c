/*
 * csc.c: function required for color space conversion.
 *
 * Copyright (c) 2010-2018, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/err.h>
#include <linux/types.h>
#include <linux/export.h>
#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"

void tegra_dc_init_win_csc_defaults(struct tegra_dc_win_csc *win_csc)
{
	win_csc->yof   = 0x00f0;
	win_csc->kyrgb = 0x012a;
	win_csc->kur   = 0x0000;
	win_csc->kvr   = 0x0198;
	win_csc->kug   = 0x039b;
	win_csc->kvg   = 0x032f;
	win_csc->kub   = 0x0204;
	win_csc->kvb   = 0x0000;
}

void tegra_dc_set_win_csc(struct tegra_dc *dc, struct tegra_dc_win_csc *win_csc)
{
	tegra_dc_writel(dc, win_csc->yof,	DC_WIN_CSC_YOF);
	tegra_dc_writel(dc, win_csc->kyrgb,	DC_WIN_CSC_KYRGB);
	tegra_dc_writel(dc, win_csc->kur,	DC_WIN_CSC_KUR);
	tegra_dc_writel(dc, win_csc->kvr,	DC_WIN_CSC_KVR);
	tegra_dc_writel(dc, win_csc->kug,	DC_WIN_CSC_KUG);
	tegra_dc_writel(dc, win_csc->kvg,	DC_WIN_CSC_KVG);
	tegra_dc_writel(dc, win_csc->kub,	DC_WIN_CSC_KUB);
	tegra_dc_writel(dc, win_csc->kvb,	DC_WIN_CSC_KVB);
}

int tegra_dc_update_win_csc(struct tegra_dc *dc, int win_idx)
{
	struct tegra_dc_win *win = tegra_dc_get_window(dc, win_idx);

	if (!win)
		return -EFAULT;
	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return -EFAULT;
	}

	tegra_dc_get(dc);
	tegra_dc_writel(dc, WINDOW_A_SELECT << win_idx,
			DC_CMD_DISPLAY_WINDOW_HEADER);

	tegra_dc_set_win_csc(dc, &win->win_csc);
	tegra_dc_put(dc);

	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_update_win_csc);

