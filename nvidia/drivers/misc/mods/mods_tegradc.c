/*
 * mods_tegradc.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/uaccess.h>
#include <../drivers/video/tegra/dc/dc_priv.h>
#include <video/tegra_dc_ext_kernel.h>
#include <linux/platform/tegra/mc.h>
#include "mods_internal.h"

static void mods_tegra_dc_set_windowattr_basic(struct tegra_dc_win *win,
		       const struct MODS_TEGRA_DC_WINDOW *mods_win)
{
	win->global_alpha = 0;
	win->z            = 0;
	win->stride       = 0;
	win->stride_uv    = 0;

	win->flags = TEGRA_WIN_FLAG_ENABLED;
	if (mods_win->flags & MODS_TEGRA_DC_WINDOW_FLAG_TILED)
		win->flags |= TEGRA_WIN_FLAG_TILED;
	if (mods_win->flags & MODS_TEGRA_DC_WINDOW_FLAG_SCAN_COL)
		win->flags |= TEGRA_WIN_FLAG_SCAN_COLUMN;

	win->fmt = mods_win->pixformat;
	win->x.full = mods_win->x;
	win->y.full = mods_win->y;
	win->w.full = mods_win->w;
	win->h.full = mods_win->h;
	/* XXX verify that this doesn't go outside display's active region */
	win->out_x = mods_win->out_x;
	win->out_y = mods_win->out_y;
	win->out_w = mods_win->out_w;
	win->out_h = mods_win->out_h;

	mods_debug_printk(DEBUG_TEGRADC,
		"set_windowattr_basic window %u:\n"
		"\tflags : 0x%08x\n"
		"\tfmt   : %u\n"
		"\tinput : (%u, %u, %u, %u)\n"
		"\toutput: (%u, %u, %u, %u)\n",
		win->idx, win->flags, win->fmt, dfixed_trunc(win->x),
		dfixed_trunc(win->y), dfixed_trunc(win->w),
		dfixed_trunc(win->h), win->out_x, win->out_y, win->out_w,
		win->out_h);
}

int esc_mods_tegra_dc_config_possible(struct file *fp,
				struct MODS_TEGRA_DC_CONFIG_POSSIBLE *args)
{
	int i;
	struct tegra_dc *dc = tegra_dc_get_dc(args->head);
	struct tegra_dc_win *dc_wins[DC_N_WINDOWS];
#ifndef CONFIG_TEGRA_ISOMGR
	struct clk *emc_clk = 0;
	unsigned long max_bandwidth = 0;
	unsigned long current_emc_freq = 0;
	unsigned long max_available_bandwidth = 0;
#else
	int ret = -EINVAL;
#endif

	LOG_ENT();

	BUG_ON(args->win_num > tegra_dc_get_numof_dispwindows());

	if (!dc) {
		LOG_EXT();
		return -EINVAL;
	}

	for (i = 0; i < args->win_num; i++) {
		int idx = args->windows[i].index;

		if (args->windows[i].flags &
			MODS_TEGRA_DC_WINDOW_FLAG_ENABLED) {
			mods_tegra_dc_set_windowattr_basic(&dc->tmp_wins[idx],
							  &args->windows[i]);
		} else {
			dc->tmp_wins[idx].flags = 0;
		}
		dc_wins[i] = &dc->tmp_wins[idx];
		mods_debug_printk(DEBUG_TEGRADC,
			"head %u, using index %d for win %d\n",
			args->head, i, idx);
	}

	mods_debug_printk(DEBUG_TEGRADC,
		"head %u, dc->mode.pclk %u\n",
		args->head, dc->mode.pclk);

#ifndef CONFIG_TEGRA_ISOMGR
	max_bandwidth = tegra_dc_get_bandwidth(dc_wins, args->win_num);

	emc_clk = clk_get_sys("tegra_emc", "emc");
	if (IS_ERR(emc_clk)) {
		mods_debug_printk(DEBUG_TEGRADC,
		"invalid clock specified when fetching EMC clock\n");
	} else {
		current_emc_freq = clk_get_rate(emc_clk);
		current_emc_freq /= 1000;
		max_available_bandwidth =
			8 * tegra_emc_freq_req_to_bw(current_emc_freq);
		max_available_bandwidth = (max_available_bandwidth / 100) * 50;
	}

	mods_debug_printk(DEBUG_TEGRADC,
		"b/w needed %lu, b/w available %lu\n",
		max_bandwidth, max_available_bandwidth);

	args->possible = (max_bandwidth <= max_available_bandwidth);
#else
	ret = tegra_dc_bandwidth_negotiate_bw(dc, dc_wins, args->win_num);
	args->possible = (ret == 0);
#endif
	for (i = 0; i < args->win_num; i++) {
		args->windows[i].bandwidth = dc_wins[i]->new_bandwidth;
		mods_debug_printk(DEBUG_TEGRADC,
			"head %u, win %d, b/w %d\n",
			args->head, dc_wins[i]->idx, dc_wins[i]->new_bandwidth);
	}

	LOG_EXT();
	return 0;
}

int mods_init_tegradc(void)
{
	return 0;
}

