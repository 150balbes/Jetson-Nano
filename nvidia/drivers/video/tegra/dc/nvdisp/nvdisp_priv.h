/*
 * nvidia/drivers/video/tegra/nvdisp/nvdisp_priv.h
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
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


#ifndef __DRIVERS_VIDEO_TEGRA_NVDISP_NVDISP_PRIV_H
#define __DRIVERS_VIDEO_TEGRA_NVDISP_NVDISP_PRIV_H


#include <dc_priv_defs.h>
#include <trace/events/display.h>
#include <linux/tegra-powergate.h>
#include "hw_win_nvdisp.h"
#include "dc_priv.h"
#include <uapi/video/tegra_dc_ext.h>

/* TODO: move to device tree */

#define BASE_ADDRESS_DC_A_WINC		0x00000a00
#define BASE_ADDRESS_DC_A_WIN		0x00000b80
#define BASE_ADDRESS_DC_A_WINBUF	0x00000bc0

#define BASE_ADDRESS_DC_B_WINC		0x00000d00
#define BASE_ADDRESS_DC_B_WIN		0x00000e80
#define BASE_ADDRESS_DC_B_WINBUF	0x00000ec0

#define BASE_ADDRESS_DC_C_WINC		0x00001000
#define BASE_ADDRESS_DC_C_WIN		0x00001180
#define BASE_ADDRESS_DC_C_WINBUF	0x000011c0

#define BASE_ADDRESS_DC_D_WINC		0x00001300
#define BASE_ADDRESS_DC_D_WIN		0x00001480
#define BASE_ADDRESS_DC_D_WINBUF	0x000014c0

#define BASE_ADDRESS_DC_E_WINC		0x00001600
#define BASE_ADDRESS_DC_E_WIN		0x00001780
#define BASE_ADDRESS_DC_E_WINBUF	0x000017c0

#define BASE_ADDRESS_DC_F_WINC		0x00001900
#define BASE_ADDRESS_DC_F_WIN		0x00001a80
#define BASE_ADDRESS_DC_F_WINBUF	0x00001ac0

static u32 BASE_ADDRESS_DC_WIN_WINC[] = {
		BASE_ADDRESS_DC_A_WINC,
		BASE_ADDRESS_DC_B_WINC,
		BASE_ADDRESS_DC_C_WINC,
		BASE_ADDRESS_DC_D_WINC,
		BASE_ADDRESS_DC_E_WINC,
		BASE_ADDRESS_DC_F_WINC,
};

static u32 BASE_ADDRESS_DC_WIN_WIN[] = {
		BASE_ADDRESS_DC_A_WIN,
		BASE_ADDRESS_DC_B_WIN,
		BASE_ADDRESS_DC_C_WIN,
		BASE_ADDRESS_DC_D_WIN,
		BASE_ADDRESS_DC_E_WIN,
		BASE_ADDRESS_DC_F_WIN,
};

static u32 BASE_ADDRESS_DC_WIN_WINBUF[] = {
		BASE_ADDRESS_DC_A_WINBUF,
		BASE_ADDRESS_DC_B_WINBUF,
		BASE_ADDRESS_DC_C_WINBUF,
		BASE_ADDRESS_DC_D_WINBUF,
		BASE_ADDRESS_DC_E_WINBUF,
		BASE_ADDRESS_DC_F_WINBUF,
};

enum {
	NVDISP_DEGAMMA_NONE,
	NVDISP_DEGAMMA_SRGB,
	NVDISP_DEGAMMA_YUV_8_10,
	NVDISP_DEGAMMA_YUV_12,
};

#define NVDISP_WIN_ADDR(head, win, word_offset) \
		nvdisp_win_offset(win, word_offset) << 2

/* convert window indirect to window direct offsets within a head */
static u32 nvdisp_win_offset(const u32 win, const u32 word_offset)
{

	if (word_offset >= win_base_addr_dc_winbuf_r()) /* WINBUF */
		return (word_offset - win_base_addr_dc_winbuf_r()) +
			BASE_ADDRESS_DC_WIN_WINBUF[win];

	else if (word_offset >= win_base_addr_dc_win_r()) /* WIN */
		return (word_offset - win_base_addr_dc_win_r()) +
			BASE_ADDRESS_DC_WIN_WIN[win];

	else if (word_offset >= win_base_addr_dc_winc_r()) /* WINC */
		return (word_offset - win_base_addr_dc_winc_r()) +
			 BASE_ADDRESS_DC_WIN_WINC[win];

	else
		pr_err("bad window address 0x%x, win %u\n", word_offset, win);

	return 0;
}

static inline u32 nvdisp_win_read(struct tegra_dc_win *win, u32 off)
{
	u32 ret;
	u32 reg;
	struct tegra_dc *dc = win->dc;

	/* assuming we create three instances of dc for the three heads */
	reg = NVDISP_WIN_ADDR(dc->ctrl_num, win->idx, off);
	ret = readl(dc->base + reg);
	trace_display_readl(dc, ret, dc->base + reg);
	return ret;
}


static inline void nvdisp_win_write(struct tegra_dc_win *win, u32 val, u32 off)
{
	u32 reg;
	struct tegra_dc *dc = win->dc;

	reg = NVDISP_WIN_ADDR(dc->ctrl_num, win->idx, off);
	trace_display_writel(dc, val, dc->base + reg);
	writel(val, dc->base + reg);
}

static inline bool tegra_dc_is_yuv_12bpc(int fmt)
{
	switch (tegra_dc_fmt(fmt)) {
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444:
		return true;
	default:
		return false;
	}

	return false;
}

void nvdisp_clock_init(struct tegra_dc *dc);
dma_addr_t __attribute__((weak))
	nvdisp_t19x_get_addr_flag(struct tegra_dc_win *win);

void tegra_nvdisp_crc_region_init(void);

#endif

