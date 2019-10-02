/*
 * dc_config.h: Declarations for tegra dc config settings.
 *
 * Copyright (c) 2010-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_CONFIG_H
#define __DRIVERS_VIDEO_TEGRA_DC_DC_CONFIG_H

#include <linux/errno.h>
#include <video/tegra_dc_ext.h>
#include "dc.h"
#include "dc_priv.h"

#define WIN_FEATURE_ENTRY_SIZE	5	/* Size of feature entry args */
#define TEGRA_WIN_SW_FORMAT_MIN		256
#define TEGRA_WIN_SW_FORMAT_MAX		267

/* adjust >32 bit shift for an individual 32-bit word */
#define BIT_FOR_WORD(word, x) ( \
		(x) >= (word) * 32 && \
		(x) < 32 + (word) * 32 \
		? BIT((x) - (word) * 32) : 0)
#define BITWORD_SW_FORMAT(x) ( \
		(x) >= TEGRA_WIN_SW_FORMAT_MIN && \
		(x) <= TEGRA_WIN_SW_FORMAT_MAX \
		? BIT(x - TEGRA_WIN_SW_FORMAT_MIN) : 0)
#define BITWORD3(x) BIT_FOR_WORD(3, x)
#define BITWORD2(x) BIT_FOR_WORD(2, x)
#define BITWORD1(x) BIT_FOR_WORD(1, x)
#define BITWORD0(x) BIT_FOR_WORD(0, x)
#define HIGHBIT(x) BIT_FOR_WORD(1, x)

/* Define the supported formats. TEGRA_WIN_FMT_WIN_x macros are defined
 * based on T20/T30 formats. */
#define TEGRA_WIN_FMT_BASE \
	(BIT(TEGRA_DC_EXT_FMT_T_P8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A4R4G4B4) | \
	BIT(TEGRA_DC_EXT_FMT_T_A1R5G5B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G6B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G5B5A1) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8) | \
	BIT(TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8) | \
	BIT(TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8_TRUE) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420_TRUE) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422_TRUE) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R) | \
	BIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R_TRUE))

#define TEGRA_WIN_FMT_T124_LOW TEGRA_WIN_FMT_BASE
#define TEGRA_WIN_FMT_T124_HIGH \
	(HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444_TRUE) | \
	HIGHBIT(TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444_TRUE))

#define TEGRA_WIN_FMT_T210_LOW TEGRA_WIN_FMT_BASE
#define TEGRA_WIN_FMT_T210_HIGH TEGRA_WIN_FMT_T124_HIGH

/* for windows that support compression */
#define TEGRA_WIN_FMT_COMPRESSION_T210_LOW \
	(BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))
#define TEGRA_WIN_FMT_COMPRESSION_T210_HIGH  (0)

/* for windows that can't support planar rotation */
#define TEGRA_WIN_FMT_ROTATION_T210_LOW TEGRA_WIN_FMT_BASE
#define TEGRA_WIN_FMT_ROTATION_T210_HIGH (0)

#define TEGRA_WIN_FMT_WIN_A \
	(BIT(TEGRA_DC_EXT_FMT_T_P8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A4R4G4B4) | \
	BIT(TEGRA_DC_EXT_FMT_T_A1R5G5B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G6B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G5B5A1) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))

#define TEGRA_WIN_FMT_WIN_B \
	(TEGRA_WIN_FMT_BASE | \
	BIT(TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8) | \
	BIT(TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE))

#define TEGRA_WIN_FMT_WIN_C \
	(TEGRA_WIN_FMT_BASE | \
	BIT(TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8) | \
	BIT(TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE))

/* preferred formats do not include 32-bpp formats */
#define TEGRA_WIN_PREF_FMT_WIN_B \
	(TEGRA_WIN_FMT_WIN_B & \
	~BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) & \
	~BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))

#define TEGRA_WIN_FMT_SIMPLE \
	(BIT(TEGRA_DC_EXT_FMT_T_A4R4G4B4) | \
	BIT(TEGRA_DC_EXT_FMT_T_A1R5G5B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G6B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))

#define TEGRA_WIN_FMT_SIMPLE_T210_LOW \
	(BIT(TEGRA_DC_EXT_FMT_T_A4R4G4B4) | \
	BIT(TEGRA_DC_EXT_FMT_T_A1R5G5B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_R5G6B5) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8R8G8B8) | \
	BIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))

#define TEGRA_WIN_FMT_SIMPLE_T210_HIGH (HIGHBIT(TEGRA_DC_EXT_FMT_T_A8B8G8R8))


/* For each entry, we define the offset to read specific feature. Define the
 * offset for TEGRA_DC_FEATURE_MAXIMUM_SCALE */
#define H_SCALE_UP	0
#define V_SCALE_UP	1
#define H_FILTER_DOWN	2
#define V_FILTER_DOWN	3

/* Define the offset for TEGRA_DC_FEATURE_MAXIMUM_SIZE */
#define MAX_WIDTH	0
#define MIN_WIDTH	1
#define MAX_HEIGHT	2
#define MIN_HEIGHT	3
#define CHECK_SIZE(val, min, max)	( \
		((val) < (min) || (val) > (max)) ? -EINVAL : 0)

/* Define the offset for TEGRA_DC_FEATURE_FILTER_TYPE */
#define V_FILTER	0
#define H_FILTER	1

/* Define the offset for TEGRA_DC_FEATURE_INVERT_TYPE */
#define H_INVERT	0
#define V_INVERT	1
#define SCAN_COLUMN	2

/* Define the offset for TEGRA_DC_FEATURE_LAYOUT_TYPE. */
#define PITCHED_LAYOUT	0
#define TILED_LAYOUT	1
#define BLOCK_LINEAR	2

/* Define the offset for TEGRA_DC_FEATURE_BLEND_TYPE. */
#define BLEND_GENERATION	0

#define INTERLACE		0

/* Available operations on feature table. */
enum {
	HAS_SCALE,
	HAS_TILED,
	HAS_V_FILTER,
	HAS_H_FILTER,
	HAS_GEN2_BLEND,
	GET_WIN_FORMATS,
	GET_WIN_SIZE,
	HAS_BLOCKLINEAR,
	HAS_INTERLACE,
	GET_INVERT,
};

enum tegra_dc_feature_option {
	TEGRA_DC_FEATURE_FORMATS,
	TEGRA_DC_FEATURE_BLEND_TYPE,
	TEGRA_DC_FEATURE_MAXIMUM_SIZE,
	TEGRA_DC_FEATURE_MAXIMUM_SCALE,
	TEGRA_DC_FEATURE_FILTER_TYPE,
	TEGRA_DC_FEATURE_LAYOUT_TYPE,
	TEGRA_DC_FEATURE_INVERT_TYPE,
	TEGRA_DC_FEATURE_PREFERRED_FORMATS,
	TEGRA_DC_FEATURE_FIELD_TYPE,
	TEGRA_DC_FEATURE_COMPRESSION_FORMATS,
	TEGRA_DC_FEATURE_ROTATION_FORMATS,
	TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
	TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE,
};

struct tegra_dc_feature_entry {
	u32 window_index;
	u32 option;
	u32 arg[WIN_FEATURE_ENTRY_SIZE];
};

struct tegra_dc_feature {
	u32 num_entries;
	struct tegra_dc_feature_entry *entries;
};

int tegra_dc_feature_has_scaling(struct tegra_dc *dc, int win_idx);
int tegra_dc_feature_has_tiling(struct tegra_dc *dc, int win_idx);
int tegra_dc_feature_has_blocklinear(struct tegra_dc *dc, int win_idx);
int tegra_dc_feature_has_interlace(struct tegra_dc *dc, int win_idx);
int tegra_dc_feature_has_filter(struct tegra_dc *dc, int win_idx, int operation);
int tegra_dc_feature_is_gen2_blender(struct tegra_dc *dc, int win_idx);
int tegra_dc_feature_has_scan_column(struct tegra_dc *dc, int win_idx);

u32 *tegra_dc_parse_feature(struct tegra_dc *dc, int win_idx, int operation);
void tegra_dc_feature_register(struct tegra_dc *dc);

static inline bool win_use_v_filter(struct tegra_dc *dc,
	const struct tegra_dc_win *win)
{
	return tegra_dc_feature_has_filter(dc, win->idx, HAS_V_FILTER) &&
		(win->flags & TEGRA_WIN_FLAG_SCAN_COLUMN ?
			win->w.full != dfixed_const(win->out_h)
			: win->h.full != dfixed_const(win->out_h));
}

static inline bool win_use_h_filter(struct tegra_dc *dc,
	const struct tegra_dc_win *win)
{
	return tegra_dc_feature_has_filter(dc, win->idx, HAS_H_FILTER) &&
		(win->flags & TEGRA_WIN_FLAG_SCAN_COLUMN ?
			win->h.full != dfixed_const(win->out_w)
			: win->w.full != dfixed_const(win->out_w));
}
#endif
