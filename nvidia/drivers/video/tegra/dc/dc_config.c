/*
 * dc_config.c: Functions required for dc configuration.
 *
 * Copyright (c) 2012-2020, NVIDIA CORPORATION, All rights reserved.
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

#include "dc_config.h"

static struct tegra_dc_feature_entry t210_feature_entries_a[] = {
	{ 0, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 0, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 0, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 0, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 0, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 0, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 0, TEGRA_DC_FEATURE_COMPRESSION_FORMATS,
		{ TEGRA_WIN_FMT_COMPRESSION_T210_LOW,
		TEGRA_WIN_FMT_COMPRESSION_T210_HIGH } },
	/* dispA:windowA can rotate the planar formats */
	{ 0, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 2304, 1} },
	{ 0, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 0, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 1, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 1, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 1, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 1, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 1, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 1, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 1, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	/* cannot rotate planar format */
	{ 1, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 1, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T210_LOW,
			TEGRA_WIN_FMT_ROTATION_T210_HIGH } },
	{ 1, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 2, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 2, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 2, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 2, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 2, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 1} },
	{ 2, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 2, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 2, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 2, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T210_LOW,
			TEGRA_WIN_FMT_ROTATION_T210_HIGH } },
	{ 2, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 3, TEGRA_DC_FEATURE_FORMATS, { TEGRA_WIN_FMT_SIMPLE_T210_LOW,
		TEGRA_WIN_FMT_SIMPLE_T210_HIGH, } },
	{ 3, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 3, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 3, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {1, 1, 1, 1} },
	{ 3, TEGRA_DC_FEATURE_FILTER_TYPE, {0, 0} },
	{ 3, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 0} },
	{ 3, TEGRA_DC_FEATURE_INVERT_TYPE, {0, 0, 0} },
	{ 3, TEGRA_DC_FEATURE_FIELD_TYPE, {0} },
	{ 3, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 3, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	/* cannot rotate any formats */
	{ 3, TEGRA_DC_FEATURE_ROTATION_FORMATS, { 0, 0 } },
	{ 3, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },
};

static struct tegra_dc_feature_entry t210_feature_entries_b[] = {
	{ 0, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 0, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 0, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 0, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 0, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 0, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 0} },
	{ 0, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 0, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	/* cannot rotate planar format */
	{ 0, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 0, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T210_LOW,
			TEGRA_WIN_FMT_ROTATION_T210_HIGH } },
	{ 0, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 1, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 1, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 1, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 1, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 1, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 1, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 0} },
	{ 1, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 1, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 1, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 1, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T210_LOW,
			TEGRA_WIN_FMT_ROTATION_T210_HIGH } },
	{ 1, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },

	{ 2, TEGRA_DC_FEATURE_FORMATS,
			{ TEGRA_WIN_FMT_T210_LOW, TEGRA_WIN_FMT_T210_HIGH } },
	{ 2, TEGRA_DC_FEATURE_BLEND_TYPE, {2} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SIZE, {4096, 1, 4096, 1} },
	{ 2, TEGRA_DC_FEATURE_MAXIMUM_SCALE, {2, 2, 2, 2} },
	{ 2, TEGRA_DC_FEATURE_FILTER_TYPE, {1, 1} },
	{ 2, TEGRA_DC_FEATURE_LAYOUT_TYPE, {1, 0, 1} },
	{ 2, TEGRA_DC_FEATURE_INVERT_TYPE, {1, 1, 0} },
	{ 2, TEGRA_DC_FEATURE_FIELD_TYPE, {1} },
	{ 2, TEGRA_DC_FEATURE_COMPRESSION_FORMATS, {0, 0} },
	{ 2, TEGRA_DC_FEATURE_PLANAR_ROTATION_MAXIMUM_SIZE, {0, 0, 0, 0} },
	{ 2, TEGRA_DC_FEATURE_ROTATION_FORMATS,
			{ TEGRA_WIN_FMT_ROTATION_T210_LOW,
			TEGRA_WIN_FMT_ROTATION_T210_HIGH } },
	{ 2, TEGRA_DC_FEATURE_PACKED_ROTATION_MAXIMUM_SIZE,
		{4096, 1, 4096, 1} },
};

static struct tegra_dc_feature t210_feature_table_a = {
	ARRAY_SIZE(t210_feature_entries_a), t210_feature_entries_a,
};

static struct tegra_dc_feature t210_feature_table_b = {
	ARRAY_SIZE(t210_feature_entries_b), t210_feature_entries_b,
};

int tegra_dc_get_feature(struct tegra_dc_feature *feature, int win_idx,
					enum tegra_dc_feature_option option)
{
	int i;
	struct tegra_dc_feature_entry *entry;

	if (!feature)
		return -EINVAL;

	for (i = 0; i < feature->num_entries; i++) {
		entry = &feature->entries[i];
		if (entry->window_index == win_idx && entry->option == option)
			return i;
	}

	return -EINVAL;
}

u32 *tegra_dc_parse_feature(struct tegra_dc *dc, int win_idx, int operation)
{
	int idx;
	struct tegra_dc_feature_entry *entry;
	enum tegra_dc_feature_option option;
	struct tegra_dc_feature *feature = dc->feature;

	switch (operation) {
	case GET_WIN_FORMATS:
		option = TEGRA_DC_FEATURE_FORMATS;
		break;
	case GET_WIN_SIZE:
		option = TEGRA_DC_FEATURE_MAXIMUM_SIZE;
		break;
	case GET_INVERT:
		option = TEGRA_DC_FEATURE_INVERT_TYPE;
		break;
	case HAS_SCALE:
		option = TEGRA_DC_FEATURE_MAXIMUM_SCALE;
		break;
	case HAS_TILED:
		option = TEGRA_DC_FEATURE_LAYOUT_TYPE;
		break;
	case HAS_V_FILTER:
		option = TEGRA_DC_FEATURE_FILTER_TYPE;
		break;
	case HAS_H_FILTER:
		option = TEGRA_DC_FEATURE_FILTER_TYPE;
		break;
	case HAS_GEN2_BLEND:
		option = TEGRA_DC_FEATURE_BLEND_TYPE;
		break;
	case HAS_BLOCKLINEAR:
		option = TEGRA_DC_FEATURE_LAYOUT_TYPE;
		break;
	case HAS_INTERLACE:
		option = TEGRA_DC_FEATURE_FIELD_TYPE;
		break;
	default:
		return NULL;
	}

	idx = tegra_dc_get_feature(feature, win_idx, option);
	if (idx < 0)
		return NULL;
	entry = &feature->entries[idx];

	return entry->arg;
}
EXPORT_SYMBOL(tegra_dc_parse_feature);

int tegra_dc_feature_has_scaling(struct tegra_dc *dc, int win_idx)
{
	int i;
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, HAS_SCALE);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	for (i = 0; i < WIN_FEATURE_ENTRY_SIZE; i++)
		if (addr[i] != 1)
			return 1;
	return 0;
}

int tegra_dc_feature_has_tiling(struct tegra_dc *dc, int win_idx)
{
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, HAS_TILED);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	return addr[TILED_LAYOUT];
}

int tegra_dc_feature_has_blocklinear(struct tegra_dc *dc, int win_idx)
{
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, HAS_BLOCKLINEAR);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	return addr[BLOCK_LINEAR];
}

int tegra_dc_feature_has_interlace(struct tegra_dc *dc, int win_idx)
{
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, HAS_INTERLACE);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	return addr[INTERLACE];
}

int tegra_dc_feature_has_filter(struct tegra_dc *dc, int win_idx, int operation)
{
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, operation);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	if (operation == HAS_V_FILTER)
		return addr[V_FILTER];
	else
		return addr[H_FILTER];
}

int tegra_dc_feature_is_gen2_blender(struct tegra_dc *dc, int win_idx)
{
	u32 *addr = tegra_dc_parse_feature(dc, win_idx, HAS_GEN2_BLEND);

	if (WARN_ONCE(!addr, "window does not exist"))
		return 0;

	if (addr[BLEND_GENERATION] == 2)
		return 1;

	return 0;
}

/* supports 90 and 270 rotation? */
int tegra_dc_feature_has_scan_column(struct tegra_dc *dc, int win_idx)
{
	int idx;
	struct tegra_dc_feature_entry *entry;

	if (WARN_ONCE(!dc, "display is NULL"))
		return 0;

	idx = tegra_dc_get_feature(dc->feature, win_idx,
		TEGRA_DC_FEATURE_INVERT_TYPE);
	if (idx < 0)
		return 0;

	entry = &dc->feature->entries[idx];
	return entry->arg[SCAN_COLUMN]; /* 1 = supported, 0 = not supported */
}

void tegra_dc_feature_register(struct tegra_dc *dc)
{
	int i;
	struct tegra_dc_feature_entry *entry;

	if (tegra_dc_is_nvdisplay()) {
		nvdisp_dc_feature_register(dc);
	} else {
		if (!dc->ndev->id)
			dc->feature = &t210_feature_table_a;
		else
			dc->feature = &t210_feature_table_b;
	}

	/* Count the number of windows using gen1 blender. */
	dc->gen1_blend_num = 0;
	for (i = 0; i < dc->feature->num_entries; i++) {
		entry = &dc->feature->entries[i];
		if (entry->option == TEGRA_DC_FEATURE_BLEND_TYPE &&
					entry->arg[BLEND_GENERATION] == 1)
			dc->gen1_blend_num++;
	}
}
