/*
 * NVIDIA Tegra Video Input Device Driver Core Helpers
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Bryan Wu <pengw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <media/mc_common.h>
#include <linux/nospec.h>

static const struct tegra_video_format tegra_default_format[] = {
	{
		TEGRA_VF_DEF,
		10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		{2, 1},
		TEGRA_IMAGE_FORMAT_DEF,
		TEGRA_IMAGE_DT_RAW10,
		V4L2_PIX_FMT_SRGGB10,
		"RGRG.. GBGB..",
	},
};

/* -----------------------------------------------------------------------------
 * Helper functions
 */

/**
 * tegra_core_get_fourcc_by_idx - get fourcc of a tegra_video format
 * @index: array index of the tegra_video_formats
 *
 * Return: fourcc code
 */
u32 tegra_core_get_fourcc_by_idx(struct tegra_channel *chan,
				unsigned int index)
{
	/* return default fourcc format if the index out of bounds */
	if (index > (chan->num_video_formats - 1))
		return V4L2_PIX_FMT_SGRBG10;

	index = array_index_nospec(index, chan->num_video_formats);

	return chan->video_formats[index]->fourcc;
}

/**
 * tegra_core_get_word_count - Calculate word count
 * @frame_width: number of pixels per line
 * @fmt: Tegra Video format struct which has BPP information
 *
 * Return: word count number
 */
u32 tegra_core_get_word_count(unsigned int frame_width,
			      const struct tegra_video_format *fmt)
{
	return frame_width * fmt->width / 8;
}

/**
 * tegra_core_get_idx_by_code - Retrieve index for a media bus code
 * @code: the format media bus code
 *
 * Return: a index to the format information structure corresponding to the
 * given V4L2 media bus format @code, or -1 if no corresponding format can
 * be found.
 */
int tegra_core_get_idx_by_code(struct tegra_channel *chan,
		unsigned int code, unsigned offset)
{
	unsigned int i;

	for (i = offset; i < chan->num_video_formats; ++i) {
		if (chan->video_formats[i]->code == code)
			return i;
	}

	return -1;
}

/**
 * tegra_core_get_code_by_fourcc - Retrieve media bus code for fourcc
 * @fourcc: the format 4CC
 *
 * Return: media bus code format information structure corresponding to the
 * given V4L2 fourcc @fourcc, or -1 if no corresponding format found.
 */
int tegra_core_get_code_by_fourcc(struct tegra_channel *chan,
		unsigned int fourcc, unsigned int offset)
{
	unsigned int i;

	for (i = offset; i < chan->num_video_formats; ++i) {
		if (chan->video_formats[i]->fourcc == fourcc)
			return chan->video_formats[i]->code;
	}
	speculation_barrier(); /* break_spec_p#5_1 */

	return -1;
}

/**
 * tegra_core_get_default_format - Get default format
 *
 * Return: pointer to the format where the default format needs
 * to be filled in.
 */
const struct tegra_video_format *tegra_core_get_default_format(void)
{
	return &tegra_default_format[0];
}

/**
 * tegra_core_get_format_by_code - Retrieve format information for a media
 * bus code
 * @code: the format media bus code
 *
 * Return: a pointer to the format information structure corresponding to the
 * given V4L2 media bus format @code, or NULL if no corresponding format can
 * be found.
 */
const struct tegra_video_format *
tegra_core_get_format_by_code(struct tegra_channel *chan,
		unsigned int code, unsigned offset)
{
	unsigned int i;

	for (i = offset; i < chan->num_video_formats; ++i) {
		if (chan->video_formats[i]->code == code)
			return chan->video_formats[i];
	}
	speculation_barrier(); /* break_spec_p#5_1 */

	return NULL;
}

/**
 * tegra_core_get_format_by_fourcc - Retrieve format information for a 4CC
 * @fourcc: the format 4CC
 *
 * Return: a pointer to the format information structure corresponding to the
 * given V4L2 format @fourcc, or NULL if no corresponding format can be
 * found.
 */
const struct tegra_video_format *
tegra_core_get_format_by_fourcc(struct tegra_channel *chan, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < chan->num_video_formats; ++i) {
		if (chan->video_formats[i]->fourcc == fourcc)
			return chan->video_formats[i];
	}
	speculation_barrier(); /* break_spec_p#5_1 */

	return NULL;
}

/**
 * tegra_core_bytes_per_line - Calculate bytes per line in one frame
 * @width: frame width
 * @align: number of alignment bytes
 * @fmt: Tegra Video format
 *
 * Simply calcualte the bytes_per_line and if it's not aligned it
 * will be padded to alignment boundary.
 */
u32 tegra_core_bytes_per_line(unsigned int width, unsigned int align,
			      const struct tegra_video_format *fmt)
{
	u32 value = ((width * fmt->bpp.numerator) / fmt->bpp.denominator);

	return roundup(value, align);
}
