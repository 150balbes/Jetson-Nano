/*
 * edid_display_id_ext.c: Functions to parse E-EDID extension blocks encoded
 *                        in display ID format
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION, All rights reserved.
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

static inline u16 combine_hi_lo(unsigned int hi, unsigned int lo)
{
	return (hi << 8) | lo;
}

static inline u32 combine_hi_mid_lo(unsigned int hi, unsigned int mid,
				    unsigned int lo)
{
	return (hi << 16) | (mid << 8) | lo;
}

/* NOTE: This function replaces the modedb residing in specs in place */
static int disp_id_add_modes(struct fb_videomode *new_modes,
			     unsigned int num_modes_new,
			     struct fb_monspecs *specs)
{
	unsigned int num_modes_current, num_modes_total;
	struct fb_videomode *new_modedb;

	num_modes_current = specs->modedb_len;
	num_modes_total = num_modes_current + num_modes_new;

	new_modedb = kcalloc(num_modes_total, sizeof(*new_modes), GFP_KERNEL);
	if (!new_modedb)
		return -ENOMEM;

	memcpy(new_modedb, specs->modedb,
	       sizeof(*new_modes) * num_modes_current);
	memcpy(new_modedb + num_modes_current, new_modes,
	       sizeof(*new_modes) * num_modes_new);

	kfree(specs->modedb);
	specs->modedb = new_modedb;
	specs->modedb_len = num_modes_total;

	return 0;
}

static void disp_id_timing1_fill_fb_mode(const struct disp_id_timing1_desc *t,
					 struct fb_videomode *mode)
{
	u32 hblank, vblank;

	/* Value in EDID is in units of '10 thousand pixels per second' */
	mode->pixclock = 1 + combine_hi_mid_lo(t->pclk_hi, t->pclk_mid,
					       t->pclk_lo);
	mode->pixclock *= 10000; /* convert to Hz */
#if defined(CONFIG_FB_MODE_PIXCLOCK_HZ)
	mode->pixclock_hz = mode->pixclock;
#endif

	/*
	 * All resolutions are manually offset by 1 as per the format
	 * Hence need to add 1 to each of them
	 */
	mode->xres = 1 + combine_hi_lo(t->hori.active_hi, t->hori.active_lo);
	mode->yres = 1 + combine_hi_lo(t->vert.active_hi, t->vert.active_lo);

	hblank = 1 + combine_hi_lo(t->hori.blank_hi, t->hori.blank_lo);
	vblank = 1 + combine_hi_lo(t->vert.blank_hi, t->vert.blank_lo);

	mode->refresh = mode->pixclock / ((mode->xres + hblank) *
						  (mode->yres + vblank));
	/* Convert to units used by fb driver */
	mode->pixclock = KHZ2PICOS(mode->pixclock / 1000);

	mode->right_margin = 1 + combine_hi_lo(t->hori.front_porch_hi,
					       t->hori.front_porch_lo);
	mode->lower_margin = 1 + combine_hi_lo(t->vert.front_porch_hi,
					       t->vert.front_porch_lo);

	mode->hsync_len = 1 + combine_hi_lo(t->hori.sync_width_hi,
					    t->hori.sync_width_lo);
	mode->vsync_len = 1 + combine_hi_lo(t->vert.sync_width_hi,
					    t->vert.sync_width_lo);

	mode->left_margin = hblank - mode->right_margin - mode->hsync_len;
	mode->upper_margin = vblank - mode->lower_margin - mode->vsync_len;

	if (t->hori.sync_polarity)
		mode->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (t->vert.sync_polarity)
		mode->sync |= FB_SYNC_VERT_HIGH_ACT;

	if (t->options.interlaced) {
		mode->yres *= 2;
		mode->upper_margin *= 2;
		mode->lower_margin *= 2;
		mode->vsync_len *= 2;
		mode->vmode |= FB_VMODE_INTERLACED;
	}

	mode->flag = FB_MODE_IS_DETAILED;
	mode->vmode |= FB_VMODE_IS_DETAILED;

	switch (t->options.aspect_ratio) {
	case DISP_ID_TIMING_ASPECT_RATIO_4_3:
		mode->flag |= FB_FLAG_RATIO_4_3;
		break;
	case DISP_ID_TIMING_ASPECT_RATIO_16_9:
		mode->flag |= FB_FLAG_RATIO_16_9;
		break;
	default:
		break;
	}
}

static int disp_id_timing1_parse(const u8 *data, struct fb_monspecs *specs)
{
	const struct disp_id_timing1_block *timing1_block;
	const struct disp_id_timing1_desc *timings;
	struct fb_videomode *new_modes;
	unsigned int i, num_timings;
	int ret = 0;

	timing1_block = (const struct disp_id_timing1_block *)data;
	num_timings = timing1_block->header.bytes / sizeof(*timings);
	timings = timing1_block->descriptors;

	new_modes = kcalloc(num_timings, sizeof(*new_modes), GFP_KERNEL);
	if (!new_modes)
		return -ENOMEM;

	for (i = 0; i < num_timings; i++)
		disp_id_timing1_fill_fb_mode(&timings[i], &new_modes[i]);

	ret = disp_id_add_modes(new_modes, num_timings, specs);
	kfree(new_modes);

	return ret;
}

int tegra_edid_disp_id_ext_block_parse(const u8 *ext_block,
				       struct fb_monspecs *specs,
				       struct tegra_edid_pvt *edid_pvt)
{
	const struct disp_id_section *section;
	const struct disp_id_data_block_header *data_hdr;
	u8 section_bytes, data_block_bytes;
	const u8 *data_block; /* A raw pointer for easier pointer arithmetic */
	int ret;

	/* As per display ID terminology, the section starts at offset 1 in the
	 * extension block, after the tag 0x70
	 */
	section = (const struct disp_id_section *)(ext_block + 1);
	section_bytes = section->bytes;
	data_block = (u8 *)&section->data; /* Points to 1st data block */

	/* Parse all data blocks in the section, one per loop iteration */
	while (data_block < ext_block + section_bytes) {
		data_hdr = (const struct disp_id_data_block_header *)data_block;

		data_block_bytes = data_hdr->bytes;

		switch (data_hdr->tag) {
		case DISP_ID_BLOCK_TYPE_TIMING1:
			ret = disp_id_timing1_parse(data_block, specs);
			if (ret)
				return ret;
			break;
		default:
			break;
		}

		data_block += sizeof(*data_hdr) + data_block_bytes;
	}

	return 0;
}
