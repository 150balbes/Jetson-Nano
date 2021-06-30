/*
 * edid_display_id_ext.h: Definitions for E-EDID extension blocks encoded
 *                        in display ID format
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __TEGRA_EDID_DISP_ID_EXT_H__
#define __TEGRA_EDID_DISP_ID_EXT_H__

#include <linux/types.h>

/* Different types of Data blocks */
#define DISP_ID_BLOCK_TYPE_PRODUCT_IDENTITY  0  /* Product Identification */
#define DISP_ID_BLOCK_TYPE_DISPLAY_PARAM     1  /* Display Parameters */
#define DISP_ID_BLOCK_TYPE_COLOR_CHAR        2  /* Color Characteristics */
#define DISP_ID_BLOCK_TYPE_TIMING1           3  /* Type 1 Detailed Timing */
#define DISP_ID_BLOCK_TYPE_TIMING2           4  /* Type 2 Detailed Timing */
#define DISP_ID_BLOCK_TYPE_TIMING3           5  /* Type 3 Short Timing */
#define DISP_ID_BLOCK_TYPE_TIMING4           6  /* Type 4 DMT ID Timing */
#define DISP_ID_BLOCK_TYPE_TIMING_VESA       7  /* VESA Standard Timing */
#define DISP_ID_BLOCK_TYPE_TIMING_CEA        8  /* CEA Standard Timing */
#define DISP_ID_BLOCK_TYPE_RANGE_LIMITS      9  /* Video Timing Range Limits */
#define DISP_ID_BLOCK_TYPE_SERIAL_NUMBER     10 /* Product Serial Number */
#define DISP_ID_BLOCK_TYPE_ASCII_STRING      11 /* General Purpose ASCII Str */
#define DISP_ID_BLOCK_TYPE_DEVICE_DATA       12 /* Display Device Data */
#define DISP_ID_BLOCK_TYPE_INTERFACE_POWER   13 /* Interface Power Sequencing */
#define DISP_ID_BLOCK_TYPE_TRANSFER_CHAR     14 /* Transfer Characteristics */
#define DISP_ID_BLOCK_TYPE_DISPLAY_INTERFACE 15 /* Display Interface Data */
#define DISP_ID_BLOCK_TYPE_STEREO            16 /* Stereo Data */
#define DISP_ID_BLOCK_TYPE_TIMING5           17 /* Type V Timing Short Desc */
#define DISP_ID_BLOCK_TYPE_TILEDDISPLAY      18 /* Tiled Display Data */
#define DISP_ID_BLOCK_TYPE_DISPLAY_INTERFACE_FEATURES 0X26 /* DisplayID2.0 Display Interface Features Data */
#define DISP_ID_BLOCK_TYPE_CEA_DATA        0x81 /* DIsplay ID data */
#define DISP_ID_BLOCK_TYPE_VENDOR_SPEC     0x7F /* Vendor Specific Data */

#define DISP_ID_SECTION_MAX_SIZE 251

/*
 * disp_id_section  - The "section" of a display ID extension block, comprising
 *                    of all but the 1st byte of the extension block.
 * @version         - Display ID version.
 * @bytes           - Number of payload bytes. Value of this field ranges from
 *                    0 to 251, since the 5 mandatory bytes of the section are
 *                    not accounted for here.
 * @product_type    - Type of sink device. This field is unused.
 * @extension_count - Value is always 0 for extension blocks.
 * @data            - Start of the section's data blocks.
 */
struct disp_id_section {
	u8 version;
	u8 bytes;
	u8 product_type;
	u8 extension_count;
	u8 data[DISP_ID_SECTION_MAX_SIZE];
};

/*
 * disp_id_data_block_header  - Metadata of a display ID data block
 * @tag      - Identifies the type of data block. See DISP_ID_BLOCK_TYPE_*
 * @revision - Version of this data structure
 * @bytes    - Number of payload bytes [0, 248]
 */
struct disp_id_data_block_header {
	u8 tag;
	u8 revision;
	u8 bytes;
};

#define DISP_ID_DATA_BLOCK_MAX_BYTES \
	(DISP_ID_SECTION_MAX_SIZE - sizeof(struct disp_id_data_block_header))

/* Aspect ratios reported within Timing Data Block */
#define DISP_ID_TIMING_ASPECT_RATIO_1_1   0
#define DISP_ID_TIMING_ASPECT_RATIO_5_4   1
#define DISP_ID_TIMING_ASPECT_RATIO_4_3   2
#define DISP_ID_TIMING_ASPECT_RATIO_15_9  3
#define DISP_ID_TIMING_ASPECT_RATIO_16_9  4
#define DISP_ID_TIMING_ASPECT_RATIO_16_10 5

/*
 * disp_id_timing1_desc - Makes up the payload bytes of a data block with tag
 *                        value DISP_ID_BLOCK_TYPE_TIMING1. The fields are
 *                        self explanatory
 */
struct disp_id_timing1_desc {
	u8 pclk_lo;
	u8 pclk_mid;
	u8 pclk_hi;

	struct {
		u8 aspect_ratio			: 3;
		u8 rsvd				: 1;
		u8 interlaced			: 1;
		u8 stereo_support		: 2;
		u8 is_preferred_detailed_timing	: 1;
	} options;

	struct {
		u8 active_lo;
		u8 active_hi;
		u8 blank_lo;
		u8 blank_hi;
		u8 front_porch_lo;
		u8 front_porch_hi		: 7;
		u8 sync_polarity		: 1;
		u8 sync_width_lo;
		u8 sync_width_hi;
	} hori;

	struct {
		u8 active_lo;
		u8 active_hi;
		u8 blank_lo;
		u8 blank_hi;
		u8 front_porch_lo;
		u8 front_porch_hi		: 7;
		u8 sync_polarity		: 1;
		u8 sync_width_lo;
		u8 sync_width_hi;
	} vert;
};

#define DISP_ID_TIMING1_MAX_DESC \
	(DISP_ID_DATA_BLOCK_MAX_BYTES / sizeof(struct disp_id_timing1_desc))

struct disp_id_timing1_block {
	struct disp_id_data_block_header header;
	struct disp_id_timing1_desc descriptors[DISP_ID_TIMING1_MAX_DESC];
};

#endif /* __TEGRA_EDID_DISP_ID_EXT_H__ */
