/*
 * include/uapi/video/tegra_dc_ext.h
 *
 * tegra_dc_ext.h: tegra dc ext interface.
 *
 * Copyright (C) 2016-2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Author: Robert Morell <rmorell@nvidia.com>
 * Some code based on fbdev extensions written by:
 *	Erik Gilling <konkers@android.com>
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
 */

#ifndef __UAPI_TEGRA_DC_EXT_H
#define __UAPI_TEGRA_DC_EXT_H

#include <linux/types.h>
#include <linux/ioctl.h>
#if defined(__KERNEL__)
# include <linux/time.h>
#else
# include <time.h>
# include <unistd.h>
#endif

/* Note: These are the actual values written to the DC_WIN_COLOR_DEPTH register
 * and may change in new tegra architectures.
 */
/* New naming for pixel format*/
#define TEGRA_DC_EXT_FMT_T_P8						(3)
#define TEGRA_DC_EXT_FMT_T_A4R4G4B4					(4)
#define TEGRA_DC_EXT_FMT_T_A1R5G5B5					(5)
#define TEGRA_DC_EXT_FMT_T_R5G6B5					(6)
#define TEGRA_DC_EXT_FMT_T_R5G5B5A1					(7)
#define TEGRA_DC_EXT_FMT_T_R4G4B4A4					(8)
#define TEGRA_DC_EXT_FMT_T_A8R8G8B8					(12)
#define TEGRA_DC_EXT_FMT_T_A8B8G8R8					(13)
#define TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8				(16)
#define TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8_TRUE		(17)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420		(18)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420_TRUE	(19)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422		(20)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422_TRUE	(21)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R		(22)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R_TRUE	(23)
#define TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8				(24)
#define TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE		(25)
#define TEGRA_DC_EXT_FMT_T_A4B4G4R4					(27)
#define TEGRA_DC_EXT_FMT_T_A1B5G5R5					(28)
#define TEGRA_DC_EXT_FMT_T_B5G5R5A1					(29)
#define TEGRA_DC_EXT_FMT_T_X8R8G8B8					(37)
#define TEGRA_DC_EXT_FMT_T_X8B8G8R8					(38)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444		(41)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420			(42)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420			(43)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422			(44)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422			(45)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R			(46)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422R			(47)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444			(48)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444			(49)
#define TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE	(52)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420_TRUE		(53)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420_TRUE		(54)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422_TRUE		(55)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422_TRUE		(56)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R_TRUE		(57)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422R_TRUE		(58)
#define TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444_TRUE		(59)
#define TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444_TRUE		(60)
#define TEGRA_DC_EXT_FMT_T_Y8_U8__Y8_V8				(61)
#define TEGRA_DC_EXT_FMT_T_A2R10G10B10				(70)
#define TEGRA_DC_EXT_FMT_T_A2B10G10R10				(71)
#define TEGRA_DC_EXT_FMT_T_X2BL10GL10RL10_XRBIAS	(72)
#define TEGRA_DC_EXT_FMT_T_X2BL10GL10RL10_XVYCC		(73)
#define TEGRA_DC_EXT_FMT_T_R16_G16_B16_A16_NVBIAS	(74)
#define TEGRA_DC_EXT_FMT_T_R16_G16_B16_A16			(75)
#define TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N420		(80)
#define TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N444		(82)
#define TEGRA_DC_EXT_FMT_T_Y10___V10U10_N420		(83)
#define TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422		(84)
#define TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422R		(86)
#define TEGRA_DC_EXT_FMT_T_Y10___U10V10_N444		(88)
#define TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420		(96)
#define TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444		(98)
#define TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420		(99)
#define TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422		(100)
#define TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R		(102)
#define TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444		(104)
/* Additional Formats: These values are not intended to be
 * programmed directly into the DC_WIN_COLOR_DEPTH register.
 * They only signal formats possible through additional
 * parameters (mainly through the UV swap control).
 */
#define TEGRA_DC_EXT_FMT_T_Y10___U10V10_N420		(256)
#define TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422		(257)
#define TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422R		(258)
#define TEGRA_DC_EXT_FMT_T_Y10___V10U10_N444		(259)
#define TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420		(260)
#define TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422		(261)
#define TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R		(262)
#define TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444		(263)
#define TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N420		(264)
#define TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N444		(265)
#define TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420		(266)
#define TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444		(267)

/* color format type field is 8-bits */
#define TEGRA_DC_EXT_FMT_SHIFT		0
#define TEGRA_DC_EXT_FMT_MASK		(0xfff << TEGRA_DC_EXT_FMT_SHIFT)

/* pixformat - byte order options ( w x y z ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_NOSWAP	(0 << 16) /* ( 3 2 1 0 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP2	(1 << 16) /* ( 2 3 0 1 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP4	(2 << 16) /* ( 0 1 2 3 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP4HW	(3 << 16) /* ( 1 0 3 2 ) */
/* the next two are not available on T30 or earlier */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAP02	(4 << 16) /* ( 3 0 1 2 ) */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SWAPLEFT	(5 << 16) /* ( 2 1 0 3 ) */
/* byte order field is 4-bits */
#define TEGRA_DC_EXT_FMT_BYTEORDER_SHIFT	16
#define TEGRA_DC_EXT_FMT_BYTEORDER_MASK		\
		(0x0f << TEGRA_DC_EXT_FMT_BYTEORDER_SHIFT)

#define TEGRA_DC_EXT_BLEND_NONE		0
#define TEGRA_DC_EXT_BLEND_PREMULT	1
#define TEGRA_DC_EXT_BLEND_COVERAGE	2
#define TEGRA_DC_EXT_BLEND_ADD		3

#define TEGRA_DC_EXT_FLIP_FLAG_INVERT_H		(1 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_INVERT_V		(1 << 1)
#define TEGRA_DC_EXT_FLIP_FLAG_TILED		(1 << 2)
#define TEGRA_DC_EXT_FLIP_FLAG_CURSOR		(1 << 3)
#define TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA	(1 << 4)
#define TEGRA_DC_EXT_FLIP_FLAG_BLOCKLINEAR	(1 << 5)
#define TEGRA_DC_EXT_FLIP_FLAG_SCAN_COLUMN	(1 << 6)
#define TEGRA_DC_EXT_FLIP_FLAG_INTERLACE	(1 << 7)
#define TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED	(1 << 8)
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC	(1 << 9)
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_NVDISP_WIN_CSC	(1 << 10)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_MASK	(3 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_FULL	(0 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_LIMITED	(1 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_INPUT_RANGE_BYPASS	(2 << 11)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_MASK		(7 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_NONE		(0 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC601	(1 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC709	(2 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020	(4 << 13)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_MASK	(15 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_DEFAULT	(0 << 16) /* driver selects */
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_NONE	(1 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_SRGB	(2 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_8_10	(4 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_DEGAMMA_YUV_12	(8 << 16)
#define TEGRA_DC_EXT_FLIP_FLAG_COLOR_EXPAND_DEFAULT	(0 << 20)
#define TEGRA_DC_EXT_FLIP_FLAG_COLOR_EXPAND_ENABLE	(0 << 20)
#define TEGRA_DC_EXT_FLIP_FLAG_COLOR_EXPAND_DISABLE	(1 << 20)
#define TEGRA_DC_EXT_FLIP_FLAG_COLOR_EXPAND_UPDATE	(1 << 21)
#define TEGRA_DC_EXT_FLIP_FLAG_CLAMP_BEFORE_BLEND_DEFAULT	(0 << 22)
#define TEGRA_DC_EXT_FLIP_FLAG_CLAMP_BEFORE_BLEND_ENABLE	(0 << 22)
#define TEGRA_DC_EXT_FLIP_FLAG_CLAMP_BEFORE_BLEND_DISABLE	(1 << 22)
/*End of window specific flip flags*/
/*Passthrough condition for running 4K HDMI*/
#define TEGRA_DC_EXT_FLIP_HEAD_FLAG_YUVBYPASS	(1 << 0)
#define TEGRA_DC_EXT_FLIP_HEAD_FLAG_VRR_MODE	(1 << 1)
/* Flag for HDR_DATA handling */
#define TEGRA_DC_EXT_FLIP_FLAG_HDR_ENABLE	(1 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_HDR_DATA_UPDATED (1 << 1)
/*
 * Following flag is used when TEGRA_DC_EXT_FLIP_USER_DATA_NVDISP_CMU is used.
 * FLAG_NVDISP_CMU_UPDATE: this flag is valid only when
 *                      tegra_dc_ext_nvdisp_cmu.cmu_enable = true.
 *                      If flag is present, driver will update LUT
 *                      values provided from userspace.
 *                      If flag is not present then driver will program LUT
 *                      from cached values.
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_NVDISP_CMU	(1 << 0)
/* FLAG_UPDATE_OCSC_CS: If flag is present, driver will update
 *                      output colorspace with values provided from userspace
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_CS	(1 << 0)
/* FLAG_UPDATE_OCSC_RANGE: If flag is present, driver will update output color
 *                      range with values provided from userspace
 */
#define TEGRA_DC_EXT_FLIP_FLAG_UPDATE_OCSC_RANGE	(1 << 1)
/* Flags for post-syncpt handling */
/* Bits 1:0 are reserved for the post-syncpt type */
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_SHIFT	0
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_MASK \
			(0x3 << TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_TYPE_SHIFT)
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_FD	(0 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_RAW	(1 << 0)

/*
 * Temporary flags for determining which IMP structs to use
 *
 * These are the legacy v1 structs:
 * - tegra_dc_ext_imp_head_results
 * - tegra_dc_ext_imp_settings
 *
 * These are the new v2 structs:
 * - tegra_dc_ext_nvdisp_imp_win_entries
 * - tegra_dc_ext_nvdisp_imp_win_settings
 * - tegra_dc_ext_nvdisp_imp_head_entries
 * - tegra_dc_ext_nvdisp_imp_head_settings
 * - tegra_dc_ext_nvdisp_imp_global_entries
 * - tegra_dc_ext_nvdisp_imp_global_settings
 * - tegra_dc_ext_nvdisp_imp_settings
 */
#define TEGRA_DC_EXT_FLIP_FLAG_IMP_V1	(0 << 0)
#define TEGRA_DC_EXT_FLIP_FLAG_IMP_V2	(1 << 0)

/* Flags for CEA861.3 defined eotfs in HDR Metadata form sink */
#define TEGRA_DC_EXT_CEA861_3_EOTF_SDR_LR	(1 << 0)
#define TEGRA_DC_EXT_CEA861_3_EOTF_HDR_LR	(1 << 1)
#define TEGRA_DC_EXT_CEA861_3_EOTF_SMPTE_2084	(1 << 2)

struct tegra_timespec {
	__s32	tv_sec; /* seconds */
	__s32	tv_nsec; /* nanoseconds */
};

/*
 * Keeping the old struct to maintain the app compatibility.
 *
 */
struct tegra_dc_ext_flip_windowattr {
	__s32	index;
	__u32	buff_id;
	__u32	blend;
	__u32	offset;
	__u32	offset_u;
	__u32	offset_v;
	__u32	stride;
	__u32	stride_uv;
	__u32	pixformat;
	/*
	 * x, y, w, h are fixed-point: 20 bits of integer (MSB) and 12 bits of
	 * fractional (LSB)
	 */
	__u32	x;
	__u32	y;
	__u32	w;
	__u32	h;
	__u32	out_x;
	__u32	out_y;
	__u32	out_w;
	__u32	out_h;
	__u32	z;
	__u32	swap_interval;
	struct tegra_timespec timestamp;
	union {
		struct {
			__u32 pre_syncpt_id;
			__u32 pre_syncpt_val;
		};
		__s32 pre_syncpt_fd;
	};
	/* These two are optional; if zero, U and V are taken from buff_id */
	__u32	buff_id_u;
	__u32	buff_id_v;
	__u32	flags;
	__u8	global_alpha; /* requires TEGRA_DC_EXT_FLIP_FLAG_GLOBAL_ALPHA */
	/* log2(blockheight) for blocklinear format */
	__u8	block_height_log2;
	__u8	pad1[2];
	union { /* fields for mutually exclusive options */
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_INTERLACE set */
			__u32	offset2;
			__u32	offset_u2;
			__u32	offset_v2;
			/* Leave some wiggle room for future expansion */
			__u32   pad2[1];
		};
		struct { /* used if TEGRA_DC_EXT_FLIP_FLAG_COMPRESSED set */
			__u32	buff_id; /* take from buff_id if zero */
			__u32	offset; /* added to base */
			__u16	offset_x;
			__u16	offset_y;
			__u32	zbc_color;
		} cde;
		struct { /* TEGRA_DC_EXT_FLIP_FLAG_UPDATE_CSC */
			__u16 yof;	/* s.7.0 */
			__u16 kyrgb;	/*   2.8 */
			__u16 kur;	/* s.2.8 */
			__u16 kvr;	/* s.2.8 */
			__u16 kug;	/* s.1.8 */
			__u16 kvg;	/* s.1.8 */
			__u16 kub;	/* s.2.8 */
			__u16 kvb;	/* s.2.8 */
		} csc;
	};
};

/*
 * Variable win is the pointer to struct tegra_dc_ext_flip_windowattr.
 * Using the modified struct to avoid code conflict in user mode,
 * To avoid any issue for a precompiled application to use with kernel update,
 * kernel code will copy only sizeof(tegra_dc_ext_flip_windowattr)
 * for flip2 use case.
 *
 */
struct tegra_dc_ext_flip_2 {
	struct tegra_dc_ext_flip_windowattr __user *win;
	__u8 win_num;
	__u8 reserved1; /* unused - must be 0 */
	__u16 reserved2; /* unused - must be 0 */
	__u32 post_syncpt_id;
	__u32 post_syncpt_val;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
};

enum tegra_dc_ext_flip_data_type {
	TEGRA_DC_EXT_FLIP_USER_DATA_NONE, /* dummy value - do not use */
	TEGRA_DC_EXT_FLIP_USER_DATA_HDR_DATA,
	TEGRA_DC_EXT_FLIP_USER_DATA_IMP_DATA, /* only valid during PROPOSE */
	TEGRA_DC_EXT_FLIP_USER_DATA_IMP_TAG, /* only valid during FLIP */
	TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT,
	TEGRA_DC_EXT_FLIP_USER_DATA_NVDISP_WIN_CSC,
	TEGRA_DC_EXT_FLIP_USER_DATA_NVDISP_CMU,
	TEGRA_DC_EXT_FLIP_USER_DATA_OUTPUT_CSC,
	TEGRA_DC_EXT_FLIP_USER_DATA_GET_FLIP_INFO,
	TEGRA_DC_EXT_FLIP_USER_DATA_BACKGROUND_COLOR,
	TEGRA_DC_EXT_FLIP_USER_DATA_AVI_DATA,
};

/*
 * Static Metadata for HDR
 * This lets us specify which HDR static metadata to specify in the infoframe.
 * Please see CEA 861.3 for more information.
 */
struct tegra_dc_ext_hdr {
	__u8 eotf;
	__u8 static_metadata_id;
	__u8 static_metadata[24];
} __attribute__((__packed__));

/*
 * Client data used by tegra_dc to force colorimetry
 */
enum tegra_dc_ext_avi_colorimetry {
	TEGRA_DC_EXT_AVI_COLORIMETRY_DEFAULT,
	TEGRA_DC_EXT_AVI_COLORIMETRY_xvYCC709,
	TEGRA_DC_EXT_AVI_COLORIMETRY_BT2020_YCC_RGB,
} __attribute__((__packed__));

struct tegra_dc_ext_avi {
	__u8 avi_colorimetry;
	__u8 reserved[25];
} __attribute__((__packed__));

/*
 * The value at index i of each window-specific array corresponds to the
 * i-th window that's assigned to this head. For the actual id of the i-th
 * window, look in win_ids[i].
 */
#define TEGRA_DC_EXT_N_HEADS	3
#define TEGRA_DC_EXT_N_WINDOWS	6
/*
 * IMP info that's exported to userspace.
 */
struct tegra_dc_ext_imp_emc_dvfs_pair {
	__u32 freq;	/* core EMC frequency (KHz) */
	__u32 latency;	/* DVFS latency (ns) */
};

struct tegra_dc_ext_imp_user_info {
	__u32 num_windows; /* in */
	__u32 __user *win_ids; /* in */
	__u32 __user *in_widths; /* in */
	__u32 __user *out_widths; /* in */
	__u32 emc_dvfs_pairs_requested; /* in */
	__u32 mempool_size; /* out */
	__u32 __user *v_taps; /* out */
	__u32 emc_dvfs_pairs_returned; /* out */
	struct tegra_dc_ext_imp_emc_dvfs_pair __user *emc_dvfs_pairs; /* out */
};

/*
 * The following structs are part of the legacy v1 IMP interface, and represent
 * the IMP settings that are sent from userspace to kernel during PROPOSE.
 *
 * These structs are selected by setting the TEGRA_DC_EXT_FLIP_FLAG_IMP_V1 flip
 * user data flag.
 */
struct tegra_dc_ext_imp_head_results {
	__u32	num_windows;
	__u8	cursor_active;
	__u32	win_ids[TEGRA_DC_EXT_N_WINDOWS];
	__u32	thread_group_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	metering_slots_value_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	thresh_lwm_dvfs_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	pipe_meter_value_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	pool_config_entries_win[TEGRA_DC_EXT_N_WINDOWS];
	__u32	metering_slots_value_cursor;
	__u32	thresh_lwm_dvfs_cursor;
	__u32	pipe_meter_value_cursor;
	__u32	pool_config_entries_cursor;
	__u8	head_active;
	__u64	reserved[4]; /* reserved - must be 0 */
};

struct tegra_dc_ext_imp_settings {
	struct tegra_dc_ext_imp_head_results imp_results[TEGRA_DC_EXT_N_HEADS];
	__u64 hubclk;
	__u32 window_slots_value;
	__u32 cursor_slots_value;
	__u64 required_total_bw_kbps;
	__u64 total_display_iso_bw_kbps;
	__u64 proposed_emc_hz;
	__u64 __user session_id_ptr; /* out - ptr to unsigned 64-bit val */
	__u64 reserved[4]; /* reserved - must be 0 */
};

/*
 * The following structs are part of the new v2 IMP interface, and represent
 * the per-window, per-head, and global IMP settings that are sent from
 * userspace to kernel during PROPOSE.
 *
 * These structs are selected by setting the TEGRA_DC_EXT_FLIP_FLAG_IMP_V2 flip
 * user data flag.
 */
struct tegra_dc_ext_nvdisp_imp_win_entries {
	__u8 id;
	__s8 thread_group;
	__u16 fetch_slots;
	__u32 pipe_meter;
	__u64 dvfs_watermark;
	__u64 min_mempool_entries;
	__u64 mempool_entries;
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_win_settings {
	struct tegra_dc_ext_nvdisp_imp_win_entries entries;

	__u64 reserved[4]; /* must be 0 */
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_head_entries {
	__u8 ctrl_num;
	__u16 curs_fetch_slots;
	__u32 curs_pipe_meter;
	__u64 curs_dvfs_watermark;
	__u64 curs_min_mempool_entries;
	__u64 curs_mempool_entries;
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_head_settings {
	struct tegra_dc_ext_nvdisp_imp_win_settings __user *win_settings;
	__u8 num_wins;

	struct tegra_dc_ext_nvdisp_imp_head_entries entries;

	__u64 reserved[4]; /* must be 0 */
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_global_entries {
	__u16 total_win_fetch_slots;
	__u16 total_curs_fetch_slots;
	__u64 emc_floor_hz;
	__u64 min_hubclk_hz;
	__u64 total_iso_bw_with_catchup_kBps;
	__u64 total_iso_bw_without_catchup_kBps;
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_global_settings {
	struct tegra_dc_ext_nvdisp_imp_global_entries entries;

	__u64 reserved[4]; /* must be 0 */
} __attribute__((__packed__));

struct tegra_dc_ext_nvdisp_imp_settings {
	struct tegra_dc_ext_nvdisp_imp_global_settings global_settings;

	struct tegra_dc_ext_nvdisp_imp_head_settings __user *head_settings;
	__u8 num_heads;

	__u64 session_id;
	__u64 reserved[4]; /* must be 0 */
} __attribute__((__packed__));

/*
 * This struct is a flip user data type (TEGRA_DC_EXT_FLIP_USER_DATA_IMP_DATA)
 * that is sent from userspace to kernel during IMP PROPOSE only.
 *
 * Variable settings is a pointer to tegra_dc_ext_imp_settings.
 * reserved is padding so that the total struct size is 26 bytes.
 */
struct tegra_dc_ext_imp_ptr {
	__u64 __user settings;
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

/*
 * This struct is a flip user data type (TEGRA_DC_EXT_FLIP_USER_DATA_IMP_TAG)
 * that is sent from userspace to kernel during IMP FLIP only.
 *
 * Variable session_id is a unique per-head id that designates which IMP
 * settings actually correspond to this flip.
 * reserved is padding so that the total struct size is 26 bytes.
 */
struct tegra_dc_ext_imp_flip_tag {
	__u64 session_id;
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

/*
 * syncpt_id and syncpt_val are used for raw syncpts. syncpt_fd is used for the
 * fd variant.
 * reserved is padding so that the total struct size is 26 bytes.
 *
 * Users can explicitly request post-syncpts as part of the flip user data by
 * doing the following:
 * 1) Set the flip user data type to TEGRA_DC_EXT_FLIP_USER_DATA_POST_SYNCPT.
 * 2) Set the flip user data flags to select the requested syncpt type. See the
 *    TEGRA_DC_EXT_FLIP_FLAG_POST_SYNCPT_* flags for the supported options.
 *
 * The kernel will fill in the actual struct fields based on the requested
 * syncpt type.
 *
 * There are a few caveats to this mechanism that users should be aware of:
 * 1) The original post-syncpt fence in whatever FLIP ioctl struct is being used
 *    will NOT be honored. This is a MUTUALLY EXCLUSIVE option.
 * 2) The requested post-syncpt type will be enforced for the window pre-syncpts
 *    as well, so it's up to the user to make sure these fields are specified in
 *    a consistent manner.
 * 3) Only one flip user data of this type is allowed. If more than one is
 *    specified, the FLIP ioctl will return failure.
 */
struct tegra_dc_ext_syncpt {
	union {
		struct {
			__u32 syncpt_id;
			__u32 syncpt_val;
		};
		__s32 syncpt_fd;
	};
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

struct tegra_dc_ext_udata_nvdisp_win_csc {
	/* pointer to an array of "tegra_dc_ext_nvdisp_win_csc" */
	__u64 __user array;
	__u8 nr_elements;
	__u8 reserved[17];
} __attribute__((__packed__));

struct tegra_dc_ext_udata_nvdisp_cmu {
	/* pointer to "tegra_dc_ext_nvdisp_cmu" */
	__u64 __user nvdisp_cmu;
	__u8 reserved[18];
} __attribute__((__packed__));

struct tegra_dc_ext_udata_output_csc {
	__u32 output_colorspace;
	/* Valid values for output colorspace:
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC601
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC709
	 * - TEGRA_DC_EXT_FLIP_FLAG_CS_REC2020
	 */
	__u8 limited_range_enable;
	__u8 reserved[21];
} __attribute__((__packed__));

struct tegra_dc_ext_udata_background_color {
	/* bits 31:24	- Background Alpha
	 * bits 23:16	- Background Blue
	 * bits 15:8	- Background Green
	 * bits 7:0	- Background Red
	 */
	__u32 background_color;
	__u8 reserved[22];
} __attribute__((__packed__));

/*
 * Variable "flip_id" is a per-head unique value that is returned from kernel to
 * user-space. User-space can then pass this flip_id to TEGRA_DC_EXT_CRC_GET
 * ioctl to retrieve CRC for that particular flip.
 * Variable "reserved" is padding so that the total struct size is 26 bytes.
 */
struct tegra_dc_ext_flip_info {
	__u64 flip_id;
	__u16 reserved[9]; /* unused - must be 0 */
} __attribute__((__packed__));

/* size of the this struct is 32 bytes */
struct tegra_dc_ext_flip_user_data {
	__u8 data_type;
	__u8 reserved0;
	__u16 flags;
	__u16 reserved1;
	union { /* data to be packed into 26 bytes */
		__u8 data8[26];
		__u16 data16[13];
		struct tegra_dc_ext_hdr hdr_info;
		struct tegra_dc_ext_avi avi_info;
		struct tegra_dc_ext_imp_ptr imp_ptr;
		struct tegra_dc_ext_imp_flip_tag imp_tag;
		struct tegra_dc_ext_syncpt post_syncpt; /* out */
		struct tegra_dc_ext_udata_nvdisp_win_csc nvdisp_win_csc;
		struct tegra_dc_ext_udata_nvdisp_cmu nvdisp_cmu;
		struct tegra_dc_ext_udata_output_csc output_csc;
		struct tegra_dc_ext_flip_info flip_info;
		struct tegra_dc_ext_udata_background_color background_color;
	};
} __attribute__((__packed__));

/*
 * tegra_dc_flip_4 : Incorporates a new pointer to an array of 32 bytes of data
 * to pass head specific info. The new nr_elements carries the number of such
 * elements.
 */
struct tegra_dc_ext_flip_4 {
	__u64 __user win;
	__u8 win_num;
	__u8 flags;
	__u16 reserved;
	__s32 post_syncpt_fd;
	__u16 dirty_rect[4]; /* x,y,w,h for partial screen update. 0 ignores */
	__u32 nr_elements; /* number of data entities pointed to by data */
	__u64 __user data; /* pointer to struct tegra_dc_ext_flip_user_data*/
};

/*
 * vblank control - enable or disable vblank events
 */

struct tegra_dc_ext_set_vblank {
	__u8	enable;
	__u8	reserved[3]; /* unused - must be 0 */
};

/*
 * tegra_dc_ext_cap_type : Defines the different types of per-display capability
 * info that could be provided by the kernel to user space.
 *
 * These cap types are specific to the tegra_dc_ext device that they're queried
 * from.
 */
enum tegra_dc_ext_cap_type {
	TEGRA_DC_EXT_CAP_TYPE_NONE, /* dummy value - do not use */
	TEGRA_DC_EXT_CAP_TYPE_HDR_SINK, /* struct tegra_dc_ext_hdr_caps */
	TEGRA_DC_EXT_CAP_TYPE_QUANT_SELECTABLE,
		/* struct tegra_dc_ext_quant_caps */
	TEGRA_DC_EXT_CAP_TYPE_MAX,
};

/*
 * tegra_dc_ext_control_cap_type : Defines the different types of
 * SOC-level/common capability info that could be provided by the kernel to user
 * space.
 *
 * These cap types are queried from the common tegra_dc_control device.
 */
enum tegra_dc_ext_control_cap_type {
	TEGRA_DC_EXT_CONTROL_CAP_TYPE_NONE, /* dummy value - do not use */
	/* struct tegra_dc_ext_imp_caps - only for NVDISPLAY */
	TEGRA_DC_EXT_CONTROL_CAP_TYPE_IMP,
	TEGRA_DC_EXT_CONTROL_CAP_TYPE_MAX,
};

/*
 * tegra_dc_ext_hdr_caps : Incorporates target display's hdr capabilities.
 * nr_elements : Indicates the number of the following data. When set to 0,
 * the sink didn't provide the hdr static metadata in the edid.
 * eotf : Indiactes the eotf supported by the sink.
 * static_metadata_type : indicates which Static Metadata Descriptors are
 * supported.
 * desired_content_max_lum : Code Value indicating the Desired Content Max
 * Luminance Data
 * desired_content_max_frame_avg_lum : Code Value indicating the Desired
 * Content Max Frame-average Luminance.
 * desired_content_min_lum : Code Value indicating the Desired Content Min
 * Luminance.
 *
 * Note:  The last 3 data are optional to declare in the edid. When nr_elements
 * = 3, they are absent. When nr_elements = 4, desired_content_max_lum is
 * present; when nr_elements = 5, desired_content_max_lum and
 * desired_content_max_frame_avg_lum are present; and when it's 6, all 3 of
 * them are present. When nr_elements > 3, each of the 3 values which are
 * indicated to be present in the HDR Static Metadata Data Block may be set to
 * zero. This value indicates that the data for the relevant Desired Max
 * Content Luminance, Desired Content Max Frameaverage Luminance or Desired
 * Content Min Luminance is not indicated.
 */
struct tegra_dc_ext_hdr_caps {
	__u8 nr_elements;
	__u8 eotf;
	__u8 static_metadata_type;
	__u8 desired_content_max_lum;
	__u8 desired_content_max_frame_avg_lum;
	__u8 desired_content_min_lum;
};

/*
 * tegra_dc_ext_imp_thread_info: Encapsulates the thread group information for
 * a given window
 *
 * win_id: The HW id of the window that the client is requesting info for
 *
 * thread_group: The thread group that's assigned to the given window.
 * Each window can only be assigned one thread group, as long as that thread
 * group isn't already assigned. A value of -1 indicates that no thread group is
 * assigned to this window.
 */
struct tegra_dc_ext_imp_thread_info {
	__u8 win_id; /* in - filled in by client */
	__s8 thread_group;

	__u64 reserved[4];
} __attribute__((__packed__));

/*
 * tegra_dc_ext_imp_mc_caps: Encapsulates the system-level MC/IHUB configs that
 * need to be exported from kernel to userspace for IMP
 */
struct tegra_dc_ext_imp_mc_caps {
	__u64 peak_hubclk_hz;
	__u32 num_dram_channels;
	__u32 total_mempool_size_bytes;
	__u32 request_batch_size;

	__u64 reserved[8]; /* must be zero */
} __attribute__((__packed__));

/*
 * tegra_dc_ext_imp_caps: Encapsulates the IMP caps that kernel needs to export
 * to userspace
 *
 * mc_caps: The system-level MC/IHUB configs required for IMP
 *
 * num_dvfs_requested: The number of EMC DVFS pairs requested by the client.
 * A safe number to use here is 14, since that's the max number of entries
 * that's currently hardcoded in the bpmp ABI headers.
 *
 * num_dvfs_returned: The number of EMC DVFS pairs that kernel was able to copy
 * back to the client. If the number of requested pairs is less than the number
 * of available entries, the first lower num_dvfs_requested pairs will be
 * returned.
 *
 * dvfs_pairs: The actual EMC DVFS pairs. The client is responsible for
 * allocating enough memory for at least num_dvfs_requested pairs.
 *
 * num_thread_info: The number of thread info entries requested by the client.
 * This cannot exceed the total number of windows that are supported on the
 * underlying SOC.
 *
 * thread_info: The actual thread info entries. The client is responsible for
 * allocating enough memory for at least num_thread_info entries.
 */
struct tegra_dc_ext_imp_caps {
	struct tegra_dc_ext_imp_mc_caps mc_caps;

	__u32 num_dvfs_requested; /* in - filled in by client */
	__u32 num_dvfs_returned;
	struct tegra_dc_ext_imp_emc_dvfs_pair __user *dvfs_pairs;

	__u32 num_thread_info; /* in - filled in by client */
	struct tegra_dc_ext_imp_thread_info __user *thread_info;

	__u64 reserved[8]; /* must be zero */
} __attribute__((__packed__));

/*
 * tegra_dc_ext_quant_caps : Incorporates target display's quantization
 * capabilities.
 * rgb_quant_selectable : indicates whether rgb quantization range is
 * selectable
 * yuv_quant_selectable : indicates whether yuv quantization range is
 * selectable
 */
struct tegra_dc_ext_quant_caps {
	__u8 rgb_quant_selectable;
	__u8 yuv_quant_selectable;
};

/*
 * tegra_dc_ext_caps : Incorporates target display capabilities.
 * data_type : Indicates the type of capability.
 * data = pointer to the actual data.
 */
struct tegra_dc_ext_caps {
	__u32 data_type;
	__u64 __user data;
};

/*
 * get display capabilities.
 * nr_elements : Inidicates the no of elements of "tegra_dc_ext_caps" type the
 * data pointer in pointing to.
 * data : pointer to tegra_dc_ext_caps.
 */
struct tegra_dc_ext_get_cap_info {
	__u32	nr_elements;
	__u64 __user data;
};

/*
 * Cursor image format:
 *
 * Tegra hardware supports two different cursor formats:
 *
 * (1) Two color cursor: foreground and background colors are
 *     specified by the client in RGB8.
 *
 *     The two-color image should be specified as two 1bpp bitmaps
 *     immediately following each other in memory.  Each pixel in the
 *     final cursor will be constructed from the bitmaps with the
 *     following logic:
 *
 *		bitmap1 bitmap0
 *		(mask)  (color)
 *		  1	   0	transparent
 *		  1	   1	inverted
 *		  0	   0	background color
 *		  0	   1	foreground color
 *
 *     This format is supported when TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES
 *     reports the TEGRA_DC_EXT_CAPABILITIES_CURSOR_TWO_COLOR bit.
 *
 * (2) RGBA cursor: in this case the image is four bytes per pixel,
 *     with alpha in the low eight bits.
 *
 *     The RGB components of the cursor image can be either
 *     premultipled by alpha:
 *
 *         cursor[r,g,b] + desktop[r,g,b] * (1 - cursor[a])
 *
 *     or not:
 *
 *         cursor[r,g,b] * cursor[a] + desktop[r,g,b] * (1 - cursor[a])
 *
 *     TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES will report one or more of
 *     TEGRA_DC_EXT_CURSOR_FLAGS_RGBA{,_NON}_PREMULT_ALPHA to indicate
 *     which are supported on the current hardware.
 *
 * Specify one of TEGRA_DC_EXT_CURSOR_FLAGS to indicate the format.
 *
 * Exactly one of the SIZE flags must be specified.
 */


#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_32x32	((1 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_64x64	((2 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_128x128	((3 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE_256x256	((4 & 0x7) << 0)
#define TEGRA_DC_EXT_CURSOR_IMAGE_FLAGS_SIZE(x)		(((x) & 0x7) >> 0)

#define TEGRA_DC_EXT_CURSOR_FORMAT_2BIT_LEGACY			(0)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA	(1)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_PREMULT_ALPHA		(3)
#define TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_XOR			(4)

#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_2BIT_LEGACY \
	(TEGRA_DC_EXT_CURSOR_FORMAT_2BIT_LEGACY << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_NON_PREMULT_ALPHA	\
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_PREMULT_ALPHA \
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_PREMULT_ALPHA << 16)
#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_XOR \
	(TEGRA_DC_EXT_CURSOR_FORMAT_RGBA_XOR << 16)

#define TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS(x)	(((x) >> 16) & 0x7)

#define TEGRA_DC_EXT_CURSOR_COLORFMT_LEGACY	(0)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_R8G8B8A8	(1)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_A1R5G5B5	(2)
#define TEGRA_DC_EXT_CURSOR_COLORFMT_A8R8G8B8	(3)

#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_LEGACY \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_LEGACY << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_R8G8B8A8 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_R8G8B8A8 << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A1R5G5B5 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_A1R5G5B5 << 8)
#define TEGRA_DC_EXT_CURSOR_FLAGS_COLORFMT_A8R8G8B8 \
	(TEGRA_DC_EXT_CURSOR_COLORFMT_A8R8G8B8 << 8)

#define TEGRA_DC_EXT_CURSOR_COLORFMT_FLAGS(x)	(((x) >> 8) & 0xf)

/* aliases for source-level backwards compatibility */
#define TEGRA_DC_EXT_CURSOR_FLAGS_RGBA_NORMAL \
	TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_RGBA_NON_PREMULT_ALPHA
#define TEGRA_DC_EXT_CURSOR_FLAGS_2BIT_LEGACY \
	TEGRA_DC_EXT_CURSOR_FORMAT_FLAGS_2BIT_LEGACY

#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MIN (0x00)
#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MAX (0xff)
#define TEGRA_DC_EXT_CURSOR_FORMAT_ALPHA_MSK (0xff)

enum CURSOR_COLOR_FORMAT {
	legacy,		/* 2bpp */
	r8g8b8a8,	/* normal */
	a1r5g5b5,
	a8r8g8b8,
};

struct tegra_dc_ext_cursor_image {
	struct {
		__u8	r;
		__u8	g;
		__u8	b;
	} foreground, background;
	__u32	buff_id;
	__u32	flags;
	__s16	x;
	__s16	y;
	__u32	alpha; /* was vis*/
	enum CURSOR_COLOR_FORMAT colorfmt; /* was mode */
};

/* Possible flags for struct nvdc_cursor's flags field */
#define TEGRA_DC_EXT_CURSOR_FLAGS_VISIBLE	(1 << 0)

struct tegra_dc_ext_cursor {
	__s16 x;
	__s16 y;
	__u32 flags;
};

/*
 * Color conversion is performed as follows:
 *
 * r = sat(kyrgb * sat(y + yof) + kur * u + kvr * v)
 * g = sat(kyrgb * sat(y + yof) + kug * u + kvg * v)
 * b = sat(kyrgb * sat(y + yof) + kub * u + kvb * v)
 *
 * Coefficients should be specified as fixed-point values; the exact format
 * varies for each coefficient.
 * The format for each coefficient is listed below with the syntax:
 * - A "s." prefix means that the coefficient has a sign bit (twos complement).
 * - The first number is the number of bits in the integer component (not
 *   including the optional sign bit).
 * - The second number is the number of bits in the fractional component.
 *
 * All three fields should be tightly packed, justified to the LSB of the
 * 16-bit value.  For example, the "s.2.8" value should be packed as:
 * (MSB) 5 bits of 0, 1 bit of sign, 2 bits of integer, 8 bits of frac (LSB)
 */
struct tegra_dc_ext_csc {
	__u32 win_index;
	__u16 yof;	/* s.7.0 */
	__u16 kyrgb;	/*   2.8 */
	__u16 kur;	/* s.2.8 */
	__u16 kvr;	/* s.2.8 */
	__u16 kug;	/* s.1.8 */
	__u16 kvg;	/* s.1.8 */
	__u16 kub;	/* s.2.8 */
	__u16 kvb;	/* s.2.8 */
};

/*
 * Coefficients should be specified as fixed-point values; the exact format
 * varies for each coefficient.
 * Each coefficient is a signed 19bit number with 3 integer bits and 16
 * fractional bits. Overall range is from -4.0 to 3.999
 * All three fields should be tightly packed in 32bit
 * For example, the "s.3.16" value should be packed as:
 * (MSB) 12 bits of 0, 1 bit of sign, 3 bits of integer, 16 bits of frac (LSB)
 */
struct tegra_dc_ext_nvdisp_win_csc {
	__u32 win_index;
	__u32 csc_enable;
	__u32 r2r;		/* s.3.16 */
	__u32 g2r;		/* s.3.16 */
	__u32 b2r;		/* s.3.16 */
	__u32 const2r;		/* s.3.16 */
	__u32 r2g;		/* s.3.16 */
	__u32 g2g;		/* s.3.16 */
	__u32 b2g;		/* s.3.16 */
	__u32 const2g;		/* s.3.16 */
	__u32 r2b;		/* s.3.16 */
	__u32 g2b;		/* s.3.16 */
	__u32 b2b;		/* s.3.16 */
	__u32 const2b;		/* s.3.16 */
};

struct tegra_dc_ext_cmu {
	__u16 cmu_enable;
	__u16 csc[9];
	__u16 lut1[256];
	__u16 lut2[960];
};

/*
 * Two types for LUT size 257 or 1025
 * Based on lut size the input width is different
 * Each component is in 16 bit format
 * For example With Unity LUT range with 1025 size
 * Each component (R,G,B) content is in 14bits
 * Index bits are in upper 10 bits
 * Black to White range from 0x6000 to 0x9FFF
 * LUT array is represented in 64 bit, each component is shifted
 * appropriately to represent in 64bit data.
 * For example, rgb[i] = (B << 32) | (G << 16) | (R << 0)
 * lut_ranges - input value range covered by lut,
 * it can be unity (0.0 ..1.0), xrbias(-0.75 .. +1.25),
 * xvycc(-1.5 to +2.5)
 * lut_mode - index or interpolate
 */
#define TEGRA_DC_EXT_LUT_SIZE_257	256
#define TEGRA_DC_EXT_LUT_SIZE_1025	1024

#define TEGRA_DC_EXT_OUTLUT_MODE_INDEX		0
#define TEGRA_DC_EXT_OUTLUT_MODE_INTERPOLATE	1

#define TEGRA_DC_EXT_OUTLUT_RANGE_UNITY		0
#define TEGRA_DC_EXT_OUTLUT_RANGE_XRBAIS	1
#define TEGRA_DC_EXT_OUTLUT_RANGE_XVYCC		2

struct tegra_dc_ext_nvdisp_cmu {
	__u16 cmu_enable;
	__u16 lut_size;
	__u16 lut_range; /* ignored in the driver */
	__u16 lut_mode; /* ignored in the driver */
	__u64 rgb[TEGRA_DC_EXT_LUT_SIZE_1025 + 1];
};

/*
 * RGB Lookup table
 *
 * In true-color and YUV modes this is used for post-CSC RGB->RGB lookup, i.e.
 * gamma-correction. In palette-indexed RGB modes, this table designates the
 * mode's color palette.
 *
 * To convert 8-bit per channel RGB values to 16-bit, duplicate the 8 bits
 * in low and high byte, e.g. r=r|(r<<8)
 *
 * To just update flags, set len to 0.
 *
 * Current Tegra DC hardware supports 8-bit per channel to 8-bit per channel,
 * and each hardware window (overlay) uses its own lookup table.
 *
 */
struct tegra_dc_ext_lut {
	__u32  win_index; /* window index to set lut for */
	__u32  flags;     /* Flag bitmask, see TEGRA_DC_EXT_LUT_FLAGS_* */
	__u32  start;     /* start index to update lut from */
	__u32  len;       /* number of valid lut entries */
	__u16 __user *r;         /* array of 16-bit red values, 0 to reset */
	__u16 __user *g;         /* array of 16-bit green values, 0 to reset */
	__u16 __user *b;         /* array of 16-bit blue values, 0 to reset */
};

/* tegra_dc_ext_lut.flags - override global fb device lookup table.
 * Default behaviour is double-lookup.
 */
#define TEGRA_DC_EXT_LUT_FLAGS_FBOVERRIDE 0x01

#define TEGRA_DC_EXT_FLAGS_ENABLED	1
struct tegra_dc_ext_status {
	__u32 flags;
	/* Leave some wiggle room for future expansion */
	__u32 pad[3];
};

/*
 * Tegra Display Screen Capture
 *
 * This feature will make a snap shot of display HW configurations and its
 * frame buffer contents, so reconstruction of full display screen would be
 * possible.
 *
 * A screen capture will be made in following sequence.
 *
 * 1. Pause display flip on all heads with the IOCTL
 *    TEGRA_DC_EXT_CONTROL_SCRNCAPT_PAUSE. The kernel driver may also set a
 *    timer for automatic resume. The default timer value for native Linux is
 *    0.5Sec and user app can override the default timer value. The timer
 *    value can vary depending on each OS and HyperVisor configuration.
 *    The display pause is an exclusive operation. Once a pause is called,
 *    no more pause call is allowed until resuming the pause in effective.
 *    For this purpose, the pause call will return a magic number. It should
 *    be saved and used by following screen capture calls until the resume.
 * 2. Collect configration information of a display head and all windows
 *    assigned to the head with the IOCTL TEGRA_DC_EXT_SCRNCAPT_GET_INFO.
 * 3. Duplicate the frame buffer of the latest flip of a window that is
 *    currently on display with the IOCTL TEGRA_DC_EXT_SCRNCAPT_DUP_FBUF.
 * 4. Repeat the step 3 for every window assigned to the head.
 * 5. Repeat the step 2 through step 4 for every head to capture.
 * 6. Resume display flip on all heads and clear the auto resume timer with
 *    the IOCTL TEGRA_DC_EXT_CONTROL_SCRNCAPT_RESUME. Pausing and resuming
 *    flips are intended for all display heads to make the internal logic
 *    simple and efficient. This means no individual head pause and no partial
 *    resumming are supported.
 */

/* To collect configuration information of one display head
 */
/* API data structure version and magic number */
#define TEGRA_DC_EXT_SCRNCAPT_VER_2(magic) (((magic) & ~0xff) | 2)
#define TEGRA_DC_EXT_SCRNCAPT_VER_V(magic) ((magic) & 0xff)
/* info_type HEAD: struct tegra_dc_ext_scrncapt_get_info_head */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_HEAD (0)
/* info_type WIN: struct tegra_dc_ext_scrncapt_get_info_win */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_WINS (1)
/* info_type CURSOR: struct tegra_dc_ext_cursor_image */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR (2)
/* info_type CURSOR_DATA: struct tegra_dc_ext_scrncapt_get_info_cursor_data */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_CURSOR_DATA (3)
/* info_type NVDISP_CMU: struct tegra_dc_ext_nvdisp_cmu */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_NVDISP_CMU (4)
/* info_type NVDISP_WIN_CSC: struct tegra_dc_ext_nvdisp_win_csc */
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_NVDISP_WIN_CSC (5)
#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO_FLAG_WINS(n)  (1u << (n))

struct tegra_dc_ext_scrncapt_get_info_head {
	__u8   sts_en;        /* returns display head enabled or not */
	__u32  hres;          /* returns display horizontal resolution */
	__u32  vres;          /* returns display vertical resolution */
	__u32  flag_val_wins; /* returns valid windows mask */
	__u32  reserved[12];
};

struct tegra_dc_ext_scrncapt_get_info_win {
	__u32 flag_wins; /* bitmask of TEGRA_DC_EXT_SCRNCAPT_GET_INFO_FLAG_WINS
			  * set bit to indicate the window to be captured */
	__u32 num_wins;    /* returns number of windows saved to 'wins' */
	__u64 __user wins; /* pointer to array of
			    * struct tegra_dc_ext_flip_windowattr.
			    * should have enough entries to hold 'flag_wins'
			    * selection. */
	__u32 reserved[8];
};

struct tegra_dc_ext_scrncapt_get_info_cursor_data {
	__u32 size;       /* byte size of total space allocated to 'ptr' */
	__u32 len;        /* returns byte size of data copied into 'ptr' */
	__u64 __user ptr; /* pointer to buffer */
	__u32 reserved[4];
};

struct tegra_dc_ext_scrncapt_get_info_data {
	__u32 type; /* type of data, TEGRA_DC_EXT_SCRNCAPT_GET_INFO_TYPE_xxx */
	__u64 __user ptr; /* pointer to its data structure */
	__u32 reserved[3];
};

struct tegra_dc_ext_scrncapt_get_info  {
	__u32 ver;      /* set to TEGRA_DC_EXT_SCRNCAPT_VER_2
			 * returns TEGRA_DC_EXT_SCRNCAPT_VER_2 */
	__u32 head;     /* head ID */
	__u32 num_data; /* number of entries in 'data' */
	__u64 __user data; /* pointer to the array of
			    * struct tegra_dc_ext_scrncapt_get_info_data */
	__u32 reserved[8];
};

/* To duplicate single window raw frame buffer that may consists with one or
 * more planes.
 *
 * RGB pixel formats use only the first plane.
 * YUV(YCbCr) pixel formats use one to 3 planes depending on its format.
 * Y plane holds Y or YUV(YCbCr) data depending on the pixel format.
 * U(Cb) plane holds U(Cb) or UV(CbCr) data depending on the pixel format.
 * V(Cr) plane holds V(Cr) data depending on the pixel format.
 * CDE plane is not used.
 *
 * The returned length of each plane indicates the data length of each plane
 * in the raw frame buffer, and 0 length indicates the plane is not used.
 * The returned offset of each plane indicates the offset to the plane within
 * the data buffer pointed by 'buffer'.
 */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_RGB  0 /* RGB plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_Y    0 /* Y plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_U    1 /* U or Cb plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_V    2 /* V or Cr plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_CDE  3 /* CDE plane */
#define  TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM  4 /* number of planes */

struct tegra_dc_ext_scrncapt_dup_fbuf  {
	__u32 ver;      /* set to TEGRA_DC_EXT_SCRNCAPT_VER_2
			 * returns TEGRA_DC_EXT_SCRNCAPT_VER_2 */
	__u32 head;     /* head ID */
	__u32 win;      /* window ID */
	__u32 buffer_max;    /* allocated byte amount to 'buffer' */
	__u64 __user buffer; /* pointer to data buffer to store all planes */
	/* returns length of each plane, 0 for plane not available */
	__u32 plane_sizes[TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM];
	/* returns offset of each plane within the 'buffer' */
	__u32 plane_offsets[TEGRA_DC_SCRNCAPT_DUP_FBUF_IDX_NUM];
	__u32 reserved[16];
};

/* Scanline sync ioctl */
#define TEGRA_DC_EXT_SCANLINE_FLAG_ENABLE (1U << 0)
#define TEGRA_DC_EXT_SCANLINE_FLAG_DISABLE (0U << 0)
#define TEGRA_DC_EXT_SCANLINE_FLAG_RAW_SYNCPT (1U << 1)
#define TEGRA_DC_EXT_SCANLINE_FLAG_SYNCFD (0U << 1)
/* unused flags are reserved and must be 0 */

struct tegra_dc_ext_scanline_info {
	__u8 id; /* 0 = VPULSE3 */
	__u8 frame; /* a.k.a. swap_interval, a.k.a. min_present. typically 0 */
	__u16 flags; /* select between fd and id:val */
	__u32 triggered_line;
	union {
		struct {
			__u32 id;
			__u32 val;
		} raw_syncpt; /* used only is RAW_SYNCPT */

		__u32 syncfd;
	};
};

#define TEGRA_DC_EXT_SET_NVMAP_FD \
	_IOW('D', 0x00, __s32)

#define TEGRA_DC_EXT_GET_WINDOW \
	_IOW('D', 0x01, __u32)
#define TEGRA_DC_EXT_PUT_WINDOW \
	_IOW('D', 0x02, __u32)

#define TEGRA_DC_EXT_GET_CURSOR \
	_IO('D', 0x04)
#define TEGRA_DC_EXT_PUT_CURSOR \
	_IO('D', 0x05)
#define TEGRA_DC_EXT_SET_CURSOR_IMAGE \
	_IOW('D', 0x06, struct tegra_dc_ext_cursor_image)
#define TEGRA_DC_EXT_SET_CURSOR \
	_IOW('D', 0x07, struct tegra_dc_ext_cursor)

#define TEGRA_DC_EXT_SET_CSC \
	_IOW('D', 0x08, struct tegra_dc_ext_csc)

#define TEGRA_DC_EXT_GET_STATUS \
	_IOR('D', 0x09, struct tegra_dc_ext_status)

/*
 * Returns the auto-incrementing vblank syncpoint for the head associated with
 * this device node
 */
#define TEGRA_DC_EXT_GET_VBLANK_SYNCPT \
	_IOR('D', 0x09, __u32)

#define TEGRA_DC_EXT_SET_LUT \
	_IOW('D', 0x0A, struct tegra_dc_ext_lut)

#define TEGRA_DC_EXT_CURSOR_CLIP \
	_IOW('D', 0x0C, __s32)

#define TEGRA_DC_EXT_SET_CMU \
	_IOW('D', 0x0D, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_GET_CMU \
	_IOR('D', 0x0F, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_GET_CUSTOM_CMU \
	_IOR('D', 0x10, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_SET_PROPOSED_BW \
	_IOR('D', 0x13, struct tegra_dc_ext_flip_2)

#define TEGRA_DC_EXT_SET_VBLANK \
	_IOW('D', 0x15, struct tegra_dc_ext_set_vblank)

#define TEGRA_DC_EXT_SET_CMU_ALIGNED \
	_IOW('D', 0x16, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_SET_NVDISP_WIN_CSC \
	_IOW('D', 0x17, struct tegra_dc_ext_nvdisp_win_csc)

#define TEGRA_DC_EXT_SET_NVDISP_CMU \
	_IOW('D', 0x18, struct tegra_dc_ext_nvdisp_cmu)

#define TEGRA_DC_EXT_GET_NVDISP_CMU \
	_IOR('D', 0x19, struct tegra_dc_ext_nvdisp_cmu)

#define TEGRA_DC_EXT_GET_CUSTOM_NVDISP_CMU \
	_IOR('D', 0x1A, struct tegra_dc_ext_nvdisp_cmu)

#define TEGRA_DC_EXT_SET_PROPOSED_BW_3 \
	_IOWR('D', 0x1B, struct tegra_dc_ext_flip_4)

#define TEGRA_DC_EXT_GET_CMU_ADBRGB\
	_IOR('D', 0x1C, struct tegra_dc_ext_cmu)

#define TEGRA_DC_EXT_FLIP4\
	_IOW('D', 0x1D, struct tegra_dc_ext_flip_4)

#define TEGRA_DC_EXT_GET_WINMASK \
	_IOR('D', 0x1E, __u32)

#define TEGRA_DC_EXT_SET_WINMASK \
	_IOW('D', 0x1F, __u32)

#define TEGRA_DC_EXT_GET_IMP_USER_INFO \
	_IOW('D', 0x20, struct tegra_dc_ext_imp_user_info)

#define TEGRA_DC_EXT_SCRNCAPT_GET_INFO \
	_IOWR('D', 0x21, struct tegra_dc_ext_scrncapt_get_info)

#define TEGRA_DC_EXT_SCRNCAPT_DUP_FBUF \
	_IOWR('D', 0x22, struct tegra_dc_ext_scrncapt_dup_fbuf)

#define TEGRA_DC_EXT_GET_CAP_INFO\
	_IOW('D', 0x23, struct tegra_dc_ext_get_cap_info)

#define TEGRA_DC_EXT_GET_SCANLINE \
	_IOR('D', 0x24, __u32)

#define TEGRA_DC_EXT_SET_SCANLINE \
	_IOWR('D', 0x25, struct tegra_dc_ext_scanline_info)

/* The TEGRA_DC_EXT_CRC_* set of IOCTLs have been added for userspace to
 * query CRCs generated by different blocks of the display pipeline, such as
 * Raster Generator (RG), Compositor (COMP), Output Resource (OR) - could be
 * SOR, DSI, etc. - and Regional CRCs located in the RG block (RG_REGIONAL).
 * The central parameter of the argument for each IOCTL is the configuration
 * member (see struct tegra_dc_ext_crc_conf) used to identify the block
 * generating CRC (see tegra_dc_ext_crc_type) and the corresponding parameters
 * to be programmed (see tegra_dc_ext_crc_conf_params). The argument can pack
 * an array of configurations as a part of the same IOCTL call, which would be
 * equivalent to calling the IOCTL multiple times, one configuration at a time.
 * Each of these types of CRCs can be enabled, disabled or queried independently
 */

/* This IOCTL is used to enable CRC collection from various blocks in the
 * display pipeline. The parameters to be programmed are represented by a union
 * structure tegra_dc_ext_crc_conf_params. Please refer to the definitions of
 * individual data structures packed in that union for more details. The
 * enumerated values are assumed to be self-explanatory, to avoid verbosity
 *
 * Returns
 * -EINVAL   if arg.magic is wrongly programmed, or
 *           if number of configurations exceed feasible limits
 * -ENODEV   if the tegra display device (head) is yet to be initialized
 * -EBUSY    if the legacy CRC mechanism (via sysfs) has been activated
 * -ENOTSUPP if the user tries to program golden CRC registers, the support for
 *           which is yet to be added, or
 *           if arg.version is wrongly programmed, or
 *           if the arg.conf.type is not supported
 */
#define TEGRA_DC_EXT_CRC_ENABLE \
	_IOW('D', 0x26, struct tegra_dc_ext_crc_arg)

/* This IOCTL is used to disable CRC collection for a block, that was
 * previously enabled via a call to TEGRA_DC_CRC_ENABLE IOCTL.
 *
 * Returns
 * -EINVAL   Same conditions as mentioned for TEGRA_DC_EXT_CRC_ENABLE
 * -ENODEV   Same conditions as mentioned for TEGRA_DC_EXT_CRC_ENABLE
 * -EPERM    if the user calls the IOCTL without a call to
 *           TEGRA_DC_EXT_CRC_ENABLE with the same CRC type and/or region
 * -ENOTSUPP if arg.version is wrongly programmed, or
 *           if the arg.conf.type is not supported
 */
#define TEGRA_DC_EXT_CRC_DISABLE \
	_IOW('D', 0x27, struct tegra_dc_ext_crc_arg)

/* Retrieve the CRCs for a frame generated as a result of a specific flip
 * request. The flip is identified using flip ID that was returned to the user
 * by the TEGRA_DC_EXT_FLIP4 IOCTL. The call to the IOCTL shall block until the
 * CRC for the frame is generated.
 *
 * Returns
 * -EINVAL   Same conditions as mentioned for TEGRA_DC_EXT_CRC_ENABLE
 * -ENODEV   Same conditions as mentioned for TEGRA_DC_EXT_CRC_ENABLE
 * -EPERM    Same conditions as mentioned for TEGRA_DC_EXT_CRC_DISABLE
 * -ENOTSUPP if the arg.conf.type is not supported
 * -ETIME    if wait for the next Frame End Interrupt timed out
 * -ENODATA  if the flip identified by arg.flip_id has not been programmed,
 *           or it was programmed a long time ago, such that the corresponding
 *           CRCs are dropped from the kernel CRC buffer, or
 *           if arg.flip_id is set to U64_MAX, but no flips have been
 *           programmed
 */
#define TEGRA_DC_EXT_CRC_GET \
	_IOWR('D', 0x28, struct tegra_dc_ext_crc_arg)

enum tegra_dc_ext_control_output_type {
	TEGRA_DC_EXT_DSI,
	TEGRA_DC_EXT_LVDS,
	TEGRA_DC_EXT_VGA,
	TEGRA_DC_EXT_HDMI,
	TEGRA_DC_EXT_DVI,
	TEGRA_DC_EXT_DP,
	TEGRA_DC_EXT_EDP,
	TEGRA_DC_EXT_NULL,
	TEGRA_DC_EXT_HDSI, /*support DSI as external display*/
};

/*
 * Get the properties for a given output.
 *
 * handle (in): Which output to query
 * type (out): Describes the type of the output
 * connected (out): Non-zero iff the output is currently connected
 * associated_head (out): The head number that the output is currently
 *      bound to.  -1 iff the output is not associated with any head.
 * head_mask (out): Bitmask of which heads the output may be bound to (some
 *      outputs are permanently bound to a single head).
 */
struct tegra_dc_ext_control_output_properties {
	__u32 handle;
	enum tegra_dc_ext_control_output_type type;
	__u32 connected;
	__s32 associated_head;
	__u32 head_mask;
};

/*
 * This allows userspace to query the raw EDID data for the specified output
 * handle.
 *
 * Here, the size parameter is both an input and an output:
 * 1. Userspace passes in the size of the buffer allocated for data.
 * 2. If size is too small, the call fails with the error EFBIG; otherwise, the
 *    raw EDID data is written to the buffer pointed to by data.  In both
 *    cases, size will be filled in with the size of the data.
 */
struct tegra_dc_ext_control_output_edid {
	__u32 handle;
	__u32 size;
	void __user *data;
};

struct tegra_dc_ext_event {
	__u32	type;
	__u32	data_size;
	char	data[0];
};

/* Events types are bits in a mask */
#define TEGRA_DC_EXT_EVENT_HOTPLUG			(1 << 0)
struct tegra_dc_ext_control_event_hotplug {
	__u32 handle;
};

#define TEGRA_DC_EXT_EVENT_VBLANK			(1 << 1)
struct tegra_dc_ext_control_event_vblank {
	__u32 handle;
	__u32 reserved; /* unused */
	__u64 timestamp_ns;
};

#define TEGRA_DC_EXT_EVENT_BANDWIDTH_INC	(1 << 2)
#define TEGRA_DC_EXT_EVENT_BANDWIDTH_DEC	(1 << 3)
struct tegra_dc_ext_control_event_bandwidth {
	__u32 handle;
	__u32 total_bw;
	__u32 avail_bw;
	__u32 resvd_bw;
};

#define TEGRA_DC_EXT_EVENT_MODECHANGE      (1 << 4)
struct tegra_dc_ext_control_event_modechange {
	__u32 handle;
};

#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_MODE			(1 << 0)
#define TEGRA_DC_EXT_CAPABILITIES_BLOCKLINEAR			(1 << 1)

#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_TWO_COLOR		(1 << 2)
#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_RGBA_NON_PREMULT_ALPHA	(1 << 3)
#define TEGRA_DC_EXT_CAPABILITIES_CURSOR_RGBA_PREMULT_ALPHA	(1 << 4)
#define TEGRA_DC_EXT_CAPABILITIES_NVDISPLAY			(1 << 5)

struct tegra_dc_ext_control_capabilities {
	__u32 caps;
	/* Leave some wiggle room for future expansion */
	__u32 pad[3];
};


/* Tegra Display Screen Capture
 * control IOCTL
 */
#define  TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC  (0x73636171)

struct tegra_dc_ext_control_scrncapt_pause {
	__u32  magic; /* call with TEGRA_DC_EXT_CONTROL_SCRNCAPT_MAGIC
		       * returns a magic value for the session */
	__u32  reserved;
	__u32  tm_resume_msec; /* auto resume timer value in mSec
				*   0: use disp driver default value
				*   -1: turn off auto resume timer
				*   others: valid timer value
				* returns the timer set value
				*   -1: auto resume timer turned off
				*   others: timer set value */
	__u32  num_heads; /* returns max number of DC heads */
	__u32  num_wins;  /* returns max number of windows */
	__u32  reserved2[3];
};

struct tegra_dc_ext_control_scrncapt_resume {
	__u32  magic; /* magic value returned from the pause call */
	__u32  reserved[7];
};

/**
 * struct tegra_dc_ext_control_frm_lck_params - Used by userspace to get and
 * update frame_lock status in kernel.
 */
struct tegra_dc_ext_control_frm_lck_params {
	/**
	 * @frame_lock_status: Tells the current frame_lock status in kernel.
	 * Since it's boolean in kernel and the data type here is u8. Values
	 * here should inly be 0 and 1.
	 */
	__u8 frame_lock_status;
	/**
	 * @valid_heads: It's bit-mapped array storing the head_ids of the
	 * participating dc heads in frame-lock.
	 */
	__u64 valid_heads;
};

enum tegra_dc_ext_crc_arg_version {
	TEGRA_DC_CRC_ARG_VERSION_0, /* Current version */
	TEGRA_DC_CRC_ARG_VERSION_MAX,
};

enum tegra_dc_ext_crc_type {
	TEGRA_DC_EXT_CRC_TYPE_RG,
	TEGRA_DC_EXT_CRC_TYPE_OR,
	TEGRA_DC_EXT_CRC_TYPE_COMP,
	TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL,
	TEGRA_DC_EXT_CRC_TYPE_MAX
};

enum tegra_dc_ext_crc_input_data {
	TEGRA_DC_EXT_CRC_INPUT_DATA_FULL_FRAME,
	TEGRA_DC_EXT_CRC_INPUT_DATA_ACTIVE_DATA,
	TEGRA_DC_EXT_CRC_INPUT_DATA_MAX
};

/* For programming convenience, the enum values are tied to bit values directly
 * So, please do not reorder
 */
enum tegra_dc_ext_crc_sor_data {
	TEGRA_DC_EXT_CRC_SOR_DATA_ACTIVE_RASTER = 0,
	TEGRA_DC_EXT_CRC_SOR_DATA_COMPLETE_RASTER = 1,
	TEGRA_DC_EXT_CRC_SOR_DATA_NON_ACTIVE_RASTER = 2
};

/* For programming convenience, the enum values are tied to bit values directly
 * So, please do not reorder
 */
enum tegra_dc_ext_crc_sor_stage {
	TEGRA_DC_EXT_CRC_SOR_STAGE_PRE_SERIALIZE = 0,
	TEGRA_DC_EXT_CRC_SOR_STAGE_POST_DESERIALIZE = 1
};

struct tegra_dc_ext_crc_sor_params {
	enum tegra_dc_ext_crc_sor_data data;
	enum tegra_dc_ext_crc_sor_stage stage;
	__u8 reserved[16];
} __attribute__((__packed__));

/* The structure is extensible to other OR types as well, example DSI */
struct tegra_dc_ext_crc_or_params {
	enum tegra_dc_ext_control_output_type out_type;
	union {
		__u8 data[25];
		struct tegra_dc_ext_crc_sor_params sor_params;
	};
	__u8 reserved[16];
} __attribute__((__packed__));

#define TEGRA_DC_EXT_MAX_REGIONS 9

/* tegra_dc_ext_crc_region - A region of the display frame to calculate CRC
 *                           over. Currently, 9 regions are supported. The
 *                           regions need to be programmed so that they do not
 *                           overlap and are within the rastor
 * @id           - valid range is [0, 8]
 * @x            - X coordinate of the point where region begins
 * @y            - Y coordinate of the point where region begins
 * @w            - Width of the region
 * @h            - Height of the region
 * @reserved     - Easier way to extend the data structure
 */
struct tegra_dc_ext_crc_region {
	__u8 id;
	__u16 x;
	__u16 y;
	__u16 w;
	__u16 h;
	__u8 reserved[32];
} __attribute__((__packed__));

/*
 * tegra_dc_ext_crc_conf - Configuration data to communicate between kernel
 *                         and userspace
 * @type       - The block generating the CRC. See TEGRA_DC_EXT_CRC_TYPE_*
 * @input_data - Provides an option to collect CRCs over just the active area
 *               of the frame, or the blank areas and sync pulses as well. Only
 *               valid for ENABLE IOCTL and for TEGRA_DC_EXT_CRC_TYPE_RG/COMP,
 *               and is ignored for the other TEGRA_DC_EXT_CRC_* IOCTLs.
 * @region     - The region of the frame to do CRC calculations over. Only
 *               valid for TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL, and ignored for
 *               the rest of configuration types
 *               For GET IOCTL, request CRC of a specific region via region ID
 *               For DIS IOCTL, disable CRC calculations over a specific region
 *               mentioned using region ID
 *               For EN IOCTL, program or modify the parameters of a specific
 *               region.
 * @or_params -  Configuration parameters for different Output Resources. Only
 *               valid for ENABLE IOCTL and for TEGRA_DC_EXT_CRC_TYPE_OR.
 *               Ignored for other combinations of configuration types and
 *               IOCTLs
 * @crc        - For GET IOCTL, the kernel space sets the valid field and
 *               returns the CRC value via the val field.
 *               For EN IOCTL, if set to non zero values, -ENOTSUPP is returned,
 *               since programming golden CRC registers is yet to be supported.
 *               For DIS IOCTL, the field is ignored.
 * @reserved   - Easier way to extend the data structure
 */
struct tegra_dc_ext_crc_conf {
	enum tegra_dc_ext_crc_type type;
	union { /* tegra_dc_ext_crc_conf_params */
		__u8 data8[45];
		enum tegra_dc_ext_crc_input_data input_data; /* RG/COMP */
		struct tegra_dc_ext_crc_region region;       /* RG_REGIONAL */
		struct tegra_dc_ext_crc_or_params or_params; /* OR */
	};
	struct tegra_dc_ext_crc {
		__u8 valid; /* A boolean with 0/1 the only valid values */
		__u32 val;
	} crc;
	__u8 reserved[32];
} __attribute__((__packed__));

/*
 * tegra_dc_ext_crc_arg - The argument to EN/DIS/GET CRC IOCTLs
 * @magic     - Magic bytes 'TCRC'
 * @version   - In case the structure needs to change in future
 * @num_conf  - Num of valid configuration data structures
 *              For programming multiple regions (see
 *              TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL), the client needs to send a
 *              separate @conf object for each region
 * @conf      - Pointer to an array of configuration data structures
 *              tegra_dc_ext_crc_conf
 * @flip_id   - Flip ID for which CRC GET request is issued.
 *              flip_id is only valid for GET IOCTL, and a don't care for
 *              the rest
 *              If set to U64_MAX, the get IOCTL shall return the CRCs
 *              corresponding to the most recently programmed flip over the
 *              IOCTL interface, or return -EAGAIN if no flips have been
 *              programmed yet.
 *              If the flip corresponds to a cursor mode flip or one that is
 *              not synchronized along VSYNC pulse boundary, the context will
 *              block forever since such flips have no CRCs associated with
 *              them
 * @reserved  - Easier way to extend the data structure
 */
struct tegra_dc_ext_crc_arg {
	__u8 magic[4];
	enum tegra_dc_ext_crc_arg_version version;
	__u8 num_conf;
	__u64 __user conf;
	__u64 flip_id;
	__u8 reserved[32]; /* unused - must be 0 */
} __attribute__((__packed__));

#define TEGRA_DC_EXT_CONTROL_GET_NUM_OUTPUTS \
	_IOR('C', 0x00, __u32)
#define TEGRA_DC_EXT_CONTROL_GET_OUTPUT_PROPERTIES \
	_IOWR('C', 0x01, struct tegra_dc_ext_control_output_properties)
#define TEGRA_DC_EXT_CONTROL_GET_OUTPUT_EDID \
	_IOWR('C', 0x02, struct tegra_dc_ext_control_output_edid)
#define TEGRA_DC_EXT_CONTROL_SET_EVENT_MASK \
	_IOW('C', 0x03, __u32)
#define TEGRA_DC_EXT_CONTROL_GET_CAPABILITIES \
	_IOR('C', 0x04, struct tegra_dc_ext_control_capabilities)
#define TEGRA_DC_EXT_CONTROL_SCRNCAPT_PAUSE \
	_IOWR('C', 0x05, struct tegra_dc_ext_control_scrncapt_pause)
#define TEGRA_DC_EXT_CONTROL_SCRNCAPT_RESUME \
	_IOW('C', 0x06, struct tegra_dc_ext_control_scrncapt_resume)
#define TEGRA_DC_EXT_CONTROL_GET_FRAME_LOCK_PARAMS\
	_IOW('C', 0x07, struct tegra_dc_ext_control_scrncapt_resume)
#define TEGRA_DC_EXT_CONTROL_SET_FRAME_LOCK_PARAMS\
	_IOW('C', 0x08, struct tegra_dc_ext_control_scrncapt_resume)
#define TEGRA_DC_EXT_CONTROL_GET_CAP_INFO \
	_IOWR('C', 0x09, struct tegra_dc_ext_get_cap_info)

#endif /* __UAPI_TEGRA_DC_EXT_H */
