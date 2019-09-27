/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAMRTC_CAPTURE_H
#define INCLUDE_CAMRTC_CAPTURE_H

#include "camrtc-common.h"

#pragma GCC diagnostic error "-Wpadded"
#define __CAPTURE_IVC_ALIGN __aligned(8)
#define __CAPTURE_DESCRIPTOR_ALIGN __aligned(64)

typedef uint64_t iova_t __CAPTURE_IVC_ALIGN;

#define SYNCPOINT_ID_INVALID	U32_C(0)
#define GOS_INDEX_INVALID	U8_C(0xFF)

#pragma GCC diagnostic warning "-Wdeprecated-declarations"
#define CAMRTC_DEPRECATED __attribute__((deprecated))

/*Status Fence Support*/
#define STATUS_FENCE_SUPPORT

typedef struct syncpoint_info {
	uint32_t id;
	uint32_t threshold;	/* When storing a fence */
	uint8_t gos_sid;
	uint8_t gos_index;
	uint16_t gos_offset;
	uint32_t pad_;
	iova_t shim_addr;
} syncpoint_info_t __CAPTURE_IVC_ALIGN;

/*
 * The size for each unit includes the standard ISP5 HW stats
 * header size.
 *
 * Size break down for each unit.
 *  FB = 32 byte header + (256 x 4) bytes. FB has 256 windows with 4 bytes
 *       of stats data per window.
 *  FM = 32 byte header + (64 x 64 x 2 x 4) bytes. FM can have 64 x 64 windows
 *       with each windows having 2 bytes of data for each color channel.
 *  AFM = 32 byte header + 8 byte statistics data per ROI.
 *  LAC = 32 byte header + ( (32 x 32) x ((4 + 2 + 2) x 4) )
 *        Each ROI has 32x32 windows with each window containing 8
 *        bytes of data per color channel.
 *  Hist = Header + (256 x 4 x 4) bytes since Hist unit has 256 bins and
 *         each bin collects 4 byte data for each color channel + 4 Dwords for
 *         excluded pixel count due to elliptical mask per color channel.
 *  Pru = 32 byte header + (8 x 4) bytes for bad pixel count and accumulated
 *        pixel adjustment for pixels both inside and outside the ROI.
 *  LTM = 32 byte header + (128 x 4) bytes for histogram data + (8 x 8 x 4 x 2)
 *        bytes for soft key average and count. Soft key statistics are
 *        collected by dividing the frame into a 8x8 array region.
 */

#define ISP5_STATS_HW_HEADER_SIZE    (32UL)
#define ISP5_STATS_FB_MAX_SIZE       (1056UL)
#define ISP5_STATS_FM_MAX_SIZE       (32800UL)
#define ISP5_STATS_AFM_ROI_MAX_SIZE  (40UL)
#define ISP5_STATS_LAC_ROI_MAX_SIZE  (32800UL)
#define ISP5_STATS_HIST_MAX_SIZE     (4144UL)
#define ISP5_STATS_OR_MAX_SIZE       (64UL)
#define ISP5_STATS_LTM_MAX_SIZE      (1056UL)

/* Stats buffer addresses muse be aligned to 64 byte (ATOM) boundaries */

#define ISP5_ALIGN_STAT_OFFSET(_offset) (((uint32_t)(_offset) + 63UL) & ~(63UL))


#define ISP5_STATS_FB_OFFSET         (0)
#define ISP5_STATS_FM_OFFSET         (ISP5_STATS_FB_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_FB_MAX_SIZE))
#define ISP5_STATS_AFM_OFFSET        (ISP5_STATS_FM_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_FM_MAX_SIZE))
#define ISP5_STATS_LAC0_OFFSET       (ISP5_STATS_AFM_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_AFM_ROI_MAX_SIZE) * 8)
#define ISP5_STATS_LAC1_OFFSET       (ISP5_STATS_LAC0_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_LAC_ROI_MAX_SIZE) * 4)
#define ISP5_STATS_HIST0_OFFSET      (ISP5_STATS_LAC1_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_LAC_ROI_MAX_SIZE) * 4)
#define ISP5_STATS_HIST1_OFFSET      (ISP5_STATS_HIST0_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_HIST_MAX_SIZE))
#define ISP5_STATS_OR_OFFSET         (ISP5_STATS_HIST1_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_HIST_MAX_SIZE))
#define ISP5_STATS_LTM_OFFSET        (ISP5_STATS_OR_OFFSET + ISP5_ALIGN_STAT_OFFSET(ISP5_STATS_OR_MAX_SIZE))

#define ISP5_STATS_TOTAL_SIZE        (ISP5_STATS_LTM_OFFSET + ISP5_STATS_LTM_MAX_SIZE)

#define ISP_NUM_GOS_TABLES	8U

#define VI_NUM_GOS_TABLES	12U
#define VI_NUM_ATOMP_SURFACES	4
#define VI_NUM_STATUS_SURFACES	1

/* Generic */
#define VI_ATOMP_SURFACE0	0
#define VI_ATOMP_SURFACE1	1
#define VI_ATOMP_SURFACE2	2

/* Sensor embedded data */
#define VI_ATOMP_SURFACE_EMBEDDED 3

/* RAW */
#define VI_ATOMP_SURFACE_MAIN	VI_ATOMP_SURFACE0
/* PDAF pixels */
#define VI_ATOMP_SURFACE_PDAF	VI_ATOMP_SURFACE1

/* YUV */
#define VI_ATOMP_SURFACE_Y	VI_ATOMP_SURFACE0
#define	VI_ATOMP_SURFACE_UV	VI_ATOMP_SURFACE1 /* semi-planar */
#define	VI_ATOMP_SURFACE_U	VI_ATOMP_SURFACE1 /* planar */
#define	VI_ATOMP_SURFACE_V	VI_ATOMP_SURFACE2 /* planar */

/* SLVS-EC */
#define SLVSEC_STREAM_DISABLED	U8_C(0xFF)

/**
 * Describes RTCPU side resources for a capture pipe-line.
 *
 * The following parameters describe the capture descriptor ring buffer.
 *
 * @param requests: base address of a memory mapped ring buffer
 *		    containing capture requests. The size of the
 *		    buffer is queue_depth * request_size.
 * @param queue_depth: number of capture requests in the @a requests queue
 * @param request_size: size of the buffer reserved for each capture request.
 *
 * The following attributes indicate what resources need to be
 * allocated for the capture channel:
 *
 * @param channel_flags: a bitmask describing the set of non-shareable
 * HW resources that the capture channel will need. These HW resources
 * will be assigned to the new capture channel and will be owned by the
 * channel until it is released with CAPTURE_CHANNEL_RELEASE_REQ.
 *
 * The HW resources that can be assigned to a channel include a VI
 * channel, ISPBUF A/B interface (T18x only), Focus Metric Lite module
 * (FML).
 *
 * VI channels can have different capabilities. The flags are checked
 * against the VI channel capabilities to make sure the allocated VI
 * channel meets the requirements.  The following flags are defined:
 * <dl>
 * <dt>VIDEO:	<dd>Channel takes input from Video Interface (VI)
 * <dt>RAW:	<dd>Channel supports RAW Bayer output
 * <dt>PLANAR:	<dd>Channel supports planar YUV output
 * <dt>SEMI_PLANAR: <dd>Channel supports semi-planar YUV output
 * <dt>PDAF:	<dd>Channel supports phase-detection auto-focus
 * <dt>FMLITE:	<dd>Channel outputs to Focus Metric Lite module (FML)
 * <dt>EMBDATA: <dd>Channel outputs sensor embedded data
 * <dt>ISPA:	<dd>Channel outputs to ISPA
 * <dt>ISPB:	<dd>Channel outputs to ISPB
 * <dt>ISP_DIRECT: <dd>Channel outputs directly to selected ISP (ISO mode)
 * <dt>ISPSW:	<dd>Channel outputs to software ISP (reserved)
 * <dt>ENABLE_ERROR_ACTIONS_MASKS:  <dd>Channel handles errors according to
 *     error_mask_correctable and error_mask_uncorrectable.
 *     This flag take precedence over RESET_ON_ERROR.
 * <dt>RESET_ON_ERROR:	<dd>Channel treats all errors as uncorrectable
 *	and requires reset for recovery. This flag is ignored if
 *	ENABLE_ERROR_ACTIONS_MASKS is set.
 *
 * </dl>
 *
 * @param vi_channel_mask  A bit mask indicating which VI channels to
 * consider for allocation. This allows the client VM to statically
 * partition VI channels for its own purposes. The RTCPU will enforce
 * any partitioning between VMs.
 */
struct capture_channel_config {
	uint32_t channel_flags;

#define CAPTURE_CHANNEL_FLAG_VIDEO		U32_C(0x0001)
#define CAPTURE_CHANNEL_FLAG_RAW		U32_C(0x0002)
#define CAPTURE_CHANNEL_FLAG_PLANAR		U32_C(0x0004)
#define CAPTURE_CHANNEL_FLAG_SEMI_PLANAR	U32_C(0x0008)
#define CAPTURE_CHANNEL_FLAG_PDAF		U32_C(0x0010)
#define CAPTURE_CHANNEL_FLAG_FMLITE		U32_C(0x0020)
#define CAPTURE_CHANNEL_FLAG_EMBDATA		U32_C(0x0040)
#define CAPTURE_CHANNEL_FLAG_ISPA		U32_C(0x0080)
#define CAPTURE_CHANNEL_FLAG_ISPB		U32_C(0x0100)
#define CAPTURE_CHANNEL_FLAG_ISP_DIRECT		U32_C(0x0200)
#define CAPTURE_CHANNEL_FLAG_ISPSW		U32_C(0x0400)
#define CAPTURE_CHANNEL_FLAG_RESET_ON_ERROR	U32_C(0x0800)
#define CAPTURE_CHANNEL_FLAG_LINETIMER		U32_C(0x1000)
#define CAPTURE_CHANNEL_FLAG_SLVSEC		U32_C(0x2000)
#define CAPTURE_CHANNEL_FLAG_ENABLE_ERROR_ACTIONS_MASKS	U32_C(0x4000)

	uint32_t channel_id;	/* rtcpu internal - set to zero */
	uint64_t vi_channel_mask;
	iova_t requests;
	uint32_t queue_depth;
	uint32_t request_size;

	uint8_t slvsec_stream_main;
	uint8_t slvsec_stream_sub;
	uint16_t reserved1;

#define HAVE_VI_GOS_TABLES
	/*
	 * GoS tables can only be programmed when there are no
	 * active channels. For subsequent channels we check that
	 * the channel configuration matches with the active
	 * configuration.
	 */
	uint32_t num_vi_gos_tables;
	iova_t vi_gos_tables[VI_NUM_GOS_TABLES];

	struct syncpoint_info progress_sp;
	struct syncpoint_info embdata_sp;
	struct syncpoint_info linetimer_sp;

	/**
	 * Errors defined in error_mask_uncorrectable are reported and stop channel.
	 * User reset is required for channel recovery.
	 */
	uint32_t error_mask_uncorrectable;
	/**
	 * Errors defined in error_mask_correctable are reported, but
	 * don't stop the channel.
	 */
	uint32_t error_mask_correctable;
#define CAPTURE_CHANNEL_ERROR_ERROR_EMBED_INCOMPLETE	(U32_C(1) << 21)
#define CAPTURE_CHANNEL_ERROR_INCOMPLETE		(U32_C(1) << 20)
#define CAPTURE_CHANNEL_ERROR_STALE_FRAME		(U32_C(1) << 19)
#define CAPTURE_CHANNEL_ERROR_COLLISION			(U32_C(1) << 18)
#define CAPTURE_CHANNEL_ERROR_FORCE_FE			(U32_C(1) << 17)
#define CAPTURE_CHANNEL_ERROR_LOAD_FRAMED		(U32_C(1) << 16)
#define CAPTURE_CHANNEL_ERROR_DTYPE_MISMATCH		(U32_C(1) << 15)
#define CAPTURE_CHANNEL_ERROR_EMBED_INFRINGE		(U32_C(1) << 14)
#define CAPTURE_CHANNEL_ERROR_EMBED_LONG_LINE		(U32_C(1) << 13)
#define CAPTURE_CHANNEL_ERROR_EMBED_SPURIOUS		(U32_C(1) << 12)
#define CAPTURE_CHANNEL_ERROR_EMBED_RUNAWAY		(U32_C(1) << 11)
#define CAPTURE_CHANNEL_ERROR_EMBED_MISSING_LE		(U32_C(1) << 10)
#define CAPTURE_CHANNEL_ERROR_PIXEL_SHORT_LINE		(U32_C(1) << 9)
#define CAPTURE_CHANNEL_ERROR_PIXEL_LONG_LINE		(U32_C(1) << 8)
#define CAPTURE_CHANNEL_ERROR_PIXEL_SPURIOUS		(U32_C(1) << 7)
#define CAPTURE_CHANNEL_ERROR_PIXEL_RUNAWAY		(U32_C(1) << 6)
#define CAPTURE_CHANNEL_ERROR_PIXEL_MISSING_LE		(U32_C(1) << 5)

} __CAPTURE_IVC_ALIGN;

struct vi_channel_config {
	/* Flags */
	unsigned dt_enable:1;
	unsigned embdata_enable:1;
	unsigned flush_enable:1;
	unsigned flush_periodic:1;
	unsigned line_timer_enable:1;
	unsigned line_timer_periodic:1;
	unsigned pixfmt_enable:1;
	unsigned pixfmt_wide_enable:1;
	unsigned pixfmt_wide_endian:1;
	unsigned pixfmt_pdaf_replace_enable:1;
	unsigned ispbufa_enable:1;
	unsigned ispbufb_enable:1;
	unsigned fmlite_enable:1;
	unsigned compand_enable:1;
	unsigned __pad_flags:18;

	/* vi channel selector */
	struct {
		uint8_t datatype;
		uint8_t datatype_mask;
		uint8_t stream;
		uint8_t stream_mask;
		uint16_t vc;
		uint16_t vc_mask;
		uint16_t frameid;
		uint16_t frameid_mask;
		uint16_t dol;
		uint16_t dol_mask;
	} match;

	/* Misc control */
	uint8_t dol_header_sel;
	uint8_t dt_override;
	uint8_t dpcm_mode;
	uint8_t __pad_dol_dt_dpcm;

	/* frame size and crop */
	struct vi_frame_config {
		uint16_t frame_x;
		uint16_t frame_y;
		uint32_t embed_x;
		uint32_t embed_y;
		struct {
			uint16_t x;
			uint16_t y;
		} skip;
		struct {
			uint16_t x;
			uint16_t y;
		} crop;
	} frame;

	/* Flush timer/slice height */
	uint16_t flush;
	uint16_t flush_first;

	/* Line timer trip-line */
	uint16_t line_timer;
	uint16_t line_timer_first;

	/* Pixel formatter */
	struct {
		uint16_t format;
		uint16_t __pad;
		struct {
			uint16_t crop_left;
			uint16_t crop_right;
			uint16_t crop_top;
			uint16_t crop_bottom;
			uint16_t replace_crop_left;
			uint16_t replace_crop_right;
			uint16_t replace_crop_top;
			uint16_t replace_crop_bottom;
			uint16_t last_pixel_x;
			uint16_t last_pixel_y;
			uint16_t replace_value;
			uint8_t format;
			uint8_t __pad_pdaf;
		} pdaf;
	} pixfmt;

	/* Pixel DPCM */
	struct {
		uint16_t strip_width;
		uint16_t strip_overfetch;

		/* Not for T186 or earlier */
		uint16_t chunk_first;
		uint16_t chunk_body;
		uint16_t chunk_body_count;
		uint16_t chunk_penultimate;
		uint16_t chunk_last;
		uint16_t __pad;
		uint32_t clamp_high;
		uint32_t clamp_low;
	} dpcm;

	/* Atom packer */
	struct {
		struct {
			uint32_t offset;
			uint32_t offset_hi;
		} surface[VI_NUM_ATOMP_SURFACES];
		uint32_t surface_stride[VI_NUM_ATOMP_SURFACES];
		uint32_t dpcm_chunk_stride;
	} atomp;

	uint16_t __pad[2];

} __CAPTURE_IVC_ALIGN;

struct engine_status_surface {
	uint32_t offset;
	uint32_t offset_hi;
} __CAPTURE_IVC_ALIGN;

struct capture_status {
	uint8_t src_stream;
	uint8_t virtual_channel;
	uint16_t frame_id;
	uint32_t status;

#define CAPTURE_STATUS_UNKNOWN			U32_C(0)
#define CAPTURE_STATUS_SUCCESS			U32_C(1)
#define CAPTURE_STATUS_CSIMUX_FRAME		U32_C(2)
#define CAPTURE_STATUS_CSIMUX_STREAM		U32_C(3)
#define CAPTURE_STATUS_CHANSEL_FAULT		U32_C(4)
#define CAPTURE_STATUS_CHANSEL_FAULT_FE		U32_C(5)
#define CAPTURE_STATUS_CHANSEL_COLLISION	U32_C(6)
#define CAPTURE_STATUS_CHANSEL_SHORT_FRAME	U32_C(7)
#define CAPTURE_STATUS_ATOMP_PACKER_OVERFLOW	U32_C(8)
#define CAPTURE_STATUS_ATOMP_FRAME_TRUNCATED	U32_C(9)
#define CAPTURE_STATUS_ATOMP_FRAME_TOSSED	U32_C(10)
#define CAPTURE_STATUS_ISPBUF_FIFO_OVERFLOW	U32_C(11)
#define CAPTURE_STATUS_SYNC_FAILURE		U32_C(12)
#define CAPTURE_STATUS_NOTIFIER_BACKEND_DOWN	U32_C(13)
#define CAPTURE_STATUS_FALCON_ERROR		U32_C(14)
#define CAPTURE_STATUS_CHANSEL_NOMATCH		U32_C(15)
#define CAPTURE_STATUS_ABORTED			U32_C(16)

	uint64_t sof_timestamp;
	uint64_t eof_timestamp;
	uint32_t err_data;

/* Channel encountered uncorrectable error and must be reset */
#define CAPTURE_STATUS_FLAG_CHANNEL_IN_ERROR			U32_C(1U << 0)

/*
 * Spurious data was received before frame start.
 * Can be badly corrupted frame or some random bits.
 * This error doesn't have effect on captured frame
 */
#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_STREAM_SPURIOUS	U32_C(1U << 1)

/*
 * Illegal data packet was encountered and dropped by CSIMUX.
 * This error may have no effect on capture result or trigger other error if
 * frame got corrupted.
 */
#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_FIFO_BADPKT		U32_C(1U << 2)

#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_FRAME_FORCE_FE		U32_C(1U << 3)
#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_FRAME_ECC_SINGLE_BIT_ERR	U32_C(1U << 4)
#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_FRAME				U32_C(1U << 5)
#define CAPTURE_STATUS_FLAG_ERROR_CSIMUX_FRAME_CSI_FAULT		U32_C(1U << 6)

/*
 * One or more frames could not be matched and got lost before captured
 * frame.
 * This error doesn't have effect on captured frame
 */
#define CAPTURE_STATUS_FLAG_ERROR_CHANSEL_NO_MATCH		U32_C(1U << 7)

/* Frame not finished */
#define CAPTURE_STATUS_FLAG_ERROR_ATOMP_FRAME_TRUNCATED		U32_C(1U << 8)

/* Frame data not written */
#define CAPTURE_STATUS_FLAG_ERROR_ATOMP_FRAME_TOSSED		U32_C(1U << 9)

	uint32_t flags;
} __CAPTURE_IVC_ALIGN;

#define VI_AFM_NUM_ROI			8
#define VI_AFM_NUM_TRANSFER_KNOTS	11

struct vi_fmlite_config {
	uint32_t vfm_prog;
	uint32_t vfm_ctrl;
	uint32_t vfm_black_level;
	uint32_t vfm_hdr_sample_map;
	uint32_t vfm_hdr_scale;
	uint32_t vfm_hdr_sat;
	uint32_t vfm_h_pi;
	uint32_t vfm_v_pi;
	uint32_t vfm_offset;
	uint32_t vfm_size;
	uint32_t vfm_hf_c0;
	uint32_t vfm_hf_c1;
	uint32_t vfm_hf_c2;
	uint32_t vfm_vf_c0;
	uint32_t vfm_vf_c1;
	uint32_t vfm_vf_c2;
	uint32_t vfm_vf_c3;
	uint32_t vfm_vf_c4;
	uint32_t ctrl;
	uint32_t color;
	uint32_t transfer_slope;
	uint32_t transfer_x;
	uint32_t transfer_y;
	uint32_t transfer_cubic_ctrl;
	uint32_t transfer_knots[VI_AFM_NUM_TRANSFER_KNOTS];
	uint32_t roi_pos[VI_AFM_NUM_ROI];
	uint32_t roi_size[VI_AFM_NUM_ROI];
	uint32_t trap_en;
	uint32_t hstart[VI_AFM_NUM_ROI];
	uint32_t vstart[VI_AFM_NUM_ROI];
	uint32_t slope[VI_AFM_NUM_ROI];
	uint32_t coeff01;
	uint32_t coeff23;
	uint32_t coeff45;
	uint32_t error;
} __CAPTURE_IVC_ALIGN;

struct vi_fmlite_result {					/* 72 bytes */
	uint32_t error;
	uint32_t __pad;
	uint64_t roi[VI_AFM_NUM_ROI];
} __CAPTURE_IVC_ALIGN;

/*
 * The compand configuration describes a piece-wise linear
 * tranformation function used by the VI companding module.
 */
#define VI_NUM_COMPAND_KNEEPTS 10
struct vi_compand_config {
	uint32_t base[VI_NUM_COMPAND_KNEEPTS];
	uint32_t scale[VI_NUM_COMPAND_KNEEPTS];
	uint32_t offset[VI_NUM_COMPAND_KNEEPTS];
} __CAPTURE_IVC_ALIGN;

/*
 * The phase-detection auto-focus data consists of special pixels that
 * will be extracted from a frame and written to a separate
 * surface. The PDAF pattern is shared by all capture channels and
 * should be configured before enabling PDAF pixel extraction for a
 * specific capture.
 *
 * Pixel { x, y } will be ouput to the PDAF surface (surface1) if the
 * bit at position (x % 32) in pattern[y % 32] is set.
 *
 * Pixel { x, y } in the main output surface (surface0) will be
 * replaced by a default pixel value if the bit at position (x % 32)
 * in pattern_replace[y % 32] is set.
 */
#define VI_PDAF_PATTERN_SIZE 32
struct vi_pdaf_config {
	uint32_t pattern[VI_PDAF_PATTERN_SIZE];
	uint32_t pattern_replace[VI_PDAF_PATTERN_SIZE];
} __CAPTURE_IVC_ALIGN;


/*
 * Configuration for VI SYNGEN unit.
 */
struct vi_syncgen_config {
	uint32_t hclk_div;
	uint8_t hclk_div_fmt;
	uint8_t xhs_width;
	uint8_t xvs_width;
	uint8_t xvs_to_xhs_delay;
	uint16_t cvs_interval;
	uint16_t __pad1;
	uint32_t __pad2;
} __CAPTURE_IVC_ALIGN;


struct capture_descriptor {
	uint32_t sequence;
	uint32_t capture_flags;

#define CAPTURE_FLAG_STATUS_REPORT_ENABLE	(U32_C(1) << 0)
#define CAPTURE_FLAG_ERROR_REPORT_ENABLE	(U32_C(1) << 1)

	uint16_t frame_start_timeout;		/**< Timeout in milliseconds */
	uint16_t frame_completion_timeout;	/**< Timeout in milliseconds */

#define CAPTURE_PREFENCE_ARRAY_SIZE		2

	uint32_t prefence_count;
	struct syncpoint_info prefence[CAPTURE_PREFENCE_ARRAY_SIZE];

	struct vi_channel_config ch_cfg;
	struct vi_fmlite_config fm_cfg;

	/* Result record – written by Falcon */
	struct engine_status_surface engine_status;

	/* FMLITE result – written by RTCPU */
	struct vi_fmlite_result fm_result;

	/* Result record – written by RTCPU */
	struct capture_status status;

	uint32_t __pad[12];
} __CAPTURE_DESCRIPTOR_ALIGN;

/* Event data used for event injection */
struct event_inject_msg {
	uint32_t tag; /* UMD populates with capture status events. RCE converts to reg offset */
	uint32_t stamp; /* Timestamp of event */
	uint32_t data; /* Bits [0:31] of event data */
	uint32_t data_ext; /* Bits [32:63] of event data */
};

/**
 * NvPhy attributes
 */

/* NvPhy types */
#define NVPHY_TYPE_CSI		U32_C(0)
#define NVPHY_TYPE_SLVSEC	U32_C(1)

/**
 * NVCSI attributes
 */

/* NVCSI ports */
#define NVCSI_PORT_A		U32_C(0x0)
#define NVCSI_PORT_B		U32_C(0x1)
#define NVCSI_PORT_C		U32_C(0x2)
#define NVCSI_PORT_D		U32_C(0x3)
#define NVCSI_PORT_E		U32_C(0x4)
#define NVCSI_PORT_F		U32_C(0x5)
#define NVCSI_PORT_G		U32_C(0x6)
#define NVCSI_PORT_H		U32_C(0x7)
#define NVCSI_PORT_UNSPECIFIED	U32_C(0xFFFFFFFF)

/* NVCSI streams */
#define NVCSI_STREAM_0		U32_C(0x0)
#define NVCSI_STREAM_1		U32_C(0x1)
#define NVCSI_STREAM_2		U32_C(0x2)
#define NVCSI_STREAM_3		U32_C(0x3)
#define NVCSI_STREAM_4		U32_C(0x4)
#define NVCSI_STREAM_5		U32_C(0x5)

/* NVCSI virtual channels */
#define NVCSI_VIRTUAL_CHANNEL_0		U32_C(0x0)
#define NVCSI_VIRTUAL_CHANNEL_1		U32_C(0x1)
#define NVCSI_VIRTUAL_CHANNEL_2		U32_C(0x2)
#define NVCSI_VIRTUAL_CHANNEL_3		U32_C(0x3)
#define NVCSI_VIRTUAL_CHANNEL_4		U32_C(0x4)
#define NVCSI_VIRTUAL_CHANNEL_5		U32_C(0x5)
#define NVCSI_VIRTUAL_CHANNEL_6		U32_C(0x6)
#define NVCSI_VIRTUAL_CHANNEL_7		U32_C(0x7)
#define NVCSI_VIRTUAL_CHANNEL_8		U32_C(0x8)
#define NVCSI_VIRTUAL_CHANNEL_9		U32_C(0x9)
#define NVCSI_VIRTUAL_CHANNEL_10	U32_C(0xA)
#define NVCSI_VIRTUAL_CHANNEL_11	U32_C(0xB)
#define NVCSI_VIRTUAL_CHANNEL_12	U32_C(0xC)
#define NVCSI_VIRTUAL_CHANNEL_13	U32_C(0xD)
#define NVCSI_VIRTUAL_CHANNEL_14	U32_C(0xE)
#define NVCSI_VIRTUAL_CHANNEL_15	U32_C(0xF)

/* NVCSI config flag */
#define NVCSI_CONFIG_FLAG_BRICK		(U32_C(1) << 0)
#define NVCSI_CONFIG_FLAG_CIL		(U32_C(1) << 1)
#define NVCSI_CONFIG_FLAG_ERROR		(U32_C(1) << 2)

/* Number of lanes/trios per brick */
#define NVCSI_BRICK_NUM_LANES	U32_C(4)
/* Number of override exception data types */
#define NVCSI_NUM_NOOVERRIDE_DT	U32_C(5)

/* NVCSI phy types */
#define NVCSI_PHY_TYPE_DPHY	U32_C(0)
#define NVCSI_PHY_TYPE_CPHY	U32_C(1)

/* NVCSI lane swizzles */
/* 00000 := A0 A1 B0 B1 -->  A0 A1 B0 B1 */
#define NVCSI_LANE_SWIZZLE_A0A1B0B1	U32_C(0x00)
/* 00001 := A0 A1 B0 B1 -->  A0 A1 B1 B0 */
#define NVCSI_LANE_SWIZZLE_A0A1B1B0	U32_C(0x01)
/* 00010 := A0 A1 B0 B1 -->  A0 B0 B1 A1 */
#define NVCSI_LANE_SWIZZLE_A0B0B1A1	U32_C(0x02)
/* 00011 := A0 A1 B0 B1 -->  A0 B0 A1 B1 */
#define NVCSI_LANE_SWIZZLE_A0B0A1B1	U32_C(0x03)
/* 00100 := A0 A1 B0 B1 -->  A0 B1 A1 B0 */
#define NVCSI_LANE_SWIZZLE_A0B1A1B0	U32_C(0x04)
/* 00101 := A0 A1 B0 B1 -->  A0 B1 B0 A1 */
#define NVCSI_LANE_SWIZZLE_A0B1B0A1	U32_C(0x05)
/* 00110 := A0 A1 B0 B1 -->  A1 A0 B0 B1 */
#define NVCSI_LANE_SWIZZLE_A1A0B0B1	U32_C(0x06)
/* 00111 := A0 A1 B0 B1 -->  A1 A0 B1 B0 */
#define NVCSI_LANE_SWIZZLE_A1A0B1B0	U32_C(0x07)
/* 01000 := A0 A1 B0 B1 -->  A1 B0 B1 A0 */
#define NVCSI_LANE_SWIZZLE_A1B0B1A0	U32_C(0x08)
/* 01001 := A0 A1 B0 B1 -->  A1 B0 A0 B1 */
#define NVCSI_LANE_SWIZZLE_A1B0A0B1	U32_C(0x09)
/* 01010 := A0 A1 B0 B1 -->  A1 B1 A0 B0 */
#define NVCSI_LANE_SWIZZLE_A1B1A0B0	U32_C(0x0A)
/* 01011 := A0 A1 B0 B1 -->  A1 B1 B0 A0 */
#define NVCSI_LANE_SWIZZLE_A1B1B0A0	U32_C(0x0B)
/* 01100 := A0 A1 B0 B1 -->  B0 A1 A0 B1 */
#define NVCSI_LANE_SWIZZLE_B0A1A0B1	U32_C(0x0C)
/* 01101 := A0 A1 B0 B1 -->  B0 A1 B1 A0 */
#define NVCSI_LANE_SWIZZLE_B0A1B1A0	U32_C(0x0D)
/* 01110 := A0 A1 B0 B1 -->  B0 A0 B1 A1 */
#define NVCSI_LANE_SWIZZLE_B0A0B1A1	U32_C(0x0E)
/* 01111 := A0 A1 B0 B1 -->  B0 A0 A1 B1 */
#define NVCSI_LANE_SWIZZLE_B0A0A1B1	U32_C(0x0F)
/* 10000 := A0 A1 B0 B1 -->  B0 B1 A1 A0 */
#define NVCSI_LANE_SWIZZLE_B0B1A1A0	U32_C(0x10)
/* 10001 := A0 A1 B0 B1 -->  B0 B1 A0 A1 */
#define NVCSI_LANE_SWIZZLE_B0B1A0A1	U32_C(0x11)
/* 10010 := A0 A1 B0 B1 -->  B1 A1 B0 A0 */
#define NVCSI_LANE_SWIZZLE_B1A1B0A0	U32_C(0x12)
/* 10011 := A0 A1 B0 B1 -->  B1 A1 A0 B0 */
#define NVCSI_LANE_SWIZZLE_B1A1A0B0	U32_C(0x13)
/* 10100 := A0 A1 B0 B1 -->  B1 B0 A0 A1 */
#define NVCSI_LANE_SWIZZLE_B1B0A0A1	U32_C(0x14)
/* 10101 := A0 A1 B0 B1 -->  B1 B0 A1 A0 */
#define NVCSI_LANE_SWIZZLE_B1B0A1A0	U32_C(0x15)
/* 10110 := A0 A1 B0 B1 -->  B1 A0 A1 B0 */
#define NVCSI_LANE_SWIZZLE_B1A0A1B0	U32_C(0x16)
/* 10111 := A0 A1 B0 B1 -->  B1 A0 B0 A1 */
#define NVCSI_LANE_SWIZZLE_B1A0B0A1	U32_C(0x17)

/* NVCSI D-phy polarity */
#define NVCSI_DPHY_POLARITY_NOSWAP	U32_C(0)
#define NVCSI_DPHY_POLARITY_SWAP	U32_C(1)

/* NVCSI C-phy polarity */
#define NVCSI_CPHY_POLARITY_ABC	U32_C(0x00) /* 000 := A B C --> A B C */
#define NVCSI_CPHY_POLARITY_ACB	U32_C(0x01) /* 001 := A B C --> A C B */
#define NVCSI_CPHY_POLARITY_BCA	U32_C(0x02) /* 010 := A B C --> B C A */
#define NVCSI_CPHY_POLARITY_BAC	U32_C(0x03) /* 011 := A B C --> B A C */
#define NVCSI_CPHY_POLARITY_CAB	U32_C(0x04) /* 100 := A B C --> C A B */
#define NVCSI_CPHY_POLARITY_CBA	U32_C(0x05) /* 101 := A B C --> C B A */

struct nvcsi_brick_config {
	/* Select PHY mode for both partitions */
	uint32_t phy_mode;
	/* Lane Swizzle control for Bricks.
	 * Valid in both C-PHY and D-PHY modes */
	uint32_t lane_swizzle;
	/* Lane polarity control. Value depends on PhyMode */
	uint8_t lane_polarity[NVCSI_BRICK_NUM_LANES];
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

struct nvcsi_cil_config {
	/* Number of data lanes used (0-4) */
	uint8_t num_lanes;
	/* LP bypass mode (boolean) */
	uint8_t lp_bypass_mode;
	/* Set MIPI THS-SETTLE timing */
	uint8_t t_hs_settle;
	/* Set MIPI TCLK-SETTLE timing */
	uint8_t t_clk_settle;
	/* NVCSI CIL clock rate [kHz] */
	uint32_t cil_clock_rate;
	/* MIPI clock rate for D-Phy. Symbol rate for C-Phy [kHz] */
	uint32_t mipi_clock_rate;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* NVCSI stream novc+vc error flags */
#define NVCSI_INTR_FLAG_STREAM_NOVC_ERR_PH_ECC_MULTI_BIT	(U32_C(1) << 0)
#define NVCSI_INTR_FLAG_STREAM_NOVC_ERR_PH_BOTH_CRC		(U32_C(1) << 1)
#define NVCSI_INTR_FLAG_STREAM_VC_ERR_PPFSM_TIMEOUT		(U32_C(1) << 2)
#define NVCSI_INTR_FLAG_STREAM_VC_ERR_PH_ECC_SINGLE_BIT		(U32_C(1) << 3)
#define NVCSI_INTR_FLAG_STREAM_VC_ERR_PD_CRC			(U32_C(1) << 4)
#define NVCSI_INTR_FLAG_STREAM_VC_ERR_PD_WC_SHORT		(U32_C(1) << 5)
#define NVCSI_INTR_FLAG_STREAM_VC_ERR_PH_SINGLE_CRC		(U32_C(1) << 6)

/* NVCSI phy/cil intr error flags */
#define NVCSI_INTR_FLAG_CIL_INTR_DPHY_ERR_CLK_LANE_CTRL		(U32_C(1) << 0)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_SOT_SB		(U32_C(1) << 1)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_SOT_MB		(U32_C(1) << 2)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_CTRL		(U32_C(1) << 3)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_RXFIFO_FULL	(U32_C(1) << 4)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_SOT_SB		(U32_C(1) << 5)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_SOT_MB		(U32_C(1) << 6)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_CTRL		(U32_C(1) << 7)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_RXFIFO_FULL	(U32_C(1) << 8)
#define NVCSI_INTR_FLAG_CIL_INTR_DPHY_DESKEW_CALIB_ERR_LANE0	(U32_C(1) << 9)
#define NVCSI_INTR_FLAG_CIL_INTR_DPHY_DESKEW_CALIB_ERR_LANE1	(U32_C(1) << 10)
#define NVCSI_INTR_FLAG_CIL_INTR_DPHY_DESKEW_CALIB_ERR_CTRL	(U32_C(1) << 11)
#define NVCSI_INTR_FLAG_CIL_INTR_DPHY_LANE_ALIGN_ERR		(U32_C(1) << 12)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_ESC_MODE_SYNC	(U32_C(1) << 13)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_ESC_MODE_SYNC	(U32_C(1) << 14)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR0_SOT_2LSB_FULL	(U32_C(1) << 15)
#define NVCSI_INTR_FLAG_CIL_INTR_DATA_LANE_ERR1_SOT_2LSB_FULL	(U32_C(1) << 16)

/* NVCSI phy/cil intr0 flags */
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_ERR_CLK_LANE_CTRL	(U32_C(1) << 0)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_SOT_SB		(U32_C(1) << 1)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_SOT_MB		(U32_C(1) << 2)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_CTRL		(U32_C(1) << 3)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_RXFIFO_FULL	(U32_C(1) << 4)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_SOT_SB		(U32_C(1) << 5)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_SOT_MB		(U32_C(1) << 6)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_CTRL		(U32_C(1) << 7)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_RXFIFO_FULL	(U32_C(1) << 8)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_SOT_2LSB_FULL	(U32_C(1) << 9)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_SOT_2LSB_FULL	(U32_C(1) << 10)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR0_ESC_MODE_SYNC	(U32_C(1) << 19)
#define NVCSI_INTR_FLAG_CIL_INTR0_DATA_LANE_ERR1_ESC_MODE_SYNC	(U32_C(1) << 20)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_DONE_LANE0	(U32_C(1) << 22)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_DONE_LANE1	(U32_C(1) << 23)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_DONE_CTRL	(U32_C(1) << 24)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_ERR_LANE0	(U32_C(1) << 25)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_ERR_LANE1	(U32_C(1) << 26)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_DESKEW_CALIB_ERR_CTRL	(U32_C(1) << 27)
#define NVCSI_INTR_FLAG_CIL_INTR0_DPHY_LANE_ALIGN_ERR		(U32_C(1) << 28)
#define NVCSI_INTR_FLAG_CIL_INTR0_CPHY_CLK_CAL_DONE_TRIO0	(U32_C(1) << 29)
#define NVCSI_INTR_FLAG_CIL_INTR0_CPHY_CLK_CAL_DONE_TRIO1	(U32_C(1) << 30)

/* NVCSI phy/cil intr1 flags */
#define NVCSI_INTR_FLAG_CIL_INTR1_DATA_LANE_ESC_CMD_REC0	(U32_C(1) << 0)
#define NVCSI_INTR_FLAG_CIL_INTR1_DATA_LANE_ESC_DATA_REC0	(U32_C(1) << 1)
#define NVCSI_INTR_FLAG_CIL_INTR1_DATA_LANE_ESC_CMD_REC1	(U32_C(1) << 2)
#define NVCSI_INTR_FLAG_CIL_INTR1_DATA_LANE_ESC_DATA_REC1	(U32_C(1) << 3)
#define NVCSI_INTR_FLAG_CIL_INTR1_REMOTERST_TRIGGER_INT0	(U32_C(1) << 4)
#define NVCSI_INTR_FLAG_CIL_INTR1_ULPS_TRIGGER_INT0		(U32_C(1) << 5)
#define NVCSI_INTR_FLAG_CIL_INTR1_LPDT_INT0			(U32_C(1) << 6)
#define NVCSI_INTR_FLAG_CIL_INTR1_REMOTERST_TRIGGER_INT1	(U32_C(1) << 7)
#define NVCSI_INTR_FLAG_CIL_INTR1_ULPS_TRIGGER_INT1		(U32_C(1) << 8)
#define NVCSI_INTR_FLAG_CIL_INTR1_LPDT_INT1			(U32_C(1) << 9)
#define NVCSI_INTR_FLAG_CIL_INTR1_DPHY_CLK_LANE_ULPM_REQ	(U32_C(1) << 10)

/* NVCSI intr config bit masks */
#define NVCSI_INTR_CONFIG_MASK_HOST1X		U32_C(0x1)
#define NVCSI_INTR_CONFIG_MASK_STATUS2VI	U32_C(0xffff)
#define NVCSI_INTR_CONFIG_MASK_STREAM_NOVC	U32_C(0x3)
#define NVCSI_INTR_CONFIG_MASK_STREAM_VC	U32_C(0x7c)
#define NVCSI_INTR_CONFIG_MASK_CIL_INTR		U32_C(0x1ffff)
#define NVCSI_INTR_CONFIG_MASK_CIL_INTR0	U32_C(0x7fd807ff)
#define NVCSI_INTR_CONFIG_MASK_CIL_INTR1	U32_C(0x7ff)

/* NVCSI intr config bit shifts */
#define NVCSI_INTR_CONFIG_SHIFT_STREAM_NOVC	U32_C(0x0)
#define NVCSI_INTR_CONFIG_SHIFT_STREAM_VC	U32_C(0x2)

struct nvcsi_error_config {
	/* Mask Host1x timeout intr*/
	uint32_t host1x_intr_mask;
	uint32_t host1x_intr_type;
	/* Mask status2vi NOTIFY reporting */
	uint32_t status2vi_notify_mask;
	/* Mask stream intrs */
	uint32_t stream_intr_mask;
	uint32_t stream_intr_type;
	/* Mask cil intrs */
	uint32_t cil_intr_mask;
	uint32_t cil_intr_type;
	/* Mask cil intr0/intr1 intrs */
	uint32_t cil_intr0_mask;
	uint32_t cil_intr1_mask;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

/* NVCSI datatypes */
#define NVCSI_DATATYPE_UNSPECIFIED	U32_C(0)
#define NVCSI_DATATYPE_YUV420_8		U32_C(24)
#define NVCSI_DATATYPE_YUV420_10	U32_C(25)
#define NVCSI_DATATYPE_LEG_YUV420_8	U32_C(26)
#define NVCSI_DATATYPE_YUV420CSPS_8	U32_C(28)
#define NVCSI_DATATYPE_YUV420CSPS_10	U32_C(29)
#define NVCSI_DATATYPE_YUV422_8		U32_C(30)
#define NVCSI_DATATYPE_YUV422_10	U32_C(31)
#define NVCSI_DATATYPE_RGB444		U32_C(32)
#define NVCSI_DATATYPE_RGB555		U32_C(33)
#define NVCSI_DATATYPE_RGB565		U32_C(34)
#define NVCSI_DATATYPE_RGB666		U32_C(35)
#define NVCSI_DATATYPE_RGB888		U32_C(36)
#define NVCSI_DATATYPE_RAW6		U32_C(40)
#define NVCSI_DATATYPE_RAW7		U32_C(41)
#define NVCSI_DATATYPE_RAW8		U32_C(42)
#define NVCSI_DATATYPE_RAW10		U32_C(43)
#define NVCSI_DATATYPE_RAW12		U32_C(44)
#define NVCSI_DATATYPE_RAW14		U32_C(45)
#define NVCSI_DATATYPE_RAW16		U32_C(46)
#define NVCSI_DATATYPE_RAW20		U32_C(47)
#define NVCSI_DATATYPE_USER_1		U32_C(48)
#define NVCSI_DATATYPE_USER_2		U32_C(49)
#define NVCSI_DATATYPE_USER_3		U32_C(50)
#define NVCSI_DATATYPE_USER_4		U32_C(51)
#define NVCSI_DATATYPE_USER_5		U32_C(52)
#define NVCSI_DATATYPE_USER_6		U32_C(53)
#define NVCSI_DATATYPE_USER_7		U32_C(54)
#define NVCSI_DATATYPE_USER_8		U32_C(55)
#define NVCSI_DATATYPE_UNKNOWN		U32_C(64)

/* DEPRECATED - to be removed */
/** T210 (also exists in T186) */
#define NVCSI_PATTERN_GENERATOR_T210	U32_C(1)
/** T186 only */
#define NVCSI_PATTERN_GENERATOR_T186	U32_C(2)
/** T194 only */
#define NVCSI_PATTERN_GENERATOR_T194	U32_C(3)

/* DEPRECATED - to be removed */
#define NVCSI_DATA_TYPE_Unspecified		U32_C(0)
#define NVCSI_DATA_TYPE_YUV420_8		U32_C(24)
#define NVCSI_DATA_TYPE_YUV420_10		U32_C(25)
#define NVCSI_DATA_TYPE_LEG_YUV420_8		U32_C(26)
#define NVCSI_DATA_TYPE_YUV420CSPS_8		U32_C(28)
#define NVCSI_DATA_TYPE_YUV420CSPS_10		U32_C(29)
#define NVCSI_DATA_TYPE_YUV422_8		U32_C(30)
#define NVCSI_DATA_TYPE_YUV422_10		U32_C(31)
#define NVCSI_DATA_TYPE_RGB444			U32_C(32)
#define NVCSI_DATA_TYPE_RGB555			U32_C(33)
#define NVCSI_DATA_TYPE_RGB565			U32_C(34)
#define NVCSI_DATA_TYPE_RGB666			U32_C(35)
#define NVCSI_DATA_TYPE_RGB888			U32_C(36)
#define NVCSI_DATA_TYPE_RAW6			U32_C(40)
#define NVCSI_DATA_TYPE_RAW7			U32_C(41)
#define NVCSI_DATA_TYPE_RAW8			U32_C(42)
#define NVCSI_DATA_TYPE_RAW10			U32_C(43)
#define NVCSI_DATA_TYPE_RAW12			U32_C(44)
#define NVCSI_DATA_TYPE_RAW14			U32_C(45)
#define NVCSI_DATA_TYPE_RAW16			U32_C(46)
#define NVCSI_DATA_TYPE_RAW20			U32_C(47)
#define NVCSI_DATA_TYPE_Unknown			U32_C(64)

/* NVCSI DPCM ratio */
#define NVCSI_DPCM_RATIO_BYPASS		U32_C(0)
#define NVCSI_DPCM_RATIO_10_8_10	U32_C(1)
#define NVCSI_DPCM_RATIO_10_7_10	U32_C(2)
#define NVCSI_DPCM_RATIO_10_6_10	U32_C(3)
#define NVCSI_DPCM_RATIO_12_8_12	U32_C(4)
#define NVCSI_DPCM_RATIO_12_7_12	U32_C(5)
#define NVCSI_DPCM_RATIO_12_6_12	U32_C(6)
#define NVCSI_DPCM_RATIO_14_10_14	U32_C(7)
#define NVCSI_DPCM_RATIO_14_8_14	U32_C(8)
#define NVCSI_DPCM_RATIO_12_10_12	U32_C(9)

/* NVCSI param type */
#define NVCSI_PARAM_TYPE_UNSPECIFIED	U32_C(0)
#define NVCSI_PARAM_TYPE_DPCM		U32_C(1)
#define NVCSI_PARAM_TYPE_DT_OVERRIDE	U32_C(2)
#define NVCSI_PARAM_TYPE_WATCHDOG	U32_C(3)

struct nvcsi_dpcm_config {
	uint32_t dpcm_ratio;
	uint32_t __pad32;
} __CAPTURE_IVC_ALIGN;

struct nvcsi_dt_override_config {
	uint8_t enable_override;
	uint8_t __pad8[7];
	uint32_t override_type;
	uint32_t exception_type[NVCSI_NUM_NOOVERRIDE_DT];
} __CAPTURE_IVC_ALIGN;

struct nvcsi_watchdog_config {
	/* Enable/disable the pixel parser watchdog */
	uint8_t enable;
	uint8_t __pad8[3];
	/* The watchdog timer period */
	uint32_t period;
} __CAPTURE_IVC_ALIGN;

/**
 * NVCSI - TPG attributes
 */

/* Number of vertical color bars in TPG (t186) */
#define NVCSI_TPG_NUM_COLOR_BARS U32_C(8)

struct nvcsi_tpg_config_t186 {
	/* Stream ID */
	uint8_t stream_id;
	/* DEPRECATED - to be removed */
	uint8_t stream;
	/* Virtual channel ID */
	uint8_t virtual_channel_id;
	/* DEPRECATED - to be removed */
	uint8_t virtual_channel;
	/* Initial frame number */
	uint16_t initial_frame_number;
	uint16_t __pad16;
	/* Enable frame number generation */
	uint32_t enable_frame_counter;
	/* NvCsi datatype */
	uint32_t datatype;
	/* DEPRECATED - to be removed */
	uint32_t data_type;
	/* Dimensions of test image */
	uint16_t image_width;
	uint16_t image_height;
	/* Pixel value for each horizontal color bar (format according to DT) */
	uint32_t pixel_values[NVCSI_TPG_NUM_COLOR_BARS];
} __CAPTURE_IVC_ALIGN;

/* TPG flags for t194 */
#define NVCSI_TPG_FLAG_PATCH_MODE	U16_C(1)
#define NVCSI_TPG_FLAG_PHASE_INCREMENT	U16_C(2)
#define NVCSI_TPG_FLAG_AUTO_STOP	U16_C(4)

struct nvcsi_tpg_config_t194 {
	/* Virtual channel ID */
	uint8_t virtual_channel_id;
	/* DEPRECATED - to be removed */
	uint8_t virtual_channel;
	uint16_t __pad16[3];
	/* NvCsi datatype * */
	uint8_t datatype;
	/* DEPRECATED - to be removed */
	uint8_t data_type;
	/* NVCSI_TPG_FLAG_* */
	uint16_t flags;
	/** Frame number generator configuration */
	uint16_t initial_frame_number;
	uint16_t maximum_frame_number;
	/* Dimensions of test image */
	uint16_t image_width;
	uint16_t image_height;
	/* Embedded data config */
	uint32_t embedded_line_width;
	uint32_t embedded_lines_top;
	uint32_t embedded_lines_bottom;
	/* Lane count */
	uint32_t lane_count;
	/* Initial phase */
	uint32_t initial_phase;
	/* Pattern frequency config */
	uint32_t red_horizontal_init_freq;
	uint32_t red_vertical_init_freq;
	uint32_t red_horizontal_freq_rate;
	uint32_t red_vertical_freq_rate;
	uint32_t green_horizontal_init_freq;
	uint32_t green_vertical_init_freq;
	uint32_t green_horizontal_freq_rate;
	uint32_t green_vertical_freq_rate;
	uint32_t blue_horizontal_init_freq;
	uint32_t blue_vertical_init_freq;
	uint32_t blue_horizontal_freq_rate;
	uint32_t blue_vertical_freq_rate;
} __CAPTURE_IVC_ALIGN;

union nvcsi_tpg_config {
	/* T186 pattern generator */
	struct nvcsi_tpg_config_t186 t186;
	/* T194 pattern generator */
	struct nvcsi_tpg_config_t194 t194;
	/* Reserved size */
	uint32_t reserved[32];
};

/*
 * TPG rate config, low level parameters
 */
struct nvcsi_tpg_rate_config {
	/* Horizontal blanking (clocks) */
	uint32_t hblank;
	/* Vertical blanking (clocks) */
	uint32_t vblank;
	/* T194 only: Interval between pixels (clocks) */
	uint32_t pixel_interval;
	/* Reserved for future */
	uint32_t reserved;
} __CAPTURE_IVC_ALIGN;

/**
 * ISP capture settings
 */

/**
 * Describes RTCPU side resources for a ISP capture pipe-line.
 *
 * Following structure defines ISP channel specific configuration;
 *
 * @param channel_id:  unique capture ISP channel ID
 *
 * @param requests: base address of a memory mapped ring buffer
 *                  containing ISP capture descriptor requests. The size of the
 *                  buffer is queue_depth * request_size.
 *
 * @param request_queue_depth: number of capture requests in the requests queue.
 * @param request_size: size of the buffer reserved for each capture request.
 *
 * @param programs: base address of a memory mapped ring buffer
 *                  containing ISP program descriptors. The size of the
 *                  buffer is program_queue_depth * program_size.
 *
 * @param program_queue_depth: number of ISP programs in the programs queue.
 *
 * @param program_size: size of the buffer reserved for each ISP program.
 *
 * @param progress_sp:  progress syncpoint for frame events
 *
 * @param stats_progress_sp:  progress syncpoint for all stats units.
 *
 * @param channel_flags channel specific flags
 *
 * @param error_mask_correctable: Bitmask of the errors that are treated as correctable.
 *                                In case of correctable errors syncpoints of active capture are
 *                                advanced (in falcon) and error is reported and capture
 *                                continues.
 *
 * @param error_mask_uncorrectable: Bitmask of the errors that are treated as uncorrectable.
 *                                In case of uncorrectable errors, syncpoints of active capture are
 *                                advanced (in falcon) and isp channel is ABORTed by ISP
 *                                tasklist driver which halts the captures on the channel with
 *                                immediate effect, and then error is reported. Client needs
 *                                to RESET the channel explicitly in reaction to the
 *                                uncorrectable errors reported.
 *
 */
struct capture_channel_isp_config {
	uint8_t channel_id;
	uint8_t __pad_chan[3];
	uint32_t channel_flags;

#define CAPTURE_ISP_CHANNEL_FLAG_RESET_ON_ERROR	U32_C(0x0001)

	/** ISP capture descriptor ring buffer */
	iova_t requests;
	uint32_t request_queue_depth;
	uint32_t request_size;

	/** ISP program descriptor ring buffer */
	iova_t programs;
	uint32_t program_queue_depth;
	uint32_t program_size;
	struct syncpoint_info progress_sp;
	struct syncpoint_info stats_progress_sp;

	/** Error action attributes */
	uint32_t error_mask_correctable;
	uint32_t error_mask_uncorrectable;

#define CAPTURE_ISP_CHANNEL_ERROR_DMA_PBUF_ERR		(U32_C(1) << 0)
#define CAPTURE_ISP_CHANNEL_ERROR_DMA_SBUF_ERR		(U32_C(1) << 1)
#define CAPTURE_ISP_CHANNEL_ERROR_DMA_SEQ_ERR		(U32_C(1) << 2)
#define CAPTURE_ISP_CHANNEL_ERROR_FRAMEID_ERR		(U32_C(1) << 3)
#define CAPTURE_ISP_CHANNEL_ERROR_TIMEOUT		(U32_C(1) << 4)
#define CAPTURE_ISP_CHANNEL_ERROR_ALL			U32_C(0x001F)

#define HAVE_ISP_GOS_TABLES
	/*
	 * GoS tables can only be programmed when there are no
	 * active channels. For subsequent channels we check that
	 * the channel configuration matches with the active
	 * configuration.
	 */
	uint32_t num_isp_gos_tables;
	uint32_t __pad_chan2;
	iova_t isp_gos_tables[ISP_NUM_GOS_TABLES];
} __CAPTURE_IVC_ALIGN;

struct capture_isp_status {
	uint8_t chan_id;
	uint8_t __pad;
	uint16_t frame_id;
	uint32_t status;	/** SUCCESS OR ERROR */
	uint32_t error_mask;	/** ZERO in case of SUCCESS. Error bitmask in case of ERROR */
	uint32_t __pad2;

#define CAPTURE_ISP_STATUS_UNKNOWN		U32_C(0)
#define CAPTURE_ISP_STATUS_SUCCESS		U32_C(1)
#define CAPTURE_ISP_STATUS_ERROR		U32_C(2)
} __CAPTURE_IVC_ALIGN;


struct capture_isp_program_status {
	uint8_t chan_id;
	uint8_t settings_id;
	uint16_t __pad_id;
	uint32_t status;	/** SUCCESS OR ERROR */
	uint32_t error_mask;	/** ZERO in case of SUCCESS/STALE. Error bitmask in case of ERROR */
	uint32_t __pad2;

#define CAPTURE_ISP_PROGRAM_STATUS_UNKNOWN	U32_C(0)
#define CAPTURE_ISP_PROGRAM_STATUS_SUCCESS	U32_C(1)
#define CAPTURE_ISP_PROGRAM_STATUS_ERROR	U32_C(2)
#define CAPTURE_ISP_PROGRAM_STATUS_STALE	U32_C(3)
} __CAPTURE_IVC_ALIGN;

/**
 * Describes ISP program structure;
 *
 * @param sequence: capture sequence id, frame id; Given ISP program will be
 *                  used from this frame ID onwards until new ISP program does
 *                  replace it.
 *
 * @param isp_program_size: size of isp program
 *
 * @param isp_program_offset: offset to memory mapped ISP program buffer from
 *                  ISP program descriptor base address, which contains the ISP
 *                  configs and PB1 containing HW settings.
 *                  Ideally the offset is the size(ATOM aligned) of ISP program
 *                  descriptor only, as each isp_program would be placed just
 *                  after it's corresponding ISP program descriptor in memory.
 *
 * @param isp_pb1_mem: base address of memory mapped ISP PB1 containing
 *                  isp HW settings.
 *
 * @param settings_id: ISP settings_id which uniquely identifies isp_program.
 *
 * @param activate_flags: activation condition for given ISP program.
 *
 * @param isp_program_status: isp_program status written by RTCPU.
 *
 * @param vi_channel_id: VI channel bound to the isp channel. In case of mem_isp_mem set this to
 * 		CAPTURE_NO_VI_ISP_BINDING
 */
struct isp_program_descriptor {
	uint8_t settings_id;
	uint8_t vi_channel_id;
#define CAPTURE_NO_VI_ISP_BINDING U8_C(0xFF)

	uint8_t __pad_sid[2];
	uint32_t sequence;

	uint32_t isp_program_offset;
	uint32_t isp_program_size;

	/** NvISP assures it to be 64 bytes aligned */
	iova_t isp_pb1_mem;

	struct capture_isp_program_status isp_program_status;

	uint32_t activate_flags;

#define CAPTURE_ACTIVATE_FLAG_ON_SEQUENCE_ID	U32_C(0x1) /* 1 << 0 */
#define CAPTURE_ACTIVATE_FLAG_ON_SETTINGS_ID	U32_C(0x2) /* 1 << 1 */
#define CAPTURE_ACTIVATE_FLAG_COUPLED		U32_C(0x4) /* 1 << 2 */

	/** Pad to aligned size */
	uint32_t __pad[5];
} __CAPTURE_DESCRIPTOR_ALIGN;

/**
 * ISP program size (ATOM aligned).
 *
 * NvCapture UMD makes sure to place isp_program just after above program
 * descriptor buffer for each request, so that KMD and RCE can co-locate
 * isp_program and it's corresponding program descriptor in memory.
 */
#define ISP_PROGRAM_MAX_SIZE 16512

struct image_surface {
	uint32_t offset;
	uint32_t offset_hi;
	uint32_t surface_stride;
	uint32_t __pad_surf;
} __CAPTURE_IVC_ALIGN;

struct stats_surface {
	uint32_t offset;
	uint32_t offset_hi;
} __CAPTURE_IVC_ALIGN;

/**
 * Describes ISP capture descriptor
 *
 * The following parameters describe the capture descriptor ring buffer.
 *
 * @param input_mr_surfaces: input surfaces' details.
 *
 * @param outputs_mw: output surfaces' details.
 *
 * @param fb_surface: Surface details of Flicker Band stat unit.
 *
 * @param fm_surface: Surface details of Focus Metric stat unit.
 *
 * @param afm_surface: Surface details of Auto-Focus Metric stat unit.
 *
 * @param lac0_surface, @param lac1_surface:
 *                  Surface details of Local Ave Clip stat units.
 *
 * @param h0_surface, @param h1_surface:
 *                  Surface details of Histogram stat units.
 *
 * @param pru_bad_surface: Surface details of Bad pixel detection block,
 *                  part of PRU stat unit.
 *
 * @param ltm_surface: Surface details of Local Tone Mapping stat unit.
 *
 * @param surface_configs: surfaces related config details.
 *
 * @param surface_configs.mr_width: Width of input surface in pixels
 *
 * @param surface_configs.mr_height: Height of input surface in pixels
 *
 * @param surface_configs.slice_height: Height of slices used in ISP to
 *                  process the image
 *
 * @param surface_configs.chunk_width_first: Width of first VI chunk in line
 *                  if DPCM compression is used. Not used if DPCM is not
 *                  in use.
 *
 * @param surface_configs.chunk_width_middle: Width of the VI chunks in middle
 *                  of a line if DPCM is user and width of ISP tiles in middle
 *                  of line regardless of DPCM usage.
 *
 * @param surface_configs.chunk_overfetch_width: Width of the overfetch region
 *                  in beginning of VI chunks if DPCM is in use. Unused if no
 *                  DPCM
 *
 * @param surface_configs.tile_width_first: Width of the first tile in a slice.
 *                  Width of the rest of tiles is defined by chunk_width_middle
 *                  field
 *
 * @param surface_configs.mr_image_cfa: MR image CFA combination. Field needed
 *                  for IspModel to reconstruct NvColorFormat from mr_image_def.
 *
 * @param surface_configs.mr_image_def: MR image format definition. Field
 *                  format is according to register ISP_MR_IMAGE_DEF_MR.
 *
 * @param surface_configs.mr_image_def1: Stream and frame ID for
 *                  this processing request.
 *
 * @param surface_configs.surf_ctrl: ISP_MR_SURFACE_CTL_MR register settings.
 *
 * @param surface_configs.surf_stride_line:  Byte stride from start of line to
 *                  start of the next line. Must be ATOM (64 byte) aligned.
 *
 * @param surface_configs.surf_stride_chunk: Byte stride from start of DPCM
 *                  chunk to start of the next chunk. Must be 64 byte aligned.
 *
 * @param isp_pb2_mem: base address of memory mapped empty ISP PB2, which RCE
 *                  would fill with out and stats surfaces details provided in
 *                  this capture descriptor.
 *
 * @param isp_pb2_size: Size of ISP PB2
 *
 * @param sequence: capture sequence ID, frame ID.
 *
 * @param capture_flags: frame specific capture flags
 *
 * @param frame_timeout: frame wait time
 *
 * @param num_inputfences: (renamed prefence_count (DEPRECATED)):
 *                  Number of inputfences for given capture request.
 *                  These fences are exclusively associated with ISP input ports and
 *                  they support subframe sychronization.
 *
 * @param inputfences: (renamed progress_prefence (DEPRECATED)):
 *                  progress syncpoint for each one of inputfences.
 *
 * @param num_prefences: Number of traditional prefences for given capture request.
 *                  They are generic, so can be used for any pre-condition but do not
 *                  support subframe synchronization.
 *
 * @param prefences: syncpoint for each one of prefences.
 *
 * @param status: capture status written by RTCPU.
 *
 */

struct isp_capture_descriptor {
	uint32_t sequence;
	uint32_t capture_flags;

#define CAPTURE_ISP_FLAG_STATUS_REPORT_ENABLE	(U32_C(1) << 0)
#define CAPTURE_ISP_FLAG_ERROR_REPORT_ENABLE	(U32_C(1) << 1)
#define CAPTURE_ISP_FLAG_ISP_PROGRAM_BINDING	(U32_C(1) << 2)

	/** 1 MR port, max 3 input surfaces */
#define ISP_MAX_INPUT_SURFACES (3U)

	/** input surfaces */
	struct image_surface input_mr_surfaces[ISP_MAX_INPUT_SURFACES];

	/**
	 * 3 MW ports, max 2 surfaces (multiplanar) per port.
	 */
#define ISP_MAX_OUTPUTS (3U)
#define ISP_MAX_OUTPUT_SURFACES (2U)

	/** output surfaces */
	struct {
		struct image_surface surfaces[ISP_MAX_OUTPUT_SURFACES];
		/**
		 * Image format definition for output surface
		 * TODO: Should we have here just image format enum value + block height instead?
		 * Dither settings would logically be part of ISP program
		 */
		uint32_t image_def;
		uint16_t width;
		uint16_t height;
	} outputs_mw[ISP_MAX_OUTPUTS];

	/**
	 * stats surfaces.
	 *
	 * AFM, LAC0 and LAC1 surfaces are their respective base addresses.
	 * RCE knows the offsets to all ROIs' addresses for each of these
	 * stats units.
	 */
	struct stats_surface fb_surface;
	struct stats_surface fm_surface;
	struct stats_surface afm_surface;
	struct stats_surface lac0_surface;
	struct stats_surface lac1_surface;
	struct stats_surface h0_surface;
	struct stats_surface h1_surface;
	struct stats_surface pru_bad_surface;
	struct stats_surface ltm_surface;

	/** surfaces related configuration */

	struct {
		/** Input image resolution */
		uint16_t mr_width;
		uint16_t mr_height;

		/** Height of slices used for processing the image */
		uint16_t slice_height;
		/** Width of first VI chunk in a line */
		uint16_t chunk_width_first;
		/** Width of VI chunks in the middle of a line, and/or width of
		 *  ISP tiles in middle of a slice */
		uint16_t chunk_width_middle;
		/** Width of overfetch area in the beginning of VI chunks */
		uint16_t chunk_overfetch_width;
		/** Width of the leftmost ISP tile in a slice */
		uint16_t tile_width_first;
		/** Input image cfa */
		uint8_t mr_image_cfa;
		uint8_t _pad;
		/** Input image format */
		uint32_t mr_image_def;
		/** TODO: should this be exposed to user mode? */
		uint32_t mr_image_def1;
		/** SURFACE_CTL_MR register value */
		uint32_t surf_ctrl;
		/** Byte stride between start of lines. Must be ATOM aligned */
		uint32_t surf_stride_line;
		/** Byte stride between start of DPCM chunks. Must be ATOM aligned */
		uint32_t surf_stride_chunk;
	} surface_configs;

	uint32_t __pad2;
	/** Base address of ISP PB2 memory */
	iova_t isp_pb2_mem;
	/** TODO: Isn't PB2 size constant, do we need this? */
	uint32_t isp_pb2_size;
	uint32_t __pad_pb;

	uint32_t frame_timeout;	 /**< Timeout in microseconds */

	union {
		/* field renamed */
		uint32_t prefence_count CAMRTC_DEPRECATED;
		uint32_t num_inputfences;
	};

	union {
		/* array renamed */
		struct syncpoint_info progress_prefence[ISP_MAX_INPUT_SURFACES] CAMRTC_DEPRECATED;
		struct syncpoint_info inputfences[ISP_MAX_INPUT_SURFACES];
	};

	/* TBD: Decide exact max count */
#define ISP_MAX_PREFENCES (ISP_MAX_OUTPUTS + ISP_MAX_INPUT_SURFACES)

	uint32_t num_prefences;
	uint32_t __pad_prefences;

	struct syncpoint_info prefences[ISP_MAX_PREFENCES];

	/* Result record – written by Falcon */
	struct engine_status_surface engine_status;

	/** Result record – written by RTCPU */
	struct capture_isp_status status;

	/* Information regarding the ISP program bound to this capture */
	uint32_t program_buffer_index;

	/** Pad to aligned size */
	uint32_t __pad[3];
} __CAPTURE_DESCRIPTOR_ALIGN;

/**
 * PB2 size (ATOM aligned).
 *
 * NvCapture UMD makes sure to place PB2 just after above capture
 * descriptor buffer for each request, so that KMD and RCE can co-locate
 * PB2 and it's corresponding capture descriptor in memory.
 */
#define ISP_PB2_MAX_SIZE 512

/**
 * Size allocated for the ISP program push buffer
 */
#define NVISP5_ISP_PROGRAM_PB_SIZE 16384

/**
* Size allocated for the push buffer containing output & stats
* surface definitions. Final value TBD
*/
#define NVISP5_SURFACE_PB_SIZE 512


/**
* Downscaler configuration information that is needed for building  ISP
* config buffer. Therefor these registers cannot be included in push buffer
* but they must be provided in a structure that RCE can parse. Format of
* the fields is same as in corresponding ISP registers.
*/
struct isp5_downscaler_configbuf {
	/**
	* Horizontal pixel increment, in U5.20 format. I.e. 2.5 means downscaling
	* by factor of 2.5. Corresponds to ISP_DM_H_PI register
	*/
	uint32_t pixel_incr_h;
	/**
	* Vertical pixel increment, in U5.20 format. I.e. 2.5 means downscaling
	* by factor of 2.5. Corresponds to ISP_DM_v_PI register
	*/
	uint32_t pixel_incr_v;

	/**
	* Offset of the first source image pixel to be used.
	* Topmost 16 bits - the leftmost column to be used
	* Lower 16 bits - the topmost line to be used
	*/
	uint32_t offset;

	/**
	* Size of the scaled destination image in pixels
	* Topmost 16 bits - height of destination image
	* Lowest 16 bits - Width of destination image
	*/
	uint32_t destsize;
};


enum isp5_block_enabled {
	ISP5BLOCK_ENABLED_PRU_OUTLIER_REJECTION = 1U,
	ISP5BLOCK_ENABLED_PRU_STATS = 1U << 1,
	ISP5BLOCK_ENABLED_PRU_HDR = 1U << 2,
	ISP5BLOCK_ENABLED_AP_DEMOSAIC = 1U << 4,
	ISP5BLOCK_ENABLED_AP_CAR = 1U << 5,
	ISP5BLOCK_ENABLED_AP_LTM_MODIFY = 1U << 6,
	ISP5BLOCK_ENABLED_AP_LTM_STATS = 1U << 7,
	ISP5BLOCK_ENABLED_AP_FOCUS_METRIC = 1U << 8,
	ISP5BLOCK_ENABLED_FLICKERBAND = 1U << 9,
	ISP5BLOCK_ENABLED_HISTOGRAM0 = 1U << 10,
	ISP5BLOCK_ENABLED_HISTOGRAM1 = 1U << 11,
	ISP5BLOCK_ENABLED_DOWNSCALER0_HOR = 1U << 12,
	ISP5BLOCK_ENABLED_DOWNSCALER0_VERT = 1U << 13,
	ISP5BLOCK_ENABLED_DOWNSCALER1_HOR = 1U << 14,
	ISP5BLOCK_ENABLED_DOWNSCALER1_VERT = 1U << 15,
	ISP5BLOCK_ENABLED_DOWNSCALER2_HOR = 1U << 16,
	ISP5BLOCK_ENABLED_DOWNSCALER2_VERT = 1U << 17,
	ISP5BLOCK_ENABLED_SHARPEN0 = 1U << 18,
	ISP5BLOCK_ENABLED_SHARPEN1 = 1U << 19,
	ISP5BLOCK_ENABLED_LAC0_REGION0 = 1U << 20,
	ISP5BLOCK_ENABLED_LAC0_REGION1 = 1U << 21,
	ISP5BLOCK_ENABLED_LAC0_REGION2 = 1U << 22,
	ISP5BLOCK_ENABLED_LAC0_REGION3 = 1U << 23,
	ISP5BLOCK_ENABLED_LAC1_REGION0 = 1U << 24,
	ISP5BLOCK_ENABLED_LAC1_REGION1 = 1U << 25,
	ISP5BLOCK_ENABLED_LAC1_REGION2 = 1U << 26,
	ISP5BLOCK_ENABLED_LAC1_REGION3 = 1U << 27
};

/**
 * ISP overfetch requirements.
 *
 * ISP kernel needs access to pixels outside the active area of a tile
 * to ensure continuous processing across tile borders. The amount of data
 * needed depends on features enabled and some ISP parameters so this
 * is program dependent.
 *
 * ISP extrapolates values outside image borders, so overfetch is needed only
 * for borders between tiles.
 */

struct isp_overfetch {
	/**
	 * Number of pixels needed from the left side of tile
	 */
	uint8_t left;
	/**
	 * Number of pixels needed from the right side of tile
	 */
	uint8_t right;
	/**
	 * Number of pixels needed from above the tile
	 */
	uint8_t top;
	/**
	 * Number of pixels needed from below the tile
	 */
	uint8_t bottom;
	/**
	 * Number of pixels needed by PRU unit from left and right sides of the tile.
	 * This is needed to adjust tile border locations so that they align correctly
	 * at demosaic input.
	 */
	uint8_t pru_ovf_h;
	/**
	 * Alignment requirement for tile width. Minimum alignment is 2 pixels, but
	 * if CAR is used this must be set to half of LPF kernel width.
	 */
	uint8_t alignment;
	uint8_t _pad1[2];
};

struct isp5_program {
	/*
	 * Settings needed by RCE ISP driver to generate config buffer.
	 * Content and format of these fields is the same as corresponding
	 * ISP config buffer fields.
	 * See T19X_ISP_Microcode.docx for detailed description.
	*/

	/**
	 * Sources for LS, AP and PRU blocks.
	 * Format is same as in ISP's XB_SRC_0 register
	*/
	uint32_t xbsrc0;

	/**
	 * Sources for AT[0-2] and TF[0-1] blocks
	 * Format is same as in ISP's XB_SRC_1 register
	*/
	uint32_t xbsrc1;

	/**
	 * Sources for DS[0-2] and MW[0-2] blocks
	 * Format is same as in ISP's XB_SRC_2 register
	*/
	uint32_t xbsrc2;

	/**
	 * Sources for FB, LAC[0-1] and HIST[0-1] blocks
	 * Format is same as in ISP's XB_SRC_3 register
	*/
	uint32_t xbsrc3;

	/**
	* Bitmask to describe which of ISP blocks are enabled.
	* See microcode documentation for details.
	*/
	uint32_t enables_config;

	/**
	* AFM configuration. See microcode documentation for details.
	*/
	uint32_t afm_ctrl;

	/**
	* Mask for stats blocks enabled.
	*/
	uint32_t stats_aidx_flag;

	/**
	* Size used for the push buffer in 4-byte words.
	*/
	uint32_t pushbuffer_size;

	/**
	* Horizontal pixel increment for downscalers, in
	* U5.20 format. I.e. 2.5 means downscaling
	* by factor of 2.5. Corresponds to ISP_DM_H_PI register.
	* This is needed by ISP Falcon firmware to program
	* tile starting state correctly.
	*/
	uint32_t ds0_pixel_incr_h;
	uint32_t ds1_pixel_incr_h;
	uint32_t ds2_pixel_incr_h;

	struct isp_overfetch overfetch;

	uint32_t _pad1[3];

	/**
	 * Push buffer containing ISP settings related to this program.
	 * No relocations will be done for this push buffer; all registers
	 * that contain memory addresses that require relocation must be
	 * specified in the capture descriptor ISP payload.
	 */
	uint32_t pushbuffer[NVISP5_ISP_PROGRAM_PB_SIZE / sizeof(uint32_t)]
			__CAPTURE_DESCRIPTOR_ALIGN;

} __CAPTURE_DESCRIPTOR_ALIGN;


struct isp5_program_entry {
	struct isp_program_descriptor prog_desc;
	struct isp5_program isp_prog;
} __CAPTURE_DESCRIPTOR_ALIGN;

#pragma GCC diagnostic ignored "-Wpadded"

#endif /* INCLUDE_CAMRTC_CAPTURE_H */
