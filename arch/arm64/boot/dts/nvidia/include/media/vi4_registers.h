/*
 * Tegra 18x VI register offsets
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VI4_REGISTERS_H__
#define __VI4_REGISTERS_H__

/* VI registers. Start from 0x0 */
#define VI_STREAMS (6)
#define VIRTUAL_CHANNELS (4)
#define VI4_CHANNEL_OFFSET         0x10000

#define CFG_INTERRUPT_STATUS		0x44
#define CFG_INTERRUPT_MASK			0x48
#define VGP6_INT_MASK					(0x1 << 29)
#define VGP5_INT_MASK					(0x1 << 28)
#define VGP4_INT_MASK					(0x1 << 27)
#define VGP3_INT_MASK					(0x1 << 26)
#define VGP2_INT_MASK					(0x1 << 25)
#define VGP1_INT_MASK					(0x1 << 24)
#define HOST_PKTINJECT_STALL_ERR_MASK	(0x1 << 7)
#define CSIMUX_FIFO_OVFL_ERR_MASK		(0x1 << 6)
#define ATOMP_PACKER_OVFL_ERR_MASK		(0x1 << 5)
#define FMLITE_BUF_OVFL_ERR_MASK		(0x1 << 4)
#define NOTIFY_FIFO_OVFL_ERR_MASK		(0x1 << 3)
#define ISPBUFA_ERR_MASK				(0x1 << 0)

#define CFG_PWM_HIGH_PULSE			0x50
#define PWM_HIGH_PULSE				(0xffffffff << 0)

#define CSIMUX_CONFIG_STREAM_0		0x424
#define CSIMUX_CONFIG_STREAM_1		0x428
#define CSIMUX_CONFIG_STREAM_2		0x42C
#define CSIMUX_CONFIG_STREAM_3		0x430
#define CSIMUX_CONFIG_STREAM_4		0x434
#define CSIMUX_CONFIG_STREAM_5		0x438
#define FRAMEIDGEN					(0xf << 26)
#define STICKYFAULT					(0x1 << 25)
#define VPR							(0x1 << 24)
#define SRESET						(0x1 << 23)
#define QBLOCK						(0x1 << 22)
#define FEINJECT					(0x1 << 21)
#define FESHORTTIMER				(0x1 << 20)
#define FEMAXTIME					(0xffff << 4)
#define WT							(0xf << 0)

#define NOTIFY_FIFO_TAG_0			0x4000
#define NOTIFY_FRAME_ID				(0xffff << 16)
#define NOTIFY_CHANNEL				(0xff << 8)
#define NOTIFY_CHANNEL_SHIFT		(8)
#define NOTIFY_TAG					(0x1f << 1)
#define NOTIFY_TAG_SHIFT			(1)
#define NOTIFY_VALID				(0x1 << 0)

#define	TAG_FS						0
#define	TAG_FE						1
#define	TAG_CSIMUX_FRAME			2
#define	TAG_CSIMUX_STREAM			3
#define	TAG_CHANSEL_PXL_SOF			4
#define	TAG_CHANSEL_PXL_EOF			5
#define	TAG_CHANSEL_EMBED_SOF		6
#define	TAG_CHANSEL_EMBED_EOF		7
#define	TAG_CHANSEL_NLINES			8
#define	TAG_CHANSEL_FAULT			9
#define	TAG_CHANSEL_FAULT_FE		10
#define	TAG_CHANSEL_NOMATCH			11
#define	TAG_CHANSEL_COLLISION		12
#define	TAG_CHANSEL_SHORT_FRAME		13
#define	TAG_CHANSEL_LOAD_FRAMED		14
#define	TAG_ATOMP_PACKER_OVERFLOW	15
#define	TAG_ATOMP_FS				16
#define	TAG_ATOMP_FE				17
#define	TAG_ATOMP_FRAME_DONE		18
#define	TAG_ATOMP_EMB_DATA_DONE		19
#define	TAG_ATOMP_FRAME_NLINES_DONE	20
#define	TAG_ATOMP_FRAME_TRUNCATED	21
#define	TAG_ATOMP_FRAME_TOSSED		22
#define	TAG_ATOMP_PDAF_DATA_DONE	23
#define	TAG_ISPBUF_FIFO_OVERFLOW	26
#define	TAG_ISPBUF_FS				27
#define	TAG_ISPBUF_FE				28
#define	TAG_VGP0_DONE				29
#define	TAG_VGP1_DONE				30
#define	TAG_FMLITE_DONE				31

#define NOTIFY_FIFO_TIMESTAMP_0		0x4004
#define NOTIFY_TIMESTAMP			(0xffffffff << 0)

#define NOTIFY_FIFO_DATA_0			0x4008
#define NOTIFY_DATA					(0xffffffff << 0)

#define NOTIFY_TAG_CLASSIFY_0		0x6000
#define NOTIFY_TAG_CLASSIFY_1		0x6004
#define NOTIFY_TAG_CLASSIFY_2		0x6008
#define NOTIFY_TAG_CLASSIFY_3		0x600c
#define NOTIFY_TAG_CLASSIFY_4		0x6010
#define STREAM5_FEINJECT_VC			(0xf << 20)
#define STREAM4_FEINJECT_VC			(0xf << 16)
#define STREAM3_FEINJECT_VC			(0xf << 12)
#define STREAM2_FEINJECT_VC			(0xf << 8)
#define STREAM1_FEINJECT_VC			(0xf << 4)
#define STREAM0_FEINJECT_VC			(0xf << 0)

#define NOTIFY_FIFO_OCCUPANCY_0		0x6014
#define NOTIFY_MAX					(0x3ff << 20)
#define NOTIFY_CURRENT				(0x3ff << 10)
#define NOTIFY_CURRENT_SHIFT		10
#define NOTIFY_SIZE					(0x3ff << 0)

/* VI_CH registers. Start from 0x10000, offset 0x10000 */
#define CHANNEL_COMMAND				0x004
#define WR_ACT_SEL					(0x1 << 5)
#define RD_MUX_SEL					(0x1 << 4)
#define AUTOLOAD					(0x1 << 1)
#define LOAD						(0x1 << 0)

#define CONTROL						0x01c
#define SPARE						(0xffff << 16)
#define POST_RUNAWAY_EMBED			(0x1 << 4)
#define POST_RUNAWAY_PIXEL			(0x1 << 3)
#define EARLY_ABORT					(0x1 << 2)
#define SINGLESHOT					(0x1 << 1)
#define MATCH_STATE_EN				(0x1 << 0)

#define MATCH						0x020
#define STREAM						(0x3f << 14)
#define STREAM_SHIFT				(14)
#define STREAM_MASK					(0x3f << 8)
#define VIRTUAL_CHANNEL				(0xf << 4)
#define VIRTUAL_CHANNEL_SHIFT		(4)
#define VIRTUAL_CHANNEL_MASK		(0xf << 0)

#define MATCH_DATATYPE				0x024
#define DATATYPE					(0x3f << 6)
#define DATATYPE_SHIFT				(6)
#define DATATYPE_MASK				(0x3f << 0)
#define DATATYPE_MASK_SHIFT			(0)

#define MATCH_FRAMEID				0x028
#define FRAMEID						(0xffff << 16)
#define FRAMEID_SHIFT				(16)
#define FRAMEID_MASK				(0xffff << 0)

#define DT_OVERRIDE					0x02c
#define OVRD_DT						(0x3f << 1)
#define DT_OVRD_EN					(0x1 << 0)

#define FRAME_X						0x030
#define CROP_X						0x04c
#define OUT_X						0x058
#define WIDTH						(0xffff < 0)

#define FRAME_Y						0x034
#define CROP_Y						0x054
#define OUT_Y						0x05c
#define HEIGHT						(0xffff < 0)

#define EMBED_X						0x038
#define MAX_BYTES					(0x3ffff < 0)

#define EMBED_Y						0x03c
#define SKIP_Y						0x050
#define LINES						(0xffff < 0)
/* for EMBED_Y only */
#define EXPECT						(0x1 << 24)

#define LINE_TIMER					0x044
#define LINE_TIMER_EN				(0x1 << 25)
#define PERIODIC					(0x1 << 24)
#define TRIPLINE					(0xffff << 0)

#define SKIP_X						0x048
#define PACKETS						(0x1fff << 0)

#define NOTIFY_MASK					0x060
#define MASK_DTYPE_MISMATCH			(0x1 << 31)
#define MASK_EMBED_INFRINGE			(0x1 << 22)
#define MASK_EMBED_LONG_LINE		(0x1 << 21)
#define MASK_EMBED_SPURIOUS			(0x1 << 20)
#define MASK_EMBED_RUNAWAY			(0x1 << 19)
#define MASK_EMBED_MISSING_LE		(0x1 << 18)
#define MASK_EMBED_EOF				(0x1 << 17)
#define MASK_EMBED_SOF				(0x1 << 16)
#define MASK_PIXEL_LINE_TIMER		(0x1 << 7)
#define MASK_PIXEL_SHORT_LINE		(0x1 << 6)
#define MASK_PIXEL_LONG_LINE		(0x1 << 5)
#define MASK_PIXEL_SPURIOUS			(0x1 << 4)
#define MASK_PIXEL_RUNAWAY			(0x1 << 3)
#define MASK_PIXEL_MISSING_LE		(0x1 << 2)
#define MASK_PIXEL_EOF				(0x1 << 1)
#define MASK_PIXEL_SOF				(0x1 << 0)

#define NOTIFY_MASK_XCPT			0x064
#define MASK_NOMATCH				(0x1 << 9)
#define MASK_EMBED_OPEN_LINE		(0x1 << 8)
#define MASK_PIXEL_OPEN_LINE		(0x1 << 7)
#define MASK_FORCE_FE				(0x1 << 6)
#define MASK_STALE_FRAME			(0x1 << 5)
#define MASK_COLLISION				(0x1 << 4)
#define MASK_EMPTY_FRAME			(0x1 << 3)
#define MASK_EMBED_SHORT_FRAME		(0x1 << 2)
#define MASK_PIXEL_SHORT_FRAME		(0x1 << 1)
#define MASK_LOAD_FRAMED			(0x1 << 0)

#define FRAME_COUNT					0x06c

#define PIXFMT_ENABLE				0x080
#define PDAF_EN						(0x1 << 2)
#define COMPAND_EN					(0x1 << 1)
#define PIXFMT_EN					(0x1 << 0)

#define PIXFMT_FORMAT				0x084
#define FORMAT						(0xff << 0)
/* refer to enum tegra_image_format in core.h */

#define PIXFMT_WIDE					0x088
#define ENDIAN_BIG					(0x0 << 1)
#define ENDIAN_LITTLE				(0x1 << 1)
#define PIXFMT_WIDE_EN				(0x1 << 0)

#define DPCM_STRIP					0x0b8
#define OVERFETCH					(0x1fff < 16)
#define STRIP_WIDTH					(0x1fff < 0)

#define ATOMP_DPCM_CHUNK			0x0ec
#define CHUNK_OFFSET				(0x3ffff << 0)

#define ATOMP_SURFACE_OFFSET0		0x0e0
#define ATOMP_SURFACE_OFFSET1		0x0f0
#define ATOMP_SURFACE_OFFSET2		0x0fc
#define ATOMP_EMB_SURFACE_OFFSET0	0x108
#define SURFACE_OFFSET				(0xffffffff << 0)

#define ATOMP_SURFACE_OFFSET0_H		0x0e4
#define ATOMP_SURFACE_OFFSET1_H		0x0f4
#define ATOMP_SURFACE_OFFSET2_H		0x100
#define ATOMP_EMB_SURFACE_OFFSET0_H		0x10c
#define SURFACE_OFFSET_HI			(0xff << 0)

#define ATOMP_SURFACE_STRIDE0		0x0e8
#define ATOMP_SURFACE_STRIDE1		0x0f8
#define ATOMP_SURFACE_STRIDE2		0x104
#define ATOMP_EMB_SURFACE_STRIDE0	0x110
#define SURFACE_STRIDE				(0x3ffff << 0)
#define ATOMP_RESERVE				0x120

#define ISPBUFA						0x134
#define ISPBUFA_EN					(0x1 << 0)

#define ISPBUFA_ERROR				0x1000
#define	FIFO_OVERFLOW				(0x1 << 0)

#define FMLITE_ERROR				0x313c
#define NOTIFY_ERROR				0x6020

#endif /* __VI4_REGISTERS_H__ */
