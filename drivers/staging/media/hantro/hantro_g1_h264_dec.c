// SPDX-License-Identifier: GPL-2.0
/*
 * Hantro VPU codec driver
 *
 * Copyright (c) 2014 Rockchip Electronics Co., Ltd.
 *	Hertz Wong <hertz.wong@rock-chips.com>
 *	Herman Chen <herman.chen@rock-chips.com>
 *
 * Copyright (C) 2014 Google, Inc.
 *	Tomasz Figa <tfiga@chromium.org>
 */

#include <linux/types.h>
#include <linux/sort.h>

#include <media/v4l2-mem2mem.h>

#include "hantro_hw.h"
#include "hantro_v4l2.h"

#define G1_SWREG(nr)			((nr) * 4)

#define G1_REG_RLC_VLC_BASE		G1_SWREG(12)
#define G1_REG_DEC_OUT_BASE		G1_SWREG(13)
#define G1_REG_REFER0_BASE		G1_SWREG(14)
#define G1_REG_REFER1_BASE		G1_SWREG(15)
#define G1_REG_REFER2_BASE		G1_SWREG(16)
#define G1_REG_REFER3_BASE		G1_SWREG(17)
#define G1_REG_REFER4_BASE		G1_SWREG(18)
#define G1_REG_REFER5_BASE		G1_SWREG(19)
#define G1_REG_REFER6_BASE		G1_SWREG(20)
#define G1_REG_REFER7_BASE		G1_SWREG(21)
#define G1_REG_REFER8_BASE		G1_SWREG(22)
#define G1_REG_REFER9_BASE		G1_SWREG(23)
#define G1_REG_REFER10_BASE		G1_SWREG(24)
#define G1_REG_REFER11_BASE		G1_SWREG(25)
#define G1_REG_REFER12_BASE		G1_SWREG(26)
#define G1_REG_REFER13_BASE		G1_SWREG(27)
#define G1_REG_REFER14_BASE		G1_SWREG(28)
#define G1_REG_REFER15_BASE		G1_SWREG(29)
#define G1_REG_QTABLE_BASE		G1_SWREG(40)
#define G1_REG_DIR_MV_BASE		G1_SWREG(41)
#define G1_REG_DEC_E(v)			((v) ? BIT(0) : 0)

#define G1_REG_DEC_AXI_RD_ID(v)		(((v) << 24) & GENMASK(31, 24))
#define G1_REG_DEC_TIMEOUT_E(v)		((v) ? BIT(23) : 0)
#define G1_REG_DEC_STRSWAP32_E(v)	((v) ? BIT(22) : 0)
#define G1_REG_DEC_STRENDIAN_E(v)	((v) ? BIT(21) : 0)
#define G1_REG_DEC_INSWAP32_E(v)	((v) ? BIT(20) : 0)
#define G1_REG_DEC_OUTSWAP32_E(v)	((v) ? BIT(19) : 0)
#define G1_REG_DEC_DATA_DISC_E(v)	((v) ? BIT(18) : 0)
#define G1_REG_DEC_LATENCY(v)		(((v) << 11) & GENMASK(16, 11))
#define G1_REG_DEC_CLK_GATE_E(v)	((v) ? BIT(10) : 0)
#define G1_REG_DEC_IN_ENDIAN(v)		((v) ? BIT(9) : 0)
#define G1_REG_DEC_OUT_ENDIAN(v)	((v) ? BIT(8) : 0)
#define G1_REG_DEC_ADV_PRE_DIS(v)	((v) ? BIT(6) : 0)
#define G1_REG_DEC_SCMD_DIS(v)		((v) ? BIT(5) : 0)
#define G1_REG_DEC_MAX_BURST(v)		(((v) << 0) & GENMASK(4, 0))

#define G1_REG_DEC_MODE(v)		(((v) << 28) & GENMASK(31, 28))
#define G1_REG_RLC_MODE_E(v)		((v) ? BIT(27) : 0)
#define G1_REG_PIC_INTERLACE_E(v)	((v) ? BIT(23) : 0)
#define G1_REG_PIC_FIELDMODE_E(v)	((v) ? BIT(22) : 0)
#define G1_REG_PIC_TOPFIELD_E(v)	((v) ? BIT(19) : 0)
#define G1_REG_FILTERING_DIS(v)		((v) ? BIT(14) : 0)
#define G1_REG_PIC_FIXED_QUANT(v)	((v) ? BIT(13) : 0)
#define G1_REG_WRITE_MVS_E(v)		((v) ? BIT(12) : 0)
#define G1_REG_SEQ_MBAFF_E(v)		((v) ? BIT(10) : 0)
#define G1_REG_PICORD_COUNT_E(v)	((v) ? BIT(9) : 0)
#define G1_REG_DEC_AXI_WR_ID(v)		(((v) << 0) & GENMASK(7, 0))

#define G1_REG_PIC_MB_WIDTH(v)		(((v) << 23) & GENMASK(31, 23))
#define G1_REG_PIC_MB_HEIGHT_P(v)	(((v) << 11) & GENMASK(18, 11))
#define G1_REG_REF_FRAMES(v)		(((v) << 0) & GENMASK(4, 0))

#define G1_REG_STRM_START_BIT(v)	(((v) << 26) & GENMASK(31, 26))
#define G1_REG_TYPE1_QUANT_E(v)		((v) ? BIT(24) : 0)
#define G1_REG_CH_QP_OFFSET(v)		(((v) << 19) & GENMASK(23, 19))
#define G1_REG_CH_QP_OFFSET2(v)		(((v) << 14) & GENMASK(18, 14))
#define G1_REG_FIELDPIC_FLAG_E(v)	((v) ? BIT(0) : 0)

#define G1_REG_START_CODE_E(v)		((v) ? BIT(31) : 0)
#define G1_REG_INIT_QP(v)		(((v) << 25) & GENMASK(30, 25))
#define G1_REG_CH_8PIX_ILEAV_E(v)	((v) ? BIT(24) : 0)
#define G1_REG_STREAM_LEN(v)		(((v) << 0) & GENMASK(23, 0))

#define G1_REG_CABAC_E(v)		((v) ? BIT(31) : 0)
#define G1_REG_BLACKWHITE_E(v)		((v) ? BIT(30) : 0)
#define G1_REG_DIR_8X8_INFER_E(v)	((v) ? BIT(29) : 0)
#define G1_REG_WEIGHT_PRED_E(v)		((v) ? BIT(28) : 0)
#define G1_REG_WEIGHT_BIPR_IDC(v)	(((v) << 26) & GENMASK(27, 26))
#define G1_REG_FRAMENUM_LEN(v)		(((v) << 16) & GENMASK(20, 16))
#define G1_REG_FRAMENUM(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_CONST_INTRA_E(v)		((v) ? BIT(31) : 0)
#define G1_REG_FILT_CTRL_PRES(v)	((v) ? BIT(30) : 0)
#define G1_REG_RDPIC_CNT_PRES(v)	((v) ? BIT(29) : 0)
#define G1_REG_8X8TRANS_FLAG_E(v)	((v) ? BIT(28) : 0)
#define G1_REG_REFPIC_MK_LEN(v)		(((v) << 17) & GENMASK(27, 17))
#define G1_REG_IDR_PIC_E(v)		((v) ? BIT(16) : 0)
#define G1_REG_IDR_PIC_ID(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_PPS_ID(v)		(((v) << 24) & GENMASK(31, 24))
#define G1_REG_REFIDX1_ACTIVE(v)	(((v) << 19) & GENMASK(23, 19))
#define G1_REG_REFIDX0_ACTIVE(v)	(((v) << 14) & GENMASK(18, 14))
#define G1_REG_POC_LENGTH(v)		(((v) << 0) & GENMASK(7, 0))

#define G1_REG_PINIT_RLIST_F9(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_PINIT_RLIST_F8(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_PINIT_RLIST_F7(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_PINIT_RLIST_F6(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_PINIT_RLIST_F5(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_PINIT_RLIST_F4(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_PINIT_RLIST_F15(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_PINIT_RLIST_F14(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_PINIT_RLIST_F13(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_PINIT_RLIST_F12(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_PINIT_RLIST_F11(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_PINIT_RLIST_F10(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_REFER1_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER0_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER3_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER2_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER5_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER4_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER7_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER6_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER9_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER8_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER11_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER10_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER13_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER12_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER15_NBR(v)		(((v) << 16) & GENMASK(31, 16))
#define G1_REG_REFER14_NBR(v)		(((v) << 0) & GENMASK(15, 0))

#define G1_REG_REFER_LTERM_E(v)		(((v) << 0) & GENMASK(31, 0))

#define G1_REG_REFER_VALID_E(v)		(((v) << 0) & GENMASK(31, 0))

#define G1_REG_BINIT_RLIST_B2(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_BINIT_RLIST_F2(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_BINIT_RLIST_B1(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_BINIT_RLIST_F1(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B0(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F0(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_BINIT_RLIST_B5(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_BINIT_RLIST_F5(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_BINIT_RLIST_B4(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_BINIT_RLIST_F4(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B3(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F3(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_BINIT_RLIST_B8(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_BINIT_RLIST_F8(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_BINIT_RLIST_B7(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_BINIT_RLIST_F7(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B6(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F6(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_BINIT_RLIST_B11(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_BINIT_RLIST_F11(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_BINIT_RLIST_B10(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_BINIT_RLIST_F10(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B9(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F9(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_BINIT_RLIST_B14(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_BINIT_RLIST_F14(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_BINIT_RLIST_B13(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_BINIT_RLIST_F13(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B12(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F12(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_PINIT_RLIST_F3(v)	(((v) << 25) & GENMASK(29, 25))
#define G1_REG_PINIT_RLIST_F2(v)	(((v) << 20) & GENMASK(24, 20))
#define G1_REG_PINIT_RLIST_F1(v)	(((v) << 15) & GENMASK(19, 15))
#define G1_REG_PINIT_RLIST_F0(v)	(((v) << 10) & GENMASK(14, 10))
#define G1_REG_BINIT_RLIST_B15(v)	(((v) << 5) & GENMASK(9, 5))
#define G1_REG_BINIT_RLIST_F15(v)	(((v) << 0) & GENMASK(4, 0))

#define G1_REG_STARTMB_X(v)		(((v) << 23) & GENMASK(31, 23))
#define G1_REG_STARTMB_Y(v)		(((v) << 15) & GENMASK(22, 15))

#define G1_REG_PRED_BC_TAP_0_0(v)	(((v) << 22) & GENMASK(31, 22))
#define G1_REG_PRED_BC_TAP_0_1(v)	(((v) << 12) & GENMASK(21, 12))
#define G1_REG_PRED_BC_TAP_0_2(v)	(((v) << 2) & GENMASK(11, 2))

#define G1_REG_REFBU_E(v)		((v) ? BIT(31) : 0)

#define G1_REG_APF_THRESHOLD(v)		(((v) << 0) & GENMASK(13, 0))

void hantro_g1_h264_dec_run(struct hantro_ctx *ctx)
{
	struct hantro_dev *vpu = ctx->dev;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	const struct hantro_h264_dec_ctrls *ctrls;
	const struct v4l2_ctrl_h264_decode_params *decode;
	const struct v4l2_ctrl_h264_slice_params *slices;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const u8 *b0_reflist, *b1_reflist, *p_reflist;
	dma_addr_t addr;
	size_t offset = 0;
	u32 reg;

	/* Prepare the H264 decoder context. */
	if (hantro_h264_dec_prepare_run(ctx))
		return;

	src_buf = hantro_get_src_buf(ctx);
	dst_buf = hantro_get_dst_buf(ctx);

	ctrls = &ctx->h264_dec.ctrls;
	decode = ctrls->decode;
	slices = ctrls->slices;
	sps = ctrls->sps;
	pps = ctrls->pps;

	b0_reflist = ctx->h264_dec.reflists.b0;
	b1_reflist = ctx->h264_dec.reflists.b1;
	p_reflist = ctx->h264_dec.reflists.p;

	reg = G1_REG_DEC_AXI_RD_ID(0xff) |
	      G1_REG_DEC_TIMEOUT_E(1) |
	      G1_REG_DEC_STRSWAP32_E(1) |
	      G1_REG_DEC_STRENDIAN_E(1) |
	      G1_REG_DEC_INSWAP32_E(1) |
	      G1_REG_DEC_OUTSWAP32_E(1) |
	      G1_REG_DEC_DATA_DISC_E(0) |
	      G1_REG_DEC_LATENCY(0) |
	      G1_REG_DEC_CLK_GATE_E(1) |
	      G1_REG_DEC_IN_ENDIAN(0) |
	      G1_REG_DEC_OUT_ENDIAN(1) |
	      G1_REG_DEC_ADV_PRE_DIS(0) |
	      G1_REG_DEC_SCMD_DIS(0) |
	      G1_REG_DEC_MAX_BURST(16);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(2));

	reg = G1_REG_DEC_MODE(0) |
	      G1_REG_RLC_MODE_E(0) |
	      G1_REG_PIC_INTERLACE_E(!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY) && (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD || slices[0].flags & V4L2_H264_SLICE_FLAG_FIELD_PIC)) |
	      G1_REG_PIC_FIELDMODE_E(slices[0].flags & V4L2_H264_SLICE_FLAG_FIELD_PIC) |
	      G1_REG_PIC_TOPFIELD_E(!(slices[0].flags & V4L2_H264_SLICE_FLAG_BOTTOM_FIELD)) |
	      G1_REG_FILTERING_DIS(0) |
	      G1_REG_PIC_FIXED_QUANT(0) |
	      G1_REG_WRITE_MVS_E(sps->profile_idc > 66 && decode->nal_ref_idc) |
	      G1_REG_SEQ_MBAFF_E(sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD) |
	      G1_REG_PICORD_COUNT_E(sps->profile_idc > 66) |
	      G1_REG_DEC_AXI_WR_ID(0);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(3));

	reg = G1_REG_PIC_MB_WIDTH(MB_WIDTH(ctx->src_fmt.width)) |
	      G1_REG_PIC_MB_HEIGHT_P(MB_HEIGHT(ctx->src_fmt.height)) |
	      G1_REG_REF_FRAMES(sps->max_num_ref_frames);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(4));

	reg = G1_REG_STRM_START_BIT(0) |
	      G1_REG_TYPE1_QUANT_E(1) |
	      G1_REG_CH_QP_OFFSET(pps->chroma_qp_index_offset) |
	      G1_REG_CH_QP_OFFSET2(pps->second_chroma_qp_index_offset) |
	      G1_REG_FIELDPIC_FLAG_E(!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(5));

	reg = G1_REG_START_CODE_E(1) |
	      G1_REG_INIT_QP(pps->pic_init_qp_minus26 + 26) |
	      G1_REG_CH_8PIX_ILEAV_E(0) |
	      G1_REG_STREAM_LEN(vb2_get_plane_payload(&src_buf->vb2_buf, 0));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(6));

	reg = G1_REG_CABAC_E(pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) |
	      G1_REG_BLACKWHITE_E(sps->profile_idc >= 100 && sps->chroma_format_idc == 0) |
	      G1_REG_DIR_8X8_INFER_E(sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE) |
	      G1_REG_WEIGHT_PRED_E(pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) |
	      G1_REG_WEIGHT_BIPR_IDC(pps->weighted_bipred_idc) |
	      G1_REG_FRAMENUM_LEN(sps->log2_max_frame_num_minus4 + 4) |
	      G1_REG_FRAMENUM(slices[0].frame_num);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(7));

	reg = G1_REG_CONST_INTRA_E(pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED) |
	      G1_REG_FILT_CTRL_PRES(pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) |
	      G1_REG_RDPIC_CNT_PRES(pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT) |
	      G1_REG_8X8TRANS_FLAG_E(pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) |
	      G1_REG_REFPIC_MK_LEN(slices[0].dec_ref_pic_marking_bit_size) |
	      G1_REG_IDR_PIC_E(decode->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC) |
	      G1_REG_IDR_PIC_ID(slices[0].idr_pic_id);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(8));

	reg = G1_REG_PPS_ID(slices[0].pic_parameter_set_id) |
	      G1_REG_REFIDX1_ACTIVE(pps->num_ref_idx_l1_default_active_minus1 + 1) |
	      G1_REG_REFIDX0_ACTIVE(pps->num_ref_idx_l0_default_active_minus1 + 1) |
	      G1_REG_POC_LENGTH(slices[0].pic_order_cnt_bit_size);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(9));

	reg = G1_REG_PINIT_RLIST_F9(p_reflist[9]) |
	      G1_REG_PINIT_RLIST_F8(p_reflist[8]) |
	      G1_REG_PINIT_RLIST_F7(p_reflist[7]) |
	      G1_REG_PINIT_RLIST_F6(p_reflist[6]) |
	      G1_REG_PINIT_RLIST_F5(p_reflist[5]) |
	      G1_REG_PINIT_RLIST_F4(p_reflist[4]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(10));

	reg = G1_REG_PINIT_RLIST_F15(p_reflist[15]) |
	      G1_REG_PINIT_RLIST_F14(p_reflist[14]) |
	      G1_REG_PINIT_RLIST_F13(p_reflist[13]) |
	      G1_REG_PINIT_RLIST_F12(p_reflist[12]) |
	      G1_REG_PINIT_RLIST_F11(p_reflist[11]) |
	      G1_REG_PINIT_RLIST_F10(p_reflist[10]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(11));

	reg = G1_REG_REFER1_NBR(hantro_h264_get_ref_nbr(ctx, 1)) |
	      G1_REG_REFER0_NBR(hantro_h264_get_ref_nbr(ctx, 0));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(30));

	reg = G1_REG_REFER3_NBR(hantro_h264_get_ref_nbr(ctx, 3)) |
	      G1_REG_REFER2_NBR(hantro_h264_get_ref_nbr(ctx, 2));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(31));

	reg = G1_REG_REFER5_NBR(hantro_h264_get_ref_nbr(ctx, 5)) |
	      G1_REG_REFER4_NBR(hantro_h264_get_ref_nbr(ctx, 4));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(32));

	reg = G1_REG_REFER7_NBR(hantro_h264_get_ref_nbr(ctx, 7)) |
	      G1_REG_REFER6_NBR(hantro_h264_get_ref_nbr(ctx, 6));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(33));

	reg = G1_REG_REFER9_NBR(hantro_h264_get_ref_nbr(ctx, 9)) |
	      G1_REG_REFER8_NBR(hantro_h264_get_ref_nbr(ctx, 8));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(34));

	reg = G1_REG_REFER11_NBR(hantro_h264_get_ref_nbr(ctx, 11)) |
	      G1_REG_REFER10_NBR(hantro_h264_get_ref_nbr(ctx, 10));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(35));

	reg = G1_REG_REFER13_NBR(hantro_h264_get_ref_nbr(ctx, 13)) |
	      G1_REG_REFER12_NBR(hantro_h264_get_ref_nbr(ctx, 12));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(36));

	reg = G1_REG_REFER15_NBR(hantro_h264_get_ref_nbr(ctx, 15)) |
	      G1_REG_REFER14_NBR(hantro_h264_get_ref_nbr(ctx, 14));
	vdpu_write_relaxed(vpu, reg, G1_SWREG(37));

	reg = G1_REG_REFER_LTERM_E(ctx->h264_dec.dpb_longterm);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(38));

	reg = G1_REG_REFER_VALID_E(ctx->h264_dec.dpb_valid);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(39));

	reg = G1_REG_BINIT_RLIST_B2(b1_reflist[2]) |
	      G1_REG_BINIT_RLIST_F2(b0_reflist[2]) |
	      G1_REG_BINIT_RLIST_B1(b1_reflist[1]) |
	      G1_REG_BINIT_RLIST_F1(b0_reflist[1]) |
	      G1_REG_BINIT_RLIST_B0(b1_reflist[0]) |
	      G1_REG_BINIT_RLIST_F0(b0_reflist[0]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(42));

	reg = G1_REG_BINIT_RLIST_B5(b1_reflist[5]) |
	      G1_REG_BINIT_RLIST_F5(b0_reflist[5]) |
	      G1_REG_BINIT_RLIST_B4(b1_reflist[4]) |
	      G1_REG_BINIT_RLIST_F4(b0_reflist[4]) |
	      G1_REG_BINIT_RLIST_B3(b1_reflist[3]) |
	      G1_REG_BINIT_RLIST_F3(b0_reflist[3]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(43));

	reg = G1_REG_BINIT_RLIST_B8(b1_reflist[8]) |
	      G1_REG_BINIT_RLIST_F8(b0_reflist[8]) |
	      G1_REG_BINIT_RLIST_B7(b1_reflist[7]) |
	      G1_REG_BINIT_RLIST_F7(b0_reflist[7]) |
	      G1_REG_BINIT_RLIST_B6(b1_reflist[6]) |
	      G1_REG_BINIT_RLIST_F6(b0_reflist[6]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(44));

	reg = G1_REG_BINIT_RLIST_B11(b1_reflist[11]) |
	      G1_REG_BINIT_RLIST_F11(b0_reflist[11]) |
	      G1_REG_BINIT_RLIST_B10(b1_reflist[10]) |
	      G1_REG_BINIT_RLIST_F10(b0_reflist[10]) |
	      G1_REG_BINIT_RLIST_B9(b1_reflist[9]) |
	      G1_REG_BINIT_RLIST_F9(b0_reflist[9]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(45));

	reg = G1_REG_BINIT_RLIST_B14(b1_reflist[14]) |
	      G1_REG_BINIT_RLIST_F14(b0_reflist[14]) |
	      G1_REG_BINIT_RLIST_B13(b1_reflist[13]) |
	      G1_REG_BINIT_RLIST_F13(b0_reflist[13]) |
	      G1_REG_BINIT_RLIST_B12(b1_reflist[12]) |
	      G1_REG_BINIT_RLIST_F12(b0_reflist[12]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(46));

	reg = G1_REG_PINIT_RLIST_F3(p_reflist[3]) |
	      G1_REG_PINIT_RLIST_F2(p_reflist[2]) |
	      G1_REG_PINIT_RLIST_F1(p_reflist[1]) |
	      G1_REG_PINIT_RLIST_F0(p_reflist[0]) |
	      G1_REG_BINIT_RLIST_B15(b1_reflist[15]) |
	      G1_REG_BINIT_RLIST_F15(b0_reflist[15]);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(47));

	reg = G1_REG_STARTMB_X(0) |
	      G1_REG_STARTMB_Y(0);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(48));

	reg = G1_REG_PRED_BC_TAP_0_0(1) |
	      G1_REG_PRED_BC_TAP_0_1((u32)-5) |
	      G1_REG_PRED_BC_TAP_0_2(20);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(49));

	reg = G1_REG_REFBU_E(0);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(51));

	reg = G1_REG_APF_THRESHOLD(8);
	vdpu_write_relaxed(vpu, reg, G1_SWREG(55));

	/* Auxiliary buffer prepared in hantro_g1_h264_dec_prepare_table(). */
	vdpu_write_relaxed(vpu, ctx->h264_dec.priv.dma, G1_REG_QTABLE_BASE);

	/* Source (stream) buffer. */
	addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
	vdpu_write_relaxed(vpu, addr, G1_REG_RLC_VLC_BASE);

	/* Destination (decoded frame) buffer. */
	addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
	/* Adjust dma addr to start at second line for bottom field */
	if (slices[0].flags & V4L2_H264_SLICE_FLAG_BOTTOM_FIELD)
		offset = ALIGN(ctx->src_fmt.width, MB_DIM);
	vdpu_write_relaxed(vpu, addr + offset, G1_REG_DEC_OUT_BASE);

	/* Higher profiles require DMV buffer appended to reference frames. */
	if (sps->profile_idc > 66 && decode->nal_ref_idc) {
		unsigned int bytes_per_mb = 384;

		/* DMV buffer for monochrome start directly after Y-plane */
		if (sps->profile_idc >= 100 && sps->chroma_format_idc == 0)
			bytes_per_mb = 256;
		offset = bytes_per_mb * MB_WIDTH(ctx->src_fmt.width) *
			 MB_HEIGHT(ctx->src_fmt.height);

		/*
		 * DMV buffer is split in two for field encoded frames,
		 * adjust offset for bottom field
		 */
		if (slices[0].flags & V4L2_H264_SLICE_FLAG_BOTTOM_FIELD)
			offset += 32 * MB_WIDTH(ctx->src_fmt.width) *
				  MB_HEIGHT(ctx->src_fmt.height);
		vdpu_write_relaxed(vpu, addr + offset, G1_REG_DIR_MV_BASE);
	}

	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 0), G1_REG_REFER0_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 1), G1_REG_REFER1_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 2), G1_REG_REFER2_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 3), G1_REG_REFER3_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 4), G1_REG_REFER4_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 5), G1_REG_REFER5_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 6), G1_REG_REFER6_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 7), G1_REG_REFER7_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 8), G1_REG_REFER8_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 9), G1_REG_REFER9_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 10), G1_REG_REFER10_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 11), G1_REG_REFER11_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 12), G1_REG_REFER12_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 13), G1_REG_REFER13_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 14), G1_REG_REFER14_BASE);
	vdpu_write_relaxed(vpu, hantro_h264_get_ref_buf(ctx, 15), G1_REG_REFER15_BASE);

	hantro_finish_run(ctx);

	/* Start decoding! */
	reg = G1_REG_DEC_E(1);
	vdpu_write(vpu, reg, G1_SWREG(1));
}
