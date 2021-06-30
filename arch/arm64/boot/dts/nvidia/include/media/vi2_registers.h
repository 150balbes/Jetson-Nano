/*
 * Tegra VI/CSI register offsets
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __REGISTERS_H__
#define __REGISTERS_H__

/* VI registers */
#define	TEGRA_VI_SYNCPT_WAIT_TIMEOUT			200
#define	TEGRA_VI_CFG_VI_INCR_SYNCPT			0x000
#define	VI_CFG_VI_INCR_SYNCPT_COND(x)			(x << 8)
#define	VI_CSI_PP_LINE_START(port)			(4 + (port) * 4)
#define	VI_CSI_PP_FRAME_START(port)			(5 + (port) * 4)
#define	VI_CSI_MW_REQ_DONE(port)			(6 + (port) * 4)
#define	VI_CSI_MW_ACK_DONE(port)			(7 + (port) * 4)

#define	TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL		0x004
#define	TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR		0x008
#define	TEGRA_VI_CFG_CTXSW				0x020
#define	TEGRA_VI_CFG_INTSTATUS				0x024
#define	TEGRA_VI_CFG_PWM_CONTROL			0x038
#define	TEGRA_VI_CFG_PWM_HIGH_PULSE			0x03c
#define	TEGRA_VI_CFG_PWM_LOW_PULSE			0x040
#define	TEGRA_VI_CFG_PWM_SELECT_PULSE_A			0x044
#define	TEGRA_VI_CFG_PWM_SELECT_PULSE_B			0x048
#define	TEGRA_VI_CFG_PWM_SELECT_PULSE_C			0x04c
#define	TEGRA_VI_CFG_PWM_SELECT_PULSE_D			0x050
#define	TEGRA_VI_CFG_VGP1				0x064
#define	TEGRA_VI_CFG_VGP2				0x068
#define	TEGRA_VI_CFG_VGP3				0x06c
#define	TEGRA_VI_CFG_VGP4				0x070
#define	TEGRA_VI_CFG_VGP5				0x074
#define	TEGRA_VI_CFG_VGP6				0x078
#define	TEGRA_VI_CFG_INTERRUPT_MASK			0x08c
#define	TEGRA_VI_CFG_INTERRUPT_TYPE_SELECT		0x090
#define	TEGRA_VI_CFG_INTERRUPT_POLARITY_SELECT		0x094
#define	TEGRA_VI_CFG_INTERRUPT_STATUS			0x098
#define	TEGRA_VI_CFG_VGP_SYNCPT_CONFIG			0x0ac
#define	TEGRA_VI_CFG_VI_SW_RESET			0x0b4
#define	TEGRA_VI_CFG_CG_CTRL				0x0b8
#define	VI_CG_2ND_LEVEL_EN				0x1
#define	TEGRA_VI_CFG_VI_MCCIF_FIFOCTRL			0x0e4
#define	TEGRA_VI_CFG_TIMEOUT_WCOAL_VI			0x0e8
#define	TEGRA_VI_CFG_DVFS				0x0f0
#define	TEGRA_VI_CFG_RESERVE				0x0f4
#define	TEGRA_VI_CFG_RESERVE_1				0x0f8

/* CSI registers */
#define	TEGRA_VI_CSI_BASE(x)				(0x100 + (x) * 0x100)

#define	TEGRA_VI_CSI_SW_RESET				0x000
#define	TEGRA_VI_CSI_SINGLE_SHOT			0x004
#define	SINGLE_SHOT_CAPTURE				0x1
#define	CAPTURE_GOOD_FRAME				0x1
#define	TEGRA_VI_CSI_SINGLE_SHOT_STATE_UPDATE		0x008
#define	TEGRA_VI_CSI_IMAGE_DEF				0x00c
#define	BYPASS_PXL_TRANSFORM_OFFSET			24
#define	IMAGE_DEF_FORMAT_OFFSET				16
#define	IMAGE_DEF_DEST_MEM				0x1
#define	TEGRA_VI_CSI_RGB2Y_CTRL				0x010
#define	TEGRA_VI_CSI_MEM_TILING				0x014
#define	TEGRA_VI_CSI_IMAGE_SIZE				0x018
#define	IMAGE_SIZE_HEIGHT_OFFSET			16
#define	TEGRA_VI_CSI_IMAGE_SIZE_WC			0x01c
#define	TEGRA_VI_CSI_IMAGE_DT				0x020
#define	TEGRA_VI_CSI_SURFACE0_OFFSET_MSB		0x024
#define	TEGRA_VI_CSI_SURFACE0_OFFSET_LSB		0x028
#define	TEGRA_VI_CSI_SURFACE1_OFFSET_MSB		0x02c
#define	TEGRA_VI_CSI_SURFACE1_OFFSET_LSB		0x030
#define	TEGRA_VI_CSI_SURFACE2_OFFSET_MSB		0x034
#define	TEGRA_VI_CSI_SURFACE2_OFFSET_LSB		0x038
#define	TEGRA_VI_CSI_SURFACE0_BF_OFFSET_MSB		0x03c
#define	TEGRA_VI_CSI_SURFACE0_BF_OFFSET_LSB		0x040
#define	TEGRA_VI_CSI_SURFACE1_BF_OFFSET_MSB		0x044
#define	TEGRA_VI_CSI_SURFACE1_BF_OFFSET_LSB		0x048
#define	TEGRA_VI_CSI_SURFACE2_BF_OFFSET_MSB		0x04c
#define	TEGRA_VI_CSI_SURFACE2_BF_OFFSET_LSB		0x050
#define	TEGRA_VI_CSI_SURFACE0_STRIDE			0x054
#define	TEGRA_VI_CSI_SURFACE1_STRIDE			0x058
#define	TEGRA_VI_CSI_SURFACE2_STRIDE			0x05c
#define	TEGRA_VI_CSI_SURFACE_HEIGHT0			0x060
#define	TEGRA_VI_CSI_ISPINTF_CONFIG			0x064
#define	TEGRA_VI_CSI_ERROR_STATUS			0x084
#define	TEGRA_VI_CSI_ERROR_INT_MASK			0x088
#define	TEGRA_VI_CSI_WD_CTRL				0x08c
#define	TEGRA_VI_CSI_WD_PERIOD				0x090

/* CSI Pixel Parser registers: Starts from 0x838, offset 0x0 */
#define TEGRA_CSI_INPUT_STREAM_CONTROL                  0x000
#define	CSI_SKIP_PACKET_THRESHOLD_OFFSET		16

#define TEGRA_CSI_PIXEL_STREAM_CONTROL0                 0x004
#define CSI_PP_PACKET_HEADER_SENT			(0x1 << 4)
#define CSI_PP_DATA_IDENTIFIER_ENABLE			(0x1 << 5)
#define CSI_PP_WORD_COUNT_SELECT_HEADER			(0x1 << 6)
#define CSI_PP_CRC_CHECK_ENABLE				(0x1 << 7)
#define CSI_PP_WC_CHECK					(0x1 << 8)
#define CSI_PP_OUTPUT_FORMAT_STORE			(0x3 << 16)
#define CSI_PPA_PAD_LINE_NOPAD				(0x2 << 24)
#define CSI_PP_HEADER_EC_DISABLE			(0x1 << 27)
#define CSI_PPA_PAD_FRAME_NOPAD				(0x2 << 28)

#define TEGRA_CSI_PIXEL_STREAM_CONTROL1                 0x008
#define CSI_PP_TOP_FIELD_FRAME_OFFSET			0
#define CSI_PP_TOP_FIELD_FRAME_MASK_OFFSET		4

#define TEGRA_CSI_PIXEL_STREAM_GAP                      0x00c
#define PP_FRAME_MIN_GAP_OFFSET				16

#define TEGRA_CSI_PIXEL_STREAM_PP_COMMAND               0x010
#define CSI_PP_ENABLE					0x1
#define CSI_PP_DISABLE					0x2
#define CSI_PP_RST					0x3
#define CSI_PP_SINGLE_SHOT_ENABLE			(0x1 << 2)
#define CSI_PP_START_MARKER_FRAME_MAX_OFFSET		12

#define TEGRA_CSI_PIXEL_STREAM_EXPECTED_FRAME           0x014
#define TEGRA_CSI_PIXEL_PARSER_INTERRUPT_MASK           0x018
#define TEGRA_CSI_PIXEL_PARSER_STATUS                   0x01c
#define TEGRA_CSI_CSI_SW_SENSOR_RESET                   0x020

/* CSI PHY registers */
/* CSI_PHY_CIL_COMMAND_0 offset 0x0d0 from TEGRA_CSI_PIXEL_PARSER_0_BASE */
#define TEGRA_CSI_PHY_CIL_COMMAND                       0x0d0
#define CSI_A_PHY_CIL_NOP				0x0
#define CSI_A_PHY_CIL_ENABLE				0x1
#define CSI_A_PHY_CIL_DISABLE				0x2
#define CSI_A_PHY_CIL_ENABLE_MASK			0x3
#define CSI_B_PHY_CIL_NOP				(0x0 << 8)
#define CSI_B_PHY_CIL_ENABLE				(0x1 << 8)
#define CSI_B_PHY_CIL_DISABLE				(0x2 << 8)
#define CSI_B_PHY_CIL_ENABLE_MASK			(0x3 << 8)

/* CSI CIL registers: Starts from 0x92c, offset 0xF4 */
#define TEGRA_CSI_CIL_OFFSET				0x0f4

#define TEGRA_CSI_CIL_PAD_CONFIG0                       0x000
#define BRICK_CLOCK_A_4X				(0x1 << 16)
#define BRICK_CLOCK_B_4X				(0x2 << 16)
#define TEGRA_CSI_CIL_PAD_CONFIG1                       0x004
#define TEGRA_CSI_CIL_PHY_CONTROL                       0x008
#define BYPASS_LP_SEQ					(0x1 << 6)
#define BYPASS_LP_SEQ_SHIFT				6
#define TEGRA_CSI_CIL_INTERRUPT_MASK                    0x00c
#define TEGRA_CSI_CIL_STATUS                            0x010
#define TEGRA_CSI_CILX_STATUS                           0x014
#define TEGRA_CSI_CIL_ESCAPE_MODE_COMMAND               0x018
#define TEGRA_CSI_CIL_ESCAPE_MODE_DATA                  0x01c
#define TEGRA_CSI_CIL_SW_SENSOR_RESET                   0x020

/* CSI Pattern Generator registers: Starts from 0x9c4, offset 0x18c */
#define TEGRA_CSI_TPG_OFFSET				0x18c

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL		0x000
#define PG_MODE_OFFSET					2
#define PG_ENABLE					0x1
#define PG_DISABLE					0x0

#define PG_VBLANK_OFFSET				16
#define TEGRA_CSI_PG_BLANK				0x004
#define TEGRA_CSI_PG_PHASE				0x008
#define TEGRA_CSI_PG_RED_FREQ				0x00c
#define PG_RED_VERT_INIT_FREQ_OFFSET			16
#define PG_RED_HOR_INIT_FREQ_OFFSET			0

#define TEGRA_CSI_PG_RED_FREQ_RATE			0x010
#define TEGRA_CSI_PG_GREEN_FREQ				0x014
#define PG_GREEN_VERT_INIT_FREQ_OFFSET			16
#define PG_GREEN_HOR_INIT_FREQ_OFFSET			0

#define TEGRA_CSI_PG_GREEN_FREQ_RATE			0x018
#define TEGRA_CSI_PG_BLUE_FREQ				0x01c
#define PG_BLUE_VERT_INIT_FREQ_OFFSET			16
#define PG_BLUE_HOR_INIT_FREQ_OFFSET			0

#define TEGRA_CSI_PG_BLUE_FREQ_RATE			0x020
#define TEGRA_CSI_PG_AOHDR				0x024

#define TEGRA_CSI_DPCM_CTRL_A				0xa2c
#define TEGRA_CSI_DPCM_CTRL_B				0xa30

/* Other CSI registers: Starts from 0xa44, offset 0x20c */
#define TEGRA_CSI_STALL_COUNTER				0x20c
#define TEGRA_CSI_CSI_READONLY_STATUS			0x210
#define TEGRA_CSI_CSI_SW_STATUS_RESET			0x214
#define TEGRA_CSI_CLKEN_OVERRIDE			0x218
#define TEGRA_CSI_DEBUG_CONTROL				0x21c
#define TEGRA_CSI_DEBUG_COUNTER_0			0x220
#define TEGRA_CSI_DEBUG_COUNTER_1			0x224
#define TEGRA_CSI_DEBUG_COUNTER_2			0x228


/* CSI Pixel Parser registers */
#define TEGRA_CSI_PIXEL_PARSER_0_BASE			0x0838
#define TEGRA_CSI_PIXEL_PARSER_1_BASE			0x086c
#define TEGRA_CSI_PIXEL_PARSER_2_BASE			0x1038
#define TEGRA_CSI_PIXEL_PARSER_3_BASE			0x106c
#define TEGRA_CSI_PIXEL_PARSER_4_BASE			0x1838
#define TEGRA_CSI_PIXEL_PARSER_5_BASE			0x186c

/* CSIA to CSIB register offset */
#define TEGRA_CSI_PORT_OFFSET				0x34

#define INVALID_CSI_PORT				0xFF
#define TEGRA_CSI_BLOCKS				3
#define	SYNCPT_FIFO_DEPTH				2
#define	PREVIOUS_BUFFER_DEC_INDEX			2

#define	TEGRA_CLOCK_VI_MAX				793600000
#define	TEGRA_CLOCK_TPG					927000000
#define	TEGRA_CLOCK_CSI_PORT_MAX			102000000

#define	TEGRA_SURFACE_ALIGNMENT				64
#endif
