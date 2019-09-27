/*
 * tc358840_regs.h - Toshiba UH2C/D HDMI-CSI bridge registers
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TC358840_TABLES__
#define __TC358840_TABLES__

#include <media/camera_common.h>

/**************************************************
 * Register Addresses
 *************************************************/

/* *** General (16 bit) *** */
#define CHIPID_ADDR				0x0000
#define MASK_CHIPID				0xFF00
#define MASK_REVID				0x00FF
#define TC358840_CHIPID				0x4700

#define SYSCTL					0x0002
#define MASK_SLEEP				(1 << 0)
#define MASK_I2SDIS				(1 << 7)
#define MASK_HDMIRST				(1 << 8)
#define MASK_CTXRST				(1 << 9)
#define MASK_CECRST				(1 << 10)
#define MASK_IRRST				(1 << 11)
#define MASK_SPLRST				(1 << 12)
#define MASK_ABRST				(1 << 14)
#define MASK_RESET_ALL				0x5F80

#define CONFCTL0				0x0004
#define MASK_VTX0EN				(1 << 0)
#define MASK_VTX1EN				(1 << 1)
#define MASK_AUTOINDEX				(1 << 2)
#define MASK_AUDOUTSEL_CSITX0			(0 << 3)
#define MASK_AUDOUTSEL_CSITX1			(1 << 3)
#define MASK_AUDOUTSEL_I2S			(2 << 3)
#define MASK_AUDOUTSEL_TDM			(3 << 3)
#define MASK_ABUFEN				(1 << 5)
#define MASK_YCBCRFMT				(3 << 6)
#define MASK_YCBCRFMT_YCBCR444			(0 << 6)
#define MASK_YCBCRFMT_YCBCR422_12		(1 << 6)
#define MASK_YCBCRFMT_VPID2			(2 << 6)
#define MASK_YCBCRFMT_YCBCR422_8		(3 << 6)
#define MASK_I2SDLYOPT				(1 << 8)
#define MASK_AUDCHSEL				(1 << 9)
#define MASK_AUDCHNUM_8				(0 << 10)
#define MASK_AUDCHNUM_6				(1 << 10)
#define MASK_AUDCHNUM_4				(2 << 10)
#define MASK_AUDCHNUM_2				(3 << 10)
#define MASK_ACLKOPT				(1 << 12)
#define MASK_IECEN				(1 << 13)
#define MASK_SLMBEN				(1 << 14)
#define MASK_TX_MSEL				(1 << 15)

#define CONFCTL1				0x0006
#define MASK_TX_OUT_FMT				0x0003
#define MASK_TX_OUT_FMT_RGB888			(0 << 0)
#define MASK_TX_OUT_FMT_RGB666			(2 << 0)
#define MASK_TX_MS_EN				(1 << 2)

/* *** Interrupt (16 bit) *** */
#define INTSTATUS				0x0014
#define INTMASK					0x0016
#define MASK_IR_DINT				(1 << 0)
#define MASK_IR_EINT				(1 << 1)
#define MASK_CEC_RINT				(1 << 2)
#define MASK_CEC_TINT				(1 << 3)
#define MASK_CEC_EINT				(1 << 4)
#define MASK_SYS_INT				(1 << 5)
#define MASK_CSITX0_INT				(1 << 8)
#define MASK_HDMI_INT				(1 << 9)
#define MASK_AMUTE_INT				(1 << 10)
#define MASK_CSITX1_INT				(1 << 11)
#define MASK_INT_STATUS_MASK_ALL		0x0F3F

/* *** Interrupt and MASKs (8 bit) *** */
#define HDMI_INT0				0x8500
#define MASK_KEY				0x01
#define MASK_MISC				0x0002

#define HDMI_INT1				0x8501
#define MASK_SYS				0x01
#define MASK_CLK				0x02
#define MASK_PACKET				0x04
#define MASK_ACBIT				0x08
#define MASK_AUD				0x10
#define MASK_ERR				0x20
#define MASK_HDCP				0x40
#define MASK_GBD				0x80

#define SYS_INT					0x8502
#define SYS_INTM				0x8512
#define MASK_DDC				0x01
#define MASK_TMDS				0x02
#define MASK_DPMBDET				0x04
#define MASK_NOPMBDET				0x08
#define MASK_HDMI				0x10
#define MASK_DVI				0x20
#define MASK_ACRN				0x40
#define MASK_ACR_CTS				0x80

#define CLK_INT					0x8503
#define CLK_INTM				0x8513
#define MASK_TMDSCLK_CHG			0x01
#define MASK_PHYCLK_CHG				0x02
#define MASK_PXCLK_CHG				0x04
#define MASK_DC_CHG				0x08
#define MASK_IN_HV_CHG				0x10
#define MASK_IN_DE_CHG				0x20
#define MASK_OUT_H_CHG				0x40
#define MASK_OUT_DE_CHG				0x80

#define PACKET_INT				0x8504
#define PACKET_INTM				0x8514
#define MASK_PK_AVI				0x01
#define MASK_PK_AUD				0x02
#define MASK_PK_MS				0x04
#define MASK_PK_SPD				0x08
#define MASK_PK_VS				0x10
#define MASK_PK_ACP				0x20
#define MASK_PK_ISRC				0x40
#define MASK_PK_ISRC2				0x80

#define CBIT_INT				0x8505
#define CBIT_INTM				0x8515
#define MASK_CBIT				0x01
#define MASK_CBIT_FS				0x02
#define MASK_CBIT_NLPCM				0x04
#define MASK_AU_HBR				0x08
#define MASK_AU_DSD				0x10
#define MASK_AF_UNLOCK				0x40
#define MASK_AF_LOCK				0x80

#define AUDIO_INT				0x8506
#define AUDIO_INTM				0x8516
#define MASK_BUFINIT_END			0x01
#define MASK_BUF_UNDER				0x02
#define MASK_BUF_NU2				0x04
#define MASK_BUF_NU1				0x08
#define MASK_BUF_CENTER				0x10
#define MASK_BUF_NO1				0x20
#define MASK_BUF_NO2				0x40
#define MASK_BUF_OVER				0x80

#define ERR_INT					0x8507
#define ERR_INTM				0x8517
#define MASK_DC_PPERR				0x01
#define MASK_DC_BUFERR				0x02
#define MASK_DC_DEERR				0x04
#define MASK_DC_NOCD				0x08
#define MASK_NO_AVI				0x10
#define MASK_NO_ACP				0x20
#define MASK_AU_FRAME				0x40
#define MASK_EESS_ERR				0x80

#define HDCP_INT				0x8508
#define HDCP_INTM				0x8518
#define MASK_AN_END				0x01
#define MASK_AKSV_END				0x02
#define MASK_KM_END				0x04
#define MASK_R0_END				0x08
#define MASK_SHA_END				0x10
#define MASK_LINKERR				0x20
#define MASK_AVM_CLR				0x40
#define MASK_AVM_SET				0x80

#define GBD_INT					0x8509
#define GBD_INTM				0x8519
#define MASK_GBD_ON				0x01
#define MASK_GBD_OFF				0x02
#define MASK_P1GBD_DET				0x04
#define MASK_P0GBD_CHG				0x10
#define MASK_P1GBD_CHG				0x20
#define MASK_GBD_ACLR				0x40
#define MASK_GBD_PKERR				0x80

#define MISC_INT				0x850B
#define MISC_INTM				0x851B
#define MASK_AUDIO_MUTE				0x01
#define MASK_SYNC_CHG				0x02
#define MASK_NO_VS				0x04
#define MASK_NO_SPD				0x08
#define MASK_AS_LAYOUT				0x10
#define MASK_VIDEO_COLOR			0x20
#define MASK_AU_HBR_OFF				0x40
#define MASK_AU_DSD_OFF				0x80

/* *** STATUS *** */
#define SYS_STATUS				0x8520
#define MASK_S_SYNC				0x80
#define MASK_S_AVMUTE				0x40
#define MASK_S_HDCP				0x20
#define MASK_S_HDMI				0x10
#define MASK_S_PHY_SCDT				0x08
#define MASK_S_PHY_PLL				0x04
#define MASK_S_TMDS				0x02
#define MASK_S_DDC5V				0x01

#define VI_STATUS1				0x8522
#define MASK_S_V_GBD				0x08
#define MASK_S_DEEPCOLOR			0x0c
#define MASK_S_V_422				0x02
#define MASK_S_V_INTERLACE			0x01

#define VI_STATUS3				0x8528
#define MASK_S_V_COLOR				0x1F
#define MASK_RGB_FULL				0x00
#define MASK_RGB_LIMITED			0x01
#define MASK_YCBCR601_FULL			0x02
#define MASK_YCBCR601_LIMITED			0x03
#define MASK_ADOBE_RGB_FULL			0x04
#define MASK_ADOBE_RGB_LIMITED			0x05
#define MASK_YCBCR709_FULL			0x06
#define MASK_YCBCR709_LIMITED			0x07
#define MASK_XVYCC601_FULL			0x0A
#define MASK_XVYCC601_LIMITED			0x0B
#define MASK_XVYCC709_FULL			0x0E
#define MASK_XVYCC709_LIMITED			0x0F
#define MASK_SYCC601_FULL			0x12
#define MASK_SYCC601_LIMITED			0x13
#define MASK_ADOBE_YCC601_FULL			0x1A
#define MASK_ADOBE_YCC601_LIMITED		0x1B
#define MASK_LIMITED                            0x01


/* *** CSI TX (32 bit) *** */
#define CSITX0_BASE_ADDR			0x0000
#define CSITX1_BASE_ADDR			0x0200

#define CSITX_CLKEN				0x0108
#define MASK_CSITX_EN				(1 << 0)

#define PPICLKEN				0x010C
#define MASK_HSTXCLKEN				0x00000001

#define MODECONF				0x0110	/* Not in Ref. v1.5 */
#define MASK_CSI2MODE				(1 << 0)
#define MASK_VSYNC_POL_SW			(1 << 1)
#define MASK_HSYNC_POL_SW			(1 << 2)
#define MASK_DTVALID_POL_SW			(1 << 3)
#define MASK_INDMODE				(1 << 4)

#define LANEEN					0x0118
#define MASK_LANES				0x00000007
#define MASK_LANE_0_EN				(1 << 0)
#define MASK_LANE_0_1_EN			(2 << 0)
#define MASK_LANE_0_1_2_EN			(3 << 0)
#define MASK_LANE_0_1_2_3_EN			(4 << 0)
#define MASK_LANES				0x00000007
#define MASK_CLANEEN				(1 << 4)

#define CSITX_START				0x011C
#define LINEINITCNT				0x0120
#define HSTOCNT					0x0124

#define INTEN					0x0128	/* Not in Ref. v1.5 */
#define MASK_VH_DLY_EN				(1 << 0)
#define MASK_VFHSYNCMASK_EN			(1 << 7)
#define MASK_IND_MODE_SEL_PORT			(0 << 8)
#define MASK_IND_MODE_SEL_REG			(1 << 8)
#define MASK_IND_TO_EN				(1 << 9)
#define MASK_HSTX_TO_EN				(1 << 10)
#define MASK_LRX_H_TO_EN			(1 << 11)
#define MASK_TA_TO_EN				(1 << 12)
#define MASK_PR_TO_EN				(1 << 13)
#define MASK_PRESP_TO_EN			(1 << 14)
#define MASK_DSI_RX_STATE_INT_EN		(1 << 16)
#define MASK_DSI_RX_TRIG_INT_EN			(1 << 17)
#define MASK_DSI_LP_TX_INT_EN			(1 << 18)
#define MASK_DSI_RX_ERR_INT_EN			(1 << 19)
#define MASK_DSI_RP_TO_INT_EN			(1 << 20)
#define MASK_APP_SIDE_ERR_INT_EN		(1 << 21)
#define MASK_INIT_INT_EN			(1 << 22)
#define MASK_DEBUG_MODE_EN			(1 << 31)

#define FUNCMODE				0x0150
#define MASK_CONTCLKMODE			(1 << 5)
#define MASK_FORCESTOP				(1 << 10)

#define CSITX_INTERNAL_STAT			0x01B0

#define LPTXTIMECNT				0x0254
#define TCLK_HEADERCNT				0x0258
#define TCLK_TRAILCNT				0x025C
#define THS_HEADERCNT				0x0260
#define TWAKEUP					0x0264
#define TCLK_POSTCNT				0x0268
#define THS_TRAILCNT				0x026C
#define HSTXVREGCNT				0x0270

#define HSTXVREGEN				0x0274
#define MASK_D3M_HSTXVREGEN			0x0010
#define MASK_D2M_HSTXVREGEN			0x0008
#define MASK_D1M_HSTXVREGEN			0x0004
#define MASK_D0M_HSTXVREGEN			0x0002
#define MASK_CLM_HSTXVREGEN			0x0001

#define MIPICLKEN				0x02A0
#define MASK_MP_ENABLE				0x00000001
#define MASK_MP_CKEN				0x00000002

#define PLLCONF					0x02AC
#define MASK_LFBREN				(1 << 9)
#define MASK_MPLBW				0x00030000
#define MASK_MPLBW_25				(0 << 16)
#define MASK_MPLBW_33				(1 << 16)
#define MASK_MPLBW_50				(2 << 16)
#define MASK_MPLBW_MAX				(3 << 16)
#define MASK_PLL_FBD				0x000000FF
#define SET_PLL_FBD(fbd)			((fbd) & MASK_PLL_FBD)
#define MASK_PLL_FRS				0x00000C00
#define SET_PLL_FRS(frs)			(((frs) << 10) & MASK_PLL_FRS)
#define MASK_PLL_PRD				0x0000F000
#define SET_PLL_PRD(prd)			(((prd) << 12) & \
						  MASK_PLL_PRD)
#define MASK_PLL_LBW				0x00030000
#define SET_PLL_LBW(lbw)			((((lbw) - 1) << 16) & \
						  MASK_PLL_LBW)

#define CECEN                                 0x0600
#define MASK_CECEN                            0x0001

/* *** Split Control (16 bit) *** */
#define SPLITTX0_CTRL				0x5000
#define SPLITTX1_CTRL				0x5080
#define MASK_LCD_CSEL				0x0001
#define MASK_IFEN				0x0002
#define MASK_SPBP				0x0100

#define SPLITTX0_WC				0x5008	/*Removed in rev. 1.1*/
#define SPLITTX1_WC				0x5088	/*Removed in rev. 1.1*/

#define SPLITTX0_SPLIT				0x500C
#define SPLITTX1_SPLIT				0x508C
#define MASK_FPXV				0x0FFF
/* NOTE: Only available for TX0 */
#define MASK_TX1SEL				0x4000
/* NOTE: Only available for TX0 */
#define MASK_EHW				0x8000

/* *** HDMI PHY (8 bit) *** */
#define PHY_CTL					0x8410
/* TODO: Check name of mask */
#define MASK_POWERCTL				(1 << 0)
/* TODO: Check name of mask */
#define MASK_48_MHZ				(1 << 1)

#define PHY_CTL2				0x8412
#define MASK_PHY_FREE_RUN			(1 << 5)

#define PHY_ENB					0x8413
#define MASK_ENABLE_PHY				0x01

#define PHY_RST					0x8414
#define MASK_RESET_CTRL				0x01	/* Reset active low */

#define APPL_CTL				0x84F0
#define MASK_APLL_ON				0x01
#define MASK_APLL_CPCTL				0x30
#define MASK_APLL_CPCTL_HIZ			0x00
#define MASK_APLL_CPCTL_LFIX			0x10
#define MASK_APLL_CPCTL_HFIX			0x20
#define MASK_APLL_CPCTL_NORMAL			0x30

#define DDCIO_CTL				0x84F4
#define MASK_DDC_PWR_ON				(1 << 0)

/** *** HDMI Clock (8 bit) *** */
#define AU_STATUS0				0x8523
#define MASK_S_A_SAMPLE				0x01

#define SYS_FREQ0				0x8540
#define SYS_FREQ1				0x8541
#define LOCK_REF_FREQA				0x8630
#define LOCK_REF_FREQB				0x8631
#define LOCK_REF_FREQC				0x8632

#define FS_SET					0x8621
#define MASK_FS					0x0F

#define NCO_F0_MOD				0x8670
#define MASK_NCO_F0_MOD_42MHZ			0x00
#define MASK_NCO_F0_MOD_REG			0x02

#define NCO_48F0A				0x8671
#define NCO_48F0B				0x8672
#define NCO_48F0C				0x8673
#define NCO_48F0D				0x8674

#define NCO_44F0A				0x8675
#define NCO_44F0B				0x8676
#define NCO_44F0C				0x8677
#define NCO_44F0D				0x8678

#define SCLK_CSC0				0x8A0C
#define SCLK_CSC1				0x8A0D

#define HDCP_MODE                             0x8560
#define MASK_MODE_RST_TN                      0x20
#define MASK_LINE_REKEY                       0x10
#define MASK_AUTO_CLR                         0x04

#define HDCP_REG1                             0x8563 /* Not in REF_01 */
#define MASK_AUTH_UNAUTH_SEL                  0x70
#define MASK_AUTH_UNAUTH_SEL_12_FRAMES        0x70
#define MASK_AUTH_UNAUTH_SEL_8_FRAMES         0x60
#define MASK_AUTH_UNAUTH_SEL_4_FRAMES         0x50
#define MASK_AUTH_UNAUTH_SEL_2_FRAMES         0x40
#define MASK_AUTH_UNAUTH_SEL_64_FRAMES        0x30
#define MASK_AUTH_UNAUTH_SEL_32_FRAMES        0x20
#define MASK_AUTH_UNAUTH_SEL_16_FRAMES        0x10
#define MASK_AUTH_UNAUTH_SEL_ONCE             0x00
#define MASK_AUTH_UNAUTH                      0x01
#define MASK_AUTH_UNAUTH_AUTO                 0x01

#define HDCP_REG2                             0x8564 /* Not in REF_01 */
#define MASK_AUTO_P3_RESET                    0x0F
#define SET_AUTO_P3_RESET_FRAMES(n)          (n & MASK_AUTO_P3_RESET)
#define MASK_AUTO_P3_RESET_OFF                0x00


/* *** VI *** */
#define VI_MODE					0x8570
/* TODO: Probably wrong bit (see p. 292 rev. 0.93) */
#define MASK_RGB_DVI				0x08

#define DE_HSIZE_LO				0x8582
#define DE_HSIZE_HI				0x8583
#define DE_VSIZE_LO				0x858C
#define DE_VSIZE_HI				0x858D

#define IN_HSIZE_LO				0x858E
#define IN_HSIZE_HI				0x858F

#define IN_VSIZE_LO				0x8590
#define IN_VSIZE_HI				0x8591

#define FV_CNT_LO				0x85C1	/* Not in Ref. v1.5 */
#define FV_CNT_HI				0x85C2	/* Not in Ref. v1.5 */

#define HDCP_REG3                             0x85D1 /* Not in REF_01 */
#define KEY_RD_CMD                            0x01

/* *** EDID (8 bit) *** */
#define EDID_MODE				0x85E0
#define MASK_DIRECT				0x00
#define MASK_RAM_DDC2B				(1 << 0)
#define MASK_RAM_EDDC				(1 << 1)
#define MASK_EDID_MODE_ALL			0x03

#define EDID_LEN1				0x85E3
#define EDID_LEN2				0x85E4

#define EDID_RAM				0x8C00

/* *** HDCP *** */
#define BKSV					0x8800

#define BCAPS                                 0x8840
#define MASK_HDMI_RSVD                        0x80
#define MASK_REPEATER                         0x40
#define MASK_READY                            0x20
#define MASK_FASTI2C                          0x10
#define MASK_1_1_FEA                          0x02
#define MASK_FAST_REAU                        0x01

#define BSTATUS0                              0x8841
#define BSTATUS1                              0x8842
#define MASK_HDMI_MODE                        0x10
#define MASK_MAX_EXCED                        0x08

/* *** Video Output Format (8 bit) *** */
#define VOUT_FMT				0x8A00
#define MASK_OUTFMT_444_RGB			(0 << 0)
#define MASK_OUTFMT_422				(1 << 0)
#define MASK_OUTFMT_THROUGH			(2 << 0)
#define MASK_422FMT_NORMAL			(0 << 4)
#define MASK_422FMT_HDMITHROUGH			(1 << 4)

#define VOUT_FIL				0x8A01
#define MASK_422FIL				0x07
#define MASK_422FIL_2_TAP			(0 << 0)
#define MASK_422FIL_3_TAP			(1 << 0)
#define MASK_422FIL_NO_FILTER			(2 << 0)
#define MASK_422FIL_2_TAP_444			(3 << 0)
#define MASK_422FIL_3_TAP_444			(4 << 0)
#define MASK_422FIL_2_TAP_444_CSC		(5 << 0)
#define MASK_422FIL_3_TAP_444_CSC		(6 << 0)
#define MASK_444FIL				0x10
#define MASK_444FIL_REPEAT			(0 << 4)
#define MASK_444FIL_2_TAP			(1 << 4)

#define VOUT_SYNC0				0x8A02
#define MASK_MODE_2				(2 << 0)
#define MASK_MODE_3				(3 << 0)
#define MASK_M3_HSIZE				0x30
#define MASK_M3_VSIZE				0xC0

#define VOUT_CSC				0x8A08
#define MASK_CSC_MODE				0x03
#define MASK_CSC_MODE_OFF			(0 << 0)
#define MASK_CSC_MODE_BUILTIN			(1 << 0)
#define MASK_CSC_MODE_AUTO			(2 << 0)
#define MASK_CSC_MODE_HOST			(3 << 0)
#define MASK_COLOR				0x70
#define MASK_COLOR_RGB_FULL			(0 << 4)
#define MASK_COLOR_RGB_LIMITED			(1 << 4)
#define MASK_COLOR_601_YCBCR_FULL		(2 << 4)
#define MASK_COLOR_601_YCBCR_LIMITED		(3 << 4)
#define MASK_COLOR_709_YCBCR_FULL		(4 << 4)
#define MASK_COLOR_709_YCBCR_LIMITED		(5 << 4)
#define MASK_COLOR_FULL_TO_LIMITED		(6 << 4)
#define MASK_COLOR_LIMITED_TO_FULL		(7 << 4)


/* *** HDMI Audio RefClk (8 bit) *** */
#define FORCE_MUTE				0x8600
#define MASK_FORCE_DMUTE			(1 << 0)
#define MASK_FORCE_AMUTE			(1 << 4)

#define AUTO_CMD0				0x8602
#define MASK_AUTO_MUTE7				0x80
#define MASK_AUTO_MUTE6				0x40
#define MASK_AUTO_MUTE5				0x20
#define MASK_AUTO_MUTE4				0x10
#define MASK_AUTO_MUTE3				0x08
#define MASK_AUTO_MUTE2				0x04
#define MASK_AUTO_MUTE1				0x02
#define MASK_AUTO_MUTE0				0x01

#define AUTO_CMD1				0x8603
#define MASK_AUTO_MUTE10			0x04
#define MASK_AUTO_MUTE9				0x02
#define MASK_AUTO_MUTE8				0x01

#define AUTO_CMD2				0x8604
#define MASK_AUTO_PLAY3				0x08
#define MASK_AUTO_PLAY2				0x04

#define BUFINIT_START				0x8606
#define SET_BUFINIT_START_MS(milliseconds)	((milliseconds) / 100)

#define FS_MUTE					0x8607
#define MASK_FS_ELSE_MUTE			0x80
#define MASK_FS22_MUTE				0x40
#define MASK_FS24_MUTE				0x20
#define MASK_FS88_MUTE				0x10
#define MASK_FS96_MUTE				0x08
#define MASK_FS176_MUTE				0x04
#define MASK_FS192_MUTE				0x02
#define MASK_FS_NO_MUTE				0x01

#define FS_IMODE				0x8620
#define MASK_NLPCM_HMODE			0x40
#define MASK_NLPCM_SMODE			0x20
#define MASK_NLPCM_IMODE			0x10
#define MASK_FS_HMODE				0x08
#define MASK_FS_AMODE				0x04
#define MASK_FS_SMODE				0x02
#define MASK_FS_IMODE				0x01

#define ACR_MODE				0x8640
#define MASK_ACR_LOAD				0x10
#define MASK_N_MODE				0x04
#define MASK_CTS_MODE				0x01

#define ACR_MDF0				0x8641
#define MASK_ACR_L2MDF				0x70
#define MASK_ACR_L2MDF_0_PPM			0x00
#define MASK_ACR_L2MDF_61_PPM			0x10
#define MASK_ACR_L2MDF_122_PPM			0x20
#define MASK_ACR_L2MDF_244_PPM			0x30
#define MASK_ACR_L2MDF_488_PPM			0x40
#define MASK_ACR_L2MDF_976_PPM			0x50
#define MASK_ACR_L2MDF_1976_PPM			0x60
#define MASK_ACR_L2MDF_3906_PPM			0x70
#define MASK_ACR_L1MDF				0x07
#define MASK_ACR_L1MDF_0_PPM			0x00
#define MASK_ACR_L1MDF_61_PPM			0x01
#define MASK_ACR_L1MDF_122_PPM			0x02
#define MASK_ACR_L1MDF_244_PPM			0x03
#define MASK_ACR_L1MDF_488_PPM			0x04
#define MASK_ACR_L1MDF_976_PPM			0x05
#define MASK_ACR_L1MDF_1976_PPM			0x06
#define MASK_ACR_L1MDF_3906_PPM			0x07

#define ACR_MDF1				0x8642
#define MASK_ACR_L3MDF				0x07
#define MASK_ACR_L3MDF_0_PPM			0x00
#define MASK_ACR_L3MDF_61_PPM			0x01
#define MASK_ACR_L3MDF_122_PPM			0x02
#define MASK_ACR_L3MDF_244_PPM			0x03
#define MASK_ACR_L3MDF_488_PPM			0x04
#define MASK_ACR_L3MDF_976_PPM			0x05
#define MASK_ACR_L3MDF_1976_PPM			0x06
#define MASK_ACR_L3MDF_3906_PPM			0x07

#define SDO_MODE1				0x8652
#define MASK_SDO_BIT_LENG			0x70
#define MASK_SDO_FMT				0x03
#define MASK_SDO_FMT_RIGHT			0x00
#define MASK_SDO_FMT_LEFT			0x01
#define MASK_SDO_FMT_I2S			0x02

#define DIV_MODE				0x8665 /* Not in REF_01 */
#define MASK_DIV_DLY				0xf0
#define SET_DIV_DLY_MS(milliseconds)		((((milliseconds)/100) << 4) & \
						MASK_DIV_DLY)
#define MASK_DIV_MODE				0x01

#define HDMIAUDIO_MODE				0x8680

/* *** HDMI General (16 bit) *** */
#define DDC_CTL					0x8543
#define MASK_DDC_ACTION				0x04
#define MASK_DDC5V_MODE				0x03
#define MASK_DDC5V_MODE_0MS			0x00
#define MASK_DDC5V_MODE_50MS			0x01
#define MASK_DDC5V_MODE_100MS			0x02
#define MASK_DDC5V_MODE_200MS			0x03

#define HPD_CTL					0x8544
#define MASK_HPD_OUT0				(1 << 0)
#define MASK_HPD_CTL0				(1 << 4)

#define INIT_END				0x854A
#define MASK_INIT_END				0x01

/* *** Video Mute *** */
#define VI_MUTE					0x857F
#define MASK_AUTO_MUTE				0xC0
#define MASK_VI_MUTE				0x10
#define MASK_VI_BLACK				0x01

/* *** Info Frame *** */
#define PK_INT_MODE				0x8709
#define MASK_ISRC2_INT_MODE			0x80
#define MASK_ISRC_INT_MODE			0x40
#define MASK_ACP_INT_MODE			0x20
#define MASK_VS_INT_MODE			0x10
#define MASK_SPD_INT_MODE			0x08
#define MASK_MS_INT_MODE			0x04
#define MASK_AUD_INT_MODE			0x02
#define MASK_AVI_INT_MODE			0x01

#define NO_PKT_LIMIT				0x870B
#define MASK_NO_ACP_LIMIT			0xF0
#define SET_NO_ACP_LIMIT_MS(milliseconds)	((((milliseconds)/80) << 4) & \
						  MASK_NO_ACP_LIMIT)

#define NO_PKT_CLR				0x870C
#define MASK_NO_VS_CLR				0x40
#define MASK_NO_SPD_CLR				0x20
#define MASK_NO_ACP_CLR				0x10
#define MASK_NO_AVI_CLR1			0x02
#define MASK_NO_AVI_CLR0			0x01

#define ERR_PK_LIMIT				0x870D
#define NO_PKT_LIMIT2				0x870E
#define PK_AVI_0HEAD				0x8710
#define PK_AVI_1HEAD				0x8711
#define PK_AVI_2HEAD				0x8712
#define PK_AVI_0BYTE				0x8713
#define PK_AVI_1BYTE				0x8714
#define PK_AVI_2BYTE				0x8715
#define PK_AVI_3BYTE				0x8716
#define PK_AVI_4BYTE				0x8717
#define PK_AVI_5BYTE				0x8718
#define PK_AVI_6BYTE				0x8719
#define PK_AVI_7BYTE				0x871A
#define PK_AVI_8BYTE				0x871B
#define PK_AVI_9BYTE				0x871C
#define PK_AVI_10BYTE				0x871D
#define PK_AVI_11BYTE				0x871E
#define PK_AVI_12BYTE				0x871F
#define PK_AVI_13BYTE				0x8720
#define PK_AVI_14BYTE				0x8721
#define PK_AVI_15BYTE				0x8722
#define PK_AVI_16BYTE				0x8723

#define NO_GDB_LIMIT				0x9007

/* *** Color Bar (16 bit) *** */
#define CB_CTL					0x7000
#define CB_HSW					0x7008
#define CB_VSW					0x700A
#define CB_HTOTOAL				0x700C
#define CB_VTOTOAL				0x700E
#define CB_HACT					0x7010
#define CB_VACT					0x7012
#define CB_HSTART				0x7014
#define CB_VSTART				0x7016

enum {
	TC358840_MODE_3840X2160,
	TC358840_MODE_1920X1080,
	TC358840_MODE_1280X720,
};

static const int tc358840_30fps[] = {
	30,
};

static const int tc358840_30_60fps[] = {
	30,
	60,
};

static const struct camera_common_frmfmt tc358840_frmfmt[] = {
	{{3840, 2160},	tc358840_30fps, 1, 1, TC358840_MODE_3840X2160},
	{{1920, 1080},	tc358840_30_60fps, 2, 1, TC358840_MODE_1920X1080},
	{{1280, 720},	tc358840_30_60fps, 2, 1, TC358840_MODE_1280X720},
};
#endif  /* __TC358840_TABLES__ */
