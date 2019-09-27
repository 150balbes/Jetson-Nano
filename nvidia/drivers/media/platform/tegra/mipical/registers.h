/*
 * registers.h
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * These register addresses are offsets from their respective bases,
 * csi_base, and dsi_base. These will be defined in the soc data structs.
 *
 * These offsets are based on t186 mipical register addresses. Other chips
 * will be handled by the ADDR(x) macro
 *
 * For t186,
 * csi_base = MIPI_CAL_MIPI_CAL_MODE_0 = 0x0
 * dsi_base MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0 = 0x3c
 *
 * for t21x, MIPI_CAL_MIPI_CAL_MODE_0 does not exist so csi_base will be -4
 * for t19x, MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0 was shifted so dsi_base = 0x44
 *
 * MIPI_CAL_MIPI_BIAS_PAD_CFG* registers are part of the DSI base
 *
 * All DSI base offsets will have a 0x100 flag applied to them so the ADDR(x)
 * macro can differentiate between CSI and DSI base registers.
 * See the ADDR macro for implementation.
 */

#define DSI_REG_MASK	0x100
#define REG_OFFSET_MASK	0x0FF

#define REG_SIZE 0x4

#define T186_CSI_BASE 0x0
#define T186_DSI_BASE 0x3c

// MIPI_CAL_MODE is not defined for t21x
#define MIPI_CAL_MODE		0x0
#define SEL_DPHY_CPHY		(1 << 0)

#define MIPI_CAL_CTRL		0x04
#define		NOISE_FLT	(0xf << 26)
#define		PRESCALE	(0x3 << 24)
#define		CLKEN_OVR	(1 << 4)
#define		AUTOCAL_EN	(1 << 1)
#define		STARTCAL	(1 << 0)
#define MIPI_CAL_AUTOCAL_CTRL0	0x08
#define CIL_MIPI_CAL_STATUS	0x0c
#define		CAL_DONE_DSID   (1 << 31)
#define		CAL_DONE_DSIC	(1 << 30)
#define		CAL_DONE_DSIB   (1 << 29)
#define		CAL_DONE_DSIA	(1 << 28)
#define		CAL_DONE_CSIH   (1 << 27)
#define		CAL_DONE_CSIG   (1 << 26)
#define		CAL_DONE_CSIF   (1 << 25)
#define		CAL_DONE_CSIE   (1 << 24)
#define		CAL_DONE_CSID   (1 << 23)
#define		CAL_DONE_CSIC   (1 << 22)
#define		CAL_DONE_CSIB   (1 << 21)
#define		CAL_DONE_CSIA   (1 << 20)
#define		CAL_DONE	(1 << 16)
#define		CAL_ACTIVE	(1 << 0)
#define CIL_MIPI_CAL_STATUS_2	0x10
#define CILA_MIPI_CAL_CONFIG	0x18
#define		OVERIDEA	(1 << 30)
#define		OVERIDEA_SHIFT	30
#define		SELA		(1 << 21)
#define		TERMOSA_CLK	(0x1f << 11)
#define		TERMOSA_CLK_SHIFT	11
#define		TERMOSA		0x1f
#define		TERMOSA_SHIFT	0
#define CILB_MIPI_CAL_CONFIG	0x1c
#define CILC_MIPI_CAL_CONFIG	0x20
#define CILD_MIPI_CAL_CONFIG	0x24
#define CILE_MIPI_CAL_CONFIG	0x28
#define CILF_MIPI_CAL_CONFIG	0x2c

// CILG/H_MIPI_CAL_CONFIG only defined for t194
#define CILG_MIPI_CAL_CONFIG	0x30
#define CILH_MIPI_CAL_CONFIG	0x34

// 0x3c is the dsi base in t186
#define DSIA_MIPI_CAL_CONFIG	(DSI_REG_MASK | (0x3c - T186_DSI_BASE))
#define		OVERIDEDSIA	(1 << 30)
#define		OVERIDEDSIA_SHIFT 30
#define		SELDSIA		(1 << 21)
#define		HSPDOSDSIA	(0x1f << 16)
#define		HSPDOSDSIA_SHIFT 16
#define         HSPUOSDSIA	(0x1f << 8)
#define		HSPUOSDSIA_SHIFT 8
#define		TERMOSDSIA	0x1f
#define		TERMOSDSIA_SHIFT 0
#define DSIB_MIPI_CAL_CONFIG	(DSI_REG_MASK | (0x40 - T186_DSI_BASE))
#define DSIC_MIPI_CAL_CONFIG	(DSI_REG_MASK | (0x44 - T186_DSI_BASE))
#define DSID_MIPI_CAL_CONFIG	(DSI_REG_MASK | (0x48 - T186_DSI_BASE))
#define MIPI_BIAS_PAD_CFG0	(DSI_REG_MASK | (0x5c - T186_DSI_BASE))
#define		E_VCLAMP_REF	(1 << 0)
#define		E_VCLAMP_REF_SHIFT	0
#define		PDVCLAMP	(1 << 1)
#define		PDVCLAMP_SHIFT	1
#define MIPI_BIAS_PAD_CFG1	(DSI_REG_MASK | (0x60 - T186_DSI_BASE))
#define		PAD_DRIV_UP_REF (0x7 << 8)
#define		PAD_DRIV_UP_REF_SHIFT 8
#define		PAD_DRIV_DN_REF (0x7 << 16)
#define		PAD_DRIV_DN_REF_SHIFT 16
#define MIPI_BIAS_PAD_CFG2	(DSI_REG_MASK | (0x64 - T186_DSI_BASE))
#define		PDVREG		(1 << 1)
#define		PDVREG_SHIFT	1
#define		PAD_VAUXP_LEVEL (0x7 << 4)
#define		PAD_VAUXP_LEVEL_SHIFT 4
#define		PAD_VCLAMP_LEVEL (0x7 << 16)
#define		PAD_VCLAMP_LEVEL_SHIFT 16
#define DSIA_MIPI_CAL_CONFIG_2	(DSI_REG_MASK | (0x68 - T186_DSI_BASE))
#define		CLKOVERIDEDSIA	(1 << 30)
#define		CLKOVERIDEDSIA_SHIFT 30
#define		CLKSELDSIA	(1 << 21)
#define		HSCLKTERMOSDSIA	(0x1f << 16)
#define		HSCLKTERMOSDSIA_SHIFT	16
#define		HSCLKPDOSDSIA	(0x1f << 8)
#define		HSCLKPDOSDSIA_SHIFT 8
#define		HSCLKPUOSDSIA	0x1f
#define		HSCLKPUOSDSIA_SHIFT 0
#define DSIB_MIPI_CAL_CONFIG_2	(DSI_REG_MASK | (0x6c - T186_DSI_BASE))
// note the weird jump in register address here
#define DSIC_MIPI_CAL_CONFIG_2	(DSI_REG_MASK | (0x74 - T186_DSI_BASE))
#define DSID_MIPI_CAL_CONFIG_2	(DSI_REG_MASK | (0x78 - T186_DSI_BASE))
