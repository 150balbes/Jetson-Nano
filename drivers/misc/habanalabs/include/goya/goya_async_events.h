/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2018 HabanaLabs, Ltd.
 * All Rights Reserved.
 *
 */

#ifndef __GOYA_ASYNC_EVENTS_H_
#define __GOYA_ASYNC_EVENTS_H_

enum goya_async_event_id {
	GOYA_ASYNC_EVENT_ID_PCIE_IF = 33,
	GOYA_ASYNC_EVENT_ID_TPC0_ECC = 36,
	GOYA_ASYNC_EVENT_ID_TPC1_ECC = 39,
	GOYA_ASYNC_EVENT_ID_TPC2_ECC = 42,
	GOYA_ASYNC_EVENT_ID_TPC3_ECC = 45,
	GOYA_ASYNC_EVENT_ID_TPC4_ECC = 48,
	GOYA_ASYNC_EVENT_ID_TPC5_ECC = 51,
	GOYA_ASYNC_EVENT_ID_TPC6_ECC = 54,
	GOYA_ASYNC_EVENT_ID_TPC7_ECC = 57,
	GOYA_ASYNC_EVENT_ID_MME_ECC = 60,
	GOYA_ASYNC_EVENT_ID_MME_ECC_EXT = 61,
	GOYA_ASYNC_EVENT_ID_MMU_ECC = 63,
	GOYA_ASYNC_EVENT_ID_DMA_MACRO = 64,
	GOYA_ASYNC_EVENT_ID_DMA_ECC = 66,
	GOYA_ASYNC_EVENT_ID_CPU_IF_ECC = 75,
	GOYA_ASYNC_EVENT_ID_PSOC_MEM = 78,
	GOYA_ASYNC_EVENT_ID_PSOC_CORESIGHT = 79,
	GOYA_ASYNC_EVENT_ID_SRAM0 = 81,
	GOYA_ASYNC_EVENT_ID_SRAM1 = 82,
	GOYA_ASYNC_EVENT_ID_SRAM2 = 83,
	GOYA_ASYNC_EVENT_ID_SRAM3 = 84,
	GOYA_ASYNC_EVENT_ID_SRAM4 = 85,
	GOYA_ASYNC_EVENT_ID_SRAM5 = 86,
	GOYA_ASYNC_EVENT_ID_SRAM6 = 87,
	GOYA_ASYNC_EVENT_ID_SRAM7 = 88,
	GOYA_ASYNC_EVENT_ID_SRAM8 = 89,
	GOYA_ASYNC_EVENT_ID_SRAM9 = 90,
	GOYA_ASYNC_EVENT_ID_SRAM10 = 91,
	GOYA_ASYNC_EVENT_ID_SRAM11 = 92,
	GOYA_ASYNC_EVENT_ID_SRAM12 = 93,
	GOYA_ASYNC_EVENT_ID_SRAM13 = 94,
	GOYA_ASYNC_EVENT_ID_SRAM14 = 95,
	GOYA_ASYNC_EVENT_ID_SRAM15 = 96,
	GOYA_ASYNC_EVENT_ID_SRAM16 = 97,
	GOYA_ASYNC_EVENT_ID_SRAM17 = 98,
	GOYA_ASYNC_EVENT_ID_SRAM18 = 99,
	GOYA_ASYNC_EVENT_ID_SRAM19 = 100,
	GOYA_ASYNC_EVENT_ID_SRAM20 = 101,
	GOYA_ASYNC_EVENT_ID_SRAM21 = 102,
	GOYA_ASYNC_EVENT_ID_SRAM22 = 103,
	GOYA_ASYNC_EVENT_ID_SRAM23 = 104,
	GOYA_ASYNC_EVENT_ID_SRAM24 = 105,
	GOYA_ASYNC_EVENT_ID_SRAM25 = 106,
	GOYA_ASYNC_EVENT_ID_SRAM26 = 107,
	GOYA_ASYNC_EVENT_ID_SRAM27 = 108,
	GOYA_ASYNC_EVENT_ID_SRAM28 = 109,
	GOYA_ASYNC_EVENT_ID_SRAM29 = 110,
	GOYA_ASYNC_EVENT_ID_GIC500 = 112,
	GOYA_ASYNC_EVENT_ID_PCIE_DEC = 115,
	GOYA_ASYNC_EVENT_ID_TPC0_DEC = 117,
	GOYA_ASYNC_EVENT_ID_TPC1_DEC = 120,
	GOYA_ASYNC_EVENT_ID_TPC2_DEC = 123,
	GOYA_ASYNC_EVENT_ID_TPC3_DEC = 126,
	GOYA_ASYNC_EVENT_ID_TPC4_DEC = 129,
	GOYA_ASYNC_EVENT_ID_TPC5_DEC = 132,
	GOYA_ASYNC_EVENT_ID_TPC6_DEC = 135,
	GOYA_ASYNC_EVENT_ID_TPC7_DEC = 138,
	GOYA_ASYNC_EVENT_ID_AXI_ECC = 139,
	GOYA_ASYNC_EVENT_ID_L2_RAM_ECC = 140,
	GOYA_ASYNC_EVENT_ID_MME_WACS = 141,
	GOYA_ASYNC_EVENT_ID_MME_WACSD = 142,
	GOYA_ASYNC_EVENT_ID_PLL0 = 143,
	GOYA_ASYNC_EVENT_ID_PLL1 = 144,
	GOYA_ASYNC_EVENT_ID_PLL3 = 146,
	GOYA_ASYNC_EVENT_ID_PLL4 = 147,
	GOYA_ASYNC_EVENT_ID_PLL5 = 148,
	GOYA_ASYNC_EVENT_ID_PLL6 = 149,
	GOYA_ASYNC_EVENT_ID_CPU_AXI_SPLITTER = 155,
	GOYA_ASYNC_EVENT_ID_PSOC_AXI_DEC = 159,
	GOYA_ASYNC_EVENT_ID_PSOC = 160,
	GOYA_ASYNC_EVENT_ID_PCIE_FLR = 171,
	GOYA_ASYNC_EVENT_ID_PCIE_HOT_RESET = 172,
	GOYA_ASYNC_EVENT_ID_PCIE_QID0_ENG0 = 174,
	GOYA_ASYNC_EVENT_ID_PCIE_QID0_ENG1 = 175,
	GOYA_ASYNC_EVENT_ID_PCIE_QID0_ENG2 = 176,
	GOYA_ASYNC_EVENT_ID_PCIE_QID0_ENG3 = 177,
	GOYA_ASYNC_EVENT_ID_PCIE_QID1_ENG0 = 178,
	GOYA_ASYNC_EVENT_ID_PCIE_QID1_ENG1 = 179,
	GOYA_ASYNC_EVENT_ID_PCIE_QID1_ENG2 = 180,
	GOYA_ASYNC_EVENT_ID_PCIE_QID1_ENG3 = 181,
	GOYA_ASYNC_EVENT_ID_PCIE_APB = 182,
	GOYA_ASYNC_EVENT_ID_PCIE_QDB = 183,
	GOYA_ASYNC_EVENT_ID_PCIE_BM_D_P_WR = 184,
	GOYA_ASYNC_EVENT_ID_PCIE_BM_D_RD = 185,
	GOYA_ASYNC_EVENT_ID_PCIE_BM_U_P_WR = 186,
	GOYA_ASYNC_EVENT_ID_PCIE_BM_U_RD = 187,
	GOYA_ASYNC_EVENT_ID_TPC0_BMON_SPMU = 190,
	GOYA_ASYNC_EVENT_ID_TPC0_KRN_ERR = 191,
	GOYA_ASYNC_EVENT_ID_TPC1_BMON_SPMU = 200,
	GOYA_ASYNC_EVENT_ID_TPC1_KRN_ERR = 201,
	GOYA_ASYNC_EVENT_ID_TPC2_BMON_SPMU = 210,
	GOYA_ASYNC_EVENT_ID_TPC2_KRN_ERR = 211,
	GOYA_ASYNC_EVENT_ID_TPC3_BMON_SPMU = 220,
	GOYA_ASYNC_EVENT_ID_TPC3_KRN_ERR = 221,
	GOYA_ASYNC_EVENT_ID_TPC4_BMON_SPMU = 230,
	GOYA_ASYNC_EVENT_ID_TPC4_KRN_ERR = 231,
	GOYA_ASYNC_EVENT_ID_TPC5_BMON_SPMU = 240,
	GOYA_ASYNC_EVENT_ID_TPC5_KRN_ERR = 241,
	GOYA_ASYNC_EVENT_ID_TPC6_BMON_SPMU = 250,
	GOYA_ASYNC_EVENT_ID_TPC6_KRN_ERR = 251,
	GOYA_ASYNC_EVENT_ID_TPC7_BMON_SPMU = 260,
	GOYA_ASYNC_EVENT_ID_TPC7_KRN_ERR = 261,
	GOYA_ASYNC_EVENT_ID_MMU_SBA_SPMU0 = 270,
	GOYA_ASYNC_EVENT_ID_MMU_SBA_SPMU1 = 271,
	GOYA_ASYNC_EVENT_ID_MME_WACS_UP = 272,
	GOYA_ASYNC_EVENT_ID_MME_WACS_DOWN = 273,
	GOYA_ASYNC_EVENT_ID_MMU_PAGE_FAULT = 280,
	GOYA_ASYNC_EVENT_ID_MMU_WR_PERM = 281,
	GOYA_ASYNC_EVENT_ID_MMU_DBG_BM = 282,
	GOYA_ASYNC_EVENT_ID_DMA_BM_CH0 = 290,
	GOYA_ASYNC_EVENT_ID_DMA_BM_CH1 = 291,
	GOYA_ASYNC_EVENT_ID_DMA_BM_CH2 = 292,
	GOYA_ASYNC_EVENT_ID_DMA_BM_CH3 = 293,
	GOYA_ASYNC_EVENT_ID_DMA_BM_CH4 = 294,
	GOYA_ASYNC_EVENT_ID_DDR0_PHY_DFI = 300,
	GOYA_ASYNC_EVENT_ID_DDR0_ECC_SCRUB = 301,
	GOYA_ASYNC_EVENT_ID_DDR0_DB_ECC = 302,
	GOYA_ASYNC_EVENT_ID_DDR0_SB_ECC = 303,
	GOYA_ASYNC_EVENT_ID_DDR0_SB_ECC_MC = 304,
	GOYA_ASYNC_EVENT_ID_DDR0_AXI_RD = 305,
	GOYA_ASYNC_EVENT_ID_DDR0_AXI_WR = 306,
	GOYA_ASYNC_EVENT_ID_DDR1_PHY_DFI = 310,
	GOYA_ASYNC_EVENT_ID_DDR1_ECC_SCRUB = 311,
	GOYA_ASYNC_EVENT_ID_DDR1_DB_ECC = 312,
	GOYA_ASYNC_EVENT_ID_DDR1_SB_ECC = 313,
	GOYA_ASYNC_EVENT_ID_DDR1_SB_ECC_MC = 314,
	GOYA_ASYNC_EVENT_ID_DDR1_AXI_RD = 315,
	GOYA_ASYNC_EVENT_ID_DDR1_AXI_WR = 316,
	GOYA_ASYNC_EVENT_ID_CPU_BMON = 320,
	GOYA_ASYNC_EVENT_ID_TS_EAST = 322,
	GOYA_ASYNC_EVENT_ID_TS_WEST = 323,
	GOYA_ASYNC_EVENT_ID_TS_NORTH = 324,
	GOYA_ASYNC_EVENT_ID_PSOC_GPIO_U16_0 = 330,
	GOYA_ASYNC_EVENT_ID_PSOC_GPIO_U16_1 = 331,
	GOYA_ASYNC_EVENT_ID_PSOC_GPIO_U16_2 = 332,
	GOYA_ASYNC_EVENT_ID_PSOC_GPIO_05_SW_RESET = 356,
	GOYA_ASYNC_EVENT_ID_PSOC_GPIO_10_VRHOT_ICRIT = 361,
	GOYA_ASYNC_EVENT_ID_TPC0_CMDQ = 430,
	GOYA_ASYNC_EVENT_ID_TPC1_CMDQ = 431,
	GOYA_ASYNC_EVENT_ID_TPC2_CMDQ = 432,
	GOYA_ASYNC_EVENT_ID_TPC3_CMDQ = 433,
	GOYA_ASYNC_EVENT_ID_TPC4_CMDQ = 434,
	GOYA_ASYNC_EVENT_ID_TPC5_CMDQ = 435,
	GOYA_ASYNC_EVENT_ID_TPC6_CMDQ = 436,
	GOYA_ASYNC_EVENT_ID_TPC7_CMDQ = 437,
	GOYA_ASYNC_EVENT_ID_TPC0_QM = 438,
	GOYA_ASYNC_EVENT_ID_TPC1_QM = 439,
	GOYA_ASYNC_EVENT_ID_TPC2_QM = 440,
	GOYA_ASYNC_EVENT_ID_TPC3_QM = 441,
	GOYA_ASYNC_EVENT_ID_TPC4_QM = 442,
	GOYA_ASYNC_EVENT_ID_TPC5_QM = 443,
	GOYA_ASYNC_EVENT_ID_TPC6_QM = 444,
	GOYA_ASYNC_EVENT_ID_TPC7_QM = 445,
	GOYA_ASYNC_EVENT_ID_MME_QM = 447,
	GOYA_ASYNC_EVENT_ID_MME_CMDQ = 448,
	GOYA_ASYNC_EVENT_ID_DMA0_QM = 449,
	GOYA_ASYNC_EVENT_ID_DMA1_QM = 450,
	GOYA_ASYNC_EVENT_ID_DMA2_QM = 451,
	GOYA_ASYNC_EVENT_ID_DMA3_QM = 452,
	GOYA_ASYNC_EVENT_ID_DMA4_QM = 453,
	GOYA_ASYNC_EVENT_ID_DMA_ON_HBW = 454,
	GOYA_ASYNC_EVENT_ID_DMA0_CH = 455,
	GOYA_ASYNC_EVENT_ID_DMA1_CH = 456,
	GOYA_ASYNC_EVENT_ID_DMA2_CH = 457,
	GOYA_ASYNC_EVENT_ID_DMA3_CH = 458,
	GOYA_ASYNC_EVENT_ID_DMA4_CH = 459,
	GOYA_ASYNC_EVENT_ID_PI_UPDATE = 484,
	GOYA_ASYNC_EVENT_ID_HALT_MACHINE = 485,
	GOYA_ASYNC_EVENT_ID_INTS_REGISTER = 486,
	GOYA_ASYNC_EVENT_ID_SOFT_RESET = 487,
	GOYA_ASYNC_EVENT_ID_LAST_VALID_ID = 1023,
	GOYA_ASYNC_EVENT_ID_SIZE
};

#endif /* __GOYA_ASYNC_EVENTS_H_ */
