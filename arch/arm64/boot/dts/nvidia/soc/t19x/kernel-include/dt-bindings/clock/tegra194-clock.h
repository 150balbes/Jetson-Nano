/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef BPMP_ABI_MACH_T194_CLOCK_H
#define BPMP_ABI_MACH_T194_CLOCK_H

/**
 * @file
 * @defgroup bpmp_clock_ids Clock ID's
 * @{
 */
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_ACTMON */
#define TEGRA194_CLK_ACTMON			1U
/** @brief output of gate CLK_ENB_ADSP */
#define TEGRA194_CLK_ADSP			2U
/** @brief output of gate CLK_ENB_ADSPNEON */
#define TEGRA194_CLK_ADSPNEON			3U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AHUB */
#define TEGRA194_CLK_AHUB			4U
/** @brief output of gate CLK_ENB_APB2APE */
#define TEGRA194_CLK_APB2APE			5U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_APE */
#define TEGRA194_CLK_APE			6U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AUD_MCLK */
#define TEGRA194_CLK_AUD_MCLK			7U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_AXI_CBB */
#define TEGRA194_CLK_AXI_CBB			8U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_CAN1 */
#define TEGRA194_CLK_CAN1			9U
/** @brief output of gate CLK_ENB_CAN1_HOST */
#define TEGRA194_CLK_CAN1_HOST			10U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_CAN2 */
#define TEGRA194_CLK_CAN2			11U
/** @brief output of gate CLK_ENB_CAN2_HOST */
#define TEGRA194_CLK_CAN2_HOST			12U
/** @brief output of gate CLK_ENB_CEC */
#define TEGRA194_CLK_CEC			13U
/** @brief output of divider CLK_RST_CONTROLLER_CLK_M_DIVIDE */
#define TEGRA194_CLK_CLK_M			14U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC1 */
#define TEGRA194_CLK_DMIC1			15U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC2 */
#define TEGRA194_CLK_DMIC2			16U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC3 */
#define TEGRA194_CLK_DMIC3			17U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC4 */
#define TEGRA194_CLK_DMIC4			18U
/** @brief output of gate CLK_ENB_DPAUX */
#define TEGRA194_CLK_DPAUX			19U
/** @brief output of gate CLK_ENB_DPAUX1 */
#define TEGRA194_CLK_DPAUX1			20U
/**
 * @brief output of mux controlled by CLK_RST_CONTROLLER_ACLK_BURST_POLICY
 * divided by the divider controlled by ACLK_CLK_DIVISOR in
 * CLK_RST_CONTROLLER_SUPER_ACLK_DIVIDER
 */
#define TEGRA194_CLK_ACLK			21U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_MSS_ENCRYPT switch divider output */
#define TEGRA194_CLK_MSS_ENCRYPT		22U
/** @brief clock recovered from EAVB input */
#define TEGRA194_CLK_EQOS_RX_INPUT		23U
/** @brief Output of gate CLK_ENB_IQC2 */
#define TEGRA194_CLK_IQC2			24U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_AON_APB switch divider output */
#define TEGRA194_CLK_AON_APB			25U
/** @brief CLK_RST_CONTROLLER_AON_NIC_RATE divider output */
#define TEGRA194_CLK_AON_NIC			26U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_AON_CPU_NIC switch divider output */
#define TEGRA194_CLK_AON_CPU_NIC		27U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLA1_BASE for use by audio clocks */
#define TEGRA194_CLK_PLLA1			28U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DSPK1 */
#define TEGRA194_CLK_DSPK1			29U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DSPK2 */
#define TEGRA194_CLK_DSPK2			30U
/**
 * @brief controls the EMC clock frequency.
 * @details Doing a clk_set_rate on this clock will select the
 * appropriate clock source, program the source rate and execute a
 * specific sequence to switch to the new clock source for both memory
 * controllers. This can be used to control the balance between memory
 * throughput and memory controller power.
 */
#define TEGRA194_CLK_EMC			31U
/** @brief output of gate CLK_ENB_EQOS */
#define TEGRA194_CLK_EQOS_AXI			32U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_EQOS_PTP_REF_CLK_0 */
#define TEGRA194_CLK_EQOS_PTP_REF		33U
/** @brief output of gate CLK_ENB_EQOS_RX */
#define TEGRA194_CLK_EQOS_RX			34U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_EQOS_TX_CLK */
#define TEGRA194_CLK_EQOS_TX			35U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH1 */
#define TEGRA194_CLK_EXTPERIPH1			36U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH2 */
#define TEGRA194_CLK_EXTPERIPH2			37U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH3 */
#define TEGRA194_CLK_EXTPERIPH3			38U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_EXTPERIPH4 */
#define TEGRA194_CLK_EXTPERIPH4			39U
/** @brief output of gate CLK_ENB_FUSE */
#define TEGRA194_CLK_FUSE			40U
/** @brief GPC2CLK-div-2 */
#define TEGRA194_CLK_GPCCLK			41U
/** @brief TODO */
#define TEGRA194_CLK_GPU_PWR			42U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HDA2CODEC_2X */
#define TEGRA194_CLK_HDA			43U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HDA2CODEC_2X */
#define TEGRA194_CLK_HDA2CODEC_2X		44U
/** @brief output of gate CLK_ENB_HDA2HDMICODEC */
#define TEGRA194_CLK_HDA2HDMICODEC		45U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_HOST1X */
#define TEGRA194_CLK_HOST1X			46U
/** @brief Obselete - maintained for ABI compatibility */
#define TEGRA194_CLK_HSIC_TRK			47U
/** @clkdesc{i2c_clks, out, mux, CLK_RST_CONTROLLER_CLK_SOURCE_I2C1} */
#define TEGRA194_CLK_I2C1			48U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C2 */
#define TEGRA194_CLK_I2C2			49U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C3 */
#define TEGRA194_CLK_I2C3			50U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C4 */
#define TEGRA194_CLK_I2C4			51U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C6 */
#define TEGRA194_CLK_I2C6			52U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C7 */
#define TEGRA194_CLK_I2C7			53U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C8 */
#define TEGRA194_CLK_I2C8			54U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C9 */
#define TEGRA194_CLK_I2C9			55U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S1 */
#define TEGRA194_CLK_I2S1			56U
/** @brief clock recovered from I2S1 input */
#define TEGRA194_CLK_I2S1_SYNC_INPUT		57U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S2 */
#define TEGRA194_CLK_I2S2			58U
/** @brief clock recovered from I2S2 input */
#define TEGRA194_CLK_I2S2_SYNC_INPUT		59U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S3 */
#define TEGRA194_CLK_I2S3			60U
/** @brief clock recovered from I2S3 input */
#define TEGRA194_CLK_I2S3_SYNC_INPUT		61U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S4 */
#define TEGRA194_CLK_I2S4			62U
/** @brief clock recovered from I2S4 input */
#define TEGRA194_CLK_I2S4_SYNC_INPUT		63U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2S5 */
#define TEGRA194_CLK_I2S5			64U
/** @brief clock recovered from I2S5 input */
#define TEGRA194_CLK_I2S5_SYNC_INPUT		65U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C6 */
#define TEGRA194_CLK_I2S6			66U
/** @brief clock recovered from I2S6 input */
#define TEGRA194_CLK_I2S6_SYNC_INPUT		67U
/** @brief output of gate CLK_ENB_IQC1 */
#define TEGRA194_CLK_IQC1			68U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_ISP */
#define TEGRA194_CLK_ISP			69U
/**
 * @brief A fake clock which must be enabled during
 * KFUSE read operations to ensure adequate VDD_CORE voltage
 */
#define TEGRA194_CLK_KFUSE			70U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_MAUD */
#define TEGRA194_CLK_MAUD			71U
/** @brief output of gate CLK_ENB_MIPI_CAL */
#define TEGRA194_CLK_MIPI_CAL			72U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_CORE_PLL_FIXED */
#define TEGRA194_CLK_MPHY_CORE_PLL_FIXED	73U
/** @brief output of gate CLK_ENB_MPHY_L0_RX_ANA */
#define TEGRA194_CLK_MPHY_L0_RX_ANA		74U
/** @brief output of gate CLK_ENB_MPHY_L0_RX_LS_BIT */
#define TEGRA194_CLK_MPHY_L0_RX_LS_BIT		75U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_L0_RX_LS_SYMB */
#define TEGRA194_CLK_MPHY_L0_RX_SYMB		76U
/** @brief output of gate CLK_ENB_MPHY_L0_TX_LS_3XBIT */
#define TEGRA194_CLK_MPHY_L0_TX_LS_3XBIT	77U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_L0_TX_LS_SYMB */
#define TEGRA194_CLK_MPHY_L0_TX_SYMB		78U
/** @brief output of gate CLK_ENB_MPHY_L1_RX_ANA */
#define TEGRA194_CLK_MPHY_L1_RX_ANA		79U
/** @brief output of the divider CLK_RST_CONTROLLER_CLK_SOURCE_MPHY_TX_1MHZ_REF */
#define TEGRA194_CLK_MPHY_TX_1MHZ_REF		80U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVCSI */
#define TEGRA194_CLK_NVCSI			81U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVCSILP */
#define TEGRA194_CLK_NVCSILP			82U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVDEC */
#define TEGRA194_CLK_NVDEC			83U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAYHUB switch divider output */
#define TEGRA194_CLK_NVDISPLAYHUB		84U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_DISP switch divider output */
#define TEGRA194_CLK_NVDISPLAY_DISP		85U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P0 switch divider output */
#define TEGRA194_CLK_NVDISPLAY_P0		86U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P1 switch divider output */
#define TEGRA194_CLK_NVDISPLAY_P1		87U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P2 switch divider output */
#define TEGRA194_CLK_NVDISPLAY_P2		88U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVENC */
#define TEGRA194_CLK_NVENC			89U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_NVJPG */
#define TEGRA194_CLK_NVJPG			90U
/** @brief input from Tegra's XTAL_IN */
#define TEGRA194_CLK_OSC			91U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_AON_TOUCH switch divider output */
#define TEGRA194_CLK_AON_TOUCH			92U
/** PLL controlled by CLK_RST_CONTROLLER_PLLA_BASE for use by audio clocks */
#define TEGRA194_CLK_PLLA			93U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLAON_BASE for use by IP blocks in the AON domain */
#define TEGRA194_CLK_PLLAON			94U
/** @brief PLLD */
#define TEGRA194_CLK_PLLD			95U
/** @brief PLLD2 */
#define TEGRA194_CLK_PLLD2			96U
/** @brief PLLD3 */
#define TEGRA194_CLK_PLLD3			97U
/** @brief PLLDP */
#define TEGRA194_CLK_PLLDP			98U
/** @brief PLLD4 */
#define TEGRA194_CLK_PLLD4			99U
/** Fixed 100MHz PLL for PCIe, SATA and superspeed USB */
#define TEGRA194_CLK_PLLE			100U
/** @brief PLLP */
#define TEGRA194_CLK_PLLP			101U
/** @brief PLLP VCO output */
#define TEGRA194_CLK_PLLP_OUT0			102U
/** Fixed frequency 960MHz PLL for USB and EAVB */
#define TEGRA194_CLK_UTMIPLL			103U
/** @brief output of the divider CLK_RST_CONTROLLER_PLLA_OUT */
#define TEGRA194_CLK_PLLA_OUT0			104U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM1 */
#define TEGRA194_CLK_PWM1			105U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM2 */
#define TEGRA194_CLK_PWM2			106U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM3 */
#define TEGRA194_CLK_PWM3			107U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM4 */
#define TEGRA194_CLK_PWM4			108U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM5 */
#define TEGRA194_CLK_PWM5			109U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM6 */
#define TEGRA194_CLK_PWM6			110U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM7 */
#define TEGRA194_CLK_PWM7			111U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_PWM8 */
#define TEGRA194_CLK_PWM8			112U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_RCE_CPU_NIC output */
#define TEGRA194_CLK_RCE_CPU_NIC		113U
/** @brief CLK_RST_CONTROLLER_RCE_NIC_RATE divider output */
#define TEGRA194_CLK_RCE_NIC			114U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SATA */
#define TEGRA194_CLK_SATA			115U
/** @brief output of gate CLK_ENB_SATA_OOB */
#define TEGRA194_CLK_SATA_OOB			116U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_AON_I2C_SLOW switch divider output */
#define TEGRA194_CLK_AON_I2C_SLOW		117U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SCE_CPU_NIC */
#define TEGRA194_CLK_SCE_CPU_NIC		118U
/** @brief output of divider CLK_RST_CONTROLLER_SCE_NIC_RATE */
#define TEGRA194_CLK_SCE_NIC			119U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC1 */
#define TEGRA194_CLK_SDMMC1			120U
/** @brief Logical clk for setting the UPHY PLL3 rate */
#define TEGRA194_CLK_UPHY_PLL3			121U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC3 */
#define TEGRA194_CLK_SDMMC3			122U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC4 */
#define TEGRA194_CLK_SDMMC4			123U
/** @brief gated version of SE_FREE clk */
#define TEGRA194_CLK_SE				124U
/** @brief output of mux controlled by SOR0_CLK_SEL1 and SOR0_CLK_SEL0 */
#define TEGRA194_CLK_SOR0_OUT			125U
/** @brief output of mux controlled by SOR0_CLK_SRC */
#define TEGRA194_CLK_SOR0_REF			126U
/** @brief SOR0 brick output which feeds into SOR0_CLK_SEL0 mux */
#define TEGRA194_CLK_SOR0_PAD_CLKOUT		127U
/** @brief output of mux controlled by SOR1_CLK_SEL1 and SOR1_CLK_SEL0 */
#define TEGRA194_CLK_SOR1_OUT			128U
/** @brief output of mux controlled by SOR1_CLK_SRC */
#define TEGRA194_CLK_SOR1_REF			129U
/** @brief SOR1 brick output which feeds into SOR1_CLK_SEL0 mux */
#define TEGRA194_CLK_SOR1_PAD_CLKOUT		130U
/** @brief output of gate CLK_ENB_SOR_SAFE */
#define TEGRA194_CLK_SOR_SAFE			131U
/** @brief Interface clock from IQC pad (1) */
#define TEGRA194_CLK_IQC1_IN			132U
/** @brief Interface clock from IQC pad (2) */
#define TEGRA194_CLK_IQC2_IN			133U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_DMIC5 */
#define TEGRA194_CLK_DMIC5			134U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SPI1 */
#define TEGRA194_CLK_SPI1			135U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_SPI2 */
#define TEGRA194_CLK_SPI2			136U
/**  @clkdesc{spi_clks, out, mux, CLK_RST_CONTROLLER_CLK_SOURCE_SPI3} */
#define TEGRA194_CLK_SPI3			137U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C_SLOW */
#define TEGRA194_CLK_I2C_SLOW			138U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC1 */
#define TEGRA194_CLK_SYNC_DMIC1			139U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC2 */
#define TEGRA194_CLK_SYNC_DMIC2			140U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC3 */
#define TEGRA194_CLK_SYNC_DMIC3			141U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DMIC4 */
#define TEGRA194_CLK_SYNC_DMIC4			142U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DSPK1 */
#define TEGRA194_CLK_SYNC_DSPK1			143U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_DSPK2 */
#define TEGRA194_CLK_SYNC_DSPK2			144U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S1 */
#define TEGRA194_CLK_SYNC_I2S1			145U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S2 */
#define TEGRA194_CLK_SYNC_I2S2			146U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S3 */
#define TEGRA194_CLK_SYNC_I2S3			147U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S4 */
#define TEGRA194_CLK_SYNC_I2S4			148U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S5 */
#define TEGRA194_CLK_SYNC_I2S5			149U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_AUDIO_SYNC_CLK_I2S6 */
#define TEGRA194_CLK_SYNC_I2S6			150U
/** @brief controls MPHY_FORCE_LS_MODE upon enable & disable */
#define TEGRA194_CLK_MPHY_FORCE_LS_MODE		151U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TACH */
#define TEGRA194_CLK_TACH			152U
/** output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TSEC */
#define TEGRA194_CLK_TSEC			153U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_TSECB */
#define TEGRA194_CLK_TSECB			154U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTA */
#define TEGRA194_CLK_UARTA			155U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTB */
#define TEGRA194_CLK_UARTB			156U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTC */
#define TEGRA194_CLK_UARTC			157U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTD */
#define TEGRA194_CLK_UARTD			158U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTE */
#define TEGRA194_CLK_UARTE			159U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTF */
#define TEGRA194_CLK_UARTF			160U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UARTG */
#define TEGRA194_CLK_UARTG			161U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UART_FST_MIPI_CAL */
#define TEGRA194_CLK_UART_FST_MIPI_CAL		162U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UFSDEV_REF */
#define TEGRA194_CLK_UFSDEV_REF			163U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_UFSHC_CG_SYS */
#define TEGRA194_CLK_UFSHC			164U
/** @brief output of gate CLK_ENB_USB2_TRK */
#define TEGRA194_CLK_USB2_TRK			165U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_VI */
#define TEGRA194_CLK_VI				166U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_VIC */
#define TEGRA194_CLK_VIC			167U
/** @brief pva0_axi */
#define TEGRA194_CLK_PVA0_AXI			168U
/** @brief PVA0_vps0_clk */
#define TEGRA194_CLK_PVA0_VPS0			169U
/** @brief PVA0_vps1_clk */
#define TEGRA194_CLK_PVA0_VPS1			170U
/** @brief pva1_axi clk */
#define TEGRA194_CLK_PVA1_AXI			171U
/** @brief PVA1_vps0_clk */
#define TEGRA194_CLK_PVA1_VPS0			172U
/** @brief PVA1_vps1_clk */
#define TEGRA194_CLK_PVA1_VPS1			173U
/** @brief DLA0_falcon_clk */
#define TEGRA194_CLK_DLA0_FALCON		174U
/** @brief DLA0_core_clk */
#define TEGRA194_CLK_DLA0_CORE			175U
/** @brief DLA1_falcon_clk */
#define TEGRA194_CLK_DLA1_FALCON		176U
/** @brief DLA1_core_clk */
#define TEGRA194_CLK_DLA1_CORE			177U
/** @brief output of mux controlled by SOR2_CLK_SEL1 and SOR2_CLK_SEL0 */
#define TEGRA194_CLK_SOR2_OUT			178U
/** @brief output of mux controlled by SOR2_CLK_SRC */
#define TEGRA194_CLK_SOR2_REF			179U
/** @brief SOR2 brick output which feeds into SOR2_CLK_SEL0 mux */
#define TEGRA194_CLK_SOR2_PAD_CLKOUT		180U
/** @brief output of mux controlled by SOR3_CLK_SEL1 and SOR3_CLK_SEL0 */
#define TEGRA194_CLK_SOR3_OUT			181U
/** @brief output of mux controlled by SOR3_CLK_SRC */
#define TEGRA194_CLK_SOR3_REF			182U
/** @brief SOR3 brick output which feeds into SOR3_CLK_SEL0 mux */
#define TEGRA194_CLK_SOR3_PAD_CLKOUT		183U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDISPLAY_P3 switch divider output */
#define TEGRA194_CLK_NVDISPLAY_P3		184U
/** @brief output of gate CLK_ENB_DPAUX2 */
#define TEGRA194_CLK_DPAUX2			185U
/** @brief output of gate CLK_ENB_DPAUX3 */
#define TEGRA194_CLK_DPAUX3			186U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVDEC1 switch divider output */
#define TEGRA194_CLK_NVDEC1			187U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVENC1 switch divider output */
#define TEGRA194_CLK_NVENC1			188U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_SE switch divider output */
#define TEGRA194_CLK_SE_FREE			189U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_UARTH switch divider output */
#define TEGRA194_CLK_UARTH			190U
/** @brief ungated version of fuse clk */
#define TEGRA194_CLK_FUSE_SERIAL		191U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_QSPI0 switch divider output */
#define TEGRA194_CLK_QSPI0			192U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_QSPI1 switch divider output */
#define TEGRA194_CLK_QSPI1			193U
/** @brief output of the divider QSPI_CLK_DIV2_SEL in CLK_RST_CONTROLLER_CLK_SOURCE_QSPI0 */
#define TEGRA194_CLK_QSPI0_PM			194U
/** @brief output of the divider QSPI_CLK_DIV2_SEL in CLK_RST_CONTROLLER_CLK_SOURCE_QSPI1 */
#define TEGRA194_CLK_QSPI1_PM			195U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_VI_CONST switch divider output */
#define TEGRA194_CLK_VI_CONST			196U
/** @brief NAFLL clock source for BPMP */
#define TEGRA194_CLK_NAFLL_BPMP			197U
/** @brief NAFLL clock source for SCE */
#define TEGRA194_CLK_NAFLL_SCE			198U
/** @brief NAFLL clock source for NVDEC */
#define TEGRA194_CLK_NAFLL_NVDEC		199U
/** @brief NAFLL clock source for NVJPG */
#define TEGRA194_CLK_NAFLL_NVJPG		200U
/** @brief NAFLL clock source for TSEC */
#define TEGRA194_CLK_NAFLL_TSEC			201U
/** @brief NAFLL clock source for TSECB */
#define TEGRA194_CLK_NAFLL_TSECB		202U
/** @brief NAFLL clock source for VI */
#define TEGRA194_CLK_NAFLL_VI			203U
/** @brief NAFLL clock source for SE */
#define TEGRA194_CLK_NAFLL_SE			204U
/** @brief NAFLL clock source for NVENC */
#define TEGRA194_CLK_NAFLL_NVENC		205U
/** @brief NAFLL clock source for ISP */
#define TEGRA194_CLK_NAFLL_ISP			206U
/** @brief NAFLL clock source for VIC */
#define TEGRA194_CLK_NAFLL_VIC			207U
/** @brief NAFLL clock source for NVDISPLAYHUB */
#define TEGRA194_CLK_NAFLL_NVDISPLAYHUB		208U
/** @brief NAFLL clock source for AXICBB */
#define TEGRA194_CLK_NAFLL_AXICBB		209U
/** @brief NAFLL clock source for DLA */
#define TEGRA194_CLK_NAFLL_DLA			210U
/** @brief NAFLL clock source for PVA_CORE */
#define TEGRA194_CLK_NAFLL_PVA_CORE		211U
/** @brief NAFLL clock source for PVA_VPS */
#define TEGRA194_CLK_NAFLL_PVA_VPS		212U
/** @brief NAFLL clock source for CVNAS */
#define TEGRA194_CLK_NAFLL_CVNAS		213U
/** @brief NAFLL clock source for RCE */
#define TEGRA194_CLK_NAFLL_RCE			214U
/** @brief NAFLL clock source for NVENC1 */
#define TEGRA194_CLK_NAFLL_NVENC1		215U
/** @brief NAFLL clock source for DLA_FALCON */
#define TEGRA194_CLK_NAFLL_DLA_FALCON		216U
/** @brief NAFLL clock source for NVDEC1 */
#define TEGRA194_CLK_NAFLL_NVDEC1		217U
/** @brief NAFLL clock source for GPU */
#define TEGRA194_CLK_NAFLL_GPU			218U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_SDMMC_LEGACY_TM switch divider output */
#define TEGRA194_CLK_SDMMC_LEGACY_TM		219U
/** @brief output of gate CLK_ENB_PEX0_CORE_0 */
#define TEGRA194_CLK_PEX0_CORE_0		220U
/** @brief output of gate CLK_ENB_PEX0_CORE_1 */
#define TEGRA194_CLK_PEX0_CORE_1		221U
/** @brief output of gate CLK_ENB_PEX0_CORE_2 */
#define TEGRA194_CLK_PEX0_CORE_2		222U
/** @brief output of gate CLK_ENB_PEX0_CORE_3 */
#define TEGRA194_CLK_PEX0_CORE_3		223U
/** @brief output of gate CLK_ENB_PEX0_CORE_4 */
#define TEGRA194_CLK_PEX0_CORE_4		224U
/** @brief output of gate CLK_ENB_PEX1_CORE_5 */
#define TEGRA194_CLK_PEX1_CORE_5		225U
/** @brief PCIE endpoint mode, HSIO UPHY PLL1 */
#define TEGRA194_CLK_PEX_REF1			226U
/** @brief PCIE endpoint mode, HSIO UPHY PLL2 */
#define TEGRA194_CLK_PEX_REF2			227U
/** @brief NVHS UPHY reference clock input */
#define TEGRA194_CLK_NVHS_REF			228U
/** @brief NVCSI_CIL clock for partition A */
#define TEGRA194_CLK_CSI_A			229U
/** @brief NVCSI_CIL clock for partition B */
#define TEGRA194_CLK_CSI_B			230U
/** @brief NVCSI_CIL clock for partition C */
#define TEGRA194_CLK_CSI_C			231U
/** @brief NVCSI_CIL clock for partition D */
#define TEGRA194_CLK_CSI_D			232U
/** @brief NVCSI_CIL clock for partition E */
#define TEGRA194_CLK_CSI_E			233U
/** @brief NVCSI_CIL clock for partition F */
#define TEGRA194_CLK_CSI_F			234U
/** @brief NVCSI_CIL clock for partition G */
#define TEGRA194_CLK_CSI_G			235U
/** @brief NVCSI_CIL clock for partition H */
#define TEGRA194_CLK_CSI_H			236U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLC4_BASE */
#define TEGRA194_CLK_PLLC4			237U
/** @brief output of gate CLK_ENB_PLLC4_OUT */
#define TEGRA194_CLK_PLLC4_OUT			238U
/** @brief PLLC4 VCO followed by DIV3 path */
#define TEGRA194_CLK_PLLC4_OUT1			239U
/** @brief PLLC4 VCO followed by DIV5 path */
#define TEGRA194_CLK_PLLC4_OUT2			240U
/** @brief output of the mux controlled by PLLC4_CLK_SEL */
#define TEGRA194_CLK_PLLC4_MUXED		241U
/** @brief PLLC4 VCO followed by DIV2 path */
#define TEGRA194_CLK_PLLC4_VCO_DIV2		242U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLNVHS_BASE */
#define TEGRA194_CLK_PLLNVHS			243U
/** @brief CSI pad brick input from partition A */
#define TEGRA194_CLK_CSI_A_PAD			244U
/** @brief CSI pad brick input from partition B */
#define TEGRA194_CLK_CSI_B_PAD			245U
/** @brief CSI pad brick input from partition C */
#define TEGRA194_CLK_CSI_C_PAD			246U
/** @brief CSI pad brick input from partition D */
#define TEGRA194_CLK_CSI_D_PAD			247U
/** @brief CSI pad brick input from partition E */
#define TEGRA194_CLK_CSI_E_PAD			248U
/** @brief CSI pad brick input from partition F */
#define TEGRA194_CLK_CSI_F_PAD			249U
/** @brief CSI pad brick input from partition G */
#define TEGRA194_CLK_CSI_G_PAD			250U
/** @brief CSI pad brick input from partition H */
#define TEGRA194_CLK_CSI_H_PAD			251U
/** @brief output of the gate CLK_ENB_SLVSEC */
#define TEGRA194_CLK_SLVSEC			252U
/** @brief output of the gate CLK_ENB_SLVSEC_PADCTRL */
#define TEGRA194_CLK_SLVSEC_PADCTRL		253U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PEX_SATA_USB_RX_BYP switch divider output */
#define TEGRA194_CLK_PEX_SATA_USB_RX_BYP	254U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL0_MGMT switch divider output */
#define TEGRA194_CLK_PEX_USB_PAD_PLL0_MGMT	255U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL1_MGMT switch divider output */
#define TEGRA194_CLK_PEX_USB_PAD_PLL1_MGMT	256U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL2_MGMT switch divider output */
#define TEGRA194_CLK_PEX_USB_PAD_PLL2_MGMT	257U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PEX_USB_PAD_PLL3_MGMT switch divider output */
#define TEGRA194_CLK_PEX_USB_PAD_PLL3_MGMT	258U
/** @brief CLK_RST_CONTROLLER_CLOCK_SOURCE_NVLINK_SYSCLK switch divider output */
#define TEGRA194_CLK_NVLINK_SYS			259U
/** @brief output ofthe gate CLK_ENB_RX_NVLINK */
#define TEGRA194_CLK_NVLINK_RX			260U
/** @brief output of the gate CLK_ENB_TX_NVLINK */
#define TEGRA194_CLK_NVLINK_TX			261U
/** @brief output of the fixed (DIV2) divider CLK_RST_CONTROLLER_NVLINK_TX_DIV_CLK_DIVISOR */
#define TEGRA194_CLK_NVLINK_TX_DIV		262U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVHS_RX_BYP switch divider output */
#define TEGRA194_CLK_NVHS_RX_BYP_REF		263U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_NVHS_PLL0_MGMT switch divider output */
#define TEGRA194_CLK_NVHS_PLL0_MGMT		264U
/** @brief xusb_core_dev_clk */
#define TEGRA194_CLK_XUSB_CORE_DEV		265U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_CORE_HOST switch divider output  */
#define TEGRA194_CLK_XUSB_CORE_MUX		266U
/** @brief xusb_core_host_clk */
#define TEGRA194_CLK_XUSB_CORE_HOST		267U
/** @brief xusb_core_superspeed_clk */
#define TEGRA194_CLK_XUSB_CORE_SS		268U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FALCON switch divider output */
#define TEGRA194_CLK_XUSB_FALCON		269U
/** @brief xusb_falcon_host_clk */
#define TEGRA194_CLK_XUSB_FALCON_HOST		270U
/** @brief xusb_falcon_superspeed_clk */
#define TEGRA194_CLK_XUSB_FALCON_SS		271U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_FS switch divider output */
#define TEGRA194_CLK_XUSB_FS			272U
/** @brief xusb_fs_host_clk */
#define TEGRA194_CLK_XUSB_FS_HOST		273U
/** @brief xusb_fs_dev_clk */
#define TEGRA194_CLK_XUSB_FS_DEV		274U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_XUSB_SS switch divider output */
#define TEGRA194_CLK_XUSB_SS			275U
/** @brief xusb_ss_dev_clk */
#define TEGRA194_CLK_XUSB_SS_DEV		276U
/** @brief xusb_ss_superspeed_clk */
#define TEGRA194_CLK_XUSB_SS_SUPERSPEED		277U
/** @brief HPLL for display hub clock */
#define TEGRA194_CLK_PLLDISPHUB			278U
/** @brief Invalid (do not use) */
#define TEGRA194_CLK_PLLDISPHUB_DIV		279U
/** @brief NAFLL clock source for CPU cluster 0 */
#define TEGRA194_CLK_NAFLL_CLUSTER0		280U
/** @brief NAFLL clock source for CPU cluster 1 */
#define TEGRA194_CLK_NAFLL_CLUSTER1		281U
/** @brief NAFLL clock source for CPU cluster 2 */
#define TEGRA194_CLK_NAFLL_CLUSTER2		282U
/** @brief NAFLL clock source for CPU cluster 3 */
#define TEGRA194_CLK_NAFLL_CLUSTER3		283U
/** @brief CLK_RST_CONTROLLER_CAN1_CORE_RATE divider output */
#define TEGRA194_CLK_CAN1_CORE			284U
/** @brief CLK_RST_CONTROLLER_CAN2_CORE_RATE divider outputt */
#define TEGRA194_CLK_CAN2_CORE			285U
/** @brief CLK_RST_CONTROLLER_PLLA1_OUT1 switch divider output */
#define TEGRA194_CLK_PLLA1_OUT1			286U
/** @brief NVHS PLL hardware power sequencer (overrides 'manual' programming of PLL) */
#define TEGRA194_CLK_PLLNVHS_HPS		287U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLREFE_BASE */
#define TEGRA194_CLK_PLLREFE_VCOOUT		288U
/** @brief 32K input clock provided by PMIC */
#define TEGRA194_CLK_CLK_32K			289U
/** @brief Clock recovered from SPDIFIN input */
#define TEGRA194_CLK_SPDIFIN_SYNC_INPUT		290U
/** @brief Fixed 48MHz clock divided down from utmipll */
#define TEGRA194_CLK_UTMIPLL_CLKOUT48		291U
/** @brief Fixed 480MHz clock divided down from utmipll */
#define TEGRA194_CLK_UTMIPLL_CLKOUT480		292U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_CVNAS switch divider output */
#define TEGRA194_CLK_CVNAS			293U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLNVCSI_BASE  */
#define TEGRA194_CLK_PLLNVCSI			294U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PVA0_CPU_AXI switch divider output */
#define TEGRA194_CLK_PVA0_CPU_AXI		295U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PVA1_CPU_AXI switch divider output */
#define TEGRA194_CLK_PVA1_CPU_AXI		296U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PVA0_VPS switch divider output */
#define TEGRA194_CLK_PVA0_VPS			297U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_PVA1_VPS switch divider output */
#define TEGRA194_CLK_PVA1_VPS			298U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_DLA0_FALCON switch divider output */
#define TEGRA194_CLK_DLA0_FALCON_MUX		299U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_DLA1_FALCON switch divider output */
#define TEGRA194_CLK_DLA1_FALCON_MUX		300U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_DLA0_CORE switch divider output */
#define TEGRA194_CLK_DLA0_CORE_MUX		301U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_DLA1_CORE switch divider output */
#define TEGRA194_CLK_DLA1_CORE_MUX		302U

/** @brief UTMI PLL HW power sequencer */
#define TEGRA194_CLK_UTMIPLL_HPS		304U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C5 */
#define TEGRA194_CLK_I2C5			305U
/** @brief output of mux controlled by CLK_RST_CONTROLLER_CLK_SOURCE_I2C10 */
#define TEGRA194_CLK_I2C10			306U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_CPU_NIC switch divider output */
#define TEGRA194_CLK_BPMP_CPU_NIC		307U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_BPMP_APB switch divider output */
#define TEGRA194_CLK_BPMP_APB			308U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_TSC switch divider output */
#define TEGRA194_CLK_TSC			309U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_EMC switch divider output */
#define TEGRA194_CLK_EMCSA			310U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_EMCSB switch divider output */
#define TEGRA194_CLK_EMCSB			311U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_EMCSC switch divider output */
#define TEGRA194_CLK_EMCSC			312U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_EMCSD switch divider output */
#define TEGRA194_CLK_EMCSD			313U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLC_BASE */
#define TEGRA194_CLK_PLLC			314U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLC2_BASE */
#define TEGRA194_CLK_PLLC2			315U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLC3_BASE */
#define TEGRA194_CLK_PLLC3			316U
/** @brief CLK_RST_CONTROLLER_TSC_HS_SUPER_CLK_DIVIDER skip divider output */
#define TEGRA194_CLK_TSC_REF			317U
/** @brief Dummy clock to ensure minimum SoC voltage for fuse burning */
#define TEGRA194_CLK_FUSE_BURN			318U
/** @brief Monitored branch of PEX0_CORE_0 clock */
#define TEGRA194_CLK_PEX0_CORE_0M		319U
/** @brief Monitored branch of PEX0_CORE_1 clock */
#define TEGRA194_CLK_PEX0_CORE_1M		320U
/** @brief Monitored branch of PEX0_CORE_2 clock */
#define TEGRA194_CLK_PEX0_CORE_2M		321U
/** @brief Monitored branch of PEX0_CORE_3 clock */
#define TEGRA194_CLK_PEX0_CORE_3M		322U
/** @brief Monitored branch of PEX0_CORE_4 clock */
#define TEGRA194_CLK_PEX0_CORE_4M		323U
/** @brief Monitored branch of PEX1_CORE_5 clock */
#define TEGRA194_CLK_PEX1_CORE_5M		324U
/** @brief NVHS UPHY PLL0-based NVLINK TX clock output */
#define TEGRA194_CLK_NVLINK_PLL_TXCLK		325U
/** @brief PLLE hardware power sequencer (overrides 'manual' programming of PLL) */
#define TEGRA194_CLK_PLLE_HPS			326U
/** @brief CLK_ENB_PLLREFE_OUT gate output */
#define TEGRA194_CLK_PLLREFE_VCOOUT_GATED	327U
/** @brief TEGRA194_CLK_SOR_SAFE clk source (PLLP_OUT0 divided by 17) */
#define TEGRA194_CLK_PLLP_DIV17			328U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_SOC_THERM switch divider output */
#define TEGRA194_CLK_SOC_THERM			329U
/** @brief CLK_RST_CONTROLLER_CLK_SOURCE_TSENSOR switch divider output */
#define TEGRA194_CLK_TSENSOR			330U
/** @brief CLK_RST_CONTROLLER_CLK_OUT_ENB_XUSB_0.CLK_ENB_XUSB gate */
#define TEGRA194_CLK_XUSB_PADCTL		331U
/** @brief PLL controlled by CLK_RST_CONTROLLER_PLLBPMPCAM_BASE */
#define TEGRA194_CLK_PLLBPMPCAM			332U

/** @} */

#endif
