/*
 * Pinctrl data for the NVIDIA Tegra194 pinmux
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>

#include "pinctrl-tegra.h"

/*
 * Most pins affected by the pinmux can also be GPIOs. Define these first.
 * These must match how the GPIO driver names/numbers its pins.
 */
#define T194_PIN_TABLE(fname)					\
	fname(DAP6_SCLK_PA0, dap6_sclk_pa0, _GPIO, 0)		\
	fname(DAP6_DOUT_PA1, dap6_dout_pa1, _GPIO, 1)		\
	fname(DAP6_DIN_PA2, dap6_din_pa2, _GPIO, 2)		\
	fname(DAP6_FS_PA3, dap6_fs_pa3, _GPIO, 3)		\
	fname(DAP4_SCLK_PA4, dap4_sclk_pa4, _GPIO, 4)		\
	fname(DAP4_DOUT_PA5, dap4_dout_pa5, _GPIO, 5)		\
	fname(DAP4_DIN_PA6, dap4_din_pa6, _GPIO, 6)		\
	fname(DAP4_FS_PA7, dap4_fs_pa7, _GPIO, 7)		\
	fname(CPU_PWR_REQ_0_PB0, cpu_pwr_req_0_pb0, _GPIO, 8)	\
	fname(CPU_PWR_REQ_1_PB1, cpu_pwr_req_1_pb1, _GPIO, 9)	\
	fname(QSPI0_SCK_PC0, qspi0_sck_pc0, _GPIO, 16)		\
	fname(QSPI0_CS_N_PC1, qspi0_cs_n_pc1, _GPIO, 17)	\
	fname(QSPI0_IO0_PC2, qspi0_io0_pc2, _GPIO, 18)		\
	fname(QSPI0_IO1_PC3, qspi0_io1_pc3, _GPIO, 19)		\
	fname(QSPI0_IO2_PC4, qspi0_io2_pc4, _GPIO, 20)		\
	fname(QSPI0_IO3_PC5, qspi0_io3_pc5, _GPIO, 21)		\
	fname(QSPI1_SCK_PC6, qspi1_sck_pc6, _GPIO, 22)		\
	fname(QSPI1_CS_N_PC7, qspi1_cs_n_pc7, _GPIO, 23)	\
	fname(QSPI1_IO0_PD0, qspi1_io0_pd0, _GPIO, 24)		\
	fname(QSPI1_IO1_PD1, qspi1_io1_pd1, _GPIO, 25)		\
	fname(QSPI1_IO2_PD2, qspi1_io2_pd2, _GPIO, 26)		\
	fname(QSPI1_IO3_PD3, qspi1_io3_pd3, _GPIO, 27)		\
	fname(EQOS_TXC_PE0, eqos_txc_pe0, _GPIO, 32)		\
	fname(EQOS_TD0_PE1, eqos_td0_pe1, _GPIO, 33)		\
	fname(EQOS_TD1_PE2, eqos_td1_pe2, _GPIO, 34)		\
	fname(EQOS_TD2_PE3, eqos_td2_pe3, _GPIO, 35)		\
	fname(EQOS_TD3_PE4, eqos_td3_pe4, _GPIO, 36)		\
	fname(EQOS_TX_CTL_PE5, eqos_tx_ctl_pe5, _GPIO, 37)	\
	fname(EQOS_RD0_PE6, eqos_rd0_pe6, _GPIO, 38)		\
	fname(EQOS_RD1_PE7, eqos_rd1_pe7, _GPIO, 39)		\
	fname(EQOS_RD2_PF0, eqos_rd2_pf0, _GPIO, 40)		\
	fname(EQOS_RD3_PF1, eqos_rd3_pf1, _GPIO, 41)		\
	fname(EQOS_RX_CTL_PF2, eqos_rx_ctl_pf2, _GPIO, 42)	\
	fname(EQOS_RXC_PF3, eqos_rxc_pf3, _GPIO, 43)		\
	fname(EQOS_SMA_MDIO_PF4, eqos_sma_mdio_pf4, _GPIO, 44)	\
	fname(EQOS_SMA_MDC_PF5, eqos_sma_mdc_pf5, _GPIO, 45)	\
	fname(SOC_GPIO00_PG0, soc_gpio00_pg0, _GPIO, 48)	\
	fname(SOC_GPIO01_PG1, soc_gpio01_pg1, _GPIO, 49)	\
	fname(SOC_GPIO02_PG2, soc_gpio02_pg2, _GPIO, 50)	\
	fname(SOC_GPIO03_PG3, soc_gpio03_pg3, _GPIO, 51)	\
	fname(SOC_GPIO08_PG4, soc_gpio08_pg4, _GPIO, 52)	\
	fname(SOC_GPIO09_PG5, soc_gpio09_pg5, _GPIO, 53)	\
	fname(SOC_GPIO10_PG6, soc_gpio10_pg6, _GPIO, 54)	\
	fname(SOC_GPIO11_PG7, soc_gpio11_pg7, _GPIO, 55)	\
	fname(SOC_GPIO12_PH0, soc_gpio12_ph0, _GPIO, 56)	\
	fname(SOC_GPIO13_PH1, soc_gpio13_ph1, _GPIO, 57)	\
	fname(SOC_GPIO14_PH2, soc_gpio14_ph2, _GPIO, 58)	\
	fname(UART4_TX_PH3, uart4_tx_ph3, _GPIO, 59)		\
	fname(UART4_RX_PH4, uart4_rx_ph4, _GPIO, 60)		\
	fname(UART4_RTS_PH5, uart4_rts_ph5, _GPIO, 61)		\
	fname(UART4_CTS_PH6, uart4_cts_ph6, _GPIO, 62)		\
	fname(DAP2_SCLK_PH7, dap2_sclk_ph7, _GPIO, 63)		\
	fname(DAP2_DOUT_PI0, dap2_dout_pi0, _GPIO, 64)		\
	fname(DAP2_DIN_PI1, dap2_din_pi1, _GPIO, 65)		\
	fname(DAP2_FS_PI2, dap2_fs_pi2, _GPIO, 66)		\
	fname(GEN1_I2C_SCL_PI3, gen1_i2c_scl_pi3, _GPIO, 67)	\
	fname(GEN1_I2C_SDA_PI4, gen1_i2c_sda_pi4, _GPIO, 68)	\
	fname(SDMMC1_CLK_PJ0, sdmmc1_clk_pj0, _GPIO, 72)	\
	fname(SDMMC1_CMD_PJ1, sdmmc1_cmd_pj1, _GPIO, 73)	\
	fname(SDMMC1_DAT0_PJ2, sdmmc1_dat0_pj2, _GPIO, 74)	\
	fname(SDMMC1_DAT1_PJ3, sdmmc1_dat1_pj3, _GPIO, 75)	\
	fname(SDMMC1_DAT2_PJ4, sdmmc1_dat2_pj4, _GPIO, 76)	\
	fname(SDMMC1_DAT3_PJ5, sdmmc1_dat3_pj5, _GPIO, 77)	\
	fname(PEX_L0_CLKREQ_N_PK0, pex_l0_clkreq_n_pk0, _GPIO, 80)\
	fname(PEX_L0_RST_N_PK1, pex_l0_rst_n_pk1, _GPIO, 81)	\
	fname(PEX_L1_CLKREQ_N_PK2, pex_l1_clkreq_n_pk2, _GPIO, 82)\
	fname(PEX_L1_RST_N_PK3, pex_l1_rst_n_pk3, _GPIO, 83)	\
	fname(PEX_L2_CLKREQ_N_PK4, pex_l2_clkreq_n_pk4, _GPIO, 84)\
	fname(PEX_L2_RST_N_PK5, pex_l2_rst_n_pk5, _GPIO, 85)	\
	fname(PEX_L3_CLKREQ_N_PK6, pex_l3_clkreq_n_pk6, _GPIO, 86)\
	fname(PEX_L3_RST_N_PK7, pex_l3_rst_n_pk7, _GPIO, 87)	\
	fname(PEX_L4_CLKREQ_N_PL0, pex_l4_clkreq_n_pl0, _GPIO, 88)\
	fname(PEX_L4_RST_N_PL1, pex_l4_rst_n_pl1, _GPIO, 89)	\
	fname(PEX_WAKE_N_PL2, pex_wake_n_pl2, _GPIO, 90)	\
	fname(SATA_DEV_SLP_PL3, sata_dev_slp_pl3, _GPIO, 91)	\
	fname(DP_AUX_CH0_HPD_PM0, dp_aux_ch0_hpd_pm0, _GPIO, 96)\
	fname(DP_AUX_CH1_HPD_PM1, dp_aux_ch1_hpd_pm1, _GPIO, 97)\
	fname(DP_AUX_CH2_HPD_PM2, dp_aux_ch2_hpd_pm2, _GPIO, 98)\
	fname(DP_AUX_CH3_HPD_PM3, dp_aux_ch3_hpd_pm3, _GPIO, 99)\
	fname(HDMI_CEC_PM4, hdmi_cec_pm4, _GPIO, 100)		\
	fname(SOC_GPIO50_PM5, soc_gpio50_pm5, _GPIO, 101)	\
	fname(SOC_GPIO51_PM6, soc_gpio51_pm6, _GPIO, 102)	\
	fname(SOC_GPIO52_PM7, soc_gpio52_pm7, _GPIO, 103)	\
	fname(SOC_GPIO53_PN0, soc_gpio53_pn0, _GPIO, 104)	\
	fname(SOC_GPIO54_PN1, soc_gpio54_pn1, _GPIO, 105)	\
	fname(SOC_GPIO55_PN2, soc_gpio55_pn2, _GPIO, 106)	\
	fname(SDMMC3_CLK_PO0, sdmmc3_clk_po0, _GPIO, 112)	\
	fname(SDMMC3_CMD_PO1, sdmmc3_cmd_po1, _GPIO, 113)	\
	fname(SDMMC3_DAT0_PO2, sdmmc3_dat0_po2, _GPIO, 114)	\
	fname(SDMMC3_DAT1_PO3, sdmmc3_dat1_po3, _GPIO, 115)	\
	fname(SDMMC3_DAT2_PO4, sdmmc3_dat2_po4, _GPIO, 116)	\
	fname(SDMMC3_DAT3_PO5, sdmmc3_dat3_po5, _GPIO, 117)	\
	fname(EXTPERIPH1_CLK_PP0, extperiph1_clk_pp0, _GPIO, 120)\
	fname(EXTPERIPH2_CLK_PP1, extperiph2_clk_pp1, _GPIO, 121)\
	fname(CAM_I2C_SCL_PP2, cam_i2c_scl_pp2, _GPIO, 122)	\
	fname(CAM_I2C_SDA_PP3, cam_i2c_sda_pp3, _GPIO, 123)	\
	fname(SOC_GPIO04_PP4, soc_gpio04_pp4, _GPIO, 124)	\
	fname(SOC_GPIO05_PP5, soc_gpio05_pp5, _GPIO, 125)	\
	fname(SOC_GPIO06_PP6, soc_gpio06_pp6, _GPIO, 126)	\
	fname(SOC_GPIO07_PP7, soc_gpio07_pp7, _GPIO, 127)	\
	fname(SOC_GPIO20_PQ0, soc_gpio20_pq0, _GPIO, 128)	\
	fname(SOC_GPIO21_PQ1, soc_gpio21_pq1, _GPIO, 129)	\
	fname(SOC_GPIO22_PQ2, soc_gpio22_pq2, _GPIO, 130)	\
	fname(SOC_GPIO23_PQ3, soc_gpio23_pq3, _GPIO, 131)	\
	fname(SOC_GPIO40_PQ4, soc_gpio40_pq4, _GPIO, 132)	\
	fname(SOC_GPIO41_PQ5, soc_gpio41_pq5, _GPIO, 133)	\
	fname(SOC_GPIO42_PQ6, soc_gpio42_pq6, _GPIO, 134)	\
	fname(SOC_GPIO43_PQ7, soc_gpio43_pq7, _GPIO, 135)	\
	fname(SOC_GPIO44_PR0, soc_gpio44_pr0, _GPIO, 136)	\
	fname(SOC_GPIO45_PR1, soc_gpio45_pr1, _GPIO, 137)	\
	fname(UART1_TX_PR2, uart1_tx_pr2, _GPIO, 138)		\
	fname(UART1_RX_PR3, uart1_rx_pr3, _GPIO, 139)		\
	fname(UART1_RTS_PR4, uart1_rts_pr4, _GPIO, 140)		\
	fname(UART1_CTS_PR5, uart1_cts_pr5, _GPIO, 141)		\
	fname(DAP1_SCLK_PS0, dap1_sclk_ps0, _GPIO, 144)		\
	fname(DAP1_DOUT_PS1, dap1_dout_ps1, _GPIO, 145)		\
	fname(DAP1_DIN_PS2, dap1_din_ps2, _GPIO, 146)		\
	fname(DAP1_FS_PS3, dap1_fs_ps3, _GPIO, 147)		\
	fname(AUD_MCLK_PS4, aud_mclk_ps4, _GPIO, 148)		\
	fname(SOC_GPIO30_PS5, soc_gpio30_ps5, _GPIO, 149)	\
	fname(SOC_GPIO31_PS6, soc_gpio31_ps6, _GPIO, 150)	\
	fname(SOC_GPIO32_PS7, soc_gpio32_ps7, _GPIO, 151)	\
	fname(SOC_GPIO33_PT0, soc_gpio33_pt0, _GPIO, 152)	\
	fname(DAP3_SCLK_PT1, dap3_sclk_pt1, _GPIO, 153)		\
	fname(DAP3_DOUT_PT2, dap3_dout_pt2, _GPIO, 154)		\
	fname(DAP3_DIN_PT3, dap3_din_pt3, _GPIO, 155)		\
	fname(DAP3_FS_PT4, dap3_fs_pt4, _GPIO, 156)		\
	fname(DAP5_SCLK_PT5, dap5_sclk_pt5, _GPIO, 157)		\
	fname(DAP5_DOUT_PT6, dap5_dout_pt6, _GPIO, 158)		\
	fname(DAP5_DIN_PT7, dap5_din_pt7, _GPIO, 159)		\
	fname(DAP5_FS_PU0, dap5_fs_pu0, _GPIO, 160)		\
	fname(DIRECTDC1_CLK_PV0, directdc1_clk_pv0, _GPIO, 168)	\
	fname(DIRECTDC1_IN_PV1, directdc1_in_pv1, _GPIO, 169)	\
	fname(DIRECTDC1_OUT0_PV2, directdc1_out0_pv2, _GPIO, 170)\
	fname(DIRECTDC1_OUT1_PV3, directdc1_out1_pv3, _GPIO, 171)\
	fname(DIRECTDC1_OUT2_PV4, directdc1_out2_pv4, _GPIO, 172)\
	fname(DIRECTDC1_OUT3_PV5, directdc1_out3_pv5, _GPIO, 173)\
	fname(DIRECTDC1_OUT4_PV6, directdc1_out4_pv6, _GPIO, 174)\
	fname(DIRECTDC1_OUT5_PV7, directdc1_out5_pv7, _GPIO, 175)\
	fname(DIRECTDC1_OUT6_PW0, directdc1_out6_pw0, _GPIO, 176)\
	fname(DIRECTDC1_OUT7_PW1, directdc1_out7_pw1, _GPIO, 177)\
	fname(GPU_PWR_REQ_PX0, gpu_pwr_req_px0, _GPIO, 184)	\
	fname(CV_PWR_REQ_PX1, cv_pwr_req_px1, _GPIO, 185)	\
	fname(GP_PWM2_PX2, gp_pwm2_px2, _GPIO, 186)		\
	fname(GP_PWM3_PX3, gp_pwm3_px3, _GPIO, 187)		\
	fname(UART2_TX_PX4, uart2_tx_px4, _GPIO, 188)		\
	fname(UART2_RX_PX5, uart2_rx_px5, _GPIO, 189)		\
	fname(UART2_RTS_PX6, uart2_rts_px6, _GPIO, 190)		\
	fname(UART2_CTS_PX7, uart2_cts_px7, _GPIO, 191)		\
	fname(SPI3_SCK_PY0, spi3_sck_py0, _GPIO, 192)		\
	fname(SPI3_MISO_PY1, spi3_miso_py1, _GPIO, 193)		\
	fname(SPI3_MOSI_PY2, spi3_mosi_py2, _GPIO, 194)		\
	fname(SPI3_CS0_PY3, spi3_cs0_py3, _GPIO, 195)		\
	fname(SPI3_CS1_PY4, spi3_cs1_py4, _GPIO, 196)		\
	fname(UART5_TX_PY5, uart5_tx_py5, _GPIO, 197)		\
	fname(UART5_RX_PY6, uart5_rx_py6, _GPIO, 198)		\
	fname(UART5_RTS_PY7, uart5_rts_py7, _GPIO, 199)		\
	fname(UART5_CTS_PZ0, uart5_cts_pz0, _GPIO, 200)		\
	fname(USB_VBUS_EN0_PZ1, usb_vbus_en0_pz1, _GPIO, 201)	\
	fname(USB_VBUS_EN1_PZ2, usb_vbus_en1_pz2, _GPIO, 202)	\
	fname(SPI1_SCK_PZ3, spi1_sck_pz3, _GPIO, 203)		\
	fname(SPI1_MISO_PZ4, spi1_miso_pz4, _GPIO, 204)		\
	fname(SPI1_MOSI_PZ5, spi1_mosi_pz5, _GPIO, 205)		\
	fname(SPI1_CS0_PZ6, spi1_cs0_pz6, _GPIO, 206)		\
	fname(SPI1_CS1_PZ7, spi1_cs1_pz7, _GPIO, 207)		\
	fname(CAN1_DOUT_PAA0, can1_dout_paa0, _GPIO, 208)	\
	fname(CAN1_DIN_PAA1, can1_din_paa1, _GPIO, 209)		\
	fname(CAN0_DOUT_PAA2, can0_dout_paa2, _GPIO, 210)	\
	fname(CAN0_DIN_PAA3, can0_din_paa3, _GPIO, 211)		\
	fname(CAN0_STB_PAA4, can0_stb_paa4, _GPIO, 212)		\
	fname(CAN0_EN_PAA5, can0_en_paa5, _GPIO, 213)		\
	fname(CAN0_WAKE_PAA6, can0_wake_paa6, _GPIO, 214)	\
	fname(CAN0_ERR_PAA7, can0_err_paa7, _GPIO, 215)		\
	fname(CAN1_STB_PBB0, can1_stb_pbb0, _GPIO, 216)		\
	fname(CAN1_EN_PBB1, can1_en_pbb1, _GPIO, 217)		\
	fname(CAN1_WAKE_PBB2, can1_wake_pbb2, _GPIO, 218)	\
	fname(CAN1_ERR_PBB3, can1_err_pbb3, _GPIO, 219)		\
	fname(SPI2_SCK_PCC0, spi2_sck_pcc0, _GPIO, 224)		\
	fname(SPI2_MISO_PCC1, spi2_miso_pcc1, _GPIO, 225)	\
	fname(SPI2_MOSI_PCC2, spi2_mosi_pcc2, _GPIO, 226)	\
	fname(SPI2_CS0_PCC3, spi2_cs0_pcc3, _GPIO, 227)		\
	fname(TOUCH_CLK_PCC4, touch_clk_pcc4, _GPIO, 228)	\
	fname(UART3_TX_PCC5, uart3_tx_pcc5, _GPIO, 229)		\
	fname(UART3_RX_PCC6, uart3_rx_pcc6, _GPIO, 230)		\
	fname(GEN2_I2C_SCL_PCC7, gen2_i2c_scl_pcc7, _GPIO, 231)	\
	fname(GEN2_I2C_SDA_PDD0, gen2_i2c_sda_pdd0, _GPIO, 232)	\
	fname(GEN8_I2C_SCL_PDD1, gen8_i2c_scl_pdd1, _GPIO, 233)	\
	fname(GEN8_I2C_SDA_PDD2, gen8_i2c_sda_pdd2, _GPIO, 234)	\
	fname(SAFE_STATE_PEE0, safe_state_pee0, _GPIO, 240)	\
	fname(VCOMP_ALERT_PEE1, vcomp_alert_pee1, _GPIO, 241)	\
	fname(AO_RETENTION_N_PEE2, ao_retention_n_pee2, _GPIO, 242)\
	fname(BATT_OC_PEE3, batt_oc_pee3, _GPIO, 243)		\
	fname(POWER_ON_PEE4, power_on_pee4, _GPIO, 244)		\
	fname(PWR_I2C_SCL_PEE5, pwr_i2c_scl_pee5, _GPIO, 245)	\
	fname(PWR_I2C_SDA_PEE6, pwr_i2c_sda_pee6, _GPIO, 246)	\
	fname(UFS0_REF_CLK_PFF0, ufs0_ref_clk_pff0, _GPIO, 248)	\
	fname(UFS0_RST_PFF1, ufs0_rst_pff1, _GPIO, 249)		\
	fname(PEX_L5_CLKREQ_N_PGG0, pex_l5_clkreq_n_pgg0, _GPIO, 256)	\
	fname(PEX_L5_RST_N_PGG1, pex_l5_rst_n_pgg1, _GPIO, 257)	\
	fname(DIRECTDC_COMP, directdc_comp, _PIN, 0)		\
	fname(SDMMC4_CLK, sdmmc4_clk, _PIN, 1)			\
	fname(SDMMC4_CMD, sdmmc4_cmd, _PIN, 2)			\
	fname(SDMMC4_DQS, sdmmc4_dqs, _PIN, 3)			\
	fname(SDMMC4_DAT7, sdmmc4_dat7, _PIN, 4)		\
	fname(SDMMC4_DAT6, sdmmc4_dat6, _PIN, 5)		\
	fname(SDMMC4_DAT5, sdmmc4_dat5, _PIN, 6)		\
	fname(SDMMC4_DAT4, sdmmc4_dat4, _PIN, 7)		\
	fname(SDMMC4_DAT3, sdmmc4_dat3, _PIN, 8)		\
	fname(SDMMC4_DAT2, sdmmc4_dat2, _PIN, 9)		\
	fname(SDMMC4_DAT1, sdmmc4_dat1, _PIN, 10)		\
	fname(SDMMC4_DAT0, sdmmc4_dat0, _PIN, 11)		\
	fname(SDMMC1_COMP, sdmmc1_comp, _PIN, 12)		\
	fname(SDMMC1_HV_TRIM, sdmmc1_hv_trim, _PIN, 13)		\
	fname(SDMMC3_COMP, sdmmc3_comp, _PIN, 14)		\
	fname(SDMMC3_HV_TRIM, sdmmc3_hv_trim, _PIN, 15)		\
	fname(EQOS_COMP, eqos_comp, _PIN, 16)			\
	fname(QSPI_COMP, qspi_comp, _PIN, 17)			\
	fname(SYS_RESET_N, sys_reset_n, _PIN, 18)		\
	fname(SHUTDOWN_N, shutdown_n, _PIN, 19)			\
	fname(PMU_INT_N, pmu_int_n, _PIN, 20)			\
	fname(SOC_PWR_REQ, soc_pwr_req, _PIN, 21)		\
	fname(CLK_32K_IN, clk_32k_in, _PIN, 22)			\

#define _GPIO(offset)			(offset)
#define NUM_GPIOS			(TEGRA_PIN_PEX_L5_RST_N_PGG1 + 1)
#define _PIN(offset)			(NUM_GPIOS + (offset))

/* Define unique ID for each pins */
#define TEGRA_PINCTRL_PIN_NUM(id, lid, _f, num)			\
	TEGRA_PIN_##id = _f(num),
enum pin_id {
	T194_PIN_TABLE(TEGRA_PINCTRL_PIN_NUM)
};

/* Table for pin descriptor */
#define TEGRA_PINCTRL_PIN(id, lid, f, num)	PINCTRL_PIN(TEGRA_PIN_##id, #id),
static const struct pinctrl_pin_desc tegra194_pins[] = {
	T194_PIN_TABLE(TEGRA_PINCTRL_PIN)
};

#define TEGRA_PINCTRL_PINS_STRUCT(id, lid, f, num)		\
	static const unsigned lid##_pins[] = {			\
		TEGRA_PIN_##id,					\
	};
T194_PIN_TABLE(TEGRA_PINCTRL_PINS_STRUCT)

#define T194_FUNCTION_TABLE(fname)		\
	fname(RSVD0, rsvd0)			\
	fname(RSVD1, rsvd1)			\
	fname(RSVD2, rsvd2)			\
	fname(RSVD3, rsvd3)			\
	fname(TOUCH, touch)			\
	fname(UARTC, uartc)			\
	fname(I2C8, i2c8)			\
	fname(UARTG, uartg)			\
	fname(SPI2, spi2)			\
	fname(GP, gp)				\
	fname(DCA, dca)				\
	fname(WDT, wdt)				\
	fname(I2C2, i2c2)			\
	fname(CAN1, can1)			\
	fname(CAN0, can0)			\
	fname(DMIC3, dmic3)			\
	fname(DMIC5, dmic5)			\
	fname(GPIO, gpio)			\
	fname(DSPK1, dspk1)			\
	fname(DSPK0, dspk0)			\
	fname(SPDIF, spdif)			\
	fname(AUD, aud)				\
	fname(I2S1, i2s1)			\
	fname(DMIC1, dmic1)			\
	fname(DMIC2, dmic2)			\
	fname(I2S3, i2s3)			\
	fname(DMIC4, dmic4)			\
	fname(I2S4, i2s4)			\
	fname(EXTPERIPH2, extperiph2)		\
	fname(EXTPERIPH1, extperiph1)		\
	fname(I2C3, i2c3)			\
	fname(VGP1, vgp1)			\
	fname(VGP2, vgp2)			\
	fname(VGP3, vgp3)			\
	fname(VGP4, vgp4)			\
	fname(VGP5, vgp5)			\
	fname(VGP6, vgp6)			\
	fname(SLVS, slvs)			\
	fname(EXTPERIPH3, extperiph3)		\
	fname(EXTPERIPH4, extperiph4)		\
	fname(I2S2, i2s2)			\
	fname(UARTD, uartd)			\
	fname(I2C1, i2c1)			\
	fname(UARTA, uarta)			\
	fname(DIRECTDC1, directdc1)		\
	fname(DIRECTDC, directdc)		\
	fname(IQC1, iqc1)			\
	fname(IQC2, iqc2)			\
	fname(I2S6, i2s6)			\
	fname(SDMMC3, sdmmc3)			\
	fname(SDMMC1, sdmmc1)			\
	fname(DP, dp)				\
	fname(HDMI, hdmi)			\
	fname(PE2, pe2)				\
	fname(IGPU, igpu)			\
	fname(SATA, sata)			\
	fname(PE1, pe1)				\
	fname(PE0, pe0)				\
	fname(PE3, pe3)				\
	fname(PE4, pe4)				\
	fname(PE5, pe5)				\
	fname(SOC, soc)				\
	fname(EQOS, eqos)			\
	fname(QSPI, qspi)			\
	fname(QSPI0, qspi0)			\
	fname(QSPI1, qspi1)			\
	fname(MIPI, mipi)			\
	fname(SCE, sce)				\
	fname(I2C5, i2c5)			\
	fname(DISPLAYA, displaya)		\
	fname(DISPLAYB, displayb)		\
	fname(DCB, dcb)				\
	fname(SPI1, spi1)			\
	fname(UARTB, uartb)			\
	fname(UARTE, uarte)			\
	fname(SPI3, spi3)			\
	fname(NV, nv)				\
	fname(CCLA, ccla)			\
	fname(I2S5, i2s5)			\
	fname(USB, usb)				\
	fname(UFS0, ufs0)			\
	fname(DGPU, dgpu)			\
	fname(SDMMC4, sdmmc4)

/* Define unique ID for each function */
#define TEGRA_PIN_FUNCTION_MUX_ENUM(id, lid)	\
	TEGRA_MUX_##id,
enum tegra_mux_dt {
	T194_FUNCTION_TABLE(TEGRA_PIN_FUNCTION_MUX_ENUM)
};

/* Make list of each function name */
#define TEGRA_PIN_FUNCTION(id, lid)		\
	{					\
		.name = #lid,			\
	},
static struct tegra_function tegra194_functions[] = {
	T194_FUNCTION_TABLE(TEGRA_PIN_FUNCTION)
};

#define PINGROUP_REG_Y(r) ((r))
#define PINGROUP_REG_N(r) -1

#define DRV_PINGROUP_Y(r) ((r))
#define DRV_PINGROUP_N(r) -1

#define DRV_PINGROUP_ENTRY_N(pg_name)				\
		.drv_reg = -1,					\
		.drv_bank = -1,					\
		.drvdn_bit = -1,				\
		.drvup_bit = -1,				\
		.slwr_bit = -1,					\
		.slwf_bit = -1

#define DRV_PINGROUP_ENTRY_Y(r, drvdn_b, drvdn_w, drvup_b,	\
			     drvup_w, slwr_b, slwr_w, slwf_b,	\
			     slwf_w, bank)			\
		.drv_reg = DRV_PINGROUP_Y(r),			\
		.drv_bank = bank,				\
		.drvdn_bit = drvdn_b,				\
		.drvdn_width = drvdn_w,				\
		.drvup_bit = drvup_b,				\
		.drvup_width = drvup_w,				\
		.slwr_bit = slwr_b,				\
		.slwr_width = slwr_w,				\
		.slwf_bit = slwf_b,				\
		.slwf_width = slwf_w

#define PIN_PINGROUP_ENTRY_N(pg_name)				\
		.mux_reg = -1,					\
		.pupd_reg = -1,					\
		.tri_reg = -1,					\
		.lpbk_reg = -1,					\
		.einput_bit = -1,				\
		.e_io_hv_bit = -1,				\
		.odrain_bit = -1,				\
		.lock_bit = -1,					\
		.parked_bit = -1,				\
		.lpmd_bit = -1,					\
		.drvtype_bit = -1,				\
		.lpdr_bit = -1,					\
		.pbias_buf_bit = -1,				\
		.preemp_bit = -1,				\
		.lpbk_bit = -1,					\
		.rfu_in_bit = -1

#define PIN_PINGROUP_ENTRY_Y(r, bank, pupd, e_io_hv, e_lpbk, e_input,	\
			     e_lpdr, e_pbias_buf, gpio_sfio_sel, \
			     e_od, schmitt_b, drvtype, epreemp,	\
			     io_reset, rfu_in, io_rail)		\
		.mux_reg = PINGROUP_REG_Y(r),			\
		.lpmd_bit = -1,					\
		.lock_bit = -1,					\
		.hsm_bit = -1,					\
		.parked_bit = -1,				\
		.mux_bank = bank,				\
		.mux_bit = 0,					\
		.pupd_reg = PINGROUP_REG_##pupd(r),		\
		.pupd_bank = bank,				\
		.pupd_bit = 2,					\
		.tri_reg = PINGROUP_REG_Y(r),			\
		.tri_bank = bank,				\
		.tri_bit = 4,					\
		.e_io_hv_bit = e_io_hv,				\
		.einput_bit = e_input,				\
		.gpio_bit = gpio_sfio_sel,			\
		.odrain_bit = e_od,				\
		.schmitt_bit = schmitt_b,			\
		.drvtype_bit = 13,				\
		.lpdr_bit = e_lpdr,				\
		.pbias_buf_bit = e_pbias_buf,			\
		.preemp_bit = epreemp,				\
		.rfu_in_bit = 20,				\
		.drv_reg = -1,					\
		.lpbk_reg = PINGROUP_REG_Y(r),			\
		.lpbk_bank = bank,				\
		.lpbk_bit = e_lpbk,				\
		.pwr_domain = #io_rail

#define drive_touch_clk_pcc4            DRV_PINGROUP_ENTRY_Y(0x2004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_rx_pcc6             DRV_PINGROUP_ENTRY_Y(0x200c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_tx_pcc5             DRV_PINGROUP_ENTRY_Y(0x2014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen8_i2c_sda_pdd2         DRV_PINGROUP_ENTRY_Y(0x201c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen8_i2c_scl_pdd1         DRV_PINGROUP_ENTRY_Y(0x2024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_spi2_mosi_pcc2            DRV_PINGROUP_ENTRY_Y(0x202c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen2_i2c_scl_pcc7         DRV_PINGROUP_ENTRY_Y(0x2034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_spi2_cs0_pcc3             DRV_PINGROUP_ENTRY_Y(0x203c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen2_i2c_sda_pdd0         DRV_PINGROUP_ENTRY_Y(0x2044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_spi2_sck_pcc0             DRV_PINGROUP_ENTRY_Y(0x204c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_spi2_miso_pcc1            DRV_PINGROUP_ENTRY_Y(0x2054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_can1_dout_paa0            DRV_PINGROUP_ENTRY_Y(0x3004,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can1_din_paa1             DRV_PINGROUP_ENTRY_Y(0x300c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_dout_paa2            DRV_PINGROUP_ENTRY_Y(0x3014,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_din_paa3             DRV_PINGROUP_ENTRY_Y(0x301c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_stb_paa4             DRV_PINGROUP_ENTRY_Y(0x3024,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_en_paa5              DRV_PINGROUP_ENTRY_Y(0x302c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_wake_paa6            DRV_PINGROUP_ENTRY_Y(0x3034,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can0_err_paa7             DRV_PINGROUP_ENTRY_Y(0x303c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can1_stb_pbb0             DRV_PINGROUP_ENTRY_Y(0x3044,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can1_en_pbb1              DRV_PINGROUP_ENTRY_Y(0x304c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can1_wake_pbb2            DRV_PINGROUP_ENTRY_Y(0x3054,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_can1_err_pbb3             DRV_PINGROUP_ENTRY_Y(0x305c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	1)
#define drive_soc_gpio33_pt0            DRV_PINGROUP_ENTRY_Y(0x1004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio32_ps7            DRV_PINGROUP_ENTRY_Y(0x100c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio31_ps6            DRV_PINGROUP_ENTRY_Y(0x1014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio30_ps5            DRV_PINGROUP_ENTRY_Y(0x101c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_aud_mclk_ps4              DRV_PINGROUP_ENTRY_Y(0x1024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_fs_ps3               DRV_PINGROUP_ENTRY_Y(0x102c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_din_ps2              DRV_PINGROUP_ENTRY_Y(0x1034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_dout_ps1             DRV_PINGROUP_ENTRY_Y(0x103c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_sclk_ps0             DRV_PINGROUP_ENTRY_Y(0x1044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap3_fs_pt4               DRV_PINGROUP_ENTRY_Y(0x104c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap3_din_pt3              DRV_PINGROUP_ENTRY_Y(0x1054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap3_dout_pt2             DRV_PINGROUP_ENTRY_Y(0x105c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap3_sclk_pt1             DRV_PINGROUP_ENTRY_Y(0x1064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap5_fs_pu0               DRV_PINGROUP_ENTRY_Y(0x106c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap5_din_pt7              DRV_PINGROUP_ENTRY_Y(0x1074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap5_dout_pt6             DRV_PINGROUP_ENTRY_Y(0x107c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap5_sclk_pt5             DRV_PINGROUP_ENTRY_Y(0x1084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap6_fs_pa3               DRV_PINGROUP_ENTRY_Y(0x2004,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap6_din_pa2              DRV_PINGROUP_ENTRY_Y(0x200c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap6_dout_pa1             DRV_PINGROUP_ENTRY_Y(0x2014,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap6_sclk_pa0             DRV_PINGROUP_ENTRY_Y(0x201c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap4_fs_pa7               DRV_PINGROUP_ENTRY_Y(0x2024,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap4_din_pa6              DRV_PINGROUP_ENTRY_Y(0x202c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap4_dout_pa5             DRV_PINGROUP_ENTRY_Y(0x2034,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_dap4_sclk_pa4             DRV_PINGROUP_ENTRY_Y(0x203c,	28,	2,	30,	2,	-1,	-1,	-1,	-1,	0)
#define drive_extperiph2_clk_pp1        DRV_PINGROUP_ENTRY_Y(0x0004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_extperiph1_clk_pp0        DRV_PINGROUP_ENTRY_Y(0x000c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cam_i2c_sda_pp3           DRV_PINGROUP_ENTRY_Y(0x0014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cam_i2c_scl_pp2           DRV_PINGROUP_ENTRY_Y(0x001c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio40_pq4            DRV_PINGROUP_ENTRY_Y(0x0024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio41_pq5            DRV_PINGROUP_ENTRY_Y(0x002c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio42_pq6            DRV_PINGROUP_ENTRY_Y(0x0034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio43_pq7            DRV_PINGROUP_ENTRY_Y(0x003c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio44_pr0            DRV_PINGROUP_ENTRY_Y(0x0044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio45_pr1            DRV_PINGROUP_ENTRY_Y(0x004c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio20_pq0            DRV_PINGROUP_ENTRY_Y(0x0054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio21_pq1            DRV_PINGROUP_ENTRY_Y(0x005c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio22_pq2            DRV_PINGROUP_ENTRY_Y(0x0064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio23_pq3            DRV_PINGROUP_ENTRY_Y(0x006c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio04_pp4            DRV_PINGROUP_ENTRY_Y(0x0074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio05_pp5            DRV_PINGROUP_ENTRY_Y(0x007c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio06_pp6            DRV_PINGROUP_ENTRY_Y(0x0084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio07_pp7            DRV_PINGROUP_ENTRY_Y(0x008c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_cts_pr5             DRV_PINGROUP_ENTRY_Y(0x0094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_rts_pr4             DRV_PINGROUP_ENTRY_Y(0x009c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_rx_pr3              DRV_PINGROUP_ENTRY_Y(0x00a4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_tx_pr2              DRV_PINGROUP_ENTRY_Y(0x00ac,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_din_pi1              DRV_PINGROUP_ENTRY_Y(0x4004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_dout_pi0             DRV_PINGROUP_ENTRY_Y(0x400c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_fs_pi2               DRV_PINGROUP_ENTRY_Y(0x4014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_sclk_ph7             DRV_PINGROUP_ENTRY_Y(0x401c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_cts_ph6             DRV_PINGROUP_ENTRY_Y(0x4024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_rts_ph5             DRV_PINGROUP_ENTRY_Y(0x402c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_rx_ph4              DRV_PINGROUP_ENTRY_Y(0x4034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_tx_ph3              DRV_PINGROUP_ENTRY_Y(0x403c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio03_pg3            DRV_PINGROUP_ENTRY_Y(0x4044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio02_pg2            DRV_PINGROUP_ENTRY_Y(0x404c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio01_pg1            DRV_PINGROUP_ENTRY_Y(0x4054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio00_pg0            DRV_PINGROUP_ENTRY_Y(0x405c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen1_i2c_scl_pi3          DRV_PINGROUP_ENTRY_Y(0x4064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen1_i2c_sda_pi4          DRV_PINGROUP_ENTRY_Y(0x406c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio08_pg4            DRV_PINGROUP_ENTRY_Y(0x4074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio09_pg5            DRV_PINGROUP_ENTRY_Y(0x407c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio10_pg6            DRV_PINGROUP_ENTRY_Y(0x4084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio11_pg7            DRV_PINGROUP_ENTRY_Y(0x408c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio12_ph0            DRV_PINGROUP_ENTRY_Y(0x4094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio13_ph1            DRV_PINGROUP_ENTRY_Y(0x409c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio14_ph2            DRV_PINGROUP_ENTRY_Y(0x40a4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio50_pm5            DRV_PINGROUP_ENTRY_Y(0x10004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio51_pm6            DRV_PINGROUP_ENTRY_Y(0x1000c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio52_pm7            DRV_PINGROUP_ENTRY_Y(0x10014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio53_pn0            DRV_PINGROUP_ENTRY_Y(0x1001c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio54_pn1            DRV_PINGROUP_ENTRY_Y(0x10024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_soc_gpio55_pn2            DRV_PINGROUP_ENTRY_Y(0x1002c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch0_hpd_pm0        DRV_PINGROUP_ENTRY_Y(0x10034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch1_hpd_pm1        DRV_PINGROUP_ENTRY_Y(0x1003c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch2_hpd_pm2        DRV_PINGROUP_ENTRY_Y(0x10044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch3_hpd_pm3        DRV_PINGROUP_ENTRY_Y(0x1004c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_hdmi_cec_pm4              DRV_PINGROUP_ENTRY_Y(0x10054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l2_clkreq_n_pk4       DRV_PINGROUP_ENTRY_Y(0x7004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_wake_n_pl2            DRV_PINGROUP_ENTRY_Y(0x700c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l1_clkreq_n_pk2       DRV_PINGROUP_ENTRY_Y(0x7014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l1_rst_n_pk3          DRV_PINGROUP_ENTRY_Y(0x701c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l0_clkreq_n_pk0       DRV_PINGROUP_ENTRY_Y(0x7024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l0_rst_n_pk1          DRV_PINGROUP_ENTRY_Y(0x702c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l2_rst_n_pk5          DRV_PINGROUP_ENTRY_Y(0x7034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l3_clkreq_n_pk6       DRV_PINGROUP_ENTRY_Y(0x703c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l3_rst_n_pk7          DRV_PINGROUP_ENTRY_Y(0x7044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l4_clkreq_n_pl0       DRV_PINGROUP_ENTRY_Y(0x704c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l4_rst_n_pl1          DRV_PINGROUP_ENTRY_Y(0x7054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_sata_dev_slp_pl3          DRV_PINGROUP_ENTRY_Y(0x705c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l5_clkreq_n_pgg0      DRV_PINGROUP_ENTRY_Y(0x14004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l5_rst_n_pgg1         DRV_PINGROUP_ENTRY_Y(0x1400c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cpu_pwr_req_1_pb1         DRV_PINGROUP_ENTRY_Y(0x16004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cpu_pwr_req_0_pb0         DRV_PINGROUP_ENTRY_Y(0x1600c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_sdmmc1_clk_pj0            DRV_PINGROUP_ENTRY_Y(0x8004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_cmd_pj1            DRV_PINGROUP_ENTRY_Y(0x800c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat3_pj5           DRV_PINGROUP_ENTRY_Y(0x801c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat2_pj4           DRV_PINGROUP_ENTRY_Y(0x8024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat1_pj3           DRV_PINGROUP_ENTRY_Y(0x802c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat0_pj2           DRV_PINGROUP_ENTRY_Y(0x8034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat3_po5           DRV_PINGROUP_ENTRY_Y(0xa004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat2_po4           DRV_PINGROUP_ENTRY_Y(0xa00c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat1_po3           DRV_PINGROUP_ENTRY_Y(0xa014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat0_po2           DRV_PINGROUP_ENTRY_Y(0xa01c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_cmd_po1            DRV_PINGROUP_ENTRY_Y(0xa02c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_clk_po0            DRV_PINGROUP_ENTRY_Y(0xa034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_shutdown_n                DRV_PINGROUP_ENTRY_Y(0x1004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pmu_int_n                 DRV_PINGROUP_ENTRY_Y(0x100c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_safe_state_pee0           DRV_PINGROUP_ENTRY_Y(0x1014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_vcomp_alert_pee1          DRV_PINGROUP_ENTRY_Y(0x101c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_soc_pwr_req               DRV_PINGROUP_ENTRY_Y(0x1024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_batt_oc_pee3              DRV_PINGROUP_ENTRY_Y(0x102c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_clk_32k_in                DRV_PINGROUP_ENTRY_Y(0x1034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_power_on_pee4             DRV_PINGROUP_ENTRY_Y(0x103c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pwr_i2c_scl_pee5          DRV_PINGROUP_ENTRY_Y(0x1044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pwr_i2c_sda_pee6          DRV_PINGROUP_ENTRY_Y(0x104c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_ao_retention_n_pee2       DRV_PINGROUP_ENTRY_Y(0x1064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpu_pwr_req_px0           DRV_PINGROUP_ENTRY_Y(0xD004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi3_miso_py1             DRV_PINGROUP_ENTRY_Y(0xD00c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi1_cs0_pz6              DRV_PINGROUP_ENTRY_Y(0xD014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi3_cs0_py3              DRV_PINGROUP_ENTRY_Y(0xD01c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi1_miso_pz4             DRV_PINGROUP_ENTRY_Y(0xD024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi3_cs1_py4              DRV_PINGROUP_ENTRY_Y(0xD02c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gp_pwm3_px3               DRV_PINGROUP_ENTRY_Y(0xD034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gp_pwm2_px2               DRV_PINGROUP_ENTRY_Y(0xD03c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi1_sck_pz3              DRV_PINGROUP_ENTRY_Y(0xD044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi3_sck_py0              DRV_PINGROUP_ENTRY_Y(0xD04c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi1_cs1_pz7              DRV_PINGROUP_ENTRY_Y(0xD054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi1_mosi_pz5             DRV_PINGROUP_ENTRY_Y(0xD05c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_spi3_mosi_py2             DRV_PINGROUP_ENTRY_Y(0xD064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cv_pwr_req_px1            DRV_PINGROUP_ENTRY_Y(0xD06c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_tx_px4              DRV_PINGROUP_ENTRY_Y(0xD074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_rx_px5              DRV_PINGROUP_ENTRY_Y(0xD07c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_rts_px6             DRV_PINGROUP_ENTRY_Y(0xD084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_cts_px7             DRV_PINGROUP_ENTRY_Y(0xD08c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_rx_py6              DRV_PINGROUP_ENTRY_Y(0xD094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_tx_py5              DRV_PINGROUP_ENTRY_Y(0xD09c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_rts_py7             DRV_PINGROUP_ENTRY_Y(0xD0a4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_cts_pz0             DRV_PINGROUP_ENTRY_Y(0xD0ac,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_usb_vbus_en0_pz1          DRV_PINGROUP_ENTRY_Y(0xD0b4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_usb_vbus_en1_pz2          DRV_PINGROUP_ENTRY_Y(0xD0bc,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_ufs0_rst_pff1             DRV_PINGROUP_ENTRY_Y(0x11004,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_ufs0_ref_clk_pff0         DRV_PINGROUP_ENTRY_Y(0x1100c,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)

#define drive_directdc_comp             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc1_comp               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_comp                 DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc3_comp               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_clk                DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_cmd                DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dqs                DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat7               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat6               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat5               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat4               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat3               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat2               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat1               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat0               DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi_comp                 DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_cs_n_pc7            DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_sck_pc6             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_io0_pd0             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_io1_pd1             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_io2_pd2             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi1_io3_pd3             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_io0_pc2             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_io1_pc3             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_io2_pc4             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_io3_pc5             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_cs_n_pc1            DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi0_sck_pc0             DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rx_ctl_pf2           DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_tx_ctl_pe5           DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rxc_pf3              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_txc_pe0              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_sma_mdc_pf5          DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_sma_mdio_pf4         DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rd0_pe6              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rd1_pe7              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rd2_pf0              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_rd3_pf1              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_td0_pe1              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_td1_pe2              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_td2_pe3              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_td3_pe4              DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out7_pw1        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out6_pw0        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out5_pv7        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out4_pv6        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out3_pv5        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out2_pv4        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out1_pv3        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_out0_pv2        DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_in_pv1          DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_directdc1_clk_pv0         DRV_PINGROUP_ENTRY_N(no_entry)

#define PINGROUP(pg_name, f0, f1, f2, f3, r, bank, pupd, e_io_hv, e_lpbk, e_input, e_lpdr, e_pbias_buf, \
			gpio_sfio_sel, e_od, schmitt_b, drvtype, epreemp, io_reset, rfu_in, io_rail)	\
	{							\
		.name = #pg_name,				\
		.pins = pg_name##_pins,				\
		.npins = ARRAY_SIZE(pg_name##_pins),		\
			.funcs = {				\
				TEGRA_MUX_##f0,			\
				TEGRA_MUX_##f1,			\
				TEGRA_MUX_##f2,			\
				TEGRA_MUX_##f3,			\
			},					\
		PIN_PINGROUP_ENTRY_Y(r, bank, pupd, e_io_hv, e_lpbk,	\
				     e_input, e_lpdr, e_pbias_buf, \
				     gpio_sfio_sel, e_od,	\
				     schmitt_b, drvtype,	\
				     epreemp, io_reset,		\
				     rfu_in, io_rail),		\
		drive_##pg_name,				\
	}

static const struct tegra_pingroup tegra194_groups[] = {

	PINGROUP(touch_clk_pcc4,	GP,		TOUCH,		RSVD2,		RSVD3,		0x2000,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(uart3_rx_pcc6,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2008,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(uart3_tx_pcc5,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2010,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(gen8_i2c_sda_pdd2,	I2C8,		RSVD1,		RSVD2,		RSVD3,		0x2018,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(gen8_i2c_scl_pdd1,	I2C8,		RSVD1,		RSVD2,		RSVD3,		0x2020,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(spi2_mosi_pcc2,	SPI2,		UARTG,		RSVD2,		RSVD3,		0x2028,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(gen2_i2c_scl_pcc7,	I2C2,		RSVD1,		RSVD2,		RSVD3,		0x2030,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(spi2_cs0_pcc3,		SPI2,		UARTG,		RSVD2,		RSVD3,		0x2038,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(gen2_i2c_sda_pdd0,	I2C2,		RSVD1,		RSVD2,		RSVD3,		0x2040,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(spi2_sck_pcc0,		SPI2,		UARTG,		RSVD2,		RSVD3,		0x2048,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(spi2_miso_pcc1,	SPI2,		UARTG,		RSVD2,		RSVD3,		0x2050,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_ao"),
	PINGROUP(can1_dout_paa0,	CAN1,		RSVD1,		RSVD2,		RSVD3,		0x3000,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can1_din_paa1,		CAN1,		RSVD1,		RSVD2,		RSVD3,		0x3008,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_dout_paa2,	CAN0,		RSVD1,		RSVD2,		RSVD3,		0x3010,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_din_paa3,		CAN0,		RSVD1,		RSVD2,		RSVD3,		0x3018,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_stb_paa4,		RSVD0,		WDT,		RSVD2,		RSVD3,		0x3020,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_en_paa5,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3028,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_wake_paa6,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3030,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can0_err_paa7,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3038,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can1_stb_pbb0,		RSVD0,		DMIC3,		DMIC5,		RSVD3,		0x3040,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can1_en_pbb1,		RSVD0,		DMIC3,		DMIC5,		RSVD3,		0x3048,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can1_wake_pbb2,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3050,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(can1_err_pbb3,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3058,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_ao_hv"),
	PINGROUP(soc_gpio33_pt0,	RSVD0,		SPDIF,		RSVD2,		RSVD3,		0x1000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(soc_gpio32_ps7,	RSVD0,		SPDIF,		RSVD2,		RSVD3,		0x1008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(soc_gpio31_ps6,	RSVD0,		SDMMC1,		RSVD2,		RSVD3,		0x1010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(soc_gpio30_ps5,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(aud_mclk_ps4,		AUD,		RSVD1,		RSVD2,		RSVD3,		0x1020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap1_fs_ps3,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap1_din_ps2,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap1_dout_ps1,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap1_sclk_ps0,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap3_fs_pt4,		I2S3,		DMIC2,		RSVD2,		RSVD3,		0x1048,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap3_din_pt3,		I2S3,		DMIC2,		RSVD2,		RSVD3,		0x1050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap3_dout_pt2,		I2S3,		DMIC1,		RSVD2,		RSVD3,		0x1058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap3_sclk_pt1,		I2S3,		DMIC1,		RSVD2,		RSVD3,		0x1060,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap5_fs_pu0,		I2S5,		DMIC4,		DSPK1,		RSVD3,		0x1068,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap5_din_pt7,		I2S5,		DMIC4,		DSPK1,		RSVD3,		0x1070,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap5_dout_pt6,		I2S5,		DSPK0,		RSVD2,		RSVD3,		0x1078,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap5_sclk_pt5,		I2S5,		DSPK0,		RSVD2,		RSVD3,		0x1080,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_audio"),
	PINGROUP(dap6_fs_pa3,		I2S6,		IQC1,		RSVD2,		RSVD3,		0x2000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap6_din_pa2,		I2S6,		IQC1,		RSVD2,		RSVD3,		0x2008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap6_dout_pa1,		I2S6,		IQC1,		RSVD2,		RSVD3,		0x2010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap6_sclk_pa0,		I2S6,		IQC1,		RSVD2,		RSVD3,		0x2018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap4_fs_pa7,		I2S4,		IQC2,		RSVD2,		RSVD3,		0x2020,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap4_din_pa6,		I2S4,		IQC2,		RSVD2,		RSVD3,		0x2028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap4_dout_pa5,		I2S4,		IQC2,		RSVD2,		RSVD3,		0x2030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(dap4_sclk_pa4,		I2S4,		IQC2,		RSVD2,		RSVD3,		0x2038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_audio_hv"),
	PINGROUP(extperiph2_clk_pp1,	EXTPERIPH2,	RSVD1,		RSVD2,		RSVD3,		0x0000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(extperiph1_clk_pp0,	EXTPERIPH1,	RSVD1,		RSVD2,		RSVD3,		0x0008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(cam_i2c_sda_pp3,	I2C3,		RSVD1,		RSVD2,		RSVD3,		0x0010,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(cam_i2c_scl_pp2,	I2C3,		RSVD1,		RSVD2,		RSVD3,		0x0018,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio40_pq4,	VGP1,		SLVS,		RSVD2,		RSVD3,		0x0020,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio41_pq5,	VGP2,		EXTPERIPH3,	RSVD2,		RSVD3,		0x0028,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio42_pq6,	VGP3,		EXTPERIPH4,	RSVD2,		RSVD3,		0x0030,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio43_pq7,	VGP4,		SLVS,		RSVD2,		RSVD3,		0x0038,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio44_pr0,	VGP5,		GP,		RSVD2,		RSVD3,		0x0040,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio45_pr1,	VGP6,		RSVD1,		RSVD2,		RSVD3,		0x0048,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio20_pq0,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x0050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio21_pq1,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x0058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio22_pq2,	RSVD0,		NV,		RSVD2,		RSVD3,		0x0060,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio23_pq3,	RSVD0,		WDT,		RSVD2,		RSVD3,		0x0068,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio04_pp4,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x0070,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio05_pp5,	RSVD0,		IGPU,		RSVD2,		RSVD3,		0x0078,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio06_pp6,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x0080,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(soc_gpio07_pp7,	RSVD0,		SATA,		SOC,		RSVD3,		0x0088,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(uart1_cts_pr5,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x0090,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(uart1_rts_pr4,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x0098,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(uart1_rx_pr3,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x00a0,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(uart1_tx_pr2,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x00a8,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_cam"),
	PINGROUP(dap2_din_pi1,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(dap2_dout_pi0,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(dap2_fs_pi2,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(dap2_sclk_ph7,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(uart4_cts_ph6,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(uart4_rts_ph5,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(uart4_rx_ph4,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(uart4_tx_ph3,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio03_pg3,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio02_pg2,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4048,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio01_pg1,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio00_pg0,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(gen1_i2c_scl_pi3,	I2C1,		RSVD1,		RSVD2,		RSVD3,		0x4060,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(gen1_i2c_sda_pi4,	I2C1,		RSVD1,		RSVD2,		RSVD3,		0x4068,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio08_pg4,	RSVD0,		CCLA,		RSVD2,		RSVD3,		0x4070,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio09_pg5,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4078,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio10_pg6,	GP,		RSVD1,		RSVD2,		RSVD3,		0x4080,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio11_pg7,	RSVD0,		SDMMC1,		RSVD2,		RSVD3,		0x4088,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio12_ph0,	RSVD0,		GP,		RSVD2,		RSVD3,		0x4090,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio13_ph1,	RSVD0,		GP,		RSVD2,		RSVD3,		0x4098,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(soc_gpio14_ph2,	RSVD0,		SDMMC1,		RSVD2,		RSVD3,		0x40a0,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_conn"),
	PINGROUP(directdc1_out7_pw1,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out6_pw0,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out5_pv7,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out4_pv6,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5020,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out3_pv5,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out2_pv4,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out1_pv3,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_out0_pv2,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5040,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_in_pv1,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5048,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc1_clk_pv0,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5050,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_debug"),
	PINGROUP(directdc_comp,		DIRECTDC,	RSVD1,		RSVD2,		RSVD3,		0x5058,		0,	N,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	Y,	"vddio_debug"),
	PINGROUP(soc_gpio50_pm5,	RSVD0,		DCA,		RSVD2,		RSVD3,		0x10000,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(soc_gpio51_pm6,	RSVD0,		DCA,		RSVD2,		RSVD3,		0x10008,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(soc_gpio52_pm7,	RSVD0,		DCB,		DGPU,		RSVD3,		0x10010,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(soc_gpio53_pn0,	RSVD0,		DCB,		RSVD2,		RSVD3,		0x10018,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(soc_gpio54_pn1,	RSVD0,		SDMMC3,		GP,		RSVD3,		0x10020,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(soc_gpio55_pn2,	RSVD0,		SDMMC3,		RSVD2,		RSVD3,		0x10028,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(dp_aux_ch0_hpd_pm0,	DP,		RSVD1,		RSVD2,		RSVD3,		0x10030,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(dp_aux_ch1_hpd_pm1,	DP,		RSVD1,		RSVD2,		RSVD3,		0x10038,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(dp_aux_ch2_hpd_pm2,	DP,		DISPLAYA,	RSVD2,		RSVD3,		0x10040,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(dp_aux_ch3_hpd_pm3,	DP,		DISPLAYB,	RSVD2,		RSVD3,		0x10048,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(hdmi_cec_pm4,		HDMI,		RSVD1,		RSVD2,		RSVD3,		0x10050,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_edp"),
	PINGROUP(eqos_td3_pe4,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15000,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_td2_pe3,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15008,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_td1_pe2,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15010,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_td0_pe1,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15018,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rd3_pf1,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15020,	0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rd2_pf0,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15028,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rd1_pe7,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15030,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_sma_mdio_pf4,	EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15038,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rd0_pe6,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15040,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_sma_mdc_pf5,	EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15048,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_comp,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15050,	0,	N,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	Y,	"vddio_eqos"),
	PINGROUP(eqos_txc_pe0,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15058,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rxc_pf3,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15060,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_tx_ctl_pe5,	EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15068,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(eqos_rx_ctl_pf2,	EQOS,		RSVD1,		RSVD2,		RSVD3,		0x15070,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_eqos"),
	PINGROUP(pex_l2_clkreq_n_pk4,	PE2,		RSVD1,		RSVD2,		RSVD3,		0x7000,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_wake_n_pl2,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x7008,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l1_clkreq_n_pk2,	PE1,		RSVD1,		RSVD2,		RSVD3,		0x7010,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l1_rst_n_pk3,	PE1,		RSVD1,		RSVD2,		RSVD3,		0x7018,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l0_clkreq_n_pk0,	PE0,		RSVD1,		RSVD2,		RSVD3,		0x7020,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l0_rst_n_pk1,	PE0,		RSVD1,		RSVD2,		RSVD3,		0x7028,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l2_rst_n_pk5,	PE2,		RSVD1,		RSVD2,		RSVD3,		0x7030,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l3_clkreq_n_pk6,	PE3,		RSVD1,		RSVD2,		RSVD3,		0x7038,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l3_rst_n_pk7,	PE3,		RSVD1,		RSVD2,		RSVD3,		0x7040,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l4_clkreq_n_pl0,	PE4,		RSVD1,		RSVD2,		RSVD3,		0x7048,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l4_rst_n_pl1,	PE4,		RSVD1,		RSVD2,		RSVD3,		0x7050,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(sata_dev_slp_pl3,	SATA,		RSVD1,		RSVD2,		RSVD3,		0x7058,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl"),
	PINGROUP(pex_l5_clkreq_n_pgg0,	PE5,		RSVD1,		RSVD2,		RSVD3,		0x14000,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl_2"),
	PINGROUP(pex_l5_rst_n_pgg1,	PE5,		RSVD1,		RSVD2,		RSVD3,		0x14008,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pex_ctl_2"),
	PINGROUP(cpu_pwr_req_1_pb1,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x16000,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pwr_ctl"),
	PINGROUP(cpu_pwr_req_0_pb0,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x16008,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_pwr_ctl"),
	PINGROUP(qspi0_io3_pc5,		QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi0_io2_pc4,		QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi0_io1_pc3,		QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi0_io0_pc2,		QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi0_sck_pc0,		QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB020,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi0_cs_n_pc1,	QSPI0,		RSVD1,		RSVD2,		RSVD3,		0xB028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_io3_pd3,		QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_io2_pd2,		QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_io1_pd1,		QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB040,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_io0_pd0,		QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB048,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_sck_pc6,		QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB050,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi1_cs_n_pc7,	QSPI1,		RSVD1,		RSVD2,		RSVD3,		0xB058,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_qspi"),
	PINGROUP(qspi_comp,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB060,		0,	N,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	Y,	"vddio_qspi"),
	PINGROUP(sdmmc1_clk_pj0,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8000,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_cmd_pj1,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_comp,		SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8010,		0,	N,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_dat3_pj5,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_dat2_pj4,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8020,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_dat1_pj3,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc1_dat0_pj2,	SDMMC1,		RSVD1,		MIPI,		RSVD3,		0x8030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc1_hv"),
	PINGROUP(sdmmc3_dat3_po5,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_dat2_po4,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_dat1_po3,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_dat0_po2,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_comp,		SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA020,		0,	N,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_cmd_po1,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc3_clk_po0,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xA030,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y,	"vddio_sdmmc3_hv"),
	PINGROUP(sdmmc4_clk,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6008,		0,	Y,	-1,	5,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_cmd,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6010,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dqs,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6018,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat7,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6020,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat6,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6028,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat5,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6030,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat4,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6038,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat3,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6040,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat2,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6048,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat1,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6050,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(sdmmc4_dat0,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6058,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	N,	"vddio_sdmmc4"),
	PINGROUP(shutdown_n,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1000,		1,	Y,	5,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(pmu_int_n,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1008,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(safe_state_pee0,	SCE,		RSVD1,		RSVD2,		RSVD3,		0x1010,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(vcomp_alert_pee1,	SOC,		RSVD1,		RSVD2,		RSVD3,		0x1018,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(soc_pwr_req,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1020,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(batt_oc_pee3,		SOC,		RSVD1,		RSVD2,		RSVD3,		0x1028,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(clk_32k_in,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1030,		1,	Y,	-1,	-1,	-1,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(power_on_pee4,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1038,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(pwr_i2c_scl_pee5,	I2C5,		RSVD1,		RSVD2,		RSVD3,		0x1040,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(pwr_i2c_sda_pee6,	I2C5,		RSVD1,		RSVD2,		RSVD3,		0x1048,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(ao_retention_n_pee2,	GPIO,		RSVD1,		RSVD2,		RSVD3,		0x1060,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_sys"),
	PINGROUP(gpu_pwr_req_px0,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xD000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi3_miso_py1,		SPI3,		RSVD1,		RSVD2,		RSVD3,		0xD008,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi1_cs0_pz6,		SPI1,		RSVD1,		RSVD2,		RSVD3,		0xD010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi3_cs0_py3,		SPI3,		RSVD1,		RSVD2,		RSVD3,		0xD018,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi1_miso_pz4,		SPI1,		RSVD1,		RSVD2,		RSVD3,		0xD020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi3_cs1_py4,		SPI3,		RSVD1,		RSVD2,		RSVD3,		0xD028,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(gp_pwm3_px3,		GP,		RSVD1,		RSVD2,		RSVD3,		0xD030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(gp_pwm2_px2,		GP,		RSVD1,		RSVD2,		RSVD3,		0xD038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi1_sck_pz3,		SPI1,		RSVD1,		RSVD2,		RSVD3,		0xD040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi3_sck_py0,		SPI3,		RSVD1,		RSVD2,		RSVD3,		0xD048,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi1_cs1_pz7,		SPI1,		RSVD1,		RSVD2,		RSVD3,		0xD050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi1_mosi_pz5,		SPI1,		RSVD1,		RSVD2,		RSVD3,		0xD058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(spi3_mosi_py2,		SPI3,		RSVD1,		RSVD2,		RSVD3,		0xD060,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(cv_pwr_req_px1,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xD068,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart2_tx_px4,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xD070,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart2_rx_px5,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xD078,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart2_rts_px6,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xD080,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart2_cts_px7,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xD088,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart5_rx_py6,		UARTE,		RSVD1,		RSVD2,		RSVD3,		0xD090,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart5_tx_py5,		UARTE,		RSVD1,		RSVD2,		RSVD3,		0xD098,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart5_rts_py7,		UARTE,		RSVD1,		RSVD2,		RSVD3,		0xD0a0,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(uart5_cts_pz0,		UARTE,		RSVD1,		RSVD2,		RSVD3,		0xD0a8,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(usb_vbus_en0_pz1,	USB,		RSVD1,		RSVD2,		RSVD3,		0xD0b0,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(usb_vbus_en1_pz2,	USB,		RSVD1,		RSVD2,		RSVD3,		0xD0b8,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N,	"vddio_uart"),
	PINGROUP(ufs0_rst_pff1,		UFS0,		RSVD1,		RSVD2,		RSVD3,		0x11000,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_ufs"),
	PINGROUP(ufs0_ref_clk_pff0,	UFS0,		RSVD1,		RSVD2,		RSVD3,		0x11008,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y,	"vddio_ufs"),
};

static const struct tegra_pinctrl_soc_data tegra194_pinctrl = {
	.ngpios = NUM_GPIOS,
	.pins = tegra194_pins,
	.npins = ARRAY_SIZE(tegra194_pins),
	.functions = tegra194_functions,
	.nfunctions = ARRAY_SIZE(tegra194_functions),
	.groups = tegra194_groups,
	.ngroups = ARRAY_SIZE(tegra194_groups),
	.is_gpio_reg_support = true,
	.input_tristate_enable = true,
	.output_input_disable = true,
	.hsm_in_mux = false,
	.schmitt_in_mux = true,
	.drvtype_in_mux = true,
};

static int tegra194_pinctrl_probe(struct platform_device *pdev)
{
	return tegra_pinctrl_probe(pdev, &tegra194_pinctrl);
}

static struct of_device_id tegra194_pinctrl_of_match[] = {
	{ .compatible = "nvidia,tegra194-pinmux", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra194_pinctrl_of_match);

static struct platform_driver tegra194_pinctrl_driver = {
	.driver = {
		.name = "tegra194-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = tegra194_pinctrl_of_match,
	},
	.probe = tegra194_pinctrl_probe,
};

static int __init tegra194_pinctrl_init(void)
{
	return platform_driver_register(&tegra194_pinctrl_driver);
}
postcore_initcall_sync(tegra194_pinctrl_init);

MODULE_AUTHOR("Suresh Mangipudi <smangipudi@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra194 pinctrl driver");
MODULE_LICENSE("GPL v2");
