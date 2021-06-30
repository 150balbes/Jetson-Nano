/*
 * Pinctrl data for the NVIDIA Tegra186 pinmux
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#define T186_PIN_TABLE(fname)						\
	fname(PEX_L0_RST_N_PA0, pex_l0_rst_n_pa0, _GPIO, 0)		\
	fname(PEX_L0_CLKREQ_N_PA1, pex_l0_clkreq_n_pa1, _GPIO, 1)	\
	fname(PEX_WAKE_N_PA2, pex_wake_n_pa2, _GPIO, 2)			\
	fname(PEX_L1_RST_N_PA3, pex_l1_rst_n_pa3, _GPIO, 3)		\
	fname(PEX_L1_CLKREQ_N_PA4, pex_l1_clkreq_n_pa4, _GPIO, 4)	\
	fname(PEX_L2_RST_N_PA5, pex_l2_rst_n_pa5, _GPIO, 5)		\
	fname(PEX_L2_CLKREQ_N_PA6, pex_l2_clkreq_n_pa6, _GPIO, 6)	\
	fname(UART4_TX_PB0, uart4_tx_pb0, _GPIO, 8)			\
	fname(UART4_RX_PB1, uart4_rx_pb1, _GPIO, 9)			\
	fname(UART4_RTS_PB2, uart4_rts_pb2, _GPIO, 10)			\
	fname(UART4_CTS_PB3, uart4_cts_pb3, _GPIO, 11)			\
	fname(GPIO_WAN1_PB4, gpio_wan1_pb4, _GPIO, 12)			\
	fname(GPIO_WAN2_PB5, gpio_wan2_pb5, _GPIO, 13)			\
	fname(GPIO_WAN3_PB6, gpio_wan3_pb6, _GPIO, 14)			\
	fname(GPIO_WAN4_PC0, gpio_wan4_pc0, _GPIO, 16)			\
	fname(DAP2_SCLK_PC1, dap2_sclk_pc1, _GPIO, 17)			\
	fname(DAP2_DOUT_PC2, dap2_dout_pc2, _GPIO, 18)			\
	fname(DAP2_DIN_PC3, dap2_din_pc3, _GPIO, 19)			\
	fname(DAP2_FS_PC4, dap2_fs_pc4, _GPIO, 20)			\
	fname(GEN1_I2C_SCL_PC5, gen1_i2c_scl_pc5, _GPIO, 21)		\
	fname(GEN1_I2C_SDA_PC6, gen1_i2c_sda_pc6, _GPIO, 22)		\
	fname(SDMMC1_CLK_PD0, sdmmc1_clk_pd0, _GPIO, 24)		\
	fname(SDMMC1_CMD_PD1, sdmmc1_cmd_pd1, _GPIO, 25)		\
	fname(SDMMC1_DAT0_PD2, sdmmc1_dat0_pd2, _GPIO, 26)		\
	fname(SDMMC1_DAT1_PD3, sdmmc1_dat1_pd3, _GPIO, 27)		\
	fname(SDMMC1_DAT2_PD4, sdmmc1_dat2_pd4, _GPIO, 28)		\
	fname(SDMMC1_DAT3_PD5, sdmmc1_dat3_pd5, _GPIO, 29)		\
	fname(EQOS_TXC_PE0, eqos_txc_pe0, _GPIO, 32)			\
	fname(EQOS_TD0_PE1, eqos_td0_pe1, _GPIO, 33)			\
	fname(EQOS_TD1_PE2, eqos_td1_pe2, _GPIO, 34)			\
	fname(EQOS_TD2_PE3, eqos_td2_pe3, _GPIO, 35)			\
	fname(EQOS_TD3_PE4, eqos_td3_pe4, _GPIO, 36)			\
	fname(EQOS_TX_CTL_PE5, eqos_tx_ctl_pe5, _GPIO, 37)		\
	fname(EQOS_RD0_PE6, eqos_rd0_pe6, _GPIO, 38)			\
	fname(EQOS_RD1_PE7, eqos_rd1_pe7, _GPIO, 39)			\
	fname(EQOS_RD2_PF0, eqos_rd2_pf0, _GPIO, 40)			\
	fname(EQOS_RD3_PF1, eqos_rd3_pf1, _GPIO, 41)			\
	fname(EQOS_RX_CTL_PF2, eqos_rx_ctl_pf2, _GPIO, 42)		\
	fname(EQOS_RXC_PF3, eqos_rxc_pf3, _GPIO, 43)			\
	fname(EQOS_MDIO_PF4, eqos_mdio_pf4, _GPIO, 44)			\
	fname(EQOS_MDC_PF5, eqos_mdc_pf5, _GPIO, 45)			\
	fname(SDMMC3_CLK_PG0, sdmmc3_clk_pg0, _GPIO, 48)		\
	fname(SDMMC3_CMD_PG1, sdmmc3_cmd_pg1, _GPIO, 49)		\
	fname(SDMMC3_DAT0_PG2, sdmmc3_dat0_pg2, _GPIO, 50)		\
	fname(SDMMC3_DAT1_PG3, sdmmc3_dat1_pg3, _GPIO, 51)		\
	fname(SDMMC3_DAT2_PG4, sdmmc3_dat2_pg4, _GPIO, 52)		\
	fname(SDMMC3_DAT3_PG5, sdmmc3_dat3_pg5, _GPIO, 53)		\
	fname(GPIO_WAN5_PH0, gpio_wan5_ph0, _GPIO, 56)			\
	fname(GPIO_WAN6_PH1, gpio_wan6_ph1, _GPIO, 57)			\
	fname(GPIO_WAN7_PH2, gpio_wan7_ph2, _GPIO, 58)			\
	fname(GPIO_WAN8_PH3, gpio_wan8_ph3, _GPIO, 59)			\
	fname(BCPU_PWR_REQ_PH4, bcpu_pwr_req_ph4, _GPIO, 60)		\
	fname(MCPU_PWR_REQ_PH5, mcpu_pwr_req_ph5, _GPIO, 61)		\
	fname(GPU_PWR_REQ_PH6, gpu_pwr_req_ph6, _GPIO, 62)		\
	fname(GPIO_PQ0_PI0, gpio_pq0_pi0, _GPIO, 64)			\
	fname(GPIO_PQ1_PI1, gpio_pq1_pi1, _GPIO, 65)			\
	fname(GPIO_PQ2_PI2, gpio_pq2_pi2, _GPIO, 66)			\
	fname(GPIO_PQ3_PI3, gpio_pq3_pi3, _GPIO, 67)			\
	fname(GPIO_PQ4_PI4, gpio_pq4_pi4, _GPIO, 68)			\
	fname(GPIO_PQ5_PI5, gpio_pq5_pi5, _GPIO, 69)			\
	fname(GPIO_PQ6_PI6, gpio_pq6_pi6, _GPIO, 70)			\
	fname(GPIO_PQ7_PI7, gpio_pq7_pi7, _GPIO, 71)			\
	fname(DAP1_SCLK_PJ0, dap1_sclk_pj0, _GPIO, 72)			\
	fname(DAP1_DOUT_PJ1, dap1_dout_pj1, _GPIO, 73)			\
	fname(DAP1_DIN_PJ2, dap1_din_pj2, _GPIO, 74)			\
	fname(DAP1_FS_PJ3, dap1_fs_pj3, _GPIO, 75)			\
	fname(AUD_MCLK_PJ4, aud_mclk_pj4, _GPIO, 76)			\
	fname(GPIO_AUD0_PJ5, gpio_aud0_pj5, _GPIO, 77)			\
	fname(GPIO_AUD1_PJ6, gpio_aud1_pj6, _GPIO, 78)			\
	fname(GPIO_AUD2_PJ7, gpio_aud2_pj7, _GPIO, 79)			\
	fname(GPIO_AUD3_PK0, gpio_aud3_pk0, _GPIO, 80)			\
	fname(GEN7_I2C_SCL_PL0, gen7_i2c_scl_pl0, _GPIO, 88)		\
	fname(GEN7_I2C_SDA_PL1, gen7_i2c_sda_pl1, _GPIO, 89)		\
	fname(GEN9_I2C_SCL_PL2, gen9_i2c_scl_pl2, _GPIO, 90)		\
	fname(GEN9_I2C_SDA_PL3, gen9_i2c_sda_pl3, _GPIO, 91)		\
	fname(USB_VBUS_EN0_PL4, usb_vbus_en0_pl4, _GPIO, 92)		\
	fname(USB_VBUS_EN1_PL5, usb_vbus_en1_pl5, _GPIO, 93)		\
	fname(GP_PWM6_PL6, gp_pwm6_pl6, _GPIO, 94)			\
	fname(GP_PWM7_PL7, gp_pwm7_pl7, _GPIO, 95)			\
	fname(DMIC1_DAT_PM0, dmic1_dat_pm0, _GPIO, 96)			\
	fname(DMIC1_CLK_PM1, dmic1_clk_pm1, _GPIO, 97)			\
	fname(DMIC2_DAT_PM2, dmic2_dat_pm2, _GPIO, 98)			\
	fname(DMIC2_CLK_PM3, dmic2_clk_pm3, _GPIO, 99)			\
	fname(DMIC4_DAT_PM4, dmic4_dat_pm4, _GPIO, 100)			\
	fname(DMIC4_CLK_PM5, dmic4_clk_pm5, _GPIO, 101)			\
	fname(GPIO_CAM1_PN0, gpio_cam1_pn0, _GPIO, 104)			\
	fname(GPIO_CAM2_PN1, gpio_cam2_pn1, _GPIO, 105)			\
	fname(GPIO_CAM3_PN2, gpio_cam3_pn2, _GPIO, 106)			\
	fname(GPIO_CAM4_PN3, gpio_cam4_pn3, _GPIO, 107)			\
	fname(GPIO_CAM5_PN4, gpio_cam5_pn4, _GPIO, 108)			\
	fname(GPIO_CAM6_PN5, gpio_cam6_pn5, _GPIO, 109)			\
	fname(GPIO_CAM7_PN6, gpio_cam7_pn6, _GPIO, 110)			\
	fname(EXTPERIPH1_CLK_PO0, extperiph1_clk_po0, _GPIO, 112)	\
	fname(EXTPERIPH2_CLK_PO1, extperiph2_clk_po1, _GPIO, 113)	\
	fname(CAM_I2C_SCL_PO2, cam_i2c_scl_po2, _GPIO, 114)		\
	fname(CAM_I2C_SDA_PO3, cam_i2c_sda_po3, _GPIO, 115)		\
	fname(DP_AUX_CH0_HPD_PP0, dp_aux_ch0_hpd_pp0, _GPIO, 120)	\
	fname(DP_AUX_CH1_HPD_PP1, dp_aux_ch1_hpd_pp1, _GPIO, 121)	\
	fname(HDMI_CEC_PP2, hdmi_cec_pp2, _GPIO, 122)			\
	fname(GPIO_EDP0_PP3, gpio_edp0_pp3, _GPIO, 123)			\
	fname(GPIO_EDP1_PP4, gpio_edp1_pp4, _GPIO, 124)			\
	fname(GPIO_EDP2_PP5, gpio_edp2_pp5, _GPIO, 125)			\
	fname(GPIO_EDP3_PP6, gpio_edp3_pp6, _GPIO, 126)			\
	fname(DIRECTDC1_CLK_PQ0, directdc1_clk_pq0, _GPIO, 128)		\
	fname(DIRECTDC1_IN_PQ1, directdc1_in_pq1, _GPIO, 129)		\
	fname(DIRECTDC1_OUT0_PQ2, directdc1_out0_pq2, _GPIO, 130)	\
	fname(DIRECTDC1_OUT1_PQ3, directdc1_out1_pq3, _GPIO, 131)	\
	fname(DIRECTDC1_OUT2_PQ4, directdc1_out2_pq4, _GPIO, 132)	\
	fname(DIRECTDC1_OUT3_PQ5, directdc1_out3_pq5, _GPIO, 133)	\
	fname(QSPI_SCK_PR0, qspi_sck_pr0, _GPIO, 136)			\
	fname(QSPI_IO0_PR1, qspi_io0_pr1, _GPIO, 137)			\
	fname(QSPI_IO1_PR2, qspi_io1_pr2, _GPIO, 138)			\
	fname(QSPI_IO2_PR3, qspi_io2_pr3, _GPIO, 139)			\
	fname(QSPI_IO3_PR4, qspi_io3_pr4, _GPIO, 140)			\
	fname(QSPI_CS_N_PR5, qspi_cs_n_pr5, _GPIO, 141)			\
	fname(PWR_I2C_SCL_PS0, pwr_i2c_scl_ps0, _GPIO, 144)		\
	fname(PWR_I2C_SDA_PS1, pwr_i2c_sda_ps1, _GPIO, 145)		\
	fname(BATT_OC_PS2, batt_oc_ps2, _GPIO, 146)			\
	fname(SAFE_STATE_PS3, safe_state_ps3, _GPIO, 147)		\
	fname(VCOMP_ALERT_PS4, vcomp_alert_ps4, _GPIO, 148)		\
	fname(UART1_TX_PT0, uart1_tx_pt0, _GPIO, 152)			\
	fname(UART1_RX_PT1, uart1_rx_pt1, _GPIO, 153)			\
	fname(UART1_RTS_PT2, uart1_rts_pt2, _GPIO, 154)			\
	fname(UART1_CTS_PT3, uart1_cts_pt3, _GPIO, 155)			\
	fname(GPIO_DIS0_PU0, gpio_dis0_pu0, _GPIO, 160)			\
	fname(GPIO_DIS1_PU1, gpio_dis1_pu1, _GPIO, 161)			\
	fname(GPIO_DIS2_PU2, gpio_dis2_pu2, _GPIO, 162)			\
	fname(GPIO_DIS3_PU3, gpio_dis3_pu3, _GPIO, 163)			\
	fname(GPIO_DIS4_PU4, gpio_dis4_pu4, _GPIO, 164)			\
	fname(GPIO_DIS5_PU5, gpio_dis5_pu5, _GPIO, 165)			\
	fname(GPIO_SEN0_PV0, gpio_sen0_pv0, _GPIO, 168)			\
	fname(GPIO_SEN1_PV1, gpio_sen1_pv1, _GPIO, 169)			\
	fname(GPIO_SEN2_PV2, gpio_sen2_pv2, _GPIO, 170)			\
	fname(GPIO_SEN3_PV3, gpio_sen3_pv3, _GPIO, 171)			\
	fname(GPIO_SEN4_PV4, gpio_sen4_pv4, _GPIO, 172)			\
	fname(GPIO_SEN5_PV5, gpio_sen5_pv5, _GPIO, 173)			\
	fname(GPIO_SEN6_PV6, gpio_sen6_pv6, _GPIO, 174)			\
	fname(GPIO_SEN7_PV7, gpio_sen7_pv7, _GPIO, 175)			\
	fname(GEN8_I2C_SCL_PW0, gen8_i2c_scl_pw0, _GPIO, 176)		\
	fname(GEN8_I2C_SDA_PW1, gen8_i2c_sda_pw1, _GPIO, 177)		\
	fname(UART3_TX_PW2, uart3_tx_pw2, _GPIO, 178)			\
	fname(UART3_RX_PW3, uart3_rx_pw3, _GPIO, 179)			\
	fname(UART3_RTS_PW4, uart3_rts_pw4, _GPIO, 180)			\
	fname(UART3_CTS_PW5, uart3_cts_pw5, _GPIO, 181)			\
	fname(UART7_TX_PW6, uart7_tx_pw6, _GPIO, 182)			\
	fname(UART7_RX_PW7, uart7_rx_pw7, _GPIO, 183)			\
	fname(UART2_TX_PX0, uart2_tx_px0, _GPIO, 184)			\
	fname(UART2_RX_PX1, uart2_rx_px1, _GPIO, 185)			\
	fname(UART2_RTS_PX2, uart2_rts_px2, _GPIO, 186)			\
	fname(UART2_CTS_PX3, uart2_cts_px3, _GPIO, 187)			\
	fname(UART5_TX_PX4, uart5_tx_px4, _GPIO, 188)			\
	fname(UART5_RX_PX5, uart5_rx_px5, _GPIO, 189)			\
	fname(UART5_RTS_PX6, uart5_rts_px6, _GPIO, 190)			\
	fname(UART5_CTS_PX7, uart5_cts_px7, _GPIO, 191)			\
	fname(GPIO_MDM1_PY0, gpio_mdm1_py0, _GPIO, 192)			\
	fname(GPIO_MDM2_PY1, gpio_mdm2_py1, _GPIO, 193)			\
	fname(GPIO_MDM3_PY2, gpio_mdm3_py2, _GPIO, 194)			\
	fname(GPIO_MDM4_PY3, gpio_mdm4_py3, _GPIO, 195)			\
	fname(GPIO_MDM5_PY4, gpio_mdm5_py4, _GPIO, 196)			\
	fname(GPIO_MDM6_PY5, gpio_mdm6_py5, _GPIO, 197)			\
	fname(GPIO_MDM7_PY6, gpio_mdm7_py6, _GPIO, 198)			\
	fname(CAN1_DOUT_PZ0, can1_dout_pz0, _GPIO, 200)			\
	fname(CAN1_DIN_PZ1, can1_din_pz1, _GPIO, 201)			\
	fname(CAN0_DOUT_PZ2, can0_dout_pz2, _GPIO, 202)			\
	fname(CAN0_DIN_PZ3, can0_din_pz3, _GPIO, 203)			\
	fname(CAN_GPIO0_PAA0, can_gpio0_paa0, _GPIO, 208)		\
	fname(CAN_GPIO1_PAA1, can_gpio1_paa1, _GPIO, 209)		\
	fname(CAN_GPIO2_PAA2, can_gpio2_paa2, _GPIO, 210)		\
	fname(CAN_GPIO3_PAA3, can_gpio3_paa3, _GPIO, 211)		\
	fname(CAN_GPIO4_PAA4, can_gpio4_paa4, _GPIO, 212)		\
	fname(CAN_GPIO5_PAA5, can_gpio5_paa5, _GPIO, 213)		\
	fname(CAN_GPIO6_PAA6, can_gpio6_paa6, _GPIO, 214)		\
	fname(CAN_GPIO7_PAA7, can_gpio7_paa7, _GPIO, 215)		\
	fname(UFS0_REF_CLK_PBB0, ufs0_ref_clk_pbb0, _GPIO, 216)		\
	fname(UFS0_RST_PBB1, ufs0_rst_pbb1, _GPIO, 217)			\
	fname(DAP4_SCLK_PCC0, dap4_sclk_pcc0, _GPIO, 224)		\
	fname(DAP4_DOUT_PCC1, dap4_dout_pcc1, _GPIO, 225)		\
	fname(DAP4_DIN_PCC2, dap4_din_pcc2, _GPIO, 226)			\
	fname(DAP4_FS_PCC3, dap4_fs_pcc3, _GPIO, 227)			\
	fname(GPIO_SEN8_PEE0, gpio_sen8_pee0, _GPIO, 240)		\
	fname(GPIO_SEN9_PEE1, gpio_sen9_pee1, _GPIO, 241)		\
	fname(TOUCH_CLK_PEE2, touch_clk_pee2, _GPIO, 242)		\
	fname(POWER_ON_PFF0, power_on_pff0, _GPIO, 248)			\
	fname(GPIO_SW1_PFF1, gpio_sw1_pff1, _GPIO, 249)			\
	fname(GPIO_SW2_PFF2, gpio_sw2_pff2, _GPIO, 250)			\
	fname(GPIO_SW3_PFF3, gpio_sw3_pff3, _GPIO, 251)			\
	fname(GPIO_SW4_PFF4, gpio_sw4_pff4, _GPIO, 252)			\
	fname(DIRECTDC_COMP, directdc_comp, _PIN, 0)			\
	fname(SDMMC1_COMP, sdmmc1_comp, _PIN, 1)			\
	fname(EQOS_COMP, eqos_comp, _PIN, 2)				\
	fname(SDMMC3_COMP, sdmmc3_comp, _PIN, 3)			\
	fname(QSPI_COMP, qspi_comp, _PIN, 4)				\
	fname(SHUTDOWN, shutdown, _PIN, 5)				\
	fname(PMU_INT, pmu_int, _PIN, 6)				\
	fname(SOC_PWR_REQ, soc_pwr_req, _PIN, 7)			\
	fname(CLK_32K_IN, clk_32k_in, _PIN, 8)				\


/**** Output **/
#define _GPIO(offset)			(offset)
#define NUM_GPIOS			(TEGRA_PIN_GPIO_SW4_PFF4 + 1)
#define _PIN(offset)			(NUM_GPIOS + (offset))

/* Define unique ID for each pins */
#define TEGRA_PINCTRL_PIN_NUM(id, lid, _f, num)		\
	TEGRA_PIN_##id = _f(num),
enum pin_id {
	T186_PIN_TABLE(TEGRA_PINCTRL_PIN_NUM)
};

/* Table for pin descriptr */
#define TEGRA_PINCTRL_PIN(id, lid, f, num)	PINCTRL_PIN(TEGRA_PIN_##id, #id),
static const struct pinctrl_pin_desc tegra186_pins[] = {
	T186_PIN_TABLE(TEGRA_PINCTRL_PIN)
};

/* Generate pins array for each pin */
#define TEGRA_PINCTRL_PINS_STRUCT(id, lid, f, num)		\
	static const unsigned lid##_pins[] = {		\
		TEGRA_PIN_##id,				\
	};
T186_PIN_TABLE(TEGRA_PINCTRL_PINS_STRUCT)

static const unsigned sdmmc4_clk_pins[] = {};

static const unsigned sdmmc4_cmd_pins[] = {};

static const unsigned sdmmc4_dqs_pins[] = {};

static const unsigned sdmmc4_dat7_pins[] = {};

static const unsigned sdmmc4_dat6_pins[] = {};

static const unsigned sdmmc4_dat5_pins[] = {};

static const unsigned sdmmc4_dat4_pins[] = {};

static const unsigned sdmmc4_dat3_pins[] = {};

static const unsigned sdmmc4_dat2_pins[] = {};

static const unsigned sdmmc4_dat1_pins[] = {};

static const unsigned sdmmc4_dat0_pins[] = {};

#define T186_FUNCTION_TABLE(fname)		\
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
	fname(EXTPERIPH3, extperiph3)		\
	fname(EXTPERIPH4, extperiph4)		\
	fname(SPI4, spi4)			\
	fname(I2S2, i2s2)			\
	fname(UARTD, uartd)			\
	fname(I2C1, i2c1)			\
	fname(UARTA, uarta)			\
	fname(DIRECTDC1, directdc1)		\
	fname(DIRECTDC, directdc)		\
	fname(IQC0, iqc0)			\
	fname(IQC1, iqc1)			\
	fname(I2S6, i2s6)			\
	fname(DTV, dtv)				\
	fname(UARTF, uartf)			\
	fname(SDMMC3, sdmmc3)			\
	fname(SDMMC4, sdmmc4)			\
	fname(SDMMC1, sdmmc1)			\
	fname(DP, dp)				\
	fname(HDMI, hdmi)			\
	fname(PE2, pe2)				\
	fname(SATA, sata)			\
	fname(PE, pe)				\
	fname(PE1, pe1)				\
	fname(PE0, pe0)				\
	fname(SOC, soc)				\
	fname(EQOS, eqos)			\
	fname(SDMMC2, sdmmc2)			\
	fname(QSPI, qspi)			\
	fname(SCE, sce)				\
	fname(I2C5, i2c5)			\
	fname(DISPLAYA, displaya)		\
	fname(DISPLAYB, displayb)		\
	fname(DCC, dcc)				\
	fname(DCB, dcb)				\
	fname(SPI1, spi1)			\
	fname(UARTB, uartb)			\
	fname(UARTE, uarte)			\
	fname(SPI3, spi3)			\
	fname(NV, nv)				\
	fname(CCLA, ccla)			\
	fname(I2C7, i2c7)			\
	fname(I2C9, i2c9)			\
	fname(I2S5, i2s5)			\
	fname(USB, usb)				\
	fname(UFS0, ufs0)

/* Define unique ID for each function */
#define TEGRA_PIN_FUNCTION_MUX_ENUM(id, lid)         \
        TEGRA_MUX_##id,
enum tegra_mux_dt {
        T186_FUNCTION_TABLE(TEGRA_PIN_FUNCTION_MUX_ENUM)
};

/* Make list of each function name */
#define TEGRA_PIN_FUNCTION(id, lid)			\
	{						\
		.name = #lid,				\
	},
static struct tegra_function tegra186_functions[] = {
	T186_FUNCTION_TABLE(TEGRA_PIN_FUNCTION)
};

#define PINGROUP_REG_Y(r) ((r))
#define PINGROUP_REG_N(r) -1


#define DRV_PINGROUP_Y(r) ((r))
#define DRV_PINGROUP_N(r) -1

#define DRV_PINGROUP_ENTRY_N(pg_name)				\
		.drv_reg = -1,					\
		.drv_bank = -1,					\
		.drvdn_bit = -1,				\
		.drvdn_width = -1,				\
		.drvup_bit = -1,				\
		.drvup_width = -1,				\
		.slwr_bit = -1,					\
		.slwr_width = -1,				\
		.slwf_bit = -1,					\
		.slwf_width = -1

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


#define PIN_PINGROUP_ENTRY_Y(r, bank, pupd, e_io_hv, e_lpbk, e_input,	\
			     e_lpdr, e_pbias_buf, gpio_sfio_sel, \
			     e_od, schmitt_b, drvtype, epreemp,	\
			     io_reset, rfu_in)			\
		.mux_reg = PINGROUP_REG_Y(r), 			\
		.lpmd_bit = -1,					\
		.lock_bit = -1,					\
		.hsm_bit = -1,					\
		.parked_bit = -1,				\
		.pad_bit = -1,					\
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
		.pbias_buf_bit = e_io_hv,			\
		.preemp_bit = e_io_hv,				\
		.rfu_in_bit = 20,				\
		.lpbk_reg = PINGROUP_REG_Y(r),			\
		.lpbk_bank = bank,				\
		.lpbk_bit = e_lpbk				\

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
		.pad_bit = -1,					\
		.lpmd_bit = -1,					\
		.drvtype_bit = -1,				\
		.lpdr_bit = -1,					\
		.pbias_buf_bit = -1,				\
		.preemp_bit = -1,				\
		.rfu_in_bit = -1,				\
		.lpbk_bit = -1,					\

#define drive_touch_clk_pee2            DRV_PINGROUP_ENTRY_Y(0x2004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_cts_pw5             DRV_PINGROUP_ENTRY_Y(0x200c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_rts_pw4             DRV_PINGROUP_ENTRY_Y(0x2014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_rx_pw3              DRV_PINGROUP_ENTRY_Y(0x201c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart3_tx_pw2              DRV_PINGROUP_ENTRY_Y(0x2024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen8_i2c_sda_pw1          DRV_PINGROUP_ENTRY_Y(0x202c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gen8_i2c_scl_pw0          DRV_PINGROUP_ENTRY_Y(0x2034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart7_rx_pw7              DRV_PINGROUP_ENTRY_Y(0x203c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_uart7_tx_pw6              DRV_PINGROUP_ENTRY_Y(0x2044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen0_pv0             DRV_PINGROUP_ENTRY_Y(0x204c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen1_pv1             DRV_PINGROUP_ENTRY_Y(0x2054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen2_pv2             DRV_PINGROUP_ENTRY_Y(0x205c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen3_pv3             DRV_PINGROUP_ENTRY_Y(0x2064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen4_pv4             DRV_PINGROUP_ENTRY_Y(0x206c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen5_pv5             DRV_PINGROUP_ENTRY_Y(0x2074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen6_pv6             DRV_PINGROUP_ENTRY_Y(0x207c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen7_pv7             DRV_PINGROUP_ENTRY_Y(0x2084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen8_pee0            DRV_PINGROUP_ENTRY_Y(0x208c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sen9_pee1            DRV_PINGROUP_ENTRY_Y(0x2094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_can_gpio7_paa7            DRV_PINGROUP_ENTRY_Y(0x3004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can1_dout_pz0             DRV_PINGROUP_ENTRY_Y(0x300C,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can1_din_pz1              DRV_PINGROUP_ENTRY_Y(0x3014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can0_dout_pz2             DRV_PINGROUP_ENTRY_Y(0x301c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can0_din_pz3              DRV_PINGROUP_ENTRY_Y(0x3024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio0_paa0            DRV_PINGROUP_ENTRY_Y(0x302c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio1_paa1            DRV_PINGROUP_ENTRY_Y(0x3034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio2_paa2            DRV_PINGROUP_ENTRY_Y(0x303c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio3_paa3            DRV_PINGROUP_ENTRY_Y(0x3044,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio4_paa4            DRV_PINGROUP_ENTRY_Y(0x304c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio5_paa5            DRV_PINGROUP_ENTRY_Y(0x3054,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_can_gpio6_paa6            DRV_PINGROUP_ENTRY_Y(0x305c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	1)
#define drive_gpio_aud3_pk0             DRV_PINGROUP_ENTRY_Y(0x1004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_aud2_pj7             DRV_PINGROUP_ENTRY_Y(0x100c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_aud1_pj6             DRV_PINGROUP_ENTRY_Y(0x1014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_aud0_pj5             DRV_PINGROUP_ENTRY_Y(0x101c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_aud_mclk_pj4              DRV_PINGROUP_ENTRY_Y(0x1024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_fs_pj3               DRV_PINGROUP_ENTRY_Y(0x102c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_din_pj2              DRV_PINGROUP_ENTRY_Y(0x1034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_dout_pj1             DRV_PINGROUP_ENTRY_Y(0x103c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap1_sclk_pj0             DRV_PINGROUP_ENTRY_Y(0x1044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dmic1_clk_pm1             DRV_PINGROUP_ENTRY_Y(0x2004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dmic1_dat_pm0             DRV_PINGROUP_ENTRY_Y(0x200c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dmic2_dat_pm2             DRV_PINGROUP_ENTRY_Y(0x2014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dmic2_clk_pm3             DRV_PINGROUP_ENTRY_Y(0x201c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dmic4_dat_pm4             DRV_PINGROUP_ENTRY_Y(0x2024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dmic4_clk_pm5             DRV_PINGROUP_ENTRY_Y(0x202c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dap4_fs_pcc3              DRV_PINGROUP_ENTRY_Y(0x2034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dap4_din_pcc2             DRV_PINGROUP_ENTRY_Y(0x203c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dap4_dout_pcc1            DRV_PINGROUP_ENTRY_Y(0x2044,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_dap4_sclk_pcc0            DRV_PINGROUP_ENTRY_Y(0x204c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_extperiph2_clk_po1        DRV_PINGROUP_ENTRY_Y(0x0004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_extperiph1_clk_po0        DRV_PINGROUP_ENTRY_Y(0x000c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cam_i2c_sda_po3           DRV_PINGROUP_ENTRY_Y(0x0014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_cam_i2c_scl_po2           DRV_PINGROUP_ENTRY_Y(0x001c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam1_pn0             DRV_PINGROUP_ENTRY_Y(0x0024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam2_pn1             DRV_PINGROUP_ENTRY_Y(0x002c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam3_pn2             DRV_PINGROUP_ENTRY_Y(0x0034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam4_pn3             DRV_PINGROUP_ENTRY_Y(0x003c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam5_pn4             DRV_PINGROUP_ENTRY_Y(0x0044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam6_pn5             DRV_PINGROUP_ENTRY_Y(0x004c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_cam7_pn6             DRV_PINGROUP_ENTRY_Y(0x0054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_din_pc3              DRV_PINGROUP_ENTRY_Y(0x4004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_dout_pc2             DRV_PINGROUP_ENTRY_Y(0x400c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_fs_pc4               DRV_PINGROUP_ENTRY_Y(0x4014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dap2_sclk_pc1             DRV_PINGROUP_ENTRY_Y(0x401c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_cts_pb3             DRV_PINGROUP_ENTRY_Y(0x4024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_rts_pb2             DRV_PINGROUP_ENTRY_Y(0x402c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_rx_pb1              DRV_PINGROUP_ENTRY_Y(0x4034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart4_tx_pb0              DRV_PINGROUP_ENTRY_Y(0x403c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan4_pc0             DRV_PINGROUP_ENTRY_Y(0x4044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan3_pb6             DRV_PINGROUP_ENTRY_Y(0x404c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan2_pb5             DRV_PINGROUP_ENTRY_Y(0x4054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan1_pb4             DRV_PINGROUP_ENTRY_Y(0x405c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen1_i2c_scl_pc5          DRV_PINGROUP_ENTRY_Y(0x4064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen1_i2c_sda_pc6          DRV_PINGROUP_ENTRY_Y(0x406c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_cts_pt3             DRV_PINGROUP_ENTRY_Y(0x5004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_rts_pt2             DRV_PINGROUP_ENTRY_Y(0x500c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_rx_pt1              DRV_PINGROUP_ENTRY_Y(0x5014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart1_tx_pt0              DRV_PINGROUP_ENTRY_Y(0x501c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_out3_pq5        DRV_PINGROUP_ENTRY_Y(0x502c,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_out2_pq4        DRV_PINGROUP_ENTRY_Y(0x5034,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_out1_pq3        DRV_PINGROUP_ENTRY_Y(0x503c,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_out0_pq2        DRV_PINGROUP_ENTRY_Y(0x5044,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_in_pq1          DRV_PINGROUP_ENTRY_Y(0x504c,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_directdc1_clk_pq0         DRV_PINGROUP_ENTRY_Y(0x5054,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_pq0_pi0              DRV_PINGROUP_ENTRY_Y(0x3004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq1_pi1              DRV_PINGROUP_ENTRY_Y(0x300c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq2_pi2              DRV_PINGROUP_ENTRY_Y(0x3014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq3_pi3              DRV_PINGROUP_ENTRY_Y(0x301c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq4_pi4              DRV_PINGROUP_ENTRY_Y(0x3024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq5_pi5              DRV_PINGROUP_ENTRY_Y(0x302c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq6_pi6              DRV_PINGROUP_ENTRY_Y(0x3034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_pq7_pi7              DRV_PINGROUP_ENTRY_Y(0x303c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_edp2_pp5             DRV_PINGROUP_ENTRY_Y(0x10004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_edp3_pp6             DRV_PINGROUP_ENTRY_Y(0x1000c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_edp0_pp3             DRV_PINGROUP_ENTRY_Y(0x10014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_edp1_pp4             DRV_PINGROUP_ENTRY_Y(0x1001c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch0_hpd_pp0        DRV_PINGROUP_ENTRY_Y(0x10024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_dp_aux_ch1_hpd_pp1        DRV_PINGROUP_ENTRY_Y(0x1002c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_hdmi_cec_pp2              DRV_PINGROUP_ENTRY_Y(0x10034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l2_clkreq_n_pa6       DRV_PINGROUP_ENTRY_Y(0x7004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_wake_n_pa2            DRV_PINGROUP_ENTRY_Y(0x700c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l1_clkreq_n_pa4       DRV_PINGROUP_ENTRY_Y(0x7014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l1_rst_n_pa3          DRV_PINGROUP_ENTRY_Y(0x701c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l0_clkreq_n_pa1       DRV_PINGROUP_ENTRY_Y(0x7024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l0_rst_n_pa0          DRV_PINGROUP_ENTRY_Y(0x702c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_pex_l2_rst_n_pa5          DRV_PINGROUP_ENTRY_Y(0x7034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_sdmmc1_clk_pd0            DRV_PINGROUP_ENTRY_Y(0x8004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_cmd_pd1            DRV_PINGROUP_ENTRY_Y(0x800c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat3_pd5           DRV_PINGROUP_ENTRY_Y(0x8018,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat2_pd4           DRV_PINGROUP_ENTRY_Y(0x8020,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat1_pd3           DRV_PINGROUP_ENTRY_Y(0x8028,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc1_dat0_pd2           DRV_PINGROUP_ENTRY_Y(0x8030,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_td3_pe4              DRV_PINGROUP_ENTRY_Y(0x9004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_td2_pe3              DRV_PINGROUP_ENTRY_Y(0x900c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_td1_pe2              DRV_PINGROUP_ENTRY_Y(0x9014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_td0_pe1              DRV_PINGROUP_ENTRY_Y(0x901c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rd3_pf1              DRV_PINGROUP_ENTRY_Y(0x9024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rd2_pf0              DRV_PINGROUP_ENTRY_Y(0x902c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rd1_pe7              DRV_PINGROUP_ENTRY_Y(0x9034,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_mdio_pf4             DRV_PINGROUP_ENTRY_Y(0x903c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rd0_pe6              DRV_PINGROUP_ENTRY_Y(0x9044,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_mdc_pf5              DRV_PINGROUP_ENTRY_Y(0x904c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_txc_pe0              DRV_PINGROUP_ENTRY_Y(0x9058,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rxc_pf3              DRV_PINGROUP_ENTRY_Y(0x9060,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_tx_ctl_pe5           DRV_PINGROUP_ENTRY_Y(0x9068,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_eqos_rx_ctl_pf2           DRV_PINGROUP_ENTRY_Y(0x9070,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat3_pg5           DRV_PINGROUP_ENTRY_Y(0xa004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat2_pg4           DRV_PINGROUP_ENTRY_Y(0xa00c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat1_pg3           DRV_PINGROUP_ENTRY_Y(0xa014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_dat0_pg2           DRV_PINGROUP_ENTRY_Y(0xa01c,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_cmd_pg1            DRV_PINGROUP_ENTRY_Y(0xa028,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_sdmmc3_clk_pg0            DRV_PINGROUP_ENTRY_Y(0xa030,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_io3_pr4              DRV_PINGROUP_ENTRY_Y(0xB004,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_io2_pr3              DRV_PINGROUP_ENTRY_Y(0xB00C,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_io1_pr2              DRV_PINGROUP_ENTRY_Y(0xB014,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_io0_pr1              DRV_PINGROUP_ENTRY_Y(0xB01C,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_sck_pr0              DRV_PINGROUP_ENTRY_Y(0xB024,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_qspi_cs_n_pr5             DRV_PINGROUP_ENTRY_Y(0xB02C,	-1,	-1,	-1,	-1,	28,	2,	30,	2,	0)
#define drive_gpio_sw1_pff1             DRV_PINGROUP_ENTRY_Y(0x1004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sw2_pff2             DRV_PINGROUP_ENTRY_Y(0x100c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sw3_pff3             DRV_PINGROUP_ENTRY_Y(0x1014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_sw4_pff4             DRV_PINGROUP_ENTRY_Y(0x101c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_shutdown                  DRV_PINGROUP_ENTRY_Y(0x1024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pmu_int                   DRV_PINGROUP_ENTRY_Y(0x102C,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_safe_state_ps3            DRV_PINGROUP_ENTRY_Y(0x1034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_vcomp_alert_ps4           DRV_PINGROUP_ENTRY_Y(0x103c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_soc_pwr_req               DRV_PINGROUP_ENTRY_Y(0x1044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_batt_oc_ps2               DRV_PINGROUP_ENTRY_Y(0x104c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_clk_32k_in                DRV_PINGROUP_ENTRY_Y(0x1054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_power_on_pff0             DRV_PINGROUP_ENTRY_Y(0x105c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pwr_i2c_scl_ps0           DRV_PINGROUP_ENTRY_Y(0x1064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_pwr_i2c_sda_ps1           DRV_PINGROUP_ENTRY_Y(0x106c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis0_pu0             DRV_PINGROUP_ENTRY_Y(0x1084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis1_pu1             DRV_PINGROUP_ENTRY_Y(0x108c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis2_pu2             DRV_PINGROUP_ENTRY_Y(0x1094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis3_pu3             DRV_PINGROUP_ENTRY_Y(0x109c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis4_pu4             DRV_PINGROUP_ENTRY_Y(0x10a4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_dis5_pu5             DRV_PINGROUP_ENTRY_Y(0x10ac,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	1)
#define drive_gpio_wan8_ph3             DRV_PINGROUP_ENTRY_Y(0xd004,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan7_ph2             DRV_PINGROUP_ENTRY_Y(0xd00c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan6_ph1             DRV_PINGROUP_ENTRY_Y(0xd014,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_wan5_ph0             DRV_PINGROUP_ENTRY_Y(0xd01c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_tx_px0              DRV_PINGROUP_ENTRY_Y(0xd024,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_rx_px1              DRV_PINGROUP_ENTRY_Y(0xd02c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_rts_px2             DRV_PINGROUP_ENTRY_Y(0xd034,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart2_cts_px3             DRV_PINGROUP_ENTRY_Y(0xd03c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_rx_px5              DRV_PINGROUP_ENTRY_Y(0xd044,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_tx_px4              DRV_PINGROUP_ENTRY_Y(0xd04c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_rts_px6             DRV_PINGROUP_ENTRY_Y(0xd054,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_uart5_cts_px7             DRV_PINGROUP_ENTRY_Y(0xd05c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm1_py0             DRV_PINGROUP_ENTRY_Y(0xd064,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm2_py1             DRV_PINGROUP_ENTRY_Y(0xd06c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm3_py2             DRV_PINGROUP_ENTRY_Y(0xd074,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm4_py3             DRV_PINGROUP_ENTRY_Y(0xd07c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm5_py4             DRV_PINGROUP_ENTRY_Y(0xd084,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm6_py5             DRV_PINGROUP_ENTRY_Y(0xd08c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpio_mdm7_py6             DRV_PINGROUP_ENTRY_Y(0xd094,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_bcpu_pwr_req_ph4          DRV_PINGROUP_ENTRY_Y(0xd09c,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_mcpu_pwr_req_ph5          DRV_PINGROUP_ENTRY_Y(0xd0a4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gpu_pwr_req_ph6           DRV_PINGROUP_ENTRY_Y(0xd0ac,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen7_i2c_scl_pl0          DRV_PINGROUP_ENTRY_Y(0xd0b4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen7_i2c_sda_pl1          DRV_PINGROUP_ENTRY_Y(0xd0bc,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen9_i2c_sda_pl3          DRV_PINGROUP_ENTRY_Y(0xd0c4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gen9_i2c_scl_pl2          DRV_PINGROUP_ENTRY_Y(0xd0cc,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_usb_vbus_en0_pl4          DRV_PINGROUP_ENTRY_Y(0xd0d4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_usb_vbus_en1_pl5          DRV_PINGROUP_ENTRY_Y(0xd0dc,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gp_pwm7_pl7               DRV_PINGROUP_ENTRY_Y(0xd0e4,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_gp_pwm6_pl6               DRV_PINGROUP_ENTRY_Y(0xd0ec,	12,	5,	20,	5,	-1,	-1,	-1,	-1,	0)
#define drive_ufs0_rst_pbb1             DRV_PINGROUP_ENTRY_Y(0x11004,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)
#define drive_ufs0_ref_clk_pbb0         DRV_PINGROUP_ENTRY_Y(0x1100c,	12,	9,	24,	8,	-1,	-1,	-1,	-1,	0)

#define drive_directdc_comp		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc1_comp		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_eqos_comp			DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc3_comp		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_clk		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_cmd		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dqs		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat7		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat6		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat5		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat4		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat3		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat2		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat1		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_sdmmc4_dat0		DRV_PINGROUP_ENTRY_N(no_entry)
#define drive_qspi_comp			DRV_PINGROUP_ENTRY_N(no_entry)

#define DRV_PINGROUP(pg_name, r, drvdn_b, drvdn_w, drvup_b, drvup_w,	\
		     slwr_b, slwr_w, slwf_b, slwf_w, bank)		\
	{								\
		.name = "drive_" #pg_name,				\
		.pins = drive_##pg_name##_pins,				\
		.npins = ARRAY_SIZE(drive_##pg_name##_pins),		\
		PIN_PINGROUP_ENTRY_N(pg_name),				\
		DRV_PINGROUP_ENTRY_Y(r, drvdn_b, drvdn_w, drvup_b,	\
				     drvup_w, slwr_b, slwr_w, slwf_b,	\
				     slwf_w, bank),			\
	}

#define PINGROUP(pg_name, f0, f1, f2, f3, r, bank, pupd, e_io_hv, e_lpbk, \
		 e_input, e_lpdr, e_pbias_buf, gpio_sfio_sel, e_od,	\
		 schmitt_b, drvtype, epreemp, io_reset, rfu_in)		\
	{								\
		.name = #pg_name,					\
		.pins = pg_name##_pins,					\
		.npins = ARRAY_SIZE(pg_name##_pins),			\
			.funcs = {					\
				TEGRA_MUX_ ## f0,			\
				TEGRA_MUX_ ## f1,			\
				TEGRA_MUX_ ## f2,			\
				TEGRA_MUX_ ## f3,			\
			},						\
		PIN_PINGROUP_ENTRY_Y(r, bank, pupd, e_io_hv, e_lpbk,	\
				     e_input,	\
				     e_lpdr, e_pbias_buf, gpio_sfio_sel, \
				     e_od, schmitt_b, drvtype, epreemp,	\
				     io_reset, rfu_in),			\
		drive_##pg_name,					\
	}

static const struct tegra_pingroup tegra186_groups[] = {
		/*      pg_name,	f0,	f1,	f2,	f3,	r,	bank,	pupd,	e_io_hv,	e_input,	e_lpdr */
	PINGROUP(touch_clk_pee2,	TOUCH,		RSVD1,		RSVD2,		RSVD3,		0x2000,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart3_cts_pw5,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2008,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart3_rts_pw4,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2010,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart3_rx_pw3,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2018,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart3_tx_pw2,		UARTC,		RSVD1,		RSVD2,		RSVD3,		0x2020,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen8_i2c_sda_pw1,	I2C8,		RSVD1,		RSVD2,		RSVD3,		0x2028,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen8_i2c_scl_pw0,	I2C8,		RSVD1,		RSVD2,		RSVD3,		0x2030,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart7_rx_pw7,		UARTG,		RSVD1,		RSVD2,		RSVD3,		0x2038,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart7_tx_pw6,		UARTG,		RSVD1,		RSVD2,		RSVD3,		0x2040,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen0_pv0,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x2048,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen1_pv1,		SPI2,		RSVD1,		RSVD2,		RSVD3,		0x2050,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen2_pv2,		SPI2,		RSVD1,		RSVD2,		RSVD3,		0x2058,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen3_pv3,		SPI2,		RSVD1,		RSVD2,		RSVD3,		0x2060,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen4_pv4,		SPI2,		RSVD1,		RSVD2,		RSVD3,		0x2068,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen5_pv5,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x2070,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen6_pv6,		RSVD0,		GP,		RSVD2,		RSVD3,		0x2078,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen7_pv7,		RSVD0,		WDT,		RSVD2,		RSVD3,		0x2080,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen8_pee0,	RSVD0,		I2C2,		RSVD2,		RSVD3,		0x2088,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sen9_pee1,	RSVD0,		I2C2,		RSVD2,		RSVD3,		0x2090,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(can_gpio7_paa7,	RSVD0,		WDT,		RSVD2,		RSVD3,		0x3000,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can1_dout_pz0,		CAN1,		RSVD1,		RSVD2,		RSVD3,		0x3008,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can1_din_pz1,		CAN1,		RSVD1,		RSVD2,		RSVD3,		0x3010,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can0_dout_pz2,		CAN0,		RSVD1,		RSVD2,		RSVD3,		0x3018,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can0_din_pz3,		CAN0,		RSVD1,		RSVD2,		RSVD3,		0x3020,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio0_paa0,	RSVD0,		DMIC3,		DMIC5,		RSVD3,		0x3028,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio1_paa1,	RSVD0,		DMIC3,		DMIC5,		RSVD3,		0x3030,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio2_paa2,	GPIO,		RSVD1,		RSVD2,		RSVD3,		0x3038,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio3_paa3,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3040,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio4_paa4,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3048,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio5_paa5,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3050,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(can_gpio6_paa6,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x3058,		1,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_aud3_pk0,		RSVD0,		DSPK1,		SPDIF,		RSVD3,		0x1000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_aud2_pj7,		RSVD0,		DSPK1,		SPDIF,		RSVD3,		0x1008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_aud1_pj6,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_aud0_pj5,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(aud_mclk_pj4,		AUD,		RSVD1,		RSVD2,		RSVD3,		0x1020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap1_fs_pj3,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap1_din_pj2,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap1_dout_pj1,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap1_sclk_pj0,		I2S1,		RSVD1,		RSVD2,		RSVD3,		0x1040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dmic1_clk_pm1,		DMIC1,		I2S3,		RSVD2,		RSVD3,		0x2000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dmic1_dat_pm0,		DMIC1,		I2S3,		RSVD2,		RSVD3,		0x2008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dmic2_dat_pm2,		DMIC2,		I2S3,		RSVD2,		RSVD3,		0x2010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dmic2_clk_pm3,		DMIC2,		I2S3,		RSVD2,		RSVD3,		0x2018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dmic4_dat_pm4,		DMIC4,		DSPK0,		RSVD2,		RSVD3,		0x2020,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dmic4_clk_pm5,		DMIC4,		DSPK0,		RSVD2,		RSVD3,		0x2028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dap4_fs_pcc3,		I2S4,		RSVD1,		RSVD2,		RSVD3,		0x2030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dap4_din_pcc2,		I2S4,		RSVD1,		RSVD2,		RSVD3,		0x2038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dap4_dout_pcc1,	I2S4,		RSVD1,		RSVD2,		RSVD3,		0x2040,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(dap4_sclk_pcc0,	I2S4,		RSVD1,		RSVD2,		RSVD3,		0x2048,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(extperiph2_clk_po1,	EXTPERIPH2,	RSVD1,		RSVD2,		RSVD3,		0x0000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(extperiph1_clk_po0,	EXTPERIPH1,	RSVD1,		RSVD2,		RSVD3,		0x0008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(cam_i2c_sda_po3,	I2C3,		RSVD1,		RSVD2,		RSVD3,		0x0010,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(cam_i2c_scl_po2,	I2C3,		RSVD1,		RSVD2,		RSVD3,		0x0018,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam1_pn0,		VGP1,		RSVD1,		RSVD2,		RSVD3,		0x0020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam2_pn1,		VGP2,		EXTPERIPH3,	RSVD2,		RSVD3,		0x0028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam3_pn2,		VGP3,		EXTPERIPH4,	RSVD2,		RSVD3,		0x0030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam4_pn3,		VGP4,		SPI4,		RSVD2,		RSVD3,		0x0038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam5_pn4,		VGP5,		SPI4,		RSVD2,		RSVD3,		0x0040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam6_pn5,		VGP6,		SPI4,		RSVD2,		RSVD3,		0x0048,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_cam7_pn6,		RSVD0,		SPI4,		RSVD2,		RSVD3,		0x0050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap2_din_pc3,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap2_dout_pc2,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap2_fs_pc4,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dap2_sclk_pc1,		I2S2,		RSVD1,		RSVD2,		RSVD3,		0x4018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart4_cts_pb3,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart4_rts_pb2,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart4_rx_pb1,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart4_tx_pb0,		UARTD,		RSVD1,		RSVD2,		RSVD3,		0x4038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan4_pc0,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan3_pb6,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4048,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan2_pb5,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan1_pb4,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x4058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen1_i2c_scl_pc5,	I2C1,		RSVD1,		RSVD2,		RSVD3,		0x4060,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen1_i2c_sda_pc6,	I2C1,		RSVD1,		RSVD2,		RSVD3,		0x4068,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart1_cts_pt3,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x5000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart1_rts_pt2,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x5008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart1_rx_pt1,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x5010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart1_tx_pt0,		UARTA,		RSVD1,		RSVD2,		RSVD3,		0x5018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(directdc1_out3_pq5,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc1_out2_pq4,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc1_out1_pq3,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc1_out0_pq2,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5040,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc1_in_pq1,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5048,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc1_clk_pq0,	DIRECTDC1,	RSVD1,		RSVD2,		RSVD3,		0x5050,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(directdc_comp,		DIRECTDC,	RSVD1,		RSVD2,		RSVD3,		0x5058,		0,	Y,	-1,	-1,	-1,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq0_pi0,		RSVD0,		IQC0,		I2S6,		RSVD3,		0x3000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq1_pi1,		RSVD0,		IQC0,		I2S6,		RSVD3,		0x3008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq2_pi2,		RSVD0,		IQC0,		I2S6,		RSVD3,		0x3010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq3_pi3,		RSVD0,		IQC0,		I2S6,		RSVD3,		0x3018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq4_pi4,		RSVD0,		IQC1,		DTV,		RSVD3,		0x3020,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq5_pi5,		RSVD0,		IQC1,		DTV,		RSVD3,		0x3028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq6_pi6,		RSVD0,		IQC1,		DTV,		RSVD3,		0x3030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_pq7_pi7,		RSVD0,		IQC1,		DTV,		RSVD3,		0x3038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_edp2_pp5,		RSVD0,		UARTF,		SDMMC3,		RSVD3,		0x10000,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_edp3_pp6,		RSVD0,		UARTF,		SDMMC1,		RSVD3,		0x10008,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_edp0_pp3,		RSVD0,		UARTF,		SDMMC3,		RSVD3,		0x10010,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_edp1_pp4,		RSVD0,		UARTF,		SDMMC1,		RSVD3,		0x10018,	0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dp_aux_ch0_hpd_pp0,	DP,		RSVD1,		RSVD2,		RSVD3,		0x10020,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(dp_aux_ch1_hpd_pp1,	DP,		RSVD1,		RSVD2,		RSVD3,		0x10028,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(hdmi_cec_pp2,		HDMI,		RSVD1,		RSVD2,		RSVD3,		0x10030,	0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l2_clkreq_n_pa6,	PE2,		GP,		SATA,		RSVD3,		0x7000,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_wake_n_pa2,	PE,		RSVD1,		RSVD2,		RSVD3,		0x7008,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l1_clkreq_n_pa4,	PE1,		RSVD1,		RSVD2,		RSVD3,		0x7010,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l1_rst_n_pa3,	PE1,		RSVD1,		RSVD2,		RSVD3,		0x7018,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l0_clkreq_n_pa1,	PE0,		RSVD1,		RSVD2,		RSVD3,		0x7020,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l0_rst_n_pa0,	PE0,		RSVD1,		RSVD2,		RSVD3,		0x7028,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pex_l2_rst_n_pa5,	PE2,		SOC,		SATA,		RSVD3,		0x7030,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(sdmmc1_clk_pd0,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8000,		0,	Y,	5,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc1_cmd_pd1,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc1_comp,		SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8010,		0,	Y,	-1,	-1,	6,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N),
	PINGROUP(sdmmc1_dat3_pd5,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8014,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc1_dat2_pd4,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x801c,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc1_dat1_pd3,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x8024,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc1_dat0_pd2,	SDMMC1,		RSVD1,		RSVD2,		RSVD3,		0x802c,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_td3_pe4,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_td2_pe3,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_td1_pe2,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_td0_pe1,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rd3_pf1,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9020,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rd2_pf0,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rd1_pe7,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9030,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_mdio_pf4,		EQOS,		SOC,		RSVD2,		RSVD3,		0x9038,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rd0_pe6,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9040,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_mdc_pf5,		EQOS,		RSVD1,		RSVD2,		RSVD3,		0x9048,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_comp,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9050,		0,	Y,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N),
	PINGROUP(eqos_txc_pe0,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9054,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rxc_pf3,		EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x905c,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_tx_ctl_pe5,	EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x9064,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(eqos_rx_ctl_pf2,	EQOS,		SDMMC2,		RSVD2,		RSVD3,		0x906c,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_dat3_pg5,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_dat2_pg4,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_dat1_pg3,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_dat0_pg2,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_comp,		SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa020,		0,	Y,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N),
	PINGROUP(sdmmc3_cmd_pg1,	SDMMC3,		RSVD1,		RSVD2,		RSVD3,		0xa024,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc3_clk_pg0,	SDMMC3,		RSVD1,		RSVD1,		RSVD3,		0xa02c,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_clk,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6004,		0,	Y,	-1,	5,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_cmd,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6008,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dqs,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x600c,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat7,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6010,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat6,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6014,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat5,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6018,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat4,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x601c,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat3,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6020,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat2,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6024,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat1,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x6028,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(sdmmc4_dat0,		SDMMC4,		RSVD1,		RSVD2,		RSVD3,		0x602c,		0,	Y,	-1,	-1,	6,	-1,	9,	-1,	-1,	12,	Y,	-1,	-1,	Y),
	PINGROUP(qspi_io3_pr4,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB000,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_io2_pr3,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB008,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_io1_pr2,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB010,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_io0_pr1,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB018,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_sck_pr0,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB020,		0,	Y,	-1,	5,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_cs_n_pr5,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB028,		0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(qspi_comp,		QSPI,		RSVD1,		RSVD2,		RSVD3,		0xB030,		0,	Y,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	-1,	Y,	-1,	-1,	Y),
	PINGROUP(gpio_sw1_pff1,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1000,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sw2_pff2,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1008,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sw3_pff3,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1010,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_sw4_pff4,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1018,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(shutdown,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1020,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N),
	PINGROUP(pmu_int,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1028,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N),
	PINGROUP(safe_state_ps3,	SCE,		RSVD1,		RSVD2,		RSVD3,		0x1030,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(vcomp_alert_ps4,	SOC,		RSVD1,		RSVD2,		RSVD3,		0x1038,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(soc_pwr_req,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1040,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	12,	N,	-1,	-1,	N),
	PINGROUP(batt_oc_ps2,		SOC,		RSVD1,		RSVD2,		RSVD3,		0x1048,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(clk_32k_in,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1050,		1,	Y,	-1,	-1,	6,	8,	-1,	-1,	-1,	-1,	N,	-1,	-1,	N),
	PINGROUP(power_on_pff0,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0x1058,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pwr_i2c_scl_ps0,	I2C5,		RSVD1,		RSVD2,		RSVD3,		0x1060,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(pwr_i2c_sda_ps1,	I2C5,		RSVD1,		RSVD2,		RSVD3,		0x1068,		1,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis0_pu0,		RSVD0,		GP,		DCB,		DCC,		0x1080,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis1_pu1,		RSVD0,		RSVD1,		DISPLAYA,	RSVD3,		0x1088,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis2_pu2,		RSVD0,		GP,		DCA,		RSVD3,		0x1090,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis3_pu3,		RSVD0,		RSVD1,		DISPLAYB,	DCC,		0x1098,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis4_pu4,		RSVD0,		SOC,		DCA,		RSVD3,		0x10a0,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_dis5_pu5,		RSVD0,		GP,		DCC,		DCB,		0x10a8,		1,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan8_ph3,		RSVD0,		RSVD1,		SPI1,		RSVD3,		0xd000,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan7_ph2,		RSVD0,		RSVD1,		SPI1,		RSVD3,		0xd008,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan6_ph1,		RSVD0,		RSVD1,		SPI1,		RSVD3,		0xd010,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_wan5_ph0,		RSVD0,		RSVD1,		SPI1,		RSVD3,		0xd018,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart2_tx_px0,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xd020,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart2_rx_px1,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xd028,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart2_rts_px2,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xd030,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart2_cts_px3,		UARTB,		RSVD1,		RSVD2,		RSVD3,		0xd038,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart5_rx_px5,		UARTE,		SPI3,		GP,		RSVD3,		0xd040,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart5_tx_px4,		UARTE,		SPI3,		NV,		RSVD3,		0xd048,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart5_rts_px6,		UARTE,		SPI3,		RSVD2,		RSVD3,		0xd050,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(uart5_cts_px7,		UARTE,		SPI3,		RSVD2,		RSVD3,		0xd058,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm1_py0,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd060,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm2_py1,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd068,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm3_py2,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd070,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm4_py3,		RSVD0,		SPI1,		CCLA,		RSVD3,		0xd078,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm5_py4,		RSVD0,		SPI1,		RSVD2,		RSVD3,		0xd080,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm6_py5,		SOC,		RSVD1,		RSVD2,		RSVD3,		0xd088,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpio_mdm7_py6,		RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd090,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(bcpu_pwr_req_ph4,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd098,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(mcpu_pwr_req_ph5,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd0a0,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gpu_pwr_req_ph6,	RSVD0,		RSVD1,		RSVD2,		RSVD3,		0xd0a8,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen7_i2c_scl_pl0,	I2C7,		I2S5,		RSVD2,		RSVD3,		0xd0b0,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen7_i2c_sda_pl1,	I2C7,		I2S5,		RSVD2,		RSVD3,		0xd0b8,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen9_i2c_sda_pl3,	I2C9,		I2S5,		RSVD2,		RSVD3,		0xd0c0,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gen9_i2c_scl_pl2,	I2C9,		I2S5,		RSVD2,		RSVD3,		0xd0c8,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(usb_vbus_en0_pl4,	USB,		RSVD1,		RSVD2,		RSVD3,		0xd0d0,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(usb_vbus_en1_pl5,	USB,		RSVD1,		RSVD2,		RSVD3,		0xd0d8,		0,	Y,	5,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gp_pwm7_pl7,		GP,		RSVD1,		RSVD2,		RSVD3,		0xd0e0,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(gp_pwm6_pl6,		GP,		RSVD1,		RSVD2,		RSVD3,		0xd0e8,		0,	Y,	-1,	-1,	6,	8,	-1,	10,	11,	12,	N,	-1,	-1,	N),
	PINGROUP(ufs0_rst_pbb1,		UFS0,		RSVD1,		RSVD2,		RSVD3,		0x11000,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
	PINGROUP(ufs0_ref_clk_pbb0,	UFS0,		RSVD1,		RSVD2,		RSVD3,		0x11008,	0,	Y,	-1,	-1,	6,	-1,	9,	10,	-1,	12,	Y,	15,	17,	Y),
};

static const struct tegra_pinctrl_soc_data tegra186_pinctrl = {
	.ngpios = NUM_GPIOS,
	.pins = tegra186_pins,
	.npins = ARRAY_SIZE(tegra186_pins),
	.functions = tegra186_functions,
	.nfunctions = ARRAY_SIZE(tegra186_functions),
	.groups = tegra186_groups,
	.ngroups = ARRAY_SIZE(tegra186_groups),
	.is_gpio_reg_support = true,
	.hsm_in_mux = false,
	.schmitt_in_mux = true,
	.drvtype_in_mux = true,
};

static int tegra186_pinctrl_probe(struct platform_device *pdev)
{
	return tegra_pinctrl_probe(pdev, &tegra186_pinctrl);
}

static struct of_device_id tegra186_pinctrl_of_match[] = {
	{ .compatible = "nvidia,tegra186-pinmux", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra186_pinctrl_of_match);

static struct platform_driver tegra186_pinctrl_driver = {
	.driver = {
		.name = "tegra186-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = tegra186_pinctrl_of_match,
	},
	.probe = tegra186_pinctrl_probe,
};

static int __init tegra186_pinctrl_init(void)
{
	return platform_driver_register(&tegra186_pinctrl_driver);
}
postcore_initcall_sync(tegra186_pinctrl_init);

static void __exit tegra186_pinctrl_exit(void)
{
	platform_driver_unregister(&tegra186_pinctrl_driver);
}
module_exit(tegra186_pinctrl_exit);

MODULE_AUTHOR("Suresh Mangipudi <smangipudi@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra186 pinctrl driver");
MODULE_LICENSE("GPL v2");
