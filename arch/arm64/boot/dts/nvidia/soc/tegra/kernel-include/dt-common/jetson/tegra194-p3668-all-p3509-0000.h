// SPDX-License-Identifier: GPL-2.0-only
/*
 * Definitions for Jetson tegra194-p3668-all-p3509-0000 board.
 *
 * Copyright (c) 2019-2020 NVIDIA CORPORATION. All rights reserved.
 *
 */

#include <dt-bindings/gpio/tegra194-gpio.h>

#define JETSON_COMPATIBLE	"nvidia,p3509-0000+p3668-0000", "nvidia,p3509-0000+p3668-0001"

/* SoC function name for clock signal on 40-pin header pin 7 */
#define HDR40_CLK	"aud"
/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s5"
/* SoC function name for PWM interface on 40-pin header pin 32 */
#define HDR40_PWM0	"pwm8"
/* SoC function name for PWM interface on 40-pin header pin 33 */
#define HDR40_PWM1	"pwm1"
/* SoC function name for SPI interface on 40-pin header pins 19, 21, 23, 24 and 26 */
#define HDR40_SPI	"spi1"
/* SoC function name for UART interface on 40-pin header pins 8, 10, 11 and 36 */
#define HDR40_UART	"uarta"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN7	"aud_mclk_ps4"
#define HDR40_PIN11	"uart1_rts_pr4"
#define HDR40_PIN12	"dap5_sclk_pt5"
#define HDR40_PIN13	"spi3_sck_py0"
#define HDR40_PIN15	"touch_clk_pcc4"
#define HDR40_PIN16	"spi3_cs1_py4"
#define HDR40_PIN18	"spi3_cs0_py3"
#define HDR40_PIN19	"spi1_mosi_pz5"
#define HDR40_PIN21	"spi1_miso_pz4"
#define HDR40_PIN22	"spi3_miso_py1"
#define HDR40_PIN23	"spi1_sck_pz3"
#define HDR40_PIN24	"spi1_cs0_pz6"
#define HDR40_PIN26	"spi1_cs1_pz7"
#define HDR40_PIN29	"soc_gpio41_pq5"
#define HDR40_PIN31	"soc_gpio42_pq6"
#define HDR40_PIN32	"soc_gpio44_pr0"
#define HDR40_PIN33	"soc_gpio54_pn1"
#define HDR40_PIN35	"dap5_fs_pu0"
#define HDR40_PIN36	"uart1_cts_pr5"
#define HDR40_PIN37	"spi3_mosi_py2"
#define HDR40_PIN38	"dap5_din_pt7"
#define HDR40_PIN40	"dap5_dout_pt6"

/* SoC GPIO definitions for 40-pin header */
#define HDR40_PIN7_GPIO		TEGRA_MAIN_GPIO(S, 4)
#define HDR40_PIN11_GPIO	TEGRA_MAIN_GPIO(R, 4)
#define HDR40_PIN12_GPIO	TEGRA_MAIN_GPIO(T, 5)
#define HDR40_PIN13_GPIO	TEGRA_MAIN_GPIO(Y, 0)
#define HDR40_PIN15_GPIO	TEGRA_AON_GPIO(CC, 4)
#define HDR40_PIN16_GPIO	TEGRA_MAIN_GPIO(Y, 4)
#define HDR40_PIN18_GPIO	TEGRA_MAIN_GPIO(Y, 3)
#define HDR40_PIN19_GPIO	TEGRA_MAIN_GPIO(Z, 5)
#define HDR40_PIN21_GPIO	TEGRA_MAIN_GPIO(Z, 4)
#define HDR40_PIN22_GPIO	TEGRA_MAIN_GPIO(Y, 1)
#define HDR40_PIN23_GPIO	TEGRA_MAIN_GPIO(Z, 3)
#define HDR40_PIN24_GPIO	TEGRA_MAIN_GPIO(Z, 6)
#define HDR40_PIN26_GPIO	TEGRA_MAIN_GPIO(Z, 7)
#define HDR40_PIN29_GPIO	TEGRA_MAIN_GPIO(Q, 5)
#define HDR40_PIN31_GPIO	TEGRA_MAIN_GPIO(Q, 6)
#define HDR40_PIN32_GPIO	TEGRA_MAIN_GPIO(R, 0)
#define HDR40_PIN33_GPIO	TEGRA_MAIN_GPIO(N, 1)
#define HDR40_PIN35_GPIO	TEGRA_MAIN_GPIO(U, 0)
#define HDR40_PIN36_GPIO	TEGRA_MAIN_GPIO(R, 5)
#define HDR40_PIN37_GPIO	TEGRA_MAIN_GPIO(Y, 2)
#define HDR40_PIN38_GPIO	TEGRA_MAIN_GPIO(T, 7)
#define HDR40_PIN40_GPIO	TEGRA_MAIN_GPIO(T, 6)
