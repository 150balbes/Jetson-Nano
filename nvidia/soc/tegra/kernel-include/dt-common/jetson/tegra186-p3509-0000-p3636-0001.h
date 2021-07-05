// SPDX-License-Identifier: GPL-2.0-only
/*
 * Definitions for Jetson tegra186-p3509-0000-p3636-0001 board.
 *
 * Copyright (c) 2021 NVIDIA CORPORATION. All rights reserved.
 *
 */

#include <dt-bindings/gpio/tegra186-gpio.h>

#define JETSON_COMPATIBLE	"nvidia,p3509-0000+p3636-0001"

/* SoC function name for clock signal on 40-pin header pin 7 */
#define HDR40_CLK	"aud"
/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s1"
/* SoC function name for SPI interface on 40-pin header pins 19, 21, 23, and 24 */
#define HDR40_SPI	"spi1"
/* SoC function name for UART interface on 40-pin header pins 8, 10, 11 and 36 */
#define HDR40_UART	"uartc"
/* SoC function name for touch clock signal on 40-pin header pin 31 */
#define HDR40_TOUCH     "touch"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN7	"aud_mclk_pj4"
#define HDR40_PIN11	"uart3_rts_pw4"
#define HDR40_PIN12	"dap1_sclk_pj0"
#define HDR40_PIN13	"gpio_sen1_pv1"
#define HDR40_PIN18	"gpio_sen4_pv4"
#define HDR40_PIN19	"gpio_wan7_ph2"
#define HDR40_PIN21	"gpio_wan6_ph1"
#define HDR40_PIN22	"gpio_sen2_pv2"
#define HDR40_PIN23	"gpio_wan5_ph0"
#define HDR40_PIN24	"gpio_wan8_ph3"
#define HDR40_PIN29	"gpio_cam2_pn1"
#define HDR40_PIN31	"touch_clk_pee2"
#define HDR40_PIN32	"gpio_dis0_pu0"
#define HDR40_PIN33	"gpio_dis5_pu5"
#define HDR40_PIN35	"dap1_fs_pj3"
#define HDR40_PIN36	"uart3_cts_pw5"
#define HDR40_PIN37	"gpio_sen3_pv3"
#define HDR40_PIN38	"dap1_din_pj2"
#define HDR40_PIN40	"dap1_dout_pj1"

/* SoC GPIO definitions for 40-pin header */
#define HDR40_PIN7_GPIO		TEGRA_MAIN_GPIO(J, 4)
#define HDR40_PIN11_GPIO	TEGRA_MAIN_GPIO(W, 4)
#define HDR40_PIN12_GPIO	TEGRA_MAIN_GPIO(J, 0)
#define HDR40_PIN13_GPIO	TEGRA_MAIN_GPIO(V, 1)
#define HDR40_PIN18_GPIO	TEGRA_MAIN_GPIO(V, 4)
#define HDR40_PIN19_GPIO	TEGRA_MAIN_GPIO(H, 2)
#define HDR40_PIN21_GPIO	TEGRA_MAIN_GPIO(H, 1)
#define HDR40_PIN22_GPIO	TEGRA_MAIN_GPIO(V, 2)
#define HDR40_PIN23_GPIO	TEGRA_MAIN_GPIO(H, 0)
#define HDR40_PIN24_GPIO	TEGRA_MAIN_GPIO(H, 3)
#define HDR40_PIN29_GPIO	TEGRA_MAIN_GPIO(N, 1)
#define HDR40_PIN31_GPIO	TEGRA_AON_GPIO(EE, 2)
#define HDR40_PIN32_GPIO	TEGRA_MAIN_GPIO(U, 0)
#define HDR40_PIN33_GPIO	TEGRA_MAIN_GPIO(U, 5)
#define HDR40_PIN35_GPIO	TEGRA_MAIN_GPIO(J, 3)
#define HDR40_PIN36_GPIO	TEGRA_MAIN_GPIO(W, 5)
#define HDR40_PIN37_GPIO	TEGRA_MAIN_GPIO(V, 3)
#define HDR40_PIN38_GPIO	TEGRA_MAIN_GPIO(J, 2)
#define HDR40_PIN40_GPIO	TEGRA_MAIN_GPIO(J, 1)
