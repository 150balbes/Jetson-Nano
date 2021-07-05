// SPDX-License-Identifier: GPL-2.0-only
/*
 * Definitions for Jetson tegra186-quill-p3310-1000-c03-00-base board.
 *
 * Copyright (c) 2019-2020 NVIDIA CORPORATION. All rights reserved.
 *
 */

#include <dt-bindings/gpio/tegra186-gpio.h>

#define JETSON_COMPATIBLE	"nvidia,p2597-0000+p3310-1000"

/* SoC function name for clock signal on 40-pin header pin 7 */
#define HDR40_CLK	"aud"
/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s1"
/* SoC function name for SPI interface on 40-pin header pins 19, 21, 23, 24 and 26 */
#define HDR40_SPI	"spi4"
/* SoC function name for UART interface on 40-pin header pins 8, 10, 11 and 36 */
#define HDR40_UART	"uarta"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN7	"aud_mclk_pj4"
#define HDR40_PIN11	"uart1_rts_pt2"
#define HDR40_PIN12	"dap1_sclk_pj0"
#define HDR40_PIN13	"gpio_aud0_pj5"
#define HDR40_PIN16	"can_gpio0_paa0"
#define HDR40_PIN18	"gpio_mdm2_py1"
#define HDR40_PIN19	"gpio_cam6_pn5"
#define HDR40_PIN21	"gpio_cam5_pn4"
#define HDR40_PIN23	"gpio_cam4_pn3"
#define HDR40_PIN24	"gpio_cam7_pn6"
#define HDR40_PIN29	"gpio_aud1_pj6"
#define HDR40_PIN31	"can_gpio2_paa2"
#define HDR40_PIN32	"can_gpio1_paa1"
#define HDR40_PIN33	"gpio_pq5_pi5"
#define HDR40_PIN35	"dap1_fs_pj3"
#define HDR40_PIN36	"uart1_cts_pt3"
#define HDR40_PIN37	"gpio_pq4_pi4"
#define HDR40_PIN38	"dap1_din_pj2"
#define HDR40_PIN40	"dap1_dout_pj1"

/* SoC GPIO definitions for 40-pin header */
#define HDR40_PIN7_GPIO		TEGRA_MAIN_GPIO(J, 4)
#define HDR40_PIN11_GPIO	TEGRA_MAIN_GPIO(T, 2)
#define HDR40_PIN12_GPIO	TEGRA_MAIN_GPIO(J, 0)
#define HDR40_PIN13_GPIO	TEGRA_MAIN_GPIO(J, 5)
#define HDR40_PIN16_GPIO	TEGRA_AON_GPIO(AA, 0)
#define HDR40_PIN18_GPIO	TEGRA_MAIN_GPIO(Y, 1)
#define HDR40_PIN19_GPIO	TEGRA_MAIN_GPIO(N, 5)
#define HDR40_PIN21_GPIO	TEGRA_MAIN_GPIO(N, 4)
#define HDR40_PIN23_GPIO	TEGRA_MAIN_GPIO(N, 3)
#define HDR40_PIN24_GPIO	TEGRA_MAIN_GPIO(N, 6)
#define HDR40_PIN29_GPIO	TEGRA_MAIN_GPIO(J, 6)
#define HDR40_PIN31_GPIO	TEGRA_AON_GPIO(AA, 2)
#define HDR40_PIN32_GPIO	TEGRA_AON_GPIO(AA, 1)
#define HDR40_PIN33_GPIO	TEGRA_MAIN_GPIO(I, 5)
#define HDR40_PIN35_GPIO	TEGRA_MAIN_GPIO(J, 3)
#define HDR40_PIN36_GPIO	TEGRA_MAIN_GPIO(T, 3)
#define HDR40_PIN37_GPIO	TEGRA_MAIN_GPIO(I, 4)
#define HDR40_PIN38_GPIO	TEGRA_MAIN_GPIO(J, 2)
#define HDR40_PIN40_GPIO	TEGRA_MAIN_GPIO(J, 1)
