// SPDX-License-Identifier: GPL-2.0-only
/*
 * Definitions for Jetson tegra210-jetson-tx1-p2597-2180-a01-devkit
 * board.
 *
 * Copyright (c) 2019-2020 NVIDIA CORPORATION. All rights reserved.
 *
 */

#include <dt-bindings/gpio/tegra-gpio.h>

#define JETSON_COMPATIBLE	"nvidia,p2597-0000+p2180-1000"

/* SoC function name for clock signal on 40-pin header pin 7 */
#define HDR40_CLK	"aud"
/* SoC function name for I2S interface on 40-pin header pins 12, 35, 38 and 40 */
#define HDR40_I2S	"i2s1"
/* SoC function name for SPI interface on 40-pin header pins 19, 21, 23, 24 and 26 */
#define HDR40_SPI	"spi1"
/* SoC function name for UART interface on 40-pin header pins 8, 10, 11 and 36 */
#define HDR40_UART	"uarta"

/* SoC pin name definitions for 40-pin header */
#define HDR40_PIN7	"aud_mclk_pbb0"
#define HDR40_PIN11	"uart1_rts_pu2"
#define HDR40_PIN12	"dap1_sclk_pb3"
#define HDR40_PIN13	"pe6"
#define HDR40_PIN16	"dmic3_dat_pe5"
#define HDR40_PIN18	"modem_wake_ap_px0"
#define HDR40_PIN19	"spi1_mosi_pc0"
#define HDR40_PIN21	"spi1_miso_pc1"
#define HDR40_PIN23	"spi1_sck_pc2"
#define HDR40_PIN24	"spi1_cs0_pc3"
#define HDR40_PIN26	"spi1_cs1_pc4"
#define HDR40_PIN29	"gpio_x1_aud_pbb3"
#define HDR40_PIN31	"motion_int_px2"
#define HDR40_PIN32	"dmic3_clk_pe4"
#define HDR40_PIN33	"ap_wake_nfc_ph7"
#define HDR40_PIN35	"dap1_fs_pb0"
#define HDR40_PIN36	"uart1_cts_pu3"
#define HDR40_PIN37	"als_prox_int_px3"
#define HDR40_PIN38	"dap1_din_pb1"
#define HDR40_PIN40	"dap1_dout_pb2"

/* SoC GPIO definitions for 40-pin header */
#define HDR40_PIN7_GPIO	TEGRA_GPIO(BB, 0)
#define HDR40_PIN11_GPIO	TEGRA_GPIO(U, 2)
#define HDR40_PIN12_GPIO	TEGRA_GPIO(B, 3)
#define HDR40_PIN13_GPIO	TEGRA_GPIO(E, 6)
#define HDR40_PIN16_GPIO	TEGRA_GPIO(E, 5)
#define HDR40_PIN18_GPIO	TEGRA_GPIO(X, 0)
#define HDR40_PIN19_GPIO	TEGRA_GPIO(C, 0)
#define HDR40_PIN21_GPIO	TEGRA_GPIO(C, 1)
#define HDR40_PIN23_GPIO	TEGRA_GPIO(C, 2)
#define HDR40_PIN24_GPIO	TEGRA_GPIO(C, 3)
#define HDR40_PIN26_GPIO	TEGRA_GPIO(C, 4)
#define HDR40_PIN29_GPIO	TEGRA_GPIO(BB, 3)
#define HDR40_PIN31_GPIO	TEGRA_GPIO(X, 2)
#define HDR40_PIN32_GPIO	TEGRA_GPIO(E, 4)
#define HDR40_PIN33_GPIO	TEGRA_GPIO(H, 7)
#define HDR40_PIN35_GPIO	TEGRA_GPIO(B, 0)
#define HDR40_PIN36_GPIO	TEGRA_GPIO(U, 3)
#define HDR40_PIN37_GPIO	TEGRA_GPIO(X, 3)
#define HDR40_PIN38_GPIO	TEGRA_GPIO(B, 1)
#define HDR40_PIN40_GPIO	TEGRA_GPIO(B, 2)
