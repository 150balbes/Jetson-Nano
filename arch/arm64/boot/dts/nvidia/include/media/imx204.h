/**
 * Copyright (c) 2015-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __IMX204_H__
#define __IMX204_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define	IMX204_CHIP_ID				0x81
#define	IMX204_CLK_PER_INT_OFFSET	104
#define	IMX204_SHR_MIN				8
#define	IMX204_SPL					0x00
#define	IMX204_INPUT_CLK			72000000

#define	IMX204_XHS_PER_XVS			3000
#define	IMX204_CLK_PER_XHS_60FPS	400
#define	IMX204_CLK_PER_XHS_30FPS	800

#define	IMX204_SVR_ADDR_LSB			0x000D
#define	IMX204_SVR_ADDR_MSB			0x000E

/* IMX204 didn't support FrameLength
 * This is only for storing the data for SVR calculation
 */
#define	IMX204_FRAME_LENGTH_ADDR_LSB	0xFEFE
#define	IMX204_FRAME_LENGTH_ADDR_MSB	0xFEFF

#define	IMX204_PGC_ADDR_LSB			0x0009
#define	IMX204_PGC_ADDR_MSB			0x000A
#define	IMX204_PGC_MSB_MASK			0x0007

#define	IMX204_SHR_ADDR_LSB			0x000B
#define	IMX204_SHR_ADDR_MSB			0x000C

#ifdef __KERNEL__
struct imx204_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct clk *mclk;
	unsigned int reset_gpio;
};

struct imx204_platform_data {
	const char *mclk_name; /* NULL for default default_mclk */
	unsigned int reset_gpio;
	int (*power_on)(struct imx204_power_rail *pw);
	int (*power_off)(struct imx204_power_rail *pw);
};

#endif /* __KERNEL__ */

#endif  /* __IMX204_H__ */
