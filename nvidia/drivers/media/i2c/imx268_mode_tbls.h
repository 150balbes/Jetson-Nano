/*
 * imx268_mode_tbls.h - imx268 sensor mode tables
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __IMX268_I2C_TABLES__
#define __IMX268_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

#define IMX268_TABLE_WAIT_MS 0
#define IMX268_TABLE_END 1
#define imx268_reg struct reg_8

static imx268_reg imx268_start[] = {
	{0x0100, 0x01},
	{IMX268_TABLE_END, 0x00}
};

static imx268_reg imx268_stop[] = {
	{0x0100, 0x00},
	{IMX268_TABLE_END, 0x00}
};

static imx268_reg imx268_common[] = {
	/* External clock setting */
	{0x0136, 0x18},
	{0x0137, 0x00},

	/* Global setting */
	{0x4E21, 0x04},
	{0x6B01, 0xB0},
	{0x6B02, 0xED},
	{0x6B05, 0x66},
	{0x6B06, 0xFB},
	{0x6B18, 0x3F},
	{0x6B19, 0xFF},
	{0x6541, 0x01},

	{IMX268_TABLE_END, 0x00}
};

static imx268_reg imx268_3872x2192_30fps[] = {
	/* Output format setting */
	{0x0112, 0x0A},
	{0x0113, 0x0A},
	{0x0114, 0x03},

	/* Clock setting */
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x00},
	{0x0307, 0xFA},
	{0x0309, 0x0A},
	{0x030B, 0x02},
	{0x030D, 0x00},
	{0x030E, 0x00},
	{0x030F, 0x00},
	{0x0310, 0x00},
	{0x0820, 0x0B},
	{0x0821, 0xB8},
	{0x0822, 0x00},
	{0x0823, 0x00},

	/* Line length setting */
	{0x0342, 0x10},
	{0x0343, 0x68},

	/* Frame length setting */
	{0x0340, 0x09},
	{0x0341, 0x44},

	/* ROI setting */
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0F},
	{0x0349, 0x1F},
	{0x034A, 0x08},
	{0x034B, 0x8F},

	/* Analog image size setting */
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},

	/* Digital image size setting */
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040A, 0x00},
	{0x040B, 0x00},
	{0x040C, 0x0F},
	{0x040D, 0x20},
	{0x040E, 0x08},
	{0x040F, 0x90},
	{0x300D, 0x00},

	/* Output size setting */
	{0x034C, 0x0F}, /* X_OUT_SIZE[12:8] */
	{0x034D, 0x20}, /* X_OUT_SIZE[7:0] */
	{0x034E, 0x08}, /* Y_OUT_SIZE[11:8] */
	{0x034F, 0x90}, /* Y_OUT_SIZE[7:8] */
	{0x4041, 0x00}, /* EBD_SIZE_V */

	/* Integration time setting */
	{0x0202, 0x09},
	{0x0203, 0x3A},

	/* Gain setting */
	{0x0204, 0x00},
	{0x0205, 0x00},
	{0x020E, 0x01},
	{0x020F, 0x00},
	{0x0210, 0x01},
	{0x0211, 0x00},
	{0x0212, 0x01},
	{0x0213, 0x00},
	{0x0214, 0x01},
	{0x0215, 0x00},

	/* Added setting (IQ) */
	{0x6568, 0x00},
	{0x6B11, 0x8F},
	{0x6B12, 0xAB},
	{0x6B13, 0xAA},

	/* Added setting (mode) */
	{0x0220, 0x00},
	{0x0B00, 0x00},

	{IMX268_TABLE_END, 0x00}
};

enum {
	IMX268_MODE_3872X2192_30FPS,

	IMX268_MODE_COMMON,
	IMX268_MODE_STREAM_START,
	IMX268_MODE_STREAM_STOP
};

static imx268_reg *mode_table[] = {
	[IMX268_MODE_3872X2192_30FPS] = imx268_3872x2192_30fps,

	[IMX268_MODE_COMMON] = imx268_common,
	[IMX268_MODE_STREAM_START] = imx268_start,
	[IMX268_MODE_STREAM_STOP] = imx268_stop
};

static const int imx268_30fps[] = {
	30,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt imx268_frmfmt[] = {
	{{3872, 2192}, imx268_30fps, 1, 0, IMX268_MODE_3872X2192_30FPS},
	/* Add modes with no device tree support after below */
};

#endif /* __IMX268_I2C_TABLES__ */
