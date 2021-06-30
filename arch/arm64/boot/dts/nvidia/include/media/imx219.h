/**
 * Copyright (c) 2014-2019, NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property
 * and proprietary rights in and to this software and related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA Corporation is strictly prohibited.
 */

#ifndef __IMX219_H__
#define __IMX219_H__

#include <uapi/media/imx219.h>

#define IMX219_FUSE_ID_SIZE		6
#define IMX219_FUSE_ID_STR_SIZE		(IMX219_FUSE_ID_SIZE * 2)

struct imx219_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vdd_af;
};

struct imx219_platform_data {
	struct imx219_flash_control flash_cap;
	const char *mclk_name; /* NULL for default default_mclk */
	int (*power_on)(struct imx219_power_rail *pw);
	int (*power_off)(struct imx219_power_rail *pw);
};

#endif  /* __IMX219_H__ */
