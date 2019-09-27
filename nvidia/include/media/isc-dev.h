/*
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __ISC_DEV_H__
#define __ISC_DEV_H__

#include <uapi/media/isc-dev.h>
#include <linux/regmap.h>

#define MAX_ISC_NAME_LENGTH	32

struct isc_dev_platform_data {
	struct device *pdev; /* parent device of isc_dev */
	int reg_bits;
	int val_bits;
	char drv_name[MAX_ISC_NAME_LENGTH];
};

#endif  /* __ISC_DEV_H__ */
