/*
 * sound/soc/aml/m8/aml_m8.h
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#ifndef AML_M8_H
#define AML_M8_H

#include <sound/soc.h>
#include <linux/gpio/consumer.h>
struct dac2_private_data {
	int bias_level;
	const char *pinctrl_name;
	struct pinctrl *pin_ctl;
	void *data;
};

#endif

