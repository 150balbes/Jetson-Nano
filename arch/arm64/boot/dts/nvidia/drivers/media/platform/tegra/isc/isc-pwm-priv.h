/*
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
 */

#ifndef __ISC_PWM_PRIV_H__
#define __ISC_PWM_PRIV_H__

struct isc_pwm_info {
	struct pwm_chip chip;
	struct pwm_device *pwm;
	atomic_t in_use;
	struct mutex mutex;
	bool force_on;
};

#endif  /* __ISC_PWM_PRIV_H__ */
