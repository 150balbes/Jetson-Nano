/**
 * Copyright (c) 2018-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef __MAX9296_H__
#define __MAX9296_H__

#include <media/gmsl-link.h>

int max9296_setup_link(struct device *dev, struct device *s_dev);

int max9296_setup_control(struct device *dev);

int max9296_reset_control(struct device *dev, struct device *s_dev);

int max9296_sdev_register(struct device *dev, struct gmsl_link_ctx *g_ctx);

int max9296_sdev_unregister(struct device *dev, struct device *s_dev);

int max9296_setup_streaming(struct device *dev, struct device *s_dev);

int max9296_start_streaming(struct device *dev, struct device *s_dev);

int max9296_stop_streaming(struct device *dev, struct device *s_dev);

int max9296_power_on(struct device *dev);

void max9296_power_off(struct device *dev);

#endif  /* __MAX9296_H__ */
