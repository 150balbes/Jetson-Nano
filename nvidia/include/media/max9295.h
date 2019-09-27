/**
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef __MAX9295_H__
#define __MAX9295_H__

#include <media/gmsl-link.h>

int max9295_setup_control(struct device *dev);

int max9295_reset_control(struct device *dev);

int max9295_sdev_pair(struct device *dev, struct gmsl_link_ctx *g_ctx);

int max9295_sdev_unpair(struct device *dev, struct device *s_dev);

int max9295_setup_streaming(struct device *dev);

#endif  /* __MAX9295_H__ */
