/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_IOCTL_H__
#define __NVGPU_IOCTL_H__

struct device;
struct class;

int gk20a_user_init(struct device *dev, const char *interface_name,
		    struct class *class);
void gk20a_user_deinit(struct device *dev, struct class *class);

#endif
