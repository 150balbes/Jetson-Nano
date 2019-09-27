/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_LINUX_INTR_H__
#define __NVGPU_LINUX_INTR_H__
struct gk20a;

irqreturn_t nvgpu_intr_stall(struct gk20a *g);
irqreturn_t nvgpu_intr_thread_stall(struct gk20a *g);
irqreturn_t nvgpu_intr_nonstall(struct gk20a *g);
void nvgpu_intr_nonstall_cb(struct work_struct *work);
#endif
