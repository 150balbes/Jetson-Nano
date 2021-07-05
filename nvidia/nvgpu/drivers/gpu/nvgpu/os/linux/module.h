/*
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_COMMON_LINUX_MODULE_H__
#define __NVGPU_COMMON_LINUX_MODULE_H__

struct gk20a;
struct device;
struct nvgpu_os_linux;

int gk20a_pm_finalize_poweron(struct device *dev);
int nvgpu_finalize_poweron_linux(struct nvgpu_os_linux *l);
void gk20a_remove_support(struct gk20a *g);
void gk20a_driver_start_unload(struct gk20a *g);
int nvgpu_quiesce(struct gk20a *g);
int nvgpu_remove(struct device *dev, struct class *class);
void nvgpu_free_irq(struct gk20a *g);
struct device_node *nvgpu_get_node(struct gk20a *g);
void __iomem *nvgpu_devm_ioremap_resource(struct platform_device *dev, int i,
		struct resource **out);
void __iomem *nvgpu_devm_ioremap(struct device *dev, resource_size_t offset,
		resource_size_t size);
u64 nvgpu_resource_addr(struct platform_device *dev, int i);
extern struct class nvgpu_class;

#endif
