/*
 * drivers/video/tegra/nvmap/nvmap_ioctl.h
 *
 * ioctl declarations for nvmap
 *
 * Copyright (c) 2010-2020, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __VIDEO_TEGRA_NVMAP_IOCTL_H
#define __VIDEO_TEGRA_NVMAP_IOCTL_H

#include <linux/nvmap.h>

int nvmap_ioctl_pinop(struct file *filp, bool is_pin, void __user *arg,
	bool is32);

int nvmap_ioctl_getid(struct file *filp, void __user *arg);

int nvmap_ioctl_get_ivcid(struct file *filp, void __user *arg);

int nvmap_ioctl_getfd(struct file *filp, void __user *arg);

int nvmap_ioctl_alloc(struct file *filp, void __user *arg);

int nvmap_ioctl_alloc_kind(struct file *filp, void __user *arg);

int nvmap_ioctl_alloc_ivm(struct file *filp, void __user *arg);

int nvmap_ioctl_vpr_floor_size(struct file *filp, void __user *arg);

int nvmap_ioctl_free(struct file *filp, unsigned long arg);

int nvmap_ioctl_create(struct file *filp, unsigned int cmd, void __user *arg);

int nvmap_ioctl_create_from_va(struct file *filp, void __user *arg);

int nvmap_ioctl_create_from_ivc(struct file *filp, void __user *arg);

int nvmap_ioctl_get_ivc_heap(struct file *filp, void __user *arg);

int nvmap_map_into_caller_ptr(struct file *filp, void __user *arg, bool is32);

int nvmap_ioctl_cache_maint(struct file *filp, void __user *arg, int size);

int nvmap_ioctl_rw_handle(struct file *filp, int is_read, void __user *arg,
	size_t op_size);

int nvmap_ioctl_cache_maint_list(struct file *filp, void __user *arg,
	bool is_rsrv_op);

int nvmap_ioctl_gup_test(struct file *filp, void __user *arg);

int nvmap_ioctl_set_tag_label(struct file *filp, void __user *arg);

int nvmap_ioctl_get_available_heaps(struct file *filp, void __user *arg);

int nvmap_ioctl_get_heap_size(struct file *filp, void __user *arg);

int nvmap_ioctl_query_heap_params(struct file *filp, void __user *arg);

int nvmap_ioctl_query_handle_parameters(struct file *filp, void __user *arg);

#endif	/*  __VIDEO_TEGRA_NVMAP_IOCTL_H */
