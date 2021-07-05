/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_IOCTL_CHANNEL_H__
#define __NVGPU_IOCTL_CHANNEL_H__

#include <linux/fs.h>

#include "gk20a/css_gr_gk20a.h"

struct inode;
struct file;
struct gk20a;
struct nvgpu_channel_open_args;

struct gk20a_cs_snapshot_client_linux {
	struct gk20a_cs_snapshot_client cs_client;

	u32			dmabuf_fd;
	struct dma_buf		*dma_handler;
};

int gk20a_channel_open(struct inode *inode, struct file *filp);
int gk20a_channel_release(struct inode *inode, struct file *filp);
long gk20a_channel_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg);
int gk20a_channel_open_ioctl(struct gk20a *g,
		struct nvgpu_channel_open_args *args);

int gk20a_channel_cycle_stats(struct channel_gk20a *ch, int dmabuf_fd);
void gk20a_channel_free_cycle_stats_buffer(struct channel_gk20a *ch);

int gk20a_attach_cycle_stats_snapshot(struct channel_gk20a *ch,
				u32 dmabuf_fd,
				u32 perfmon_id_count,
				u32 *perfmon_id_start);
int gk20a_flush_cycle_stats_snapshot(struct channel_gk20a *ch);
int gk20a_channel_free_cycle_stats_snapshot(struct channel_gk20a *ch);

extern const struct file_operations gk20a_channel_ops;

u32 nvgpu_get_common_runlist_level(u32 level);

u32 nvgpu_get_ioctl_graphics_preempt_mode_flags(u32 graphics_preempt_mode_flags);
u32 nvgpu_get_ioctl_compute_preempt_mode_flags(u32 compute_preempt_mode_flags);
u32 nvgpu_get_ioctl_graphics_preempt_mode(u32 graphics_preempt_mode);
u32 nvgpu_get_ioctl_compute_preempt_mode(u32 compute_preempt_mode);
#endif
