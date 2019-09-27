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
#ifndef NVGPU_IOCTL_TSG_H
#define NVGPU_IOCTL_TSG_H

struct inode;
struct file;
struct gk20a;
struct nvgpu_ref;

int nvgpu_ioctl_tsg_dev_release(struct inode *inode, struct file *filp);
int nvgpu_ioctl_tsg_dev_open(struct inode *inode, struct file *filp);
int nvgpu_ioctl_tsg_open(struct gk20a *g, struct file *filp);
long nvgpu_ioctl_tsg_dev_ioctl(struct file *filp,
			       unsigned int cmd, unsigned long arg);
void nvgpu_ioctl_tsg_release(struct nvgpu_ref *ref);

#endif
