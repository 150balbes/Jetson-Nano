/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef __NVGPU_SCHED_H
#define __NVGPU_SCHED_H

struct gk20a;
struct gpu_ops;
struct tsg_gk20a;
struct poll_table_struct;

int gk20a_sched_dev_release(struct inode *inode, struct file *filp);
int gk20a_sched_dev_open(struct inode *inode, struct file *filp);
long gk20a_sched_dev_ioctl(struct file *, unsigned int, unsigned long);
ssize_t gk20a_sched_dev_read(struct file *, char __user *, size_t, loff_t *);
unsigned int gk20a_sched_dev_poll(struct file *, struct poll_table_struct *);

void gk20a_sched_ctrl_tsg_added(struct gk20a *, struct tsg_gk20a *);
void gk20a_sched_ctrl_tsg_removed(struct gk20a *, struct tsg_gk20a *);
int gk20a_sched_ctrl_init(struct gk20a *);

void gk20a_sched_ctrl_cleanup(struct gk20a *g);

#endif /* __NVGPU_SCHED_H */
