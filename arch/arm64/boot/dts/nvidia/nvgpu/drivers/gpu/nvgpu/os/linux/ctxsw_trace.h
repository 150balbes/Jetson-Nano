/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __CTXSW_TRACE_H__
#define __CTXSW_TRACE_H__

#include <nvgpu/types.h>

#define GK20A_CTXSW_TRACE_NUM_DEVS			1

struct file;
struct inode;
struct poll_table_struct;

struct gk20a;

int gk20a_ctxsw_dev_release(struct inode *inode, struct file *filp);
int gk20a_ctxsw_dev_open(struct inode *inode, struct file *filp);
long gk20a_ctxsw_dev_ioctl(struct file *filp,
			 unsigned int cmd, unsigned long arg);
ssize_t gk20a_ctxsw_dev_read(struct file *filp, char __user *buf,
			     size_t size, loff_t *offs);
unsigned int gk20a_ctxsw_dev_poll(struct file *filp,
				  struct poll_table_struct *pts);

#endif /* __CTXSW_TRACE_H__ */
