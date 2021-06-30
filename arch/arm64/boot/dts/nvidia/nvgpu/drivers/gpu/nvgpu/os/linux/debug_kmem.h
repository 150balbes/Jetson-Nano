/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __NVGPU_DEBUG_KMEM_H__
#define __NVGPU_DEBUG_KMEM_H__

struct gk20a;
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
void nvgpu_kmem_debugfs_init(struct gk20a *g);
#endif

#endif /* __NVGPU_DEBUG_KMEM_H__ */
