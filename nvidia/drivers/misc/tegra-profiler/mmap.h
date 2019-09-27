/*
 * drivers/misc/tegra-profiler/mmap.h
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
 */

#ifndef __QUADD_MMAP_H
#define __QUADD_MMAP_H

struct vm_area_struct;
struct task_struct;
struct quadd_ctx;

void quadd_process_mmap(struct vm_area_struct *vma, struct task_struct *task);
void quadd_get_mmaps(struct quadd_ctx *ctx);
void quadd_get_task_mmaps(struct quadd_ctx *ctx, struct task_struct *task);

#endif  /* __QUADD_MMAP_H */
