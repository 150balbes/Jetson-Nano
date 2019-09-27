/*
 * include/linux/tegra_profiler.h
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA_PROFILER_H
#define __TEGRA_PROFILER_H

#include <uapi/linux/tegra_profiler.h>

struct task_struct;
struct vm_area_struct;

#ifdef CONFIG_TEGRA_PROFILER
extern void __quadd_task_sched_in(struct task_struct *prev,
				  struct task_struct *task);
extern void __quadd_task_sched_out(struct task_struct *prev,
				   struct task_struct *next);

extern void __quadd_event_mmap(struct vm_area_struct *vma);
extern void __quadd_event_fork(struct task_struct *task);
extern void __quadd_event_exit(struct task_struct *task);
extern void __quadd_event_comm(struct task_struct *task, bool exec);

static inline void quadd_task_sched_in(struct task_struct *prev,
				       struct task_struct *task)
{
	__quadd_task_sched_in(prev, task);
}

static inline void quadd_task_sched_out(struct task_struct *prev,
					struct task_struct *next)
{
	__quadd_task_sched_out(prev, next);
}

static inline void quadd_event_mmap(struct vm_area_struct *vma)
{
	__quadd_event_mmap(vma);
}

static inline void quadd_event_fork(struct task_struct *task)
{
	__quadd_event_fork(task);
}

static inline void quadd_event_exit(struct task_struct *task)
{
	__quadd_event_exit(task);
}

static inline void quadd_event_comm(struct task_struct *task, bool exec)
{
	__quadd_event_comm(task, exec);
}

#else	/* CONFIG_TEGRA_PROFILER */

static inline void quadd_task_sched_in(struct task_struct *prev,
				       struct task_struct *task)
{
}

static inline void quadd_task_sched_out(struct task_struct *prev,
					struct task_struct *next)
{
}

static inline void quadd_event_mmap(struct vm_area_struct *vma)
{
}

static inline void quadd_event_fork(struct task_struct *task)
{
}

static inline void quadd_event_exit(struct task_struct *task)
{
}

static inline void quadd_event_comm(struct task_struct *task, bool exec)
{
}

#endif	/* CONFIG_TEGRA_PROFILER */

#endif  /* __TEGRA_PROFILER_H */
