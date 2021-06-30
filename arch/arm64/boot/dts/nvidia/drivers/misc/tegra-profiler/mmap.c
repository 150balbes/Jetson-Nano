/*
 * drivers/misc/tegra-profiler/mmap.c
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/mm.h>
#include <linux/sched/task.h>
#include <linux/sched/signal.h>
#endif

#include <linux/tegra_profiler.h>

#include "quadd.h"
#include "mmap.h"
#include "comm.h"
#include "hrt.h"
#include "tegra.h"

#define TMP_BUFFER_SIZE			(PATH_MAX)
#define QUADD_MMAP_TREE_MAX_LEVEL	32

static void
put_mmap_sample(struct quadd_mmap_data *s, char *filename,
		size_t length, unsigned long pgoff, pid_t __tgid)
{
	int vec_idx = 0;
	unsigned long flags;
	struct quadd_record_data r;
	struct quadd_iovec vec[3];
	u64 pgoff_val = (u64)pgoff << PAGE_SHIFT;
	u32 tgid = (u32)__tgid;

	r.record_type = QUADD_RECORD_TYPE_MMAP;

	memcpy(&r.mmap, s, sizeof(*s));

	raw_local_irq_save(flags);

	r.mmap.time = quadd_get_time();
	r.mmap.filename_length = length;

	vec[vec_idx].base = &pgoff_val;
	vec[vec_idx].len = sizeof(pgoff_val);
	vec_idx++;

	vec[vec_idx].base = filename;
	vec[vec_idx].len = length;
	vec_idx++;

	vec[vec_idx].base = &tgid;
	vec[vec_idx].len = sizeof(tgid);
	vec_idx++;

	pr_debug("[%d] MMAP: tid: %u,pid: %u,'%s',%#llx-%#llx(%llx,%#llx)\n",
		 s->cpu_id, s->pid, tgid, filename,
		 s->addr, s->addr + s->len, s->len, pgoff_val);

	quadd_put_sample_this_cpu(&r, vec, vec_idx);

	raw_local_irq_restore(flags);
}

static void
process_mmap(struct vm_area_struct *vma, struct task_struct *task,
	     char *buf, size_t buf_size)
{
	pid_t tgid;
	unsigned int cpu_flags;
	int is_file_exists;
	struct file *vm_file;
	char *file_name;
	struct quadd_mmap_data sample;
	size_t length, length_aligned;

	if (!(vma->vm_flags & VM_EXEC))
		return;

	vm_file = vma->vm_file;
	if (vm_file) {
		file_name = file_path(vm_file, buf, buf_size - sizeof(u64));
		if (IS_ERR(file_name))
			return;

		length = strlen(file_name) + 1;
		is_file_exists = 1;
	} else {
		const char *name = NULL;

		name = arch_vma_name(vma);
		if (!name) {
			struct mm_struct *mm = vma->vm_mm;

			if (!mm) {
				name = "[vdso]";
			} else if (vma->vm_start <= mm->start_brk &&
				   vma->vm_end >= mm->brk) {
				name = "[heap]";
			} else if (vma->vm_start <= mm->start_stack &&
				   vma->vm_end >= mm->start_stack) {
				name = "[stack]";
			}
		}

		if (name)
			strlcpy(buf, name, buf_size);
		else
			snprintf(buf, buf_size, "[vma:%08lx-%08lx]",
				 vma->vm_start, vma->vm_end);

		file_name = buf;
		length = strlen(file_name) + 1;

		is_file_exists = 0;
	}

	length_aligned = ALIGN(length, sizeof(u64));
	memset(&file_name[length - 1], 0, length_aligned - length + 1);

	sample.pid = task_pid_nr(task);
	tgid = task_tgid_nr(task);

	sample.addr = vma->vm_start;
	sample.len = vma->vm_end - vma->vm_start;

	sample.flags = 0;

	sample.cpu_id = quadd_get_processor_id(NULL, &cpu_flags);
	if (cpu_flags & QUADD_CPUMODE_TEGRA_POWER_CLUSTER_LP)
		sample.flags |= QUADD_MMAP_FLAG_LP_MODE;

	if (is_file_exists)
		sample.flags |= QUADD_MMAP_FLAG_IS_FILE_EXISTS;

	sample.flags |= QUADD_MMAP_FLAG_USER_MODE;

	put_mmap_sample(&sample, file_name, length_aligned,
			vma->vm_pgoff, tgid);
}

void quadd_process_mmap(struct vm_area_struct *vma, struct task_struct *task)
{
	char *buf;

	buf = kmalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		return;

	preempt_disable();
	process_mmap(vma, task, buf, TMP_BUFFER_SIZE);
	preempt_enable();

	kfree(buf);
}

static void get_process_vmas(struct task_struct *task)
{
	char *buf;
	struct mm_struct *mm;
	struct vm_area_struct *vma;

	if (!task)
		return;

	mm = get_task_mm(task);
	if (!mm)
		return;

	down_read(&mm->mmap_sem);

	buf = kmalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		goto out_put_mm;

	preempt_disable();
	for (vma = mm->mmap; vma; vma = vma->vm_next)
		process_mmap(vma, task, buf, TMP_BUFFER_SIZE);
	preempt_enable();

	kfree(buf);

out_put_mm:
	up_read(&mm->mmap_sem);
	mmput(mm);
}

static void
__get_process_vmas(struct task_struct *task,
		   struct mm_struct *mm, char *buf, size_t buf_size)
{
	struct vm_area_struct *vma;

	for (vma = mm->mmap; vma; vma = vma->vm_next)
		process_mmap(vma, task, buf, buf_size);
}

static void get_all_processes(bool is_root_pid)
{
	char *buf;
	struct task_struct *p;

	buf = kmalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
	if (!buf)
		return;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct mm_struct *mm;

		if (p->flags & PF_KTHREAD)
			continue;

		if (is_root_pid && !quadd_is_inherited(p))
			continue;

		task_lock(p);

		if (unlikely(!p->mm))
			goto __continue;
		mm = p->mm;

		if (!down_read_trylock(&mm->mmap_sem))
			goto __continue;

		__get_process_vmas(p, mm, buf, TMP_BUFFER_SIZE);

		up_read(&mm->mmap_sem);
__continue:
		task_unlock(p);
	}
	read_unlock(&tasklist_lock);

	kfree(buf);
}

void quadd_get_mmaps(struct quadd_ctx *ctx)
{
	struct task_struct *task;
	struct quadd_parameters *param = &ctx->param;

	if (!quadd_mode_is_sampling(ctx))
		return;

	if (quadd_mode_is_sample_all(ctx)) {
		get_all_processes(false);
		return;
	}

	rcu_read_lock();
	task = get_pid_task(find_vpid(param->pids[0]), PIDTYPE_PID);
	rcu_read_unlock();
	if (task) {
		if (quadd_mode_is_sample_tree(ctx))
			get_all_processes(true);
		else
			get_process_vmas(task);

		put_task_struct(task);
	}
}

void quadd_get_task_mmaps(struct quadd_ctx *ctx, struct task_struct *task)
{
	char *buf;
	struct mm_struct *mm;

	if (!quadd_mode_is_sampling(ctx))
		return;

	if (task->flags & PF_KTHREAD)
		return;

	task_lock(task);

	mm = task->mm;
	if (!mm)
		goto out_task_unlock;

	if (down_read_trylock(&mm->mmap_sem)) {
		buf = kmalloc(TMP_BUFFER_SIZE, GFP_ATOMIC);
		if (buf) {
			__get_process_vmas(task, mm, buf, TMP_BUFFER_SIZE);
			kfree(buf);
		}
		up_read(&mm->mmap_sem);
	}

out_task_unlock:
	task_unlock(task);
}
