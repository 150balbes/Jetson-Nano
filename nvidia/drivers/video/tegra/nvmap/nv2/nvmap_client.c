/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
 * Copyright (c) 2009-2017, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/platform/tegra/tegra_fd.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_handle_ref.h"
#include "nvmap_handle.h"
#include "nvmap_dmabuf.h"
#include "nvmap_client.h"
#include "nvmap_dev.h"
#include "nvmap_stats.h"

struct nvmap_client {
	const char			*name;
	struct rb_root			handle_refs;
	struct mutex			ref_lock;
	bool				kernel_client;
	atomic_t			count;
	struct task_struct		*task;
	struct list_head		list;
	u32				handle_count;
	u32				next_fd;
	int				warned;
	int				tag_warned;
};

extern bool dmabuf_is_nvmap(struct dma_buf *dmabuf);
extern u32 nvmap_max_handle_count;

struct nvmap_client *nvmap_client_create(struct list_head *dev_client_list,
		const char *name)
{
	struct nvmap_client *client;
	struct task_struct *task;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return NULL;

	client->name = name;
	client->kernel_client = true;
	client->handle_refs = RB_ROOT;

	get_task_struct(current->group_leader);
	task_lock(current->group_leader);
	/* don't bother to store task struct for kernel threads,
	   they can't be killed anyway */
	if (current->flags & PF_KTHREAD) {
		put_task_struct(current->group_leader);
		task = NULL;
	} else {
		task = current->group_leader;
	}
	task_unlock(current->group_leader);
	client->task = task;

	mutex_init(&client->ref_lock);
	atomic_set(&client->count, 1);

	list_add(&client->list, dev_client_list);

	trace_nvmap_open(client, client->name);

	client->kernel_client = false;

	return client;
}

void nvmap_client_destroy(struct nvmap_client *client)
{
	struct rb_node *n;

	if (!client)
		return;

	while ((n = rb_first(&client->handle_refs))) {
		struct nvmap_handle_ref *ref;
		int count;

		ref = rb_entry(n, struct nvmap_handle_ref, node);
		smp_rmb();

		count = nvmap_handle_ref_count(ref);

		while (count--)
			nvmap_client_remove_handle(client, ref->handle);
	}

	if (client->task)
		put_task_struct(client->task);

	trace_nvmap_release(client, client->name);
	kfree(client);
}

const char *nvmap_client_name(struct nvmap_client *client)
{
	return client->name;
}

static void client_lock(struct nvmap_client *c)
{
	mutex_lock(&c->ref_lock);
}

static void client_unlock(struct nvmap_client *c)
{
	mutex_unlock(&c->ref_lock);
}

pid_t nvmap_client_pid(struct nvmap_client *client)
{
	return client->task ? client->task->pid : 0;
}

void nvmap_client_stats_alloc(struct nvmap_client *client, size_t size)
{
	if (client->kernel_client)
		nvmap_stats_inc(NS_KALLOC, size);
	else
		nvmap_stats_inc(NS_UALLOC, size);
}

int nvmap_client_add_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;

	ref = nvmap_client_to_handle_ref(client, handle);
	if (ref) {
		nvmap_handle_ref_get(ref);
		return 0;
	}

	ref = nvmap_handle_ref_create(handle);
	if (!ref) {
		return -ENOMEM;
	}

	nvmap_client_add_ref(client, ref);
	nvmap_handle_add_owner(handle, client);

	return 0;
}

void nvmap_client_remove_handle(struct nvmap_client *client,
			   struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref;
	int ref_count;

	ref = nvmap_client_to_handle_ref(client, handle);
	if (!ref)
		return;

	ref_count = nvmap_handle_ref_put(ref);
	if (ref_count == 0) {
		nvmap_client_remove_ref(client, ref);
		nvmap_handle_ref_free(ref);
		// TODO set ref->handle->owner to NULL
	}
}

int nvmap_client_create_handle(struct nvmap_client *client, size_t size)
{
	struct nvmap_handle *handle = NULL;
	int err;
	int fd;

	handle = nvmap_handle_create(size);
	if (IS_ERR_OR_NULL(handle)) {
		return -1;
	}

	err = nvmap_client_add_handle(client, handle);
	if (err) {
		nvmap_handle_put(handle);
		return -1;
	}
	/* This is the first handle ref we are creating and we want the dmabuf
	 * to have a ref of 1.
	 * client_add_handle increases the dmabuf ref so decrease it again
	 */
	dma_buf_put(nvmap_handle_to_dmabuf(handle));

	fd = nvmap_client_create_fd(client);
	if (fd < 0) {
		nvmap_client_remove_handle(client, handle);
		nvmap_handle_put(handle);
		return -1;
	}
	nvmap_handle_install_fd(handle, fd);

	return fd;
}

void nvmap_client_add_ref(struct nvmap_client *client,
			   struct nvmap_handle_ref *ref)
{
	struct rb_node **p, *parent = NULL;

	if(IS_ERR(ref)) {
		pr_warn("Putting Error Ref into client\n");
		return;
	}

	client_lock(client);
	p = &client->handle_refs.rb_node;
	while (*p) {
		struct nvmap_handle_ref *node;
		parent = *p;
		node = rb_entry(parent, struct nvmap_handle_ref, node);
		if (ref->handle > node->handle)
			p = &parent->rb_right;
		else
			p = &parent->rb_left;
	}
	rb_link_node(&ref->node, parent, p);
	rb_insert_color(&ref->node, &client->handle_refs);
	client->handle_count++;
	if (client->handle_count > nvmap_max_handle_count)
		nvmap_max_handle_count = client->handle_count;

	client_unlock(client);
}

void nvmap_client_remove_ref(struct nvmap_client *client,
					struct nvmap_handle_ref *ref)
{
	client_lock(client);

	smp_rmb();
	rb_erase(&ref->node, &client->handle_refs);
	client->handle_count--;

	client_unlock(client);
}

struct nvmap_handle_ref *nvmap_client_to_handle_ref(struct nvmap_client *client,
					struct nvmap_handle *handle)
{
	struct rb_node *n = client->handle_refs.rb_node;
	struct nvmap_handle_ref *return_ref = NULL;

	client_lock(client);

	n = client->handle_refs.rb_node;
	while (n) {
		struct nvmap_handle_ref *ref;
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		if (IS_ERR(ref)) {
			pr_warn("Ref error in client!\n");
			client_unlock(client);
			return NULL;
		}
		if (ref->handle == handle) {
			return_ref = ref;
			break;
		}
		else if ((uintptr_t)handle > (uintptr_t)ref->handle)
			n = n->rb_right;
		else
			n = n->rb_left;
	}

	client_unlock(client);
	return return_ref;

}

int nvmap_client_create_fd(struct nvmap_client *client)
{
	int flags = O_CLOEXEC;
	int start_fd = CONFIG_NVMAP_FD_START;

#ifdef CONFIG_NVMAP_DEFER_FD_RECYCLE
	if (client->next_fd < CONFIG_NVMAP_FD_START)
		client->next_fd = CONFIG_NVMAP_FD_START;
	start_fd = client->next_fd++;
	if (client->next_fd >= CONFIG_NVMAP_DEFER_FD_RECYCLE_MAX_FD)
		client->next_fd = CONFIG_NVMAP_FD_START;
#endif
	/* Allocate fd from start_fd(>=1024) onwards to overcome
	 * __FD_SETSIZE limitation issue for select(),
	 * pselect() syscalls.
	 */
	// TODO: What is this current?
	return tegra_alloc_fd(current->files, start_fd, flags);
}

int nvmap_client_give_dmabuf_new_fd(struct nvmap_client *client,
				struct dma_buf *dmabuf)
{
	int fd;

	fd = nvmap_client_create_fd(client);
	if (fd > 0)
		nvmap_dmabuf_install_fd(dmabuf, fd);
	return fd;
}

void nvmap_client_warn_if_bad_heap(struct nvmap_client *client,
				u32 heap_type, u32 userflags)
{
	if (heap_type != NVMAP_HEAP_CARVEOUT_VPR && client && !client->warned) {
		char task_comm[TASK_COMM_LEN];
		client->warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: TAG: 0x%04x WARNING: "
				"NVMAP_HANDLE_WRITE_COMBINE "
				"should be used in place of "
				"NVMAP_HANDLE_UNCACHEABLE on ARM64\n",
				client->task->pid, task_comm,
				userflags >> 16);
	}
}

void nvmap_client_warn_if_no_tag(struct nvmap_client *client,
					unsigned int flags)
{
	int tag = flags >> 16;
	char task_comm[TASK_COMM_LEN];

	if (!tag && client && !client->tag_warned) {
		client->tag_warned = 1;
		get_task_comm(task_comm, client->task);
		pr_err("PID %d: %s: WARNING: "
			"All NvMap Allocations must have a tag "
			"to identify the subsystem allocating memory."
			"Please pass the tag to the API call"
			" NvRmMemHanldeAllocAttr() or relevant. \n",
			client->task->pid, task_comm);
	}
}

/**************************************************************************
 * Client Print methods
 * ************************************************************************/

void nvmap_client_stringify(struct nvmap_client *client, struct seq_file *s)
{
	char task_comm[TASK_COMM_LEN];
	if (!client->task) {
		seq_printf(s, "%-18s %18s %8u", client->name, "kernel", 0);
		return;
	}
	get_task_comm(task_comm, client->task);
	seq_printf(s, "%-18s %18s %8u", client->name, task_comm,
		   client->task->pid);
}

void nvmap_client_allocations_stringify(struct nvmap_client *client,
				  struct seq_file *s, u32 heap_type)
{
	struct nvmap_device *dev = nvmap_dev;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *handle;
	struct rb_node *n;

	client_lock(client);
	mutex_lock(&dev->tags_lock);

	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		handle = ref->handle;

		nvmap_handle_stringify(handle, s, heap_type,
						atomic_read(&ref->dupes));
	}

	mutex_unlock(&dev->tags_lock);
	client_unlock(client);
}

void nvmap_client_maps_stringify(struct nvmap_client *client,
				struct seq_file *s, u32 heap_type)
{
	struct nvmap_device *dev = nvmap_dev;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *handle;
	struct rb_node *n;

	client_lock(client);
	mutex_lock(&dev->tags_lock);

	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		handle = ref->handle;

		nvmap_handle_maps_stringify(handle, s, heap_type,
							client->task->pid);
	}

	mutex_unlock(&dev->tags_lock);
	client_unlock(client);

}

int nvmap_client_show_by_pid(struct nvmap_client *client, struct seq_file *s,
				pid_t pid)
{
	struct rb_node *n;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *handle;
	int ret = 0;

	if (client->task->pid != pid)
		return 0;

	client_lock(client);

	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		handle = ref->handle;
		ret = nvmap_handle_pid_show(handle, s, client->task->pid);
		if (ret)
			break;
	}

	client_unlock(client);

	return 0;
}

u64 nvmap_client_calc_mss(struct nvmap_client *client, u32 heap_type)
{
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *handle;
	struct rb_node *n;
	u64 total = 0;

	client_lock(client);

	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		handle = ref->handle;

		total += nvmap_handle_share_size(handle, heap_type);
	}

	client_unlock(client);

	return total;
}

#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/backing-dev.h>
#include <linux/ptrace.h>

#define PSS_SHIFT 12

#ifndef PTRACE_MODE_READ_FSCREDS
#define PTRACE_MODE_READ_FSCREDS PTRACE_MODE_READ
#endif

struct procrank_stats {
	struct vm_area_struct *vma;
	u64 pss;
};

static int procrank_pte_entry(pte_t *pte, unsigned long addr, unsigned long end,
		struct mm_walk *walk)
{
	struct procrank_stats *mss = walk->private;
	struct vm_area_struct *vma = mss->vma;
	struct page *page = NULL;
	int mapcount;

	if (pte_present(*pte))
		page = vm_normal_page(vma, addr, *pte);
	else if (is_swap_pte(*pte)) {
		swp_entry_t swpent = pte_to_swp_entry(*pte);

		if (is_migration_entry(swpent))
			page = migration_entry_to_page(swpent);
	}

	if (!page)
		return 0;

	mapcount = page_mapcount(page);
	if (mapcount >= 2)
		mss->pss += (PAGE_SIZE << PSS_SHIFT) / mapcount;
	else
		mss->pss += (PAGE_SIZE << PSS_SHIFT);

	return 0;
}


void nvmap_client_calc_iovmm_mss(struct nvmap_client *client, u64 *pss,
				   u64 *total)
{
	struct rb_node *n;
	struct nvmap_handle_ref *ref;
	struct nvmap_handle *h;
	struct procrank_stats mss;
	struct mm_walk procrank_walk = {
		.pte_entry = procrank_pte_entry,
		.private = &mss,
	};
	struct mm_struct *mm;

	memset(&mss, 0, sizeof(mss));
	*pss = 0;
	*total = 0;

	mm = mm_access(client->task,
			PTRACE_MODE_READ_FSCREDS);

	if (!mm || IS_ERR(mm)) {
		return;
	}

	down_read(&mm->mmap_sem);
	procrank_walk.mm = mm;

	client_lock(client);

	n = rb_first(&client->handle_refs);
	for (; n != NULL; n = rb_next(n)) {
		ref = rb_entry(n, struct nvmap_handle_ref, node);
		h = ref->handle;

		*total += nvmap_handle_procrank_walk(h, &procrank_walk,
				client->task->pid);
	}

	up_read(&mm->mmap_sem);
	mmput(mm);
	*pss = (mss.pss >> PSS_SHIFT);

	client_unlock(client);
}

struct nvmap_client *nvmap_client_from_list(struct list_head *n)
{
	return list_entry(n, struct nvmap_client, list);
}

void nvmap_client_del_list(struct nvmap_client *client)
{
	list_del(&client->list);
}
