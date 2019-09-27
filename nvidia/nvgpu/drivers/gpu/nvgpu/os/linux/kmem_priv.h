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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __KMEM_PRIV_H__
#define __KMEM_PRIV_H__

#include <nvgpu/rbtree.h>
#include <nvgpu/lock.h>

struct seq_file;

#define __pstat(s, fmt, msg...)				\
	do {						\
		if (s)					\
			seq_printf(s, fmt, ##msg);	\
		else					\
			pr_info(fmt, ##msg);		\
	} while (0)

#define MAX_STACK_TRACE				20

/*
 * Linux specific version of the nvgpu_kmem_cache struct. This type is
 * completely opaque to the rest of the driver.
 */
struct nvgpu_kmem_cache {
	struct gk20a *g;
	struct kmem_cache *cache;

	/*
	 * Memory to hold the kmem_cache unique name. Only necessary on our
	 * k3.10 kernel when not using the SLUB allocator but it's easier to
	 * just carry this on to newer kernels.
	 */
	char name[128];
};

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE

struct nvgpu_mem_alloc {
	struct nvgpu_mem_alloc_tracker *owner;

	void *ip;
#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	unsigned long stack[MAX_STACK_TRACE];
	int stack_length;
#endif

	u64 addr;

	unsigned long size;
	unsigned long real_size;

	struct nvgpu_rbtree_node allocs_entry;
};

static inline struct nvgpu_mem_alloc *
nvgpu_mem_alloc_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_mem_alloc *)
	((uintptr_t)node - offsetof(struct nvgpu_mem_alloc, allocs_entry));
};

/*
 * Linux specific tracking of vmalloc, kmalloc, etc.
 */
struct nvgpu_mem_alloc_tracker {
	const char *name;
	struct nvgpu_kmem_cache *allocs_cache;
	struct nvgpu_rbtree_node *allocs;
	struct nvgpu_mutex lock;

	u64 bytes_alloced;
	u64 bytes_freed;
	u64 bytes_alloced_real;
	u64 bytes_freed_real;
	u64 nr_allocs;
	u64 nr_frees;

	unsigned long min_alloc;
	unsigned long max_alloc;
};

void nvgpu_lock_tracker(struct nvgpu_mem_alloc_tracker *tracker);
void nvgpu_unlock_tracker(struct nvgpu_mem_alloc_tracker *tracker);

void kmem_print_mem_alloc(struct gk20a *g,
			 struct nvgpu_mem_alloc *alloc,
			 struct seq_file *s);
#endif /* CONFIG_NVGPU_TRACK_MEM_USAGE */

#endif /* __KMEM_PRIV_H__ */
