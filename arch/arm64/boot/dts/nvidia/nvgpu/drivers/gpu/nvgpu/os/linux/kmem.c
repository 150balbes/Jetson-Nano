/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/stacktrace.h>

#include <nvgpu/lock.h>
#include <nvgpu/kmem.h>
#include <nvgpu/atomic.h>
#include <nvgpu/bug.h>
#include <nvgpu/gk20a.h>

#include "kmem_priv.h"

/*
 * Statically declared because this needs to be shared across all nvgpu driver
 * instances. This makes sure that all kmem caches are _definitely_ uniquely
 * named.
 */
static atomic_t kmem_cache_id;

void *__nvgpu_big_alloc(struct gk20a *g, size_t size, bool clear)
{
	void *p;

	if (size > PAGE_SIZE) {
		if (clear)
			p = nvgpu_vzalloc(g, size);
		else
			p = nvgpu_vmalloc(g, size);
	} else {
		if (clear)
			p = nvgpu_kzalloc(g, size);
		else
			p = nvgpu_kmalloc(g, size);
	}

	return p;
}

void nvgpu_big_free(struct gk20a *g, void *p)
{
	/*
	 * This will have to be fixed eventually. Allocs that use
	 * nvgpu_big_[mz]alloc() will need to remember the size of the alloc
	 * when freeing.
	 */
	if (is_vmalloc_addr(p))
		nvgpu_vfree(g, p);
	else
		nvgpu_kfree(g, p);
}

void *__nvgpu_kmalloc(struct gk20a *g, size_t size, void *ip)
{
	void *alloc;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	alloc = __nvgpu_track_kmalloc(g, size, ip);
#else
	alloc = kmalloc(size, GFP_KERNEL);
#endif

	kmem_dbg(g, "kmalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 size, alloc, GFP_KERNEL);

	return alloc;
}

void *__nvgpu_kzalloc(struct gk20a *g, size_t size, void *ip)
{
	void *alloc;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	alloc = __nvgpu_track_kzalloc(g, size, ip);
#else
	alloc = kzalloc(size, GFP_KERNEL);
#endif

	kmem_dbg(g, "kzalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 size, alloc, GFP_KERNEL);

	return alloc;
}

void *__nvgpu_kcalloc(struct gk20a *g, size_t n, size_t size, void *ip)
{
	void *alloc;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	alloc = __nvgpu_track_kcalloc(g, n, size, ip);
#else
	alloc = kcalloc(n, size, GFP_KERNEL);
#endif

	kmem_dbg(g, "kcalloc: size=%-6ld addr=0x%p gfp=0x%08x",
		 n * size, alloc, GFP_KERNEL);

	return alloc;
}

void *__nvgpu_vmalloc(struct gk20a *g, unsigned long size, void *ip)
{
	void *alloc;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	alloc = __nvgpu_track_vmalloc(g, size, ip);
#else
	alloc = vmalloc(size);
#endif

	kmem_dbg(g, "vmalloc: size=%-6ld addr=0x%p", size, alloc);

	return alloc;
}

void *__nvgpu_vzalloc(struct gk20a *g, unsigned long size, void *ip)
{
	void *alloc;

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	alloc = __nvgpu_track_vzalloc(g, size, ip);
#else
	alloc = vzalloc(size);
#endif

	kmem_dbg(g, "vzalloc: size=%-6ld addr=0x%p", size, alloc);

	return alloc;
}

void __nvgpu_kfree(struct gk20a *g, void *addr)
{
	kmem_dbg(g, "kfree: addr=0x%p", addr);
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	__nvgpu_track_kfree(g, addr);
#else
	kfree(addr);
#endif
}

void __nvgpu_vfree(struct gk20a *g, void *addr)
{
	kmem_dbg(g, "vfree: addr=0x%p", addr);
#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE
	__nvgpu_track_vfree(g, addr);
#else
	vfree(addr);
#endif
}

#ifdef CONFIG_NVGPU_TRACK_MEM_USAGE

void nvgpu_lock_tracker(struct nvgpu_mem_alloc_tracker *tracker)
{
	nvgpu_mutex_acquire(&tracker->lock);
}

void nvgpu_unlock_tracker(struct nvgpu_mem_alloc_tracker *tracker)
{
	nvgpu_mutex_release(&tracker->lock);
}

void kmem_print_mem_alloc(struct gk20a *g,
			 struct nvgpu_mem_alloc *alloc,
			 struct seq_file *s)
{
#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	int i;

	__pstat(s, "nvgpu-alloc: addr=0x%llx size=%ld\n",
		alloc->addr, alloc->size);
	for (i = 0; i < alloc->stack_length; i++)
		__pstat(s, "  %3d [<%p>] %pS\n", i,
			(void *)alloc->stack[i],
			(void *)alloc->stack[i]);
	__pstat(s, "\n");
#else
	__pstat(s, "nvgpu-alloc: addr=0x%llx size=%ld src=%pF\n",
		alloc->addr, alloc->size, alloc->ip);
#endif
}

static int nvgpu_add_alloc(struct nvgpu_mem_alloc_tracker *tracker,
			   struct nvgpu_mem_alloc *alloc)
{
	alloc->allocs_entry.key_start = alloc->addr;
	alloc->allocs_entry.key_end = alloc->addr + alloc->size;

	nvgpu_rbtree_insert(&alloc->allocs_entry, &tracker->allocs);
	return 0;
}

static struct nvgpu_mem_alloc *nvgpu_rem_alloc(
	struct nvgpu_mem_alloc_tracker *tracker, u64 alloc_addr)
{
	struct nvgpu_mem_alloc *alloc;
	struct nvgpu_rbtree_node *node = NULL;

	nvgpu_rbtree_search(alloc_addr, &node, tracker->allocs);
	if (!node)
		return NULL;

	alloc = nvgpu_mem_alloc_from_rbtree_node(node);

	nvgpu_rbtree_unlink(node, &tracker->allocs);

	return alloc;
}

static int __nvgpu_save_kmem_alloc(struct nvgpu_mem_alloc_tracker *tracker,
				   unsigned long size, unsigned long real_size,
				   u64 addr, void *ip)
{
	int ret;
	struct nvgpu_mem_alloc *alloc;
#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	struct stack_trace stack_trace;
#endif

	alloc = kzalloc(sizeof(*alloc), GFP_KERNEL);
	if (!alloc)
		return -ENOMEM;

	alloc->owner = tracker;
	alloc->size = size;
	alloc->real_size = real_size;
	alloc->addr = addr;
	alloc->ip = ip;

#ifdef __NVGPU_SAVE_KALLOC_STACK_TRACES
	stack_trace.max_entries = MAX_STACK_TRACE;
	stack_trace.nr_entries = 0;
	stack_trace.entries = alloc->stack;
	/*
	 * This 4 here skips the 2 function calls that happen for all traced
	 * allocs due to nvgpu:
	 *
	 *   __nvgpu_save_kmem_alloc+0x7c/0x128
	 *   __nvgpu_track_kzalloc+0xcc/0xf8
	 *
	 * And the function calls that get made by the stack trace code itself.
	 * If the trace savings code changes this will likely have to change
	 * as well.
	 */
	stack_trace.skip = 4;
	save_stack_trace(&stack_trace);
	alloc->stack_length = stack_trace.nr_entries;
#endif

	nvgpu_lock_tracker(tracker);
	tracker->bytes_alloced += size;
	tracker->bytes_alloced_real += real_size;
	tracker->nr_allocs++;

	/* Keep track of this for building a histogram later on. */
	if (tracker->max_alloc < size)
		tracker->max_alloc = size;
	if (tracker->min_alloc > size)
		tracker->min_alloc = size;

	ret = nvgpu_add_alloc(tracker, alloc);
	if (ret) {
		WARN(1, "Duplicate alloc??? 0x%llx\n", addr);
		kfree(alloc);
		nvgpu_unlock_tracker(tracker);
		return ret;
	}
	nvgpu_unlock_tracker(tracker);

	return 0;
}

static int __nvgpu_free_kmem_alloc(struct nvgpu_mem_alloc_tracker *tracker,
				   u64 addr)
{
	struct nvgpu_mem_alloc *alloc;

	nvgpu_lock_tracker(tracker);
	alloc = nvgpu_rem_alloc(tracker, addr);
	if (WARN(!alloc, "Possible double-free detected: 0x%llx!", addr)) {
		nvgpu_unlock_tracker(tracker);
		return -EINVAL;
	}

	memset((void *)alloc->addr, 0, alloc->size);

	tracker->nr_frees++;
	tracker->bytes_freed += alloc->size;
	tracker->bytes_freed_real += alloc->real_size;
	nvgpu_unlock_tracker(tracker);

	return 0;
}

static void __nvgpu_check_valloc_size(unsigned long size)
{
	WARN(size < PAGE_SIZE, "Alloc smaller than page size! (%lu)!\n", size);
}

static void __nvgpu_check_kalloc_size(size_t size)
{
	WARN(size > PAGE_SIZE, "Alloc larger than page size! (%zu)!\n", size);
}

void *__nvgpu_track_vmalloc(struct gk20a *g, unsigned long size,
			    void *ip)
{
	void *alloc = vmalloc(size);

	if (!alloc)
		return NULL;

	__nvgpu_check_valloc_size(size);

	/*
	 * Ignore the return message. If this fails let's not cause any issues
	 * for the rest of the driver.
	 */
	__nvgpu_save_kmem_alloc(g->vmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_vzalloc(struct gk20a *g, unsigned long size,
			    void *ip)
{
	void *alloc = vzalloc(size);

	if (!alloc)
		return NULL;

	__nvgpu_check_valloc_size(size);

	/*
	 * Ignore the return message. If this fails let's not cause any issues
	 * for the rest of the driver.
	 */
	__nvgpu_save_kmem_alloc(g->vmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kmalloc(struct gk20a *g, size_t size, void *ip)
{
	void *alloc = kmalloc(size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	__nvgpu_check_kalloc_size(size);

	__nvgpu_save_kmem_alloc(g->kmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kzalloc(struct gk20a *g, size_t size, void *ip)
{
	void *alloc = kzalloc(size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	__nvgpu_check_kalloc_size(size);

	__nvgpu_save_kmem_alloc(g->kmallocs, size, roundup_pow_of_two(size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void *__nvgpu_track_kcalloc(struct gk20a *g, size_t n, size_t size,
			    void *ip)
{
	void *alloc = kcalloc(n, size, GFP_KERNEL);

	if (!alloc)
		return NULL;

	__nvgpu_check_kalloc_size(n * size);

	__nvgpu_save_kmem_alloc(g->kmallocs, n * size,
				roundup_pow_of_two(n * size),
				(u64)(uintptr_t)alloc, ip);

	return alloc;
}

void __nvgpu_track_vfree(struct gk20a *g, void *addr)
{
	/*
	 * Often it is accepted practice to pass NULL pointers into free
	 * functions to save code.
	 */
	if (!addr)
		return;

	__nvgpu_free_kmem_alloc(g->vmallocs, (u64)(uintptr_t)addr);

	vfree(addr);
}

void __nvgpu_track_kfree(struct gk20a *g, void *addr)
{
	if (!addr)
		return;

	__nvgpu_free_kmem_alloc(g->kmallocs, (u64)(uintptr_t)addr);

	kfree(addr);
}

static int __do_check_for_outstanding_allocs(
	struct gk20a *g,
	struct nvgpu_mem_alloc_tracker *tracker,
	const char *type, bool silent)
{
	struct nvgpu_rbtree_node *node;
	int count = 0;

	nvgpu_rbtree_enum_start(0, &node, tracker->allocs);
	while (node) {
		struct nvgpu_mem_alloc *alloc =
			nvgpu_mem_alloc_from_rbtree_node(node);

		if (!silent)
			kmem_print_mem_alloc(g, alloc, NULL);

		count++;
		nvgpu_rbtree_enum_next(&node, node);
	}

	return count;
}

/**
 * check_for_outstanding_allocs - Count and display outstanding allocs
 *
 * @g      - The GPU.
 * @silent - If set don't print anything about the allocs.
 *
 * Dump (or just count) the number of allocations left outstanding.
 */
static int check_for_outstanding_allocs(struct gk20a *g, bool silent)
{
	int count = 0;

	count += __do_check_for_outstanding_allocs(g, g->kmallocs, "kmalloc",
						   silent);
	count += __do_check_for_outstanding_allocs(g, g->vmallocs, "vmalloc",
						   silent);

	return count;
}

static void do_nvgpu_kmem_cleanup(struct nvgpu_mem_alloc_tracker *tracker,
				  void (*force_free_func)(const void *))
{
	struct nvgpu_rbtree_node *node;

	nvgpu_rbtree_enum_start(0, &node, tracker->allocs);
	while (node) {
		struct nvgpu_mem_alloc *alloc =
			nvgpu_mem_alloc_from_rbtree_node(node);

		if (force_free_func)
			force_free_func((void *)alloc->addr);

		nvgpu_rbtree_unlink(node, &tracker->allocs);
		kfree(alloc);

		nvgpu_rbtree_enum_start(0, &node, tracker->allocs);
	}
}

/**
 * nvgpu_kmem_cleanup - Cleanup the kmem tracking
 *
 * @g          - The GPU.
 * @force_free - If set will also free leaked objects if possible.
 *
 * Cleanup all of the allocs made by nvgpu_kmem tracking code. If @force_free
 * is non-zero then the allocation made by nvgpu is also freed. This is risky,
 * though, as it is possible that the memory is still in use by other parts of
 * the GPU driver not aware that this has happened.
 *
 * In theory it should be fine if the GPU driver has been deinitialized and
 * there are no bugs in that code. However, if there are any bugs in that code
 * then they could likely manifest as odd crashes indeterminate amounts of time
 * in the future. So use @force_free at your own risk.
 */
static void nvgpu_kmem_cleanup(struct gk20a *g, bool force_free)
{
	do_nvgpu_kmem_cleanup(g->kmallocs, force_free ? kfree : NULL);
	do_nvgpu_kmem_cleanup(g->vmallocs, force_free ? vfree : NULL);
}

void nvgpu_kmem_fini(struct gk20a *g, int flags)
{
	int count;
	bool silent, force_free;

	if (!flags)
		return;

	silent = !(flags & NVGPU_KMEM_FINI_DUMP_ALLOCS);
	force_free = !!(flags & NVGPU_KMEM_FINI_FORCE_CLEANUP);

	count = check_for_outstanding_allocs(g, silent);
	nvgpu_kmem_cleanup(g, force_free);

	/*
	 * If we leak objects we can either BUG() out or just WARN(). In general
	 * it doesn't make sense to BUG() on here since leaking a few objects
	 * won't crash the kernel but it can be helpful for development.
	 *
	 * If neither flag is set then we just silently do nothing.
	 */
	if (count > 0) {
		if (flags & NVGPU_KMEM_FINI_WARN) {
			WARN(1, "Letting %d allocs leak!!\n", count);
		} else if (flags & NVGPU_KMEM_FINI_BUG) {
			nvgpu_err(g, "Letting %d allocs leak!!", count);
			BUG();
		}
	}
}

int nvgpu_kmem_init(struct gk20a *g)
{
	int err;

	g->vmallocs = kzalloc(sizeof(*g->vmallocs), GFP_KERNEL);
	g->kmallocs = kzalloc(sizeof(*g->kmallocs), GFP_KERNEL);

	if (!g->vmallocs || !g->kmallocs) {
		err = -ENOMEM;
		goto fail;
	}

	g->vmallocs->name = "vmalloc";
	g->kmallocs->name = "kmalloc";

	g->vmallocs->allocs = NULL;
	g->kmallocs->allocs = NULL;

	nvgpu_mutex_init(&g->vmallocs->lock);
	nvgpu_mutex_init(&g->kmallocs->lock);

	g->vmallocs->min_alloc = PAGE_SIZE;
	g->kmallocs->min_alloc = KMALLOC_MIN_SIZE;

	/*
	 * This needs to go after all the other initialization since they use
	 * the nvgpu_kzalloc() API.
	 */
	g->vmallocs->allocs_cache = nvgpu_kmem_cache_create(g,
						sizeof(struct nvgpu_mem_alloc));
	g->kmallocs->allocs_cache = nvgpu_kmem_cache_create(g,
						sizeof(struct nvgpu_mem_alloc));

	if (!g->vmallocs->allocs_cache || !g->kmallocs->allocs_cache) {
		err = -ENOMEM;
		if (g->vmallocs->allocs_cache)
			nvgpu_kmem_cache_destroy(g->vmallocs->allocs_cache);
		if (g->kmallocs->allocs_cache)
			nvgpu_kmem_cache_destroy(g->kmallocs->allocs_cache);
		goto fail;
	}

	return 0;

fail:
	if (g->vmallocs)
		kfree(g->vmallocs);
	if (g->kmallocs)
		kfree(g->kmallocs);
	return err;
}

#else /* !CONFIG_NVGPU_TRACK_MEM_USAGE */

int nvgpu_kmem_init(struct gk20a *g)
{
	return 0;
}

void nvgpu_kmem_fini(struct gk20a *g, int flags)
{
}
#endif /* CONFIG_NVGPU_TRACK_MEM_USAGE */

struct nvgpu_kmem_cache *nvgpu_kmem_cache_create(struct gk20a *g, size_t size)
{
	struct nvgpu_kmem_cache *cache =
		nvgpu_kzalloc(g, sizeof(struct nvgpu_kmem_cache));

	if (!cache)
		return NULL;

	cache->g = g;

	snprintf(cache->name, sizeof(cache->name),
		 "nvgpu-cache-0x%p-%d-%d", g, (int)size,
		 atomic_inc_return(&kmem_cache_id));
	cache->cache = kmem_cache_create(cache->name,
					 size, size, 0, NULL);
	if (!cache->cache) {
		nvgpu_kfree(g, cache);
		return NULL;
	}

	return cache;
}

void nvgpu_kmem_cache_destroy(struct nvgpu_kmem_cache *cache)
{
	struct gk20a *g = cache->g;

	kmem_cache_destroy(cache->cache);
	nvgpu_kfree(g, cache);
}

void *nvgpu_kmem_cache_alloc(struct nvgpu_kmem_cache *cache)
{
	return kmem_cache_alloc(cache->cache, GFP_KERNEL);
}

void nvgpu_kmem_cache_free(struct nvgpu_kmem_cache *cache, void *ptr)
{
	kmem_cache_free(cache->cache, ptr);
}
