/*
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_ALLOCATOR_H
#define NVGPU_ALLOCATOR_H

#ifdef __KERNEL__
/*
 * The Linux kernel has this notion of seq_files for printing info to userspace.
 * One of the allocator function pointers takes advantage of this and allows the
 * debug output to be directed either to nvgpu_log() or a seq_file.
 */
#include <linux/seq_file.h>
#endif

#include <nvgpu/log.h>
#include <nvgpu/lock.h>
#include <nvgpu/list.h>
#include <nvgpu/types.h>

/* #define ALLOCATOR_DEBUG_FINE */

struct nvgpu_allocator;
struct nvgpu_alloc_carveout;
struct vm_gk20a;
struct gk20a;

/*
 * Operations for an allocator to implement.
 */
struct nvgpu_allocator_ops {
	u64  (*alloc)(struct nvgpu_allocator *allocator, u64 len);
	u64  (*alloc_pte)(struct nvgpu_allocator *allocator, u64 len,
			  u32 page_size);
	void (*free)(struct nvgpu_allocator *allocator, u64 addr);

	/*
	 * Special interface to allocate a memory region with a specific
	 * starting address. Yikes. Note: if free() works for freeing both
	 * regular and fixed allocations then free_fixed() does not need to
	 * be implemented. This behavior exists for legacy reasons and should
	 * not be propagated to new allocators.
	 *
	 * For allocators where the @page_size field is not applicable it can
	 * be left as 0. Otherwise a valid page size should be passed (4k or
	 * what the large page size is).
	 */
	u64  (*alloc_fixed)(struct nvgpu_allocator *allocator,
			    u64 base, u64 len, u32 page_size);
	void (*free_fixed)(struct nvgpu_allocator *allocator,
			   u64 base, u64 len);

	/*
	 * Allow allocators to reserve space for carveouts.
	 */
	int  (*reserve_carveout)(struct nvgpu_allocator *allocator,
				 struct nvgpu_alloc_carveout *co);
	void (*release_carveout)(struct nvgpu_allocator *allocator,
				 struct nvgpu_alloc_carveout *co);

	/*
	 * Returns info about the allocator.
	 */
	u64  (*base)(struct nvgpu_allocator *allocator);
	u64  (*length)(struct nvgpu_allocator *allocator);
	u64  (*end)(struct nvgpu_allocator *allocator);
	bool (*inited)(struct nvgpu_allocator *allocator);
	u64  (*space)(struct nvgpu_allocator *allocator);

	/* Destructor. */
	void (*fini)(struct nvgpu_allocator *allocator);

#ifdef __KERNEL__
	/* Debugging. */
	void (*print_stats)(struct nvgpu_allocator *allocator,
			    struct seq_file *s, int lock);
#endif
};

struct nvgpu_allocator {
	struct gk20a *g;

	char name[32];
	struct nvgpu_mutex lock;

	void *priv;
	const struct nvgpu_allocator_ops *ops;

	struct dentry *debugfs_entry;
	bool debug;				/* Control for debug msgs. */
};

struct nvgpu_alloc_carveout {
	const char *name;
	u64 base;
	u64 length;

	struct nvgpu_allocator *allocator;

	/*
	 * For usage by the allocator implementation.
	 */
	struct nvgpu_list_node co_entry;
};

static inline struct nvgpu_alloc_carveout *
nvgpu_alloc_carveout_from_co_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_alloc_carveout *)
	((uintptr_t)node - offsetof(struct nvgpu_alloc_carveout, co_entry));
};

#define NVGPU_CARVEOUT(local_name, local_base, local_length)	\
	{							\
		.name = (local_name),				\
		.base = (local_base),				\
		.length = (local_length)			\
	}

/*
 * These are the available allocator flags.
 *
 *   GPU_ALLOC_GVA_SPACE
 *
 *     This flag makes sense for the buddy allocator only. It specifies that the
 *     allocator will be used for managing a GVA space. When managing GVA spaces
 *     special care has to be taken to ensure that allocations of similar PTE
 *     sizes are placed in the same PDE block. This allows the higher level
 *     code to skip defining both small and large PTE tables for every PDE. That
 *     can save considerable memory for address spaces that have a lot of
 *     allocations.
 *
 *   GPU_ALLOC_NO_ALLOC_PAGE
 *
 *     For any allocator that needs to manage a resource in a latency critical
 *     path this flag specifies that the allocator should not use any kmalloc()
 *     or similar functions during normal operation. Initialization routines
 *     may still use kmalloc(). This prevents the possibility of long waits for
 *     pages when using alloc_page(). Currently only the bitmap allocator
 *     implements this functionality.
 *
 *     Also note that if you accept this flag then you must also define the
 *     free_fixed() function. Since no meta-data is allocated to help free
 *     allocations you need to keep track of the meta-data yourself (in this
 *     case the base and length of the allocation as opposed to just the base
 *     of the allocation).
 *
 *   GPU_ALLOC_4K_VIDMEM_PAGES
 *
 *     We manage vidmem pages at a large page granularity for performance
 *     reasons; however, this can lead to wasting memory. For page allocators
 *     setting this flag will tell the allocator to manage pools of 4K pages
 *     inside internally allocated large pages.
 *
 *     Currently this flag is ignored since the only usage of the page allocator
 *     uses a 4K block size already. However, this flag has been reserved since
 *     it will be necessary in the future.
 *
 *   GPU_ALLOC_FORCE_CONTIG
 *
 *     Force allocations to be contiguous. Currently only relevant for page
 *     allocators since all other allocators are naturally contiguous.
 *
 *   GPU_ALLOC_NO_SCATTER_GATHER
 *
 *     The page allocator normally returns a scatter gather data structure for
 *     allocations (to handle discontiguous pages). However, at times that can
 *     be annoying so this flag forces the page allocator to return a u64
 *     pointing to the allocation base (requires GPU_ALLOC_FORCE_CONTIG to be
 *     set as well).
 */
#define GPU_ALLOC_GVA_SPACE		BIT64(0)
#define GPU_ALLOC_NO_ALLOC_PAGE		BIT64(1)
#define GPU_ALLOC_4K_VIDMEM_PAGES	BIT64(2)
#define GPU_ALLOC_FORCE_CONTIG		BIT64(3)
#define GPU_ALLOC_NO_SCATTER_GATHER	BIT64(4)

static inline void alloc_lock(struct nvgpu_allocator *a)
{
	nvgpu_mutex_acquire(&a->lock);
}

static inline void alloc_unlock(struct nvgpu_allocator *a)
{
	nvgpu_mutex_release(&a->lock);
}

/*
 * Buddy allocator specific initializers.
 */
int nvgpu_buddy_allocator_init(struct gk20a *g, struct nvgpu_allocator *na,
			       struct vm_gk20a *vm, const char *name,
			       u64 base, u64 size, u64 blk_size,
			       u64 max_order, u64 flags);

/*
 * Bitmap initializers.
 */
int nvgpu_bitmap_allocator_init(struct gk20a *g, struct nvgpu_allocator *na,
				const char *name, u64 base, u64 length,
				u64 blk_size, u64 flags);

/*
 * Page allocator initializers.
 */
int nvgpu_page_allocator_init(struct gk20a *g, struct nvgpu_allocator *na,
			      const char *name, u64 base, u64 length,
			      u64 blk_size, u64 flags);

/*
 * Lockless allocatior initializers.
 * Note: This allocator can only allocate fixed-size structures of a
 * pre-defined size.
 */
int nvgpu_lockless_allocator_init(struct gk20a *g, struct nvgpu_allocator *na,
				  const char *name, u64 base, u64 length,
				  u64 struct_size, u64 flags);

#define GPU_BALLOC_MAX_ORDER		31U

/*
 * Allocator APIs.
 */
u64  nvgpu_alloc(struct nvgpu_allocator *allocator, u64 len);
u64  nvgpu_alloc_pte(struct nvgpu_allocator *a, u64 len, u32 page_size);
void nvgpu_free(struct nvgpu_allocator *allocator, u64 addr);

u64  nvgpu_alloc_fixed(struct nvgpu_allocator *allocator, u64 base, u64 len,
		       u32 page_size);
void nvgpu_free_fixed(struct nvgpu_allocator *allocator, u64 base, u64 len);

int  nvgpu_alloc_reserve_carveout(struct nvgpu_allocator *a,
				  struct nvgpu_alloc_carveout *co);
void nvgpu_alloc_release_carveout(struct nvgpu_allocator *a,
				  struct nvgpu_alloc_carveout *co);

u64  nvgpu_alloc_base(struct nvgpu_allocator *a);
u64  nvgpu_alloc_length(struct nvgpu_allocator *a);
u64  nvgpu_alloc_end(struct nvgpu_allocator *a);
bool nvgpu_alloc_initialized(struct nvgpu_allocator *a);
u64  nvgpu_alloc_space(struct nvgpu_allocator *a);

void nvgpu_alloc_destroy(struct nvgpu_allocator *allocator);

#ifdef __KERNEL__
void nvgpu_alloc_print_stats(struct nvgpu_allocator *a,
			     struct seq_file *s, int lock);
#endif

static inline struct gk20a *nvgpu_alloc_to_gpu(struct nvgpu_allocator *a)
{
	return a->g;
}

#ifdef CONFIG_DEBUG_FS
/*
 * Common functionality for the internals of the allocators.
 */
void nvgpu_init_alloc_debug(struct gk20a *g, struct nvgpu_allocator *a);
void nvgpu_fini_alloc_debug(struct nvgpu_allocator *a);
#endif

int  nvgpu_alloc_common_init(struct nvgpu_allocator *a, struct gk20a *g,
			     const char *name, void *priv, bool dbg,
			     const struct nvgpu_allocator_ops *ops);

static inline void nvgpu_alloc_enable_dbg(struct nvgpu_allocator *a)
{
	a->debug = true;
}

static inline void nvgpu_alloc_disable_dbg(struct nvgpu_allocator *a)
{
	a->debug = false;
}

/*
 * Debug stuff.
 */
#ifdef __KERNEL__
#define __alloc_pstat(seq, allocator, fmt, arg...)		\
	do {							\
		if (seq)					\
			seq_printf(seq, fmt "\n", ##arg);	\
		else						\
			alloc_dbg(allocator, fmt, ##arg);	\
	} while (0)
#endif

#define do_alloc_dbg(a, fmt, arg...)				\
	nvgpu_log((a)->g, gpu_dbg_alloc, "%25s " fmt, (a)->name, ##arg)

/*
 * This gives finer control over debugging messages. By defining the
 * ALLOCATOR_DEBUG_FINE macro prints for an allocator will only get made if
 * that allocator's debug flag is set.
 *
 * Otherwise debugging is as normal: debug statements for all allocators
 * if the GPU debugging mask bit is set. Note: even when ALLOCATOR_DEBUG_FINE
 * is set gpu_dbg_alloc must still also be set to true.
 */
#if defined(ALLOCATOR_DEBUG_FINE)
#define alloc_dbg(a, fmt, arg...)				\
	do {							\
		if ((a)->debug)					\
			do_alloc_dbg((a), fmt, ##arg);		\
	} while (0)
#else
#define alloc_dbg(a, fmt, arg...) do_alloc_dbg(a, fmt, ##arg)
#endif

#endif /* NVGPU_ALLOCATOR_H */
