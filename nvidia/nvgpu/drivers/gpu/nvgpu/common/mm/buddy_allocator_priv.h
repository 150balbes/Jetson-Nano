/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_MM_BUDDY_ALLOCATOR_PRIV_H
#define NVGPU_MM_BUDDY_ALLOCATOR_PRIV_H

#include <nvgpu/rbtree.h>
#include <nvgpu/list.h>

struct nvgpu_kmem_cache;
struct nvgpu_allocator;
struct vm_gk20a;

/*
 * Each buddy is an element in a binary tree.
 */
struct nvgpu_buddy {
	struct nvgpu_buddy *parent;	/* Parent node. */
	struct nvgpu_buddy *buddy;	/* This node's buddy. */
	struct nvgpu_buddy *left;	/* Lower address sub-node. */
	struct nvgpu_buddy *right;	/* Higher address sub-node. */

	struct nvgpu_list_node buddy_entry;	/* List entry for various lists. */
	struct nvgpu_rbtree_node alloced_entry;	/* RB tree of allocations. */

	u64 start;			/* Start address of this buddy. */
	u64 end;			/* End address of this buddy. */
	u64 order;			/* Buddy order. */

#define BALLOC_BUDDY_ALLOCED	0x1U
#define BALLOC_BUDDY_SPLIT	0x2U
#define BALLOC_BUDDY_IN_LIST	0x4U
	u32 flags;			/* List of associated flags. */

	/*
	 * Size of the PDE this buddy is using. This allows for grouping like
	 * sized allocations into the same PDE.
	 */
#define BALLOC_PTE_SIZE_ANY	(~0U)
#define BALLOC_PTE_SIZE_INVALID	0U
#define BALLOC_PTE_SIZE_SMALL	1U
#define BALLOC_PTE_SIZE_BIG	2U
	u32 pte_size;
};

static inline struct nvgpu_buddy *
nvgpu_buddy_from_buddy_entry(struct nvgpu_list_node *node)
{
	return (struct nvgpu_buddy *)
		((uintptr_t)node - offsetof(struct nvgpu_buddy, buddy_entry));
};

static inline struct nvgpu_buddy *
nvgpu_buddy_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_buddy *)
		((uintptr_t)node - offsetof(struct nvgpu_buddy, alloced_entry));
};

#define __buddy_flag_ops(flag, flag_up)					\
	static inline int buddy_is_ ## flag(struct nvgpu_buddy *b)	\
	{								\
		return b->flags & BALLOC_BUDDY_ ## flag_up;		\
	}								\
	static inline void buddy_set_ ## flag(struct nvgpu_buddy *b)	\
	{								\
		b->flags |= BALLOC_BUDDY_ ## flag_up;			\
	}								\
	static inline void buddy_clr_ ## flag(struct nvgpu_buddy *b)	\
	{								\
		b->flags &= ~BALLOC_BUDDY_ ## flag_up;			\
	}

/*
 * int  buddy_is_alloced(struct nvgpu_buddy *b);
 * void buddy_set_alloced(struct nvgpu_buddy *b);
 * void buddy_clr_alloced(struct nvgpu_buddy *b);
 *
 * int  buddy_is_split(struct nvgpu_buddy *b);
 * void buddy_set_split(struct nvgpu_buddy *b);
 * void buddy_clr_split(struct nvgpu_buddy *b);
 *
 * int  buddy_is_in_list(struct nvgpu_buddy *b);
 * void buddy_set_in_list(struct nvgpu_buddy *b);
 * void buddy_clr_in_list(struct nvgpu_buddy *b);
 */
__buddy_flag_ops(alloced, ALLOCED);
__buddy_flag_ops(split,   SPLIT);
__buddy_flag_ops(in_list, IN_LIST);

/*
 * Keeps info for a fixed allocation.
 */
struct nvgpu_fixed_alloc {
	struct nvgpu_list_node buddies;	/* List of buddies. */
	struct nvgpu_rbtree_node alloced_entry;	/* RB tree of fixed allocations. */

	u64 start;			/* Start of fixed block. */
	u64 end;			/* End address. */
};

static inline struct nvgpu_fixed_alloc *
nvgpu_fixed_alloc_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_fixed_alloc *)
	((uintptr_t)node - offsetof(struct nvgpu_fixed_alloc, alloced_entry));
};

/*
 * GPU buddy allocator for the various GPU address spaces. Each addressable unit
 * doesn't have to correspond to a byte. In some cases each unit is a more
 * complex object such as a comp_tag line or the like.
 *
 * The max order is computed based on the size of the minimum order and the size
 * of the address space.
 *
 * order_size is the size of an order 0 buddy.
 */
struct nvgpu_buddy_allocator {
	struct nvgpu_allocator *owner;	/* Owner of this buddy allocator. */
	struct vm_gk20a *vm;		/* Parent VM - can be NULL. */

	u64 base;			/* Base address of the space. */
	u64 length;			/* Length of the space. */
	u64 blk_size;			/* Size of order 0 allocation. */
	u64 blk_shift;			/* Shift to divide by blk_size. */

	/* Internal stuff. */
	u64 start;			/* Real start (aligned to blk_size). */
	u64 end;			/* Real end, trimmed if needed. */
	u64 count;			/* Count of objects in space. */
	u64 blks;			/* Count of blks in the space. */
	u64 max_order;			/* Specific maximum order. */

	struct nvgpu_rbtree_node *alloced_buddies;	/* Outstanding allocations. */
	struct nvgpu_rbtree_node *fixed_allocs;	/* Outstanding fixed allocations. */

	struct nvgpu_list_node co_list;

	struct nvgpu_kmem_cache *buddy_cache;

	/*
	 * Impose an upper bound on the maximum order.
	 */
#define GPU_BALLOC_ORDER_LIST_LEN	(GPU_BALLOC_MAX_ORDER + 1U)

	struct nvgpu_list_node buddy_list[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_len[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_split[GPU_BALLOC_ORDER_LIST_LEN];
	u64 buddy_list_alloced[GPU_BALLOC_ORDER_LIST_LEN];

	/*
	 * This is for when the allocator is managing a GVA space (the
	 * GPU_ALLOC_GVA_SPACE bit is set in @flags). This requires
	 * that we group like sized allocations into PDE blocks.
	 */
	u64 pte_blk_order;

	bool initialized;
	bool alloc_made;		/* True after the first alloc. */

	u64 flags;

	u64 bytes_alloced;
	u64 bytes_alloced_real;
	u64 bytes_freed;
};

static inline struct nvgpu_buddy_allocator *buddy_allocator(
	struct nvgpu_allocator *a)
{
	return (struct nvgpu_buddy_allocator *)(a)->priv;
}

static inline struct nvgpu_list_node *balloc_get_order_list(
	struct nvgpu_buddy_allocator *a, u64 order)
{
	return &a->buddy_list[order];
}

static inline u64 balloc_order_to_len(struct nvgpu_buddy_allocator *a,
				      int order)
{
	return (1 << order) * a->blk_size;
}

static inline u64 balloc_base_shift(struct nvgpu_buddy_allocator *a,
				    u64 base)
{
	return base - a->start;
}

static inline u64 balloc_base_unshift(struct nvgpu_buddy_allocator *a,
				      u64 base)
{
	return base + a->start;
}

static inline struct nvgpu_allocator *balloc_owner(
	struct nvgpu_buddy_allocator *a)
{
	return a->owner;
}

#endif /* NVGPU_MM_BUDDY_ALLOCATOR_PRIV_H */
