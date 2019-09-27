/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef BITMAP_ALLOCATOR_PRIV_H
#define BITMAP_ALLOCATOR_PRIV_H


#include <nvgpu/rbtree.h>
#include <nvgpu/kmem.h>

struct nvgpu_allocator;

struct nvgpu_bitmap_allocator {
	struct nvgpu_allocator *owner;

	u64 base;			/* Base address of the space. */
	u64 length;			/* Length of the space. */
	u64 blk_size;			/* Size that corresponds to 1 bit. */
	u64 blk_shift;			/* Bit shift to divide by blk_size. */
	u64 num_bits;			/* Number of allocatable bits. */
	u64 bit_offs;			/* Offset of bitmap. */

	/*
	 * Optimization for making repeated allocations faster. Keep track of
	 * the next bit after the most recent allocation. This is where the next
	 * search will start from. This should make allocation faster in cases
	 * where lots of allocations get made one after another. It shouldn't
	 * have a negative impact on the case where the allocator is fragmented.
	 */
	u64 next_blk;

	unsigned long *bitmap;		/* The actual bitmap! */
	struct nvgpu_rbtree_node *allocs;  /* Tree of outstanding allocations */

	struct nvgpu_kmem_cache *meta_data_cache;

	u64 flags;

	bool inited;

	/* Statistics */
	u64 nr_allocs;
	u64 nr_fixed_allocs;
	u64 bytes_alloced;
	u64 bytes_freed;
};

struct nvgpu_bitmap_alloc {
	u64 base;
	u64 length;
	struct nvgpu_rbtree_node alloc_entry;	/* RB tree of allocations. */
};

static inline struct nvgpu_bitmap_alloc *
nvgpu_bitmap_alloc_from_rbtree_node(struct nvgpu_rbtree_node *node)
{
	return (struct nvgpu_bitmap_alloc *)
	((uintptr_t)node - offsetof(struct nvgpu_bitmap_alloc, alloc_entry));
};

static inline struct nvgpu_bitmap_allocator *bitmap_allocator(
	struct nvgpu_allocator *a)
{
	return (struct nvgpu_bitmap_allocator *)(a)->priv;
}


#endif
