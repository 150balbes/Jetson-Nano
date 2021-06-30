/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef __NVMAP_HANDLE_PRIV_H
#define __NVMAP_HANDLE_PRIV_H

// TODO: Remove this definition from here
/* handles allocated using shared system memory (either IOVMM- or high-order
 * page allocations */
struct nvmap_pgalloc {
	struct page **pages;
	bool contig;			/* contiguous system memory */
	atomic_t reserved;
	atomic_t ndirty;	/* count number of dirty pages */
};

struct nvmap_handle {
	struct rb_node node;	/* entry on global handle tree */
	atomic_t ref;		/* reference count (i.e., # of duplications) */
	atomic_t pin;		/* pin count */
	u32 flags;		/* caching flags */
	size_t size;		/* padded (as-allocated) size */
	size_t orig_size;	/* original (as-requested) size */
	size_t align;
	struct nvmap_client *owner;
	struct dma_buf *dmabuf;
	union {
		struct nvmap_pgalloc pgalloc;
		struct nvmap_heap_block *carveout;
	};
	bool heap_pgalloc;	/* handle is page allocated (sysmem / iovmm) */
	bool alloc;		/* handle has memory allocated */
	bool from_va;		/* handle memory is from VA */
	u32 heap_type;		/* handle heap is allocated from */
	u32 userflags;		/* flags passed from userspace */
	void *vaddr;		/* mapping used inside kernel */
	struct list_head vmas;	/* list of all user vma's */
	atomic_t umap_count;	/* number of outstanding maps from user */
	atomic_t kmap_count;	/* number of outstanding map from kernel */
	atomic_t share_count;	/* number of processes sharing the handle */
	struct list_head lru;	/* list head to track the lru */
	struct mutex lock;
	struct list_head dmabuf_priv;
	u64 ivm_id;
	int peer;		/* Peer VM number */
	int fd;
};

void nvmap_handle_mkclean(struct nvmap_handle *h, u32 offset, u32 size);
void nvmap_handle_mkdirty(struct nvmap_handle *h, u32 offset, u32 size);

#endif /* __NVMAP_HANDLE_PRIV_H */
