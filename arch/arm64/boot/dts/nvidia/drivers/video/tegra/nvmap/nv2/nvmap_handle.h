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

#ifndef __NVMAP_HANDLE_H
#define __NVMAP_HANDLE_H

#include <linux/dma-buf.h>
#include "nvmap_structs.h"

struct nvmap_handle;

enum NVMAP_PROT_OP {
	NVMAP_HANDLE_PROT_NONE = 1,
	NVMAP_HANDLE_PROT_RESTORE = 2,
};

struct nvmap_handle *nvmap_handle_create(size_t size);
struct nvmap_handle *nvmap_handle_create_from_dmabuf(
			struct nvmap_client * client, struct dma_buf *dmabuf);
void nvmap_handle_add_owner(struct nvmap_handle *handle,
					struct nvmap_client *client);
void nvmap_handle_destroy(struct nvmap_handle *handle);
void nvmap_handle_install_fd(struct nvmap_handle *handle, int fd);

struct nvmap_handle *nvmap_handle_get(struct nvmap_handle *h);
void nvmap_handle_put(struct nvmap_handle *h);

int nvmap_handle_alloc(struct nvmap_handle *h,
				unsigned int heap_mask,
				size_t align,
				u8 kind,
				unsigned int flags,
				int peer);

int nvmap_handle_alloc_from_ivmid(struct nvmap_handle *handle, u64 ivm_id);

int nvmap_handle_alloc_carveout(struct nvmap_handle *handle,
					      unsigned long type,
					      phys_addr_t *start);
int nvmap_handle_alloc_from_va(struct nvmap_handle *h,
			       ulong addr,
			       unsigned int flags);
int nvmap_handle_alloc_from_ivmid(struct nvmap_handle *handle, u64 ivm_id);

struct nvmap_handle *nvmap_handle_from_fd(int fd);
struct nvmap_handle *nvmap_handle_from_ivmid(u64 ivm_id);

int nvmap_handle_cache_maint(struct nvmap_handle *handle, unsigned long start,
		unsigned long end, unsigned int op);
int nvmap_handles_cache_maint(struct nvmap_handle **handles,
				u64 *offsets, u64 *sizes, int op, int nr);
void nvmap_handle_zap(struct nvmap_handle *handle, u64 offset, u64 size);

void *nvmap_handle_mmap(struct nvmap_handle *h);
void nvmap_handle_munmap(struct nvmap_handle *h, void *addr);

int nvmap_handles_reserve(struct nvmap_handle **handles, u64 *offsets,
						u64 *sizes, int op, int nr);
ssize_t nvmap_handle_rw(struct nvmap_handle *h,
			 unsigned long h_offs, unsigned long h_stride,
			 unsigned long sys_addr, unsigned long sys_stride,
			 unsigned long elem_size, unsigned long count,
			 int is_read);

int nvmap_handle_owns_vma(struct nvmap_handle *h, struct vm_area_struct *vma);
int nvmap_handle_add_vma(struct nvmap_handle *handle,
					struct vm_area_struct *vma);
int nvmap_handle_del_vma(struct nvmap_handle *handle,
					struct vm_area_struct *vma);
int nvmap_handle_open_vma(struct nvmap_handle *handle);
int nvmap_handle_close_vma(struct nvmap_handle *handle);
int nvmap_handle_fault_vma(struct nvmap_handle *handle,
		unsigned long offs, struct page **page_ptr);
bool nvmap_handle_fixup_prot_vma(struct nvmap_handle *handle,
					unsigned long offs);

struct dma_buf *nvmap_handle_to_dmabuf(struct nvmap_handle *handle);

pgprot_t nvmap_handle_pgprot(struct nvmap_handle *handle, pgprot_t prot);

void nvmap_handle_kmap_inc(struct nvmap_handle *h);
void nvmap_handle_kmap_dec(struct nvmap_handle *h);
void nvmap_handle_umap_inc(struct nvmap_handle *h);
void nvmap_handle_umap_dec(struct nvmap_handle *h);

size_t nvmap_handle_size(struct nvmap_handle *handle);

int nvmap_handle_is_allocated(struct nvmap_handle *h);
size_t nvmap_handle_ivm_id(struct nvmap_handle *h);
u32 nvmap_handle_heap_type(struct nvmap_handle *h);
// TODO: What is difference between userflags and flags?
u32 nvmap_handle_userflag(struct nvmap_handle *h);
u32 nvmap_handle_flags(struct nvmap_handle *h);

bool nvmap_handle_is_heap(struct nvmap_handle *h);
bool nvmap_handle_track_dirty(struct nvmap_handle *h);
pgprot_t nvmap_handle_pgprot(struct nvmap_handle *h, pgprot_t prot);

// TODO Remove these, only needed by dmabuf_ops
struct list_head *nvmap_handle_lru(struct nvmap_handle *h);
atomic_t *nvmap_handle_pin(struct nvmap_handle *h);

// TODO: Rename these
void *__nvmap_kmap(struct nvmap_handle *h, unsigned int pagenum);
void __nvmap_kunmap(struct nvmap_handle *h, unsigned int pagenum,
		  void *addr);
void *__nvmap_mmap(struct nvmap_handle *h);
void __nvmap_munmap(struct nvmap_handle *h, void *addr);
struct sg_table *__nvmap_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h);
void __nvmap_free_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h, struct sg_table *sgt);

void nvmap_handle_stringify(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type,
				  int ref_dupes);
void nvmap_handle_maps_stringify(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type,
				  pid_t client_pid);
int nvmap_handle_pid_show(struct nvmap_handle *handle, struct seq_file *s,
					pid_t client_pid);

void nvmap_handle_all_allocations_show(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type);
void nvmap_handle_orphans_allocations_show(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type);

u64 nvmap_handle_share_size(struct nvmap_handle *handle, u32 heap_type);
u64 nvmap_handle_total_mss(struct nvmap_handle *h, u32 heap_type);
u64 nvmap_handle_total_pss(struct nvmap_handle *h, u32 heap_type);

int nvmap_handle_is_migratable(struct nvmap_handle *h);
void nvmap_handle_lru_show(struct nvmap_handle *h, struct seq_file *s);
struct nvmap_handle *nvmap_handle_from_node(struct rb_node *n);

struct nvmap_handle *nvmap_handle_from_lru(struct list_head *n);

#include "nvmap_dev.h"

static inline void nvmap_lru_add(struct list_head *handle_lru)
{
	spin_lock(&nvmap_dev->lru_lock);
	BUG_ON(!list_empty(handle_lru));
	list_add_tail(handle_lru, &nvmap_dev->lru_handles);
	spin_unlock(&nvmap_dev->lru_lock);
}

static inline void nvmap_lru_del(struct list_head *handle_lru)
{
	spin_lock(&nvmap_dev->lru_lock);
	list_del(handle_lru);
	INIT_LIST_HEAD(handle_lru);
	spin_unlock(&nvmap_dev->lru_lock);
}

static inline void nvmap_lru_reset(struct list_head *handle_lru)
{
	spin_lock(&nvmap_dev->lru_lock);
	BUG_ON(list_empty(handle_lru));
	list_del(handle_lru);
	list_add_tail(handle_lru, &nvmap_dev->lru_handles);
	spin_unlock(&nvmap_dev->lru_lock);
}

#endif /* __NVMAP_HANDLE_H */
