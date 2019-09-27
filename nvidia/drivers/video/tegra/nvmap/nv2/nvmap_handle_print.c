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

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/uaccess.h>

#include <soc/tegra/chip-id.h>

#include <asm/pgtable.h>

#include <trace/events/nvmap.h>

#include "nvmap_stats.h"

#include "nvmap_misc.h"
#include "nvmap_cache.h"
#include "nvmap_heap_alloc.h"
#include "nvmap_carveout.h"
#include "nvmap_carveout.h"
#include "nvmap_dev.h"
#include "nvmap_dmabuf.h"
#include "nvmap_vma.h"
#include "nvmap_tag.h"

#include "nvmap_handle.h"
#include "nvmap_handle_priv.h"

#define K(x) (x >> 10)

void nvmap_handle_stringify(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type,
				  int ref_dupes)
{
	struct nvmap_device *dev = nvmap_dev;

	if (handle->alloc && handle->heap_type == heap_type) {
		phys_addr_t base = heap_type == NVMAP_HEAP_IOVMM ? 0 :
			handle->heap_pgalloc ? 0 :
			(handle->carveout->base);
		size_t size = K(handle->size);
		int i = 0;

		// TODO: Remove GOTO
next_page:
		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			base = page_to_phys(handle->pgalloc.pages[i++]);
			size = K(PAGE_SIZE);
		}

		seq_printf(s,
				"%-18s %-18s %8llx %10zuK %8x %6u %6u %6u %6u %6u %6u %8pK %s\n",
				"", "",
				(unsigned long long)base, size,
				handle->userflags,
				atomic_read(&handle->ref),
				ref_dupes,
				0,
				atomic_read(&handle->kmap_count),
				atomic_read(&handle->umap_count),
				atomic_read(&handle->share_count),
				handle,
				__nvmap_tag_name(dev, handle->userflags >> 16));

		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			i++;
			if (i < (handle->size >> PAGE_SHIFT))
				goto next_page;
		}
	}
}

/* compute the total amount of handle physical memory that is mapped
 * into client's virtual address space. Remember that vmas list is
 * sorted in ascending order of handle offsets.
 * NOTE: This function should be called while holding handle's lock mutex.
 */
static u64 vma_calc_mss(pid_t client_pid, struct list_head *vmas)
{
	struct nvmap_vma_list *vma_list = NULL;
	struct vm_area_struct *vma = NULL;
	u64 end_offset = 0, vma_start_offset, vma_size;
	int64_t overlap_size;
	u64 total = 0;

	list_for_each_entry(vma_list, vmas, list) {

		if (client_pid == vma_list->pid) {
			vma = vma_list->vma;
			vma_size = vma->vm_end - vma->vm_start;

			vma_start_offset = vma->vm_pgoff << PAGE_SHIFT;
			if (end_offset < vma_start_offset + vma_size) {
				total += vma_size;

				overlap_size = end_offset - vma_start_offset;
				if (overlap_size > 0)
					total -= overlap_size;
				end_offset = vma_start_offset + vma_size;
			}
		}
	}
	return total;
}

// TODO: Find a way to unify this with the above function
void nvmap_handle_maps_stringify(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type,
				  pid_t client_pid)
{
	struct nvmap_vma_list *vma_list = NULL;
	struct vm_area_struct *vma = NULL;
	u64 total_mapped_size, vma_size;

	if (handle->alloc && handle->heap_type == heap_type) {
		phys_addr_t base = heap_type == NVMAP_HEAP_IOVMM ? 0 :
			handle->heap_pgalloc ? 0 :
			(handle->carveout->base);
		size_t size = K(handle->size);
		int i = 0;

next_page:
		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			base = page_to_phys(handle->pgalloc.pages[i++]);
			size = K(PAGE_SIZE);
		}

		seq_printf(s,
				"%-18s %-18s %8llx %10zuK %8x %6u %16pK "
				"%12s %12s ",
				"", "",
				(unsigned long long)base, K(handle->size),
				handle->userflags,
				atomic_read(&handle->share_count),
				handle, "", "");

		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			i++;
			if (i < (handle->size >> PAGE_SHIFT))
				goto next_page;
		}

		mutex_lock(&handle->lock);
		total_mapped_size = vma_calc_mss(client_pid, &handle->vmas);
		seq_printf(s, "%6lluK\n", K(total_mapped_size));

		list_for_each_entry(vma_list, &handle->vmas, list) {

			if (vma_list->pid == client_pid) {
				vma = vma_list->vma;
				vma_size = vma->vm_end - vma->vm_start;
				seq_printf(s,
						"%-18s %-18s %8s %11s %8s %6s %16s "
						"%-12lx-%12lx %6lluK\n",
						"", "", "", "", "", "", "",
						vma->vm_start, vma->vm_end,
						K(vma_size));
			}
		}
		mutex_unlock(&handle->lock);
	}

}

// TODO: Unify all the functions that print
void nvmap_handle_all_allocations_show(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type)
{
	int i = 0;

	if (handle->alloc && handle->heap_type == heap_type) {
		phys_addr_t base = heap_type == NVMAP_HEAP_IOVMM ? 0 :
			handle->heap_pgalloc ? 0 :
			(handle->carveout->base);
		size_t size = K(handle->size);

next_page:
		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			base = page_to_phys(handle->pgalloc.pages[i++]);
			size = K(PAGE_SIZE);
		}

		seq_printf(s,
				"%8llx %10zuK %9x %6u %6u %6u %6u %8p\n",
				(unsigned long long)base, K(handle->size),
				handle->userflags,
				atomic_read(&handle->ref),
				atomic_read(&handle->kmap_count),
				atomic_read(&handle->umap_count),
				atomic_read(&handle->share_count),
				handle);

		if ((heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			i++;
			if (i < (handle->size >> PAGE_SHIFT))
				goto next_page;
		}
	}

}
void nvmap_handle_orphans_allocations_show(struct nvmap_handle *handle,
				  struct seq_file *s, u32 heap_type)
{
	if(atomic_read(&handle->share_count))
		return;

	nvmap_handle_all_allocations_show(handle, s, heap_type);
}

u64 nvmap_handle_share_size(struct nvmap_handle *handle, u32 heap_type)
{
	if (handle->alloc && handle->heap_type == heap_type) {
		return handle->size / atomic_read(&handle->share_count);
	} else {
		return 0;
	}
}

// TODO: Unify this between handle and client
struct procrank_stats {
	struct vm_area_struct *vma;
	u64 pss;
};

u64 nvmap_handle_procrank_walk(struct nvmap_handle *h, struct mm_walk *walk,
		pid_t client_pid)
{
	struct nvmap_vma_list *tmp;
	struct procrank_stats *mss = walk->private;

	if (!h || !h->alloc || !h->heap_pgalloc)
		return 0;

	mutex_lock(&h->lock);
	list_for_each_entry(tmp, &h->vmas, list) {
		if (client_pid == tmp->pid) {
			mss->vma = tmp->vma;
			walk_page_range(tmp->vma->vm_start,
					tmp->vma->vm_end,
					walk);
		}
	}
	mutex_unlock(&h->lock);

	return h->size / atomic_read(&h->share_count);

}

u64 nvmap_handle_total_pss(struct nvmap_handle *h, u32 heap_type)
{
	int i;
	u64 pss = 0;

	if (!h || !h->alloc || h->heap_type != heap_type)
		return 0;

	for (i = 0; i < h->size >> PAGE_SHIFT; i++) {
		struct page *page = nvmap_to_page(h->pgalloc.pages[i]);

		if (page_mapcount(page) > 0)
			pss += PAGE_SIZE;
	}

	return pss;
}

u64 nvmap_handle_total_mss(struct nvmap_handle *h, u32 heap_type)
{
	if (!h || !h->alloc || h->heap_type != heap_type)
		return 0;

	return h->size;
}

int nvmap_handle_pid_show(struct nvmap_handle *handle, struct seq_file *s,
					pid_t client_pid)
{
		struct nvmap_debugfs_handles_entry entry;
		u64 total_mapped_size = 0;
		int i = 0;
		int ret = 0;

		if (!handle->alloc)
			return 0;

		mutex_lock(&handle->lock);

		total_mapped_size = vma_calc_mss(client_pid, &handle->vmas);
		mutex_unlock(&handle->lock);

		entry.base = handle->heap_type == NVMAP_HEAP_IOVMM ? 0 :
			     handle->heap_pgalloc ? 0 :
			     (handle->carveout->base);
		entry.size = handle->size;
		entry.flags = handle->userflags;
		entry.share_count = atomic_read(&handle->share_count);
		entry.mapped_size = total_mapped_size;

next_page:
		if ((handle->heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			entry.base = page_to_phys(handle->pgalloc.pages[i++]);
			entry.size = K(PAGE_SIZE);
		}

		ret = seq_write(s, &entry, sizeof(entry));
		if (ret < 0)
			return ret;

		if ((handle->heap_type == NVMAP_HEAP_CARVEOUT_VPR) && handle->heap_pgalloc) {
			i++;
			if (i < (handle->size >> PAGE_SHIFT))
				goto next_page;
		}

		return 0;
}

int nvmap_handle_is_migratable(struct nvmap_handle *h)
{
	return (!atomic_read(&h->pin) && !atomic_read(&h->kmap_count));
}

void nvmap_handle_lru_show(struct nvmap_handle *h, struct seq_file *s)
{
		seq_printf(s, "%-18s %18s %8s %10zuK %8s %6s %6s %6u %6u "
			"%6u %8p\n", "", "", "", K(h->size), "", "",
			"", atomic_read(&h->pin),
			    atomic_read(&h->kmap_count),
			    atomic_read(&h->umap_count),
			    h);
}
