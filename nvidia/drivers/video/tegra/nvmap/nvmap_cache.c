/*
 * drivers/video/tegra/nvmap/nvmap_cache.c
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/of.h>
#include <soc/tegra/chip-id.h>

#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

#ifndef CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS
/* This is basically the L2 cache size but may be tuned as per requirement */
size_t cache_maint_inner_threshold = SIZE_MAX;
int nvmap_cache_maint_by_set_ways;
#else
int nvmap_cache_maint_by_set_ways = 1;
size_t cache_maint_inner_threshold = 8 * SZ_2M;
#endif

static struct static_key nvmap_disable_vaddr_for_cache_maint;

inline static void nvmap_flush_dcache_all(void *dummy)
{
#if defined(CONFIG_DENVER_CPU)
	u64 id_afr0;
	u64 midr;

	asm volatile ("mrs %0, MIDR_EL1" : "=r"(midr));
	/* check if current core is a Denver processor */
	if ((midr & 0xFF8FFFF0) == 0x4e0f0000) {
		asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
		/* check if complete cache flush through msr is supported */
		if (likely((id_afr0 & 0xf00) == 0x100)) {
			asm volatile ("msr s3_0_c15_c13_0, %0" : : "r" (0));
			asm volatile ("dsb sy");
			return;
		}
	}
#endif
	tegra_flush_dcache_all(NULL);
}

static void nvmap_inner_flush_cache_all(void)
{
	nvmap_flush_dcache_all(NULL);
}
void (*inner_flush_cache_all)(void) = nvmap_inner_flush_cache_all;

extern void __clean_dcache_louis(void *);
static void nvmap_inner_clean_cache_all(void)
{
#ifdef CONFIG_ARCH_TEGRA_210_SOC
	on_each_cpu(__clean_dcache_louis, NULL, 1);
#endif
	tegra_clean_dcache_all(NULL);
}
void (*inner_clean_cache_all)(void) = nvmap_inner_clean_cache_all;

static void nvmap_cache_of_setup(struct nvmap_chip_cache_op *op)
{
	op->inner_clean_cache_all = nvmap_inner_clean_cache_all;
	op->inner_flush_cache_all = nvmap_inner_flush_cache_all;
	op->name = kstrdup("set/ways", GFP_KERNEL);
	BUG_ON(!op->name);
}
NVMAP_CACHE_OF_DECLARE("nvidia,carveouts", nvmap_cache_of_setup);

void nvmap_select_cache_ops(struct device *dev)
{
	struct nvmap_chip_cache_op op;
	bool match_found = false;
	const struct of_device_id *matches = &__nvmapcache_of_table;

	memset(&op, 0, sizeof(op));

	for (; matches; matches++) {
		if (of_device_is_compatible(dev->of_node,
					    matches->compatible)) {
			const nvmap_setup_chip_cache_fn init_fn = matches->data;
			init_fn(&op);
			match_found = true;
			break;
		}
	}

	if (WARN_ON(match_found == false)) {
		pr_err("%s: no cache ops found\n",__func__);
		return;
	}
	inner_flush_cache_all = op.inner_flush_cache_all;
	inner_clean_cache_all = op.inner_clean_cache_all;
	pr_info("nvmap cache ops set to %s\n", op.name);
	kfree(op.name);

	if (inner_clean_cache_all && (op.flags & CALL_CLEAN_CACHE_ON_INIT)) {
		pr_info("calling cache operation %pF\n",
					inner_clean_cache_all);
		inner_clean_cache_all();
	}

	if (inner_flush_cache_all && (op.flags & CALL_FLUSH_CACHE_ON_INIT)) {
		pr_info("calling cache operation %pF\n",
					inner_flush_cache_all);
		inner_flush_cache_all();
	}
}

/*
 * FIXME:
 *
 *   __clean_dcache_page() is only available on ARM64 (well, we haven't
 *   implemented it on ARMv7).
 */
void nvmap_clean_cache_page(struct page *page)
{
	__clean_dcache_page(page);
}

void nvmap_clean_cache(struct page **pages, int numpages)
{
	int i;

	/* Not technically a flush but that's what nvmap knows about. */
	nvmap_stats_inc(NS_CFLUSH_DONE, numpages << PAGE_SHIFT);
	trace_nvmap_cache_flush(numpages << PAGE_SHIFT,
		nvmap_stats_read(NS_ALLOC),
		nvmap_stats_read(NS_CFLUSH_RQ),
		nvmap_stats_read(NS_CFLUSH_DONE));

	for (i = 0; i < numpages; i++)
		nvmap_clean_cache_page(pages[i]);
}

__weak void nvmap_override_cache_ops(void)
{
	nvmap_select_cache_ops(nvmap_dev->dev_user.parent);
}

void inner_cache_maint(unsigned int op, void *vaddr, size_t size)
{
	if (op == NVMAP_CACHE_OP_WB_INV)
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		__dma_flush_range(vaddr, vaddr + size);
#else
		__dma_flush_area(vaddr, size);
#endif
	else if (op == NVMAP_CACHE_OP_INV)
		__dma_map_area(vaddr, size, DMA_FROM_DEVICE);
	else
		__dma_map_area(vaddr, size, DMA_TO_DEVICE);
}

static void heap_page_cache_maint(
	struct nvmap_handle *h, unsigned long start, unsigned long end,
	unsigned int op, bool inner, bool outer, bool clean_only_dirty)
{
	if (h->userflags & NVMAP_HANDLE_CACHE_SYNC) {
		/*
		 * zap user VA->PA mappings so that any access to the pages
		 * will result in a fault and can be marked dirty
		 */
		nvmap_handle_mkclean(h, start, end-start);
		nvmap_zap_handle(h, start, end - start);
	}

	if (static_key_false(&nvmap_disable_vaddr_for_cache_maint))
		goto per_page_cache_maint;

	if (inner) {
		if (!h->vaddr) {
			if (__nvmap_mmap(h))
				__nvmap_munmap(h, h->vaddr);
			else
				goto per_page_cache_maint;
		}
		/* Fast inner cache maintenance using single mapping */
		inner_cache_maint(op, h->vaddr + start, end - start);
		if (!outer)
			return;
		/* Skip per-page inner maintenance in loop below */
		inner = false;

	}
per_page_cache_maint:

	while (start < end) {
		struct page *page;
		phys_addr_t paddr;
		unsigned long next;
		unsigned long off;
		size_t size;
		int ret;

		page = nvmap_to_page(h->pgalloc.pages[start >> PAGE_SHIFT]);
		next = min(((start + PAGE_SIZE) & PAGE_MASK), end);
		off = start & ~PAGE_MASK;
		size = next - start;
		paddr = page_to_phys(page) + off;

		ret = nvmap_cache_maint_phys_range(op, paddr, paddr + size,
				inner, outer);
		BUG_ON(ret != 0);
		start = next;
	}
}

static inline bool can_fast_cache_maint(unsigned long start,
			unsigned long end, unsigned int op)
{
	if (!nvmap_cache_maint_by_set_ways)
		return false;

	if ((op == NVMAP_CACHE_OP_INV) ||
		((end - start) < cache_maint_inner_threshold))
		return false;
	return true;
}

static bool fast_cache_maint(struct nvmap_handle *h,
	unsigned long start,
	unsigned long end, unsigned int op,
	bool clean_only_dirty)
{
	if (!can_fast_cache_maint(start, end, op))
		return false;

	if (h->userflags & NVMAP_HANDLE_CACHE_SYNC) {
		nvmap_handle_mkclean(h, 0, h->size);
		nvmap_zap_handle(h, 0, h->size);
	}

	if (op == NVMAP_CACHE_OP_WB_INV)
		inner_flush_cache_all();
	else if (op == NVMAP_CACHE_OP_WB)
		inner_clean_cache_all();

	return true;
}

struct cache_maint_op {
	phys_addr_t start;
	phys_addr_t end;
	unsigned int op;
	struct nvmap_handle *h;
	bool inner;
	bool outer;
	bool clean_only_dirty;
};

int nvmap_cache_maint_phys_range(unsigned int op, phys_addr_t pstart,
		phys_addr_t pend, int inner, int outer)
{
	unsigned long kaddr;
	struct vm_struct *area = NULL;
	phys_addr_t loop;

	if (!inner)
		goto do_outer;

	if (can_fast_cache_maint((unsigned long)pstart,
				 (unsigned long)pend, op)) {
		if (op == NVMAP_CACHE_OP_WB_INV)
			inner_flush_cache_all();
		else if (op == NVMAP_CACHE_OP_WB)
			inner_clean_cache_all();
		goto do_outer;
	}

	area = alloc_vm_area(PAGE_SIZE, NULL);
	if (!area)
		return -ENOMEM;
	kaddr = (ulong)area->addr;

	loop = pstart;
	while (loop < pend) {
		phys_addr_t next = (loop + PAGE_SIZE) & PAGE_MASK;
		void *base = (void *)kaddr + (loop & ~PAGE_MASK);

		next = min(next, pend);
		ioremap_page_range(kaddr, kaddr + PAGE_SIZE,
			loop, PG_PROT_KERNEL);
		inner_cache_maint(op, base, next - loop);
		loop = next;
		unmap_kernel_range(kaddr, PAGE_SIZE);
	}

	free_vm_area(area);
do_outer:
	return 0;
}

static int do_cache_maint(struct cache_maint_op *cache_work)
{
	phys_addr_t pstart = cache_work->start;
	phys_addr_t pend = cache_work->end;
	int err = 0;
	struct nvmap_handle *h = cache_work->h;
	unsigned int op = cache_work->op;

	if (!h || !h->alloc)
		return -EFAULT;

	wmb();
	if (h->flags == NVMAP_HANDLE_UNCACHEABLE ||
	    h->flags == NVMAP_HANDLE_WRITE_COMBINE || pstart == pend)
		goto out;

	trace_nvmap_cache_maint(h->owner, h, pstart, pend, op, pend - pstart);
	if (pstart > h->size || pend > h->size) {
		pr_warn("cache maintenance outside handle\n");
		err = -EINVAL;
		goto out;
	}

	if (fast_cache_maint(h, pstart, pend, op, cache_work->clean_only_dirty))
		goto out;

	if (h->heap_pgalloc) {
		heap_page_cache_maint(h, pstart, pend, op, true,
			(h->flags == NVMAP_HANDLE_INNER_CACHEABLE) ?
			false : true, cache_work->clean_only_dirty);
		goto out;
	}

	pstart += h->carveout->base;
	pend += h->carveout->base;

	err = nvmap_cache_maint_phys_range(op, pstart, pend, true,
			h->flags != NVMAP_HANDLE_INNER_CACHEABLE);

out:
	if (!err) {
		if (can_fast_cache_maint(pstart, pend, op))
			nvmap_stats_inc(NS_CFLUSH_DONE,
					cache_maint_inner_threshold);
		else
			nvmap_stats_inc(NS_CFLUSH_DONE, pend - pstart);
	}

	trace_nvmap_cache_flush(pend - pstart,
		nvmap_stats_read(NS_ALLOC),
		nvmap_stats_read(NS_CFLUSH_RQ),
		nvmap_stats_read(NS_CFLUSH_DONE));

	return 0;
}

__weak void nvmap_handle_get_cacheability(struct nvmap_handle *h,
		bool *inner, bool *outer)
{
	*inner = h->flags == NVMAP_HANDLE_CACHEABLE ||
		 h->flags == NVMAP_HANDLE_INNER_CACHEABLE;
	*outer = h->flags == NVMAP_HANDLE_CACHEABLE;
}

int __nvmap_do_cache_maint(struct nvmap_client *client,
			struct nvmap_handle *h,
			unsigned long start, unsigned long end,
			unsigned int op, bool clean_only_dirty)
{
	int err;
	struct cache_maint_op cache_op;

	h = nvmap_handle_get(h);
	if (!h)
		return -EFAULT;

	if ((start >= h->size) || (end > h->size)) {
		pr_debug("%s start: %ld end: %ld h->size: %zu\n", __func__,
				start, end, h->size);
		nvmap_handle_put(h);
		return -EFAULT;
	}

	if (!(h->heap_type & nvmap_dev->cpu_access_mask)) {
		pr_debug("%s heap_type %u access_mask 0x%x\n", __func__,
				h->heap_type, nvmap_dev->cpu_access_mask);
		nvmap_handle_put(h);
		return -EPERM;
	}

	nvmap_kmaps_inc(h);
	if (op == NVMAP_CACHE_OP_INV)
		op = NVMAP_CACHE_OP_WB_INV;

	/* clean only dirty is applicable only for Write Back operation */
	if (op != NVMAP_CACHE_OP_WB)
		clean_only_dirty = false;

	cache_op.h = h;
	cache_op.start = start ? start : 0;
	cache_op.end = end ? end : h->size;
	cache_op.op = op;
	nvmap_handle_get_cacheability(h, &cache_op.inner, &cache_op.outer);
	cache_op.clean_only_dirty = clean_only_dirty;

	nvmap_stats_inc(NS_CFLUSH_RQ, end - start);
	err = do_cache_maint(&cache_op);
	nvmap_kmaps_dec(h);
	nvmap_handle_put(h);
	return err;
}

int __nvmap_cache_maint(struct nvmap_client *client,
			       struct nvmap_cache_op_64 *op)
{
	struct vm_area_struct *vma;
	struct nvmap_vma_priv *priv;
	struct nvmap_handle *handle;
	unsigned long start;
	unsigned long end;
	int err = 0;

	if (!op->addr || op->op < NVMAP_CACHE_OP_WB ||
	    op->op > NVMAP_CACHE_OP_WB_INV)
		return -EINVAL;

	handle = nvmap_handle_get_from_fd(op->handle);
	if (!handle)
		return -EINVAL;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->active_mm, (unsigned long)op->addr);
	if (!vma || !is_nvmap_vma(vma) ||
	    (ulong)op->addr < vma->vm_start ||
	    (ulong)op->addr >= vma->vm_end ||
	    op->len > vma->vm_end - (ulong)op->addr) {
		err = -EADDRNOTAVAIL;
		goto out;
	}

	priv = (struct nvmap_vma_priv *)vma->vm_private_data;

	if (priv->handle != handle) {
		err = -EFAULT;
		goto out;
	}

	start = (unsigned long)op->addr - vma->vm_start +
		(vma->vm_pgoff << PAGE_SHIFT);
	end = start + op->len;

	err = __nvmap_do_cache_maint(client, priv->handle, start, end, op->op,
				     false);
out:
	up_read(&current->mm->mmap_sem);
	nvmap_handle_put(handle);
	return err;
}

/*
 * Perform cache op on the list of memory regions within passed handles.
 * A memory region within handle[i] is identified by offsets[i], sizes[i]
 *
 * sizes[i] == 0  is a special case which causes handle wide operation,
 * this is done by replacing offsets[i] = 0, sizes[i] = handles[i]->size.
 * So, the input arrays sizes, offsets  are not guaranteed to be read-only
 *
 * This will optimze the op if it can.
 * In the case that all the handles together are larger than the inner cache
 * maint threshold it is possible to just do an entire inner cache flush.
 *
 * NOTE: this omits outer cache operations which is fine for ARM64
 */
static int __nvmap_do_cache_maint_list(struct nvmap_handle **handles,
				u64 *offsets, u64 *sizes, int op, int nr,
				bool is_32)
{
	int i;
	u64 total = 0;
	u64 thresh = ~0;

	WARN(!IS_ENABLED(CONFIG_ARM64),
		"cache list operation may not function properly");

	if (nvmap_cache_maint_by_set_ways)
		thresh = cache_maint_inner_threshold;

	for (i = 0; i < nr; i++) {
		bool inner, outer;
		u32 *sizes_32 = (u32 *)sizes;
		u64 size = is_32 ? sizes_32[i] : sizes[i];

		nvmap_handle_get_cacheability(handles[i], &inner, &outer);

		if (!inner && !outer)
			continue;

		if ((op == NVMAP_CACHE_OP_WB) && nvmap_handle_track_dirty(handles[i]))
			total += atomic_read(&handles[i]->pgalloc.ndirty);
		else
			total += size ? size : handles[i]->size;
	}

	if (!total)
		return 0;

	/* Full flush in the case the passed list is bigger than our
	 * threshold. */
	if (total >= thresh) {
		for (i = 0; i < nr; i++) {
			if (handles[i]->userflags &
			    NVMAP_HANDLE_CACHE_SYNC) {
				nvmap_handle_mkclean(handles[i], 0,
						     handles[i]->size);
				nvmap_zap_handle(handles[i], 0,
						 handles[i]->size);
			}
		}

		if (op == NVMAP_CACHE_OP_WB)
			inner_clean_cache_all();
		else
			inner_flush_cache_all();
		nvmap_stats_inc(NS_CFLUSH_RQ, total);
		nvmap_stats_inc(NS_CFLUSH_DONE, thresh);
		trace_nvmap_cache_flush(total,
					nvmap_stats_read(NS_ALLOC),
					nvmap_stats_read(NS_CFLUSH_RQ),
					nvmap_stats_read(NS_CFLUSH_DONE));
	} else {
		for (i = 0; i < nr; i++) {
			u32 *offs_32 = (u32 *)offsets, *sizes_32 = (u32 *)sizes;
			u64 size = is_32 ? sizes_32[i] : sizes[i];
			u64 offset = is_32 ? offs_32[i] : offsets[i];
			int err;

			size = size ?: handles[i]->size;
			offset = offset ?: 0;
			err = __nvmap_do_cache_maint(handles[i]->owner,
						     handles[i], offset,
						     offset + size,
						     op, false);
			if (err) {
				pr_err("cache maint per handle failed [%d]\n",
						err);
				return err;
			}
		}
	}

	return 0;
}

inline int nvmap_do_cache_maint_list(struct nvmap_handle **handles,
				u64 *offsets, u64 *sizes, int op, int nr,
				bool is_32)
{
	int ret = 0;

	switch (tegra_get_chip_id()) {
	case TEGRA194:
		/*
		 * As io-coherency is enabled by default from T194 onwards,
		 * Don't do cache maint from CPU side. The HW, SCF will do.
		 */
		break;
	default:
		ret = __nvmap_do_cache_maint_list(handles,
					offsets, sizes, op, nr, is_32);
		break;
	}

	return ret;
}

static int cache_inner_threshold_show(struct seq_file *m, void *v)
{
	if (nvmap_cache_maint_by_set_ways)
		seq_printf(m, "%zuB\n", cache_maint_inner_threshold);
	else
		seq_printf(m, "%zuB\n", SIZE_MAX);
	return 0;
}

static int cache_inner_threshold_open(struct inode *inode, struct file *file)
{
	return single_open(file, cache_inner_threshold_show, inode->i_private);
}

static ssize_t cache_inner_threshold_write(struct file *file,
					const char __user *buffer,
					size_t count, loff_t *pos)
{
	int ret;
	struct seq_file *p = file->private_data;
	char str[] = "0123456789abcdef";

	count = min_t(size_t, strlen(str), count);
	if (copy_from_user(str, buffer, count))
		return -EINVAL;

	if (!nvmap_cache_maint_by_set_ways)
		return -EINVAL;

	mutex_lock(&p->lock);
	ret = sscanf(str, "%16zu", &cache_maint_inner_threshold);
	mutex_unlock(&p->lock);
	if (ret != 1)
		return -EINVAL;

	pr_debug("nvmap:cache_maint_inner_threshold is now :%zuB\n",
			cache_maint_inner_threshold);
	return count;
}

static const struct file_operations cache_inner_threshold_fops = {
	.open		= cache_inner_threshold_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= cache_inner_threshold_write,
};

int nvmap_cache_debugfs_init(struct dentry *nvmap_root)
{
	struct dentry *cache_root;

	if (!nvmap_root)
		return -ENODEV;

	cache_root = debugfs_create_dir("cache", nvmap_root);
	if (!cache_root)
		return -ENODEV;

	if (nvmap_cache_maint_by_set_ways) {
		debugfs_create_x32("nvmap_cache_maint_by_set_ways",
				   S_IRUSR | S_IWUSR,
				   cache_root,
				   &nvmap_cache_maint_by_set_ways);

	debugfs_create_file("cache_maint_inner_threshold",
			    S_IRUSR | S_IWUSR,
			    cache_root,
			    NULL,
			    &cache_inner_threshold_fops);
	}

	debugfs_create_atomic_t("nvmap_disable_vaddr_for_cache_maint",
				S_IRUSR | S_IWUSR,
				cache_root,
				&nvmap_disable_vaddr_for_cache_maint.enabled);

	return 0;
}
