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
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/tegra-mce.h>

#include <soc/tegra/chip-id.h>

#include <trace/events/nvmap.h>

#include "nvmap_tag.h"
#include "nvmap_misc.h"
#include "nvmap_cache.h"
#include "nvmap_handle.h"
#include "nvmap_stats.h"

typedef void (*nvmap_setup_chip_cache_fn)(struct nvmap_chip_cache_op *);

extern struct of_device_id __nvmapcache_of_table;

#define NVMAP_CACHE_OF_DECLARE(compat, fn) \
	_OF_DECLARE(nvmapcache, nvmapcache_of, compat, fn, \
			nvmap_setup_chip_cache_fn)

#ifndef CONFIG_NVMAP_CACHE_MAINT_BY_SET_WAYS
/* This is basically the L2 cache size but may be tuned as per requirement */
size_t cache_maint_inner_threshold = SIZE_MAX;
int nvmap_cache_maint_by_set_ways;
#else
int nvmap_cache_maint_by_set_ways = 1;
size_t cache_maint_inner_threshold = 8 * SZ_2M;
#endif

struct static_key nvmap_disable_vaddr_for_cache_maint;

static void nvmap_flush_dcache_all(void *dummy)
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

void nvmap_cache_inner_flush_all(void) {
	nvmap_flush_dcache_all(NULL);
}

// TODO: Clean up these global function pointers
void (*inner_flush_cache_all)(void) = nvmap_cache_inner_flush_all;

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
	op->inner_flush_cache_all = nvmap_cache_inner_flush_all;
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

__weak void nvmap_override_cache_ops(void)
{
	nvmap_select_cache_ops(nvmap_dev->dev_user.parent);
}

__weak void nvmap_handle_get_cacheability(struct nvmap_handle *h,
		bool *inner, bool *outer)
{
	u32 flags = nvmap_handle_flags(h);

	*inner = flags == NVMAP_HANDLE_CACHEABLE ||
		 flags == NVMAP_HANDLE_INNER_CACHEABLE;
	*outer = flags == NVMAP_HANDLE_CACHEABLE;
}

/*
 * FIXME:
 *
 *   __clean_dcache_page() is only available on ARM64 (well, we haven't
 *   implemented it on ARMv7).
 */
static void cache_clean_page(struct page *page)
{
	__clean_dcache_page(page);
}

void nvmap_cache_clean_pages(struct page **pages, int numpages)
{
	int i;

	/* Not technically a flush but that's what nvmap knows about. */
	nvmap_stats_inc(NS_CFLUSH_DONE, numpages << PAGE_SHIFT);
	trace_nvmap_cache_flush(numpages << PAGE_SHIFT,
		nvmap_stats_read(NS_ALLOC),
		nvmap_stats_read(NS_CFLUSH_RQ),
		nvmap_stats_read(NS_CFLUSH_DONE));

	for (i = 0; i < numpages; i++)
		cache_clean_page(pages[i]);
}

void nvmap_cache_inner_clean_all(void)
{
#ifdef CONFIG_ARCH_TEGRA_210_SOC
	on_each_cpu(__clean_dcache_louis, NULL, 1);
#endif
	tegra_clean_dcache_all(NULL);

}

void nvmap_cache_inner_maint(unsigned int op, void *vaddr, size_t size)
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

bool nvmap_cache_can_fast_maint(unsigned long start,
			unsigned long end, unsigned int op)
{
	if (!nvmap_cache_maint_by_set_ways)
		return false;

	if ((op == NVMAP_CACHE_OP_INV) ||
		((end - start) < cache_maint_inner_threshold))
		return false;
	return true;
}

void nvmap_cache_fast_maint(unsigned int op)
{

	if (op == NVMAP_CACHE_OP_WB_INV)
		nvmap_cache_inner_flush_all();
	else if (op == NVMAP_CACHE_OP_WB)
		nvmap_cache_inner_clean_all();
}

/*
 * If you want to call this function with inner = false,
 * then don't call this function at all
 */
int nvmap_cache_maint_phys_range(unsigned int op, phys_addr_t pstart,
		phys_addr_t pend)
{
	unsigned long kaddr;
	struct vm_struct *area = NULL;
	phys_addr_t cur_addr;

	/* TODO: Move this outside of everywhere this is called */
	if (nvmap_cache_can_fast_maint((unsigned long)pstart,
				 (unsigned long)pend, op)) {
		nvmap_cache_fast_maint(op);
		return 0;
	}

	area = alloc_vm_area(PAGE_SIZE, NULL);
	if (!area)
		return -ENOMEM;
	kaddr = (ulong)area->addr;

	cur_addr = pstart;
	while (cur_addr < pend) {
		phys_addr_t next = (cur_addr + PAGE_SIZE) & PAGE_MASK;
		void *base = (void *)kaddr + (cur_addr & ~PAGE_MASK);

		next = min(next, pend);
		ioremap_page_range(kaddr, kaddr + PAGE_SIZE,
			cur_addr, PG_PROT_KERNEL);
		nvmap_cache_inner_maint(op, base, next - cur_addr);
		cur_addr = next;
		unmap_kernel_range(kaddr, PAGE_SIZE);
	}

	free_vm_area(area);
	return 0;
}

void nvmap_cache_maint_heap_page_outer(struct page **pages,
				unsigned int op,
				unsigned long start, unsigned long end)
{
	while (start < end) {
		struct page *page;
		phys_addr_t paddr;
		unsigned long next;
		unsigned long off;
		size_t size;
		int ret;

		page = nvmap_to_page(pages[start >> PAGE_SHIFT]);
		next = min(((start + PAGE_SIZE) & PAGE_MASK), end);
		off = start & ~PAGE_MASK;
		size = next - start;
		paddr = page_to_phys(page) + off;

		ret = nvmap_cache_maint_phys_range(op, paddr, paddr + size);
		BUG_ON(ret != 0);
		start = next;
	}
}

void nvmap_cache_maint_inner(unsigned int op, void *vaddr, size_t size)
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
