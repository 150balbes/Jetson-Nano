/*
 * drivers/video/tegra/nvmap/nvmap_pp.c
 *
 * Manage page pools to speed up page allocation.
 *
 * Copyright (c) 2009-2018, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/moduleparam.h>
#include <linux/nodemask.h>
#include <linux/shrinker.h>
#include <linux/kthread.h>
#include <linux/debugfs.h>
#include <linux/freezer.h>
#include <linux/highmem.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/clock.h>
#include <uapi/linux/sched/types.h>
#endif

#include <trace/events/nvmap.h>

#include "nvmap_cache.h"
#include "nvmap_pp.h"
#include "nvmap_dev.h"

#define NVMAP_TEST_PAGE_POOL_SHRINKER     1
#define PENDING_PAGES_SIZE                (SZ_1M / PAGE_SIZE)

u64 nvmap_big_page_allocs;
u64 nvmap_total_page_allocs;

static bool enable_pp = 1;
static u32 pool_size;

static struct task_struct *background_allocator;
static DECLARE_WAIT_QUEUE_HEAD(nvmap_bg_wait);

#ifdef CONFIG_NVMAP_PAGE_POOL_DEBUG
static inline void __pp_dbg_var_add(u64 *dbg_var, u32 nr)
{
	*dbg_var += nr;
}
#else
#define __pp_dbg_var_add(dbg_var, nr)
#endif

#define pp_alloc_add(pool, nr) __pp_dbg_var_add(&(pool)->allocs, nr)
#define pp_fill_add(pool, nr)  __pp_dbg_var_add(&(pool)->fills, nr)
#define pp_hit_add(pool, nr)   __pp_dbg_var_add(&(pool)->hits, nr)
#define pp_miss_add(pool, nr)  __pp_dbg_var_add(&(pool)->misses, nr)

static int __nvmap_page_pool_fill_lots_locked(struct nvmap_page_pool *pool,
				       struct page **pages, u32 nr);

static inline struct page *get_zero_list_page(struct nvmap_page_pool *pool)
{
	struct page *page;

	trace_get_zero_list_page(pool->to_zero);

	if (list_empty(&pool->zero_list))
		return NULL;

	page = list_first_entry(&pool->zero_list, struct page, lru);
	list_del(&page->lru);

	pool->to_zero--;

	return page;
}

static inline struct page *get_page_list_page(struct nvmap_page_pool *pool)
{
	struct page *page;

	trace_get_page_list_page(pool->count);

	if (list_empty(&pool->page_list))
		return NULL;

	page = list_first_entry(&pool->page_list, struct page, lru);
	list_del(&page->lru);

	pool->count--;

	return page;
}

static inline struct page *get_page_list_page_bp(struct nvmap_page_pool *pool)
{
	struct page *page;

	if (list_empty(&pool->page_list_bp))
		return NULL;

	page = list_first_entry(&pool->page_list_bp, struct page, lru);
	list_del(&page->lru);

	pool->count -= pool->pages_per_big_pg;
	pool->big_page_count -= pool->pages_per_big_pg;

	return page;
}

static inline bool nvmap_bg_should_run(struct nvmap_page_pool *pool)
{
	return !list_empty(&pool->zero_list);
}

static void nvmap_pp_zero_pages(struct page **pages, int nr)
{
	int i;

	for (i = 0; i < nr; i++) {
		clear_highpage(pages[i]);
	}
	nvmap_cache_clean_pages(pages, nr);

	trace_nvmap_pp_zero_pages(nr);
}

static void nvmap_pp_do_background_zero_pages(struct nvmap_page_pool *pool)
{
	int i;
	struct page *page;
	int ret;
	/*
	 * Statically declared array of pages to be zeroed in a batch,
	 * local to this thread but too big for the stack.
	 */
	static struct page *pending_zero_pages[PENDING_PAGES_SIZE];

	rt_mutex_lock(&pool->lock);
	for (i = 0; i < PENDING_PAGES_SIZE; i++) {
		page = get_zero_list_page(pool);
		if (page == NULL)
			break;
		pending_zero_pages[i] = page;
		pool->under_zero++;
	}
	rt_mutex_unlock(&pool->lock);

	nvmap_pp_zero_pages(pending_zero_pages, i);

	rt_mutex_lock(&pool->lock);
	ret = __nvmap_page_pool_fill_lots_locked(pool, pending_zero_pages, i);
	pool->under_zero -= i;
	rt_mutex_unlock(&pool->lock);

	trace_nvmap_pp_do_background_zero_pages(ret, i);

	for (; ret < i; ret++)
		__free_page(pending_zero_pages[ret]);
}

/*
 * This thread fills the page pools with zeroed pages. We avoid releasing the
 * pages directly back into the page pools since we would then have to zero
 * them ourselves. Instead it is easier to just reallocate zeroed pages. This
 * happens in the background so that the overhead of allocating zeroed pages is
 * not directly seen by userspace. Of course if the page pools are empty user
 * space will suffer.
 */
static int nvmap_background_zero_thread(void *arg)
{
	struct nvmap_page_pool *pool = &nvmap_dev->pool;
	struct sched_param param = { .sched_priority = 0 };

	pr_info("PP zeroing thread starting.\n");

	set_freezable();
	sched_setscheduler(current, SCHED_IDLE, &param);

	while (!kthread_should_stop()) {
		while (nvmap_bg_should_run(pool))
			nvmap_pp_do_background_zero_pages(pool);

		wait_event_freezable(nvmap_bg_wait,
				nvmap_bg_should_run(pool) ||
				kthread_should_stop());
	}

	return 0;
}

static void nvmap_pgcount(struct page *page, bool incr)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	if (incr)
		atomic_inc(&page->_count);
	else
		atomic_dec(&page->_count);
#else
	page_ref_add(page, incr ? 1 : -1);
#endif
}

/*
 * Free the passed number of pages from the page pool. This happens regardless
 * of whether the page pools are enabled. This lets one disable the page pools
 * and then free all the memory therein.
 *
 * FIXME: Pages in pending_zero_pages[] can still be unreleased.
 */
static ulong nvmap_page_pool_free_pages_locked(struct nvmap_page_pool *pool,
						      ulong nr_pages)
{
	struct page *page;
	bool use_page_list = false;
	bool use_page_list_bp = false;

	pr_debug("req to release pages=%ld\n", nr_pages);

	while (nr_pages) {
		int i;

		if (use_page_list_bp)
			page = get_page_list_page_bp(pool);
		else if (use_page_list)
			page = get_page_list_page(pool);
		else
			page = get_zero_list_page(pool);

		if (!page) {
			if (!use_page_list) {
				use_page_list = true;
				continue;
			} else if (!use_page_list_bp) {
				use_page_list_bp = true;
				continue;
			}
			break;
		}

		if (use_page_list_bp) {
			for (i = 0; i < pool->pages_per_big_pg; i++)
				__free_page(nth_page(page, i));
			pr_debug("released %d pages\n", pool->pages_per_big_pg);
			if (nr_pages > pool->pages_per_big_pg)
				nr_pages -= pool->pages_per_big_pg;
			else
				nr_pages = 0;
		} else {
			__free_page(page);
			nr_pages--;
			pr_debug("released 1 page\n");
		}
	}

	pr_debug("remaining pages to release=%ld\n", nr_pages);
	return nr_pages;
}

/*
 * Alloc a bunch of pages from the page pool. This will alloc as many as it can
 * and return the number of pages allocated. Pages are placed into the passed
 * array in a linear fashion starting from index 0.
 */
int nvmap_page_pool_alloc_lots(struct nvmap_page_pool *pool,
				struct page **pages, u32 nr)
{
	u32 ind = 0;
	u32 non_zero_idx;
	u32 non_zero_cnt = 0;

	if (!enable_pp || !nr)
		return 0;

	rt_mutex_lock(&pool->lock);

	while (ind < nr) {
		struct page *page = NULL;

		if (!non_zero_cnt)
			page = get_page_list_page(pool);

		if (!page) {
			page = get_zero_list_page(pool);
			if (!page)
				break;
			if (!non_zero_cnt)
				non_zero_idx = ind;
			non_zero_cnt++;
		}

		pages[ind++] = page;
		if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
			nvmap_pgcount(page, false);
			BUG_ON(page_count(page) != 1);
		}
	}

	rt_mutex_unlock(&pool->lock);

	/* Zero non-zeroed pages, if any */
	if (non_zero_cnt)
		nvmap_pp_zero_pages(&pages[non_zero_idx], non_zero_cnt);

	pp_alloc_add(pool, ind);
	pp_hit_add(pool, ind);
	pp_miss_add(pool, nr - ind);

	trace_nvmap_pp_alloc_lots(ind, nr);

	return ind;
}

int nvmap_page_pool_alloc_lots_bp(struct nvmap_page_pool *pool,
				struct page **pages, u32 nr)
{
	int ind = 0, nr_pages = nr;
	struct page *page;

	if (!enable_pp || pool->pages_per_big_pg <= 1 ||
	    nr_pages < pool->pages_per_big_pg)
		return 0;

	rt_mutex_lock(&pool->lock);

	while (nr_pages - ind >= pool->pages_per_big_pg) {
		int i;

		page = get_page_list_page_bp(pool);
		if (!page)
			break;

		for (i = 0; i < pool->pages_per_big_pg; i++)
			pages[ind + i] = nth_page(page, i);

		ind += pool->pages_per_big_pg;
	}

	rt_mutex_unlock(&pool->lock);
	return ind;
}

static bool nvmap_is_big_page(struct nvmap_page_pool *pool,
			      struct page **pages, int idx, int nr)
{
	int i;
	struct page *page = pages[idx];

	if (pool->pages_per_big_pg <= 1)
		return false;

	if (nr - idx < pool->pages_per_big_pg)
		return false;

	/* Allow coalescing pages at big page boundary only */
	if (page_to_phys(page) & (pool->big_pg_sz - 1))
		return false;

	for (i = 1; i < pool->pages_per_big_pg; i++)
		if (pages[idx + i] != nth_page(page, i))
			break;

	return i == pool->pages_per_big_pg ? true: false;
}

/*
 * Fill a bunch of pages into the page pool. This will fill as many as it can
 * and return the number of pages filled. Pages are used from the start of the
 * passed page pointer array in a linear fashion.
 *
 * You must lock the page pool before using this.
 */
static int __nvmap_page_pool_fill_lots_locked(struct nvmap_page_pool *pool,
				       struct page **pages, u32 nr)
{
	int real_nr;
	int ind = 0;

	if (!enable_pp)
		return 0;

	real_nr = min_t(u32, pool->max - pool->count, nr);
	BUG_ON(real_nr < 0);
	if (real_nr == 0)
		return 0;

	while (real_nr > 0) {
		if (IS_ENABLED(CONFIG_NVMAP_PAGE_POOL_DEBUG)) {
			nvmap_pgcount(pages[ind], true);
			BUG_ON(page_count(pages[ind]) != 2);
		}

		if (nvmap_is_big_page(pool, pages, ind, nr)) {
			list_add_tail(&pages[ind]->lru, &pool->page_list_bp);
			ind += pool->pages_per_big_pg;
			real_nr -= pool->pages_per_big_pg;
			pool->big_page_count += pool->pages_per_big_pg;
		} else {
			list_add_tail(&pages[ind++]->lru, &pool->page_list);
			real_nr--;
		}
	}

	pool->count += ind;
	BUG_ON(pool->count > pool->max);
	pp_fill_add(pool, ind);

	return ind;
}

int nvmap_page_pool_fill_lots(struct nvmap_page_pool *pool,
				       struct page **pages, u32 nr)
{
	int ret = 0;
	int i;
	u32 save_to_zero;

	rt_mutex_lock(&pool->lock);

	save_to_zero = pool->to_zero;

	ret = min(nr, pool->max - pool->count - pool->to_zero - pool->under_zero);

	for (i = 0; i < ret; i++) {
		/* If page has additonal referecnces, Don't add it into
		 * page pool. get_user_pages() on mmap'ed nvmap handle can
		 * hold a refcount on the page. These pages can't be
		 * reused till the additional refs are dropped.
		 */
		if (page_count(pages[i]) > 1) {
			__free_page(pages[i]);
		} else {
			list_add_tail(&pages[i]->lru, &pool->zero_list);
			pool->to_zero++;
		}
	}

	if (pool->to_zero)
		wake_up_interruptible(&nvmap_bg_wait);
	ret = i;

	trace_nvmap_pp_fill_zero_lots(save_to_zero, pool->to_zero,
			ret, nr);

	rt_mutex_unlock(&pool->lock);

	return ret;
}

ulong nvmap_page_pool_get_unused_pages(void)
{
	int total = 0;

	if (!nvmap_dev)
		return 0;

	total = nvmap_dev->pool.count + nvmap_dev->pool.to_zero;

	return total;
}

/*
 * Remove and free to the system all the pages currently in the page
 * pool. This operation will happen even if the page pools are disabled.
 */
int nvmap_page_pool_clear(void)
{
	struct nvmap_page_pool *pool = &nvmap_dev->pool;

	rt_mutex_lock(&pool->lock);

	(void)nvmap_page_pool_free_pages_locked(pool, pool->count + pool->to_zero);

	/* For some reason, if an error occured... */
	if (!list_empty(&pool->page_list) || !list_empty(&pool->zero_list)) {
		rt_mutex_unlock(&pool->lock);
		return -ENOMEM;
	}

	rt_mutex_unlock(&pool->lock);

	return 0;
}

/*
 * Resizes the page pool to the passed size. If the passed size is 0 then
 * all associated resources are released back to the system. This operation
 * will only occur if the page pools are enabled.
 */
static void nvmap_page_pool_resize(struct nvmap_page_pool *pool, u32 size)
{
	u32 curr;

	rt_mutex_lock(&pool->lock);

	curr = nvmap_page_pool_get_unused_pages();
	if (curr > size)
		(void)nvmap_page_pool_free_pages_locked(pool, curr - size);

	pr_debug("page pool resized to %d from %d pages\n", size, pool->max);
	pool->max = size;

	rt_mutex_unlock(&pool->lock);
}

static unsigned long nvmap_page_pool_count_objects(struct shrinker *shrinker,
						   struct shrink_control *sc)
{
	return nvmap_page_pool_get_unused_pages();
}

static unsigned long nvmap_page_pool_scan_objects(struct shrinker *shrinker,
						  struct shrink_control *sc)
{
	unsigned long remaining;

	pr_debug("sh_pages=%lu", sc->nr_to_scan);

	rt_mutex_lock(&nvmap_dev->pool.lock);
	remaining = nvmap_page_pool_free_pages_locked(
			&nvmap_dev->pool, sc->nr_to_scan);
	rt_mutex_unlock(&nvmap_dev->pool.lock);

	return (remaining == sc->nr_to_scan) ? \
			   SHRINK_STOP : (sc->nr_to_scan - remaining);
}

static struct shrinker nvmap_page_pool_shrinker = {
	.count_objects = nvmap_page_pool_count_objects,
	.scan_objects = nvmap_page_pool_scan_objects,
	.seeks = 1,
};

static void shrink_page_pools(int *total_pages, int *available_pages)
{
	struct shrink_control sc;

	if (*total_pages == 0) {
		sc.gfp_mask = GFP_KERNEL;
		sc.nr_to_scan = 0;
		*total_pages = nvmap_page_pool_count_objects(NULL, &sc);
	}
	sc.nr_to_scan = *total_pages;
	nvmap_page_pool_scan_objects(NULL, &sc);
	*available_pages = nvmap_page_pool_count_objects(NULL, &sc);
}

#if NVMAP_TEST_PAGE_POOL_SHRINKER
static int shrink_pp;
static int shrink_set(const char *arg, const struct kernel_param *kp)
{
	int cpu = smp_processor_id();
	unsigned long long t1, t2;
	int total_pages, available_pages;

	param_set_int(arg, kp);

	if (shrink_pp) {
		total_pages = shrink_pp;
		t1 = cpu_clock(cpu);
		shrink_page_pools(&total_pages, &available_pages);
		t2 = cpu_clock(cpu);
		pr_debug("shrink page pools: time=%lldns, "
			"total_pages_released=%d, free_pages_available=%d",
			t2-t1, total_pages, available_pages);
	}
	return 0;
}

static int shrink_get(char *buff, const struct kernel_param *kp)
{
	return param_get_int(buff, kp);
}

static struct kernel_param_ops shrink_ops = {
	.get = shrink_get,
	.set = shrink_set,
};

module_param_cb(shrink_page_pools, &shrink_ops, &shrink_pp, 0644);
#endif

static int enable_pp_set(const char *arg, const struct kernel_param *kp)
{
	int ret;

	ret = param_set_bool(arg, kp);
	if (ret)
		return ret;

	if (!enable_pp)
		nvmap_page_pool_clear();

	return 0;
}

static int enable_pp_get(char *buff, const struct kernel_param *kp)
{
	return param_get_bool(buff, kp);
}

static struct kernel_param_ops enable_pp_ops = {
	.get = enable_pp_get,
	.set = enable_pp_set,
};

module_param_cb(enable_page_pools, &enable_pp_ops, &enable_pp, 0644);

static int pool_size_set(const char *arg, const struct kernel_param *kp)
{
	int ret = param_set_uint(arg, kp);

	if (!ret && (pool_size != nvmap_dev->pool.max))
		nvmap_page_pool_resize(&nvmap_dev->pool, pool_size);

	return ret;
}

static int pool_size_get(char *buff, const struct kernel_param *kp)
{
	return param_get_int(buff, kp);
}

static struct kernel_param_ops pool_size_ops = {
	.get = pool_size_get,
	.set = pool_size_set,
};

module_param_cb(pool_size, &pool_size_ops, &pool_size, 0644);

int nvmap_page_pool_debugfs_init(struct dentry *nvmap_root)
{
	struct dentry *pp_root;

	if (!nvmap_root)
		return -ENODEV;

	pp_root = debugfs_create_dir("pagepool", nvmap_root);
	if (!pp_root)
		return -ENODEV;

	debugfs_create_u32("page_pool_available_pages",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.count);
	debugfs_create_u32("page_pool_pages_to_zero",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.to_zero);
	debugfs_create_u32("page_pool_available_big_pages",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.big_page_count);
	debugfs_create_u32("page_pool_big_page_size",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.big_pg_sz);
	debugfs_create_u64("total_big_page_allocs",
			   S_IRUGO, pp_root,
			   &nvmap_big_page_allocs);
	debugfs_create_u64("total_page_allocs",
			   S_IRUGO, pp_root,
			   &nvmap_total_page_allocs);

#ifdef CONFIG_NVMAP_PAGE_POOL_DEBUG
	debugfs_create_u64("page_pool_allocs",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.allocs);
	debugfs_create_u64("page_pool_fills",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.fills);
	debugfs_create_u64("page_pool_hits",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.hits);
	debugfs_create_u64("page_pool_misses",
			   S_IRUGO, pp_root,
			   &nvmap_dev->pool.misses);
#endif

	return 0;
}

int nvmap_page_pool_init(struct nvmap_device *dev)
{
	struct sysinfo info;
	struct nvmap_page_pool *pool = &dev->pool;

	memset(pool, 0x0, sizeof(*pool));
	rt_mutex_init(&pool->lock);
	INIT_LIST_HEAD(&pool->page_list);
	INIT_LIST_HEAD(&pool->zero_list);
	INIT_LIST_HEAD(&pool->page_list_bp);

	pool->big_pg_sz = NVMAP_PP_BIG_PAGE_SIZE;
	pool->pages_per_big_pg = NVMAP_PP_BIG_PAGE_SIZE >> PAGE_SHIFT;

	si_meminfo(&info);
	pr_info("Total RAM pages: %lu\n", info.totalram);

	if (!CONFIG_NVMAP_PAGE_POOL_SIZE)
		/* The ratio is pool pages per 1K ram pages.
		 * So, the >> 10 */
		pool->max = (info.totalram * NVMAP_PP_POOL_SIZE) >> 10;
	else
		pool->max = CONFIG_NVMAP_PAGE_POOL_SIZE;

	if (pool->max >= info.totalram)
		goto fail;
	pool_size = pool->max;

	pr_info("nvmap page pool size: %u pages (%u MB)\n", pool->max,
		(pool->max * info.mem_unit) >> 20);

	background_allocator = kthread_run(nvmap_background_zero_thread,
					    NULL, "nvmap-bz");
	if (IS_ERR(background_allocator))
		goto fail;

	register_shrinker(&nvmap_page_pool_shrinker);

	return 0;
fail:
	nvmap_page_pool_fini(dev);
	return -ENOMEM;
}

int nvmap_page_pool_fini(struct nvmap_device *dev)
{
	struct nvmap_page_pool *pool = &dev->pool;

	/*
	 * if background allocator is not initialzed or not
	 * properly initialized, then shrinker is also not
	 * registered
	 */
	if (!IS_ERR_OR_NULL(background_allocator)) {
		unregister_shrinker(&nvmap_page_pool_shrinker);
		kthread_stop(background_allocator);
	}

	WARN_ON(!list_empty(&pool->page_list));

	return 0;
}
