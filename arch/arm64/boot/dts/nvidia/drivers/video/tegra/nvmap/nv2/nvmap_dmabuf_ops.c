/*
 * dma_buf exporter for nvmap
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/export.h>
#include <linux/nvmap.h>
#include <linux/dma-buf.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/stringify.h>
#include <linux/of.h>
#include <linux/platform/tegra/tegra_fd.h>
#include <linux/version.h>
#include <linux/iommu.h>

#include <trace/events/nvmap.h>

#include "nvmap_handle.h"
// TODO: refactor dmabuf_ops and remove this
#include "nvmap_handle_priv.h"

#include "nvmap_dmabuf.h"
#include "nvmap_dev.h"
#include "nvmap_vma.h"
#include "nvmap_client.h"

#include "nvmap_ioctl.h"

struct nvmap_handle_dmabuf_priv {
	void *priv;
	struct device *dev;
	void (*priv_release)(void *priv);
	struct list_head list;
};

// TODO: Remove these references to nvmap_fault
extern struct vm_operations_struct nvmap_vma_ops;
extern void nvmap_vma_open(struct vm_area_struct *vma);


/**
 * List node for maps of nvmap handles via the dma_buf API. These store the
 * necessary info for stashing mappings.
 *
 * @iommu_domain Domain for which this SGT is valid - for supporting multi-asid.
 * @dir DMA direction.
 * @sgt The scatter gather table to stash.
 * @refs Reference counting.
 * @maps_entry Entry on a given attachment's list of maps.
 * @stash_entry Entry on the stash list.
 * @owner The owner of this struct. There can be only one.
 */
struct nvmap_handle_sgt {
	struct iommu_domain *domain;
	enum dma_data_direction dir;
	struct sg_table *sgt;
	struct device *dev;

	atomic_t refs;

	struct list_head maps_entry;
	struct list_head stash_entry; /* lock the stash before accessing. */

	struct nvmap_handle_info *owner;
} ____cacheline_aligned_in_smp;

static DEFINE_MUTEX(nvmap_stashed_maps_lock);
static LIST_HEAD(nvmap_stashed_maps);
static struct kmem_cache *handle_sgt_cache;
struct dma_buf_ops nvmap_dma_buf_ops;

static bool nvmap_attach_handle_same_asid(struct dma_buf_attachment *attach,
					struct nvmap_handle_sgt *nvmap_sgt)
{
	return iommu_get_domain_for_dev(attach->dev) == nvmap_sgt->domain;

}

void nvmap_dmabufs_free(struct list_head *dmabuf_list)
{
	struct nvmap_handle_dmabuf_priv *curr, *next;

	list_for_each_entry_safe(curr, next, dmabuf_list, list) {
		curr->priv_release(curr->priv);
		list_del(&curr->list);
		kzfree(curr);
	}
}
/*
 * Initialize a kmem cache for allocating nvmap_handle_sgt's.
 */
int nvmap_dmabuf_stash_init(void)
{
	handle_sgt_cache = KMEM_CACHE(nvmap_handle_sgt, 0);
	if (IS_ERR_OR_NULL(handle_sgt_cache)) {
		pr_err("Failed to make kmem cache for nvmap_handle_sgt.\n");
		return -ENOMEM;
	}

	return 0;
}

static int nvmap_dmabuf_attach(struct dma_buf *dmabuf, struct device *dev,
			       struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_attach(dmabuf, dev);

	dev_dbg(dev, "%s() 0x%p\n", __func__, info->handle);
	return 0;
}

static void nvmap_dmabuf_detach(struct dma_buf *dmabuf,
				struct dma_buf_attachment *attach)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_detach(dmabuf, attach->dev);

	dev_dbg(attach->dev, "%s() 0x%p\n", __func__, info->handle);
}

/*
 * Make sure this mapping is no longer stashed - this corresponds to a "hit". If
 * the mapping is not stashed this is just a no-op.
 */
static void __nvmap_dmabuf_del_stash(struct nvmap_handle_sgt *nvmap_sgt)
{
	mutex_lock(&nvmap_stashed_maps_lock);
	if (list_empty(&nvmap_sgt->stash_entry)) {
		mutex_unlock(&nvmap_stashed_maps_lock);
		return;
	}

	pr_debug("Removing map from stash.\n");
	list_del_init(&nvmap_sgt->stash_entry);
	mutex_unlock(&nvmap_stashed_maps_lock);
}

static inline bool access_vpr_phys(struct device *dev)
{
	if (!device_is_iommuable(dev))
		return true;

	/*
	 * Assumes gpu nodes always have DT entry, this is valid as device
	 * specifying access-vpr-phys will do so through its DT entry.
	 */
	if (!dev->of_node)
		return false;

	return !!of_find_property(dev->of_node, "access-vpr-phys", NULL);
}

/*
 * Free an sgt completely. This will bypass the ref count. This also requires
 * the nvmap_sgt's owner's lock is already taken.
 */
static void __nvmap_dmabuf_free_sgt_locked(struct nvmap_handle_sgt *nvmap_sgt)
{
	struct nvmap_handle_info *info = nvmap_sgt->owner;
	u32 heap_type;
	DEFINE_DMA_ATTRS(attrs);

	list_del(&nvmap_sgt->maps_entry);

	heap_type = nvmap_handle_heap_type(info->handle);

	if (!(nvmap_dev->dynamic_dma_map_mask & heap_type)) {
		sg_dma_address(nvmap_sgt->sgt->sgl) = 0;
	} else if (heap_type == NVMAP_HEAP_CARVEOUT_VPR &&
			access_vpr_phys(nvmap_sgt->dev)) {
		sg_dma_address(nvmap_sgt->sgt->sgl) = 0;
	} else {
		dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, __DMA_ATTR(attrs));
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, __DMA_ATTR(attrs));
		dma_unmap_sg_attrs(nvmap_sgt->dev,
				   nvmap_sgt->sgt->sgl, nvmap_sgt->sgt->nents,
				   nvmap_sgt->dir, __DMA_ATTR(attrs));
	}
	__nvmap_free_sg_table(NULL, info->handle, nvmap_sgt->sgt);

	WARN(atomic_read(&nvmap_sgt->refs), "nvmap: Freeing reffed SGT!");
	kmem_cache_free(handle_sgt_cache, nvmap_sgt);
}

/*
 * Evict an entry from the IOVA stash. This does not do anything to the actual
 * mapping itself - this merely takes the passed nvmap_sgt out of the stash
 * and decrements the necessary cache stats.
 */
static void __nvmap_dmabuf_evict_stash_locked(
			struct nvmap_handle_sgt *nvmap_sgt)
{
	if (!list_empty(&nvmap_sgt->stash_entry))
		list_del_init(&nvmap_sgt->stash_entry);
}

/*
 * Locks the stash before doing the eviction.
 */
static void __nvmap_dmabuf_evict_stash(struct nvmap_handle_sgt *nvmap_sgt)
{
	mutex_lock(&nvmap_stashed_maps_lock);
	__nvmap_dmabuf_evict_stash_locked(nvmap_sgt);
	mutex_unlock(&nvmap_stashed_maps_lock);
}

/*
 * Prepare an SGT for potential stashing later on.
 */
static int __nvmap_dmabuf_prep_sgt_locked(struct dma_buf_attachment *attach,
				   enum dma_data_direction dir,
				   struct sg_table *sgt)
{
	struct nvmap_handle_sgt *nvmap_sgt;
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	pr_debug("Prepping SGT.\n");
	nvmap_sgt = kmem_cache_alloc(handle_sgt_cache, GFP_KERNEL);
	if (IS_ERR_OR_NULL(nvmap_sgt)) {
		pr_err("Prepping SGT failed.\n");
		return -ENOMEM;
	}

	nvmap_sgt->domain = iommu_get_domain_for_dev(attach->dev);
	nvmap_sgt->dir = dir;
	nvmap_sgt->sgt = sgt;
	nvmap_sgt->dev = attach->dev;
	nvmap_sgt->owner = info;
	INIT_LIST_HEAD(&nvmap_sgt->stash_entry);
	atomic_set(&nvmap_sgt->refs, 1);
	list_add(&nvmap_sgt->maps_entry, &info->maps);
	return 0;
}

/*
 * Called when an SGT is no longer being used by a device. This will not
 * necessarily free the SGT - instead it may stash the SGT.
 */
static void __nvmap_dmabuf_stash_sgt_locked(struct dma_buf_attachment *attach,
				    enum dma_data_direction dir,
				    struct sg_table *sgt)
{
	struct nvmap_handle_sgt *nvmap_sgt;
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	pr_debug("Stashing SGT - if necessary.\n");
	list_for_each_entry(nvmap_sgt, &info->maps, maps_entry) {
		if (nvmap_sgt->sgt == sgt) {
			if (!atomic_sub_and_test(1, &nvmap_sgt->refs))
				goto done;

			__nvmap_dmabuf_free_sgt_locked(nvmap_sgt);
			goto done;
		}
	}

done:
	return;
}

/*
 * Checks if there is already a map for this attachment. If so increment the
 * ref count on said map and return the associated sg_table. Otherwise return
 * NULL.
 *
 * If it turns out there is a map, this also checks to see if the map needs to
 * be removed from the stash - if so, the map is removed.
 */
static struct sg_table *__nvmap_dmabuf_get_sgt_locked(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct nvmap_handle_sgt *nvmap_sgt;
	struct sg_table *sgt = NULL;
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	pr_debug("Getting SGT from stash.\n");
	list_for_each_entry(nvmap_sgt, &info->maps, maps_entry) {
		if (!nvmap_attach_handle_same_asid(attach, nvmap_sgt))
			continue;

		/* We have a hit. */
		pr_debug("Stash hit (%s)!\n", dev_name(attach->dev));
		sgt = nvmap_sgt->sgt;
		atomic_inc(&nvmap_sgt->refs);
		__nvmap_dmabuf_del_stash(nvmap_sgt);
		break;
	}

	return sgt;
}

/*
 * If stashing is disabled then the stash related ops become no-ops.
 */
struct sg_table *_nvmap_dmabuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;
	int ents = 0;
	struct sg_table *sgt;
	DEFINE_DMA_ATTRS(attrs);
	u32 heap_type = nvmap_handle_heap_type(info->handle);
	atomic_t *h_pin = nvmap_handle_pin(info->handle);

	trace_nvmap_dmabuf_map_dma_buf(attach->dmabuf, attach->dev);

	nvmap_lru_reset(nvmap_handle_lru(info->handle));
	mutex_lock(&info->maps_lock);

	atomic_inc(h_pin);

	sgt = __nvmap_dmabuf_get_sgt_locked(attach, dir);
	if (sgt)
		goto cache_hit;

	sgt = __nvmap_sg_table(NULL, info->handle);
	if (IS_ERR(sgt)) {
		atomic_dec(h_pin);
		mutex_unlock(&info->maps_lock);
		return sgt;
	}

	if (!nvmap_handle_is_allocated(info->handle)) {
		goto err_map;
	} else if (!(nvmap_dev->dynamic_dma_map_mask & heap_type)) {
		sg_dma_address(sgt->sgl) = info->handle->carveout->base;
	} else if (heap_type == NVMAP_HEAP_CARVEOUT_VPR &&
			access_vpr_phys(attach->dev)) {
		sg_dma_address(sgt->sgl) = 0;
	} else {
		dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, __DMA_ATTR(attrs));
		dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, __DMA_ATTR(attrs));
		ents = dma_map_sg_attrs(attach->dev, sgt->sgl,
					sgt->nents, dir, __DMA_ATTR(attrs));
		if (ents <= 0)
			goto err_map;
	}

	if (__nvmap_dmabuf_prep_sgt_locked(attach, dir, sgt)) {
		WARN(1, "No mem to prep sgt.\n");
		goto err_prep;
	}

cache_hit:
	attach->priv = sgt;
	mutex_unlock(&info->maps_lock);
	return sgt;

err_prep:
	dma_unmap_sg_attrs(attach->dev, sgt->sgl, sgt->nents, dir, __DMA_ATTR(attrs));
err_map:
	__nvmap_free_sg_table(NULL, info->handle, sgt);
	atomic_dec(h_pin);
	mutex_unlock(&info->maps_lock);
	return ERR_PTR(-ENOMEM);
}

__weak struct sg_table *nvmap_dmabuf_map_dma_buf(
	struct dma_buf_attachment *attach, enum dma_data_direction dir)
{
	return _nvmap_dmabuf_map_dma_buf(attach, dir);
}

void _nvmap_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = attach->dmabuf->priv;

	trace_nvmap_dmabuf_unmap_dma_buf(attach->dmabuf, attach->dev);

	mutex_lock(&info->maps_lock);
	if (!atomic_add_unless(nvmap_handle_pin(info->handle), -1, 0)) {
		mutex_unlock(&info->maps_lock);
		WARN(1, "Unpinning handle that has yet to be pinned!\n");
		return;
	}
	__nvmap_dmabuf_stash_sgt_locked(attach, dir, sgt);
	mutex_unlock(&info->maps_lock);
}

__weak void nvmap_dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
	_nvmap_dmabuf_unmap_dma_buf(attach, sgt, dir);
}

static void nvmap_dmabuf_release(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;
	struct nvmap_handle_sgt *nvmap_sgt;

	trace_nvmap_dmabuf_release(info->handle->owner ?
			   nvmap_client_name(info->handle->owner) : "unknown",
			   info->handle,
			   dmabuf);

	mutex_lock(&info->handle->lock);
	BUG_ON(dmabuf != info->handle->dmabuf);
	info->handle->dmabuf = NULL;
	mutex_unlock(&info->handle->lock);

	mutex_lock(&info->maps_lock);
	while (!list_empty(&info->maps)) {
		nvmap_sgt = list_first_entry(&info->maps,
					     struct nvmap_handle_sgt,
					     maps_entry);
		__nvmap_dmabuf_evict_stash(nvmap_sgt);
		__nvmap_dmabuf_free_sgt_locked(nvmap_sgt);
	}
	mutex_unlock(&info->maps_lock);

	nvmap_handle_put(info->handle);
	kfree(info);
}

static int nvmap_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
					  size_t start, size_t len,
					  enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_begin_cpu_access(dmabuf, start, len);
	return nvmap_handle_cache_maint(info->handle,
						start, start + len,
						NVMAP_CACHE_OP_WB_INV);
}

static void nvmap_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
				       size_t start, size_t len,
				       enum dma_data_direction dir)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_end_cpu_access(dmabuf, start, len);
	nvmap_handle_cache_maint(info->handle,
				   start, start + len,
				   NVMAP_CACHE_OP_WB);
}

static void *nvmap_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long page_num)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_kmap(dmabuf);
	return __nvmap_kmap(info->handle, page_num);
}

static void nvmap_dmabuf_kunmap(struct dma_buf *dmabuf,
		unsigned long page_num, void *addr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_kunmap(dmabuf);
	__nvmap_kunmap(info->handle, page_num, addr);
}

static void *nvmap_dmabuf_kmap_atomic(struct dma_buf *dmabuf,
				      unsigned long page_num)
{
	WARN(1, "%s() can't be called from atomic\n", __func__);
	return NULL;
}

int __nvmap_map(struct nvmap_handle *h, struct vm_area_struct *vma)
{
	struct nvmap_vma_priv *priv;
	u32 heap_type;

	h = nvmap_handle_get(h);
	if (!h)
		return -EINVAL;

	heap_type = nvmap_handle_heap_type(h);
	if (!(heap_type & nvmap_dev->cpu_access_mask)) {
		nvmap_handle_put(h);
		return -EPERM;
	}

	/*
	 * Don't allow mmap on VPR memory as it would be mapped
	 * as device memory. User space shouldn't be accessing
	 * device memory.
	 */
	if (heap_type == NVMAP_HEAP_CARVEOUT_VPR)  {
		nvmap_handle_put(h);
		return -EPERM;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		nvmap_handle_put(h);
		return -ENOMEM;
	}
	priv->handle = h;

	vma->vm_flags |= VM_SHARED | VM_DONTEXPAND |
			  VM_DONTDUMP | VM_DONTCOPY |
			  (h->heap_pgalloc ? 0 : VM_PFNMAP);
	vma->vm_ops = &nvmap_vma_ops;
	BUG_ON(vma->vm_private_data != NULL);
	vma->vm_private_data = priv;
	vma->vm_page_prot = nvmap_handle_pgprot(h, vma->vm_page_prot);
	nvmap_vma_open(vma);
	return 0;
}

static int nvmap_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_mmap(dmabuf);

	return __nvmap_map(info->handle, vma);
}

static void *nvmap_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_vmap(dmabuf);
	return __nvmap_mmap(info->handle);
}

static void nvmap_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct nvmap_handle_info *info = dmabuf->priv;

	trace_nvmap_dmabuf_vunmap(dmabuf);
	__nvmap_munmap(info->handle, vaddr);
}

static int nvmap_dmabuf_set_private(struct dma_buf *dmabuf,
		struct device *dev, void *priv, void (*delete)(void *priv))
{
	struct nvmap_handle_info *info = dmabuf->priv;
	struct nvmap_handle *handle = info->handle;
	struct nvmap_handle_dmabuf_priv *curr = NULL;
	int ret = 0;

	mutex_lock(&handle->lock);
	list_for_each_entry(curr, &handle->dmabuf_priv, list) {
		if (curr->dev == dev) {
			ret = -EEXIST;
			goto unlock;
		}
	}

	curr = kmalloc(sizeof(*curr), GFP_KERNEL);
	if (!curr) {
		ret = -ENOMEM;
		goto unlock;
	}
	curr->priv = priv;
	curr->dev = dev;
	curr->priv_release = delete;
	list_add_tail(&curr->list, &handle->dmabuf_priv);
unlock:
	mutex_unlock(&handle->lock);
	return ret;
}

static void *nvmap_dmabuf_get_private(struct dma_buf *dmabuf,
		struct device *dev)
{
	void *priv = NULL;
	struct nvmap_handle_info *info = dmabuf->priv;
	struct nvmap_handle *handle = info->handle;
	struct nvmap_handle_dmabuf_priv *curr = NULL;

	mutex_lock(&handle->lock);
	list_for_each_entry(curr, &handle->dmabuf_priv, list) {
		if (curr->dev == dev) {
			priv = curr->priv;
			goto unlock;
		}
	}
unlock:
	mutex_unlock(&handle->lock);
	return priv;
}

struct dma_buf_ops nvmap_dma_buf_ops = {
	.attach		= nvmap_dmabuf_attach,
	.detach		= nvmap_dmabuf_detach,
	.map_dma_buf	= nvmap_dmabuf_map_dma_buf,
	.unmap_dma_buf	= nvmap_dmabuf_unmap_dma_buf,
	.release	= nvmap_dmabuf_release,
	.begin_cpu_access = nvmap_dmabuf_begin_cpu_access,
	.end_cpu_access = nvmap_dmabuf_end_cpu_access,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	.map_atomic	= nvmap_dmabuf_kmap_atomic,
	.map		= nvmap_dmabuf_kmap,
	.unmap		= nvmap_dmabuf_kunmap,
#else
	.kmap_atomic	= nvmap_dmabuf_kmap_atomic,
	.kmap		= nvmap_dmabuf_kmap,
	.kunmap		= nvmap_dmabuf_kunmap,
#endif
	.mmap		= nvmap_dmabuf_mmap,
	.vmap		= nvmap_dmabuf_vmap,
	.vunmap		= nvmap_dmabuf_vunmap,
	.set_drvdata	= nvmap_dmabuf_set_private,
	.get_drvdata	= nvmap_dmabuf_get_private,
};

bool dmabuf_is_nvmap(struct dma_buf *dmabuf)
{
	return dmabuf->ops == &nvmap_dma_buf_ops;
}
EXPORT_SYMBOL(dmabuf_is_nvmap);
