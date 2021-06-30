/*
 * Tegra capture common operations
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/nvhost.h>
#include <linux/slab.h>
#include <media/capture_common.h>
#include <media/mc_common.h>

struct surface_t {
	uint32_t offset;
	uint32_t offset_hi;
};

int capture_common_setup_progress_status_notifier(
		struct capture_common_status_notifier *status_notifier,
		uint32_t mem,
		uint32_t buffer_size,
		uint32_t mem_offset)
{
	struct dma_buf *dmabuf;
	void *va;

	/* take reference for the userctx */
	dmabuf = dma_buf_get(mem);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if ((buffer_size + mem_offset) > dmabuf->size) {
		dma_buf_put(dmabuf);
		pr_err("%s: invalid offset\n", __func__);
		return -EINVAL;
	}

	/* map handle and clear error notifier struct */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("%s: Cannot map notifier handle\n", __func__);
		return -ENOMEM;
	}

	memset(va, 0, buffer_size);

	status_notifier->buf = dmabuf;
	status_notifier->va = va;
	status_notifier->offset = mem_offset;
	return 0;
}

int capture_common_set_progress_status(
		struct capture_common_status_notifier *progress_status_notifier,
		uint32_t buffer_slot,
		uint32_t buffer_depth,
		uint8_t new_val)
{
	uint32_t *status_notifier = (uint32_t *) (progress_status_notifier->va +
			progress_status_notifier->offset);

	if (buffer_slot >= buffer_depth) {
		pr_err("%s: Invalid offset!", __func__);
		return -EINVAL;
	}

	/*
	 * Since UMD and KMD can both write to the shared progress status
	 * notifier buffer, insert memory barrier here to ensure that any
	 * other store operations to the buffer would be done before the
	 * write below.
	 */
	wmb();

	status_notifier[buffer_slot] = new_val;

	return 0;
}

int capture_common_release_progress_status_notifier(
		struct capture_common_status_notifier *progress_status_notifier)
{
	struct dma_buf *dmabuf = progress_status_notifier->buf;
	void *va = progress_status_notifier->va;

	if (dmabuf != NULL) {
		if (va != NULL)
			dma_buf_vunmap(dmabuf, va);

		dma_buf_put(dmabuf);
	}

	progress_status_notifier->buf = NULL;
	progress_status_notifier->va = NULL;
	progress_status_notifier->offset = 0;

	return 0;
}

int capture_common_pin_memory(struct device *dev,
		uint32_t mem, struct capture_common_buf *unpin_data)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	int err = 0;

	buf = dma_buf_get(mem);
	if (IS_ERR(buf)) {
		err = PTR_ERR(buf);
		goto fail;
	}

	attach = dma_buf_attach(buf, dev);
	if (IS_ERR(attach)) {
		err = PTR_ERR(attach);
		goto fail;
	}

	sgt = dma_buf_map_attachment(attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		err = PTR_ERR(sgt);
		goto fail;
	}

	if (sg_dma_address(sgt->sgl) == 0)
		sg_dma_address(sgt->sgl) = sg_phys(sgt->sgl);

	unpin_data->iova = sg_dma_address(sgt->sgl);
	unpin_data->buf = buf;
	unpin_data->attach = attach;
	unpin_data->sgt = sgt;

	return 0;

fail:
	capture_common_unpin_memory(unpin_data);
	return err;
}

void capture_common_unpin_memory(struct capture_common_buf *unpin_data)
{
	if (unpin_data->sgt != NULL)
		dma_buf_unmap_attachment(unpin_data->attach, unpin_data->sgt,
				DMA_BIDIRECTIONAL);
	if (unpin_data->attach != NULL)
		dma_buf_detach(unpin_data->buf, unpin_data->attach);
	if (unpin_data->buf != NULL)
		dma_buf_put(unpin_data->buf);

	unpin_data->sgt = NULL;
	unpin_data->attach = NULL;
	unpin_data->buf = NULL;
	unpin_data->iova = 0;
}

int capture_common_request_pin_and_reloc(struct capture_common_pin_req *req)
{
	uint32_t *reloc_relatives;
	void *reloc_page_addr = NULL;
	int last_page = -1;
	int i;
	int err = 0;

	if (!req) {
		pr_err("%s: NULL pin request", __func__);
		return -EINVAL;
	}

	if (req->unpins) {
		dev_err(req->dev, "%s: request unpins already exist", __func__);
		return -EEXIST;
	}

	req->unpins = kzalloc(sizeof(struct capture_common_unpins) +
		(sizeof(struct capture_common_buf) * req->num_relocs),
		GFP_KERNEL);
	if (unlikely(req->unpins == NULL)) {
		dev_err(req->dev, "failed to allocate request unpins\n");
		return -ENOMEM;
	}

	reloc_relatives = kcalloc(req->num_relocs, sizeof(uint32_t),
		GFP_KERNEL);
	if (unlikely(reloc_relatives == NULL)) {
		dev_err(req->dev, "failed to allocate request reloc array\n");
		err = -ENOMEM;
		goto reloc_fail;
	}

	err = copy_from_user(reloc_relatives, req->reloc_user,
			req->num_relocs * sizeof(uint32_t)) ? -EFAULT : 0;
	if (err < 0) {
		dev_err(req->dev, "failed to copy request user relocs\n");
		goto reloc_fail;
	}

	dev_dbg(req->dev, "%s: relocating %u addresses", __func__,
			req->num_relocs);

	for (i = 0; i < req->num_relocs; i++) {
		uint32_t reloc_relative = reloc_relatives[i];
		uint32_t reloc_offset = req->request_offset + reloc_relative;

		uint64_t surface_raw;
		struct surface_t *surface;
		uint32_t mem;
		uint32_t target_offset;
		dma_addr_t target_phys_addr = 0;

		dev_dbg(req->dev,
			"%s: idx:%i reloc:%u reloc_offset:%u", __func__,
			i, reloc_relative, reloc_offset);

		/* locate page of request in capture desc reloc is on */
		if (last_page != reloc_offset >> PAGE_SHIFT) {
			if (reloc_page_addr != NULL)
				dma_buf_kunmap(req->requests->buf,
					last_page, reloc_page_addr);

			reloc_page_addr = dma_buf_kmap(req->requests->buf,
						reloc_offset >> PAGE_SHIFT);
			last_page = reloc_offset >> PAGE_SHIFT;

			if (unlikely(reloc_page_addr == NULL)) {
				dev_err(req->dev,
					"%s: couldn't map request\n", __func__);
				err = -ENOMEM;
				goto pin_fail;
			}
		}

		/* read surf offset and mem handle from request descr */
		surface_raw = __raw_readq(
			(void __iomem *)(reloc_page_addr +
			(reloc_offset & ~PAGE_MASK)));
		surface = (struct surface_t *)&surface_raw;
		target_offset = surface->offset;
		mem = surface->offset_hi;

		if (!mem) {
			dev_err(req->dev,
					"%s: invalid mem handle\n", __func__);
			err = -EINVAL;
			goto pin_fail;
		}

		dev_dbg(req->dev, "%s: hmem:0x%x offset:0x%x\n", __func__,
				mem, target_offset);

		if (mem == req->requests_mem) {
			target_phys_addr = req->requests_dev->iova +
					req->request_offset + target_offset;
		} else {
			err = capture_common_pin_memory(req->dev, mem,
				&req->unpins->data[req->unpins->num_unpins]);
			if (err < 0) {
				dev_info(req->dev,
					"%s: pin memory failed pin count %d\n",
					__func__, req->unpins->num_unpins);
				goto pin_fail;
			}
			target_phys_addr =
				req->unpins->data[req->unpins->num_unpins].iova
					+ target_offset;

			req->unpins->num_unpins++;
		}

		if (!target_phys_addr) {
			dev_err(req->dev,
				"%s: target addr is NULL for mem 0x%x\n",
				__func__, mem);
			err = -EINVAL;
			goto pin_fail;
		}

		dev_dbg(req->dev,
			"%s: target addr 0x%llx at desc loc 0x%llx\n",
			__func__, (uint64_t)target_phys_addr,
			(uint64_t)reloc_page_addr +
			(reloc_offset & ~PAGE_MASK));

		/* write relocated physical address to request descr */
		__raw_writeq(
			target_phys_addr,
			(void __iomem *)(reloc_page_addr +
				(reloc_offset & ~PAGE_MASK)));

		dma_sync_single_range_for_device(req->rtcpu_dev,
				req->requests->iova, req->request_offset,
				req->request_size, DMA_TO_DEVICE);
	}

pin_fail:
	if (err) {
		for (i = 0; i < req->unpins->num_unpins; i++)
			capture_common_unpin_memory(&req->unpins->data[i]);
	}

reloc_fail:
	if (err)
		kfree(req->unpins);

	if (reloc_page_addr != NULL)
		dma_buf_kunmap(req->requests->buf, last_page,
			reloc_page_addr);

	kfree(reloc_relatives);

	return err;
}
