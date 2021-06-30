/*
 * drivers/video/tegra/nvmap/nvmap_handle.c
 *
 * Handle allocation and freeing routines for nvmap
 *
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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/dma-buf.h>
#include <linux/moduleparam.h>
#include <linux/nvmap.h>

#include <trace/events/nvmap.h>

#include "nvmap_handle.h"
#include "nvmap_handle_priv.h"
#include "nvmap_handle_ref.h"

int nvmap_handle_ref_get(struct nvmap_handle_ref *ref)
{
	if (!ref)
		return -1;

	atomic_inc(&ref->dupes);
	nvmap_handle_get(ref->handle);
	return 0;
}

int nvmap_handle_ref_count(struct nvmap_handle_ref *ref)
{
	return atomic_read(&ref->dupes);
}

struct nvmap_handle_ref *nvmap_handle_ref_create(struct nvmap_handle *handle)
{
	struct nvmap_handle_ref *ref = NULL;

	ref = kzalloc(sizeof(*ref), GFP_KERNEL);
	if (!ref) {
		return NULL;
	}

	atomic_set(&ref->dupes, 1);

	handle = nvmap_handle_get(handle);
	if (!handle) {
		kfree(ref);
		return NULL;
	}

	ref->handle = handle;
	atomic_inc(&handle->share_count);

	get_dma_buf(handle->dmabuf);
	return ref;
}

void nvmap_handle_ref_free(struct nvmap_handle_ref *ref)
{
	atomic_dec(&ref->handle->share_count);
	dma_buf_put(ref->handle->dmabuf);
	kfree(ref);
}

int nvmap_handle_ref_put(struct nvmap_handle_ref *ref)
{
	// TODO: Do the same error that handle does on ref dec
	nvmap_handle_put(ref->handle);
	return atomic_dec_return(&ref->dupes);
}
