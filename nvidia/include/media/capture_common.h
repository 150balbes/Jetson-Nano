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

#include <media/mc_common.h>

/* Progress status */
#define PROGRESS_STATUS_BUSY		(U32_C(0x1))
#define PROGRESS_STATUS_DONE		(U32_C(0x2))

/* buffer details including dma_buf and iova etc. */
struct capture_common_buf {
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t iova;
};

/* unpin details for a capture channel, per request */
struct capture_common_unpins {
	uint32_t num_unpins;
	struct capture_common_buf data[];
};

struct capture_common_pin_req {
	struct device *dev;
	struct device *rtcpu_dev;
	struct capture_common_unpins *unpins;
	struct capture_common_buf *requests;
	struct capture_common_buf *requests_dev;
	uint32_t request_size;
	uint32_t request_offset;
	uint32_t requests_mem;
	uint32_t num_relocs;
	uint32_t __user *reloc_user;
};

struct capture_common_status_notifier {
	struct dma_buf *buf;
	void *va;
	uint32_t offset;
};

int capture_common_setup_progress_status_notifier(
		struct capture_common_status_notifier *status_notifier,
		uint32_t mem,
		uint32_t buffer_size,
		uint32_t mem_offset);

int capture_common_set_progress_status(
		struct capture_common_status_notifier *progress_status_notifier,
		uint32_t buffer_slot,
		uint32_t buffer_depth, uint8_t new_val);

int capture_common_release_progress_status_notifier(
		struct capture_common_status_notifier *progress_status_notifier);

int capture_common_pin_memory(struct device *dev,
		uint32_t mem, struct capture_common_buf *unpin_data);

void capture_common_unpin_memory(struct capture_common_buf *unpin_data);

int capture_common_request_pin_and_reloc(struct capture_common_pin_req *req);
