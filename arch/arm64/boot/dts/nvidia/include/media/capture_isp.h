/*
 * Tegra ISP capture operations
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: Sudhir Vyas <svyas@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ISP_CAPTURE_H__
#define __ISP_CAPTURE_H__

#if defined(__KERNEL__)
#include <linux/compiler.h>
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include <linux/ioctl.h>

#define __ISP_CAPTURE_ALIGN __aligned(8)

struct tegra_isp_channel;

struct capture_isp_reloc {
	uint32_t num_relocs;
	uint32_t __pad;
	uint64_t reloc_relatives;
}__ISP_CAPTURE_ALIGN;

struct isp_capture_setup {
	uint32_t channel_flags;
	uint32_t __pad_flags;

	/* isp capture descriptor ring buffer */
	uint32_t queue_depth;
	uint32_t request_size;
	uint32_t mem;

	/* isp_prgram descriptor ring buffer */
	uint32_t isp_program_queue_depth;
	uint32_t isp_program_request_size;
	uint32_t isp_program_mem;
} __ISP_CAPTURE_ALIGN;

struct isp_capture_info {
	struct isp_capture_syncpts {
		uint32_t progress_syncpt;
		uint32_t progress_syncpt_val;
		uint32_t stats_progress_syncpt;
		uint32_t stats_progress_syncpt_val;
	} syncpts;
} __ISP_CAPTURE_ALIGN;

struct isp_capture_req {
	uint32_t buffer_index;
	uint32_t __pad;
	/* relocation relatives */
	struct capture_isp_reloc isp_relocs;
	struct capture_isp_reloc inputfences_relocs;
	uint32_t gos_relative;
	uint32_t sp_relative;
	struct capture_isp_reloc prefences_relocs;
} __ISP_CAPTURE_ALIGN;

struct isp_program_req {
	uint32_t buffer_index;
	uint32_t __pad;
	/* relocation relatives */
	struct capture_isp_reloc isp_program_relocs;
} __ISP_CAPTURE_ALIGN;

struct isp_capture_req_ex {
	struct isp_capture_req capture_req;
	struct isp_program_req program_req;
	uint32_t __pad[4];
} __ISP_CAPTURE_ALIGN;

struct isp_capture_progress_status_req {
	uint32_t mem;
	uint32_t mem_offset;

	uint32_t process_buffer_depth;
	uint32_t program_buffer_depth;
	uint32_t __pad[4];
} __ISP_CAPTURE_ALIGN;


int isp_capture_init(struct tegra_isp_channel *chan);
void isp_capture_shutdown(struct tegra_isp_channel *chan);
int isp_capture_setup(struct tegra_isp_channel *chan,
		struct isp_capture_setup *setup);
int isp_capture_reset(struct tegra_isp_channel *chan,
		uint32_t reset_flags);
int isp_capture_release(struct tegra_isp_channel *chan,
		uint32_t reset_flags);
int isp_capture_get_info(struct tegra_isp_channel *chan,
		struct isp_capture_info *info);
int isp_capture_request(struct tegra_isp_channel *chan,
		struct isp_capture_req *req);
int isp_capture_status(struct tegra_isp_channel *chan,
		int32_t timeout_ms);
int isp_capture_program_request(struct tegra_isp_channel *chan,
		struct isp_program_req *req);
int isp_capture_program_status(struct tegra_isp_channel *chan);
int isp_capture_request_ex(struct tegra_isp_channel *chan,
		struct isp_capture_req_ex *capture_req_ex);
int isp_capture_set_progress_status_notifier(struct tegra_isp_channel *chan,
		struct isp_capture_progress_status_req *req);
#endif
