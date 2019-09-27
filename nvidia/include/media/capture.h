/*
 * Tegra Video Input capture operations
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author: David Wang <davidw@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __VI_CAPTURE_H__
#define __VI_CAPTURE_H__

#if defined(__KERNEL__)
#include <linux/compiler.h>
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include <media/capture_common.h>
#include <media/capture_vi_channel.h>
#include "soc/tegra/camrtc-capture.h"
#include "soc/tegra/camrtc-capture-messages.h"

#define __VI_CAPTURE_ALIGN __aligned(8)

struct tegra_vi_channel;

struct vi_capture {
	uint16_t channel_id;
	struct device *rtcpu_dev;
	struct tegra_vi_channel *vi_channel;
	struct capture_common_buf requests;
	size_t request_buf_size;
	uint32_t queue_depth;
	uint32_t request_size;
	bool is_mem_pinned;

	struct capture_common_status_notifier progress_status_notifier;
	uint32_t progress_status_buffer_depth;
	bool is_progress_status_notifier_set;

	uint32_t stream_id;
	uint32_t csi_port;
	uint32_t virtual_channel_id;

	uint32_t num_gos_tables;
	const dma_addr_t *gos_tables;

	struct syncpoint_info progress_sp;
	struct syncpoint_info embdata_sp;
	struct syncpoint_info linetimer_sp;

	struct completion control_resp;
	struct completion capture_resp;
	struct mutex control_msg_lock;
	struct CAPTURE_CONTROL_MSG control_resp_msg;

	struct mutex reset_lock;
	struct mutex unpins_list_lock;
	struct capture_common_unpins **unpins_list;

	uint64_t vi_channel_mask;
};

struct vi_capture_setup {
	uint32_t channel_flags;
	uint32_t error_mask_correctable;
	uint64_t vi_channel_mask;
	uint32_t queue_depth;
	uint32_t request_size;
	union {
		uint32_t mem;
		uint64_t iova;
	};
	uint8_t slvsec_stream_main;
	uint8_t slvsec_stream_sub;
	uint16_t __pad_slvsec1;
	uint32_t error_mask_uncorrectable;
} __VI_CAPTURE_ALIGN;

struct vi_capture_info {
	struct vi_capture_syncpts {
		uint32_t progress_syncpt;
		uint32_t progress_syncpt_val;
		uint32_t emb_data_syncpt;
		uint32_t emb_data_syncpt_val;
		uint32_t line_timer_syncpt;
		uint32_t line_timer_syncpt_val;
	} syncpts;
	uint32_t hw_channel_id;
	uint32_t __pad;
	uint64_t vi_channel_mask;
} __VI_CAPTURE_ALIGN;

struct vi_capture_control_msg {
	uint64_t ptr;
	uint32_t size;
	uint32_t __pad;
	uint64_t response;
} __VI_CAPTURE_ALIGN;

struct vi_capture_req {
	uint32_t buffer_index;
	uint32_t num_relocs;
	uint64_t reloc_relatives;
} __VI_CAPTURE_ALIGN;

struct vi_capture_progress_status_req {
	uint32_t mem;
	uint32_t mem_offset;

	uint32_t buffer_depth;
	uint32_t __pad[3];
} __VI_CAPTURE_ALIGN;
/*
 * The compand configuration describes a piece-wise linear
 * tranformation function used by the VI companding module.
 */
#define VI_CAPTURE_NUM_COMPAND_KNEEPTS 10
struct vi_capture_compand {
	uint32_t base[VI_CAPTURE_NUM_COMPAND_KNEEPTS];
	uint32_t scale[VI_CAPTURE_NUM_COMPAND_KNEEPTS];
	uint32_t offset[VI_CAPTURE_NUM_COMPAND_KNEEPTS];
} __VI_CAPTURE_ALIGN;


int vi_capture_init(struct tegra_vi_channel *chan, bool is_mem_pinned);
void vi_capture_shutdown(struct tegra_vi_channel *chan);
int vi_capture_setup(struct tegra_vi_channel *chan,
		struct vi_capture_setup *setup);
int vi_capture_reset(struct tegra_vi_channel *chan,
		uint32_t reset_flags);
int vi_capture_release(struct tegra_vi_channel *chan,
		uint32_t reset_flags);
int vi_capture_get_info(struct tegra_vi_channel *chan,
		struct vi_capture_info *info);
int vi_capture_control_message(struct tegra_vi_channel *chan,
		struct vi_capture_control_msg *msg);
int vi_capture_request(struct tegra_vi_channel *chan,
		struct vi_capture_req *req);
int vi_capture_status(struct tegra_vi_channel *chan,
		int32_t timeout_ms);
int vi_capture_set_compand(struct tegra_vi_channel *chan,
		struct vi_capture_compand *compand);
long vi_capture_ioctl(struct file *file, void *fh,
		bool use_prio, unsigned int cmd, void *arg);
int vi_capture_set_progress_status_notifier(struct tegra_vi_channel *chan,
		struct vi_capture_progress_status_req *req);
int csi_stream_release(struct tegra_vi_channel *chan);
#endif

