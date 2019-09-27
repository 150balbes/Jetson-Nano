/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_SEC2_H
#define NVGPU_SEC2_H

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/allocator.h>
#include <nvgpu/lock.h>
#include <nvgpu/flcnif_cmn.h>
#include <nvgpu/falcon.h>

#include <nvgpu/sec2if/sec2_cmd_if.h>
#include <nvgpu/sec2if/sec2_if_sec2.h>

#define NVGPU_SEC2_TRACE_BUFSIZE	(32U*1024U)

#define SEC2_MAX_NUM_SEQUENCES	(256U)
#define SEC2_SEQ_BIT_SHIFT		(5U)
#define SEC2_SEQ_TBL_SIZE	\
	(SEC2_MAX_NUM_SEQUENCES >> SEC2_SEQ_BIT_SHIFT)

#define SEC2_INVALID_SEQ_DESC	(~0U)

enum {
	SEC2_SEQ_STATE_FREE = 0U,
	SEC2_SEQ_STATE_PENDING,
	SEC2_SEQ_STATE_USED,
	SEC2_SEQ_STATE_CANCELLED
};

typedef void (*sec2_callback)(struct gk20a *, struct nv_flcn_msg_sec2 *,
	void *, u32, u32);

struct sec2_sequence {
	u8 id;
	u32 state;
	u32 desc;
	struct nv_flcn_msg_sec2 *msg;
	u8 *out_payload;
	sec2_callback callback;
	void *cb_params;
};

struct nvgpu_sec2 {
	struct gk20a *g;
	struct nvgpu_falcon *flcn;
	u32 falcon_id;

	struct nvgpu_falcon_queue queue[SEC2_QUEUE_NUM];

	struct sec2_sequence *seq;
	unsigned long sec2_seq_tbl[SEC2_SEQ_TBL_SIZE];
	u32 next_seq_desc;
	struct nvgpu_mutex sec2_seq_lock;

	bool isr_enabled;
	struct nvgpu_mutex isr_mutex;

	struct nvgpu_allocator dmem;

	/* set to true once init received */
	bool sec2_ready;

	struct nvgpu_mem trace_buf;

	void (*remove_support)(struct nvgpu_sec2 *sec2);

	u32 command_ack;
};

/* sec2 init */
int nvgpu_init_sec2_support(struct gk20a *g);
int nvgpu_sec2_destroy(struct gk20a *g);

#endif /* NVGPU_SEC2_H */
