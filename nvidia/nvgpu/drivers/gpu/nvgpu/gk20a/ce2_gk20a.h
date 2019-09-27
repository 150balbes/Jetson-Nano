/*
 * drivers/video/tegra/host/gk20a/fifo_gk20a.h
 *
 * GK20A graphics copy engine (gr host)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_GK20A_CE2_GK20A_H
#define NVGPU_GK20A_CE2_GK20A_H

struct channel_gk20a;
struct tsg_gk20a;

void gk20a_ce2_isr(struct gk20a *g, u32 inst_id, u32 pri_base);
u32 gk20a_ce2_nonstall_isr(struct gk20a *g, u32 inst_id, u32 pri_base);

/* CE command utility macros */
#define NVGPU_CE_LOWER_ADDRESS_OFFSET_MASK 0xffffffff
#define NVGPU_CE_UPPER_ADDRESS_OFFSET_MASK 0xff

#define NVGPU_CE_MAX_INFLIGHT_JOBS 32
#define NVGPU_CE_MAX_COMMAND_BUFF_BYTES_PER_KICKOFF 256

/* dma launch_flags */
enum {
	/* location */
	NVGPU_CE_SRC_LOCATION_COHERENT_SYSMEM                    = (1 << 0),
	NVGPU_CE_SRC_LOCATION_NONCOHERENT_SYSMEM                 = (1 << 1),
	NVGPU_CE_SRC_LOCATION_LOCAL_FB                           = (1 << 2),
	NVGPU_CE_DST_LOCATION_COHERENT_SYSMEM                    = (1 << 3),
	NVGPU_CE_DST_LOCATION_NONCOHERENT_SYSMEM                 = (1 << 4),
	NVGPU_CE_DST_LOCATION_LOCAL_FB                           = (1 << 5),

	/* memory layout */
	NVGPU_CE_SRC_MEMORY_LAYOUT_PITCH                         = (1 << 6),
	NVGPU_CE_SRC_MEMORY_LAYOUT_BLOCKLINEAR                   = (1 << 7),
	NVGPU_CE_DST_MEMORY_LAYOUT_PITCH                         = (1 << 8),
	NVGPU_CE_DST_MEMORY_LAYOUT_BLOCKLINEAR                   = (1 << 9),

	/* transfer type */
	NVGPU_CE_DATA_TRANSFER_TYPE_PIPELINED                   = (1 << 10),
	NVGPU_CE_DATA_TRANSFER_TYPE_NON_PIPELINED               = (1 << 11),
};

/* CE operation mode */
enum {
	NVGPU_CE_PHYS_MODE_TRANSFER        = (1 << 0),
	NVGPU_CE_MEMSET                    = (1 << 1),
};

/* CE app state machine flags */
enum {
	NVGPU_CE_ACTIVE                    = (1 << 0),
	NVGPU_CE_SUSPEND                   = (1 << 1),
};

/* gpu context state machine flags */
enum {
	NVGPU_CE_GPU_CTX_ALLOCATED         = (1 << 0),
	NVGPU_CE_GPU_CTX_DELETED           = (1 << 1),
};

/* global ce app db */
struct gk20a_ce_app {
	bool initialised;
	struct nvgpu_mutex app_mutex;
	int app_state;

	struct nvgpu_list_node allocated_contexts;
	u32 ctx_count;
	u32 next_ctx_id;
};

/* ce context db */
struct gk20a_gpu_ctx {
	struct gk20a *g;
	u32 ctx_id;
	struct nvgpu_mutex gpu_ctx_mutex;
	int gpu_ctx_state;

	/* tsg related data */
	struct tsg_gk20a *tsg;

	/* channel related data */
	struct channel_gk20a *ch;
	struct vm_gk20a *vm;

	/* cmd buf mem_desc */
	struct nvgpu_mem cmd_buf_mem;
	struct gk20a_fence *postfences[NVGPU_CE_MAX_INFLIGHT_JOBS];

	struct nvgpu_list_node list;

	u32 cmd_buf_read_queue_offset;
};

static inline struct gk20a_gpu_ctx *
gk20a_gpu_ctx_from_list(struct nvgpu_list_node *node)
{
	return (struct gk20a_gpu_ctx *)
		((uintptr_t)node - offsetof(struct gk20a_gpu_ctx, list));
};

/* global CE app related apis */
int gk20a_init_ce_support(struct gk20a *g);
void gk20a_ce_suspend(struct gk20a *g);
void gk20a_ce_destroy(struct gk20a *g);

/* CE app utility functions */
u32 gk20a_ce_create_context(struct gk20a *g,
		int runlist_id,
		int timeslice,
		int runlist_level);
int gk20a_ce_execute_ops(struct gk20a *g,
		u32 ce_ctx_id,
		u64 src_buf,
		u64 dst_buf,
		u64 size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		u32 submit_flags,
		struct gk20a_fence **gk20a_fence_out);
void gk20a_ce_delete_context_priv(struct gk20a *g,
		u32 ce_ctx_id);
void gk20a_ce_delete_context(struct gk20a *g,
		u32 ce_ctx_id);
int gk20a_ce_prepare_submit(u64 src_buf,
		u64 dst_buf,
		u64 size,
		u32 *cmd_buf_cpu_va,
		u32 max_cmd_buf_size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		u32 dma_copy_class);

#endif /*NVGPU_GK20A_CE2_GK20A_H*/
