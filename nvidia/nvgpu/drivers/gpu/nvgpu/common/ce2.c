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

#include <nvgpu/types.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/utils.h>

#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

#include "gk20a/ce2_gk20a.h"
#include "gk20a/fence_gk20a.h"

static inline int gk20a_get_valid_launch_flags(struct gk20a *g, int launch_flags)
{
	/* there is no local memory available,
	don't allow local memory related CE flags */
	if (!g->mm.vidmem.size) {
		launch_flags &= ~(NVGPU_CE_SRC_LOCATION_LOCAL_FB |
			NVGPU_CE_DST_LOCATION_LOCAL_FB);
	}
	return launch_flags;
}

int gk20a_ce_execute_ops(struct gk20a *g,
		u32 ce_ctx_id,
		u64 src_buf,
		u64 dst_buf,
		u64 size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		u32 submit_flags,
		struct gk20a_fence **gk20a_fence_out)
{
	int ret = -EPERM;
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;
	bool found = false;
	u32 *cmd_buf_cpu_va;
	u64 cmd_buf_gpu_va = 0;
	u32 methodSize;
	u32 cmd_buf_read_offset;
	u32 dma_copy_class;
	struct nvgpu_gpfifo_entry gpfifo;
	struct nvgpu_channel_fence fence = {0, 0};
	struct gk20a_fence *ce_cmd_buf_fence_out = NULL;

	if (!ce_app->initialised ||ce_app->app_state != NVGPU_CE_ACTIVE)
		goto end;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		if (ce_ctx->ctx_id == ce_ctx_id) {
			found = true;
			break;
		}
	}

	nvgpu_mutex_release(&ce_app->app_mutex);

	if (!found) {
		ret = -EINVAL;
		goto end;
	}

	if (ce_ctx->gpu_ctx_state != NVGPU_CE_GPU_CTX_ALLOCATED) {
		ret = -ENODEV;
		goto end;
	}

	nvgpu_mutex_acquire(&ce_ctx->gpu_ctx_mutex);

	ce_ctx->cmd_buf_read_queue_offset %= NVGPU_CE_MAX_INFLIGHT_JOBS;

	cmd_buf_read_offset = (ce_ctx->cmd_buf_read_queue_offset *
			(NVGPU_CE_MAX_COMMAND_BUFF_BYTES_PER_KICKOFF / sizeof(u32)));

	cmd_buf_cpu_va = (u32 *)ce_ctx->cmd_buf_mem.cpu_va;

	if (ce_ctx->postfences[ce_ctx->cmd_buf_read_queue_offset]) {
		struct gk20a_fence **prev_post_fence =
			&ce_ctx->postfences[ce_ctx->cmd_buf_read_queue_offset];

		ret = gk20a_fence_wait(g, *prev_post_fence,
				       gk20a_get_gr_idle_timeout(g));

		gk20a_fence_put(*prev_post_fence);
		*prev_post_fence = NULL;
		if (ret)
			goto noop;
	}

	cmd_buf_gpu_va = (ce_ctx->cmd_buf_mem.gpu_va + (u64)(cmd_buf_read_offset *sizeof(u32)));

	dma_copy_class = g->ops.get_litter_value(g, GPU_LIT_DMA_COPY_CLASS);
	methodSize = gk20a_ce_prepare_submit(src_buf,
					dst_buf,
					size,
					&cmd_buf_cpu_va[cmd_buf_read_offset],
					NVGPU_CE_MAX_COMMAND_BUFF_BYTES_PER_KICKOFF,
					payload,
					gk20a_get_valid_launch_flags(g, launch_flags),
					request_operation,
					dma_copy_class);

	if (methodSize) {
		/* store the element into gpfifo */
		gpfifo.entry0 =
			u64_lo32(cmd_buf_gpu_va);
		gpfifo.entry1 =
			(u64_hi32(cmd_buf_gpu_va) |
			pbdma_gp_entry1_length_f(methodSize));

		/* take always the postfence as it is needed for protecting the ce context */
		submit_flags |= NVGPU_SUBMIT_FLAGS_FENCE_GET;

		nvgpu_smp_wmb();

		ret = nvgpu_submit_channel_gpfifo_kernel(ce_ctx->ch, &gpfifo,
				1, submit_flags, &fence, &ce_cmd_buf_fence_out);

		if (!ret) {
			ce_ctx->postfences[ce_ctx->cmd_buf_read_queue_offset] =
				ce_cmd_buf_fence_out;
			if (gk20a_fence_out) {
				gk20a_fence_get(ce_cmd_buf_fence_out);
				*gk20a_fence_out = ce_cmd_buf_fence_out;
			}

			/* Next available command buffer queue Index */
			++ce_ctx->cmd_buf_read_queue_offset;
		}
	} else {
		ret = -ENOMEM;
	}
noop:
	nvgpu_mutex_release(&ce_ctx->gpu_ctx_mutex);
end:
	return ret;
}
