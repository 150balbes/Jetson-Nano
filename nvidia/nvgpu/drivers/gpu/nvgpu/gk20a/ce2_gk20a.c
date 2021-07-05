/*
 * GK20A Graphics Copy Engine  (gr host)
 *
 * Copyright (c) 2011-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/kmem.h>
#include <nvgpu/dma.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>
#include <nvgpu/power_features/cg.h>

#include "gk20a.h"
#include "gk20a/fence_gk20a.h"

#include <nvgpu/hw/gk20a/hw_ce2_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ram_gk20a.h>
#include <nvgpu/hw/gk20a/hw_top_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/barrier.h>

/*
 * Copy engine defines line size in pixels
 */
#define MAX_CE_SHIFT	31	/* 4Gpixels -1 */
#define MAX_CE_MASK	((u32) (~(~0U << MAX_CE_SHIFT)))
#define MAX_CE_ALIGN(a)	(a & MAX_CE_MASK)


static u32 ce2_nonblockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	nvgpu_log(g, gpu_dbg_intr, "ce2 non-blocking pipe interrupt\n");

	return ce2_intr_status_nonblockpipe_pending_f();
}

static u32 ce2_blockpipe_isr(struct gk20a *g, u32 fifo_intr)
{
	nvgpu_log(g, gpu_dbg_intr, "ce2 blocking pipe interrupt\n");

	return ce2_intr_status_blockpipe_pending_f();
}

static u32 ce2_launcherr_isr(struct gk20a *g, u32 fifo_intr)
{
	nvgpu_log(g, gpu_dbg_intr, "ce2 launch error interrupt\n");

	return ce2_intr_status_launcherr_pending_f();
}

void gk20a_ce2_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
{
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());
	u32 clear_intr = 0;

	nvgpu_log(g, gpu_dbg_intr, "ce2 isr %08x\n", ce2_intr);

	/* clear blocking interrupts: they exibit broken behavior */
	if (ce2_intr & ce2_intr_status_blockpipe_pending_f()) {
		clear_intr |= ce2_blockpipe_isr(g, ce2_intr);
	}

	if (ce2_intr & ce2_intr_status_launcherr_pending_f()) {
		clear_intr |= ce2_launcherr_isr(g, ce2_intr);
	}

	gk20a_writel(g, ce2_intr_status_r(), clear_intr);
	return;
}

u32 gk20a_ce2_nonstall_isr(struct gk20a *g, u32 inst_id, u32 pri_base)
{
	u32 ops = 0;
	u32 ce2_intr = gk20a_readl(g, ce2_intr_status_r());

	nvgpu_log(g, gpu_dbg_intr, "ce2 nonstall isr %08x\n", ce2_intr);

	if (ce2_intr & ce2_intr_status_nonblockpipe_pending_f()) {
		gk20a_writel(g, ce2_intr_status_r(),
			ce2_nonblockpipe_isr(g, ce2_intr));
		ops |= (GK20A_NONSTALL_OPS_WAKEUP_SEMAPHORE |
			GK20A_NONSTALL_OPS_POST_EVENTS);
	}
	return ops;
}

/* static CE app api */
static void gk20a_ce_put_fences(struct gk20a_gpu_ctx *ce_ctx)
{
	u32 i;

	for (i = 0; i < NVGPU_CE_MAX_INFLIGHT_JOBS; i++) {
		struct gk20a_fence **fence = &ce_ctx->postfences[i];
		if (*fence) {
			gk20a_fence_put(*fence);
		}
		*fence = NULL;
	}
}

/* assume this api should need to call under nvgpu_mutex_acquire(&ce_app->app_mutex) */
static void gk20a_ce_delete_gpu_context(struct gk20a_gpu_ctx *ce_ctx)
{
	struct nvgpu_list_node *list = &ce_ctx->list;

	ce_ctx->gpu_ctx_state = NVGPU_CE_GPU_CTX_DELETED;

	nvgpu_mutex_acquire(&ce_ctx->gpu_ctx_mutex);

	if (nvgpu_mem_is_valid(&ce_ctx->cmd_buf_mem)) {
		gk20a_ce_put_fences(ce_ctx);
		nvgpu_dma_unmap_free(ce_ctx->vm, &ce_ctx->cmd_buf_mem);
	}

	/*
	 * free the channel
	 * gk20a_channel_close() will also unbind the channel from TSG
	 */
	gk20a_channel_close(ce_ctx->ch);
	nvgpu_ref_put(&ce_ctx->tsg->refcount, gk20a_tsg_release);

	/* housekeeping on app */
	if (list->prev && list->next) {
		nvgpu_list_del(list);
	}

	nvgpu_mutex_release(&ce_ctx->gpu_ctx_mutex);
	nvgpu_mutex_destroy(&ce_ctx->gpu_ctx_mutex);

	nvgpu_kfree(ce_ctx->g, ce_ctx);
}

static inline unsigned int gk20a_ce_get_method_size(int request_operation,
			u64 size)
{
	/* failure size */
	unsigned int methodsize = UINT_MAX;
	unsigned int iterations = 0;
	u32 shift;
	u64 chunk = size;
	u32 height, width;

	while (chunk) {
		iterations++;

		shift = MAX_CE_ALIGN(chunk) ? __ffs(MAX_CE_ALIGN(chunk)) :
						MAX_CE_SHIFT;
		width = chunk >> shift;
		height = 1 << shift;
		width = MAX_CE_ALIGN(width);

		chunk -= (u64) height * width;
	}

	if (request_operation & NVGPU_CE_PHYS_MODE_TRANSFER) {
		methodsize = (2 + (16 * iterations)) * sizeof(u32);
	} else if (request_operation & NVGPU_CE_MEMSET) {
		methodsize = (2 + (15 * iterations)) * sizeof(u32);
	}

	return methodsize;
}

int gk20a_ce_prepare_submit(u64 src_buf,
		u64 dst_buf,
		u64 size,
		u32 *cmd_buf_cpu_va,
		u32 max_cmd_buf_size,
		unsigned int payload,
		int launch_flags,
		int request_operation,
		u32 dma_copy_class)
{
	u32 launch = 0;
	u32 methodSize = 0;
	u64 offset = 0;
	u64 chunk_size = 0;
	u64 chunk = size;

	/* failure case handling */
	if ((gk20a_ce_get_method_size(request_operation, size) >
		max_cmd_buf_size) || (!size) ||
		(request_operation > NVGPU_CE_MEMSET)) {
		return 0;
	}

	/* set the channel object */
	cmd_buf_cpu_va[methodSize++] = 0x20018000;
	cmd_buf_cpu_va[methodSize++] = dma_copy_class;

	/*
	 * The purpose clear the memory in 2D rectangles. We get the ffs to
	 * determine the number of lines to copy. The only constraint is that
	 * maximum number of pixels per line is 4Gpix - 1, which is awkward for
	 * calculation, so we settle to 2Gpix per line to make calculatione
	 * more agreable
	 */

	/* The copy engine in 2D mode can have (2^32 - 1) x (2^32 - 1) pixels in
	 * a single submit, we are going to try to clear a range of up to 2Gpix
	 * multiple lines. Because we want to copy byte aligned we will be
	 * setting 1 byte pixels */

	/*
	 * per iteration
	 * <------------------------- 40 bits ------------------------------>
	 *                                             1 <------ ffs ------->
	 *        <-----------up to 30 bits----------->
	 */
	while (chunk) {
		u32 width, height, shift;

		/*
		 * We will be aligning to bytes, making the maximum number of
		 * pix per line 2Gb
		 */

		shift = MAX_CE_ALIGN(chunk) ? __ffs(MAX_CE_ALIGN(chunk)) :
						MAX_CE_SHIFT;
		height = chunk >> shift;
		width = 1 << shift;
		height = MAX_CE_ALIGN(height);

		chunk_size = (u64) height * width;

		/* reset launch flag */
		launch = 0;

		if (request_operation & NVGPU_CE_PHYS_MODE_TRANSFER) {
			/* setup the source */
			cmd_buf_cpu_va[methodSize++] = 0x20028100;
			cmd_buf_cpu_va[methodSize++] = (u64_hi32(src_buf +
				offset) & NVGPU_CE_UPPER_ADDRESS_OFFSET_MASK);
			cmd_buf_cpu_va[methodSize++] = (u64_lo32(src_buf +
				offset) & NVGPU_CE_LOWER_ADDRESS_OFFSET_MASK);

			cmd_buf_cpu_va[methodSize++] = 0x20018098;
			if (launch_flags & NVGPU_CE_SRC_LOCATION_LOCAL_FB) {
				cmd_buf_cpu_va[methodSize++] = 0x00000000;
			} else if (launch_flags &
				NVGPU_CE_SRC_LOCATION_NONCOHERENT_SYSMEM) {
				cmd_buf_cpu_va[methodSize++] = 0x00000002;
			} else {
				cmd_buf_cpu_va[methodSize++] = 0x00000001;
			}

			launch |= 0x00001000;
		} else if (request_operation & NVGPU_CE_MEMSET) {
			/* Remap from component A on 1 byte wide pixels */
			cmd_buf_cpu_va[methodSize++] = 0x200181c2;
			cmd_buf_cpu_va[methodSize++] = 0x00000004;

			cmd_buf_cpu_va[methodSize++] = 0x200181c0;
			cmd_buf_cpu_va[methodSize++] = payload;

			launch |= 0x00000400;
		} else {
			/* Illegal size */
			return 0;
		}

		/* setup the destination/output */
		cmd_buf_cpu_va[methodSize++] = 0x20068102;
		cmd_buf_cpu_va[methodSize++] = (u64_hi32(dst_buf +
			offset) & NVGPU_CE_UPPER_ADDRESS_OFFSET_MASK);
		cmd_buf_cpu_va[methodSize++] = (u64_lo32(dst_buf +
			offset) & NVGPU_CE_LOWER_ADDRESS_OFFSET_MASK);
		/* Pitch in/out */
		cmd_buf_cpu_va[methodSize++] = width;
		cmd_buf_cpu_va[methodSize++] = width;
		/* width and line count */
		cmd_buf_cpu_va[methodSize++] = width;
		cmd_buf_cpu_va[methodSize++] = height;

		cmd_buf_cpu_va[methodSize++] = 0x20018099;
		if (launch_flags & NVGPU_CE_DST_LOCATION_LOCAL_FB) {
			cmd_buf_cpu_va[methodSize++] = 0x00000000;
		} else if (launch_flags &
				NVGPU_CE_DST_LOCATION_NONCOHERENT_SYSMEM) {
			cmd_buf_cpu_va[methodSize++] = 0x00000002;
		} else {
			cmd_buf_cpu_va[methodSize++] = 0x00000001;
		}

		launch |= 0x00002005;

		if (launch_flags & NVGPU_CE_SRC_MEMORY_LAYOUT_BLOCKLINEAR) {
			launch |= 0x00000000;
		} else {
			launch |= 0x00000080;
		}

		if (launch_flags & NVGPU_CE_DST_MEMORY_LAYOUT_BLOCKLINEAR) {
			launch |= 0x00000000;
		} else {
			launch |= 0x00000100;
		}

		cmd_buf_cpu_va[methodSize++] = 0x200180c0;
		cmd_buf_cpu_va[methodSize++] = launch;
		offset += chunk_size;
		chunk -= chunk_size;
	}

	return methodSize;
}

/* global CE app related apis */
int gk20a_init_ce_support(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	int err;
	u32 ce_reset_mask;

	ce_reset_mask = gk20a_fifo_get_all_ce_engine_reset_mask(g);

	g->ops.mc.reset(g, ce_reset_mask);

	nvgpu_cg_slcg_ce2_load_enable(g);

	nvgpu_cg_blcg_ce_load_enable(g);

	if (ce_app->initialised) {
		/* assume this happen during poweron/poweroff GPU sequence */
		ce_app->app_state = NVGPU_CE_ACTIVE;
		return 0;
	}

	nvgpu_log(g, gpu_dbg_fn, "ce: init");

	err = nvgpu_mutex_init(&ce_app->app_mutex);
	if (err) {
		return err;
	}

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_init_list_node(&ce_app->allocated_contexts);
	ce_app->ctx_count = 0;
	ce_app->next_ctx_id = 0;
	ce_app->initialised = true;
	ce_app->app_state = NVGPU_CE_ACTIVE;

	nvgpu_mutex_release(&ce_app->app_mutex);

	if (g->ops.ce2.init_prod_values != NULL) {
		g->ops.ce2.init_prod_values(g);
	}

	nvgpu_log(g, gpu_dbg_cde_ctx, "ce: init finished");

	return 0;
}

void gk20a_ce_destroy(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;

	if (!ce_app->initialised) {
		return;
	}

	ce_app->app_state = NVGPU_CE_SUSPEND;
	ce_app->initialised = false;

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		gk20a_ce_delete_gpu_context(ce_ctx);
	}

	nvgpu_init_list_node(&ce_app->allocated_contexts);
	ce_app->ctx_count = 0;
	ce_app->next_ctx_id = 0;

	nvgpu_mutex_release(&ce_app->app_mutex);

	nvgpu_mutex_destroy(&ce_app->app_mutex);
}

void gk20a_ce_suspend(struct gk20a *g)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;

	if (!ce_app->initialised) {
		return;
	}

	ce_app->app_state = NVGPU_CE_SUSPEND;

	return;
}

/* CE app utility functions */
u32 gk20a_ce_create_context(struct gk20a *g,
		int runlist_id,
		int timeslice,
		int runlist_level)
{
	struct gk20a_gpu_ctx *ce_ctx;
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct nvgpu_setup_bind_args setup_bind_args;
	u32 ctx_id = ~0;
	int err = 0;

	if (!ce_app->initialised || ce_app->app_state != NVGPU_CE_ACTIVE) {
		return ctx_id;
	}

	ce_ctx = nvgpu_kzalloc(g, sizeof(*ce_ctx));
	if (!ce_ctx) {
		return ctx_id;
	}

	err = nvgpu_mutex_init(&ce_ctx->gpu_ctx_mutex);
	if (err) {
		nvgpu_kfree(g, ce_ctx);
		return ctx_id;
	}

	ce_ctx->g = g;

	ce_ctx->cmd_buf_read_queue_offset = 0;

	ce_ctx->vm = g->mm.ce.vm;

	/* allocate a tsg if needed */
	ce_ctx->tsg = gk20a_tsg_open(g, nvgpu_current_pid(g));
	if (!ce_ctx->tsg) {
		nvgpu_err(g, "ce: gk20a tsg not available");
		err = -ENOMEM;
		goto end;
	}

	/* always kernel client needs privileged channel */
	ce_ctx->ch = gk20a_open_new_channel(g, runlist_id, true,
				nvgpu_current_pid(g), nvgpu_current_tid(g));
	if (!ce_ctx->ch) {
		nvgpu_err(g, "ce: gk20a channel not available");
		err = -ENOMEM;
		goto end;
	}
	ce_ctx->ch->timeout.enabled = false;

	/* bind the channel to the vm */
	err = g->ops.mm.vm_bind_channel(g->mm.ce.vm, ce_ctx->ch);
	if (err) {
		nvgpu_err(g, "ce: could not bind vm");
		goto end;
	}

	err = gk20a_tsg_bind_channel(ce_ctx->tsg, ce_ctx->ch);
	if (err) {
		nvgpu_err(g, "ce: unable to bind to tsg");
		goto end;
	}

	setup_bind_args.num_gpfifo_entries = 1024;
	setup_bind_args.num_inflight_jobs = 0;
	setup_bind_args.flags = 0;
	/* allocate gpfifo (1024 should be more than enough) */
	err = nvgpu_channel_setup_bind(ce_ctx->ch, &setup_bind_args);
	if (err) {
		nvgpu_err(g, "ce: unable to setup and bind channel");
		goto end;
	}

	/* allocate command buffer from sysmem */
	err = nvgpu_dma_alloc_map_sys(ce_ctx->vm,
			NVGPU_CE_MAX_INFLIGHT_JOBS *
			NVGPU_CE_MAX_COMMAND_BUFF_BYTES_PER_KICKOFF,
			&ce_ctx->cmd_buf_mem);
	 if (err) {
		nvgpu_err(g,
			"ce: could not allocate command buffer for CE context");
		goto end;
	}

	memset(ce_ctx->cmd_buf_mem.cpu_va, 0x00, ce_ctx->cmd_buf_mem.size);

	/* -1 means default channel timeslice value */
	if (timeslice != -1) {
		err = gk20a_fifo_tsg_set_timeslice(ce_ctx->tsg, timeslice);
		if (err) {
			nvgpu_err(g,
				"ce: could not set the channel timeslice value for CE context");
			goto end;
		}
	}

	/* -1 means default channel runlist level */
	if (runlist_level != -1) {
		err = gk20a_tsg_set_runlist_interleave(ce_ctx->tsg,
						       runlist_level);
		if (err) {
			nvgpu_err(g,
				"ce: could not set the runlist interleave for CE context");
			goto end;
		}
	}

	nvgpu_mutex_acquire(&ce_app->app_mutex);
	ctx_id = ce_ctx->ctx_id = ce_app->next_ctx_id;
	nvgpu_list_add(&ce_ctx->list, &ce_app->allocated_contexts);
	++ce_app->next_ctx_id;
	++ce_app->ctx_count;
	nvgpu_mutex_release(&ce_app->app_mutex);

	ce_ctx->gpu_ctx_state = NVGPU_CE_GPU_CTX_ALLOCATED;

end:
	if (ctx_id == (u32)~0) {
		nvgpu_mutex_acquire(&ce_app->app_mutex);
		gk20a_ce_delete_gpu_context(ce_ctx);
		nvgpu_mutex_release(&ce_app->app_mutex);
	}
	return ctx_id;

}

void gk20a_ce_delete_context(struct gk20a *g,
		u32 ce_ctx_id)
{
	gk20a_ce_delete_context_priv(g, ce_ctx_id);
}

void gk20a_ce_delete_context_priv(struct gk20a *g,
		u32 ce_ctx_id)
{
	struct gk20a_ce_app *ce_app = &g->ce_app;
	struct gk20a_gpu_ctx *ce_ctx, *ce_ctx_save;

	if (!ce_app->initialised || ce_app->app_state != NVGPU_CE_ACTIVE) {
		return;
	}

	nvgpu_mutex_acquire(&ce_app->app_mutex);

	nvgpu_list_for_each_entry_safe(ce_ctx, ce_ctx_save,
			&ce_app->allocated_contexts, gk20a_gpu_ctx, list) {
		if (ce_ctx->ctx_id == ce_ctx_id) {
			gk20a_ce_delete_gpu_context(ce_ctx);
			--ce_app->ctx_count;
			break;
		}
	}

	nvgpu_mutex_release(&ce_app->app_mutex);
	return;
}
