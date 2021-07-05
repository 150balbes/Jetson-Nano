/*
 * Virtualized GPU Graphics
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/bug.h>
#include <nvgpu/dma.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/dma.h>
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/channel.h>
#include <nvgpu/tsg.h>

#include "gr_vgpu.h"
#include "gk20a/gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"

#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_ctxsw_prog_gk20a.h>

void vgpu_gr_detect_sm_arch(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	g->params.sm_arch_sm_version =
			priv->constants.sm_arch_sm_version;
	g->params.sm_arch_spa_version =
			priv->constants.sm_arch_spa_version;
	g->params.sm_arch_warp_count =
			priv->constants.sm_arch_warp_count;
}

int vgpu_gr_commit_inst(struct channel_gk20a *c, u64 gpu_va)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;
	struct gk20a *g = c->g;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_CTX;
	msg.handle = vgpu_get_handle(c->g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

static int vgpu_gr_commit_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c, bool patch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_GLOBAL_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

/* load saved fresh copy of gloden image into channel gr_ctx */
static int vgpu_gr_load_golden_ctx_image(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_LOAD_GR_GOLDEN_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -1 : 0;
}

int vgpu_gr_init_ctx_state(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	g->gr.ctx_vars.golden_image_size = priv->constants.golden_ctx_size;
	g->gr.ctx_vars.zcull_ctxsw_image_size = priv->constants.zcull_ctx_size;
	g->gr.ctx_vars.pm_ctxsw_image_size = priv->constants.hwpm_ctx_size;
	if (!g->gr.ctx_vars.golden_image_size ||
		!g->gr.ctx_vars.zcull_ctxsw_image_size ||
		!g->gr.ctx_vars.pm_ctxsw_image_size)
		return -ENXIO;

	gr->ctx_vars.buffer_size = g->gr.ctx_vars.golden_image_size;
	g->gr.ctx_vars.priv_access_map_size = 512 * 1024;
#ifdef CONFIG_GK20A_CTXSW_TRACE
	g->gr.ctx_vars.fecs_trace_buffer_size = gk20a_fecs_trace_buffer_size(g);
#endif
	return 0;
}

static int vgpu_gr_alloc_global_ctx_buffers(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int attr_buffer_size;

	u32 cb_buffer_size = gr->bundle_cb_default_size *
		gr_scc_bundle_cb_size_div_256b_byte_granularity_v();

	u32 pagepool_buffer_size = g->ops.gr.pagepool_default_size(g) *
		gr_scc_pagepool_total_pages_byte_granularity_v();

	nvgpu_log_fn(g, " ");

	attr_buffer_size = g->ops.gr.calc_global_ctx_buffer_size(g);

	nvgpu_log_info(g, "cb_buffer_size : %d", cb_buffer_size);
	gr->global_ctx_buffer[CIRCULAR].mem.size = cb_buffer_size;

	nvgpu_log_info(g, "pagepool_buffer_size : %d", pagepool_buffer_size);
	gr->global_ctx_buffer[PAGEPOOL].mem.size = pagepool_buffer_size;

	nvgpu_log_info(g, "attr_buffer_size : %d", attr_buffer_size);
	gr->global_ctx_buffer[ATTRIBUTE].mem.size = attr_buffer_size;

	nvgpu_log_info(g, "priv access map size : %d",
		gr->ctx_vars.priv_access_map_size);
	gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size =
		gr->ctx_vars.priv_access_map_size;
#ifdef CONFIG_GK20A_CTXSW_TRACE
	nvgpu_log_info(g, "fecs_trace_buffer_size : %d",
		gr->ctx_vars.fecs_trace_buffer_size);
	gr->global_ctx_buffer[FECS_TRACE_BUFFER].mem.size =
		gr->ctx_vars.fecs_trace_buffer_size;
#endif
	return 0;
}

static int vgpu_gr_map_global_ctx_buffers(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	struct tsg_gk20a *tsg;
	u64 *g_bfr_va;
	u64 *g_bfr_size;
	struct gr_gk20a *gr = &g->gr;
	u64 gpu_va;
	u32 i;
	int err;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg)
		return -EINVAL;

	g_bfr_va = tsg->gr_ctx.global_ctx_buffer_va;
	g_bfr_size = tsg->gr_ctx.global_ctx_buffer_size;

	/* Circular Buffer */
	gpu_va = __nvgpu_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[CIRCULAR].mem.size,
			GMMU_PAGE_SIZE_KERNEL);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[CIRCULAR_VA] = gpu_va;
	g_bfr_size[CIRCULAR_VA] = gr->global_ctx_buffer[CIRCULAR].mem.size;

	/* Attribute Buffer */
	gpu_va = __nvgpu_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[ATTRIBUTE].mem.size,
			GMMU_PAGE_SIZE_KERNEL);

	if (!gpu_va)
		goto clean_up;
	g_bfr_va[ATTRIBUTE_VA] = gpu_va;
	g_bfr_size[ATTRIBUTE_VA] = gr->global_ctx_buffer[ATTRIBUTE].mem.size;

	/* Page Pool */
	gpu_va = __nvgpu_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PAGEPOOL].mem.size,
			GMMU_PAGE_SIZE_KERNEL);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PAGEPOOL_VA] = gpu_va;
	g_bfr_size[PAGEPOOL_VA] = gr->global_ctx_buffer[PAGEPOOL].mem.size;

	/* Priv register Access Map */
	gpu_va = __nvgpu_vm_alloc_va(ch_vm,
			gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size,
			GMMU_PAGE_SIZE_KERNEL);
	if (!gpu_va)
		goto clean_up;
	g_bfr_va[PRIV_ACCESS_MAP_VA] = gpu_va;
	g_bfr_size[PRIV_ACCESS_MAP_VA] =
		gr->global_ctx_buffer[PRIV_ACCESS_MAP].mem.size;

	/* FECS trace Buffer */
#ifdef CONFIG_GK20A_CTXSW_TRACE
	gpu_va = __nvgpu_vm_alloc_va(ch_vm,
		gr->global_ctx_buffer[FECS_TRACE_BUFFER].mem.size,
		GMMU_PAGE_SIZE_KERNEL);

	if (!gpu_va)
		goto clean_up;

	g_bfr_va[FECS_TRACE_BUFFER_VA] = gpu_va;
	g_bfr_size[FECS_TRACE_BUFFER_VA] =
		gr->global_ctx_buffer[FECS_TRACE_BUFFER].mem.size;
#endif
	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_MAP_GR_GLOBAL_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->cb_va = g_bfr_va[CIRCULAR_VA];
	p->attr_va = g_bfr_va[ATTRIBUTE_VA];
	p->page_pool_va = g_bfr_va[PAGEPOOL_VA];
	p->priv_access_map_va = g_bfr_va[PRIV_ACCESS_MAP_VA];
#ifdef CONFIG_GK20A_CTXSW_TRACE
	p->fecs_trace_va = g_bfr_va[FECS_TRACE_BUFFER_VA];
#endif
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		goto clean_up;

	tsg->gr_ctx.global_ctx_buffer_mapped = true;
	return 0;

 clean_up:
	for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
		if (g_bfr_va[i]) {
			__nvgpu_vm_free_va(ch_vm, g_bfr_va[i],
					   GMMU_PAGE_SIZE_KERNEL);
			g_bfr_va[i] = 0;
		}
	}
	return -ENOMEM;
}

static void vgpu_gr_unmap_global_ctx_buffers(struct tsg_gk20a *tsg)
{
	struct vm_gk20a *ch_vm = tsg->vm;
	u64 *g_bfr_va = tsg->gr_ctx.global_ctx_buffer_va;
	u64 *g_bfr_size = tsg->gr_ctx.global_ctx_buffer_size;
	u32 i;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	if (tsg->gr_ctx.global_ctx_buffer_mapped) {
		/* server will unmap on channel close */

		for (i = 0; i < NR_GLOBAL_CTX_BUF_VA; i++) {
			if (g_bfr_va[i]) {
				__nvgpu_vm_free_va(ch_vm, g_bfr_va[i],
						   GMMU_PAGE_SIZE_KERNEL);
				g_bfr_va[i] = 0;
				g_bfr_size[i] = 0;
			}
		}

		tsg->gr_ctx.global_ctx_buffer_mapped = false;
	}
}

int vgpu_gr_alloc_gr_ctx(struct gk20a *g,
			struct nvgpu_gr_ctx *gr_ctx,
			struct vm_gk20a *vm,
			u32 class,
			u32 flags)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
	struct gr_gk20a *gr = &g->gr;
	int err;

	nvgpu_log_fn(g, " ");

	if (gr->ctx_vars.buffer_size == 0)
		return 0;

	/* alloc channel gr ctx buffer */
	gr->ctx_vars.buffer_size = gr->ctx_vars.golden_image_size;
	gr->ctx_vars.buffer_total_size = gr->ctx_vars.golden_image_size;

	gr_ctx->mem.gpu_va = __nvgpu_vm_alloc_va(vm,
						gr->ctx_vars.buffer_total_size,
						GMMU_PAGE_SIZE_KERNEL);

	if (!gr_ctx->mem.gpu_va)
		return -ENOMEM;
	gr_ctx->mem.size = gr->ctx_vars.buffer_total_size;
	gr_ctx->mem.aperture = APERTURE_SYSMEM;

	msg.cmd = TEGRA_VGPU_CMD_GR_CTX_ALLOC;
	msg.handle = vgpu_get_handle(g);
	p->as_handle = vm->handle;
	p->gr_ctx_va = gr_ctx->mem.gpu_va;
	p->class_num = class;
	p->tsg_id = gr_ctx->tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;

	if (unlikely(err)) {
		nvgpu_err(g, "fail to alloc gr_ctx");
		__nvgpu_vm_free_va(vm, gr_ctx->mem.gpu_va,
				   GMMU_PAGE_SIZE_KERNEL);
		gr_ctx->mem.aperture = APERTURE_INVALID;
	} else {
		gr_ctx->virt_ctx = p->gr_ctx_handle;
	}

	return err;
}

static int vgpu_gr_alloc_channel_patch_ctx(struct gk20a *g,
					struct channel_gk20a *c)
{
	struct tsg_gk20a *tsg;
	struct patch_desc *patch_ctx;
	struct vm_gk20a *ch_vm = c->vm;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_ch_ctx_params *p = &msg.params.ch_ctx;
	int err;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg)
		return -EINVAL;

	patch_ctx = &tsg->gr_ctx.patch_ctx;
	patch_ctx->mem.size = 128 * sizeof(u32);
	patch_ctx->mem.gpu_va = __nvgpu_vm_alloc_va(ch_vm,
						patch_ctx->mem.size,
						GMMU_PAGE_SIZE_KERNEL);
	if (!patch_ctx->mem.gpu_va)
		return -ENOMEM;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_PATCH_CTX;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->patch_ctx_va = patch_ctx->mem.gpu_va;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		__nvgpu_vm_free_va(ch_vm, patch_ctx->mem.gpu_va,
				   GMMU_PAGE_SIZE_KERNEL);
		err = -ENOMEM;
	}

	return err;
}

static void vgpu_gr_free_channel_patch_ctx(struct tsg_gk20a *tsg)
{
	struct patch_desc *patch_ctx = &tsg->gr_ctx.patch_ctx;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	if (patch_ctx->mem.gpu_va) {
		/* server will free on channel close */

		__nvgpu_vm_free_va(tsg->vm, patch_ctx->mem.gpu_va,
				   GMMU_PAGE_SIZE_KERNEL);
		patch_ctx->mem.gpu_va = 0;
	}
}

static void vgpu_gr_free_channel_pm_ctx(struct tsg_gk20a *tsg)
{
	struct nvgpu_gr_ctx *ch_ctx = &tsg->gr_ctx;
	struct pm_ctx_desc *pm_ctx = &ch_ctx->pm_ctx;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	/* check if hwpm was ever initialized. If not, nothing to do */
	if (pm_ctx->mem.gpu_va == 0)
		return;

	/* server will free on channel close */

	__nvgpu_vm_free_va(tsg->vm, pm_ctx->mem.gpu_va,
			   GMMU_PAGE_SIZE_KERNEL);
	pm_ctx->mem.gpu_va = 0;
}

void vgpu_gr_free_gr_ctx(struct gk20a *g,
			 struct vm_gk20a *vm, struct nvgpu_gr_ctx *gr_ctx)
{
	struct tsg_gk20a *tsg;

	nvgpu_log_fn(g, " ");

	if (gr_ctx->mem.gpu_va) {
		struct tegra_vgpu_cmd_msg msg;
		struct tegra_vgpu_gr_ctx_params *p = &msg.params.gr_ctx;
		int err;

		msg.cmd = TEGRA_VGPU_CMD_GR_CTX_FREE;
		msg.handle = vgpu_get_handle(g);
		p->gr_ctx_handle = gr_ctx->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		WARN_ON(err || msg.ret);

		__nvgpu_vm_free_va(vm, gr_ctx->mem.gpu_va,
				   GMMU_PAGE_SIZE_KERNEL);

		tsg = &g->fifo.tsg[gr_ctx->tsgid];
		vgpu_gr_unmap_global_ctx_buffers(tsg);
		vgpu_gr_free_channel_patch_ctx(tsg);
		vgpu_gr_free_channel_pm_ctx(tsg);

		nvgpu_dma_unmap_free(vm, &gr_ctx->pagepool_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->betacb_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->spill_ctxsw_buffer);
		nvgpu_dma_unmap_free(vm, &gr_ctx->preempt_ctxsw_buffer);

		memset(gr_ctx, 0, sizeof(*gr_ctx));
	}
}

static int vgpu_gr_ch_bind_gr_ctx(struct channel_gk20a *c)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_channel_bind_gr_ctx_params *p =
				&msg.params.ch_bind_gr_ctx;
	int err;

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg)
		return -EINVAL;

	gr_ctx = &tsg->gr_ctx;

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTX;
	msg.handle = vgpu_get_handle(c->g);
	p->ch_handle = c->virt_ctx;
	p->gr_ctx_handle = gr_ctx->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

static int vgpu_gr_tsg_bind_gr_ctx(struct tsg_gk20a *tsg)
{
	struct nvgpu_gr_ctx *gr_ctx = &tsg->gr_ctx;
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_tsg_bind_gr_ctx_params *p =
					&msg.params.tsg_bind_gr_ctx;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_BIND_GR_CTX;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->gr_ctx_handle = gr_ctx->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

int vgpu_gr_alloc_obj_ctx(struct channel_gk20a  *c, u32 class_num, u32 flags)
{
	struct gk20a *g = c->g;
	struct nvgpu_gr_ctx *gr_ctx = NULL;
	struct tsg_gk20a *tsg = NULL;
	int err = 0;

	nvgpu_log_fn(g, " ");

	/* an address space needs to have been bound at this point.*/
	if (!gk20a_channel_as_bound(c)) {
		nvgpu_err(g, "not bound to address space at time"
			   " of grctx allocation");
		return -EINVAL;
	}

	if (!g->ops.gr.is_valid_class(g, class_num)) {
		nvgpu_err(g, "invalid obj class 0x%x", class_num);
		err = -EINVAL;
		goto out;
	}
	c->obj_class = class_num;

	tsg = tsg_gk20a_from_ch(c);
	if (tsg == NULL) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;

	if (!nvgpu_mem_is_valid(&gr_ctx->mem)) {
		tsg->vm = c->vm;
		nvgpu_vm_get(tsg->vm);
		gr_ctx->tsgid = tsg->tsgid;
		err = g->ops.gr.alloc_gr_ctx(g, gr_ctx,
					c->vm,
					class_num,
					flags);
		if (!err) {
			gr_ctx->tsgid = tsg->tsgid;
			err = vgpu_gr_tsg_bind_gr_ctx(tsg);
		}
		if (err) {
			nvgpu_err(g,
				"fail to allocate TSG gr ctx buffer, err=%d", err);
			nvgpu_vm_put(tsg->vm);
			tsg->vm = NULL;
			goto out;
		}

		err = vgpu_gr_ch_bind_gr_ctx(c);
		if (err) {
			nvgpu_err(g, "fail to bind gr ctx buffer");
			goto out;
		}

		/* allocate patch buffer */
		err = vgpu_gr_alloc_channel_patch_ctx(g, c);
		if (err) {
			nvgpu_err(g, "fail to allocate patch buffer");
			goto out;
		}

		/* map global buffer to channel gpu_va and commit */
		err = vgpu_gr_map_global_ctx_buffers(g, c);
		if (err) {
			nvgpu_err(g, "fail to map global ctx buffer");
			goto out;
		}

		err = vgpu_gr_commit_global_ctx_buffers(g, c, true);
		if (err) {
			nvgpu_err(g, "fail to commit global ctx buffers");
			goto out;
		}

		/* commit gr ctx buffer */
		err = g->ops.gr.commit_inst(c, gr_ctx->mem.gpu_va);
		if (err) {
			nvgpu_err(g, "fail to commit gr ctx buffer");
			goto out;
		}

		/* load golden image */
		err = gr_gk20a_elpg_protected_call(g,
				vgpu_gr_load_golden_ctx_image(g, c));
		if (err) {
			nvgpu_err(g, "fail to load golden ctx image");
			goto out;
		}
	} else {
		err = vgpu_gr_ch_bind_gr_ctx(c);
		if (err) {
			nvgpu_err(g, "fail to bind gr ctx buffer");
			goto out;
		}

		/* commit gr ctx buffer */
		err = g->ops.gr.commit_inst(c, gr_ctx->mem.gpu_va);
		if (err) {
			nvgpu_err(g, "fail to commit gr ctx buffer");
			goto out;
		}
#ifdef CONFIG_GK20A_CTXSW_TRACE
		/* for fecs bind channel */
		err = gr_gk20a_elpg_protected_call(g,
				vgpu_gr_load_golden_ctx_image(g, c));
		if (err) {
			nvgpu_err(g, "fail to load golden ctx image");
			goto out;
		}
#endif
	}

	/* PM ctxt switch is off by default */
	gr_ctx->pm_ctx.pm_mode = ctxsw_prog_main_image_pm_mode_no_ctxsw_f();

	nvgpu_log_fn(g, "done");
	return 0;
out:
	/* 1. gr_ctx, patch_ctx and global ctx buffer mapping
	   can be reused so no need to release them.
	   2. golden image load is a one time thing so if
	   they pass, no need to undo. */
	nvgpu_err(g, "fail");
	return err;
}

static int vgpu_gr_init_gr_config(struct gk20a *g, struct gr_gk20a *gr)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 gpc_index;
	u32 sm_per_tpc;
	int err = -ENOMEM;

	nvgpu_log_fn(g, " ");

	gr->max_gpc_count = priv->constants.max_gpc_count;
	gr->gpc_count = priv->constants.gpc_count;
	gr->max_tpc_per_gpc_count = priv->constants.max_tpc_per_gpc_count;

	gr->max_tpc_count = gr->max_gpc_count * gr->max_tpc_per_gpc_count;

	gr->gpc_tpc_count = nvgpu_kzalloc(g, gr->gpc_count * sizeof(u32));
	if (!gr->gpc_tpc_count)
		goto cleanup;

	gr->gpc_tpc_mask = nvgpu_kzalloc(g, gr->gpc_count * sizeof(u32));
	if (!gr->gpc_tpc_mask)
		goto cleanup;

	sm_per_tpc = priv->constants.sm_per_tpc;
	gr->sm_to_cluster = nvgpu_kzalloc(g, gr->gpc_count *
					  gr->max_tpc_per_gpc_count *
					  sm_per_tpc *
					  sizeof(struct sm_info));
	if (!gr->sm_to_cluster)
		goto cleanup;

	gr->tpc_count = 0;
	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		gr->gpc_tpc_count[gpc_index] =
			priv->constants.gpc_tpc_count[gpc_index];

		gr->tpc_count += gr->gpc_tpc_count[gpc_index];

		if (g->ops.gr.get_gpc_tpc_mask)
			gr->gpc_tpc_mask[gpc_index] =
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index);
	}

	g->ops.gr.bundle_cb_defaults(g);
	g->ops.gr.cb_size_default(g);
	g->ops.gr.calc_global_ctx_buffer_size(g);
	err = g->ops.gr.init_fs_state(g);
	if (err)
		goto cleanup;
	return 0;
cleanup:
	nvgpu_err(g, "out of memory");

	nvgpu_kfree(g, gr->gpc_tpc_count);
	gr->gpc_tpc_count = NULL;

	nvgpu_kfree(g, gr->gpc_tpc_mask);
	gr->gpc_tpc_mask = NULL;

	return err;
}

int vgpu_gr_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
				struct channel_gk20a *c, u64 zcull_va,
				u32 mode)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_bind_params *p = &msg.params.zcull_bind;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_BIND_ZCULL;
	msg.handle = vgpu_get_handle(g);
	p->handle = c->virt_ctx;
	p->zcull_va = zcull_va;
	p->mode = mode;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

int vgpu_gr_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
				struct gr_zcull_info *zcull_params)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_zcull_info_params *p = &msg.params.zcull_info;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_GET_ZCULL_INFO;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	zcull_params->width_align_pixels = p->width_align_pixels;
	zcull_params->height_align_pixels = p->height_align_pixels;
	zcull_params->pixel_squares_by_aliquots = p->pixel_squares_by_aliquots;
	zcull_params->aliquot_total = p->aliquot_total;
	zcull_params->region_byte_multiplier = p->region_byte_multiplier;
	zcull_params->region_header_size = p->region_header_size;
	zcull_params->subregion_header_size = p->subregion_header_size;
	zcull_params->subregion_width_align_pixels =
		p->subregion_width_align_pixels;
	zcull_params->subregion_height_align_pixels =
		p->subregion_height_align_pixels;
	zcull_params->subregion_count = p->subregion_count;

	return 0;
}

u32 vgpu_gr_get_gpc_tpc_mask(struct gk20a *g, u32 gpc_index)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	return priv->constants.gpc_tpc_mask[gpc_index];
}

u32 vgpu_gr_get_max_fbps_count(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	return priv->constants.num_fbps;
}

u32 vgpu_gr_get_fbp_en_mask(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	return priv->constants.fbp_en_mask;
}

u32 vgpu_gr_get_max_ltc_per_fbp(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	return priv->constants.ltc_per_fbp;
}

u32 vgpu_gr_get_max_lts_per_ltc(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_log_fn(g, " ");

	return priv->constants.max_lts_per_ltc;
}

u32 *vgpu_gr_rop_l2_en_mask(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	u32 i, max_fbps_count = priv->constants.num_fbps;

	nvgpu_log_fn(g, " ");

	if (g->gr.fbp_rop_l2_en_mask == NULL) {
		g->gr.fbp_rop_l2_en_mask =
			nvgpu_kzalloc(g, max_fbps_count * sizeof(u32));
		if (!g->gr.fbp_rop_l2_en_mask)
			return NULL;
	}

	g->gr.max_fbps_count = max_fbps_count;
	for (i = 0; i < max_fbps_count; i++)
		g->gr.fbp_rop_l2_en_mask[i] = priv->constants.l2_en_mask[i];

	return g->gr.fbp_rop_l2_en_mask;
}

int vgpu_gr_add_zbc(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *zbc_val)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_zbc_set_table_params *p = &msg.params.zbc_set_table;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_ZBC_SET_TABLE;
	msg.handle = vgpu_get_handle(g);

	p->type = zbc_val->type;
	p->format = zbc_val->format;
	switch (p->type) {
	case GK20A_ZBC_TYPE_COLOR:
		memcpy(p->color_ds, zbc_val->color_ds, sizeof(p->color_ds));
		memcpy(p->color_l2, zbc_val->color_l2, sizeof(p->color_l2));
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		p->depth = zbc_val->depth;
		break;
	default:
		return -EINVAL;
	}

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? -ENOMEM : 0;
}

int vgpu_gr_query_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_zbc_query_table_params *p =
					&msg.params.zbc_query_table;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_ZBC_QUERY_TABLE;
	msg.handle = vgpu_get_handle(g);

	p->type = query_params->type;
	p->index_size = query_params->index_size;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	switch (query_params->type) {
	case GK20A_ZBC_TYPE_COLOR:
		memcpy(query_params->color_ds, p->color_ds,
				sizeof(query_params->color_ds));
		memcpy(query_params->color_l2, p->color_l2,
				sizeof(query_params->color_l2));
		break;
	case GK20A_ZBC_TYPE_DEPTH:
		query_params->depth = p->depth;
		break;
	case GK20A_ZBC_TYPE_INVALID:
		query_params->index_size = p->index_size;
		break;
	default:
		return -EINVAL;
	}
	query_params->ref_cnt = p->ref_cnt;
	query_params->format = p->format;

	return 0;
}

static void vgpu_remove_gr_support(struct gr_gk20a *gr)
{
	nvgpu_log_fn(gr->g, " ");

	gk20a_comptag_allocator_destroy(gr->g, &gr->comp_tags);

	nvgpu_kfree(gr->g, gr->gpc_tpc_mask);
	gr->gpc_tpc_mask = NULL;

	nvgpu_kfree(gr->g, gr->sm_to_cluster);
	gr->sm_to_cluster = NULL;

	nvgpu_kfree(gr->g, gr->gpc_tpc_count);
	gr->gpc_tpc_count = NULL;

	nvgpu_kfree(gr->g, gr->fbp_rop_l2_en_mask);
	gr->fbp_rop_l2_en_mask = NULL;
}

static int vgpu_gr_init_gr_setup_sw(struct gk20a *g)
{
	struct gr_gk20a *gr = &g->gr;
	int err;

	nvgpu_log_fn(g, " ");

	if (gr->sw_ready) {
		nvgpu_log_fn(g, "skip init");
		return 0;
	}

	gr->g = g;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	nvgpu_mutex_init(&g->gr.cs_lock);
#endif

	err = vgpu_gr_init_gr_config(g, gr);
	if (err)
		goto clean_up;

	err = g->ops.gr.init_ctx_state(g);
	if (err)
		goto clean_up;

	err = g->ops.ltc.init_comptags(g, gr);
	if (err)
		goto clean_up;

	err = vgpu_gr_alloc_global_ctx_buffers(g);
	if (err)
		goto clean_up;

	nvgpu_mutex_init(&gr->ctx_mutex);
	nvgpu_spinlock_init(&gr->ch_tlb_lock);

	gr->remove_support = vgpu_remove_gr_support;
	gr->sw_ready = true;

	nvgpu_log_fn(g, "done");
	return 0;

clean_up:
	nvgpu_err(g, "fail");
	vgpu_remove_gr_support(gr);
	return err;
}

int vgpu_init_gr_support(struct gk20a *g)
{
	nvgpu_log_fn(g, " ");

	return vgpu_gr_init_gr_setup_sw(g);
}

int vgpu_gr_isr(struct gk20a *g, struct tegra_vgpu_gr_intr_info *info)
{
	struct channel_gk20a *ch = gk20a_channel_from_id(g, info->chid);

	nvgpu_log_fn(g, " ");

	if (!ch)
		return 0;

	if (info->type != TEGRA_VGPU_GR_INTR_NOTIFY &&
		info->type != TEGRA_VGPU_GR_INTR_SEMAPHORE)
		nvgpu_err(g, "gr intr (%d) on ch %u", info->type, info->chid);

	switch (info->type) {
	case TEGRA_VGPU_GR_INTR_NOTIFY:
		nvgpu_cond_broadcast_interruptible(&ch->notifier_wq);
		break;
	case TEGRA_VGPU_GR_INTR_SEMAPHORE:
		nvgpu_cond_broadcast_interruptible(&ch->semaphore_wq);
		break;
	case TEGRA_VGPU_GR_INTR_SEMAPHORE_TIMEOUT:
		g->ops.fifo.set_error_notifier(ch,
				NVGPU_ERR_NOTIFIER_GR_SEMAPHORE_TIMEOUT);
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_NOTIFY:
		g->ops.fifo.set_error_notifier(ch,
					NVGPU_ERR_NOTIFIER_GR_ILLEGAL_NOTIFY);
	case TEGRA_VGPU_GR_INTR_ILLEGAL_METHOD:
		break;
	case TEGRA_VGPU_GR_INTR_ILLEGAL_CLASS:
		g->ops.fifo.set_error_notifier(ch,
					NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FECS_ERROR:
		break;
	case TEGRA_VGPU_GR_INTR_CLASS_ERROR:
		g->ops.fifo.set_error_notifier(ch,
					NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_FIRMWARE_METHOD:
		g->ops.fifo.set_error_notifier(ch,
				NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_EXCEPTION:
		g->ops.fifo.set_error_notifier(ch,
				NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY);
		break;
	case TEGRA_VGPU_GR_INTR_SM_EXCEPTION:
		g->ops.debugger.post_events(ch);
		break;
	default:
		WARN_ON(1);
		break;
	}

	gk20a_channel_put(ch);
	return 0;
}

int vgpu_gr_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_sm_debug_mode *p = &msg.params.sm_debug_mode;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_SET_SM_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->sms = sms;
	p->enable = (u32)enable;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

int vgpu_gr_update_smpc_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_set_ctxsw_mode *p = &msg.params.set_ctxsw_mode;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_SMPC_CTXSW_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;

	if (enable)
		p->mode = TEGRA_VGPU_CTXSW_MODE_CTXSW;
	else
		p->mode = TEGRA_VGPU_CTXSW_MODE_NO_CTXSW;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

int vgpu_gr_update_hwpm_ctxsw_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 gpu_va, u32 mode)
{
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *ch_ctx;
	struct pm_ctx_desc *pm_ctx;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_set_ctxsw_mode *p = &msg.params.set_ctxsw_mode;
	int err;

	nvgpu_log_fn(g, " ");

	tsg = tsg_gk20a_from_ch(ch);
	if (!tsg)
		return -EINVAL;

	if (gpu_va) {
		nvgpu_err(g, "gpu_va suppose to be allocated by this function.");
		return -EINVAL;
	}

	ch_ctx = &tsg->gr_ctx;
	pm_ctx = &ch_ctx->pm_ctx;

	if (mode == NVGPU_DBG_HWPM_CTXSW_MODE_CTXSW) {
		/*
		 * send command to enable HWPM only once - otherwise server
		 * will return an error due to using the same GPU VA twice.
		 */

		if (pm_ctx->pm_mode == ctxsw_prog_main_image_pm_mode_ctxsw_f()) {
			return 0;
		}
		p->mode = TEGRA_VGPU_CTXSW_MODE_CTXSW;
	} else if (mode == NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW) {
		if (pm_ctx->pm_mode == ctxsw_prog_main_image_pm_mode_no_ctxsw_f()) {
			return 0;
		}
		p->mode = TEGRA_VGPU_CTXSW_MODE_NO_CTXSW;
	} else if ((mode == NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW) &&
			(g->ops.gr.get_hw_accessor_stream_out_mode)){
		if (pm_ctx->pm_mode == g->ops.gr.get_hw_accessor_stream_out_mode()) {
			return 0;
		}
		p->mode = TEGRA_VGPU_CTXSW_MODE_STREAM_OUT_CTXSW;
	} else {
		nvgpu_err(g, "invalid hwpm context switch mode");
		return -EINVAL;
	}

	if (mode != NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW) {
		/* Allocate buffer if necessary */
		if (pm_ctx->mem.gpu_va == 0) {
			pm_ctx->mem.gpu_va = __nvgpu_vm_alloc_va(ch->vm,
					g->gr.ctx_vars.pm_ctxsw_image_size,
					GMMU_PAGE_SIZE_KERNEL);

			if (!pm_ctx->mem.gpu_va)
				return -ENOMEM;
			pm_ctx->mem.size = g->gr.ctx_vars.pm_ctxsw_image_size;
		}
	}

	msg.cmd = TEGRA_VGPU_CMD_CHANNEL_SET_HWPM_CTXSW_MODE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->gpu_va = pm_ctx->mem.gpu_va;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
	err = err ? err : msg.ret;
	if (!err) {
		if (mode == NVGPU_DBG_HWPM_CTXSW_MODE_CTXSW) {
			pm_ctx->pm_mode = ctxsw_prog_main_image_pm_mode_ctxsw_f();
		} else if (mode == NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW) {
			pm_ctx->pm_mode = ctxsw_prog_main_image_pm_mode_no_ctxsw_f();
		} else {
			pm_ctx->pm_mode = g->ops.gr.get_hw_accessor_stream_out_mode();
		}
	}

	return err;
}

int vgpu_gr_clear_sm_error_state(struct gk20a *g,
		struct channel_gk20a *ch, u32 sm_id)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_clear_sm_error_state *p =
			&msg.params.clear_sm_error_state;
	struct tsg_gk20a *tsg;
	int err;

	tsg = tsg_gk20a_from_ch(ch);
	if (!tsg) {
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	msg.cmd = TEGRA_VGPU_CMD_CLEAR_SM_ERROR_STATE;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	p->sm_id = sm_id;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	memset(&tsg->sm_error_states[sm_id], 0, sizeof(*tsg->sm_error_states));
	nvgpu_mutex_release(&g->dbg_sessions_lock);

	return err ? err : msg.ret;


	return 0;
}

static int vgpu_gr_suspend_resume_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd, u32 cmd)
{
	struct dbg_session_channel_data *ch_data;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_suspend_resume_contexts *p;
	size_t n;
	int channel_fd = -1;
	int err = 0;
	void *handle = NULL;
	u16 *oob;
	size_t oob_size;

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);
	nvgpu_mutex_acquire(&dbg_s->ch_list_lock);

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
			TEGRA_VGPU_QUEUE_CMD,
			(void **)&oob, &oob_size);
	if (!handle) {
		err = -EINVAL;
		goto done;
	}

	n = 0;
	nvgpu_list_for_each_entry(ch_data, &dbg_s->ch_list,
			dbg_session_channel_data, ch_entry)
		n++;

	if (oob_size < n * sizeof(u16)) {
		err = -ENOMEM;
		goto done;
	}

	msg.cmd = cmd;
	msg.handle = vgpu_get_handle(g);
	p = &msg.params.suspend_contexts;
	p->num_channels = n;
	n = 0;
	nvgpu_list_for_each_entry(ch_data, &dbg_s->ch_list,
			dbg_session_channel_data, ch_entry)
		oob[n++] = (u16)ch_data->chid;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret) {
		err = -ENOMEM;
		goto done;
	}

	if (p->resident_chid != (u16)~0) {
		nvgpu_list_for_each_entry(ch_data, &dbg_s->ch_list,
				dbg_session_channel_data, ch_entry) {
			if (ch_data->chid == p->resident_chid) {
				channel_fd = ch_data->channel_fd;
				break;
			}
		}
	}

done:
	if (handle)
		vgpu_ivc_oob_put_ptr(handle);
	nvgpu_mutex_release(&dbg_s->ch_list_lock);
	nvgpu_mutex_release(&g->dbg_sessions_lock);
	*ctx_resident_ch_fd = channel_fd;
	return err;
}

int vgpu_gr_suspend_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd)
{
	return vgpu_gr_suspend_resume_contexts(g, dbg_s,
			ctx_resident_ch_fd, TEGRA_VGPU_CMD_SUSPEND_CONTEXTS);
}

int vgpu_gr_resume_contexts(struct gk20a *g,
		struct dbg_session_gk20a *dbg_s,
		int *ctx_resident_ch_fd)
{
	return vgpu_gr_suspend_resume_contexts(g, dbg_s,
			ctx_resident_ch_fd, TEGRA_VGPU_CMD_RESUME_CONTEXTS);
}

void vgpu_gr_handle_sm_esr_event(struct gk20a *g,
			struct tegra_vgpu_sm_esr_info *info)
{
	struct nvgpu_tsg_sm_error_state *sm_error_states;
	struct tsg_gk20a *tsg;

	if (info->sm_id >= g->gr.no_of_sm) {
		nvgpu_err(g, "invalid smd_id %d / %d",
			info->sm_id, g->gr.no_of_sm);
		return;
	}

	if (info->tsg_id >= g->fifo.num_channels) {
		nvgpu_err(g, "invalid tsg_id in sm esr event");
		return;
	}

	tsg = &g->fifo.tsg[info->tsg_id];
	if (tsg == NULL) {
		nvgpu_err(g, "invalid tsg");
		return;
	}

	nvgpu_mutex_acquire(&g->dbg_sessions_lock);

	sm_error_states = &tsg->sm_error_states[info->sm_id];

	sm_error_states->hww_global_esr = info->hww_global_esr;
	sm_error_states->hww_warp_esr = info->hww_warp_esr;
	sm_error_states->hww_warp_esr_pc = info->hww_warp_esr_pc;
	sm_error_states->hww_global_esr_report_mask =
				info->hww_global_esr_report_mask;
	sm_error_states->hww_warp_esr_report_mask =
				info->hww_warp_esr_report_mask;

	nvgpu_mutex_release(&g->dbg_sessions_lock);
}

int vgpu_gr_init_sm_id_table(struct gk20a *g)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_vsms_mapping_params *p = &msg.params.vsms_mapping;
	struct tegra_vgpu_vsms_mapping_entry *entry;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	struct sm_info *sm_info;
	int err;
	struct gr_gk20a *gr = &g->gr;
	size_t oob_size;
	void *handle = NULL;
	u32 sm_id;
	u32 max_sm;

	msg.cmd = TEGRA_VGPU_CMD_GET_VSMS_MAPPING;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "get vsms mapping failed err %d", err);
		return err;
	}

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
					   TEGRA_VGPU_QUEUE_CMD,
					   (void **)&entry, &oob_size);
	if (!handle)
		return -EINVAL;

	max_sm = gr->gpc_count *
			gr->max_tpc_per_gpc_count *
			priv->constants.sm_per_tpc;
	if (p->num_sm > max_sm)
		return -EINVAL;

	if ((p->num_sm * sizeof(*entry)) > oob_size)
		return -EINVAL;

	gr->no_of_sm = p->num_sm;
	for (sm_id = 0; sm_id < p->num_sm; sm_id++, entry++) {
		sm_info = &gr->sm_to_cluster[sm_id];
		sm_info->tpc_index = entry->tpc_index;
		sm_info->gpc_index = entry->gpc_index;
		sm_info->sm_index = entry->sm_index;
		sm_info->global_tpc_index = entry->global_tpc_index;
	}
	vgpu_ivc_oob_put_ptr(handle);

	return 0;
}

int vgpu_gr_init_fs_state(struct gk20a *g)
{
	if (!g->ops.gr.init_sm_id_table)
		return -EINVAL;

	return g->ops.gr.init_sm_id_table(g);
}

int vgpu_gr_update_pc_sampling(struct channel_gk20a *ch, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_channel_update_pc_sampling *p =
						&msg.params.update_pc_sampling;
	struct gk20a *g;
	int err = -EINVAL;

	if (!ch->g)
		return err;
	g = ch->g;
	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_UPDATE_PC_SAMPLING;
	msg.handle = vgpu_get_handle(g);
	p->handle = ch->virt_ctx;
	if (enable)
		p->mode = TEGRA_VGPU_ENABLE_SAMPLING;
	else
		p->mode = TEGRA_VGPU_DISABLE_SAMPLING;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}

int vgpu_gr_set_mmu_debug_mode(struct gk20a *g,
		struct channel_gk20a *ch, bool enable)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_gr_set_mmu_debug_mode_params *p =
					&msg.params.gr_set_mmu_debug_mode;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_GR_SET_MMU_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->ch_handle = ch->virt_ctx;
	p->enable = enable ? 1U : 0U;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);

	return err ? err : msg.ret;
}
