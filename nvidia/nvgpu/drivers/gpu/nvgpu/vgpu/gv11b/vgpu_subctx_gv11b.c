/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "vgpu_subctx_gv11b.h"

#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/channel.h>

#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>

int vgpu_gv11b_alloc_subctx_header(struct channel_gk20a *c)
{
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_alloc_ctx_header_params *p =
				&msg.params.alloc_ctx_header;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_ALLOC_CTX_HEADER;
	msg.handle = vgpu_get_handle(c->g);
	p->ch_handle = c->virt_ctx;
	p->ctx_header_va = __nvgpu_vm_alloc_va(c->vm,
				ctxsw_prog_fecs_header_v(),
				GMMU_PAGE_SIZE_KERNEL);
	if (!p->ctx_header_va) {
		nvgpu_err(c->g, "alloc va failed for ctx_header");
		return -ENOMEM;
	}
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (unlikely(err)) {
		nvgpu_err(c->g, "alloc ctx_header failed err %d", err);
		__nvgpu_vm_free_va(c->vm, p->ctx_header_va,
			GMMU_PAGE_SIZE_KERNEL);
		return err;
	}
	ctxheader->gpu_va = p->ctx_header_va;

	return err;
}

void vgpu_gv11b_free_subctx_header(struct channel_gk20a *c)
{
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_free_ctx_header_params *p =
				&msg.params.free_ctx_header;
	int err;

	if (ctxheader->gpu_va) {
		msg.cmd = TEGRA_VGPU_CMD_FREE_CTX_HEADER;
		msg.handle = vgpu_get_handle(c->g);
		p->ch_handle = c->virt_ctx;
		err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
		err = err ? err : msg.ret;
		if (unlikely(err))
			nvgpu_err(c->g, "free ctx_header failed err %d", err);
		__nvgpu_vm_free_va(c->vm, ctxheader->gpu_va,
				GMMU_PAGE_SIZE_KERNEL);
		ctxheader->gpu_va = 0;
	}
}
