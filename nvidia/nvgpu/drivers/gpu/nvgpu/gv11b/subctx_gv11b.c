/*
 * Volta GPU series Subcontext
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>

#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/utils.h>
#include <nvgpu/channel.h>

#include <nvgpu/hw/gv11b/hw_ram_gv11b.h>
#include <nvgpu/hw/gv11b/hw_ctxsw_prog_gv11b.h>
#include <nvgpu/hw/gv11b/hw_gr_gv11b.h>

#include "gv11b/subctx_gv11b.h"

static void gv11b_subctx_commit_valid_mask(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block);
static void gv11b_subctx_commit_pdb(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block,
				bool replayable);

void gv11b_free_subctx_header(struct channel_gk20a *c)
{
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	struct gk20a *g = c->g;

	nvgpu_log(g, gpu_dbg_fn, "gv11b_free_subctx_header");

	if (ctxheader->gpu_va) {
		nvgpu_gmmu_unmap(c->vm, ctxheader, ctxheader->gpu_va);

		nvgpu_dma_free(g, ctxheader);
	}
}

int gv11b_alloc_subctx_header(struct channel_gk20a *c)
{
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	struct gk20a *g = c->g;
	int ret = 0;

	nvgpu_log(g, gpu_dbg_fn, "gv11b_alloc_subctx_header");

	if (!nvgpu_mem_is_valid(ctxheader)) {
		ret = nvgpu_dma_alloc_sys(g, ctxsw_prog_fecs_header_v(),
				ctxheader);
		if (ret) {
			nvgpu_err(g, "failed to allocate sub ctx header");
			return ret;
		}
		ctxheader->gpu_va = nvgpu_gmmu_map(c->vm,
					ctxheader,
					ctxheader->size,
					0, /* not GPU-cacheable */
					gk20a_mem_flag_none, true,
					ctxheader->aperture);
		if (!ctxheader->gpu_va) {
			nvgpu_err(g, "failed to map ctx header");
			nvgpu_dma_free(g, ctxheader);
			return -ENOMEM;
		}
	}
	return ret;
}

void gv11b_init_subcontext_pdb(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block,
				bool replayable)
{
	gv11b_subctx_commit_pdb(vm, inst_block, replayable);
	gv11b_subctx_commit_valid_mask(vm, inst_block);

}

int gv11b_update_subctx_header(struct channel_gk20a *c, u64 gpu_va)
{
	struct nvgpu_mem *ctxheader = &c->ctx_header;
	struct gk20a *g = c->g;
	int ret = 0;
	u32 addr_lo, addr_hi;
	struct tsg_gk20a *tsg;
	struct nvgpu_gr_ctx *gr_ctx;

	tsg = tsg_gk20a_from_ch(c);
	if (!tsg) {
		return -EINVAL;
	}

	gr_ctx = &tsg->gr_ctx;

	g->ops.mm.l2_flush(g, true);

	/* set priv access map */
	addr_lo = u64_lo32(gr_ctx->global_ctx_buffer_va[PRIV_ACCESS_MAP_VA]);
	addr_hi = u64_hi32(gr_ctx->global_ctx_buffer_va[PRIV_ACCESS_MAP_VA]);
	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_priv_access_map_addr_lo_o(),
		addr_lo);
	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_priv_access_map_addr_hi_o(),
		addr_hi);

	addr_lo = u64_lo32(gr_ctx->patch_ctx.mem.gpu_va);
	addr_hi = u64_hi32(gr_ctx->patch_ctx.mem.gpu_va);
	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_patch_adr_lo_o(),
		addr_lo);
	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_patch_adr_hi_o(),
		addr_hi);

	g->ops.gr.write_pm_ptr(g, ctxheader, gr_ctx->pm_ctx.mem.gpu_va);
	g->ops.gr.write_zcull_ptr(g, ctxheader, gr_ctx->zcull_ctx.gpu_va);

	addr_lo = u64_lo32(gpu_va);
	addr_hi = u64_hi32(gpu_va);

	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_context_buffer_ptr_hi_o(), addr_hi);
	nvgpu_mem_wr(g, ctxheader,
		ctxsw_prog_main_image_context_buffer_ptr_o(), addr_lo);

	nvgpu_mem_wr(g, ctxheader,
                ctxsw_prog_main_image_ctl_o(),
                ctxsw_prog_main_image_ctl_type_per_veid_header_v());

	return ret;
}

void gv11b_subctx_commit_valid_mask(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block)
{
	struct gk20a *g = gk20a_from_vm(vm);

	/* Make all subctx pdbs valid */
	nvgpu_mem_wr32(g, inst_block, 166, 0xffffffff);
	nvgpu_mem_wr32(g, inst_block, 167, 0xffffffff);
}

static void gv11b_subctx_commit_pdb(struct vm_gk20a *vm,
				struct nvgpu_mem *inst_block,
				bool replayable)
{
	struct gk20a *g = gk20a_from_vm(vm);
	u32 lo, hi;
	u32 subctx_id = 0;
	u32 format_word;
	u32 pdb_addr_lo, pdb_addr_hi;
	u64 pdb_addr;
	u32 max_subctx_count = gr_pri_fe_chip_def_info_max_veid_count_init_v();
	u32 aperture = nvgpu_aperture_mask(g, vm->pdb.mem,
				ram_in_sc_page_dir_base_target_sys_mem_ncoh_v(),
				ram_in_sc_page_dir_base_target_sys_mem_coh_v(),
				ram_in_sc_page_dir_base_target_vid_mem_v());

	pdb_addr = nvgpu_mem_get_addr(g, vm->pdb.mem);
	pdb_addr_lo = u64_lo32(pdb_addr >> ram_in_base_shift_v());
	pdb_addr_hi = u64_hi32(pdb_addr);
	format_word = ram_in_sc_page_dir_base_target_f(
		aperture, 0) |
		ram_in_sc_page_dir_base_vol_f(
		ram_in_sc_page_dir_base_vol_true_v(), 0) |
		ram_in_sc_use_ver2_pt_format_f(1, 0) |
		ram_in_sc_big_page_size_f(1, 0) |
		ram_in_sc_page_dir_base_lo_0_f(pdb_addr_lo);

	if (replayable) {
		format_word |=
			ram_in_sc_page_dir_base_fault_replay_tex_f(1, 0) |
			ram_in_sc_page_dir_base_fault_replay_gcc_f(1, 0);
	}

	nvgpu_log(g, gpu_dbg_info, " pdb info lo %x hi %x",
					format_word, pdb_addr_hi);
	for (subctx_id = 0; subctx_id < max_subctx_count; subctx_id++) {
		lo = ram_in_sc_page_dir_base_vol_0_w() + (4 * subctx_id);
		hi = ram_in_sc_page_dir_base_hi_0_w() + (4 * subctx_id);
		nvgpu_mem_wr32(g, inst_block, lo, format_word);
		nvgpu_mem_wr32(g, inst_block, hi, pdb_addr_hi);
	}
}
