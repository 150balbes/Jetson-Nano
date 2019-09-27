/*
 * Virtualized GPU Memory Management
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/bug.h>
#include <nvgpu/vm.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/channel.h>

#include <nvgpu/vgpu/vm.h>
#include <nvgpu/vgpu/vgpu.h>

#include "mm_vgpu.h"
#include "gk20a/gk20a.h"
#include "gk20a/mm_gk20a.h"
#include "gm20b/mm_gm20b.h"

static int vgpu_init_mm_setup_sw(struct gk20a *g)
{
	struct mm_gk20a *mm = &g->mm;

	nvgpu_log_fn(g, " ");

	if (mm->sw_ready) {
		nvgpu_log_fn(g, "skip init");
		return 0;
	}

	nvgpu_mutex_init(&mm->tlb_lock);
	nvgpu_mutex_init(&mm->priv_lock);

	mm->g = g;

	/*TBD: make channel vm size configurable */
	mm->channel.user_size = NV_MM_DEFAULT_USER_SIZE;
	mm->channel.kernel_size = NV_MM_DEFAULT_KERNEL_SIZE;

	nvgpu_log_info(g, "channel vm size: user %dMB  kernel %dMB",
		       (int)(mm->channel.user_size >> 20),
		       (int)(mm->channel.kernel_size >> 20));

	mm->sw_ready = true;

	return 0;
}

int vgpu_init_mm_support(struct gk20a *g)
{
	int err;

	nvgpu_log_fn(g, " ");

	err = vgpu_init_mm_setup_sw(g);
	if (err)
		return err;

	if (g->ops.mm.init_mm_setup_hw)
		err = g->ops.mm.init_mm_setup_hw(g);

	return err;
}

void vgpu_locked_gmmu_unmap(struct vm_gk20a *vm,
				u64 vaddr,
				u64 size,
				u32 pgsz_idx,
				bool va_allocated,
				enum gk20a_mem_rw_flag rw_flag,
				bool sparse,
				struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_params *p = &msg.params.as_map;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_AS_UNMAP;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	p->gpu_va = vaddr;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		nvgpu_err(g, "failed to update gmmu ptes on unmap");

	if (va_allocated) {
		err = __nvgpu_vm_free_va(vm, vaddr, pgsz_idx);
		if (err)
			nvgpu_err(g, "failed to free va");
	}
	/* TLB invalidate handled on server side */
}

/*
 * This is called by the common VM init routine to handle vGPU specifics of
 * intializing a VM on a vGPU. This alone is not enough to init a VM. See
 * nvgpu_vm_init().
 */
int vgpu_vm_init(struct gk20a *g, struct vm_gk20a *vm)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_share_params *p = &msg.params.as_share;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_AS_ALLOC_SHARE;
	msg.handle = vgpu_get_handle(g);
	p->size = vm->va_limit;
	p->big_page_size = vm->big_page_size;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		return -ENOMEM;

	vm->handle = p->handle;

	return 0;
}

/*
 * Similar to vgpu_vm_init() this is called as part of the cleanup path for
 * VMs. This alone is not enough to remove a VM - see nvgpu_vm_remove().
 */
void vgpu_vm_remove(struct vm_gk20a *vm)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_share_params *p = &msg.params.as_share;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_AS_FREE_SHARE;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

u64 vgpu_bar1_map(struct gk20a *g, struct nvgpu_mem *mem)
{
	u64 addr = nvgpu_mem_get_addr(g, mem);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_params *p = &msg.params.as_map;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_MAP_BAR1;
	msg.handle = vgpu_get_handle(g);
	p->addr = addr;
	p->size = mem->size;
	p->iova = 0;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		addr = 0;
	else
		addr = p->gpu_va;

	return addr;
}

int vgpu_vm_bind_channel(struct vm_gk20a *vm,
				struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_bind_share_params *p = &msg.params.as_bind_share;
	int err;
	struct gk20a *g = ch->g;

	nvgpu_log_fn(g, " ");

	ch->vm = vm;
	msg.cmd = TEGRA_VGPU_CMD_AS_BIND_SHARE;
	msg.handle = vgpu_get_handle(ch->g);
	p->as_handle = vm->handle;
	p->chan_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret) {
		ch->vm = NULL;
		err = -ENOMEM;
	}

	if (ch->vm)
		nvgpu_vm_get(ch->vm);

	return err;
}

static void vgpu_cache_maint(u64 handle, u8 op)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_cache_maint_params *p = &msg.params.cache_maint;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_CACHE_MAINT;
	msg.handle = handle;
	p->op = op;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}

int vgpu_mm_fb_flush(struct gk20a *g)
{

	nvgpu_log_fn(g, " ");

	vgpu_cache_maint(vgpu_get_handle(g), TEGRA_VGPU_FB_FLUSH);
	return 0;
}

void vgpu_mm_l2_invalidate(struct gk20a *g)
{

	nvgpu_log_fn(g, " ");

	vgpu_cache_maint(vgpu_get_handle(g), TEGRA_VGPU_L2_MAINT_INV);
}

void vgpu_mm_l2_flush(struct gk20a *g, bool invalidate)
{
	u8 op;

	nvgpu_log_fn(g, " ");

	if (invalidate)
		op = TEGRA_VGPU_L2_MAINT_FLUSH_INV;
	else
		op =  TEGRA_VGPU_L2_MAINT_FLUSH;

	vgpu_cache_maint(vgpu_get_handle(g), op);
}

int vgpu_mm_tlb_invalidate(struct gk20a *g, struct nvgpu_mem *pdb)
{
	nvgpu_log_fn(g, " ");

	nvgpu_err(g, "call to RM server not supported");
	return 0;
}

void vgpu_mm_mmu_set_debug_mode(struct gk20a *g, bool enable)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_mmu_debug_mode *p = &msg.params.mmu_debug_mode;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_SET_MMU_DEBUG_MODE;
	msg.handle = vgpu_get_handle(g);
	p->enable = (u32)enable;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	WARN_ON(err || msg.ret);
}
