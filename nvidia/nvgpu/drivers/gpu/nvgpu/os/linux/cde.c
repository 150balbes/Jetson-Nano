/*
 * Color decompression engine support
 *
 * Copyright (c) 2014-2018, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/dma-buf.h>
#include <uapi/linux/nvgpu.h>

#include <trace/events/gk20a.h>

#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/timers.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/firmware.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/channel.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/linux/vm.h>

#include "gk20a/mm_gk20a.h"
#include "gk20a/fence_gk20a.h"
#include "gk20a/gr_gk20a.h"

#include "cde.h"
#include "os_linux.h"
#include "dmabuf.h"
#include "channel.h"
#include "cde_gm20b.h"
#include "cde_gp10b.h"

#include <nvgpu/hw/gk20a/hw_ccsr_gk20a.h>
#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

static int gk20a_cde_load(struct gk20a_cde_ctx *cde_ctx);
static struct gk20a_cde_ctx *gk20a_cde_allocate_context(struct nvgpu_os_linux *l);

#define CTX_DELETE_TIME 1000

#define MAX_CTX_USE_COUNT 42
#define MAX_CTX_RETRY_TIME 2000

static dma_addr_t gpuva_to_iova_base(struct vm_gk20a *vm, u64 gpu_vaddr)
{
	struct nvgpu_mapped_buf *buffer;
	dma_addr_t addr = 0;
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);
	buffer = __nvgpu_vm_find_mapped_buf(vm, gpu_vaddr);
	if (buffer)
		addr = nvgpu_mem_get_addr_sgl(g, buffer->os_priv.sgt->sgl);
	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return addr;
}

static void gk20a_deinit_cde_img(struct gk20a_cde_ctx *cde_ctx)
{
	unsigned int i;

	for (i = 0; i < cde_ctx->num_bufs; i++) {
		struct nvgpu_mem *mem = cde_ctx->mem + i;
		nvgpu_dma_unmap_free(cde_ctx->vm, mem);
	}

	nvgpu_kfree(&cde_ctx->l->g, cde_ctx->init_convert_cmd);

	cde_ctx->convert_cmd = NULL;
	cde_ctx->init_convert_cmd = NULL;
	cde_ctx->num_bufs = 0;
	cde_ctx->num_params = 0;
	cde_ctx->init_cmd_num_entries = 0;
	cde_ctx->convert_cmd_num_entries = 0;
	cde_ctx->init_cmd_executed = false;
}

static void gk20a_cde_remove_ctx(struct gk20a_cde_ctx *cde_ctx)
__must_hold(&cde_app->mutex)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct channel_gk20a *ch = cde_ctx->ch;
	struct vm_gk20a *vm = ch->vm;

	trace_gk20a_cde_remove_ctx(cde_ctx);

	/* release mapped memory */
	gk20a_deinit_cde_img(cde_ctx);
	nvgpu_gmmu_unmap(vm, &g->gr.compbit_store.mem,
			 cde_ctx->backing_store_vaddr);

	/*
	 * free the channel
	 * gk20a_channel_close() will also unbind the channel from TSG
	 */
	gk20a_channel_close(ch);
	nvgpu_ref_put(&cde_ctx->tsg->refcount, gk20a_tsg_release);

	/* housekeeping on app */
	nvgpu_list_del(&cde_ctx->list);
	l->cde_app.ctx_count--;
	nvgpu_kfree(g, cde_ctx);
}

static void gk20a_cde_cancel_deleter(struct gk20a_cde_ctx *cde_ctx,
		bool wait_finish)
__releases(&cde_app->mutex)
__acquires(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &cde_ctx->l->cde_app;

	/* permanent contexts do not have deleter works */
	if (!cde_ctx->is_temporary)
		return;

	if (wait_finish) {
		nvgpu_mutex_release(&cde_app->mutex);
		cancel_delayed_work_sync(&cde_ctx->ctx_deleter_work);
		nvgpu_mutex_acquire(&cde_app->mutex);
	} else {
		cancel_delayed_work(&cde_ctx->ctx_deleter_work);
	}
}

static void gk20a_cde_remove_contexts(struct nvgpu_os_linux *l)
__must_hold(&l->cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a_cde_ctx *cde_ctx, *cde_ctx_save;

	/* safe to go off the mutex in cancel_deleter since app is
	 * deinitialised; no new jobs are started. deleter works may be only at
	 * waiting for the mutex or before, going to abort */

	nvgpu_list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->free_contexts, gk20a_cde_ctx, list) {
		gk20a_cde_cancel_deleter(cde_ctx, true);
		gk20a_cde_remove_ctx(cde_ctx);
	}

	nvgpu_list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->used_contexts, gk20a_cde_ctx, list) {
		gk20a_cde_cancel_deleter(cde_ctx, true);
		gk20a_cde_remove_ctx(cde_ctx);
	}
}

static void gk20a_cde_stop(struct nvgpu_os_linux *l)
__must_hold(&l->cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;

	/* prevent further conversions and delayed works from working */
	cde_app->initialised = false;
	/* free all data, empty the list */
	gk20a_cde_remove_contexts(l);
}

void gk20a_cde_destroy(struct nvgpu_os_linux *l)
__acquires(&l->cde_app->mutex)
__releases(&l->cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;

	if (!cde_app->initialised)
		return;

	nvgpu_mutex_acquire(&cde_app->mutex);
	gk20a_cde_stop(l);
	nvgpu_mutex_release(&cde_app->mutex);

	nvgpu_mutex_destroy(&cde_app->mutex);
}

void gk20a_cde_suspend(struct nvgpu_os_linux *l)
__acquires(&l->cde_app->mutex)
__releases(&l->cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a_cde_ctx *cde_ctx, *cde_ctx_save;

	if (!cde_app->initialised)
		return;

	nvgpu_mutex_acquire(&cde_app->mutex);

	nvgpu_list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->free_contexts, gk20a_cde_ctx, list) {
		gk20a_cde_cancel_deleter(cde_ctx, false);
	}

	nvgpu_list_for_each_entry_safe(cde_ctx, cde_ctx_save,
			&cde_app->used_contexts, gk20a_cde_ctx, list) {
		gk20a_cde_cancel_deleter(cde_ctx, false);
	}

	nvgpu_mutex_release(&cde_app->mutex);

}

static int gk20a_cde_create_context(struct nvgpu_os_linux *l)
__must_hold(&l->cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a_cde_ctx *cde_ctx;

	cde_ctx = gk20a_cde_allocate_context(l);
	if (IS_ERR(cde_ctx))
		return PTR_ERR(cde_ctx);

	nvgpu_list_add(&cde_ctx->list, &cde_app->free_contexts);
	cde_app->ctx_count++;
	if (cde_app->ctx_count > cde_app->ctx_count_top)
		cde_app->ctx_count_top = cde_app->ctx_count;

	return 0;
}

static int gk20a_cde_create_contexts(struct nvgpu_os_linux *l)
__must_hold(&l->cde_app->mutex)
{
	int err;
	int i;

	for (i = 0; i < NUM_CDE_CONTEXTS; i++) {
		err = gk20a_cde_create_context(l);
		if (err)
			goto out;
	}

	return 0;
out:
	gk20a_cde_remove_contexts(l);
	return err;
}

static int gk20a_init_cde_buf(struct gk20a_cde_ctx *cde_ctx,
			      struct nvgpu_firmware *img,
			      struct gk20a_cde_hdr_buf *buf)
{
	struct nvgpu_mem *mem;
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	int err;

	/* check that the file can hold the buf */
	if (buf->data_byte_offset != 0 &&
	    buf->data_byte_offset + buf->num_bytes > img->size) {
		nvgpu_warn(g, "cde: invalid data section. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	/* check that we have enough buf elems available */
	if (cde_ctx->num_bufs >= MAX_CDE_BUFS) {
		nvgpu_warn(g, "cde: invalid data section. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -ENOMEM;
	}

	/* allocate buf */
	mem = cde_ctx->mem + cde_ctx->num_bufs;
	err = nvgpu_dma_alloc_map_sys(cde_ctx->vm, buf->num_bytes, mem);
	if (err) {
		nvgpu_warn(g, "cde: could not allocate device memory. buffer idx = %d",
			   cde_ctx->num_bufs);
		return -ENOMEM;
	}

	/* copy the content */
	if (buf->data_byte_offset != 0)
		memcpy(mem->cpu_va, img->data + buf->data_byte_offset,
		       buf->num_bytes);

	cde_ctx->num_bufs++;

	return 0;
}

static int gk20a_replace_data(struct gk20a_cde_ctx *cde_ctx, void *target,
			      int type, s32 shift, u64 mask, u64 value)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	u32 *target_mem_ptr = target;
	u64 *target_mem_ptr_u64 = target;
	u64 current_value, new_value;

	value = (shift >= 0) ? value << shift : value >> -shift;
	value &= mask;

	/* read current data from the location */
	current_value = 0;
	if (type == TYPE_PARAM_TYPE_U32) {
		if (mask != 0xfffffffful)
			current_value = *target_mem_ptr;
	} else if (type == TYPE_PARAM_TYPE_U64_LITTLE) {
		if (mask != ~0ul)
			current_value = *target_mem_ptr_u64;
	} else if (type == TYPE_PARAM_TYPE_U64_BIG) {
		current_value = *target_mem_ptr_u64;
		current_value = (u64)(current_value >> 32) |
			(u64)(current_value << 32);
	} else {
		nvgpu_warn(g, "cde: unknown type. type=%d",
			   type);
		return -EINVAL;
	}

	current_value &= ~mask;
	new_value = current_value | value;

	/* store the element data back */
	if (type == TYPE_PARAM_TYPE_U32)
		*target_mem_ptr = (u32)new_value;
	else if (type == TYPE_PARAM_TYPE_U64_LITTLE)
		*target_mem_ptr_u64 = new_value;
	else  {
		new_value = (u64)(new_value >> 32) |
			(u64)(new_value << 32);
		*target_mem_ptr_u64 = new_value;
	}

	return 0;
}

static int gk20a_init_cde_replace(struct gk20a_cde_ctx *cde_ctx,
				  struct nvgpu_firmware *img,
				  struct gk20a_cde_hdr_replace *replace)
{
	struct nvgpu_mem *source_mem;
	struct nvgpu_mem *target_mem;
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	u32 *target_mem_ptr;
	u64 vaddr;
	int err;

	if (replace->target_buf >= cde_ctx->num_bufs ||
	    replace->source_buf >= cde_ctx->num_bufs) {
		nvgpu_warn(g, "cde: invalid buffer. target_buf=%u, source_buf=%u, num_bufs=%d",
			   replace->target_buf, replace->source_buf,
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	source_mem = cde_ctx->mem + replace->source_buf;
	target_mem = cde_ctx->mem + replace->target_buf;
	target_mem_ptr = target_mem->cpu_va;

	if (source_mem->size < (replace->source_byte_offset + 3) ||
	    target_mem->size < (replace->target_byte_offset + 3)) {
		nvgpu_warn(g, "cde: invalid buffer offsets. target_buf_offs=%lld, source_buf_offs=%lld, source_buf_size=%zu, dest_buf_size=%zu",
			   replace->target_byte_offset,
			   replace->source_byte_offset,
			 source_mem->size,
			 target_mem->size);
		return -EINVAL;
	}

	/* calculate the target pointer */
	target_mem_ptr += (replace->target_byte_offset / sizeof(u32));

	/* determine patch value */
	vaddr = source_mem->gpu_va + replace->source_byte_offset;
	err = gk20a_replace_data(cde_ctx, target_mem_ptr, replace->type,
				 replace->shift, replace->mask,
				 vaddr);
	if (err) {
		nvgpu_warn(g, "cde: replace failed. err=%d, target_buf=%u, target_buf_offs=%lld, source_buf=%u, source_buf_offs=%lld",
			   err, replace->target_buf,
			   replace->target_byte_offset,
			   replace->source_buf,
			   replace->source_byte_offset);
	}

	return err;
}

static int gk20a_cde_patch_params(struct gk20a_cde_ctx *cde_ctx)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct nvgpu_mem *target_mem;
	u32 *target_mem_ptr;
	u64 new_data;
	int user_id = 0, err;
	unsigned int i;

	for (i = 0; i < cde_ctx->num_params; i++) {
		struct gk20a_cde_hdr_param *param = cde_ctx->params + i;
		target_mem = cde_ctx->mem + param->target_buf;
		target_mem_ptr = target_mem->cpu_va;
		target_mem_ptr += (param->target_byte_offset / sizeof(u32));

		switch (param->id) {
		case TYPE_PARAM_COMPTAGS_PER_CACHELINE:
			new_data = g->gr.comptags_per_cacheline;
			break;
		case TYPE_PARAM_GPU_CONFIGURATION:
			new_data = (u64)g->ltc_count * g->gr.slices_per_ltc *
				g->gr.cacheline_size;
			break;
		case TYPE_PARAM_FIRSTPAGEOFFSET:
			new_data = cde_ctx->surf_param_offset;
			break;
		case TYPE_PARAM_NUMPAGES:
			new_data = cde_ctx->surf_param_lines;
			break;
		case TYPE_PARAM_BACKINGSTORE:
			new_data = cde_ctx->backing_store_vaddr;
			break;
		case TYPE_PARAM_DESTINATION:
			new_data = cde_ctx->compbit_vaddr;
			break;
		case TYPE_PARAM_DESTINATION_SIZE:
			new_data = cde_ctx->compbit_size;
			break;
		case TYPE_PARAM_BACKINGSTORE_SIZE:
			new_data = g->gr.compbit_store.mem.size;
			break;
		case TYPE_PARAM_SOURCE_SMMU_ADDR:
			new_data = gpuva_to_iova_base(cde_ctx->vm,
						      cde_ctx->surf_vaddr);
			if (new_data == 0) {
				nvgpu_warn(g, "cde: failed to find 0x%llx",
						cde_ctx->surf_vaddr);
				return -EINVAL;
			}
			break;
		case TYPE_PARAM_BACKINGSTORE_BASE_HW:
			new_data = g->gr.compbit_store.base_hw;
			break;
		case TYPE_PARAM_GOBS_PER_COMPTAGLINE_PER_SLICE:
			new_data = g->gr.gobs_per_comptagline_per_slice;
			break;
		case TYPE_PARAM_SCATTERBUFFER:
			new_data = cde_ctx->scatterbuffer_vaddr;
			break;
		case TYPE_PARAM_SCATTERBUFFER_SIZE:
			new_data = cde_ctx->scatterbuffer_size;
			break;
		default:
			user_id = param->id - NUM_RESERVED_PARAMS;
			if (user_id < 0 || user_id >= MAX_CDE_USER_PARAMS)
				continue;
			new_data = cde_ctx->user_param_values[user_id];
		}

		nvgpu_log(g, gpu_dbg_cde, "cde: patch: idx_in_file=%d	param_id=%d	target_buf=%u	target_byte_offset=%lld	data_value=0x%llx	data_offset/data_diff=%lld	data_type=%d	data_shift=%d	data_mask=0x%llx",
			  i, param->id, param->target_buf,
			  param->target_byte_offset, new_data,
			  param->data_offset, param->type, param->shift,
			  param->mask);

		new_data += param->data_offset;

		err = gk20a_replace_data(cde_ctx, target_mem_ptr, param->type,
					 param->shift, param->mask, new_data);

		if (err) {
			nvgpu_warn(g, "cde: patch failed. err=%d, idx=%d, id=%d, target_buf=%u, target_buf_offs=%lld, patch_value=%llu",
				   err, i, param->id, param->target_buf,
				   param->target_byte_offset, new_data);
			return err;
		}
	}

	return 0;
}

static int gk20a_init_cde_param(struct gk20a_cde_ctx *cde_ctx,
				struct nvgpu_firmware *img,
				struct gk20a_cde_hdr_param *param)
{
	struct nvgpu_mem *target_mem;
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;

	if (param->target_buf >= cde_ctx->num_bufs) {
		nvgpu_warn(g, "cde: invalid buffer parameter. param idx = %d, target_buf=%u, num_bufs=%u",
			   cde_ctx->num_params, param->target_buf,
			   cde_ctx->num_bufs);
		return -EINVAL;
	}

	target_mem = cde_ctx->mem + param->target_buf;
	if (target_mem->size < (param->target_byte_offset + 3)) {
		nvgpu_warn(g, "cde: invalid buffer parameter. param idx = %d, target_buf_offs=%lld, target_buf_size=%zu",
			   cde_ctx->num_params, param->target_byte_offset,
			   target_mem->size);
		return -EINVAL;
	}

	/* does this parameter fit into our parameter structure */
	if (cde_ctx->num_params >= MAX_CDE_PARAMS) {
		nvgpu_warn(g, "cde: no room for new parameters param idx = %d",
			   cde_ctx->num_params);
		return -ENOMEM;
	}

	/* is the given id valid? */
	if (param->id >= NUM_RESERVED_PARAMS + MAX_CDE_USER_PARAMS) {
		nvgpu_warn(g, "cde: parameter id is not valid. param idx = %d, id=%u, max=%u",
			   param->id, cde_ctx->num_params,
			   NUM_RESERVED_PARAMS + MAX_CDE_USER_PARAMS);
		return -EINVAL;
	}

	cde_ctx->params[cde_ctx->num_params] = *param;
	cde_ctx->num_params++;

	return 0;
}

static int gk20a_init_cde_required_class(struct gk20a_cde_ctx *cde_ctx,
					 struct nvgpu_firmware *img,
					 u32 required_class)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	int err;

	/* CDE enabled */
	cde_ctx->ch->cde = true;

	err = gk20a_alloc_obj_ctx(cde_ctx->ch, required_class, 0);
	if (err) {
		nvgpu_warn(g, "cde: failed to allocate ctx. err=%d",
			   err);
		return err;
	}

	return 0;
}

static int gk20a_init_cde_command(struct gk20a_cde_ctx *cde_ctx,
				  struct nvgpu_firmware *img,
				  u32 op,
				  struct gk20a_cde_cmd_elem *cmd_elem,
				  u32 num_elems)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct nvgpu_gpfifo_entry **gpfifo, *gpfifo_elem;
	u32 *num_entries;
	unsigned int i;

	/* check command type */
	if (op == TYPE_BUF_COMMAND_INIT) {
		gpfifo = &cde_ctx->init_convert_cmd;
		num_entries = &cde_ctx->init_cmd_num_entries;
	} else if (op == TYPE_BUF_COMMAND_CONVERT) {
		gpfifo = &cde_ctx->convert_cmd;
		num_entries = &cde_ctx->convert_cmd_num_entries;
	} else {
		nvgpu_warn(g, "cde: unknown command. op=%u",
			   op);
		return -EINVAL;
	}

	/* allocate gpfifo entries to be pushed */
	*gpfifo = nvgpu_kzalloc(g,
				sizeof(struct nvgpu_gpfifo_entry) * num_elems);
	if (!*gpfifo) {
		nvgpu_warn(g, "cde: could not allocate memory for gpfifo entries");
		return -ENOMEM;
	}

	gpfifo_elem = *gpfifo;
	for (i = 0; i < num_elems; i++, cmd_elem++, gpfifo_elem++) {
		struct nvgpu_mem *target_mem;

		/* validate the current entry */
		if (cmd_elem->target_buf >= cde_ctx->num_bufs) {
			nvgpu_warn(g, "cde: target buffer is not available (target=%u, num_bufs=%u)",
				   cmd_elem->target_buf, cde_ctx->num_bufs);
			return -EINVAL;
		}

		target_mem = cde_ctx->mem + cmd_elem->target_buf;
		if (target_mem->size<
		    cmd_elem->target_byte_offset + cmd_elem->num_bytes) {
			nvgpu_warn(g, "cde: target buffer cannot hold all entries (target_size=%zu, target_byte_offset=%lld, num_bytes=%llu)",
				   target_mem->size,
				   cmd_elem->target_byte_offset,
				   cmd_elem->num_bytes);
			return -EINVAL;
		}

		/* store the element into gpfifo */
		gpfifo_elem->entry0 =
			u64_lo32(target_mem->gpu_va +
			cmd_elem->target_byte_offset);
		gpfifo_elem->entry1 =
			u64_hi32(target_mem->gpu_va +
			cmd_elem->target_byte_offset) |
			pbdma_gp_entry1_length_f(cmd_elem->num_bytes /
						 sizeof(u32));
	}

	*num_entries = num_elems;
	return 0;
}

static int gk20a_cde_pack_cmdbufs(struct gk20a_cde_ctx *cde_ctx)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	unsigned long init_bytes = cde_ctx->init_cmd_num_entries *
		sizeof(struct nvgpu_gpfifo_entry);
	unsigned long conv_bytes = cde_ctx->convert_cmd_num_entries *
		sizeof(struct nvgpu_gpfifo_entry);
	unsigned long total_bytes = init_bytes + conv_bytes;
	struct nvgpu_gpfifo_entry *combined_cmd;

	/* allocate buffer that has space for both */
	combined_cmd = nvgpu_kzalloc(g, total_bytes);
	if (!combined_cmd) {
		nvgpu_warn(g,
			"cde: could not allocate memory for gpfifo entries");
		return -ENOMEM;
	}

	/* move the original init here and append convert */
	memcpy(combined_cmd, cde_ctx->init_convert_cmd, init_bytes);
	memcpy(combined_cmd + cde_ctx->init_cmd_num_entries,
			cde_ctx->convert_cmd, conv_bytes);

	nvgpu_kfree(g, cde_ctx->init_convert_cmd);
	nvgpu_kfree(g, cde_ctx->convert_cmd);

	cde_ctx->init_convert_cmd = combined_cmd;
	cde_ctx->convert_cmd = combined_cmd
		+ cde_ctx->init_cmd_num_entries;

	return 0;
}

static int gk20a_init_cde_img(struct gk20a_cde_ctx *cde_ctx,
			      struct nvgpu_firmware *img)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct gk20a_cde_app *cde_app = &l->cde_app;
	u32 *data = (u32 *)img->data;
	u32 num_of_elems;
	struct gk20a_cde_hdr_elem *elem;
	u32 min_size = 0;
	int err = 0;
	unsigned int i;

	min_size += 2 * sizeof(u32);
	if (img->size < min_size) {
		nvgpu_warn(g, "cde: invalid image header");
		return -EINVAL;
	}

	cde_app->firmware_version = data[0];
	num_of_elems = data[1];

	min_size += num_of_elems * sizeof(*elem);
	if (img->size < min_size) {
		nvgpu_warn(g, "cde: bad image");
		return -EINVAL;
	}

	elem = (struct gk20a_cde_hdr_elem *)&data[2];
	for (i = 0; i < num_of_elems; i++) {
		int err = 0;
		switch (elem->type) {
		case TYPE_BUF:
			err = gk20a_init_cde_buf(cde_ctx, img, &elem->buf);
			break;
		case TYPE_REPLACE:
			err = gk20a_init_cde_replace(cde_ctx, img,
						     &elem->replace);
			break;
		case TYPE_PARAM:
			err = gk20a_init_cde_param(cde_ctx, img, &elem->param);
			break;
		case TYPE_REQUIRED_CLASS:
			err = gk20a_init_cde_required_class(cde_ctx, img,
				elem->required_class);
			break;
		case TYPE_COMMAND:
		{
			struct gk20a_cde_cmd_elem *cmd = (void *)
				&img->data[elem->command.data_byte_offset];
			err = gk20a_init_cde_command(cde_ctx, img,
				elem->command.op, cmd,
				elem->command.num_entries);
			break;
		}
		case TYPE_ARRAY:
			memcpy(&cde_app->arrays[elem->array.id][0],
				elem->array.data,
				MAX_CDE_ARRAY_ENTRIES*sizeof(u32));
			break;
		default:
			nvgpu_warn(g, "cde: unknown header element");
			err = -EINVAL;
		}

		if (err)
			goto deinit_image;

		elem++;
	}

	if (!cde_ctx->init_convert_cmd || !cde_ctx->init_cmd_num_entries) {
		nvgpu_warn(g, "cde: convert command not defined");
		err = -EINVAL;
		goto deinit_image;
	}

	if (!cde_ctx->convert_cmd || !cde_ctx->convert_cmd_num_entries) {
		nvgpu_warn(g, "cde: convert command not defined");
		err = -EINVAL;
		goto deinit_image;
	}

	err = gk20a_cde_pack_cmdbufs(cde_ctx);
	if (err)
		goto deinit_image;

	return 0;

deinit_image:
	gk20a_deinit_cde_img(cde_ctx);
	return err;
}

static int gk20a_cde_execute_buffer(struct gk20a_cde_ctx *cde_ctx,
				    u32 op, struct nvgpu_channel_fence *fence,
				    u32 flags, struct gk20a_fence **fence_out)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct nvgpu_gpfifo_entry *gpfifo = NULL;
	int num_entries = 0;

	/* check command type */
	if (op == TYPE_BUF_COMMAND_INIT) {
		/* both init and convert combined */
		gpfifo = cde_ctx->init_convert_cmd;
		num_entries = cde_ctx->init_cmd_num_entries
			+ cde_ctx->convert_cmd_num_entries;
	} else if (op == TYPE_BUF_COMMAND_CONVERT) {
		gpfifo = cde_ctx->convert_cmd;
		num_entries = cde_ctx->convert_cmd_num_entries;
	} else if (op == TYPE_BUF_COMMAND_NOOP) {
		/* Any non-null gpfifo will suffice with 0 num_entries */
		gpfifo = cde_ctx->init_convert_cmd;
		num_entries = 0;
	} else {
		nvgpu_warn(g, "cde: unknown buffer");
		return -EINVAL;
	}

	if (gpfifo == NULL) {
		nvgpu_warn(g, "cde: buffer not available");
		return -ENOSYS;
	}

	return nvgpu_submit_channel_gpfifo_kernel(cde_ctx->ch, gpfifo,
			num_entries, flags, fence, fence_out);
}

static void gk20a_cde_ctx_release(struct gk20a_cde_ctx *cde_ctx)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &cde_ctx->l->cde_app;
	struct gk20a *g = &cde_ctx->l->g;

	nvgpu_log(g, gpu_dbg_cde_ctx, "releasing use on %p", cde_ctx);
	trace_gk20a_cde_release(cde_ctx);

	nvgpu_mutex_acquire(&cde_app->mutex);

	if (cde_ctx->in_use) {
		cde_ctx->in_use = false;
		nvgpu_list_move(&cde_ctx->list, &cde_app->free_contexts);
		cde_app->ctx_usecount--;
	} else {
		nvgpu_log_info(g, "double release cde context %p", cde_ctx);
	}

	nvgpu_mutex_release(&cde_app->mutex);
}

static void gk20a_cde_ctx_deleter_fn(struct work_struct *work)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct delayed_work *delay_work = to_delayed_work(work);
	struct gk20a_cde_ctx *cde_ctx = container_of(delay_work,
			struct gk20a_cde_ctx, ctx_deleter_work);
	struct gk20a_cde_app *cde_app = &cde_ctx->l->cde_app;
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	int err;

	/* someone has just taken it? engine deletion started? */
	if (cde_ctx->in_use || !cde_app->initialised)
		return;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: attempting to delete temporary %p", cde_ctx);

	err = gk20a_busy(g);
	if (err) {
		/* this context would find new use anyway later, so not freeing
		 * here does not leak anything */
		nvgpu_warn(g, "cde: cannot set gk20a on, postponing"
				" temp ctx deletion");
		return;
	}

	nvgpu_mutex_acquire(&cde_app->mutex);
	if (cde_ctx->in_use || !cde_app->initialised) {
		nvgpu_log(g, gpu_dbg_cde_ctx,
				"cde: context use raced, not deleting %p",
				cde_ctx);
		goto out;
	}

	WARN(delayed_work_pending(&cde_ctx->ctx_deleter_work),
			"double pending %p", cde_ctx);

	gk20a_cde_remove_ctx(cde_ctx);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: destroyed %p count=%d use=%d max=%d",
			cde_ctx, cde_app->ctx_count, cde_app->ctx_usecount,
			cde_app->ctx_count_top);

out:
	nvgpu_mutex_release(&cde_app->mutex);
	gk20a_idle(g);
}

static struct gk20a_cde_ctx *gk20a_cde_do_get_context(struct nvgpu_os_linux *l)
__must_hold(&cde_app->mutex)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a_cde_ctx *cde_ctx;

	/* exhausted? */

	if (cde_app->ctx_usecount >= MAX_CTX_USE_COUNT)
		return ERR_PTR(-EAGAIN);

	/* idle context available? */

	if (!nvgpu_list_empty(&cde_app->free_contexts)) {
		cde_ctx = nvgpu_list_first_entry(&cde_app->free_contexts,
				gk20a_cde_ctx, list);
		nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx,
				"cde: got free %p count=%d use=%d max=%d",
				cde_ctx, cde_app->ctx_count,
				cde_app->ctx_usecount,
				cde_app->ctx_count_top);
		trace_gk20a_cde_get_context(cde_ctx);

		/* deleter work may be scheduled, but in_use prevents it */
		cde_ctx->in_use = true;
		nvgpu_list_move(&cde_ctx->list, &cde_app->used_contexts);
		cde_app->ctx_usecount++;

		/* cancel any deletions now that ctx is in use */
		gk20a_cde_cancel_deleter(cde_ctx, true);
		return cde_ctx;
	}

	/* no free contexts, get a temporary one */

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx,
			"cde: no free contexts, count=%d",
			cde_app->ctx_count);

	cde_ctx = gk20a_cde_allocate_context(l);
	if (IS_ERR(cde_ctx)) {
		nvgpu_warn(g, "cde: cannot allocate context: %ld",
				PTR_ERR(cde_ctx));
		return cde_ctx;
	}

	trace_gk20a_cde_get_context(cde_ctx);
	cde_ctx->in_use = true;
	cde_ctx->is_temporary = true;
	cde_app->ctx_usecount++;
	cde_app->ctx_count++;
	if (cde_app->ctx_count > cde_app->ctx_count_top)
		cde_app->ctx_count_top = cde_app->ctx_count;
	nvgpu_list_add(&cde_ctx->list, &cde_app->used_contexts);

	return cde_ctx;
}

static struct gk20a_cde_ctx *gk20a_cde_get_context(struct nvgpu_os_linux *l)
__releases(&cde_app->mutex)
__acquires(&cde_app->mutex)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a_cde_ctx *cde_ctx = NULL;
	struct nvgpu_timeout timeout;

	nvgpu_timeout_init(g, &timeout, MAX_CTX_RETRY_TIME,
			   NVGPU_TIMER_CPU_TIMER);

	do {
		cde_ctx = gk20a_cde_do_get_context(l);
		if (PTR_ERR(cde_ctx) != -EAGAIN)
			break;

		/* exhausted, retry */
		nvgpu_mutex_release(&cde_app->mutex);
		cond_resched();
		nvgpu_mutex_acquire(&cde_app->mutex);
	} while (!nvgpu_timeout_expired(&timeout));

	return cde_ctx;
}

static struct gk20a_cde_ctx *gk20a_cde_allocate_context(struct nvgpu_os_linux *l)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_ctx *cde_ctx;
	int ret;

	cde_ctx = nvgpu_kzalloc(g, sizeof(*cde_ctx));
	if (!cde_ctx)
		return ERR_PTR(-ENOMEM);

	cde_ctx->l = l;
	cde_ctx->dev = dev_from_gk20a(g);

	ret = gk20a_cde_load(cde_ctx);
	if (ret) {
		nvgpu_kfree(g, cde_ctx);
		return ERR_PTR(ret);
	}

	nvgpu_init_list_node(&cde_ctx->list);
	cde_ctx->is_temporary = false;
	cde_ctx->in_use = false;
	INIT_DELAYED_WORK(&cde_ctx->ctx_deleter_work,
			gk20a_cde_ctx_deleter_fn);

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: allocated %p", cde_ctx);
	trace_gk20a_cde_allocate_context(cde_ctx);
	return cde_ctx;
}

static u32 gk20a_cde_mapping_page_size(struct vm_gk20a *vm,
				       u32 map_offset, u32 map_size)
{
	struct gk20a *g = gk20a_from_vm(vm);

	/*
	 * To be simple we will just make the map size depend on the
	 * iommu'ability of the driver. If there's an IOMMU we can rely on
	 * buffers being contiguous. If not, then we'll use 4k pages since we
	 * know that will work for any buffer.
	 */
	if (!nvgpu_iommuable(g))
		return SZ_4K;

	/*
	 * If map size or offset is not 64K aligned then use small pages.
	 */
	if (map_size & (vm->big_page_size - 1) ||
	    map_offset & (vm->big_page_size - 1))
		return SZ_4K;

	return vm->big_page_size;
}

int gk20a_cde_convert(struct nvgpu_os_linux *l,
		      struct dma_buf *compbits_scatter_buf,
		      u64 compbits_byte_offset,
		      u64 scatterbuffer_byte_offset,
		      struct nvgpu_channel_fence *fence,
		      u32 __flags, struct gk20a_cde_param *params,
		      int num_params, struct gk20a_fence **fence_out)
__acquires(&l->cde_app->mutex)
__releases(&l->cde_app->mutex)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_ctx *cde_ctx = NULL;
	struct gk20a_comptags comptags;
	struct nvgpu_os_buffer os_buf = {
		compbits_scatter_buf,
		NULL,
		dev_from_gk20a(g)
	};
	u64 mapped_compbits_offset = 0;
	u64 compbits_size = 0;
	u64 mapped_scatterbuffer_offset = 0;
	u64 scatterbuffer_size = 0;
	u64 map_vaddr = 0;
	u64 map_offset = 0;
	u64 map_size = 0;
	u8 *surface = NULL;
	u64 big_page_mask = 0;
	u32 flags;
	int err, i;
	const s16 compbits_kind = 0;
	u32 submit_op;
	struct dma_buf_attachment *attachment;

	nvgpu_log(g, gpu_dbg_cde, "compbits_byte_offset=%llu scatterbuffer_byte_offset=%llu",
		  compbits_byte_offset, scatterbuffer_byte_offset);

	/* scatter buffer must be after compbits buffer */
	if (scatterbuffer_byte_offset &&
	    scatterbuffer_byte_offset < compbits_byte_offset)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	nvgpu_mutex_acquire(&l->cde_app.mutex);
	cde_ctx = gk20a_cde_get_context(l);
	nvgpu_mutex_release(&l->cde_app.mutex);
	if (IS_ERR(cde_ctx)) {
		err = PTR_ERR(cde_ctx);
		goto exit_idle;
	}

	/* First, map the buffer to local va */

	/* ensure that the compbits buffer has drvdata */
	err = gk20a_dmabuf_alloc_drvdata(compbits_scatter_buf,
			dev_from_gk20a(g));
	if (err)
		goto exit_idle;

	/* compbits don't start at page aligned offset, so we need to align
	   the region to be mapped */
	big_page_mask = cde_ctx->vm->big_page_size - 1;
	map_offset = compbits_byte_offset & ~big_page_mask;
	map_size = compbits_scatter_buf->size - map_offset;


	/* compute compbit start offset from the beginning of the mapped
	   area */
	mapped_compbits_offset = compbits_byte_offset - map_offset;
	if (scatterbuffer_byte_offset) {
		compbits_size = scatterbuffer_byte_offset -
				compbits_byte_offset;
		mapped_scatterbuffer_offset = scatterbuffer_byte_offset -
					      map_offset;
		scatterbuffer_size = compbits_scatter_buf->size -
				     scatterbuffer_byte_offset;
	} else {
		compbits_size = compbits_scatter_buf->size -
				compbits_byte_offset;
	}

	nvgpu_log(g, gpu_dbg_cde, "map_offset=%llu map_size=%llu",
		  map_offset, map_size);
	nvgpu_log(g, gpu_dbg_cde, "mapped_compbits_offset=%llu compbits_size=%llu",
		  mapped_compbits_offset, compbits_size);
	nvgpu_log(g, gpu_dbg_cde, "mapped_scatterbuffer_offset=%llu scatterbuffer_size=%llu",
		  mapped_scatterbuffer_offset, scatterbuffer_size);


	/* map the destination buffer */
	get_dma_buf(compbits_scatter_buf); /* a ref for nvgpu_vm_map_linux */
	err = nvgpu_vm_map_linux(cde_ctx->vm, compbits_scatter_buf, 0,
				 NVGPU_VM_MAP_CACHEABLE |
				 NVGPU_VM_MAP_DIRECT_KIND_CTRL,
				 gk20a_cde_mapping_page_size(cde_ctx->vm,
							     map_offset,
							     map_size),
				 NV_KIND_INVALID,
				 compbits_kind, /* incompressible kind */
				 gk20a_mem_flag_none,
				 map_offset, map_size,
				 NULL,
				 &map_vaddr);
	if (err) {
		nvgpu_warn(g, "cde: failed to map compbits scatter buf at %lld size %lld",
				map_offset, map_size);
		dma_buf_put(compbits_scatter_buf);
		err = -EINVAL;
		goto exit_idle;
	}

	if (scatterbuffer_byte_offset &&
	    l->ops.cde.need_scatter_buffer &&
	    l->ops.cde.need_scatter_buffer(g)) {
		struct sg_table *sgt;
		void *scatter_buffer;

		surface = dma_buf_vmap(compbits_scatter_buf);
		if (IS_ERR(surface)) {
			nvgpu_warn(g,
				   "dma_buf_vmap failed");
			err = -EINVAL;
			goto exit_unmap_vaddr;
		}

		scatter_buffer = surface + scatterbuffer_byte_offset;

		nvgpu_log(g, gpu_dbg_cde, "surface=0x%p scatterBuffer=0x%p",
			  surface, scatter_buffer);
		sgt = gk20a_mm_pin(dev_from_gk20a(g), compbits_scatter_buf,
				   &attachment);
		if (IS_ERR(sgt)) {
			nvgpu_warn(g,
				   "mm_pin failed");
			err = -EINVAL;
			goto exit_unmap_surface;
		} else {
			err = l->ops.cde.populate_scatter_buffer(g, sgt,
					compbits_byte_offset, scatter_buffer,
					scatterbuffer_size);
			WARN_ON(err);

			gk20a_mm_unpin(dev_from_gk20a(g), compbits_scatter_buf,
				       attachment, sgt);
			if (err)
				goto exit_unmap_surface;
		}

		__cpuc_flush_dcache_area(scatter_buffer, scatterbuffer_size);
		dma_buf_vunmap(compbits_scatter_buf, surface);
		surface = NULL;
	}

	/* store source buffer compression tags */
	gk20a_get_comptags(&os_buf, &comptags);
	cde_ctx->surf_param_offset = comptags.offset;
	cde_ctx->surf_param_lines = comptags.lines;

	/* store surface vaddr. This is actually compbit vaddr, but since
	   compbits live in the same surface, and we can get the alloc base
	   address by using gpuva_to_iova_base, this will do */
	cde_ctx->surf_vaddr = map_vaddr;

	/* store information about destination */
	cde_ctx->compbit_vaddr = map_vaddr + mapped_compbits_offset;
	cde_ctx->compbit_size = compbits_size;

	cde_ctx->scatterbuffer_vaddr = map_vaddr + mapped_scatterbuffer_offset;
	cde_ctx->scatterbuffer_size = scatterbuffer_size;

	/* remove existing argument data */
	memset(cde_ctx->user_param_values, 0,
	       sizeof(cde_ctx->user_param_values));

	/* read user space arguments for the conversion */
	for (i = 0; i < num_params; i++) {
		struct gk20a_cde_param *param = params + i;
		int id = param->id - NUM_RESERVED_PARAMS;

		if (id < 0 || id >= MAX_CDE_USER_PARAMS) {
			nvgpu_warn(g, "cde: unknown user parameter");
			err = -EINVAL;
			goto exit_unmap_surface;
		}
		cde_ctx->user_param_values[id] = param->value;
	}

	/* patch data */
	err = gk20a_cde_patch_params(cde_ctx);
	if (err) {
		nvgpu_warn(g, "cde: failed to patch parameters");
		goto exit_unmap_surface;
	}

	nvgpu_log(g, gpu_dbg_cde, "cde: buffer=cbc, size=%zu, gpuva=%llx\n",
		 g->gr.compbit_store.mem.size, cde_ctx->backing_store_vaddr);
	nvgpu_log(g, gpu_dbg_cde, "cde: buffer=compbits, size=%llu, gpuva=%llx\n",
		 cde_ctx->compbit_size, cde_ctx->compbit_vaddr);
	nvgpu_log(g, gpu_dbg_cde, "cde: buffer=scatterbuffer, size=%llu, gpuva=%llx\n",
		 cde_ctx->scatterbuffer_size, cde_ctx->scatterbuffer_vaddr);

	/* take always the postfence as it is needed for protecting the
	 * cde context */
	flags = __flags | NVGPU_SUBMIT_FLAGS_FENCE_GET;

	/* gk20a_cde_execute_buffer() will grab a power reference of it's own */
	gk20a_idle(g);

	if (comptags.lines == 0) {
		/*
		 * Nothing to do on the buffer, but do a null kickoff for
		 * managing the pre and post fences.
		 */
		submit_op = TYPE_BUF_COMMAND_NOOP;
	} else if (!cde_ctx->init_cmd_executed) {
		/*
		 * First time, so include the init pushbuf too in addition to
		 * the conversion code.
		 */
		submit_op = TYPE_BUF_COMMAND_INIT;
	} else {
		/*
		 * The usual condition: execute just the conversion.
		 */
		submit_op = TYPE_BUF_COMMAND_CONVERT;
	}
	err = gk20a_cde_execute_buffer(cde_ctx, submit_op,
			fence, flags, fence_out);

	if (comptags.lines != 0 && !err)
		cde_ctx->init_cmd_executed = true;

	/* unmap the buffers - channel holds references to them now */
	nvgpu_vm_unmap(cde_ctx->vm, map_vaddr, NULL);

	return err;

exit_unmap_surface:
	if (surface)
		dma_buf_vunmap(compbits_scatter_buf, surface);
exit_unmap_vaddr:
	nvgpu_vm_unmap(cde_ctx->vm, map_vaddr, NULL);
exit_idle:
	gk20a_idle(g);
	return err;
}

static void gk20a_cde_finished_ctx_cb(struct channel_gk20a *ch, void *data)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_ctx *cde_ctx = data;
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct gk20a_cde_app *cde_app = &l->cde_app;
	bool channel_idle;

	channel_gk20a_joblist_lock(ch);
	channel_idle = channel_gk20a_joblist_is_empty(ch);
	channel_gk20a_joblist_unlock(ch);

	if (!channel_idle)
		return;

	trace_gk20a_cde_finished_ctx_cb(cde_ctx);
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: finished %p", cde_ctx);
	if (!cde_ctx->in_use)
		nvgpu_log_info(g, "double finish cde context %p on channel %p",
				cde_ctx, ch);

	if (gk20a_channel_check_timedout(ch)) {
		if (cde_ctx->is_temporary) {
			nvgpu_warn(g,
					"cde: channel had timed out"
					" (temporary channel)");
			/* going to be deleted anyway */
		} else {
			nvgpu_warn(g,
					"cde: channel had timed out"
					", reloading");
			/* mark it to be deleted, replace with a new one */
			nvgpu_mutex_acquire(&cde_app->mutex);
			cde_ctx->is_temporary = true;
			if (gk20a_cde_create_context(l)) {
				nvgpu_err(g, "cde: can't replace context");
			}
			nvgpu_mutex_release(&cde_app->mutex);
		}
	}

	/* delete temporary contexts later (watch for doubles) */
	if (cde_ctx->is_temporary && cde_ctx->in_use) {
		WARN_ON(delayed_work_pending(&cde_ctx->ctx_deleter_work));
		schedule_delayed_work(&cde_ctx->ctx_deleter_work,
			msecs_to_jiffies(CTX_DELETE_TIME));
	}

	if (!gk20a_channel_check_timedout(ch)) {
		gk20a_cde_ctx_release(cde_ctx);
	}
}

static int gk20a_cde_load(struct gk20a_cde_ctx *cde_ctx)
{
	struct nvgpu_os_linux *l = cde_ctx->l;
	struct gk20a *g = &l->g;
	struct nvgpu_firmware *img;
	struct channel_gk20a *ch;
	struct tsg_gk20a *tsg;
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_setup_bind_args setup_bind_args;
	int err = 0;
	u64 vaddr;

	img = nvgpu_request_firmware(g, "gpu2cde.bin", 0);
	if (!img) {
		nvgpu_err(g, "cde: could not fetch the firmware");
		return -ENOSYS;
	}

	tsg = gk20a_tsg_open(g, nvgpu_current_pid(g));
	if (!tsg) {
		nvgpu_err(g, "cde: could not create TSG");
		err = -ENOMEM;
		goto err_get_gk20a_channel;
	}

	ch = gk20a_open_new_channel_with_cb(g, gk20a_cde_finished_ctx_cb,
			cde_ctx,
			-1,
			false);
	if (!ch) {
		nvgpu_warn(g, "cde: gk20a channel not available");
		err = -ENOMEM;
		goto err_get_gk20a_channel;
	}

	ch->timeout.enabled = false;

	/* bind the channel to the vm */
	err = g->ops.mm.vm_bind_channel(g->mm.cde.vm, ch);
	if (err) {
		nvgpu_warn(g, "cde: could not bind vm");
		goto err_commit_va;
	}

	err = gk20a_tsg_bind_channel(tsg, ch);
	if (err) {
		nvgpu_err(g, "cde: unable to bind to tsg");
		goto err_setup_bind;
	}

	setup_bind_args.num_gpfifo_entries = 1024;
	setup_bind_args.num_inflight_jobs = 0;
	setup_bind_args.flags = 0;
	err = nvgpu_channel_setup_bind(ch, &setup_bind_args);
	if (err) {
		nvgpu_warn(g, "cde: unable to setup channel");
		goto err_setup_bind;
	}

	/* map backing store to gpu virtual space */
	vaddr = nvgpu_gmmu_map(ch->vm, &gr->compbit_store.mem,
			       g->gr.compbit_store.mem.size,
			       NVGPU_VM_MAP_CACHEABLE,
			       gk20a_mem_flag_read_only,
			       false,
			       gr->compbit_store.mem.aperture);

	if (!vaddr) {
		nvgpu_warn(g, "cde: cannot map compression bit backing store");
		err = -ENOMEM;
		goto err_map_backingstore;
	}

	/* store initialisation data */
	cde_ctx->ch = ch;
	cde_ctx->tsg = tsg;
	cde_ctx->vm = ch->vm;
	cde_ctx->backing_store_vaddr = vaddr;

	/* initialise the firmware */
	err = gk20a_init_cde_img(cde_ctx, img);
	if (err) {
		nvgpu_warn(g, "cde: image initialisation failed");
		goto err_init_cde_img;
	}

	/* initialisation done */
	nvgpu_release_firmware(g, img);

	return 0;

err_init_cde_img:
	nvgpu_gmmu_unmap(ch->vm, &g->gr.compbit_store.mem, vaddr);
err_map_backingstore:
err_setup_bind:
	nvgpu_vm_put(ch->vm);
err_commit_va:
err_get_gk20a_channel:
	nvgpu_release_firmware(g, img);
	nvgpu_err(g, "cde: couldn't initialise buffer converter: %d", err);
	return err;
}

int gk20a_cde_reload(struct nvgpu_os_linux *l)
__acquires(&l->cde_app->mutex)
__releases(&l->cde_app->mutex)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_app *cde_app = &l->cde_app;
	int err;

	if (!cde_app->initialised)
		return -ENOSYS;

	err = gk20a_busy(g);
	if (err)
		return err;

	nvgpu_mutex_acquire(&cde_app->mutex);

	gk20a_cde_stop(l);

	err = gk20a_cde_create_contexts(l);
	if (!err)
		cde_app->initialised = true;

	nvgpu_mutex_release(&cde_app->mutex);

	gk20a_idle(g);
	return err;
}

int gk20a_init_cde_support(struct nvgpu_os_linux *l)
__acquires(&cde_app->mutex)
__releases(&cde_app->mutex)
{
	struct gk20a_cde_app *cde_app = &l->cde_app;
	struct gk20a *g = &l->g;
	int err;

	if (cde_app->initialised)
		return 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_cde_ctx, "cde: init");

	err = nvgpu_mutex_init(&cde_app->mutex);
	if (err)
		return err;

	nvgpu_mutex_acquire(&cde_app->mutex);

	nvgpu_init_list_node(&cde_app->free_contexts);
	nvgpu_init_list_node(&cde_app->used_contexts);
	cde_app->ctx_count = 0;
	cde_app->ctx_count_top = 0;
	cde_app->ctx_usecount = 0;

	err = gk20a_cde_create_contexts(l);
	if (!err)
		cde_app->initialised = true;

	nvgpu_mutex_release(&cde_app->mutex);
	nvgpu_log(g, gpu_dbg_cde_ctx, "cde: init finished: %d", err);

	if (err)
		nvgpu_mutex_destroy(&cde_app->mutex);

	return err;
}

enum cde_launch_patch_id {
	PATCH_H_QMD_CTA_RASTER_WIDTH_ID     = 1024,
	PATCH_H_QMD_CTA_RASTER_HEIGHT_ID    = 1025,
	PATCH_QMD_CTA_RASTER_DEPTH_ID       = 1026, /* for firmware v0 only */
	PATCH_QMD_CTA_THREAD_DIMENSION0_ID  = 1027,
	PATCH_QMD_CTA_THREAD_DIMENSION1_ID  = 1028,
	PATCH_QMD_CTA_THREAD_DIMENSION2_ID  = 1029, /* for firmware v0 only */
	PATCH_USER_CONST_XTILES_ID          = 1030, /* for firmware v0 only */
	PATCH_USER_CONST_YTILES_ID          = 1031, /* for firmware v0 only */
	PATCH_USER_CONST_BLOCKHEIGHTLOG2_ID = 1032,
	PATCH_USER_CONST_DSTPITCH_ID        = 1033, /* for firmware v0 only */
	PATCH_H_USER_CONST_FLAGS_ID         = 1034, /* for firmware v0 only */
	PATCH_H_VPC_CURRENT_GRID_SIZE_X_ID  = 1035,
	PATCH_H_VPC_CURRENT_GRID_SIZE_Y_ID  = 1036,
	PATCH_H_VPC_CURRENT_GRID_SIZE_Z_ID  = 1037,
	PATCH_VPC_CURRENT_GROUP_SIZE_X_ID   = 1038,
	PATCH_VPC_CURRENT_GROUP_SIZE_Y_ID   = 1039,
	PATCH_VPC_CURRENT_GROUP_SIZE_Z_ID   = 1040,
	PATCH_USER_CONST_XBLOCKS_ID         = 1041,
	PATCH_H_USER_CONST_DSTOFFSET_ID     = 1042,
	PATCH_V_QMD_CTA_RASTER_WIDTH_ID     = 1043,
	PATCH_V_QMD_CTA_RASTER_HEIGHT_ID    = 1044,
	PATCH_V_USER_CONST_DSTOFFSET_ID     = 1045,
	PATCH_V_VPC_CURRENT_GRID_SIZE_X_ID  = 1046,
	PATCH_V_VPC_CURRENT_GRID_SIZE_Y_ID  = 1047,
	PATCH_V_VPC_CURRENT_GRID_SIZE_Z_ID  = 1048,
	PATCH_H_LAUNCH_WORD1_ID             = 1049,
	PATCH_H_LAUNCH_WORD2_ID             = 1050,
	PATCH_V_LAUNCH_WORD1_ID             = 1051,
	PATCH_V_LAUNCH_WORD2_ID             = 1052,
	PATCH_H_QMD_PROGRAM_OFFSET_ID       = 1053,
	PATCH_H_QMD_REGISTER_COUNT_ID       = 1054,
	PATCH_V_QMD_PROGRAM_OFFSET_ID       = 1055,
	PATCH_V_QMD_REGISTER_COUNT_ID       = 1056,
};

/* maximum number of WRITE_PATCHes in the below function */
#define MAX_CDE_LAUNCH_PATCHES		  32

static int gk20a_buffer_convert_gpu_to_cde_v1(
		struct nvgpu_os_linux *l,
		struct dma_buf *dmabuf, u32 consumer,
		u64 offset, u64 compbits_hoffset, u64 compbits_voffset,
		u64 scatterbuffer_offset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_channel_fence *fence_in,
		struct gk20a_buffer_state *state)
{
	struct gk20a *g = &l->g;
	struct gk20a_cde_param params[MAX_CDE_LAUNCH_PATCHES];
	int param = 0;
	int err = 0;
	struct gk20a_fence *new_fence = NULL;
	const int wgx = 8;
	const int wgy = 8;
	const int compbits_per_byte = 4; /* one byte stores 4 compbit pairs */
	const int xalign = compbits_per_byte * wgx;
	const int yalign = wgy;

	/* Compute per launch parameters */
	const int xtiles = (width + 7) >> 3;
	const int ytiles = (height + 7) >> 3;
	const int gridw_h = roundup(xtiles, xalign) / xalign;
	const int gridh_h = roundup(ytiles, yalign) / yalign;
	const int gridw_v = roundup(ytiles, xalign) / xalign;
	const int gridh_v = roundup(xtiles, yalign) / yalign;
	const int xblocks = (xtiles + 1) >> 1;
	const int voffset = compbits_voffset - compbits_hoffset;

	int hprog = -1;
	int vprog = -1;

	if (l->ops.cde.get_program_numbers)
		l->ops.cde.get_program_numbers(g, block_height_log2,
					       l->cde_app.shader_parameter,
					       &hprog, &vprog);
	else {
		nvgpu_warn(g, "cde: chip not supported");
		return -ENOSYS;
	}

	if (hprog < 0 || vprog < 0) {
		nvgpu_warn(g, "cde: could not determine programs");
		return -ENOSYS;
	}

	if (xtiles > 8192 / 8 || ytiles > 8192 / 8)
		nvgpu_warn(g, "cde: surface is exceptionally large (xtiles=%d, ytiles=%d)",
			   xtiles, ytiles);

	nvgpu_log(g, gpu_dbg_cde, "w=%d, h=%d, bh_log2=%d, compbits_hoffset=0x%llx, compbits_voffset=0x%llx, scatterbuffer_offset=0x%llx",
		  width, height, block_height_log2,
		  compbits_hoffset, compbits_voffset, scatterbuffer_offset);
	nvgpu_log(g, gpu_dbg_cde, "resolution (%d, %d) tiles (%d, %d)",
		  width, height, xtiles, ytiles);
	nvgpu_log(g, gpu_dbg_cde, "group (%d, %d) gridH (%d, %d) gridV (%d, %d)",
		  wgx, wgy, gridw_h, gridh_h, gridw_v, gridh_v);
	nvgpu_log(g, gpu_dbg_cde, "hprog=%d, offset=0x%x, regs=%d, vprog=%d, offset=0x%x, regs=%d",
		  hprog,
		  l->cde_app.arrays[ARRAY_PROGRAM_OFFSET][hprog],
		  l->cde_app.arrays[ARRAY_REGISTER_COUNT][hprog],
		  vprog,
		  l->cde_app.arrays[ARRAY_PROGRAM_OFFSET][vprog],
		  l->cde_app.arrays[ARRAY_REGISTER_COUNT][vprog]);

	/* Write parameters */
#define WRITE_PATCH(NAME, VALUE) \
	params[param++] = (struct gk20a_cde_param){NAME##_ID, 0, VALUE}
	WRITE_PATCH(PATCH_USER_CONST_XBLOCKS, xblocks);
	WRITE_PATCH(PATCH_USER_CONST_BLOCKHEIGHTLOG2,
		block_height_log2);
	WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION0, wgx);
	WRITE_PATCH(PATCH_QMD_CTA_THREAD_DIMENSION1, wgy);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_X, wgx);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Y, wgy);
	WRITE_PATCH(PATCH_VPC_CURRENT_GROUP_SIZE_Z, 1);

	WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_WIDTH, gridw_h);
	WRITE_PATCH(PATCH_H_QMD_CTA_RASTER_HEIGHT, gridh_h);
	WRITE_PATCH(PATCH_H_USER_CONST_DSTOFFSET, 0);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_X, gridw_h);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Y, gridh_h);
	WRITE_PATCH(PATCH_H_VPC_CURRENT_GRID_SIZE_Z, 1);

	WRITE_PATCH(PATCH_V_QMD_CTA_RASTER_WIDTH, gridw_v);
	WRITE_PATCH(PATCH_V_QMD_CTA_RASTER_HEIGHT, gridh_v);
	WRITE_PATCH(PATCH_V_USER_CONST_DSTOFFSET, voffset);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_X, gridw_v);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_Y, gridh_v);
	WRITE_PATCH(PATCH_V_VPC_CURRENT_GRID_SIZE_Z, 1);

	WRITE_PATCH(PATCH_H_QMD_PROGRAM_OFFSET,
		l->cde_app.arrays[ARRAY_PROGRAM_OFFSET][hprog]);
	WRITE_PATCH(PATCH_H_QMD_REGISTER_COUNT,
		l->cde_app.arrays[ARRAY_REGISTER_COUNT][hprog]);
	WRITE_PATCH(PATCH_V_QMD_PROGRAM_OFFSET,
		l->cde_app.arrays[ARRAY_PROGRAM_OFFSET][vprog]);
	WRITE_PATCH(PATCH_V_QMD_REGISTER_COUNT,
		l->cde_app.arrays[ARRAY_REGISTER_COUNT][vprog]);

	if (consumer & NVGPU_GPU_COMPBITS_CDEH) {
		WRITE_PATCH(PATCH_H_LAUNCH_WORD1,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][0]);
		WRITE_PATCH(PATCH_H_LAUNCH_WORD2,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][1]);
	} else {
		WRITE_PATCH(PATCH_H_LAUNCH_WORD1,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][2]);
		WRITE_PATCH(PATCH_H_LAUNCH_WORD2,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][3]);
	}

	if (consumer & NVGPU_GPU_COMPBITS_CDEV) {
		WRITE_PATCH(PATCH_V_LAUNCH_WORD1,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][0]);
		WRITE_PATCH(PATCH_V_LAUNCH_WORD2,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][1]);
	} else {
		WRITE_PATCH(PATCH_V_LAUNCH_WORD1,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][2]);
		WRITE_PATCH(PATCH_V_LAUNCH_WORD2,
			l->cde_app.arrays[ARRAY_LAUNCH_COMMAND][3]);
	}
#undef WRITE_PATCH

	err = gk20a_cde_convert(l, dmabuf,
				compbits_hoffset,
				scatterbuffer_offset,
				fence_in, submit_flags,
				params, param, &new_fence);
	if (err)
		goto out;

	/* compbits generated, update state & fence */
	gk20a_fence_put(state->fence);
	state->fence = new_fence;
	state->valid_compbits |= consumer &
		(NVGPU_GPU_COMPBITS_CDEH | NVGPU_GPU_COMPBITS_CDEV);
out:
	return err;
}

static int gk20a_buffer_convert_gpu_to_cde(
		struct nvgpu_os_linux *l, struct dma_buf *dmabuf, u32 consumer,
		u64 offset, u64 compbits_hoffset, u64 compbits_voffset,
		u64 scatterbuffer_offset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_channel_fence *fence_in,
		struct gk20a_buffer_state *state)
{
	struct gk20a *g = &l->g;
	int err = 0;

	if (!l->cde_app.initialised)
		return -ENOSYS;

	nvgpu_log(g, gpu_dbg_cde, "firmware version = %d\n",
		l->cde_app.firmware_version);

	if (l->cde_app.firmware_version == 1) {
		err = gk20a_buffer_convert_gpu_to_cde_v1(
		    l, dmabuf, consumer, offset, compbits_hoffset,
		    compbits_voffset, scatterbuffer_offset,
		    width, height, block_height_log2,
		    submit_flags, fence_in, state);
	} else {
		nvgpu_err(g, "unsupported CDE firmware version %d",
			l->cde_app.firmware_version);
		err = -EINVAL;
	}

	return err;
}

int gk20a_prepare_compressible_read(
		struct nvgpu_os_linux *l, u32 buffer_fd, u32 request, u64 offset,
		u64 compbits_hoffset, u64 compbits_voffset,
		u64 scatterbuffer_offset,
		u32 width, u32 height, u32 block_height_log2,
		u32 submit_flags, struct nvgpu_channel_fence *fence,
		u32 *valid_compbits, u32 *zbc_color,
		struct gk20a_fence **fence_out)
{
	struct gk20a *g = &l->g;
	int err = 0;
	struct gk20a_buffer_state *state;
	struct dma_buf *dmabuf;
	u32 missing_bits;

	dmabuf = dma_buf_get(buffer_fd);
	if (IS_ERR(dmabuf))
		return -EINVAL;

	err = gk20a_dmabuf_get_state(dmabuf, g, offset, &state);
	if (err) {
		dma_buf_put(dmabuf);
		return err;
	}

	missing_bits = (state->valid_compbits ^ request) & request;

	nvgpu_mutex_acquire(&state->lock);

	if (state->valid_compbits && request == NVGPU_GPU_COMPBITS_NONE) {

		gk20a_fence_put(state->fence);
		state->fence = NULL;
		/* state->fence = decompress();
		state->valid_compbits = 0; */
		err = -EINVAL;
		goto out;
	} else if (missing_bits) {
		u32 missing_cde_bits = missing_bits &
			 (NVGPU_GPU_COMPBITS_CDEH | NVGPU_GPU_COMPBITS_CDEV);
		if ((state->valid_compbits & NVGPU_GPU_COMPBITS_GPU) &&
		    missing_cde_bits) {
			err = gk20a_buffer_convert_gpu_to_cde(
					l, dmabuf,
					missing_cde_bits,
					offset, compbits_hoffset,
					compbits_voffset, scatterbuffer_offset,
					width, height, block_height_log2,
					submit_flags, fence,
					state);
			if (err)
				goto out;
		}
	}

	if (state->fence && fence_out)
		*fence_out = gk20a_fence_get(state->fence);

	if (valid_compbits)
		*valid_compbits = state->valid_compbits;

	if (zbc_color)
		*zbc_color = state->zbc_color;

out:
	nvgpu_mutex_release(&state->lock);
	dma_buf_put(dmabuf);
	return err;
}

int gk20a_mark_compressible_write(struct gk20a *g, u32 buffer_fd,
				  u32 valid_compbits, u64 offset, u32 zbc_color)
{
	int err;
	struct gk20a_buffer_state *state;
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(buffer_fd);
	if (IS_ERR(dmabuf)) {
		nvgpu_err(g, "invalid dmabuf");
		return -EINVAL;
	}

	err = gk20a_dmabuf_get_state(dmabuf, g, offset, &state);
	if (err) {
		nvgpu_err(g, "could not get state from dmabuf");
		dma_buf_put(dmabuf);
		return err;
	}

	nvgpu_mutex_acquire(&state->lock);

	/* Update the compbits state. */
	state->valid_compbits = valid_compbits;
	state->zbc_color = zbc_color;

	/* Discard previous compbit job fence. */
	gk20a_fence_put(state->fence);
	state->fence = NULL;

	nvgpu_mutex_release(&state->lock);
	dma_buf_put(dmabuf);
	return 0;
}
