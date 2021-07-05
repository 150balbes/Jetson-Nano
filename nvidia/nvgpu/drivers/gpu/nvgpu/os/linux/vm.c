/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/log.h>
#include <nvgpu/lock.h>
#include <nvgpu/rbtree.h>
#include <nvgpu/vm_area.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/page_allocator.h>
#include <nvgpu/vidmem.h>
#include <nvgpu/utils.h>
#include <nvgpu/gk20a.h>

#include <nvgpu/linux/vm.h>
#include <nvgpu/linux/nvgpu_mem.h>

#include "gk20a/mm_gk20a.h"

#include "platform_gk20a.h"
#include "os_linux.h"
#include "dmabuf.h"
#include "dmabuf_vidmem.h"

static u32 nvgpu_vm_translate_linux_flags(struct gk20a *g, u32 flags)
{
	u32 core_flags = 0;

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)
		core_flags |= NVGPU_VM_MAP_FIXED_OFFSET;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_CACHEABLE)
		core_flags |= NVGPU_VM_MAP_CACHEABLE;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_IO_COHERENT)
		core_flags |= NVGPU_VM_MAP_IO_COHERENT;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_UNMAPPED_PTE)
		core_flags |= NVGPU_VM_MAP_UNMAPPED_PTE;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_L3_ALLOC)
		core_flags |= NVGPU_VM_MAP_L3_ALLOC;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL)
		core_flags |= NVGPU_VM_MAP_DIRECT_KIND_CTRL;
	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_PLATFORM_ATOMIC)
		core_flags |= NVGPU_VM_MAP_PLATFORM_ATOMIC;

	if (flags & NVGPU_AS_MAP_BUFFER_FLAGS_MAPPABLE_COMPBITS)
		nvgpu_warn(g, "Ignoring deprecated flag: "
			   "NVGPU_AS_MAP_BUFFER_FLAGS_MAPPABLE_COMPBITS");

	return core_flags;
}

static struct nvgpu_mapped_buf *__nvgpu_vm_find_mapped_buf_reverse(
	struct vm_gk20a *vm, struct dma_buf *dmabuf, u32 kind)
{
	struct nvgpu_rbtree_node *node = NULL;
	struct nvgpu_rbtree_node *root = vm->mapped_buffers;

	nvgpu_rbtree_enum_start(0, &node, root);

	while (node) {
		struct nvgpu_mapped_buf *mapped_buffer =
				mapped_buffer_from_rbtree_node(node);

		if (mapped_buffer->os_priv.dmabuf == dmabuf &&
		    mapped_buffer->kind == kind)
			return mapped_buffer;

		nvgpu_rbtree_enum_next(&node, node);
	}

	return NULL;
}

int nvgpu_vm_find_buf(struct vm_gk20a *vm, u64 gpu_va,
		      struct dma_buf **dmabuf,
		      u64 *offset)
{
	struct nvgpu_mapped_buf *mapped_buffer;
	struct gk20a *g = gk20a_from_vm(vm);

	nvgpu_log_fn(g, "gpu_va=0x%llx", gpu_va);

	nvgpu_mutex_acquire(&vm->update_gmmu_lock);

	mapped_buffer = __nvgpu_vm_find_mapped_buf_range(vm, gpu_va);
	if (!mapped_buffer) {
		nvgpu_mutex_release(&vm->update_gmmu_lock);
		return -EINVAL;
	}

	*dmabuf = mapped_buffer->os_priv.dmabuf;
	*offset = gpu_va - mapped_buffer->addr;

	nvgpu_mutex_release(&vm->update_gmmu_lock);

	return 0;
}

u64 nvgpu_os_buf_get_size(struct nvgpu_os_buffer *os_buf)
{
	return os_buf->dmabuf->size;
}

/*
 * vm->update_gmmu_lock must be held. This checks to see if we already have
 * mapped the passed buffer into this VM. If so, just return the existing
 * mapping address.
 */
struct nvgpu_mapped_buf *nvgpu_vm_find_mapping(struct vm_gk20a *vm,
					       struct nvgpu_os_buffer *os_buf,
					       u64 map_addr,
					       u32 flags,
					       int kind)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct nvgpu_mapped_buf *mapped_buffer = NULL;

	if (flags & NVGPU_VM_MAP_FIXED_OFFSET) {
		mapped_buffer = __nvgpu_vm_find_mapped_buf(vm, map_addr);
		if (!mapped_buffer)
			return NULL;

		if (mapped_buffer->os_priv.dmabuf != os_buf->dmabuf ||
		    mapped_buffer->kind != (u32)kind)
			return NULL;
	} else {
		mapped_buffer =
			__nvgpu_vm_find_mapped_buf_reverse(vm,
							   os_buf->dmabuf,
							   kind);
		if (!mapped_buffer)
			return NULL;
	}

	if (mapped_buffer->flags != flags)
		return NULL;

	/*
	 * If we find the mapping here then that means we have mapped it already
	 * and the prior pin and get must be undone.
	 */
	gk20a_mm_unpin(os_buf->dev, os_buf->dmabuf, os_buf->attachment,
		       mapped_buffer->os_priv.sgt);
	dma_buf_put(os_buf->dmabuf);

	nvgpu_log(g, gpu_dbg_map,
		  "gv: 0x%04x_%08x + 0x%-7zu "
		  "[dma: 0x%010llx, pa: 0x%010llx] "
		  "pgsz=%-3dKb as=%-2d "
		  "flags=0x%x apt=%s (reused)",
		  u64_hi32(mapped_buffer->addr), u64_lo32(mapped_buffer->addr),
		  os_buf->dmabuf->size,
		  (u64)sg_dma_address(mapped_buffer->os_priv.sgt->sgl),
		  (u64)sg_phys(mapped_buffer->os_priv.sgt->sgl),
		  vm->gmmu_page_sizes[mapped_buffer->pgsz_idx] >> 10,
		  vm_aspace_id(vm),
		  mapped_buffer->flags,
		  nvgpu_aperture_str(g,
				     gk20a_dmabuf_aperture(g, os_buf->dmabuf)));

	return mapped_buffer;
}

int nvgpu_vm_map_linux(struct vm_gk20a *vm,
		       struct dma_buf *dmabuf,
		       u64 map_addr,
		       u32 flags,
		       u32 page_size,
		       s16 compr_kind,
		       s16 incompr_kind,
		       int rw_flag,
		       u64 buffer_offset,
		       u64 mapping_size,
		       struct vm_gk20a_mapping_batch *batch,
		       u64 *gpu_va)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct device *dev = dev_from_gk20a(g);
	struct nvgpu_os_buffer os_buf;
	struct sg_table *sgt;
	struct nvgpu_sgt *nvgpu_sgt = NULL;
	struct nvgpu_mapped_buf *mapped_buffer = NULL;
	struct dma_buf_attachment *attachment;
	int err = 0;

	sgt = gk20a_mm_pin(dev, dmabuf, &attachment);
	if (IS_ERR(sgt)) {
		nvgpu_warn(g, "Failed to pin dma_buf!");
		return PTR_ERR(sgt);
	}
	os_buf.dmabuf = dmabuf;
	os_buf.attachment = attachment;
	os_buf.dev = dev;

	if (gk20a_dmabuf_aperture(g, dmabuf) == APERTURE_INVALID) {
		err = -EINVAL;
		goto clean_up;
	}

	nvgpu_sgt = nvgpu_linux_sgt_create(g, sgt);
	if (!nvgpu_sgt) {
		err = -ENOMEM;
		goto clean_up;
	}

	mapped_buffer = nvgpu_vm_map(vm,
				     &os_buf,
				     nvgpu_sgt,
				     map_addr,
				     mapping_size,
				     buffer_offset,
				     rw_flag,
				     flags,
				     compr_kind,
				     incompr_kind,
				     batch,
				     gk20a_dmabuf_aperture(g, dmabuf));

	nvgpu_sgt_free(g, nvgpu_sgt);

	if (IS_ERR(mapped_buffer)) {
		err = PTR_ERR(mapped_buffer);
		goto clean_up;
	}

	mapped_buffer->os_priv.dmabuf = dmabuf;
	mapped_buffer->os_priv.attachment = attachment;
	mapped_buffer->os_priv.sgt    = sgt;

	*gpu_va = mapped_buffer->addr;
	return 0;

clean_up:
	gk20a_mm_unpin(dev, dmabuf, attachment, sgt);

	return err;
}

int nvgpu_vm_map_buffer(struct vm_gk20a *vm,
			int dmabuf_fd,
			u64 *map_addr,
			u32 flags, /*NVGPU_AS_MAP_BUFFER_FLAGS_*/
			u32 page_size,
			s16 compr_kind,
			s16 incompr_kind,
			u64 buffer_offset,
			u64 mapping_size,
			struct vm_gk20a_mapping_batch *batch)
{
	struct gk20a *g = gk20a_from_vm(vm);
	struct dma_buf *dmabuf;
	u64 ret_va;
	int err = 0;

	/* get ref to the mem handle (released on unmap_locked) */
	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		nvgpu_warn(g, "%s: fd %d is not a dmabuf",
			   __func__, dmabuf_fd);
		return PTR_ERR(dmabuf);
	}

	/*
	 * For regular maps we do not accept either an input address or a
	 * buffer_offset.
	 */
	if (!(flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET) &&
	    (buffer_offset || *map_addr)) {
		nvgpu_err(g,
			  "Regular map with addr/buf offset is not supported!");
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	/*
	 * Map size is always buffer size for non fixed mappings. As such map
	 * size should be left as zero by userspace for non-fixed maps.
	 */
	if (mapping_size && !(flags & NVGPU_AS_MAP_BUFFER_FLAGS_FIXED_OFFSET)) {
		nvgpu_err(g, "map_size && non-fixed-mapping!");
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	/* verify that we're not overflowing the buffer, i.e.
	 * (buffer_offset + mapping_size) > dmabuf->size.
	 *
	 * Since buffer_offset + mapping_size could overflow, first check
	 * that mapping size < dmabuf_size, at which point we can subtract
	 * mapping_size from both sides for the final comparison.
	 */
	if ((mapping_size > dmabuf->size) ||
			(buffer_offset > (dmabuf->size - mapping_size))) {
		nvgpu_err(g,
			  "buf size %llx < (offset(%llx) + map_size(%llx))",
			  (u64)dmabuf->size, buffer_offset, mapping_size);
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev_from_vm(vm));
	if (err) {
		dma_buf_put(dmabuf);
		return err;
	}

	err = nvgpu_vm_map_linux(vm, dmabuf, *map_addr,
				 nvgpu_vm_translate_linux_flags(g, flags),
				 page_size,
				 compr_kind, incompr_kind,
				 gk20a_mem_flag_none,
				 buffer_offset,
				 mapping_size,
				 batch,
				 &ret_va);

	if (!err)
		*map_addr = ret_va;
	else
		dma_buf_put(dmabuf);

	return err;
}

/*
 * This is the function call-back for freeing OS specific components of an
 * nvgpu_mapped_buf. This should most likely never be called outside of the
 * core MM framework!
 *
 * Note: the VM lock will be held.
 */
void nvgpu_vm_unmap_system(struct nvgpu_mapped_buf *mapped_buffer)
{
	struct vm_gk20a *vm = mapped_buffer->vm;

	gk20a_mm_unpin(dev_from_vm(vm), mapped_buffer->os_priv.dmabuf,
		       mapped_buffer->os_priv.attachment,
		       mapped_buffer->os_priv.sgt);

	dma_buf_put(mapped_buffer->os_priv.dmabuf);
}
