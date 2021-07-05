/*
 * Virtualized GPU Memory Management
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include "vgpu_mm_gp10b.h"
#include "gk20a/mm_gk20a.h"

#include <nvgpu/bug.h>
#include <nvgpu/dma.h>
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/vgpu.h>

int vgpu_gp10b_init_mm_setup_hw(struct gk20a *g)
{
	g->mm.disable_bigpage = true;
	return 0;
}

static inline int add_mem_desc(struct tegra_vgpu_mem_desc *mem_desc,
				u64 addr, u64 size, size_t *oob_size)
{
	if (*oob_size < sizeof(*mem_desc))
		return -ENOMEM;

	mem_desc->addr = addr;
	mem_desc->length = size;
	*oob_size -= sizeof(*mem_desc);
	return 0;
}

u64 vgpu_gp10b_locked_gmmu_map(struct vm_gk20a *vm,
				u64 map_offset,
				struct nvgpu_sgt *sgt,
				u64 buffer_offset,
				u64 size,
				u32 pgsz_idx,
				u8 kind_v,
				u32 ctag_offset,
				u32 flags,
				enum gk20a_mem_rw_flag rw_flag,
				bool clear_ctags,
				bool sparse,
				bool priv,
				struct vm_gk20a_mapping_batch *batch,
				enum nvgpu_aperture aperture)
{
	int err = 0;
	struct gk20a *g = gk20a_from_vm(vm);
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_as_map_ex_params *p = &msg.params.as_map_ex;
	struct tegra_vgpu_mem_desc *mem_desc;
	u32 page_size  = vm->gmmu_page_sizes[pgsz_idx];
	u64 buffer_size = PAGE_ALIGN(size);
	u64 space_to_skip = buffer_offset;
	u32 mem_desc_count = 0, i;
	void *handle = NULL;
	size_t oob_size;
	u8 prot;
	struct nvgpu_sgl *sgl;

	nvgpu_log_fn(g, " ");

	/* FIXME: add support for sparse mappings */

	if (WARN_ON(!sgt) || WARN_ON(nvgpu_iommuable(g)))
		return 0;

	if (space_to_skip & (page_size - 1))
		return 0;

	memset(&msg, 0, sizeof(msg));

	/* Allocate (or validate when map_offset != 0) the virtual address. */
	if (!map_offset) {
		map_offset = __nvgpu_vm_alloc_va(vm, size, pgsz_idx);
		if (!map_offset) {
			nvgpu_err(g, "failed to allocate va space");
			err = -ENOMEM;
			goto fail;
		}
	}

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
					TEGRA_VGPU_QUEUE_CMD,
					(void **)&mem_desc, &oob_size);
	if (!handle) {
		err = -EINVAL;
		goto fail;
	}
	sgl = sgt->sgl;

	/* Align size to page size */
	size = ALIGN(size, page_size);

	while (sgl) {
		u64 phys_addr;
		u64 chunk_length;

		/*
		 * Cut out sgl ents for space_to_skip.
		 */
		if (space_to_skip &&
		    space_to_skip >= nvgpu_sgt_get_length(sgt, sgl)) {
			space_to_skip -= nvgpu_sgt_get_length(sgt, sgl);
			sgl = nvgpu_sgt_get_next(sgt, sgl);
			continue;
		}

		phys_addr = nvgpu_sgt_get_phys(g, sgt, sgl) + space_to_skip;
		chunk_length = min(size,
			   nvgpu_sgt_get_length(sgt, sgl) - space_to_skip);

		if (add_mem_desc(&mem_desc[mem_desc_count++], phys_addr,
				 chunk_length, &oob_size)) {
			err = -ENOMEM;
			goto fail;
		}

		space_to_skip = 0;
		size -= chunk_length;
		sgl   = nvgpu_sgt_get_next(sgt, sgl);

		if (size == 0)
			break;
	}

	if (rw_flag == gk20a_mem_flag_read_only)
		prot = TEGRA_VGPU_MAP_PROT_READ_ONLY;
	else if (rw_flag == gk20a_mem_flag_write_only)
		prot = TEGRA_VGPU_MAP_PROT_WRITE_ONLY;
	else
		prot = TEGRA_VGPU_MAP_PROT_NONE;

	if (pgsz_idx == GMMU_PAGE_SIZE_KERNEL) {
		if (page_size == vm->gmmu_page_sizes[GMMU_PAGE_SIZE_SMALL]) {
			pgsz_idx = GMMU_PAGE_SIZE_SMALL;
		} else if (page_size ==
				vm->gmmu_page_sizes[GMMU_PAGE_SIZE_BIG]) {
			pgsz_idx = GMMU_PAGE_SIZE_BIG;
		} else {
			nvgpu_err(g, "invalid kernel page size %d",
				page_size);
			goto fail;
		}
	}

	msg.cmd = TEGRA_VGPU_CMD_AS_MAP_EX;
	msg.handle = vgpu_get_handle(g);
	p->handle = vm->handle;
	p->gpu_va = map_offset;
	p->size = buffer_size;
	p->mem_desc_count = mem_desc_count;
	p->pgsz_idx = pgsz_idx;
	p->iova = 0;
	p->kind = kind_v;
	if (flags & NVGPU_VM_MAP_CACHEABLE)
		p->flags = TEGRA_VGPU_MAP_CACHEABLE;
	if (flags & NVGPU_VM_MAP_IO_COHERENT)
		p->flags |= TEGRA_VGPU_MAP_IO_COHERENT;
	if (flags & NVGPU_VM_MAP_L3_ALLOC)
		p->flags |= TEGRA_VGPU_MAP_L3_ALLOC;
	if (flags & NVGPU_VM_MAP_PLATFORM_ATOMIC) {
		p->flags |= TEGRA_VGPU_MAP_PLATFORM_ATOMIC;
	}

	p->prot = prot;
	p->ctag_offset = ctag_offset;
	p->clear_ctags = clear_ctags;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	if (err || msg.ret)
		goto fail;

	/* TLB invalidate handled on server side */

	vgpu_ivc_oob_put_ptr(handle);
	return map_offset;
fail:
	if (handle)
		vgpu_ivc_oob_put_ptr(handle);
	nvgpu_err(g, "Failed: err=%d, msg.ret=%d", err, msg.ret);
	nvgpu_err(g,
		  "  Map: %-5s GPU virt %#-12llx +%#-9llx "
		  "phys offset: %#-4llx;  pgsz: %3dkb perm=%-2s | "
		  "kind=%#02x APT=%-6s",
		  vm->name, map_offset, buffer_size, buffer_offset,
		  vm->gmmu_page_sizes[pgsz_idx] >> 10,
		  nvgpu_gmmu_perm_str(rw_flag),
		  kind_v, "SYSMEM");
	for (i = 0; i < mem_desc_count; i++)
		nvgpu_err(g, "  > 0x%010llx + 0x%llx",
			  mem_desc[i].addr, mem_desc[i].length);

	return 0;
}
