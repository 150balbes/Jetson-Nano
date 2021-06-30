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

#ifndef NVGPU_VM_AREA_H
#define NVGPU_VM_AREA_H

#include <nvgpu/list.h>
#include <nvgpu/types.h>

struct vm_gk20a;
struct gk20a_as_share;
struct nvgpu_as_alloc_space_args;
struct nvgpu_as_free_space_args;

struct nvgpu_vm_area {
	/*
	 * Entry into the list of VM areas owned by a VM.
	 */
	struct nvgpu_list_node vm_area_list;

	/*
	 * List of buffers mapped into this vm_area.
	 */
	struct nvgpu_list_node buffer_list_head;

	u32 flags;
	u32 pgsz_idx;
	u64 addr;
	u64 size;
	bool sparse;
};

static inline struct nvgpu_vm_area *
nvgpu_vm_area_from_vm_area_list(struct nvgpu_list_node *node)
{
	return (struct nvgpu_vm_area *)
		((uintptr_t)node - offsetof(struct nvgpu_vm_area,
					    vm_area_list));
};

/*
 * Alloc space flags.
 */
#define NVGPU_VM_AREA_ALLOC_FIXED_OFFSET		BIT(0)
#define NVGPU_VM_AREA_ALLOC_SPARSE			BIT(1)

int nvgpu_vm_area_alloc(struct vm_gk20a *vm, u32 pages, u32 page_size,
			u64 *addr, u32 flags);
int nvgpu_vm_area_free(struct vm_gk20a *vm, u64 addr);

struct nvgpu_vm_area *nvgpu_vm_area_find(struct vm_gk20a *vm, u64 addr);
int nvgpu_vm_area_validate_buffer(struct vm_gk20a *vm,
				  u64 map_offset, u64 map_size, u32 pgsz_idx,
				  struct nvgpu_vm_area **pvm_area);

#endif /* NVGPU_VM_AREA_H */
