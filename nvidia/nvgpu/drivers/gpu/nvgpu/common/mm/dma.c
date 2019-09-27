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

#include <nvgpu/mm.h>
#include <nvgpu/vm.h>
#include <nvgpu/dma.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/enabled.h>
#include <nvgpu/nvgpu_mem.h>

int nvgpu_dma_alloc(struct gk20a *g, size_t size, struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_flags(g, 0, size, mem);
}

int nvgpu_dma_alloc_flags(struct gk20a *g, unsigned long flags, size_t size,
		struct nvgpu_mem *mem)
{
	if (!nvgpu_is_enabled(g, NVGPU_MM_UNIFIED_MEMORY)) {
		/*
		 * Force the no-kernel-mapping flag on because we don't support
		 * the lack of it for vidmem - the user should not care when
		 * using nvgpu_gmmu_alloc_map and it's vidmem, or if there's a
		 * difference, the user should use the flag explicitly anyway.
		 *
		 * Incoming flags are ignored here, since bits other than the
		 * no-kernel-mapping flag are ignored by the vidmem mapping
		 * functions anyway.
		 */
		int err = nvgpu_dma_alloc_flags_vid(g,
				NVGPU_DMA_NO_KERNEL_MAPPING,
				size, mem);

		if (!err) {
			return 0;
		}

		/*
		 * Fall back to sysmem (which may then also fail) in case
		 * vidmem is exhausted.
		 */
	}

	return nvgpu_dma_alloc_flags_sys(g, flags, size, mem);
}

int nvgpu_dma_alloc_sys(struct gk20a *g, size_t size, struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_flags_sys(g, 0, size, mem);
}

int nvgpu_dma_alloc_vid(struct gk20a *g, size_t size, struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_flags_vid(g,
			NVGPU_DMA_NO_KERNEL_MAPPING, size, mem);
}

int nvgpu_dma_alloc_flags_vid(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_flags_vid_at(g, flags, size, mem, 0);
}

int nvgpu_dma_alloc_vid_at(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem, u64 at)
{
	return nvgpu_dma_alloc_flags_vid_at(g,
			NVGPU_DMA_NO_KERNEL_MAPPING, size, mem, at);
}

int nvgpu_dma_alloc_map(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_map_flags(vm, 0, size, mem);
}

int nvgpu_dma_alloc_map_flags(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem)
{
	if (!nvgpu_is_enabled(gk20a_from_vm(vm), NVGPU_MM_UNIFIED_MEMORY)) {
		/*
		 * Force the no-kernel-mapping flag on because we don't support
		 * the lack of it for vidmem - the user should not care when
		 * using nvgpu_dma_alloc_map and it's vidmem, or if there's a
		 * difference, the user should use the flag explicitly anyway.
		 */
		int err = nvgpu_dma_alloc_map_flags_vid(vm,
				flags | NVGPU_DMA_NO_KERNEL_MAPPING,
				size, mem);

		if (!err) {
			return 0;
		}

		/*
		 * Fall back to sysmem (which may then also fail) in case
		 * vidmem is exhausted.
		 */
	}

	return nvgpu_dma_alloc_map_flags_sys(vm, flags, size, mem);
}

int nvgpu_dma_alloc_map_sys(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_map_flags_sys(vm, 0, size, mem);
}

int nvgpu_dma_alloc_map_flags_sys(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem)
{
	int err = nvgpu_dma_alloc_flags_sys(vm->mm->g, flags, size, mem);

	if (err) {
		return err;
	}

	mem->gpu_va = nvgpu_gmmu_map(vm, mem, size, 0,
				     gk20a_mem_flag_none, false,
				     mem->aperture);
	if (!mem->gpu_va) {
		err = -ENOMEM;
		goto fail_free;
	}

	return 0;

fail_free:
	nvgpu_dma_free(vm->mm->g, mem);
	return err;
}

int nvgpu_dma_alloc_map_vid(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem)
{
	return nvgpu_dma_alloc_map_flags_vid(vm,
			NVGPU_DMA_NO_KERNEL_MAPPING, size, mem);
}

int nvgpu_dma_alloc_map_flags_vid(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem)
{
	int err = nvgpu_dma_alloc_flags_vid(vm->mm->g, flags, size, mem);

	if (err) {
		return err;
	}

	mem->gpu_va = nvgpu_gmmu_map(vm, mem, size, 0,
				     gk20a_mem_flag_none, false,
				     mem->aperture);
	if (!mem->gpu_va) {
		err = -ENOMEM;
		goto fail_free;
	}

	return 0;

fail_free:
	nvgpu_dma_free(vm->mm->g, mem);
	return err;
}

void nvgpu_dma_free(struct gk20a *g, struct nvgpu_mem *mem)
{
	switch (mem->aperture) {
	case APERTURE_SYSMEM:
		nvgpu_dma_free_sys(g, mem);
		break;
	case APERTURE_VIDMEM:
		nvgpu_dma_free_vid(g, mem);
		break;
	default:
		break; /* like free() on "null" memory */
	}
}

void nvgpu_dma_unmap_free(struct vm_gk20a *vm, struct nvgpu_mem *mem)
{
	if (mem->gpu_va) {
		nvgpu_gmmu_unmap(vm, mem, mem->gpu_va);
	}
	mem->gpu_va = 0;

	nvgpu_dma_free(vm->mm->g, mem);
}
