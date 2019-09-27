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

#ifndef NVGPU_DMA_H
#define NVGPU_DMA_H

#include <nvgpu/types.h>

struct gk20a;
struct vm_gk20a;
struct nvgpu_mem;

/*
 * Flags for the below nvgpu_dma_{alloc,alloc_map}_flags*
 */

/*
 * Don't create a virtual kernel mapping for the buffer but only allocate it;
 * this may save some resources. The buffer can be mapped later explicitly.
 */
#define NVGPU_DMA_NO_KERNEL_MAPPING	BIT32(0)

/*
 * Don't allow building the buffer from individual pages but require a
 * physically contiguous block.
 */
#define NVGPU_DMA_FORCE_CONTIGUOUS	BIT32(1)

/*
 * Make the mapping read-only.
 */
#define NVGPU_DMA_READ_ONLY		BIT32(2)

/**
 * nvgpu_iommuable - Check if GPU is behind IOMMU
 *
 * @g - The GPU.
 *
 * Returns true if the passed GPU is behind an IOMMU; false otherwise. If the
 * GPU is iommuable then the DMA address in nvgpu_mem_sgl is valid.
 *
 * Note that even if a GPU is behind an IOMMU that does not necessarily mean the
 * GPU _must_ use DMA addresses. GPUs may still use physical addresses if it
 * makes sense.
 */
bool nvgpu_iommuable(struct gk20a *g);

/**
 * nvgpu_dma_alloc - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * memory can be either placed in VIDMEM or SYSMEM, which ever is more
 * convenient for the driver.
 */
int nvgpu_dma_alloc(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_flags - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * memory can be either placed in VIDMEM or SYSMEM, which ever is more
 * convenient for the driver.
 *
 * The following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int nvgpu_dma_alloc_flags(struct gk20a *g, unsigned long flags, size_t size,
		struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_sys - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in SYSMEM.
 */
int nvgpu_dma_alloc_sys(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_flags_sys - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in SYSMEM.
 *
 * The following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int nvgpu_dma_alloc_flags_sys(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_vid - Allocate DMA memory
 *
 * @g    - The GPU.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 */
int nvgpu_dma_alloc_vid(struct gk20a *g, size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_flags_vid - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 *
 * Only the following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *
 */
int nvgpu_dma_alloc_flags_vid(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);


/**
 * nvgpu_dma_alloc_flags_vid_at - Allocate DMA memory
 *
 * @g     - The GPU.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 * @at    - A specific location to attempt to allocate memory from or 0 if the
 *          caller does not care what the address is.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 *
 */
int nvgpu_dma_alloc_vid_at(struct gk20a *g,
		size_t size, struct nvgpu_mem *mem, u64 at);

/**
 * nvgpu_dma_alloc_flags_vid_at - Allocate DMA memory
 *
 * @g     - The GPU.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 * @at    - A specific location to attempt to allocate memory from or 0 if the
 *          caller does not care what the address is.
 *
 * Allocate memory suitable for doing DMA. Store the allocation info in @mem.
 * Returns 0 on success and a suitable error code when there's an error. This
 * allocates memory specifically in VIDMEM.
 *
 * Only the following flags are accepted:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 */
int nvgpu_dma_alloc_flags_vid_at(struct gk20a *g, unsigned long flags,
		size_t size, struct nvgpu_mem *mem, u64 at);

/**
 * nvgpu_dma_free - Free a DMA allocation
 *
 * @g   - The GPU.
 * @mem - An allocation to free.
 *
 * Free memory created with any of:
 *
 *   nvgpu_dma_alloc()
 *   nvgpu_dma_alloc_flags()
 *   nvgpu_dma_alloc_sys()
 *   nvgpu_dma_alloc_flags_sys()
 *   nvgpu_dma_alloc_vid()
 *   nvgpu_dma_alloc_flags_vid()
 *   nvgpu_dma_alloc_flags_vid_at()
 */
void nvgpu_dma_free(struct gk20a *g, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * Note this is different than mapping it into the CPU. This memory can be
 * either placed in VIDMEM or SYSMEM, which ever is more convenient for the
 * driver.
 *
 * Note: currently a bug exists in the nvgpu_dma_alloc_map*() routines: you
 * cannot use nvgpu_gmmu_map() on said buffer - it will overwrite the necessary
 * information for the DMA unmap routines to actually unmap the buffer. You
 * will either leak mappings or see GMMU faults.
 */
int nvgpu_dma_alloc_map(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map_flags - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * Note this is different than mapping it into the CPU. This memory can be
 * either placed in VIDMEM or SYSMEM, which ever is more convenient for the
 * driver.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int nvgpu_dma_alloc_map_flags(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map_sys - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in SYSMEM.
 */
int nvgpu_dma_alloc_map_sys(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map_flags_sys - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in SYSMEM.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int nvgpu_dma_alloc_map_flags_sys(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map_vid - Allocate DMA memory and map into GMMU.
 *
 * @vm   - VM context for GMMU mapping.
 * @size - Size of the allocation in bytes.
 * @mem  - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in VIDMEM.
 */
int nvgpu_dma_alloc_map_vid(struct vm_gk20a *vm, size_t size,
		struct nvgpu_mem *mem);

/**
 * nvgpu_dma_alloc_map_flags_vid - Allocate DMA memory and map into GMMU.
 *
 * @vm    - VM context for GMMU mapping.
 * @flags - Flags modifying the operation of the DMA allocation.
 * @size  - Size of the allocation in bytes.
 * @mem   - Struct for storing the allocation information.
 *
 * Allocate memory suitable for doing DMA and map that memory into the GMMU.
 * This memory will be placed in VIDMEM.
 *
 * This version passes @flags on to the underlying DMA allocation. The accepted
 * flags are:
 *
 *   %NVGPU_DMA_NO_KERNEL_MAPPING
 *   %NVGPU_DMA_FORCE_CONTIGUOUS
 *   %NVGPU_DMA_READ_ONLY
 */
int nvgpu_dma_alloc_map_flags_vid(struct vm_gk20a *vm, unsigned long flags,
		size_t size, struct nvgpu_mem *mem);

/**
 * nvgpu_dma_unmap_free - Free a DMA allocation
 *
 * @g   - The GPU.
 * @mem - An allocation to free.
 *
 * Free memory created with any of:
 *
 *   nvgpu_dma_alloc_map()
 *   nvgpu_dma_alloc_map_flags()
 *   nvgpu_dma_alloc_map_sys()
 *   nvgpu_dma_alloc_map_flags_sys()
 *   nvgpu_dma_alloc_map_vid()
 *   nvgpu_dma_alloc_map_flags_vid()
 */
void nvgpu_dma_unmap_free(struct vm_gk20a *vm, struct nvgpu_mem *mem);

/*
 * Don't use these directly. Instead use nvgpu_dma_free().
 */
void nvgpu_dma_free_sys(struct gk20a *g, struct nvgpu_mem *mem);
void nvgpu_dma_free_vid(struct gk20a *g, struct nvgpu_mem *mem);

#endif /*  NVGPU_DMA_H */
