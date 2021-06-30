/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __COMMON_LINUX_VM_PRIV_H__
#define __COMMON_LINUX_VM_PRIV_H__

#include <nvgpu/types.h>

#include <asm/cacheflush.h>

/*
 * Couple of places explicitly flush caches still. Any DMA buffer we allocate
 * from within the GPU is writecombine and as a result does not need this but
 * there seem to be exceptions.
 */
#ifdef CONFIG_ARM64
#define outer_flush_range(a, b)
#define __cpuc_flush_dcache_area __flush_dcache_area
#endif

struct sg_table;
struct dma_buf;
struct device;

struct vm_gk20a;
struct vm_gk20a_mapping_batch;
struct nvgpu_vm_area;

struct nvgpu_os_buffer {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct device *dev;
};

struct nvgpu_mapped_buf_priv {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
};

/* NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL must be set */
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
		       struct vm_gk20a_mapping_batch *mapping_batch,
		       u64 *gpu_va);

/*
 * Notes:
 * - Batch may be NULL if map op is not part of a batch.
 * - NVGPU_AS_MAP_BUFFER_FLAGS_DIRECT_KIND_CTRL must be set
 */
int nvgpu_vm_map_buffer(struct vm_gk20a *vm,
			int dmabuf_fd,
			u64 *map_addr,
			u32 flags, /* NVGPU_AS_MAP_BUFFER_FLAGS_ */
			u32 page_size,
			s16 compr_kind,
			s16 incompr_kind,
			u64 buffer_offset,
			u64 mapping_size,
			struct vm_gk20a_mapping_batch *batch);

/* find buffer corresponding to va */
int nvgpu_vm_find_buf(struct vm_gk20a *vm, u64 gpu_va,
		      struct dma_buf **dmabuf,
		      u64 *offset);

enum nvgpu_aperture gk20a_dmabuf_aperture(struct gk20a *g,
					  struct dma_buf *dmabuf);

#endif
