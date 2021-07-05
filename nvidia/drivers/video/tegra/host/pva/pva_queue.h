/*
 * PVA Task Management
 *
 * Copyright (c) 2016-2019, NVIDIA Corporation.  All rights reserved.
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

#ifndef PVA_QUEUE_H
#define PVA_QUEUE_H

#include <uapi/linux/nvhost_pva_ioctl.h>
#include <uapi/linux/nvdev_fence.h>

#include "nvhost_queue.h"
#include "nvhost_buffer.h"

#include "pva-interface.h"

struct dma_buf;

extern struct nvhost_queue_ops pva_queue_ops;

struct pva_parameter_ext {
	dma_addr_t dma_addr;
	size_t size;
	struct dma_buf *dmabuf;
	enum nvhost_buffers_heap heap;
};

/**
 * @brief	Describe a task for PVA
 *
 * This is an internal representation of the task structure. All
 * pointers refer to kernel memory.
 *
 * pva				Pointer to struct pva
 * buffers			Pointer to struct nvhost_buffers
 * queue			Pointer to struct nvhost_queue
 * node				Used to build queue task list
 * kref				Used to manage allocation and freeing
 * dma_addr			task dma_addr_t
 * va				task virtual address
 * pool_index			task pool index
 * postfence_va			postfence virtual address
 * num_prefences		Number of pre-fences in this task
 * num_postfences		Number of post-fences in this task
 * num_input_surfaces		Number of input surfaces
 * num_output_surfaces		Number of output surfaces
 * num_input_task_status	Number of input task status structures
 * num_output_task_status	Number of output task status structures
 * operation			task operation
 * timeout			Latest Unix time when the task must complete or
 *				0 if disabled.
 * prefences			Pre-fence structures
 * postfences			Post-fence structures
 * input_surfaces		Input surfaces structures
 * input_scalars		Information for input scalars
 * output_surfaces		Output surfaces
 * output_scalars		Information for output scalars
 * input_task_status		Input status structure
 * output_task_status		Output status structure
 *
 */
struct pva_submit_task {
	struct pva *pva;
	struct nvhost_buffers *buffers;
	struct nvhost_queue *queue;

	struct list_head node;
	struct kref ref;

	dma_addr_t dma_addr;
	void *va;
	int pool_index;

	u8 num_prefences;
	u8 num_postfences;
	u8 num_input_surfaces;
	u8 num_output_surfaces;
	u8 num_input_task_status;
	u8 num_output_task_status;
	u32 primary_payload_size;
	u8 num_pointers;
	u32 operation;
	u64 timeout;
	bool invalid;
	u32 syncpt_thresh;
	u32 fence_num;

	/* Data provided by userspace "as is" */
	struct nvdev_fence prefences[PVA_MAX_PREFENCES];
	struct nvdev_fence postfences[PVA_MAX_POSTFENCES];
	struct nvpva_fence pvafences[PVA_MAX_FENCE_TYPES]
		[PVA_MAX_FENCES_PER_TYPE];
	struct pva_surface input_surfaces[PVA_MAX_INPUT_SURFACES];
	struct pva_task_parameter input_scalars;
	struct pva_surface output_surfaces[PVA_MAX_OUTPUT_SURFACES];
	struct pva_task_parameter output_scalars;
	struct pva_status_handle input_task_status[PVA_MAX_INPUT_STATUS];
	struct pva_status_handle output_task_status[PVA_MAX_OUTPUT_STATUS];
	struct pva_memory_handle pointers[PVA_MAX_POINTERS];
	u8 primary_payload[PVA_MAX_PRIMARY_PAYLOAD_SIZE];
	u8 num_pvafences[PVA_MAX_FENCE_TYPES];
	u8 num_pva_ts_buffers[PVA_MAX_FENCE_TYPES];

	/* External data that is added by the KMD */
	struct pva_parameter_ext prefences_ext[PVA_MAX_PREFENCES];
	struct pva_parameter_ext postfences_ext[PVA_MAX_POSTFENCES];
	struct pva_parameter_ext pvafences_ext[PVA_MAX_FENCE_TYPES]
		[PVA_MAX_FENCES_PER_TYPE];
	struct pva_parameter_ext prefences_sema_ext[PVA_MAX_PREFENCES];
	struct pva_parameter_ext postfences_sema_ext[PVA_MAX_POSTFENCES];
	struct pva_parameter_ext pvafences_sema_ext[PVA_MAX_FENCE_TYPES]
		[PVA_MAX_FENCES_PER_TYPE];
	struct pva_parameter_ext input_surfaces_ext[PVA_MAX_INPUT_SURFACES];
	struct pva_parameter_ext input_scalars_ext;
	struct pva_parameter_ext output_surfaces_ext[PVA_MAX_OUTPUT_SURFACES];
	struct pva_parameter_ext output_scalars_ext;
	struct pva_parameter_ext input_task_status_ext[PVA_MAX_INPUT_STATUS];
	struct pva_parameter_ext output_task_status_ext[PVA_MAX_OUTPUT_STATUS];
	struct pva_parameter_ext
			input_surface_rois_ext[PVA_MAX_INPUT_SURFACES];
	struct pva_parameter_ext
			output_surface_rois_ext[PVA_MAX_OUTPUT_SURFACES];
	struct pva_parameter_ext pointers_ext[PVA_MAX_POINTERS];
	struct pva_parameter_ext pva_ts_buffers_ext[PVA_MAX_FENCE_TYPES]
		[PVA_MAX_FENCES_PER_TYPE];
};

struct pva_submit_tasks {
	struct pva_submit_task *tasks[PVA_MAX_TASKS];
	u32 task_thresh[PVA_MAX_TASKS];
	u16 flags;
	u16 num_tasks;
};

struct pva_queue_attribute {
	enum pva_queue_attr_id id;
	uint32_t value;
};

struct pva_queue_set_attribute {
	struct pva *pva;
	struct pva_queue_attribute *attr;
	bool bootup;
};

void pva_task_remove(struct pva_submit_task *task);
void pva_task_free(struct kref *ref);

#endif
