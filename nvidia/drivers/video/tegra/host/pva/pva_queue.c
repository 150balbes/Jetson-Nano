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

#include <linux/delay.h>
#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/nvhost.h>
#include <linux/cvnas.h>

#include <trace/events/nvhost.h>

#ifdef CONFIG_EVENTLIB
#include <linux/keventlib.h>
#include <uapi/linux/nvdev_fence.h>
#include <uapi/linux/nvhost_events.h>
#endif


#include <uapi/linux/nvhost_pva_ioctl.h>

#include "nvhost_syncpt_unit_interface.h"
#include "../drivers/staging/android/sync.h"
#include "pva.h"
#include "pva-task.h"
#include "nvhost_buffer.h"
#include "nvhost_queue.h"
#include "pva_mailbox.h"
#include "pva_ccq.h"
#include "pva_queue.h"
#include "dev.h"
#include "hw_cfg_pva.h"
#include "t194/hardware_t194.h"
#include "pva-vpu-perf.h"

#include <trace/events/nvhost_pva.h>

#define ACTION_LIST_FENCE_SIZE 13
#define ACTION_LIST_STATUS_OPERATION_SIZE 11
#define ACTION_LIST_TERMINATION_SIZE 1
#define ACTION_LIST_STATS_SIZE 9

/*
 * The worst-case input action buffer size:
 * - Prefences trigger a word memory operation (size 13 bytes)
 * - Input status reads trigger a half-word memory operation (size 11 bytes)
 * - The action list is terminated by a null action (1 byte)
 */
#define INPUT_ACTION_BUFFER_SIZE ( \
	ALIGN((PVA_MAX_PREFENCES + 10) * ACTION_LIST_FENCE_SIZE + \
	      PVA_MAX_INPUT_STATUS * ACTION_LIST_STATUS_OPERATION_SIZE  + \
	      ACTION_LIST_TERMINATION_SIZE, 256))

/*
 * The worst-case output action buffer size:
 * - Postfences trigger a word memory operation (size 13 bytes)
 * - Output status write triggers a half-word memory operation (size 11 bytes)
 * - Output action list includes a operation for stats purpose (size 9 bytes)
 * - Output action list includes syncpoint and semaphore increments
 * - The action list is terminated by a null action (1 byte)
 */
#define OUTPUT_ACTION_BUFFER_SIZE ( \
	ALIGN((PVA_MAX_POSTFENCES + 10) * ACTION_LIST_FENCE_SIZE + \
	      PVA_MAX_OUTPUT_STATUS * ACTION_LIST_STATUS_OPERATION_SIZE  + \
	      ACTION_LIST_STATS_SIZE * 2 + \
	      ACTION_LIST_FENCE_SIZE  * 2 + \
	      ACTION_LIST_TERMINATION_SIZE, 256))

struct pva_hw_task {
	struct pva_task task;
	struct pva_action_list preaction_list;
	struct pva_action_list postaction_list;
	struct pva_task_parameter_array input_parameter_array[PVA_PARAM_LAST];
	struct pva_task_parameter_array output_parameter_array[PVA_PARAM_LAST];
	u8 preactions[INPUT_ACTION_BUFFER_SIZE];
	u8 postactions[OUTPUT_ACTION_BUFFER_SIZE];
	struct pva_task_parameter_desc input_surface_desc;
	struct pva_task_surface input_surfaces[PVA_MAX_INPUT_SURFACES];
	struct pva_task_parameter_desc output_surface_desc;
	struct pva_task_surface output_surfaces[PVA_MAX_OUTPUT_SURFACES];
	struct pva_task_statistics statistics;
	struct pva_task_vpu_perf_counter
		vpu_perf_counters[PVA_TASK_VPU_NUM_PERF_COUNTERS];
	u8 opaque_data[PVA_MAX_PRIMARY_PAYLOAD_SIZE];
};

static void pva_task_dump(struct pva_submit_task *task)
{
	int i;

	nvhost_dbg_info("task=%p, "
			"input_scalars=(handle=%u, offset=%x), "
			"input_surfaces=%p, "
			"output_scalars=(handle=%u, offset=%u), "
			"output_surfaces=%p, "
			"primary_payload=%p (size=%u)",
			task,
			task->input_scalars.handle, task->input_scalars.offset,
			task->input_surfaces,
			task->output_scalars.handle, task->output_scalars.offset,
			task->output_surfaces,
			task->primary_payload, task->primary_payload_size);

	for (i = 0; i < task->num_prefences; i++)
		nvhost_dbg_info("prefence %d: type=%u, "
				"syncpoint_index=%u, syncpoint_value=%u, "
				"sync_fd=%u, semaphore_handle=%u, "
				"semaphore_offset=%u, semaphore_value=%u", i,
				task->prefences[i].type,
				task->prefences[i].syncpoint_index,
				task->prefences[i].syncpoint_value,
				task->prefences[i].sync_fd,
				task->prefences[i].semaphore_handle,
				task->prefences[i].semaphore_offset,
				task->prefences[i].semaphore_value);

	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		int j;

		for (j = 0; j < task->num_pvafences[i]; j++) {
			nvhost_dbg_info("pvafence %d: type=%u, "
				"syncpoint_index=%u, syncpoint_value=%u, "
				"sync_fd=%u, semaphore_handle=%u, "
				"semaphore_offset=%u, semaphore_value=%u", i,
				task->pvafences[i][j].fence.type,
				task->pvafences[i][j].fence.syncpoint_index,
				task->pvafences[i][j].fence.syncpoint_value,
				task->pvafences[i][j].fence.sync_fd,
				task->pvafences[i][j].fence.semaphore_handle,
				task->pvafences[i][j].fence.semaphore_offset,
				task->pvafences[i][j].fence.semaphore_value);
		}
	}

	for (i = 0; i < task->num_input_surfaces; i++)
		nvhost_dbg_info("input surface %d: format=%llu, "
				"surface_handle=%u, surface_offset=%u, "
				"roi_handle=%u, roi_offset=%u, surface_stride=%u, "
				"line_stride=%u, depth=%u, width=%u, height=%u, "
				"layout=%u", i,
				task->input_surfaces[i].format,
				task->input_surfaces[i].surface_handle,
				task->input_surfaces[i].surface_offset,
				task->input_surfaces[i].roi_handle,
				task->input_surfaces[i].roi_offset,
				task->input_surfaces[i].surface_stride,
				task->input_surfaces[i].line_stride,
				task->input_surfaces[i].depth,
				task->input_surfaces[i].width,
				task->input_surfaces[i].height,
				task->input_surfaces[i].layout);

	for (i = 0; i < task->num_output_surfaces; i++)
		nvhost_dbg_info("output surface %d: format=%llu, "
				"surface_handle=%u, surface_offset=%u, "
				"roi_handle=%u, roi_offset=%u, surface_stride=%u,"
				"line_stride=%u, depth=%u, width=%u, height=%u, "
				"layout=%u", i,
				task->output_surfaces[i].format,
				task->output_surfaces[i].surface_handle,
				task->output_surfaces[i].surface_offset,
				task->output_surfaces[i].roi_handle,
				task->output_surfaces[i].roi_offset,
				task->output_surfaces[i].surface_stride,
				task->output_surfaces[i].line_stride,
				task->output_surfaces[i].depth,
				task->output_surfaces[i].width,
				task->output_surfaces[i].height,
				task->output_surfaces[i].layout);

	for (i = 0; i < task->num_pointers; i++)
		nvhost_dbg_info("pointer %d: handle=%u, offset=%u",
				i, task->pointers[i].handle,
				task->pointers[i].offset);

	for (i = 0; i < task->num_input_task_status; i++)
		nvhost_dbg_info("input task status %d: handle=%u, offset=%u",
				i, task->input_task_status[i].handle,
				task->input_task_status[i].offset);

	for (i = 0; i < task->num_output_task_status; i++)
		nvhost_dbg_info("output task status %d: handle=%u, offset=%u",
				i, task->output_task_status[i].handle,
				task->output_task_status[i].offset);
}

static void pva_task_get_memsize(size_t *dma_size, size_t *kmem_size)
{
	/* Align task addr to 64bytes boundary for DMA use*/
	*dma_size = ALIGN(sizeof(struct pva_hw_task) + 64, 64);
	*kmem_size = sizeof(struct pva_submit_task);
}

static void pva_task_unpin_mem(struct pva_submit_task *task)
{
	int i;
	int j;

#define UNPIN_MEMORY(dst_name)						\
	do {								\
		if ((((dst_name).dmabuf) != NULL) &&			\
				((dst_name).dma_addr != 0)) {		\
			nvhost_buffer_submit_unpin(task->buffers,	\
				&((dst_name).dmabuf), 1);		\
			dma_buf_put((dst_name).dmabuf);			\
		}							\
	} while (0)

	for (i = 0; i < task->num_input_surfaces; i++) {
		UNPIN_MEMORY(task->input_surfaces_ext[i]);
		UNPIN_MEMORY(task->input_surface_rois_ext[i]);
	}


	for (i = 0; i < task->num_output_surfaces; i++) {
		UNPIN_MEMORY(task->output_surfaces_ext[i]);
		UNPIN_MEMORY(task->output_surface_rois_ext[i]);
	}

	for (i = 0; i < task->num_prefences; i++) {
		if ((task->prefences[i].type == NVDEV_FENCE_TYPE_SEMAPHORE)
			&& task->prefences[i].semaphore_handle)
			UNPIN_MEMORY(task->prefences_sema_ext[i]);
	}

	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		for (j = 0; j < task->num_pvafences[i]; j++) {
			struct nvpva_fence *fence = &task->pvafences[i][j];

			if ((fence->fence.type == NVDEV_FENCE_TYPE_SEMAPHORE)
			    && fence->fence.semaphore_handle) {
				UNPIN_MEMORY(task->pvafences_sema_ext[i][j]);
			}
		}
		for (j = 0; j < task->num_pva_ts_buffers[i]; j++) {
			struct nvpva_fence *fence = &task->pvafences[i][j];

			if (fence->ts_buf_ptr.handle) {
				UNPIN_MEMORY(task->pva_ts_buffers_ext[i][j]);
			}
		}
	}

	for (i = 0; i < task->num_input_task_status; i++) {
		if (task->input_task_status[i].handle) {
			UNPIN_MEMORY(task->input_task_status_ext[i]);
		}
	}

	for (i = 0; i < task->num_output_task_status; i++) {
		if (task->output_task_status[i].handle) {
			UNPIN_MEMORY(task->output_task_status_ext[i]);
		}
	}

	for (i = 0; i < task->num_pointers; i++) {
		if (task->pointers[i].handle) {
			UNPIN_MEMORY(task->pointers_ext[i]);
		}
	}

	UNPIN_MEMORY(task->input_scalars_ext);
	UNPIN_MEMORY(task->output_scalars_ext);

#undef UNPIN_MEMORY
}

static int pva_task_pin_mem(struct pva_submit_task *task)
{
	u32 cvsram_base = nvcvnas_get_cvsram_base();
	u32 cvsram_sz = nvcvnas_get_cvsram_size();
	int err;
	int i;
	int j;

#define PIN_MEMORY(dst_name, dmabuf_fd)					\
	do {								\
		if (!(dmabuf_fd)) {					\
			err = -EFAULT;					\
			goto err_map_handle;				\
		}							\
									\
		((dst_name).dmabuf) = dma_buf_get(dmabuf_fd);		\
		if (IS_ERR_OR_NULL((dst_name).dmabuf)) {		\
			(dst_name).dmabuf = NULL;			\
			err = -EFAULT;					\
			goto err_map_handle;				\
		}							\
									\
		err = nvhost_buffer_submit_pin(task->buffers,		\
				&(dst_name).dmabuf, 1,			\
				&(dst_name).dma_addr,			\
				&(dst_name).size,			\
				&(dst_name).heap);			\
		if (err < 0)						\
			goto err_map_handle;				\
	} while (0)

	/* Pin input surfaces */
	for (i = 0; i < task->num_input_surfaces; i++) {
		/* HACK: nvmap doesn't support CVNAS yet */
		if (task->input_surfaces[i].surface_handle == 0) {
			u32 offset = task->input_surfaces[i].surface_offset;

			if (offset > cvsram_sz) {
				err = -EINVAL;
				goto err_map_handle;
			}

			task->input_surfaces_ext[i].dma_addr = cvsram_base;
			task->input_surfaces_ext[i].size = cvsram_sz - offset;
			task->input_surfaces_ext[i].heap =
				NVHOST_BUFFERS_HEAP_CVNAS;
		} else {
			PIN_MEMORY(task->input_surfaces_ext[i],
				task->input_surfaces[i].surface_handle);
		}

		if (task->input_surfaces[i].roi_handle)
			PIN_MEMORY(task->input_surface_rois_ext[i],
				task->input_surfaces[i].roi_handle);
	}

	/* ...and then output surfaces */
	for (i = 0; i < task->num_output_surfaces; i++) {
		if (task->output_surfaces[i].surface_handle == 0) {
			/* HACK: To support the MISR test
			 * Kernel is not suppose to convert the address being
			 * passed from the UMD. So setting dma_addr as  the
			 * offset passed from KMD and size to 4MB
			 */
			u32 offset = task->output_surfaces[i].surface_offset;

			/* Only root is allowed to use offsets */
			if (current_uid().val != 0) {
				err = -EINVAL;
				goto err_map_handle;
			}

			task->output_surfaces_ext[i].dma_addr = offset;
			task->output_surfaces_ext[i].size =  0x400000;
		} else {
			PIN_MEMORY(task->output_surfaces_ext[i],
				task->output_surfaces[i].surface_handle);
		}

		if (task->output_surfaces[i].roi_handle)
			PIN_MEMORY(task->output_surface_rois_ext[i],
				task->output_surfaces[i].roi_handle);
	}

	/* check fence semaphore_type before memory pin */
	for (i = 0; i < task->num_prefences; i++) {
		if ((task->prefences[i].type == NVDEV_FENCE_TYPE_SEMAPHORE)
			&& task->prefences[i].semaphore_handle) {
			PIN_MEMORY(task->prefences_sema_ext[i],
				task->prefences[i].semaphore_handle);
		}
	}

	/* check the generalized fence structures */
	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		for (j = 0; j < task->num_pvafences[i]; j++) {
			struct nvpva_fence *fence = &task->pvafences[i][j];

			if ((fence->fence.type == NVDEV_FENCE_TYPE_SEMAPHORE)
			    && fence->fence.semaphore_handle) {
				PIN_MEMORY(task->pvafences_sema_ext[i][j],
					   fence->fence.semaphore_handle);
			}
		}
		for (j = 0; j < task->num_pva_ts_buffers[i]; j++) {
			struct nvpva_fence *fence = &task->pvafences[i][j];

			if (fence->ts_buf_ptr.handle) {
				PIN_MEMORY(task->pva_ts_buffers_ext[i][j],
					   fence->ts_buf_ptr.handle);
			}
		}
	}

	/* Pin the input and output action status */
	for (i = 0; i < task->num_input_task_status; i++) {
		if (task->input_task_status[i].handle) {
			PIN_MEMORY(task->input_task_status_ext[i],
				task->input_task_status[i].handle);
		}
	}

	for (i = 0; i < task->num_output_task_status; i++) {
		if (task->output_task_status[i].handle) {
			PIN_MEMORY(task->output_task_status_ext[i],
				task->output_task_status[i].handle);
		}
	}

	/* Pin task pointers */
	for (i = 0; i < task->num_pointers; i++) {
		if (task->pointers[i].handle) {
			PIN_MEMORY(task->pointers_ext[i],
				   task->pointers[i].handle);
		}
	}

	/* Pin rest */
	if (task->input_scalars.handle)
		PIN_MEMORY(task->input_scalars_ext,
			task->input_scalars.handle);

	if (task->output_scalars.handle)
		PIN_MEMORY(task->output_scalars_ext,
			task->output_scalars.handle);

#undef PIN_MEMORY

	return 0;

err_map_handle:
	pva_task_unpin_mem(task);
	return err;
}

static void pva_task_write_surfaces(struct pva_task_surface *hw_surface,
		struct pva_surface *surface,
		struct pva_parameter_ext *surface_ext,
		struct pva_parameter_ext *roi_ext,
		unsigned int count)
{
	int i;

	for (i = 0; i < count; i++) {
		hw_surface[i].address = surface_ext[i].dma_addr +
			surface[i].surface_offset;
		hw_surface[i].surface_size = surface_ext[i].size;
		hw_surface[i].roi_addr = roi_ext[i].dma_addr +
			surface[i].roi_offset;
		hw_surface[i].roi_size = roi_ext[i].size;
		hw_surface[i].format = surface[i].format;
		hw_surface[i].width = surface[i].width;
		hw_surface[i].height = surface[i].height;
		hw_surface[i].line_stride = surface[i].line_stride;
		hw_surface[i].plane_stride = surface[i].surface_stride;
		hw_surface[i].num_planes = surface[i].depth;
		hw_surface[i].layout = surface[i].layout;
		hw_surface[i].block_height_log2 = surface[i].block_height_log2;

		/* Set bit 39 for block linear surfaces in the address field.
		*  This bit is used for indicating that memory subsystem should
		*  convert the block linear format into common block linear format
		*  that is used by other engines in Tegra. Thebit in itself is
		*  dropped before making the address translation in SMMU.
		*/
		if (surface[i].layout == PVA_TASK_SURFACE_LAYOUT_BLOCK_LINEAR)
			hw_surface[i].address |= PVA_BIT64(39);

		hw_surface[i].memory = surface_ext[i].heap;
	}
}

static inline int pva_task_write_atomic_op(u8 *base, u8 action)
{
	*base = action;

	return 1;
}

static inline int
pva_task_write_struct_ptr_op(u8 *base, u8 action, u64 addr, u16 val)
{
	int i = 0;

	base[i++] = action;
	base[i++] = (u8)((addr >> 0) & 0xff);
	base[i++] = (u8)((addr >> 8) & 0xff);
	base[i++] = (u8)((addr >> 16) & 0xff);
	base[i++] = (u8)((addr >> 24) & 0xff);
	base[i++] = (u8)((addr >> 32) & 0xff);
	base[i++] = (u8)((addr >> 40) & 0xff);
	base[i++] = (u8)((addr >> 48) & 0xff);
	base[i++] = (u8)((addr >> 56) & 0xff);

	return i;
}

static inline int pva_task_write_ptr_16b_op(u8 *base, u8 action, u64 addr, u16 val)
{
	int i = 0;

	base[i++] = action;
	base[i++] = (u8)((addr >> 0) & 0xff);
	base[i++] = (u8)((addr >> 8) & 0xff);
	base[i++] = (u8)((addr >> 16) & 0xff);
	base[i++] = (u8)((addr >> 24) & 0xff);
	base[i++] = (u8)((addr >> 32) & 0xff);
	base[i++] = (u8)((addr >> 40) & 0xff);
	base[i++] = (u8)((addr >> 48) & 0xff);
	base[i++] = (u8)((addr >> 56) & 0xff);
	base[i++] = (u8)((val >> 0) & 0xff);
	base[i++] = (u8)((val >> 8) & 0xff);

	return i;
}

static inline int pva_task_write_ptr_op(u8 *base, u8 action, u64 addr, u32 val)
{
	int i = 0;

	base[i++] = action;
	base[i++] = (u8)((addr >> 0) & 0xff);
	base[i++] = (u8)((addr >> 8) & 0xff);
	base[i++] = (u8)((addr >> 16) & 0xff);
	base[i++] = (u8)((addr >> 24) & 0xff);
	base[i++] = (u8)((addr >> 32) & 0xff);
	base[i++] = (u8)((addr >> 40) & 0xff);
	base[i++] = (u8)((addr >> 48) & 0xff);
	base[i++] = (u8)((addr >> 56) & 0xff);
	base[i++] = (u8)((val >> 0) & 0xff);
	base[i++] = (u8)((val >> 8) & 0xff);
	base[i++] = (u8)((val >> 16) & 0xff);
	base[i++] = (u8)((val >> 24) & 0xff);

	return i;
}

static int pva_task_write_preactions(struct pva_submit_task *task,
				     struct pva_hw_task *hw_task)
{
	struct platform_device *host1x_pdev =
		to_platform_device(task->pva->pdev->dev.parent);
	u8 *hw_preactions = hw_task->preactions;
	int i = 0, j = 0, ptr = 0;
	u8 action_ts;
	u8 action_f;
	u32 increment;

	/* Add waits to preactions list */
	for (i = 0; i < task->num_prefences; i++) {
		struct nvdev_fence *fence = task->prefences + i;

		switch (fence->type) {
		case NVDEV_FENCE_TYPE_SYNCPT: {
			dma_addr_t syncpt_addr = nvhost_syncpt_gos_address(
							task->pva->pdev,
							fence->syncpoint_index);
			if (!syncpt_addr)
				syncpt_addr = nvhost_syncpt_address(
							task->queue->vm_pdev,
							fence->syncpoint_index);

			ptr += pva_task_write_ptr_op(&hw_preactions[ptr],
				TASK_ACT_PTR_BLK_GTREQL, syncpt_addr,
				fence->syncpoint_value);
			break;
		}
		case NVDEV_FENCE_TYPE_SEMAPHORE:
		case NVDEV_FENCE_TYPE_SEMAPHORE_TS:{
			ptr += pva_task_write_ptr_op(&hw_preactions[ptr],
				TASK_ACT_PTR_BLK_GTREQL,
				task->prefences_sema_ext[i].dma_addr  +
					fence->semaphore_offset,
				fence->semaphore_value);
			break;
		}
		case NVDEV_FENCE_TYPE_SYNC_FD: {
			int thresh, id;
			dma_addr_t syncpt_addr;
			struct sync_fence *syncfd_fence;
			struct sync_pt *pt;
			struct nvhost_master *host = nvhost_get_host(
							task->pva->pdev);
			struct nvhost_syncpt *sp = &host->syncpt;

			if (!fence->sync_fd)
				break;

			syncfd_fence = nvhost_sync_fdget(fence->sync_fd);
			if (!syncfd_fence)
				break;

			for (j = 0; j < syncfd_fence->num_fences; j++) {
				pt = sync_pt_from_fence(
					syncfd_fence->cbs[j].sync_pt);
				if (!pt)
					break;

				id = nvhost_sync_pt_id(pt);
				thresh = nvhost_sync_pt_thresh(pt);

				/* validate the synpt ids */
				if (!id ||
				!nvhost_syncpt_is_valid_hw_pt(sp, id)) {
					sync_fence_put(syncfd_fence);
					break;
				}

				if (nvhost_syncpt_is_expired(sp,
							id, thresh))
					continue;

				syncpt_addr = nvhost_syncpt_gos_address(
							task->pva->pdev, id);
				if (!syncpt_addr)
					syncpt_addr = nvhost_syncpt_address(
							task->queue->vm_pdev, id);

				ptr += pva_task_write_ptr_op(
						&hw_preactions[ptr],
						TASK_ACT_PTR_BLK_GTREQL,
						syncpt_addr, thresh);

			}
			break;
		}
		default:
			return -ENOSYS;
		}
	}

	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		increment = 0;
		switch (i) {
		case PVA_FENCE_SOT_V:
			action_ts = TASK_ACT_PTR_WRITE_SOT_V_TS;
			action_f = TASK_ACT_PTR_WRITE_VAL_SOT_V;
			increment = 1;
			break;
		case PVA_FENCE_SOT_R:
			action_ts = TASK_ACT_PTR_WRITE_SOT_R_TS;
			action_f = TASK_ACT_PTR_WRITE_VAL_SOT_R;
			increment = 1;
			break;
		default:
			action_ts = 0;
			action_f = 0;
			break;
		};
		if ((action_ts == 0) || (task->num_pvafences[i] == 0))
			continue;
		for (j = 0; j < task->num_pva_ts_buffers[i]; j++) {
			if (task->pvafences[i][j].ts_buf_ptr.handle) {
				int dif;

				dif = pva_task_write_ptr_op(
				       &hw_preactions[ptr],
				       action_ts,
				       task->pva_ts_buffers_ext[i][j].dma_addr +
				       task->pvafences[i][j].ts_buf_ptr.offset,
				       1U);
				ptr += dif;
			}
		}
		for (j = 0; j < task->num_pvafences[i]; j++) {
			struct nvdev_fence *fence =
				&task->pvafences[i][j].fence;
			u32 thresh;

			switch (fence->type) {
			case NVDEV_FENCE_TYPE_SYNCPT: {
				dma_addr_t syncpt_gos_addr =
					nvhost_syncpt_gos_address(
						task->pva->pdev,
						fence->syncpoint_index);
				dma_addr_t syncpt_addr =
					nvhost_syncpt_address(
						task->queue->vm_pdev,
						task->queue->syncpt_id);

				ptr += pva_task_write_ptr_op(
					&hw_preactions[ptr],
					action_f,
					syncpt_addr,
					1U);
				task->fence_num += increment;
				/* Make a syncpoint increment */
				if (syncpt_gos_addr) {
					thresh = nvhost_syncpt_read_maxval(
						host1x_pdev,
						task->queue->syncpt_id) +
						task->fence_num;
					ptr += pva_task_write_ptr_op(
						&hw_preactions[ptr],
						TASK_ACT_PTR_WRITE_VAL,
						syncpt_gos_addr, thresh);
				}
				break;
			}
			case NVDEV_FENCE_TYPE_SEMAPHORE:
			case NVDEV_FENCE_TYPE_SEMAPHORE_TS: {
				int dif;

				dif = pva_task_write_ptr_op(&hw_preactions[ptr],
				       action_f,
				       task->pvafences_sema_ext[i][j].dma_addr +
				       fence->semaphore_offset,
				       fence->semaphore_value);
				ptr += dif;
				break;
			}
			case NVDEV_FENCE_TYPE_SYNC_FD:
				/* TODO XXX*/
			default:
				return -ENOSYS;
			}
		}
	}

	/* Perform input status checks */
	for (i = 0; i < task->num_input_task_status; i++) {
		struct pva_status_handle *input_status =
					task->input_task_status + i;
		dma_addr_t input_status_addr =
				task->input_task_status_ext[i].dma_addr  +
				input_status->offset;

		ptr += pva_task_write_ptr_16b_op(
					&hw_preactions[ptr],
					TASK_ACT_READ_STATUS,
					input_status_addr, 0);
	}

	ptr += pva_task_write_atomic_op(&hw_preactions[ptr],
		TASK_ACT_TERMINATE);

	/* Store the preaction list */
	hw_task->preaction_list.offset = offsetof(struct pva_hw_task, preactions);
	hw_task->preaction_list.length = ptr;

	nvhost_dbg_info("preaction buffer alloted size %d: used size %d",
				INPUT_ACTION_BUFFER_SIZE, ptr);
	return 0;
}

static void pva_task_write_postactions(struct pva_submit_task *task,
				       struct pva_hw_task *hw_task)
{
	dma_addr_t syncpt_addr = nvhost_syncpt_address(task->queue->vm_pdev,
				task->queue->syncpt_id);
	dma_addr_t syncpt_gos_addr = nvhost_syncpt_gos_address(task->pva->pdev,
				task->queue->syncpt_id);
	u8 *hw_postactions = hw_task->postactions;
	int ptr = 0, i = 0;
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	dma_addr_t output_status_addr;
	u32 thresh;
	u8 action_ts;
	int j;

	/* Write Output action status */
	for (i = 0; i < task->num_output_task_status; i++) {
		struct pva_status_handle *output_status =
					task->output_task_status + i;

		output_status_addr = task->output_task_status_ext[i].dma_addr +
				     output_status->offset;
		ptr += pva_task_write_ptr_16b_op(
					&hw_postactions[ptr],
					TASK_ACT_WRITE_STATUS,
					output_status_addr, 1);
	}

	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		switch (i) {
		case PVA_FENCE_EOT_V:
			action_ts = TASK_ACT_PTR_WRITE_EOT_V_TS;
			break;
		case PVA_FENCE_EOT_R:
			action_ts = TASK_ACT_PTR_WRITE_EOT_R_TS;
			break;
		case PVA_FENCE_POST:
			action_ts = TASK_ACT_PTR_WRITE_TS;
			break;
		default:
			action_ts = 0;
			break;
		};
		if (action_ts == 0)
			continue;
		for (j = 0; j < task->num_pva_ts_buffers[i]; j++) {
			if (task->pvafences[i][j].ts_buf_ptr.handle) {
				int dif;

				dif = pva_task_write_ptr_op(
				       &hw_postactions[ptr],
				       action_ts,
				       task->pva_ts_buffers_ext[i][j].dma_addr +
				       task->pvafences[i][j].ts_buf_ptr.offset,
				       1U);

				ptr += dif;
			}
		}
	}

	/* Add postactions list for semaphore */
	j = PVA_FENCE_POST;
	for (i = 0; i < task->num_pvafences[j]; i++) {
		struct nvdev_fence *fence = &task->pvafences[j][i].fence;

		if (fence->type == NVDEV_FENCE_TYPE_SEMAPHORE) {
			ptr += pva_task_write_ptr_op(&hw_postactions[ptr],
				TASK_ACT_PTR_WRITE_VAL,
				task->pvafences_sema_ext[j][i].dma_addr +
				fence->semaphore_offset,
				fence->semaphore_value);
		} else if (fence->type == NVDEV_FENCE_TYPE_SEMAPHORE_TS) {
			/*
			* Timestamp will be filled by ucode hence making the
			* place holder for timestamp size, sizeof(u64).
			*/
			ptr += sizeof(u64) +
			       pva_task_write_ptr_op(
				&hw_postactions[ptr],
				TASK_ACT_PTR_WRITE_VAL_TS,
				task->pvafences_sema_ext[j][i].dma_addr +
				fence->semaphore_offset,
				fence->semaphore_value);
		}
	}

	task->fence_num += 1;

	/* Make a syncpoint increment */
	if (syncpt_gos_addr) {
		thresh = nvhost_syncpt_read_maxval(
			host1x_pdev,
			task->queue->syncpt_id) + task->fence_num;
		ptr += pva_task_write_ptr_op(
			&hw_postactions[ptr],
			TASK_ACT_PTR_WRITE_VAL,
			syncpt_gos_addr, thresh);
	}

	ptr += pva_task_write_ptr_op(&hw_postactions[ptr],
		TASK_ACT_PTR_WRITE_VAL, syncpt_addr, 1);


	output_status_addr = task->dma_addr +
			     offsetof(struct pva_hw_task, statistics);
	ptr += pva_task_write_struct_ptr_op(
				&hw_postactions[ptr],
				TASK_ACT_PVA_STATISTICS,
				output_status_addr, 1);

	if (task->pva->vpu_perf_counters_enable) {
		ptr += pva_task_write_struct_ptr_op(
				&hw_postactions[ptr],
				TASK_ACT_PVA_VPU_PERF_COUNTERS,
				task->dma_addr + offsetof(struct pva_hw_task,
					vpu_perf_counters),
				1);
	}

	ptr += pva_task_write_atomic_op(&hw_postactions[ptr],
		TASK_ACT_TERMINATE);

	/* Store the postaction list */
	hw_task->postaction_list.offset = offsetof(struct pva_hw_task,
						   postactions);
	hw_task->postaction_list.length = ptr;

	nvhost_dbg_info("postaction buffer alloted size %d: used size %d",
				OUTPUT_ACTION_BUFFER_SIZE, ptr);

}

static void pva_task_write_output_surfaces(struct pva_submit_task *task,
					   struct pva_hw_task *hw_task)
{
	struct pva_task_parameter_array *surface_parameter;

	if (task->num_output_surfaces == 0)
		return;

	surface_parameter = hw_task->output_parameter_array +
			    hw_task->task.num_output_parameters;

	/* Write parameter descriptor */
	surface_parameter->address = task->dma_addr +
				     offsetof(struct pva_hw_task,
					      output_surface_desc);
	surface_parameter->type = PVA_PARAM_SURFACE_LIST;
	surface_parameter->size = sizeof(struct pva_task_parameter_desc) +
				  sizeof(struct pva_task_surface) *
				  task->num_output_surfaces;
	hw_task->task.num_output_parameters++;

	/* Write the surface descriptor base information */
	hw_task->output_surface_desc.num_parameters = task->num_output_surfaces;
	hw_task->output_surface_desc.reserved = 0;

	/* Write the output surfaces */
	pva_task_write_surfaces(hw_task->output_surfaces,
			task->output_surfaces,
			task->output_surfaces_ext,
			task->output_surface_rois_ext,
			task->num_output_surfaces);
}

static void pva_task_write_input_surfaces(struct pva_submit_task *task,
					  struct pva_hw_task *hw_task)
{
	struct pva_task_parameter_array *surface_parameter;

	if (task->num_input_surfaces == 0)
		return;

	surface_parameter = hw_task->input_parameter_array +
			    hw_task->task.num_input_parameters;

	/* Write parameter descriptor */
	surface_parameter->address = task->dma_addr +
				     offsetof(struct pva_hw_task,
					      input_surface_desc);
	surface_parameter->type = PVA_PARAM_SURFACE_LIST;
	surface_parameter->size = sizeof(struct pva_task_parameter_desc) +
				  sizeof(struct pva_task_surface) *
				  task->num_input_surfaces;
	hw_task->task.num_input_parameters++;

	/* Write the surface descriptor base information */
	hw_task->input_surface_desc.num_parameters = task->num_input_surfaces;
	hw_task->input_surface_desc.reserved = 0;

	/* Write the input surfaces */
	pva_task_write_surfaces(hw_task->input_surfaces,
			task->input_surfaces,
			task->input_surfaces_ext,
			task->input_surface_rois_ext,
			task->num_input_surfaces);
}

static void pva_task_write_non_surfaces(struct pva_submit_task *task,
					struct pva_hw_task *hw_task)
{
	struct pva_task_parameter_array *hw_input_parameters =
			hw_task->input_parameter_array;
	struct pva_task_parameter_array *hw_output_parameters =
			hw_task->output_parameter_array;

#define COPY_PARAMETER(target, name, name_ext, param_type, count)	\
	do {								\
		if ((name).handle) {					\
			target[(count)].address = (name_ext).dma_addr + \
						  (name).offset;	\
			target[(count)].size = (name_ext).size -	\
					       (name).offset;		\
			target[(count)].type = (param_type);		\
			(count)++;					\
		}							\
	} while (0)

	COPY_PARAMETER(hw_input_parameters, task->input_scalars,
		       task->input_scalars_ext,
		       PVA_PARAM_SCALAR_LIST,
		       hw_task->task.num_input_parameters);
	COPY_PARAMETER(hw_output_parameters, task->output_scalars,
		       task->output_scalars_ext,
		       PVA_PARAM_SCALAR_LIST,
		       hw_task->task.num_output_parameters);
#undef COPY_PARAMETER
}

static int pva_task_write_opaque_data(struct pva_submit_task *task,
				      struct pva_hw_task *hw_task)
{
	struct pva_task_parameter_array *opaque_parameter;
	struct pva_task_opaque_data_desc *opaque_desc;
	struct pva_parameter_ext *handle_ext;
	unsigned int primary_payload_offset;
	struct pva_memory_handle *handle;
	unsigned int pointer_list_offset;
	struct pva_task_pointer pointer;
	u8 *primary_payload, *pointers;
	unsigned int num_bytes;
	u64 aux, size, flags;
	unsigned int i;

	if (task->num_pointers == 0 && task->primary_payload_size == 0)
		return 0;

	/* Calculate size of the opaque data */
	num_bytes = sizeof(struct pva_task_opaque_data_desc);
	num_bytes += task->primary_payload_size;
	num_bytes += sizeof(struct pva_task_pointer) * task->num_pointers;

	if (num_bytes > PVA_MAX_PRIMARY_PAYLOAD_SIZE)
		return -ENOMEM;

	/* Opaque parameter resides always in the input parameter block */
	opaque_parameter = hw_task->input_parameter_array +
			   hw_task->task.num_input_parameters;

	/* Write parameter descriptor */
	opaque_parameter->address = task->dma_addr +
				     offsetof(struct pva_hw_task,
					      opaque_data);
	opaque_parameter->type = PVA_PARAM_OPAQUE_DATA;
	opaque_parameter->size = num_bytes;
	hw_task->task.num_input_parameters++;

	/* Determine offset to the primary_payload start */
	primary_payload_offset = sizeof(struct pva_task_opaque_data_desc);
	primary_payload = hw_task->opaque_data + primary_payload_offset;

	/* Determine offset to the start of the pointer list */
	pointer_list_offset = primary_payload_offset +
		task->primary_payload_size;
	pointers = hw_task->opaque_data + pointer_list_offset;

	/* Initialize the opaque data descriptor */
	opaque_desc = (void *)hw_task->opaque_data;
	opaque_desc->primary_payload_size = task->primary_payload_size;

	/* Copy the primary_payload */
	memcpy(primary_payload,
	       task->primary_payload,
	       task->primary_payload_size);

	/* Copy the pointers */
	for (i = 0; i < task->num_pointers; i++) {
		handle = task->pointers + i;
		handle_ext = task->pointers_ext + i;

		size = handle_ext->size & PVA_TASK_POINTER_AUX_SIZE_MASK;
		if (size != handle_ext->size) {
			return -EINVAL;
		}

		flags = 0;
		if (handle_ext->heap == NVHOST_BUFFERS_HEAP_CVNAS)
			flags |= PVA_TASK_POINTER_AUX_FLAGS_CVNAS;

		aux = (size << PVA_TASK_POINTER_AUX_SIZE_SHIFT) |
		      (flags << PVA_TASK_POINTER_AUX_FLAGS_SHIFT);

		pointer.address = handle_ext->dma_addr + handle->offset;
		pointer.aux = aux;

		/* The data might be unaligned. copy it byte-by-byte */
		memcpy(pointers, &pointer, sizeof(pointer));
		pointers += sizeof(pointer);
	}

	return 0;
}

static int pva_task_write(struct pva_submit_task *task, bool atomic)
{
	struct pva_hw_task *hw_task;
	int err;
	int i;

	/* Task start from the memory base */
	hw_task = task->va;

	/* Write the preaction list */
	err = pva_task_write_preactions(task, hw_task);
	if (err < 0)
		return err;

	/* Write the postaction list */
	pva_task_write_postactions(task, hw_task);

	/* Initialize parameters */
	pva_task_write_non_surfaces(task, hw_task);

	/* Write the pointers and the primary payload */
	err = pva_task_write_opaque_data(task, hw_task);
	if (err < 0)
		return err;

	/* Write input surfaces */
	pva_task_write_input_surfaces(task, hw_task);

	/* Write output surfaces */
	pva_task_write_output_surfaces(task, hw_task);

	hw_task->task.input_parameters = offsetof(struct pva_hw_task,
						  input_parameter_array);
	hw_task->task.output_parameters = offsetof(struct pva_hw_task,
						  output_parameter_array);
	hw_task->task.gen_task.versionid = TASK_VERSION_ID;
	hw_task->task.gen_task.engineid = PVA_ENGINE_ID;
	hw_task->task.gen_task.sequence = 0;
	hw_task->task.gen_task.length = offsetof(struct pva_hw_task,
						 input_surface_desc);
	hw_task->task.gen_task.n_preaction_lists = 1;
	hw_task->task.gen_task.preaction_lists_p = offsetof(struct pva_hw_task,
							    preaction_list);
	hw_task->task.gen_task.n_postaction_lists = 1;
	hw_task->task.gen_task.postaction_lists_p = offsetof(struct pva_hw_task,
							     postaction_list);
	hw_task->task.runlist_version = PVA_TASK_VERSION_ID;
	hw_task->task.queue_id = task->queue->id;
	hw_task->task.flags = atomic ? PVA_TASK_FL_ATOMIC : 0;
	hw_task->task.operation = task->operation;
	hw_task->task.timeout = task->timeout;

	/* Set flags to debug the vpu application if debugfs node is set for vpu id */
	if (task->pva->dbg_vpu_app_id == task->operation)
			hw_task->task.flags |= PVA_TASK_FL_VPU_DEBUG;

	/* This should be delivered from userspace - hard-code
	 * until the mechanism is in place.
	 */
	hw_task->task.operation_version = 1;

	for (i = 0; i < roundup(sizeof(struct pva_hw_task), 16) / 16; i++) {
		u8 *task_va = task->va;
		u32 base = i * 16;

		nvhost_dbg_info("%02x, %02x, %02x, %02x, %02x, %02x, %02x %02x, "
				"%02x, %02x, %02x, %02x, %02x, %02x, %02x %02x",
				task_va[base],
				task_va[base + 1],
				task_va[base + 2],
				task_va[base + 3],
				task_va[base + 4],
				task_va[base + 5],
				task_va[base + 6],
				task_va[base + 7],
				task_va[base + 8],
				task_va[base + 9],
				task_va[base + 10],
				task_va[base + 11],
				task_va[base + 12],
				task_va[base + 13],
				task_va[base + 14],
				task_va[base + 15]);
	}

	return 0;
}

#ifdef CONFIG_EVENTLIB
static void
pva_eventlib_record_perf_counter(struct platform_device *pdev,
				 u32 syncpt_id,
				 u32 syncpt_thresh,
				 u32 operation,
				 u32 tag,
				 u32 count,
				 u32 sum,
				 u64 sum_squared,
				 u32 min,
				 u32 max,
				 u64 timestamp_begin,
				 u64 timestamp_end)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_vpu_perf_counter perf_counter;

	if (!pdata->eventlib_id)
		return;

	perf_counter.class_id = pdata->class;
	perf_counter.syncpt_id = syncpt_id;
	perf_counter.syncpt_thresh = syncpt_thresh;
	perf_counter.operation = operation;
	perf_counter.tag = tag;
	perf_counter.count = count;
	perf_counter.average = sum / count;
	perf_counter.variance =
		((u64)count * sum_squared - (u64)sum * (u64)sum)
			/ (u64)count / (u64)count;
	perf_counter.minimum = min;
	perf_counter.maximum = max;

	keventlib_write(pdata->eventlib_id,
			&perf_counter,
			sizeof(perf_counter),
			NVHOST_VPU_PERF_COUNTER_BEGIN,
			timestamp_begin);

	keventlib_write(pdata->eventlib_id,
			&perf_counter,
			sizeof(perf_counter),
			NVHOST_VPU_PERF_COUNTER_END,
			timestamp_end);
}
static void
pva_eventlib_record_r5_states(struct platform_device *pdev,
			      u32 syncpt_id,
			      u32 syncpt_thresh,
			      struct pva_task_statistics *stats,
			      u32 operation)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_pva_task_state state;

	if (!pdata->eventlib_id)
		return;

	state.class_id = pdata->class;
	state.syncpt_id = syncpt_id;
	state.syncpt_thresh = syncpt_thresh;
	state.operation = operation;

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_QUEUE_BEGIN,
			stats->queued_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_QUEUE_END,
			stats->vpu_assigned_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_PREPARE_BEGIN,
			stats->vpu_assigned_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_PREPARE_END,
			stats->vpu_start_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			stats->vpu_assigned == 0 ? NVHOST_PVA_VPU0_BEGIN
						 : NVHOST_PVA_VPU1_BEGIN,
			stats->vpu_start_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			stats->vpu_assigned == 0 ? NVHOST_PVA_VPU0_END
						 : NVHOST_PVA_VPU1_END,
			stats->vpu_complete_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_POST_BEGIN,
			stats->vpu_complete_time);

	keventlib_write(pdata->eventlib_id,
			&state,
			sizeof(state),
			NVHOST_PVA_POST_END,
			stats->complete_time);
}
#else
static void
pva_eventlib_record_perf_counter(struct platform_device *pdev,
				 u32 syncpt_id,
				 u32 syncpt_thresh,
				 u32 operation,
				 u32 tag,
				 u32 count,
				 u32 sum,
				 u64 sum_squared,
				 u32 min,
				 u32 max,
				 u64 timestamp_begin,
				 u64 timestamp_end)
{
}
static void
pva_eventlib_record_r5_states(struct platform_device *pdev,
			      struct pva_task_statistics *stats,
			      u32 operation)
{
}
#endif

static void pva_task_update(struct pva_submit_task *task)
{
	struct nvhost_queue *queue = task->queue;
	struct pva_hw_task *hw_task = task->va;
	struct pva *pva = task->pva;
	struct platform_device *pdev = pva->pdev;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva_task_statistics *stats = &hw_task->statistics;
	struct pva_task_vpu_perf_counter *perf;
	u32 idx;

	trace_nvhost_task_timestamp(dev_name(&pdev->dev),
				    pdata->class,
				    queue->syncpt_id,
				    task->syncpt_thresh,
				    stats->vpu_assigned_time,
				    stats->complete_time);
	nvhost_eventlib_log_task(pdev,
				 queue->syncpt_id,
				 task->syncpt_thresh,
				 stats->vpu_assigned_time,
				 stats->complete_time);
	nvhost_dbg_info("Completed task %p (0x%llx), start_time=%llu, end_time=%llu",
			task, (u64)task->dma_addr,
			stats->vpu_assigned_time,
			stats->complete_time);

	trace_nvhost_pva_task_stats(pdev->name,
			stats->queued_time,
			stats->head_time,
			stats->input_actions_complete,
			stats->vpu_assigned_time,
			stats->vpu_start_time,
			stats->vpu_complete_time,
			stats->complete_time,
			stats->vpu_assigned);

	nvhost_dbg_info("QueuedTime %llu, HeadTime 0x%llu, "
			"InputActionComplete %llu, VpuAssignedTime %llu, "
			"VpuStartTime %llu, VpuCompleteTime %llu, "
			"TaskCompeteTime %llu, AssignedVpu %d",
			stats->queued_time,
			stats->head_time,
			stats->input_actions_complete,
			stats->vpu_assigned_time,
			stats->vpu_start_time,
			stats->vpu_complete_time,
			stats->complete_time,
			stats->vpu_assigned);

	pva_eventlib_record_r5_states(pdev,
			queue->syncpt_id,
			task->syncpt_thresh,
			stats, task->operation);

	/* Record task postfences */
	nvhost_eventlib_log_fences(pdev,
			queue->syncpt_id,
			task->syncpt_thresh,
			&(task->pvafences[PVA_FENCE_POST][0].fence),
			1,
			NVDEV_FENCE_KIND_POST,
			stats->complete_time);

	if (task->pva->vpu_perf_counters_enable) {
		for (idx = 0; idx < PVA_TASK_VPU_NUM_PERF_COUNTERS; idx++) {
			perf = &hw_task->vpu_perf_counters[idx];
			if (perf->count != 0) {
				trace_nvhost_pva_task_vpu_perf(
					pdev->name, idx, perf->count,
					perf->sum, perf->sum_squared,
					perf->min, perf->max);
				pva_eventlib_record_perf_counter(
					pdev, queue->syncpt_id,
					task->syncpt_thresh,
					task->operation, idx, perf->count,
					perf->sum, perf->sum_squared,
					perf->min, perf->max,
					stats->vpu_assigned_time,
					stats->complete_time);
			}
		}
	}

	/* Unpin job memory. PVA shouldn't be using it anymore */
	pva_task_unpin_mem(task);

	/* Drop PM runtime reference of PVA */
	nvhost_module_idle(task->pva->pdev);

	/* remove the task from the queue */
	list_del(&task->node);

	/* Not linked anymore so drop the reference */
	kref_put(&task->ref, pva_task_free);

	/* Drop queue reference to allow reusing it */
	nvhost_queue_put(queue);
}

void pva_task_free(struct kref *ref)
{
	struct pva_submit_task *task =
		container_of(ref, struct pva_submit_task, ref);
	/* Release memory that was allocated for the task */
	nvhost_queue_free_task_memory(task->queue, task->pool_index);
}

static void pva_queue_update(void *priv, int nr_completed)
{
	struct nvhost_queue *queue = priv;
	struct pva_submit_task *task, *n;
	struct list_head completed;

	INIT_LIST_HEAD(&completed);

	/* Move completed tasks to a separate list */
	mutex_lock(&queue->list_lock);
	list_for_each_entry_safe(task, n, &queue->tasklist, node) {
		if (!nvhost_syncpt_is_expired_ext(queue->pool->pdev,
						  queue->syncpt_id,
						  task->syncpt_thresh))
			break;

		list_move_tail(&task->node, &completed);
	}
	mutex_unlock(&queue->list_lock);

	/* Handle completed tasks */
	list_for_each_entry_safe(task, n, &completed, node)
		pva_task_update(task);
}

static void pva_queue_dump(struct nvhost_queue *queue, struct seq_file *s)
{
	struct pva_submit_task *task;
	int i = 0;
	int k = PVA_FENCE_POST;

	seq_printf(s, "Queue %u, Tasks\n", queue->id);

	mutex_lock(&queue->list_lock);
	list_for_each_entry(task, &queue->tasklist, node) {
		int j;

		seq_printf(s, "    #%u: Operation = %u\n",
				i++, task->operation);

		for (j = 0; j < task->num_prefences; j++)
			seq_printf(s, "    prefence %d: \n\t"
				"syncpoint_index=%u, syncpoint_value=%u\n",
				j,
				task->prefences[j].syncpoint_index,
				task->prefences[j].syncpoint_value);

		for (j = 0; j < task->num_pvafences[k]; j++)
			seq_printf(s, "    postfence %d: \n\t"
				"syncpoint_index=%u, syncpoint_value=%u\n",
				j,
				task->pvafences[k][j].fence.syncpoint_index,
				task->pvafences[k][j].fence.syncpoint_value);


	}
	mutex_unlock(&queue->list_lock);
}

static int pva_task_submit_channel_ccq(struct pva_submit_task *task,
				       u32 *thresh)
{
	struct nvhost_queue *queue = task->queue;
	u64 fifo_flags = PVA_FIFO_INT_ON_ERR;
	u64 fifo_cmd = pva_fifo_submit(queue->id,
				       task->dma_addr,
				       fifo_flags);
	u32 syncpt_wait_ids[PVA_MAX_PREFENCES];
	u32 syncpt_wait_thresh[PVA_MAX_PREFENCES];
	unsigned int i;
	u32 cmdbuf[4];
	int err = 0;

	/* Pick up fences... */
	for (i = 0; i < task->num_prefences; i++) {
		/* ..and ensure that we have only syncpoints present */
		if (task->prefences[i].type != NVDEV_FENCE_TYPE_SYNCPT)
			return -EINVAL;

		/* Put fences into a separate array */
		syncpt_wait_ids[i] =
			task->prefences[i].syncpoint_index;
		syncpt_wait_thresh[i] =
			task->prefences[i].syncpoint_value;
	}

	/* A simple command buffer: Write two words into the ccq
	 * register
	 */
	cmdbuf[0] = nvhost_opcode_setpayload(2);
	cmdbuf[1] = nvhost_opcode_nonincr_w(cfg_ccq_r() >> 2);
	cmdbuf[2] = (u32)(fifo_cmd >> 32);
	cmdbuf[3] = (u32)(fifo_cmd & 0xffffffff);

	/* Submit the command buffer and waits to channel */
	err = nvhost_queue_submit_to_host1x(queue,
					    cmdbuf,
					    ARRAY_SIZE(cmdbuf),
					    1,
					    syncpt_wait_ids,
					    syncpt_wait_thresh,
					    task->num_prefences,
					    thresh);
	return err;
}

static int pva_task_submit_mmio_ccq(struct pva_submit_task *task,
				    u32 *thresh)
{
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	struct nvhost_queue *queue = task->queue;
	u32 old_maxval, new_maxval;
	u64 fifo_flags = PVA_FIFO_INT_ON_ERR;
	u64 fifo_cmd = pva_fifo_submit(queue->id,
				       task->dma_addr,
				       fifo_flags);
	int err = 0;

	/* Increment syncpoint to capture threshold */
	old_maxval = nvhost_syncpt_read_maxval(host1x_pdev, queue->syncpt_id);
	new_maxval = nvhost_syncpt_incr_max_ext(host1x_pdev,
						queue->syncpt_id,
						task->fence_num);

	err = pva_ccq_send(task->pva, fifo_cmd);
	if (err < 0)
		goto err_submit;

	*thresh = new_maxval;

	return 0;

err_submit:
	nvhost_syncpt_set_maxval(host1x_pdev, queue->syncpt_id, old_maxval);

	return err;
}

static int pva_task_submit_mailbox(struct pva_submit_task *task,
				   u32 *thresh)
{
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	struct nvhost_queue *queue = task->queue;
	struct pva_mailbox_status_regs status;
	u32 old_maxval, new_maxval;
	struct pva_cmd cmd;
	u32 flags, nregs;
	int err = 0;

	/* Construct submit command */
	flags = PVA_CMD_INT_ON_ERR | PVA_CMD_INT_ON_COMPLETE;
	nregs = pva_cmd_submit(&cmd, queue->id,
				task->dma_addr, flags);

	/* Increment syncpoint to capture threshold */
	old_maxval = nvhost_syncpt_read_maxval(host1x_pdev, queue->syncpt_id);
	new_maxval = nvhost_syncpt_incr_max_ext(host1x_pdev,
						queue->syncpt_id,
						task->fence_num);

	/* Submit request to PVA and wait for response */
	err = pva_mailbox_send_cmd_sync(task->pva, &cmd, nregs, &status);
	if (err < 0) {
		nvhost_warn(&task->pva->pdev->dev,
			"Failed to submit task: %d", err);
		goto err_submit;
	}

	/* Ensure that response is valid */
	if (status.error != PVA_ERR_NO_ERROR) {
		nvhost_warn(&task->pva->pdev->dev, "PVA task rejected: %u",
			status.error);
		err = -EINVAL;
		goto err_submit;
	}

	*thresh = new_maxval;

	return 0;

err_submit:
	nvhost_syncpt_set_maxval(host1x_pdev, queue->syncpt_id, old_maxval);

	return err;
}

static int pva_task_submit(struct pva_submit_task *task,
			   u32 *task_thresh)
{
	struct platform_device *host1x_pdev =
			to_platform_device(task->pva->pdev->dev.parent);
	struct nvhost_queue *queue = task->queue;
	u32 thresh = 0;
	u64 timestamp;
	int err = 0;

	nvhost_dbg_info("Submitting task %p (0x%llx)", task,
			(u64)task->dma_addr);

	/* Get a reference of the queue to avoid it being reused. It
	 * gets freed in the callback...
	 */
	nvhost_queue_get(queue);

	/* Turn on the hardware */
	err = nvhost_module_busy(task->pva->pdev);
	if (err)
		goto err_module_busy;

	/*
	 * TSC timestamp is same as CNTVCT. Task statistics are being
	 * reported in TSC ticks.
	 */
	timestamp = arch_counter_get_cntvct();

	/* Choose the submit policy based on the mode */
	switch (task->pva->submit_mode) {
	case PVA_SUBMIT_MODE_MAILBOX:
		err = pva_task_submit_mailbox(task, &thresh);
		break;

	case PVA_SUBMIT_MODE_MMIO_CCQ:
		err = pva_task_submit_mmio_ccq(task, &thresh);
		break;

	case PVA_SUBMIT_MODE_CHANNEL_CCQ:
		err = pva_task_submit_channel_ccq(task, &thresh);
		break;
	}

	if (err < 0)
		goto err_submit;

	/* Record task prefences */
	nvhost_eventlib_log_fences(task->pva->pdev,
				   queue->syncpt_id,
				   thresh,
				   task->prefences,
				   task->num_prefences,
				   NVDEV_FENCE_KIND_PRE,
				   timestamp);

	nvhost_eventlib_log_submit(task->pva->pdev,
				   queue->syncpt_id,
				   thresh,
				   timestamp);

	task->syncpt_thresh = thresh;

	nvhost_dbg_info("Postfence id=%u, value=%u",
			queue->syncpt_id, thresh);

	*task_thresh = thresh;

	/* Going to be linked so obtain the reference */
	kref_get(&task->ref);

	/*
	 * Tasks in the queue list can be modified by the interrupt handler.
	 * Adding the task into the list must be the last step before
	 * registering the interrupt handler.
	 */
	mutex_lock(&queue->list_lock);
	list_add_tail(&task->node, &queue->tasklist);
	mutex_unlock(&queue->list_lock);

	/*
	 * Register the interrupt handler. This must be done after adding
	 * the tasks into the queue since otherwise we may miss the completion
	 * event.
	 */
	WARN_ON(nvhost_intr_register_notifier(host1x_pdev,
					      queue->syncpt_id, thresh,
					      pva_queue_update, queue));

	return err;

err_submit:
	nvhost_module_idle(task->pva->pdev);
err_module_busy:
	nvhost_queue_put(queue);
	return err;
}

static int pva_queue_submit(struct nvhost_queue *queue, void *args)
{
	struct pva_submit_tasks *task_header = args;
	int err = 0;
	int i;

	for (i = 0; i < task_header->num_tasks; i++) {
		struct pva_submit_task *task = task_header->tasks[i];
		u32 *thresh = &task_header->task_thresh[i];

		task->fence_num = 0;

		/* First, dump the task that we are submitting */
		pva_task_dump(task);

		/* Pin job memory */
		err = pva_task_pin_mem(task);
		if (err < 0)
			break;

		/* Write the task data */
		pva_task_write(task, false);

		err = pva_task_submit(task, thresh);
		if (err < 0)
			break;
	}

	return err;
}

static int pva_queue_set_attribute(struct nvhost_queue *queue, void *args)
{
	uint32_t flags = PVA_CMD_INT_ON_ERR | PVA_CMD_INT_ON_COMPLETE;
	struct pva_queue_set_attribute *set_attr = args;
	struct pva_queue_attribute *attr = set_attr->attr;
	struct pva_mailbox_status_regs status;
	struct pva_cmd cmd;
	int err = 0;
	u32 nregs;

	nregs = pva_cmd_set_queue_attributes(&cmd, queue->id, attr->id,
			attr->value,
			flags);

	/* Submit request to PVA and wait for response */
	if (set_attr->bootup)
		err = pva_mailbox_send_cmd_sync_locked(set_attr->pva,
						       &cmd,
						       nregs,
						       &status);
	else
		err = pva_mailbox_send_cmd_sync(set_attr->pva,
						&cmd,
						nregs,
						&status);
	if (err < 0) {
		nvhost_warn(&set_attr->pva->pdev->dev,
			    "Failed to set attributes: %d\n",
			    err);
		goto end;
	}

	/* Ensure that response is valid */
	if (status.error != PVA_ERR_NO_ERROR) {
		nvhost_warn(&set_attr->pva->pdev->dev,
			    "PVA Q attribute rejected: %u\n",
			    status.error);
		err = -EINVAL;
	}

 end:
	return err;
}

static void pva_queue_cleanup_fence(struct nvdev_fence *fence,
				    struct pva_parameter_ext *fence_ext)
{
	struct dma_buf *dmabuf;
	u8 *dmabuf_cpuva;
	u32 *fence_cpuva;

	if (fence->type != NVDEV_FENCE_TYPE_SEMAPHORE)
		return;

	dmabuf = fence_ext->dmabuf;
	dmabuf_cpuva = dma_buf_vmap(dmabuf);

	if (!dmabuf_cpuva)
		return;

	if (!(fence->semaphore_offset % 4))
		return;

	fence_cpuva = (void *)&dmabuf_cpuva[fence->semaphore_offset];
	*fence_cpuva = fence->semaphore_value;

	dma_buf_vunmap(dmabuf, dmabuf_cpuva);
}

static void pva_queue_cleanup_status(struct pva_status_handle *status_h,
				     struct pva_parameter_ext *status_h_ext)
{
	struct dma_buf *dmabuf = status_h_ext->dmabuf;
	u8 *dmabuf_cpuva = dma_buf_vmap(dmabuf);
	struct nvhost_notification *status_ptr;

	if (!dmabuf_cpuva)
		return;

	status_ptr = (void *)&dmabuf_cpuva[status_h->offset];
	status_ptr->status = 0x8888;

	dma_buf_vunmap(dmabuf, dmabuf_cpuva);
}

static void pva_queue_cleanup(struct nvhost_queue *queue,
			      struct pva_submit_task *task)
{
	struct platform_device *pdev = queue->pool->pdev;
	struct nvhost_master *host = nvhost_get_host(pdev);
	bool expired = nvhost_syncpt_is_expired(&host->syncpt,
						queue->syncpt_id,
						task->syncpt_thresh);
	unsigned int i;
	unsigned int j;

	/*
	 * Ensure that there won't be communication with PVA for
	 * checking the task status
	 */
	task->invalid = true;

	/* Ignore expired fences */
	if (expired)
		return;

	/* Write task status first */
	for (i = 0; i < task->num_output_task_status; i++)
		pva_queue_cleanup_status(task->output_task_status,
					 task->output_task_status_ext);

	/* Finish up non-syncpoint fences */
	for (i = 0; i < PVA_MAX_FENCE_TYPES; i++) {
		for (j = 0; j < task->num_pvafences[i]; j++) {
			pva_queue_cleanup_fence(&task->pvafences[i][j].fence,
					&task->pvafences_sema_ext[i][j]);
		}
	}

	/* Finish syncpoint increments to release waiters */
	nvhost_syncpt_cpu_incr_ext(pdev, queue->syncpt_id);
}

static int pva_queue_abort(struct nvhost_queue *queue)
{
	struct pva_submit_task *task;

	mutex_lock(&queue->list_lock);

	list_for_each_entry(task, &queue->tasklist, node)
		pva_queue_cleanup(queue, task);

	mutex_unlock(&queue->list_lock);

	return 0;
}

struct nvhost_queue_ops pva_queue_ops = {
	.abort = pva_queue_abort,
	.submit = pva_queue_submit,
	.get_task_size = pva_task_get_memsize,
	.dump = pva_queue_dump,
	.set_attribute = pva_queue_set_attribute,
};
