/*
 * Eventlib interface for PVA
 *
 * Copyright (c) 2016-2018, NVIDIA Corporation.  All rights reserved.
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

#ifndef NVHOST_EVENTS_H
#define NVHOST_EVENTS_H

enum {
	NVHOST_SCHEMA_VERSION = 1
};

#define NVHOST_EVENT_PROVIDER_NAME "nv_mm_nvhost"

/* Marks that the task is submitted to hardware */
struct nvhost_task_submit {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;

	/* PID */
	u32 pid;

	/* TID */
	u32 tid;
} __packed;

/* Marks that the task is moving to execution */
struct nvhost_task_begin {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;
} __packed;

/* Marks that the task is completed */
struct nvhost_task_end {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;
} __packed;

struct nvhost_vpu_perf_counter {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;

	/* Identifier for the R5/VPU algorithm executed */
	u32 operation;

	/* Algorithm specific identifying tag for the perf counter */
	u32 tag;

	u32 count;
	u32 average;
	u64 variance;
	u32 minimum;
	u32 maximum;
} __packed;

/* Marks the pre/postfence associated with the task */
struct nvhost_task_fence {
	/* Engine class ID */
	u32 class_id;

	/* Kind (prefence or postfence) */
	u32 kind;

	/* Type (see nvdev_fence.h) */
	u32 type;

	/* Valid for NVDEV_FENCE_TYPE_SYNCPT only */
	u32 syncpoint_index;
	u32 syncpoint_value;

	/* Valid for NVDEV_FENCE_TYPE_SYNC_FD only */
	u32 sync_fd;

	/* Valid for NVDEV_FENCE_TYPE_SEMAPHORE
	   and NVDEV_FENCE_TYPE_SEMAPHORE_TS */
	u32 semaphore_handle;
	u32 semaphore_offset;
	u32 semaphore_value;
} __packed;

struct nvhost_pva_task_state {
	/* Engine class ID */
	u32 class_id;

	/* Syncpoint ID */
	u32 syncpt_id;

	/* Threshold for task completion */
	u32 syncpt_thresh;

	/* Identifier for the R5/VPU algorithm executed */
	u32 operation;
} __packed;

enum {
	/* struct nvhost_task_submit */
	NVHOST_TASK_SUBMIT = 0,

	/* struct nvhost_task_begin */
	NVHOST_TASK_BEGIN = 1,

	/* struct nvhost_task_end */
	NVHOST_TASK_END = 2,

	/* struct nvhost_task_fence */
	NVHOST_TASK_FENCE = 3,

	NVHOST_VPU_PERF_COUNTER_BEGIN = 4,
	NVHOST_VPU_PERF_COUNTER_END = 5,

	/* struct nvhost_pva_task_state */
	NVHOST_PVA_QUEUE_BEGIN = 6,
	NVHOST_PVA_QUEUE_END = 7,
	NVHOST_PVA_PREPARE_BEGIN = 8,
	NVHOST_PVA_PREPARE_END = 9,
	NVHOST_PVA_VPU0_BEGIN = 10,
	NVHOST_PVA_VPU0_END = 11,
	NVHOST_PVA_VPU1_BEGIN = 12,
	NVHOST_PVA_VPU1_END = 13,
	NVHOST_PVA_POST_BEGIN = 14,
	NVHOST_PVA_POST_END = 15,

	NVHOST_NUM_EVENT_TYPES = 16
};

enum {
	NVHOST_NUM_CUSTOM_FILTER_FLAGS = 0
};

#endif /* NVHOST_EVENTS_H */
