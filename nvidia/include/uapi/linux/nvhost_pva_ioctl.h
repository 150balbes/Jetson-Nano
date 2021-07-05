/*
 * Tegra PVA Driver ioctls
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program;  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LINUX_NVHOST_PVA_IOCTL_H
#define __LINUX_NVHOST_PVA_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>
#include "nvdev_fence.h"

#if !defined(__KERNEL__)
#define __user
#endif

#define NVHOST_PVA_IOCTL_MAGIC 'P'

/**
 * struct pva_characteristics_req - Request filling of characteristics struct
 *
 * @characteristics: pointer to be filled with characteristics
 * @characteristics_size: size in bytes
 * @characteristics_filled: reserved(set to zero)
 *
 */
struct pva_characteristics_req {
	__u64 characteristics;
	__u64 characteristics_size;
	__u64 characteristics_filled;
};

/**
 * struct pva_characteristics - the information of the pva cluster
 *
 * @num_vpu: number of vpu per pva
 * @vpu_generation: vpu hardware generation
 * @num_queues: number of queues per pva
 * @submit_mode: PVA submission mode
 * @pva_r5_revision: PVA R5 firmware revision
 * @pva_compat_version: Earliest version supporting the firmware
 * @pva_revision: PVA revision
 * @pva_built_on: Firmware build information
 *
 */
struct pva_characteristics {
	__u8 num_vpu;
	__u8 vpu_generation;
	__u8 num_queues;
#define PVA_CHARACTERISTCS_IOCTL_SUBMIT_MODE_MAILBOX		0
#define PVA_CHARACTERISTCS_IOCTL_SUBMIT_MODE_MMIO_CCQ		1
#define PVA_CHARACTERISTCS_IOCTL_SUBMIT_MODE_CHANNEL_CCQ	2
	__u8 submit_mode;
	__u32 pva_r5_version;
	__u32 pva_compat_version;
	__u32 pva_revision;
	__u32 pva_built_on;
};

/**
 * struct pva_pin_unpin_args - buffer handles to pin or unpin
 *
 * @buffers: Pointer to the table of u32
 * @num_buffers: elements in the buffer table
 * @reserved: reserved
 *
 * Used to deliver information about the buffer handles that should be
 * be pinned into (or unpinned from) the PVA address space.
 *
 */
struct pva_pin_unpin_args {
	__u64 buffers;
	__u32 num_buffers;
	__u32 reserved;
};

#define PVA_MAX_PIN_BUFFERS	64

/**
 * struct pva_memory_handle - A handle to PVA pointer
 *
 * @handle: Handle to a dmabuf that holds the data
 * @offset: An offset within the buffer to the data within the buffer
 */
struct pva_memory_handle {
	__u32 handle;
	__u32 offset;
};

/**
 * struct pva_ioctl_status_handle - A handle to a status structure
 *
 * @handle: Handle to a dmabuf that holds the status buffer
 * @offset: An offset within the buffer to the status structure.
 */
struct pva_status_handle {
	__u32 handle;
	__u32 offset;
};

/**
 * struct pva_ioctl_surface - The surface descriptor
 *
 * @format: Surface pixel format
 * @surface_handle: Memory handle that holds the surface
 * @surface_offset: Offset within the surface memory buffer to the surface
 * @roi_handle: Memory handle that holds the ROI
 * @roi_offset: Offset within the ROI memory buffer to the ROI
 * @surface_stride: Offset between planes in bytes
 * @line_stride: Offset between two consequent lines in bytes.
 * @depth: Number of planes in the surface
 * @width: Width of the surface
 * @height: Height of the surface
 * @layout: Surface layout (pitch linear, block linear)
 * @block_height_log2: Block height
 *
 * This structure defines a list of surfaces to be delivered for
 * PVA.
 */
struct pva_surface {
	__u64 format;
	__u32 surface_handle;
	__u32 surface_offset;
	__u32 roi_handle;
	__u32 roi_offset;
	__u32 surface_stride;
	__u32 line_stride;
	__u32 depth;
	__u32 width;
	__u32 height;
	__u16 layout;
	__u16 block_height_log2;
};

/**
 * struct pva_ioct_task_parameter - Parameter structure for a task
 *
 * @handle: Memory handle including the parameter array. This field shall be
 *	    used in cases where the UMD prepares the data in advance to a
 *	    shared buffer or the input data is prepared by the upstream engine.
 * @offset: Offset within the memory handle to the parameter array
 *
 * The parameter descriptor defines a single parameter array that is
 * received. The handle and offset is translated into IOVA by the kernel
 * driver and delivered to PVA.
 */
struct pva_task_parameter {
	__u32 handle;
	__u32 offset;
};

/**
 * struct pva_ioctl_fence structure for passing fence information
 *
 * NOTE: this will be removed soon, please use generic fence type
 * from nvdev_fence.h
 */
struct pva_fence {
	__u32 type;
#define PVA_FENCE_TYPE_SYNCPT		0
#define PVA_FENCE_TYPE_SYNC_FD		1
#define PVA_FENCE_TYPE_SEMAPHORE	2
#define PVA_FENCE_TYPE_SEMAPHORE_TS	3
	__u32 syncpoint_index;
	__u32 syncpoint_value;
	__u32 sync_fd;
	__u32 semaphore_handle;
	__u32 semaphore_offset;
	__u32 semaphore_value;
};
/**
 * PVA extended fence
 *
 * @param type		fence type
 * @param fence		nvdev_fence
 * @param ts_buf_handle	Handle of timestamp buffer
 */
struct nvpva_fence {
	 __u32 type;
#define PVA_FENCE_PRE   1U
#define PVA_FENCE_SOT_V 2U
#define PVA_FENCE_SOT_R 3U
#define PVA_FENCE_EOT_V 4U
#define PVA_FENCE_EOT_R 5U
#define PVA_FENCE_POST  6U
	 struct pva_memory_handle ts_buf_ptr;
	 struct nvdev_fence fence;
};

#define PVA_MAX_TASKS			1
#define PVA_MAX_PREFENCES		8
#define PVA_MAX_POSTFENCES		8
#define PVA_MAX_FENCE_TYPES		7
#define PVA_MAX_FENCES_PER_TYPE		8
#define PVA_MAX_INPUT_STATUS		8
#define PVA_MAX_OUTPUT_STATUS		8
#define PVA_MAX_INPUT_SURFACES		8
#define PVA_MAX_OUTPUT_SURFACES		8
#define PVA_MAX_POINTERS		128
#define PVA_MAX_PRIMARY_PAYLOAD_SIZE	4096

/**
 * struct pva_ioctl_submit_task - Describe a task for PVA
 *
 * @num_prefences: Number of pre-fences in this task
 * @num_postfences: Number of post-fences in this task
 * @num_input_surfaces: Number of input surfaces
 * @num_output_surfaces: Number of output surfaces
 * @num_input_task_status: Number of input task status structures
 * @num_output_task_status: Number of output task status structures
 * @reserved: Reserved for future usage.
 * @timeout: Latest Unix time when the task must complete. 0 if disabled.
 * @prefences: Pointer to pre-fence structures
 * @postfences: Pointer to post-fence structures
 * @input_surfaces: Pointer to input surfaces
 * @input_scalars: Information for input scalars
 * @output_surfaces: Pointer to output surfaces
 * @output_scalars: Information for output scalars
 * @input_task_status: Pointer to input status structure
 * @output_task_status: Pointer to output status structure
 *
 * This structure is used for delivering information that is required to
 * finish a single task on PVA.
 *
 */
struct pva_ioctl_submit_task {
	__u8 num_prefences;
	__u8 redserved1;
	__u8 num_input_surfaces;
	__u8 num_output_surfaces;
	__u8 num_input_task_status;
	__u8 num_output_task_status;
	__u16 num_pointers;
	__u64 pointers;
	__u32 primary_payload_size;
	__u32 operation;
	__u64 timeout;
	__u64 prefences;
	__u8 reserved2[8];
	__u64 input_surfaces;
	struct pva_task_parameter input_scalars;
	__u64 primary_payload;
	__u8 reserved3[8];
	__u64 output_surfaces;
	struct pva_task_parameter output_scalars;
	__u64 num_pva_ts_buffers;
	__u64 num_pvafences;
	__u64 pvafences;
	__u64 input_task_status;
	__u64 output_task_status;
};
/**
 * struct pva_submit_args - submit tasks to PVA
 *
 * @tasks: Pointer to a list of tasks structures
 * @flags: Flags for the given tasks
 * @num_tasks: Number of tasks in the list
 * @version: Version of the task structure.
 *
 * This ioctl is used for submitting tasks to PVA. The given structures
 * are modified to include information about post-fences.
 *
 */
struct pva_ioctl_submit_args {
	__u64 tasks;
	__u16 flags;
	__u16 num_tasks;
	__u32 version;
};

/**
 * struct pva_ioctl_queue_attr - set queue attributes
 *
 * @attr_id: Attribute id which defines the attribute to be set
 * @reserved: reserved
 * @attr_val: The value to be set for the attribute
 *
 * This ioctl is used for setting attributes for a queue with id queue_id
 * on the R5.
 *
 */
struct pva_ioctl_queue_attr {
	__u16 id;
	__u16 reserved;
	__u32 val;
};

/**
 * struct pva_ioctl_vpu_func_table - ioctl vpu function table entries
 *
 * @addr: Userspace address space to which the function table needs to be copied
 * @entries: The number of entries in the vpu table
 * @size: Size of the user buffer passed/ Size of the function table
 *
 * This ioctl is used to fetch the VPU function table available on a PVA, which
 * is copied to user space buffer starting at "addr" with size "size". Once the
 * function table is copied the the number of entries is updated along with the
 * size of the vpu function table.
 *
 */
struct pva_ioctl_vpu_func_table {
	__u64 addr;
	__u32 entries;
	__u32 size;
};

/**
 * enum pva_clk_type - Clock type identifier while setting the frequency.
 *
 * PVA KMD supports three request types: Minimum, bandwidth and bandwidth
 * (kHz). All bandwidth requests are summed up and treated as one of the
 * minimum frequency requests. KMD then takes the maximum over all minimum
 * requests and tries to set the frequency to PVA.
 */
enum pva_clk_type {
	PVA_CLOCK	= 0,
	PVA_BW		= 1,
	PVA_BW_KHZ	= 3,
};

/**
 * struct pva_ioctl_rate - Requesting PVA frequency update
 *
 * @param rate: Requested rate
 * @param type: Type of the request according to pva_clk_type
 * @param reserved: Reserved for future usage. Must be 0.
 */
struct pva_ioctl_rate {
	__u64 rate;
	__u32 type;
	__u32 reserved;
};

#define PVA_IOCTL_CHARACTERISTICS	\
	_IOWR(NVHOST_PVA_IOCTL_MAGIC, 1, struct pva_characteristics_req)
#define PVA_IOCTL_PIN	\
	_IOW(NVHOST_PVA_IOCTL_MAGIC, 2, struct pva_pin_unpin_args)
#define PVA_IOCTL_UNPIN	\
	_IOW(NVHOST_PVA_IOCTL_MAGIC, 3, struct pva_pin_unpin_args)
#define PVA_IOCTL_SUBMIT	\
	_IOW(NVHOST_PVA_IOCTL_MAGIC, 4, struct pva_ioctl_submit_args)
#define PVA_IOCTL_SET_QUEUE_ATTRIBUTES	\
	_IOW(NVHOST_PVA_IOCTL_MAGIC, 5, struct pva_ioctl_queue_attr)
#define PVA_IOCTL_COPY_VPU_FUNCTION_TABLE	\
	_IOWR(NVHOST_PVA_IOCTL_MAGIC, 6, struct pva_ioctl_vpu_func_table)
#define PVA_IOCTL_SET_RATE	\
	_IOWR(NVHOST_PVA_IOCTL_MAGIC, 7, struct pva_ioctl_rate)
#define PVA_IOCTL_GET_RATE	\
	_IOWR(NVHOST_PVA_IOCTL_MAGIC, 8, struct pva_ioctl_rate)


#define NVHOST_PVA_IOCTL_LAST _IOC_NR(PVA_IOCTL_GET_RATE)
#define NVHOST_PVA_IOCTL_MAX_ARG_SIZE sizeof(struct pva_characteristics_req)

#endif /* __LINUX_NVHOST_PVA_IOCTL_H */

