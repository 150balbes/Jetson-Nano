/*
 * include/uapi/linux/nvhost_nvdla_ioctl.h
 *
 * Tegra NvDLA Driver
 *
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
 * this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UAPI_LINUX_NVHOST_NVDLA_IOCTL_H
#define __UAPI_LINUX_NVHOST_NVDLA_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#if !defined(__KERNEL__)
#define __user
#endif

/**
 * struct nvdla_queue_stat_args strcture
 *
 * @status		queue status flags
 *
 */
struct nvdla_queue_status_args {
#define NVDLA_QUEUE_FLAGS_SUSPEND	(1 << 0)
#define NVDLA_QUEUE_FLAGS_RESUME	(1 << 1)
	__u64 status;
};

/**
 * struct nvdla_ping_args structure for ping data
 *
 * @in_challenge	challenge data to be sent
 * @out_response	response/CRC on challenge data from engine
 *
 */
struct nvdla_ping_args {
	__u32 in_challenge;
	__u32 out_response;
};


/**
 * struct nvdla_pin_unpin_args strcture args for buffer pin/unpin
 *
 * @buffers		list of buffers to pin/unpin'ed
 * @num_buffers		number of buffers count
 * @reserved		reserved for future use
 *
 */
struct nvdla_pin_unpin_args {
	__u64 buffers;
	__u32 num_buffers;
	__u32 reserved;
};

/**
 * struct nvdla_submit_args structure for task submit
 *
 * @tasks		pointer to task list
 * @num_tasks		number of tasks count
 * @flags		flags for task submit, like atomic
 * @version		version of task structure
 *
 */
struct nvdla_submit_args {
	__u64 tasks;
	__u16 num_tasks;
#define MAX_TASKS_PER_SUBMIT		16
#define NVDLA_SUBMIT_FLAGS_ATOMIC	(1 << 0)
	__u16 flags;
	__u32 version;
};

/**
 * struct nvdla_get_fw_ver_args strcture
 *
 * @version	Firmware version
 *
 */
struct nvdla_get_fw_ver_args {
	__u32 version;
};

/**
 * struct nvdla_get_q_status_args strcture
 *
 * @id		queue id
 * @fence	fence assigned to queue
 *
 */
struct nvdla_get_q_status_args {
	__u32 id;
	__u64 fence;
};

/**
 * struct nvdla_mem_handle structure for memory handles
 *
 * @handle		handle to buffer allocated in userspace
 * @offset		offset in buffer
 *
 */
struct nvdla_mem_handle {
	__u32 handle;
	__u32 offset;
};

/**
 * struct nvdla_ioctl_submit_task structure for single task information
 *
 * @num_prefences		number of pre-fences in task
 * @num_postfences		number of post-fences in task
 * @num_input_task_status	number of input task status
 * @num_sof_task_status  	number of sof task status
 * @num_eof_task_status  	number of eof task status
 * @num_sof_timestamps   	number of sof timestamp
 * @num_eof_timestamps   	number of eof timestamp
 * @flags			flags for bitwise task info embeddeing
 * @reserved			reserved for future use
 * @prefences			pointer to pre-fence struct table
 * @postfences			pointer to post-fence struct table
 * @input_task_status		pointer to input task status struct table
 * @sof_task_status  		pointer to sof task status struct table
 * @eof_task_status  		pointer to eof task status struct table
 * @sof_timestamps   		pointer to sof timestamp handle list
 * @eof_timestamps   		pointer to eof timestamp handle list
 * @num_addresses		total number of addressed passed in structure
 * @address_list		pointer to address list
 * @timeout			task timeout
 *
 */
struct nvdla_ioctl_submit_task {
	__u8 num_prefences;
	__u8 num_postfences;
	__u8 num_input_task_status;
	__u8 num_sof_task_status;
	__u8 num_eof_task_status;
	__u8 num_sof_timestamps;
	__u8 num_eof_timestamps;
	__u8 reserved0[1];
#define NVDLA_MAX_BUFFERS_PER_TASK (6144)
	__u32 num_addresses;
	__u16 flags;
	__u16 reserved1;

	__u64 prefences;
	__u64 postfences;

	__u64 input_task_status;
	__u64 sof_task_status;
	__u64 eof_task_status;
	__u64 sof_timestamps;
	__u64 eof_timestamps;
	__u64 address_list;
	__u64 timeout;
};

/**
 * struct nvdla_ioctl_emu_submit_task structure for single emulator task
 * information
 *
 * @num_prefences 		number of pre-fences in task
 * @num_postfences		number of post-fences in task
 * @prefences     		pointer to pre-fence struct table
 * @postfences    		pointer to post-fence struct table
 *
 */
struct nvdla_ioctl_emu_submit_task {
	__u32 num_prefences;
	__u32 num_postfences;

	__u64 prefences;
	__u64 postfences;
};

/**
 * struct nvdla_fence structure for passing fence information
 *
 * NOTE: this will be removed soon, please use generic fence type
 * from nvdev_fence.h
 */
struct nvdla_fence {
	__u32 type;
#define NVDLA_FENCE_TYPE_SYNCPT		0
#define NVDLA_FENCE_TYPE_SYNC_FD	1
#define NVDLA_FENCE_TYPE_SEMAPHORE	2
#define NVDLA_FENCE_TYPE_TS_SEMAPHORE	3
	__u32 syncpoint_index;
	__u32 syncpoint_value;
	__u32 sync_fd;
	__u32 sem_handle;
	__u32 sem_offset;
	__u32 sem_val;
};

/**
 * struct nvdla_status_notify structure for passing status notify information
 *
 * @handle		handle to buffer allocated in userspace
 * @offset		offset in buffer
 * @status		status
 *
 */
struct nvdla_status_notify {
	__u32 handle;
	__u32 offset;
	__u32 status;
};

#define NVHOST_NVDLA_IOCTL_MAGIC 'D'

#define NVDLA_IOCTL_PING		\
		_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 1, struct nvdla_ping_args)
#define NVDLA_IOCTL_PIN   \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 2, struct nvdla_pin_unpin_args)
#define NVDLA_IOCTL_UNPIN \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 3, struct nvdla_pin_unpin_args)
#define NVDLA_IOCTL_SUBMIT \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 4, struct nvdla_submit_args)
#define NVDLA_IOCTL_SET_QUEUE_STATUS \
	_IOW(NVHOST_NVDLA_IOCTL_MAGIC, 5, struct nvdla_queue_status_args)
#define NVDLA_IOCTL_GET_FIRMWARE_VERSION \
	_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 6, struct nvdla_get_fw_ver_args)
#define NVDLA_IOCTL_GET_QUEUE_STATUS \
	_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 7, struct nvdla_get_q_status_args)
#define NVDLA_IOCTL_EMU_TASK_SUBMIT \
	_IOWR(NVHOST_NVDLA_IOCTL_MAGIC, 8, struct nvdla_submit_args)
#define NVDLA_IOCTL_LAST		\
		_IOC_NR(NVDLA_IOCTL_EMU_TASK_SUBMIT)

#define NVDLA_IOCTL_MAX_ARG_SIZE  \
		sizeof(struct nvdla_pin_unpin_args)

#endif /* __UAPI_LINUX_NVHOST_NVDLA_IOCTL_H */
