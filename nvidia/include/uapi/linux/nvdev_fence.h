/*
 * include/uapi/linux/nvdev_fence.h
 *
 * Tegra PVA/DLA fence support
 *
 * Copyright (c) 2018-2019, NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LINUX_NVDEV_FENCE_H
#define LINUX_NVDEV_FENCE_H

#include <linux/types.h>

/* used for the recording with keventlib */
enum nvdev_fence_kind {
	NVDEV_FENCE_KIND_PRE = 0,
	NVDEV_FENCE_KIND_POST
};

/**
 * struct nvdev_fence structure for passing fence information
 *
 * @type: Type of the fence (syncpoint, sync fd or semaphore)
 * @type: fence action (wait or signal)
 * @syncpoint_index: Syncpoint id
 * @syncpoint_value: Value of syncpoint id
 * @sync_fd: Linux sync FD handle
 * @semaphore_handle: File handle to the semaphore memory buffer
 * @semaphore_offset: Offset to the semaphore within the buffer
 * @semaphore_value: Value of the semaphore
 */
struct nvdev_fence {
	__u32 type;
#define NVDEV_FENCE_TYPE_SYNCPT       0
#define NVDEV_FENCE_TYPE_SYNC_FD      1
#define NVDEV_FENCE_TYPE_SEMAPHORE    2
#define NVDEV_FENCE_TYPE_SEMAPHORE_TS 3
	__u32 action;
#define NVDEV_FENCE_WAIT  	0
#define NVDEV_FENCE_SIGNAL	1
	__u32 syncpoint_index;
	__u32 syncpoint_value;
	__u32 sync_fd;
	__u32 semaphore_handle;
	__u32 semaphore_offset;
	__u32 semaphore_value;
};

#endif /* LINUX_NVDEV_FENCE_H */
