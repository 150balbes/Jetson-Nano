/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_COND_LINUX_H__
#define __NVGPU_COND_LINUX_H__

#include <linux/wait.h>
#include <linux/sched.h>

struct nvgpu_cond {
	bool initialized;
	wait_queue_head_t wq;
};

/**
 * NVGPU_COND_WAIT - Wait for a condition to be true
 *
 * @c - The condition variable to sleep on
 * @condition - The condition that needs to be true
 * @timeout_ms - Timeout in milliseconds, or 0 for infinite wait
 *
 * Wait for a condition to become true. Returns -ETIMEOUT if
 * the wait timed out with condition false.
 */
#define NVGPU_COND_WAIT(c, condition, timeout_ms) \
({\
	int ret = 0; \
	long _timeout_ms = timeout_ms;\
	if (_timeout_ms > 0) { \
		long _ret = wait_event_timeout((c)->wq, condition, \
						 msecs_to_jiffies(_timeout_ms)); \
		if (_ret == 0) \
			ret = -ETIMEDOUT; \
	} else { \
		wait_event((c)->wq, condition); \
	} \
	ret;\
})

/**
 * NVGPU_COND_WAIT_INTERRUPTIBLE - Wait for a condition to be true
 *
 * @c - The condition variable to sleep on
 * @condition - The condition that needs to be true
 * @timeout_ms - Timeout in milliseconds, or 0 for infinite wait
 *
 * Wait for a condition to become true. Returns -ETIMEOUT if
 * the wait timed out with condition false or -ERESTARTSYS on
 * signal.
 */
#define NVGPU_COND_WAIT_INTERRUPTIBLE(c, condition, timeout_ms) \
({ \
	int ret = 0; \
	long _timeout_ms = timeout_ms;\
	if (_timeout_ms > 0) { \
		long _ret = wait_event_interruptible_timeout((c)->wq, condition, \
						 msecs_to_jiffies(_timeout_ms)); \
		if (_ret == 0) \
			ret = -ETIMEDOUT; \
		else if (_ret == -ERESTARTSYS) \
			ret = -ERESTARTSYS; \
	} else { \
		ret = wait_event_interruptible((c)->wq, condition); \
	} \
	ret; \
})

#endif /* __NVGPU_LOCK_LINUX_H__ */
