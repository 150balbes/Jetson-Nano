/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_POSIX_LOCK_H__
#define __NVGPU_POSIX_LOCK_H__

#include <stdlib.h>

#include <pthread.h>

/*
 * All locks for posix nvgpu are just pthread locks. There's not a lot of reason
 * to have real spinlocks in userspace since we aren't using real HW or running
 * perf critical code where a sleep could be devestating.
 *
 * This could be revisited later, though.
 */
struct __nvgpu_posix_lock {
	pthread_mutex_t mutex;
};

static inline void __nvgpu_posix_lock_acquire(struct __nvgpu_posix_lock *lock)
{
	pthread_mutex_lock(&lock->mutex);
}

static inline int __nvgpu_posix_lock_try_acquire(
	struct __nvgpu_posix_lock *lock)
{
	return pthread_mutex_trylock(&lock->mutex);
}

static inline void __nvgpu_posix_lock_release(struct __nvgpu_posix_lock *lock)
{
	pthread_mutex_unlock(&lock->mutex);
}

struct nvgpu_mutex {
	struct __nvgpu_posix_lock lock;
};

struct nvgpu_spinlock {
	struct __nvgpu_posix_lock lock;
};

struct nvgpu_raw_spinlock {
	struct __nvgpu_posix_lock lock;
};

#endif /* NVGPU_LOCK_LINUX_H */
