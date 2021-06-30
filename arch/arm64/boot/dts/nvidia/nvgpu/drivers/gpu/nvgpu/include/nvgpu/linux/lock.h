/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_LOCK_LINUX_H
#define NVGPU_LOCK_LINUX_H

#include <linux/mutex.h>
#include <linux/spinlock.h>

struct nvgpu_mutex {
	struct mutex mutex;
};
struct nvgpu_spinlock {
	spinlock_t spinlock;
};
struct nvgpu_raw_spinlock {
	raw_spinlock_t spinlock;
};

static inline int nvgpu_mutex_init(struct nvgpu_mutex *mutex)
{
	mutex_init(&mutex->mutex);
	return 0;
};
static inline void nvgpu_mutex_acquire(struct nvgpu_mutex *mutex)
{
	mutex_lock(&mutex->mutex);
};
static inline void nvgpu_mutex_release(struct nvgpu_mutex *mutex)
{
	mutex_unlock(&mutex->mutex);
};
static inline int nvgpu_mutex_tryacquire(struct nvgpu_mutex *mutex)
{
	return mutex_trylock(&mutex->mutex);
};
static inline void nvgpu_mutex_destroy(struct nvgpu_mutex *mutex)
{
	mutex_destroy(&mutex->mutex);
};

static inline void nvgpu_spinlock_init(struct nvgpu_spinlock *spinlock)
{
	spin_lock_init(&spinlock->spinlock);
};
static inline void nvgpu_spinlock_acquire(struct nvgpu_spinlock *spinlock)
{
	spin_lock(&spinlock->spinlock);
};
static inline void nvgpu_spinlock_release(struct nvgpu_spinlock *spinlock)
{
	spin_unlock(&spinlock->spinlock);
};

static inline void nvgpu_raw_spinlock_init(struct nvgpu_raw_spinlock *spinlock)
{
	raw_spin_lock_init(&spinlock->spinlock);
};
static inline void nvgpu_raw_spinlock_acquire(struct nvgpu_raw_spinlock *spinlock)
{
	raw_spin_lock(&spinlock->spinlock);
};
static inline void nvgpu_raw_spinlock_release(struct nvgpu_raw_spinlock *spinlock)
{
	raw_spin_unlock(&spinlock->spinlock);
};

#endif /* NVGPU_LOCK_LINUX_H */
