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

#include <linux/wait.h>
#include <linux/sched.h>

#include <nvgpu/cond.h>

int nvgpu_cond_init(struct nvgpu_cond *cond)
{
	init_waitqueue_head(&cond->wq);
	cond->initialized = true;

	return 0;
}

void nvgpu_cond_destroy(struct nvgpu_cond *cond)
{
	cond->initialized = false;
}

int nvgpu_cond_signal(struct nvgpu_cond *cond)
{
	if (!cond->initialized)
		return -EINVAL;

	wake_up(&cond->wq);

	return 0;
}

int nvgpu_cond_signal_interruptible(struct nvgpu_cond *cond)
{
	if (!cond->initialized)
		return -EINVAL;

	wake_up_interruptible(&cond->wq);

	return 0;
}

int nvgpu_cond_broadcast(struct nvgpu_cond *cond)
{
	if (!cond->initialized)
		return -EINVAL;

	wake_up_all(&cond->wq);

	return 0;
}

int nvgpu_cond_broadcast_interruptible(struct nvgpu_cond *cond)
{
	if (!cond->initialized)
		return -EINVAL;

	wake_up_interruptible_all(&cond->wq);

	return 0;
}
