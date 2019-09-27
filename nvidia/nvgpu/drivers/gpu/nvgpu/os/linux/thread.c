/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kthread.h>

#include <nvgpu/thread.h>
#include <nvgpu/timers.h>

int nvgpu_thread_proxy(void *threaddata)
{
	struct nvgpu_thread *thread = threaddata;
	int ret = thread->fn(thread->data);

	thread->running = false;
	return ret;
}

int nvgpu_thread_create(struct nvgpu_thread *thread,
		void *data,
		int (*threadfn)(void *data), const char *name)
{
	struct task_struct *task = kthread_create(nvgpu_thread_proxy,
			thread, name);
	if (IS_ERR(task))
		return PTR_ERR(task);

	thread->task = task;
	thread->fn = threadfn;
	thread->data = data;
	thread->running = true;
	wake_up_process(task);
	return 0;
};

void nvgpu_thread_stop(struct nvgpu_thread *thread)
{
	if (thread->task) {
		kthread_stop(thread->task);
		thread->task = NULL;
	}
};

bool nvgpu_thread_should_stop(struct nvgpu_thread *thread)
{
	return kthread_should_stop();
};

bool nvgpu_thread_is_running(struct nvgpu_thread *thread)
{
	return ACCESS_ONCE(thread->running);
};

void nvgpu_thread_join(struct nvgpu_thread *thread)
{
	while (ACCESS_ONCE(thread->running))
		nvgpu_msleep(10);
};
