/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bug.h>
#include <nvgpu/thread.h>

#include <nvgpu/posix/thread.h>

/**
 * Use pthreads to mostly emulate the Linux kernel APIs. There are some things
 * that are quite different - especially the stop/should_stop notions. In user
 * space threads can send signals to one another but of course within the kernel
 * that is not as simple.
 *
 * This could use some nice debugging some day as well.
 */

/*
 * nvgpu thread functions return int. POSIX threads return void *. This little
 * wrapper takes the int returning nvgpu thread and instead passes that int back
 * through the void * pointer.
 */
static void *__nvgpu_posix_thread_wrapper(void *data)
{
	struct nvgpu_posix_thread_data *nvgpu = data;

	return ERR_PTR(nvgpu->fn(nvgpu->data));
}

int nvgpu_thread_create(struct nvgpu_thread *thread,
			void *data,
			int (*threadfn)(void *data), const char *name)
{
	int ret;

	BUG_ON(thread->running);

	memset(thread, 0, sizeof(*thread));

	/*
	 * By subtracting 1 the above memset ensures that we have a zero
	 * terminated string.
	 */
	strncpy(thread->tname, name, NVGPU_THREAD_POSIX_MAX_NAMELEN - 1);

	thread->nvgpu.data = data;
	thread->nvgpu.fn = threadfn;

	ret = pthread_create(&thread->thread, NULL,
			     __nvgpu_posix_thread_wrapper,
			     &thread->nvgpu);
	if (ret != 0)
		return ret;

#ifdef _GNU_SOURCE
	pthread_setname_np(thread->thread, thread->tname);
#endif

	thread->running = true;

	return 0;
}

void nvgpu_thread_stop(struct nvgpu_thread *thread)
{
	thread->should_stop = true;
}

bool nvgpu_thread_should_stop(struct nvgpu_thread *thread)
{
	return thread->should_stop;
}

bool nvgpu_thread_is_running(struct nvgpu_thread *thread)
{
	return thread->running;
}

void nvgpu_thread_join(struct nvgpu_thread *thread)
{
	(void) pthread_join(thread->thread, NULL);
}
