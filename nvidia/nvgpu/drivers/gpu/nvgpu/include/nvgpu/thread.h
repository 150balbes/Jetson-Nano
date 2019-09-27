/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_THREAD_H
#define NVGPU_THREAD_H

#ifdef __KERNEL__
#include <nvgpu/linux/thread.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/thread.h>
#else
#include <nvgpu_rmos/include/thread.h>
#endif

#include <nvgpu/types.h>

/**
 * nvgpu_thread_create - Create and run a new thread.
 *
 * @thread - thread structure to use
 * @data - data to pass to threadfn
 * @threadfn - Thread function
 * @name - name of the thread
 *
 * Create a thread and run threadfn in it. The thread stays alive as long as
 * threadfn is running. As soon as threadfn returns the thread is destroyed.
 *
 * threadfn needs to continuously poll nvgpu_thread_should_stop() to determine
 * if it should exit.
 */
int nvgpu_thread_create(struct nvgpu_thread *thread,
		void *data,
		int (*threadfn)(void *data), const char *name);

/**
 * nvgpu_thread_stop - Destroy or request to destroy a thread
 *
 * @thread - thread to stop
 *
 * Request a thread to stop by setting nvgpu_thread_should_stop() to
 * true and wait for thread to exit.
 */
void nvgpu_thread_stop(struct nvgpu_thread *thread);

/**
 * nvgpu_thread_should_stop - Query if thread should stop
 *
 * @thread
 *
 * Return true if thread should exit. Can be run only in the thread's own
 * context and with the thread as parameter.
 */
bool nvgpu_thread_should_stop(struct nvgpu_thread *thread);

/**
 * nvgpu_thread_is_running - Query if thread is running
 *
 * @thread
 *
 * Return true if thread is started.
 */
bool nvgpu_thread_is_running(struct nvgpu_thread *thread);

/**
 * nvgpu_thread_join - join a thread to reclaim resources
 * after it has exited
 *
 * @thread - thread to join
 *
 */
void nvgpu_thread_join(struct nvgpu_thread *thread);

#endif /* NVGPU_THREAD_H */
