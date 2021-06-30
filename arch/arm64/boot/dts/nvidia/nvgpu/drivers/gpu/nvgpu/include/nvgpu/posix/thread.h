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

#ifndef __NVGPU_POSIX_THREAD_H__
#define __NVGPU_POSIX_THREAD_H__

#include <pthread.h>

#include <nvgpu/types.h>

/*
 * Handles passing an nvgpu thread function into a posix thread.
 */
struct nvgpu_posix_thread_data {
	int (*fn)(void *data);
	void *data;
};

/*
 * For some reason POSIX only allows 16 bytes of name length.
 */
#define NVGPU_THREAD_POSIX_MAX_NAMELEN		16

struct nvgpu_thread {
	bool running;
	bool should_stop;
	pthread_t thread;
	struct nvgpu_posix_thread_data nvgpu;
	char tname[16];
};

#endif
