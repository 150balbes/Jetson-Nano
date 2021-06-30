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

#ifndef NVGPU_OS_SCHED_H
#define NVGPU_OS_SCHED_H

#include <nvgpu/log.h>

struct gk20a;

/**
 * nvgpu_current_tid - Query the id of current thread
 *
 */
int nvgpu_current_tid(struct gk20a *g);

/**
 * nvgpu_current_pid - Query the id of current process
 *
 */
int nvgpu_current_pid(struct gk20a *g);

void __nvgpu_print_current(struct gk20a *g, const char *func_name, int line,
		void *ctx, enum nvgpu_log_type type);
/**
 * nvgpu_print_current - print the name of current calling process
 *
 */
#define nvgpu_print_current(g, ctx, type) \
	__nvgpu_print_current(g, __func__, __LINE__, ctx, type)

#endif /* NVGPU_OS_SCHED_H */
