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

#ifndef NVGPU_COND_H
#define NVGPU_COND_H

#ifdef __KERNEL__
#include <nvgpu/linux/cond.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/cond.h>
#else
#include <nvgpu_rmos/include/cond.h>
#endif

/*
 * struct nvgpu_cond
 *
 * Should be implemented per-OS in a separate library
 */
struct nvgpu_cond;

/**
 * nvgpu_cond_init - Initialize a condition variable
 *
 * @cond - The condition variable to initialize
 *
 * Initialize a condition variable before using it.
 */
int nvgpu_cond_init(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_signal - Signal a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up a waiter for a condition variable to check if its condition has been
 * satisfied.
 *
 * The waiter is using an uninterruptible wait.
 */
int nvgpu_cond_signal(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_signal_interruptible - Signal a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up a waiter for a condition variable to check if its condition has been
 * satisfied.
 *
 * The waiter is using an interruptible wait.
 */
int nvgpu_cond_signal_interruptible(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_broadcast - Signal all waiters of a condition variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up all waiters for a condition variable to check if their conditions
 * have been satisfied.
 *
 * The waiters are using an uninterruptible wait.
 */
int nvgpu_cond_broadcast(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_broadcast_interruptible - Signal all waiters of a condition
 * variable
 *
 * @cond - The condition variable to signal
 *
 * Wake up all waiters for a condition variable to check if their conditions
 * have been satisfied.
 *
 * The waiters are using an interruptible wait.
 */
int nvgpu_cond_broadcast_interruptible(struct nvgpu_cond *cond);

/**
 * nvgpu_cond_destroy - Destroy a condition variable
 *
 * @cond - The condition variable to destroy
 */
void nvgpu_cond_destroy(struct nvgpu_cond *cond);

#endif /* NVGPU_COND_H */
