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
#ifndef NVGPU_ATOMIC_H
#define NVGPU_ATOMIC_H

#ifdef __KERNEL__
#include <nvgpu/linux/atomic.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/atomic.h>
#else
#include <nvgpu_rmos/include/atomic.h>
#endif

#define NVGPU_ATOMIC_INIT(i)	__nvgpu_atomic_init(i)
#define NVGPU_ATOMIC64_INIT(i)	__nvgpu_atomic64_init(i)

static inline void nvgpu_atomic_set(nvgpu_atomic_t *v, int i)
{
	__nvgpu_atomic_set(v, i);
}
static inline int nvgpu_atomic_read(nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_read(v);
}
static inline void nvgpu_atomic_inc(nvgpu_atomic_t *v)
{
	__nvgpu_atomic_inc(v);
}
static inline int nvgpu_atomic_inc_return(nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_inc_return(v);
}
static inline void nvgpu_atomic_dec(nvgpu_atomic_t *v)
{
	 __nvgpu_atomic_dec(v);
}
static inline int nvgpu_atomic_dec_return(nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_dec_return(v);
}
static inline int nvgpu_atomic_cmpxchg(nvgpu_atomic_t *v, int old, int new)
{
	return __nvgpu_atomic_cmpxchg(v, old, new);
}
static inline int nvgpu_atomic_xchg(nvgpu_atomic_t *v, int new)
{
	return __nvgpu_atomic_xchg(v, new);
}
static inline bool nvgpu_atomic_inc_and_test(nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_inc_and_test(v);
}
static inline bool nvgpu_atomic_dec_and_test(nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_dec_and_test(v);
}
static inline bool nvgpu_atomic_sub_and_test(int i, nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_sub_and_test(i, v);
}
static inline int nvgpu_atomic_add_return(int i, nvgpu_atomic_t *v)
{
	return __nvgpu_atomic_add_return(i, v);
}
static inline int nvgpu_atomic_add_unless(nvgpu_atomic_t *v, int a, int u)
{
	return __nvgpu_atomic_add_unless(v, a, u);
}
static inline void nvgpu_atomic64_set(nvgpu_atomic64_t *v, long i)
{
	return  __nvgpu_atomic64_set(v, i);
}
static inline long nvgpu_atomic64_read(nvgpu_atomic64_t *v)
{
	return  __nvgpu_atomic64_read(v);
}
static inline void nvgpu_atomic64_add(long x, nvgpu_atomic64_t *v)
{
	__nvgpu_atomic64_add(x, v);
}
static inline void nvgpu_atomic64_inc(nvgpu_atomic64_t *v)
{
	__nvgpu_atomic64_inc(v);
}
static inline long nvgpu_atomic64_inc_return(nvgpu_atomic64_t *v)
{
	return __nvgpu_atomic64_inc_return(v);
}
static inline void nvgpu_atomic64_dec(nvgpu_atomic64_t *v)
{
	__nvgpu_atomic64_dec(v);
}
static inline void nvgpu_atomic64_dec_return(nvgpu_atomic64_t *v)
{
	__nvgpu_atomic64_dec_return(v);
}
static inline long nvgpu_atomic64_cmpxchg(nvgpu_atomic64_t *v, long old,
					long new)
{
	return __nvgpu_atomic64_cmpxchg(v, old, new);
}
static inline void nvgpu_atomic64_sub(long x, nvgpu_atomic64_t *v)
{
	__nvgpu_atomic64_sub(x, v);
}
static inline long nvgpu_atomic64_sub_return(long x, nvgpu_atomic64_t *v)
{
	return __nvgpu_atomic64_sub_return(x, v);
}

#endif /* NVGPU_ATOMIC_H */
