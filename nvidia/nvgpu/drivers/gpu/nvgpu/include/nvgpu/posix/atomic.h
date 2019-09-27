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

#ifndef __NVGPU_POSIX_ATOMIC_H__
#define __NVGPU_POSIX_ATOMIC_H__

#include <nvgpu/types.h>

/*
 * Note: this code uses the GCC builtins to implement atomics.
 */

#define __atomic_cmpxchg(p, v, c)	__sync_val_compare_and_swap(p, v, c)
#define __atomic_and(p, v)		__sync_fetch_and_and(p, v)
#define __atomic_or(p, v)		__sync_fetch_and_or(p, v)

#define cmpxchg				__atomic_cmpxchg

/*
 * Place holders until real atomics can be implemented... Yay for GCC builtins!
 * We can use those eventually to define all the Linux atomic ops.
 *
 * TODO: make these _actually_ atomic!
 */
typedef struct __nvgpu_posix_atomic {
	int v;
} nvgpu_atomic_t;

typedef struct __nvgpu_posix_atomic64 {
	long v;
} nvgpu_atomic64_t;

#define __nvgpu_atomic_init(i)		{ i }
#define __nvgpu_atomic64_init(i)	{ i }

static inline void __nvgpu_atomic_set(nvgpu_atomic_t *v, int i)
{
	v->v = i;
}

static inline int __nvgpu_atomic_read(nvgpu_atomic_t *v)
{
	return v->v;
}

static inline void __nvgpu_atomic_inc(nvgpu_atomic_t *v)
{
	v->v++;
}

static inline int __nvgpu_atomic_inc_return(nvgpu_atomic_t *v)
{
	v->v++;
	return v->v;
}

static inline void __nvgpu_atomic_dec(nvgpu_atomic_t *v)
{
	v->v--;
}

static inline int __nvgpu_atomic_dec_return(nvgpu_atomic_t *v)
{
	v->v--;
	return v->v;
}

static inline int __nvgpu_atomic_cmpxchg(nvgpu_atomic_t *v, int old, int new)
{
	if (v->v == old)
		v->v = new;

	return v->v;
}

static inline int __nvgpu_atomic_xchg(nvgpu_atomic_t *v, int new)
{
	v->v = new;
	return new;
}

static inline bool __nvgpu_atomic_inc_and_test(nvgpu_atomic_t *v)
{
	v->v++;
	return v->v ? true : false;
}

static inline bool __nvgpu_atomic_dec_and_test(nvgpu_atomic_t *v)
{
	v->v--;
	return v->v ? true : false;
}

static inline bool __nvgpu_atomic_sub_and_test(int i, nvgpu_atomic_t *v)
{
	v->v -= i;
	return v->v ? true : false;
}

static inline int __nvgpu_atomic_add_return(int i, nvgpu_atomic_t *v)
{
	v->v += i;
	return v->v;
}

static inline int __nvgpu_atomic_add_unless(nvgpu_atomic_t *v, int a, int u)
{
	if (v->v != u)
		v->v += a;

	return v->v;
}

static inline void __nvgpu_atomic64_set(nvgpu_atomic64_t *v, long i)
{
	v->v = i;
}

static inline long __nvgpu_atomic64_read(nvgpu_atomic64_t *v)
{
	return v->v;
}

static inline void __nvgpu_atomic64_add(long x, nvgpu_atomic64_t *v)
{
	v->v += x;
}

static inline void __nvgpu_atomic64_inc(nvgpu_atomic64_t *v)
{
	v->v++;
}

static inline long __nvgpu_atomic64_inc_return(nvgpu_atomic64_t *v)
{
	v->v++;
	return v->v;
}

static inline void __nvgpu_atomic64_dec(nvgpu_atomic64_t *v)
{
	v->v--;
}

static inline long __nvgpu_atomic64_dec_return(nvgpu_atomic64_t *v)
{
	v->v--;
	return v->v;
}

static inline long __nvgpu_atomic64_cmpxchg(nvgpu_atomic64_t *v,
					long old, long new)
{

	if (v->v == old)
		v->v = new;

	return v->v;
}

static inline void __nvgpu_atomic64_sub(long x, nvgpu_atomic64_t *v)
{
	v->v -= x;
}

static inline long __nvgpu_atomic64_sub_return(long x, nvgpu_atomic64_t *v)
{
	v->v -= x;
	return v->v;
}

#endif
