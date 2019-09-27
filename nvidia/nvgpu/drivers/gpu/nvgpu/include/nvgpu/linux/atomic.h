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
#ifndef __NVGPU_ATOMIC_LINUX_H__
#define __NVGPU_ATOMIC_LINUX_H__

#ifdef __KERNEL__
#include <linux/atomic.h>
#endif

typedef struct nvgpu_atomic {
  atomic_t atomic_var;
} nvgpu_atomic_t;

typedef struct nvgpu_atomic64 {
  atomic64_t atomic_var;
} nvgpu_atomic64_t;

#define __nvgpu_atomic_init(i)	{ ATOMIC_INIT(i) }
#define __nvgpu_atomic64_init(i)	{ ATOMIC64_INIT(i) }

static inline void __nvgpu_atomic_set(nvgpu_atomic_t *v, int i)
{
	atomic_set(&v->atomic_var, i);
}

static inline int __nvgpu_atomic_read(nvgpu_atomic_t *v)
{
	return atomic_read(&v->atomic_var);
}

static inline void __nvgpu_atomic_inc(nvgpu_atomic_t *v)
{
	atomic_inc(&v->atomic_var);
}

static inline int __nvgpu_atomic_inc_return(nvgpu_atomic_t *v)
{
	return atomic_inc_return(&v->atomic_var);
}

static inline void __nvgpu_atomic_dec(nvgpu_atomic_t *v)
{
	atomic_dec(&v->atomic_var);
}

static inline int __nvgpu_atomic_dec_return(nvgpu_atomic_t *v)
{
	return atomic_dec_return(&v->atomic_var);
}

static inline int __nvgpu_atomic_cmpxchg(nvgpu_atomic_t *v, int old, int new)
{
	return atomic_cmpxchg(&v->atomic_var, old, new);
}

static inline int __nvgpu_atomic_xchg(nvgpu_atomic_t *v, int new)
{
	return atomic_xchg(&v->atomic_var, new);
}

static inline bool __nvgpu_atomic_inc_and_test(nvgpu_atomic_t *v)
{
	return atomic_inc_and_test(&v->atomic_var);
}

static inline bool __nvgpu_atomic_dec_and_test(nvgpu_atomic_t *v)
{
	return atomic_dec_and_test(&v->atomic_var);
}

static inline bool __nvgpu_atomic_sub_and_test(int i, nvgpu_atomic_t *v)
{
	return atomic_sub_and_test(i, &v->atomic_var);
}

static inline int __nvgpu_atomic_add_return(int i, nvgpu_atomic_t *v)
{
	return atomic_add_return(i, &v->atomic_var);
}

static inline int __nvgpu_atomic_add_unless(nvgpu_atomic_t *v, int a, int u)
{
	return atomic_add_unless(&v->atomic_var, a, u);
}

static inline void __nvgpu_atomic64_set(nvgpu_atomic64_t *v, long i)
{
	atomic64_set(&v->atomic_var, i);
}

static inline long __nvgpu_atomic64_read(nvgpu_atomic64_t *v)
{
	return atomic64_read(&v->atomic_var);
}

static inline void __nvgpu_atomic64_add(long x, nvgpu_atomic64_t *v)
{
	atomic64_add(x, &v->atomic_var);
}

static inline void __nvgpu_atomic64_inc(nvgpu_atomic64_t *v)
{
	atomic64_inc(&v->atomic_var);
}

static inline long __nvgpu_atomic64_inc_return(nvgpu_atomic64_t *v)
{
	return atomic64_inc_return(&v->atomic_var);
}

static inline void __nvgpu_atomic64_dec(nvgpu_atomic64_t *v)
{
	atomic64_dec(&v->atomic_var);
}

static inline void __nvgpu_atomic64_dec_return(nvgpu_atomic64_t *v)
{
	atomic64_dec_return(&v->atomic_var);
}

static inline long __nvgpu_atomic64_cmpxchg(nvgpu_atomic64_t *v,
					long old, long new)
{
	return atomic64_cmpxchg(&v->atomic_var, old, new);
}

static inline void __nvgpu_atomic64_sub(long x, nvgpu_atomic64_t *v)
{
	atomic64_sub(x, &v->atomic_var);
}

static inline long __nvgpu_atomic64_sub_return(long x, nvgpu_atomic64_t *v)
{
	return atomic64_sub_return(x, &v->atomic_var);
}
#endif /*__NVGPU_ATOMIC_LINUX_H__ */
