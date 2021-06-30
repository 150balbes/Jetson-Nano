/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
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

#ifndef UTILITY_H
#define UTILITY_H

#include <linux/printk.h>
#include <stdbool.h>

#define assert(x)						\
do {								\
	if (!(x)) {						\
		pr_err_once("assertion failed %s: %d: %s\n",	\
			    __FILE__, __LINE__, #x);		\
	}							\
} while (0)

#define GET_TOP32(x) ((x) >> 32ULL)
#define SET_TOP32(x) ((x) << 32ULL)
#define GET_LOW32(x) ((x) & 0x00000000FFFFFFFFULL)
#define SET_LOW32(x) ((x) & 0x00000000FFFFFFFFULL)

static inline void write_barrier(void)
{
#if defined(__aarch64__)
	asm volatile("dmb ishst" ::: "memory");
#elif defined(__arm__)
	asm volatile("dmb" ::: "memory");
#elif defined(__x86_64__)
	asm volatile("" ::: "memory");
#else
	assert(false);
#endif
}

static inline void read_barrier(void)
{
#if defined(__aarch64__)
	asm volatile("dmb ishld" ::: "memory");
#elif defined(__arm__)
	asm volatile("dmb" ::: "memory");
#elif defined(__x86_64__)
	asm volatile("" ::: "memory");
#else
	assert(false);
#endif
}

static inline uint64_t read64(volatile uint64_t *addr)
{
	uint64_t value;

#if defined(USE_GNU_ATOMICS_64)
	value = __sync_fetch_and_add(addr, 0ULL);
#elif (defined(__aarch64__) || defined(__x86_64__))
	assert(((uintptr_t)addr & (sizeof(uint64_t) - 1)) == 0);
	value = *addr;
#elif defined(__arm__)
	register uint32_t r2 asm("r2");
	register uint32_t r3 asm("r3");

	assert(((uintptr_t)addr & (sizeof(uint64_t) - 1)) == 0);
	asm volatile("ldrd %0, %1, [%2]"
		: "=r"(r2), "=r"(r3) : "r"(addr) : );

	value = ((uint64_t)r2 | ((uint64_t)r3 << 32));
#else
	assert(false);
	value = 0;
#endif

	return value;
}

static inline void write64(volatile uint64_t *addr, uint64_t value)
{
#if defined(USE_GNU_ATOMICS_64)
	uint64_t prev;

	while (1) {
		prev = __sync_fetch_and_add(addr, 0ULL);
		if (__sync_bool_compare_and_swap(addr, prev, value) == true)
			break;
	}
#elif (defined(__aarch64__) || defined(__x86_64__))
	assert(((uintptr_t)addr & (sizeof(uint64_t) - 1)) == 0);
	*addr = value;
#elif defined(__arm__)
	register uint32_t r2 asm("r2") = (uint32_t)value;
	register uint32_t r3 asm("r3") = (uint32_t)(value >> 32);

	assert(((uintptr_t)addr & (sizeof(uint64_t) - 1)) == 0);
	asm volatile("strd %0, %1, [%2]"
		: : "r"(r2), "r"(r3), "r"(addr) : "memory");
#else
	assert(false);
#endif
}

static inline uint64_t increment64(volatile uint64_t *addr)
{
	uint64_t prev;

	prev = read64(addr);
	write64(addr, prev + 1);
	return prev;
}

#endif
