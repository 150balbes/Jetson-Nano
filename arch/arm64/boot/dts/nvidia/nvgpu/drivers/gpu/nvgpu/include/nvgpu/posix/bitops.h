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

#ifndef __NVGPU_POSIX_BITOPS_H__
#define __NVGPU_POSIX_BITOPS_H__

#include <nvgpu/types.h>

/*
 * Assume an 8 bit byte, of course.
 */
#define BITS_PER_BYTE	8UL
#define BITS_PER_LONG 	(__SIZEOF_LONG__ * BITS_PER_BYTE)
#define BITS_TO_LONGS(bits)			\
	(bits + (BITS_PER_LONG - 1) / BITS_PER_LONG)

/*
 * Deprecated; use the explicit BITxx() macros instead.
 */
#define BIT(i)		BIT64(i)

#define GENMASK(h, l) \
	(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define DECLARE_BITMAP(bmap, bits)		\
	unsigned long bmap[BITS_TO_LONGS(bits)]

#define for_each_set_bit(bit, addr, size) \
	for ((bit) = find_first_bit((addr), (size));		\
	     (bit) < (size);					\
	     (bit) = find_next_bit((addr), (size), (bit) + 1))

#define ffs(word)	__ffs(word)
#define ffz(word)	__ffs(~(word))
#define fls(word)	__fls(word)

/*
 * Clashes with symbols in libc it seems.
 */
#define __ffs(word)	__nvgpu_posix_ffs(word)
#define __fls(word)	__nvgpu_posix_fls(word)

unsigned long __nvgpu_posix_ffs(unsigned long word);
unsigned long __nvgpu_posix_fls(unsigned long word);

unsigned long find_first_bit(const unsigned long *addr, unsigned long size);
unsigned long find_next_bit(const unsigned long *addr, unsigned long size,
			    unsigned long offset);
unsigned long find_first_zero_bit(const unsigned long *addr,
				  unsigned long size);

bool test_bit(int nr, const volatile unsigned long *addr);
bool test_and_set_bit(int nr, volatile unsigned long *addr);
bool test_and_clear_bit(int nr, volatile unsigned long *addr);

/*
 * These two are atomic.
 */
void set_bit(int nr, volatile unsigned long *addr);
void clear_bit(int nr, volatile unsigned long *addr);

void bitmap_set(unsigned long *map, unsigned int start, int len);
void bitmap_clear(unsigned long *map, unsigned int start, int len);
unsigned long bitmap_find_next_zero_area_off(unsigned long *map,
					     unsigned long size,
					     unsigned long start,
					     unsigned int nr,
					     unsigned long align_mask,
					     unsigned long align_offset);
unsigned long bitmap_find_next_zero_area(unsigned long *map,
					 unsigned long size,
					 unsigned long start,
					 unsigned int nr,
					 unsigned long align_mask);

#endif
