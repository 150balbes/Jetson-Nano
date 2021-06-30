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
#ifndef NVGPU_TYPES_H
#define NVGPU_TYPES_H

#ifdef __KERNEL__
#include <linux/types.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/types.h>
#else
#include <nvgpu_rmos/include/types.h>
#endif

/*
 * These macros exist to make integer literals used in certain arithmetic
 * operations explicitly large enough to hold the results of that operation.
 * The following is an example of this.
 *
 * In MISRA the destination for a bitwise shift must be able to hold the number
 * of bits shifted. Otherwise the results are undefined. For example:
 *
 *   256U << 20U
 *
 * This is valid C code but the results of this _may_ be undefined if the size
 * of an unsigned by default is less than 24 bits (i.e 16 bits). The MISRA
 * checker sees the 256U and determines that the 256U fits in a 16 bit data type
 * (i.e a u16). Since a u16 has 16 bits, which is less than 20, this is an
 * issue.
 *
 * Of course most compilers these days use 32 bits for the default unsigned type
 * this is not a requirement. Moreover this same problem could exist like so:
 *
 *   0xfffffU << 40U
 *
 * The 0xfffffU is a 32 bit unsigned type; but we are shifting 40 bits which
 * overflows the 32 bit data type. So in this case we need an explicit cast to
 * 64 bits in order to prevent undefined behavior.
 */
#define U8(x)	((u8)(x))
#define U16(x)  ((u16)(x))
#define U32(x)	((u32)(x))
#define U64(x)	((u64)(x))

/* Linux uses U8_MAX instead of UCHAR_MAX. We define it here for non-Linux
 * OSes
 */
#if !defined(__KERNEL__) && !defined(U8_MAX)
#define U8_MAX ((u8)255)
#define U32_MAX		((u32)~0U)
#endif

#endif /* NVGPU_TYPES_H */
