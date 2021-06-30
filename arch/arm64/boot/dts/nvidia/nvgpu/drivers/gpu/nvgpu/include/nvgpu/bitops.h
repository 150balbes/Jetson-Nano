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
#ifndef NVGPU_BITOPS_H
#define NVGPU_BITOPS_H

#include <nvgpu/types.h>

/*
 * Explicit sizes for bit definitions. Please use these instead of BIT().
 */
#define BIT8(i)		(U8(1)  << (i))
#define BIT16(i)	(U16(1) << (i))
#define BIT32(i)	(U32(1) << (i))
#define BIT64(i)	(U64(1) << (i))

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/bitmap.h>
#elif defined(__NVGPU_POSIX__)
#include <nvgpu/posix/bitops.h>
#else
#include <nvgpu_rmos/include/bitops.h>
#endif

#endif /* NVGPU_BITOPS_H */
