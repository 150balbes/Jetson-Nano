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

#ifndef EVENTLIB_TBUF_H
#define EVENTLIB_TBUF_H

#include "eventlib.h"
#include "tracebuf.h"

struct eventlib_tbuf_w2r {
	uint32_t compat;
	uint32_t _pad;   /* ensure 64-bit alignment for tracebuf */
	uint8_t tbuf[0]; /* rest of space managed by tracebuf library */
} __attribute__((__packed__));

struct eventlib_tbuf_ctx {
	struct tracectx tbuf_ctx;

	/* Events with up to this seqid have been either already delivered,
	 * or already considered lost
	 */
	uint64_t seqid_ack;
};

extern int tbuf_init(struct eventlib_ctx *ctx);

#endif
