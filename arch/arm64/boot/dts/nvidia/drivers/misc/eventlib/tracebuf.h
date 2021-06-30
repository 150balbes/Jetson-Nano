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

#ifndef TRACEBUF_H
#define TRACEBUF_H

#include <linux/stddef.h>
#include <stdbool.h>

struct tracectx {
	volatile struct tracebuf *shared;
	uintptr_t                 begin;
	uintptr_t                 end;
	uint32_t                  length;
	uint32_t                  maxsize;
};

struct tracebuf {
	uint64_t version;
	uint64_t position;
	uint64_t seqid;
	uint32_t length;
	uint32_t maxsize;
} __packed;

struct tracehdr {
	uint64_t params;
	uint64_t seqid;
	uint32_t length;
	uint32_t reserved;
} __packed;

struct pullstate {
	uint64_t wrapcnt;
	uint64_t current;
	bool     wrapped;
	bool     copos;
};

/*
 * Description for pull_init()
 *   - Initialize a new pullstate structure used to mark the location
 *     from which subsequent reads will occur.
 *   - Subsequent reads will advance the read position within the
 *     pullstate structure.
 * Parameters
 *   - Param `ctx` is provided by the caller.
 *   - Param `state` is filled by the callee.
 * Return values
 *   - Operation never fails.
 */

void pull_init(struct tracectx *ctx, struct pullstate *state);

/*
 * Description for tracebuf_init()
 *   - Initialize or re-initialize memory to prepare for binding.
 *   - Memory address and length must be aligned to MIN_WORD_SIZE and
 *     of "sufficient" size, where "sufficient" is currently some
 *     implementation-defined value not to be smaller than 4KB.
 * Parameters
 *   - Param `ctx` is filled by the callee on success.
 *   - Param `buffer` and `length` are provided by the caller.
 * Return values
 *   - Returns 0 on success.
 */

int tracebuf_init(struct tracectx *ctx, void *buffer, uint32_t length);

/*
 * Description for tracebuf_bind()
 *   - Binds to the specified region of memory. The specified region
 *     of memory must have been previously initialized with the exact
 *     same parameters for memory address and length.
 *   - Memory address and length must be aligned to MIN_WORD_SIZE and
 *     of "sufficient" size, where "sufficient" is currently some
 *     implementation-defined value not to be smaller than 4KB.
 * Parameters
 *   - Param `ctx` is filled by the callee on success.
 *   - Param `buffer` and `length` are provided by the caller.
 * Return values
 *   - Returns 0 on success.
 */

int tracebuf_bind(struct tracectx *ctx, void *buffer, uint32_t length);

/*
 * Description for tracebuf_push()
 *   - Add a message to the buffer; this operation never fails.
 *   - If there is insufficient message space available, the oldest
 *     message(s) will be overwritten.
 *   - Payloads may get truncated if its size is larger than
 *     tracebuf.maxsize, or padded up to at least MIN_WORD_SIZE.
 * Parameters
 *   - Param `ctx` is provided by the caller.
 *   - Param `hdr.params` is provided by the caller.
 *   - Param `hdr.seqid` and `hdr.length` is filled by the callee.
 *   - Param `payload` and `paylen` are provided by the caller.
 * Return values
 *   - Operation never fails.
 */

void tracebuf_push(struct tracectx *ctx, struct tracehdr *hdr,
	void *payload, uint32_t paylen);

/*
 * Description for tracebuf_pull()
 *   - Attempt to get a message from the buffer; may fail for many
 *     different reasons. See possible return values below.
 *   - Read position is determined by state specified in the pullstate
 *     structure; this state is implicitly updated after each request.
 *   - Value in *paylen specifies bytes available on input, and
 *     is updated to bytes written after a successful request.
 *   - If input space is insufficient, the payload will be truncated.
 * Parameters
 *   - Param `ctx` is provided by the caller.
 *   - Param `state` is provided by the caller.
 *   - Param `hdr` is filled by the callee on success.
 *   - Param `payload` is provided by the caller.
 *   - Param `paylen` is provided by the caller and updated by the callee.
 * Return values
 *   - Returns -EAGAIN when the current read request should be retried.
 *   - Returns -EINTR when the overall read sequence needs to be restarted.
 *   - Returns -ENOBUFS when there is no more data available.
 *   - Returns 0 on success.
 */

int tracebuf_pull(struct tracectx *ctx, struct pullstate *state,
	struct tracehdr *hdr, void *payload, uint32_t *paylen);

#ifdef ENABLE_DEBUG_HOOK
void debug_callback(struct tracectx *ctx, char *prefix);
#endif

#endif
