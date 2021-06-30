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

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/stddef.h>
#include <stdbool.h>

#include "tracebuf.h"
#include "utility.h"

#define TRACE_VERSION ((1ULL << 16ULL) | sizeof(struct tracebuf))
#define MIN_WORD_SIZE ((uint32_t)sizeof(uint64_t))
#define MIN_MSG_LEVEL (16)
#define NEXT_DATA_OFF (0xFFFFCAFEULL)

/*
 * The below macros are accessors for the 64-bit `position` value,
 * which is broken down into the following sub-fields:
 *   ===============================================================
 *   || wrapcnt (16 bits) || reserve (24 bits) || valid (24 bits) ||
 *   ===============================================================
 */

#define GET_WRAPCNT(x) (((x) <<  0ULL) >> 48ULL)
#define GET_RESERVE(x) (((x) << 16ULL) >> 40ULL)
#define GET_VALID(x)   (((x) << 40ULL) >> 40ULL)

#define SET_WRAPCNT(x) (((x) << 48ULL) >>  0ULL)
#define SET_RESERVE(x) (((x) << 40ULL) >> 16ULL)
#define SET_VALID(x)   (((x) << 40ULL) >> 40ULL)

static inline int fill_context(struct tracectx *ctx, void *buffer,
	uint32_t length)
{
	uint32_t available = length - (uint32_t)sizeof(struct tracebuf);
	uint32_t maxsize;

	if (((uintptr_t)buffer % MIN_WORD_SIZE) != 0)
		return -EINVAL;

	if ((length % MIN_WORD_SIZE) != 0)
		return -EINVAL;

	if (length < sizeof(struct tracebuf))
		return -EINVAL;

	if ((available % MIN_WORD_SIZE) != 0)
		return -EINVAL;

	/* available space must fit within 24 bits */
	if (available >= (1 << 24))
		return -EINVAL;

	if ((available / MIN_MSG_LEVEL) < sizeof(struct tracehdr))
		return -EINVAL;

	maxsize = (available / MIN_MSG_LEVEL) -
		(uint32_t)sizeof(struct tracehdr);
	maxsize = (maxsize / MIN_WORD_SIZE) * MIN_WORD_SIZE;

	ctx->shared  = (volatile struct tracebuf *)buffer;
	ctx->begin   = (uintptr_t)ctx->shared + sizeof(struct tracebuf);
	ctx->end     = (uintptr_t)ctx->shared + length;
	ctx->length  = available;
	ctx->maxsize = maxsize;

	return 0;
}

void pull_init(struct tracectx *ctx, struct pullstate *state)
{
	uint64_t position = read64(&ctx->shared->position);

	read_barrier();

	state->wrapcnt = GET_WRAPCNT(position);
	state->current = GET_VALID(position);
	state->wrapped = false;

	if (state->current > GET_RESERVE(position))
		state->copos = true;
	else
		state->copos = false;
}

int tracebuf_init(struct tracectx *ctx, void *buffer, uint32_t length)
{
	int ret;

	ret = fill_context(ctx, buffer, length);
	if (ret != 0)
		return ret;

	memset((void *)ctx->begin, 0, ctx->length);

	ctx->shared->position = 0;
	ctx->shared->seqid    = 1;
	ctx->shared->length   = ctx->length;
	ctx->shared->maxsize  = ctx->maxsize;

	write_barrier();

	ctx->shared->version = TRACE_VERSION;

	write_barrier();

	return 0;
}

int tracebuf_bind(struct tracectx *ctx, void *buffer, uint32_t length)
{
	int ret;

	ret = fill_context(ctx, buffer, length);
	if (ret != 0)
		return ret;

	if (ctx->shared->version != TRACE_VERSION)
		return -EINVAL;

	read_barrier();

	if (ctx->shared->length != ctx->length)
		return -EINVAL;

	if  (ctx->shared->maxsize != ctx->maxsize)
		return -EINVAL;

	return 0;
}

void tracebuf_push(struct tracectx *ctx, struct tracehdr *hdr,
	void *payload, uint32_t paylen)
{
	uint32_t padding;
	uint64_t position;
	uint64_t offset;
	uintptr_t addr;
	bool wrapped = false;

	hdr->seqid = increment64(&ctx->shared->seqid);
	hdr->length = (uint32_t)paylen;

	if (paylen > ctx->maxsize)
		paylen = ctx->maxsize;

	padding = (MIN_WORD_SIZE - (paylen % MIN_WORD_SIZE)) % MIN_WORD_SIZE;
	offset = (uint32_t)sizeof(uint64_t) +
		(uint32_t)sizeof(struct tracehdr) + paylen + padding;

	/*
	 * Preare to update the reserve index. This will indicate to any
	 * reader that the space after it may become corrupt asynchronously.
	 */

	position = read64(&ctx->shared->position);

	if ((GET_RESERVE(position) + offset) > ctx->length) {
		wrapped = true;

		position = SET_WRAPCNT(GET_WRAPCNT(position) + 1ULL)
			| SET_RESERVE(offset)
			| SET_VALID(GET_VALID(position));

		write64(&ctx->shared->position, position);
	} else {
		position = SET_WRAPCNT(GET_WRAPCNT(position))
			| SET_RESERVE(GET_RESERVE(position) + offset)
			| SET_VALID(GET_VALID(position));

		write64(&ctx->shared->position, position);
	}

#ifdef ENABLE_DEBUG_HOOK
	debug_callback(ctx, "reserved");
#endif

	/*
	 * The reserve index has been updated. We will now proceed
	 * to fill in message data. Note two tricky scenarios:
	 *   1. Padding bytes keep messages aligned to MIN_WORD_SIZE.
	 *   2. Skip messages are used to indicate unused space.
	 */

	write_barrier();

	position = read64(&ctx->shared->position);
	addr = ctx->begin + GET_RESERVE(position);

	if ((addr % MIN_WORD_SIZE) != 0)
		return;

	if ((addr > ctx->end) || (addr - offset < ctx->begin))
		return;

	addr -= sizeof(uint64_t);
	*(uint64_t *)addr = offset;

	addr -= sizeof(struct tracehdr);
	memcpy((void *)addr, hdr, sizeof(struct tracehdr));

	addr -= padding;
	memset((void *)addr, 0, padding);

	addr -= paylen;
	memcpy((void *)addr, payload, paylen);

	if (wrapped == true) {
		addr = ctx->begin + GET_VALID(position);

		if (addr < ctx->end) {
			*((uint64_t *)ctx->end - 1) =
				SET_TOP32(NEXT_DATA_OFF)
				| SET_LOW32(ctx->end - addr);
		}
	}

#ifdef ENABLE_DEBUG_HOOK
	debug_callback(ctx, " written");
#endif

	/*
	 * The message data has now been filled in. We will now
	 * indicate to any reader that data is valid for reading.
	 */

	write_barrier();

	position = SET_WRAPCNT(GET_WRAPCNT(position))
		| SET_RESERVE(GET_RESERVE(position))
		| SET_VALID(GET_RESERVE(position));

	write64(&ctx->shared->position, position);

#ifdef ENABLE_DEBUG_HOOK
	debug_callback(ctx, "advanced");
#endif
}

int tracebuf_pull(struct tracectx *ctx, struct pullstate *state,
	struct tracehdr *hdr, void *payload, uint32_t *paylen)
{
	uint64_t remainder;
	uint64_t position;
	uint64_t offset;
	uintptr_t addr;

	if (state->current == 0) {
		state->current = ctx->length;
		state->wrapped = true;
	}

	/*
	 * Get the size of the current message. After reading the
	 * value, we need to check that the space wasn't corrupted
	 * by the writer asynchronously.
	 */

	addr = ctx->begin + state->current - sizeof(uint64_t);

	if ((addr % MIN_WORD_SIZE) != 0)
		return -EIO;

	if ((addr < ctx->begin) || (addr + sizeof(uint64_t) > ctx->end))
		return -EIO;

	offset = *(uint64_t *)addr;

	read_barrier();

	position = read64(&ctx->shared->position);

	if ((state->copos == true) || (state->wrapped == true)) {
		if (state->current - sizeof(uint64_t) < GET_RESERVE(position))
			return -ENOBUFS;
	}

	if (state->wrapcnt != GET_WRAPCNT(position))
		return -EINTR;

	/*
	 * Process the message length. Note that some lengths are used
	 * to communicate special conditions.
	 */

	if (offset == 0)
		return -ENOBUFS;

	if (GET_TOP32(offset) == NEXT_DATA_OFF) {
		state->current -= GET_LOW32(offset);
		return -EAGAIN;
	}

	if ((offset % MIN_WORD_SIZE) != 0)
		return -EIO;

	if (offset < sizeof(uint64_t) + sizeof(struct tracehdr))
		return -EIO;

	if (offset > sizeof(uint64_t) + sizeof(struct tracehdr)
			+ ctx->maxsize) {
		return -EIO;
	}

	/*
	 * Get a copy of the data. After reading the contents,
	 * we need to check that the space wasn't corrupted
	 * by the writer asynchronously.
	 */

	if (addr - sizeof(struct tracehdr) < ctx->begin)
		return -EIO;

	addr -= sizeof(struct tracehdr);
	memcpy(hdr, (void *)addr, sizeof(struct tracehdr));

	remainder = offset - sizeof(uint64_t) - sizeof(struct tracehdr);

	if (addr - remainder < ctx->begin)
		return -EIO;

	if (*paylen > hdr->length)
		*paylen = hdr->length;

	if (*paylen > remainder)
		*paylen = (uint32_t)remainder;

	if (*paylen > 0) {
		addr -= remainder;
		memcpy(payload, (void *)addr, *paylen);
	}

	read_barrier();

	position = ctx->shared->position;

	if ((state->copos == true) || (state->wrapped == true)) {
		if (state->current - offset < GET_RESERVE(position))
			return -ENOBUFS;
	}

	if (state->wrapcnt != GET_WRAPCNT(position))
		return -EINTR;

	/*
	 * This message is valid. Return it and move to the next
	 * message position.
	 */

	state->current -= offset;

	return 0;
}
