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

#include <linux/string.h>

#include "eventlib.h"
#include "tracebuf.h"
#include "eventlib_init.h"
#include "eventlib_tbuf.h"

/* Reads sequences from shared memory can be attempted at most two times */
#define PULL_COUNT_MAX 2
#define INIT_COUNT_MAX 2

/* Valid sequence IDs are always in the below range (inclusive) */
#define SEQUENCE_ID_MIN  (0ULL)
#define SEQUENCE_ID_MAX (~0ULL)

/* Bit 0 of compat word shows if writer has initialized filtering.
 * Other bits in compat word must be zero in this version
 */
#define TBUF_COMPAT(ctx) ((ctx->flags & EVENTLIB_FLAG_INIT_FILTERING) ? 1 : 0)
#define TBUF_CHECK_COMPAT(compat) (((compat) & ~1u) == 0)

static int tbuf_writer_init(struct eventlib_ctx *ctx)
{
	uint8_t *area;
	shmptr struct eventlib_tbuf_w2r *w2r;
	uint32_t size;
	int ret = 0;
	uint32_t idx;

	ret = subsys_w2r_carve(ctx, TRACEBUF,
		ctx->w2r_shm_size - ctx->priv->w2r_offset);
	if (ret != 0)
		return ret;

	if (ctx->num_buffers == 0 || ctx->num_buffers > EVENTLIB_TBUFS_MAX)
		return -EIO;

	area = subsys_w2r_area(ctx, TRACEBUF);
	size = subsys_w2r_size(ctx, TRACEBUF);
	size /= ctx->num_buffers;
	size = ALIGN_64_DOWN(size);

	if (size < sizeof(struct eventlib_tbuf_w2r))
		return -ENOSPC;

	for (idx = 0; idx < ctx->num_buffers; idx++) {
		w2r = (struct eventlib_tbuf_w2r *)(area + (size * idx));
		w2r->compat = TBUF_COMPAT(ctx);

		ret = tracebuf_init(&ctx->priv->tbuf[idx].tbuf_ctx,
			&w2r->tbuf,
			size - (uint32_t)sizeof(struct eventlib_tbuf_w2r));

		if (ret != 0)
			break;
	}

	return ret;
}

static int tbuf_reader_init(struct eventlib_ctx *ctx)
{
	uint8_t *area;
	shmptr struct eventlib_tbuf_w2r *w2r;
	uint32_t size;
	uint32_t idx;
	int ret = 0;

	if (ctx->priv->w2r_copy.num_buffers == 0 ||
		ctx->priv->w2r_copy.num_buffers > EVENTLIB_TBUFS_MAX)
		return -EIO;

	area = subsys_w2r_area(ctx, TRACEBUF);
	size = subsys_w2r_size(ctx, TRACEBUF);
	size /= ctx->priv->w2r_copy.num_buffers;
	size = ALIGN_64_DOWN(size);

	if (size < sizeof(struct eventlib_tbuf_w2r))
		return -EIO;

	for (idx = 0; idx < ctx->priv->w2r_copy.num_buffers; idx++) {
		w2r = (struct eventlib_tbuf_w2r *)(area + (size * idx));

		if (!TBUF_CHECK_COMPAT(w2r->compat))
			return -EPROTONOSUPPORT;

		ret = tracebuf_bind(&ctx->priv->tbuf[idx].tbuf_ctx,
			&w2r->tbuf,
			size - (uint32_t)sizeof(struct eventlib_tbuf_w2r));

		if (ret != 0)
			break;
	}

	return ret;
}

int tbuf_init(struct eventlib_ctx *ctx)
{
	int ret;

	switch (ctx->direction) {
	case EVENTLIB_DIRECTION_WRITER:
		ret = tbuf_writer_init(ctx);
		break;
	case EVENTLIB_DIRECTION_READER:
		ret = tbuf_reader_init(ctx);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

void eventlib_write(struct eventlib_ctx *ctx, uint32_t idx,
	event_type_t type, event_timestamp_t ts, void *data, uint32_t size)
{
	struct tracehdr hdr;

	if (ctx->direction != EVENTLIB_DIRECTION_WRITER)
		return;

	if (idx >= ctx->num_buffers)
		return;

	hdr.params = ts;
	hdr.reserved = type;

	tracebuf_push(&ctx->priv->tbuf[idx].tbuf_ctx, &hdr, data, size);
}

static int tbuf_pull_single(struct eventlib_tbuf_ctx *tbuf,
	struct pullstate *state, uint64_t *seqid, struct record *rec,
	void *payload, uint32_t *paylen)
{
	struct tracehdr hdr;
	unsigned int i;
	uint32_t length;
	int ret = -EPROTO;

	for (i = 0; i < PULL_COUNT_MAX; i++) {
		length = *paylen;

		ret = tracebuf_pull(&tbuf->tbuf_ctx, state,
			&hdr, payload, &length);

		if (ret != -EAGAIN)
			break;
	}

	if (ret != 0)
		return ret;

	/* We need to use a generic copy here because the destination address
	 * may not be aligned. For example, the address may be an odd value.
	 */
	memcpy(&rec->size, &length, sizeof(uint32_t));
	memcpy(&rec->type, &hdr.reserved, sizeof(uint32_t));
	memcpy(&rec->ts, &hdr.params, sizeof(uint64_t));

	*seqid = hdr.seqid;
	*paylen = length;

	return 0;
}

static int tbuf_pull_multiple(struct eventlib_tbuf_ctx *tbuf, void *buffer,
	uint32_t *size, uint64_t *min, uint64_t *max)
{
	uint64_t seqid = 0ULL;
	struct pullstate state;
	uintptr_t current;
	uint32_t length;
	uint32_t avail;
	int ret;

	*min = SEQUENCE_ID_MAX;
	*max = SEQUENCE_ID_MIN;

	pull_init(&tbuf->tbuf_ctx, &state);
	current = (uintptr_t)buffer;
	avail = *size;

	while (avail >= sizeof(struct record)) {
		length = avail - (uint32_t)sizeof(struct record);

		ret = tbuf_pull_single(tbuf, &state, &seqid,
			(struct record *)current,
			(void *)(current + sizeof(struct record)),
			&length);

		/* Check if all events have been processed. This happens
		 * when we've consumed all available event data and we did
		 * not encounter any duplicates.
		 */
		if (ret == -ENOBUFS)
			break;

		/* Check if there was any other type of error. We may have
		 * been interrupted, or maybe we detected data corruption.
		 * Just report the error back to the caller.
		 */
		if (ret != 0)
			return ret;

		/* Check if this is a duplicate event. If so, there's no
		 * need to advance any further as all subsequent events
		 * will have occurred earlier than this event.
		 */
		if (seqid <= tbuf->seqid_ack)
			break;

		if (seqid > *max)
			*max = seqid;

		if (seqid < *min)
			*min = seqid;

		/* Check if it's safe to advance to the next event position.
		 * This should not fail under all expecteed scenarios if the
		 * underlying subsystem is behaving properly.
		 */
		if (avail < (uint32_t)sizeof(struct record) + length)
			return -EIO;

		avail -= (uint32_t)sizeof(struct record) + length;
		current += (uint32_t)sizeof(struct record) + length;
	}

	*size = *size - avail;

	return 0;
}

static int eventlib_read_from_tbuf(struct eventlib_tbuf_ctx *tbuf, void *buffer,
	uint32_t *size, uint64_t *lost)
{
	int ret = -EPROTO;
	unsigned int i;
	uint64_t min;
	uint64_t max;

	if (lost)
		*lost = 0;

	for (i = 0; i < INIT_COUNT_MAX; i++) {
		ret = tbuf_pull_multiple(tbuf, buffer, size, &min, &max);
		if (ret != -EINTR)
			break;
	}

	if (ret != 0 || *size == 0)
		return ret;

	if (lost) {
		/* Check if we have any newly detected lost events to report.
		 * These lost events are seq IDs in range (seqid_ack, min).
		 */
		if (min > tbuf->seqid_ack + 1)
			*lost = min - tbuf->seqid_ack - 1;
		else
			*lost = 0;
	}

	tbuf->seqid_ack = max;

	return 0;
}

int eventlib_read(struct eventlib_ctx *ctx, void *buffer, uint32_t *size,
	uint64_t *lost)
{
	int ret = -EPROTO;
	unsigned int idx;
	uint64_t accum_lost;
	uint8_t *copy_buffer;
	uint32_t copy_size;
	uint32_t accum_empty;

	if (lost)
		*lost = 0;

	accum_empty = *size;
	copy_buffer = (uint8_t *)(buffer);

	if (ctx->direction != EVENTLIB_DIRECTION_READER)
		return -EPROTO;

	for (idx = 0; idx < ctx->priv->w2r_copy.num_buffers; idx++) {
		accum_lost = 0;
		copy_size = accum_empty;

		ret = eventlib_read_from_tbuf(&ctx->priv->tbuf[idx],
			copy_buffer, &copy_size, &accum_lost);

		if (ret != 0)
			break;

		/* Update empty slots */
		accum_empty -= copy_size;

		/* Advanced to next free slot */
		copy_buffer = copy_buffer + copy_size;

		if (lost)
			*lost += accum_lost;
	}

	/* Number of bytes written */
	*size -= accum_empty;

	return ret;
}
