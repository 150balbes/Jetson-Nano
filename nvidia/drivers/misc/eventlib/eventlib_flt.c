/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION. All rights reserved.
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
#include "eventlib_flt.h"

/* This eventlib version supports only one version of filtering */
#define FLT_COMPAT_INFO 0

/*****************************************************************************
 * Filter Mask Exchange Protocol
 *****************************************************************************/

/* Get n-th slot in r2w memory */
static inline shmptr struct eventlib_flt_slot *flt_r2w_slot(
	struct eventlib_flt_ctx *flt, uint8_t n)
{
	return (struct eventlib_flt_slot *)
		((uintptr_t)(&flt->r2w->slots[0]) + n * flt->slot_size);
}

/* Reader: push local mask to slot */
static void flt_reader_push(struct eventlib_flt_ctx *flt)
{
	uint8_t n = flt->r.slot_index;
	shmptr struct eventlib_flt_slot *slot = flt_r2w_slot(flt, n);
	uint32_t seqlock, ack;

	/* This is not first access to seqlock of this slot by this reader.
	 * Nobody should have changed seqlock since previous access (unless
	 * somebody violates access protocol... but in that caase, barrier
	 * won't help anyway)
	 */

	seqlock = slot->seqlock;

	slot->seqlock = ++seqlock;
	write_barrier();

	memcpy(slot->mask, flt->r.mask, MAX_MASK_SIZE);
	write_barrier();

	slot->seqlock = ++seqlock;
	write_barrier();

	read_barrier();
	ack = flt->r2w->ack;
	if (ack & (1u << n))
		sync_clear_bit(n, &flt->r2w->notify);
	else
		sync_set_bit(n, &flt->r2w->notify);
}

/* Writer: fetch any slot updates from readers */
static void flt_writer_refresh(struct eventlib_flt_ctx *flt)
{
	uint8_t n;
	shmptr struct eventlib_flt_slot *slot;
	uint32_t notify, dirty, seqlock, remaining;
	bool fetched = false;
	unsigned int i;
	uint32_t *s, *d, v;

	/* quick check for updates */
	read_barrier();
	notify = flt->r2w->notify;
	dirty = notify ^ flt->w.ack;
	if (!dirty)
		return;

	/* continue only if update exists */
	flt->r2w->ack = flt->w.ack = notify;
	write_barrier();

	/* fetch any updated masks */
	for (n = 0; dirty && n < NUM_SLOTS; n++) {
		if (!(dirty & (1u << n)))
			continue;

		dirty &= ~(1u << n);
		slot = flt_r2w_slot(flt, n);

		read_barrier();
		seqlock = slot->seqlock;

		if ((seqlock & 1u) != 0)
			continue;

		read_barrier();
		memcpy(flt->w.spare_mask, slot->mask, MAX_MASK_SIZE);

		read_barrier();
		if (seqlock != slot->seqlock)
			continue;

		memcpy(&flt->w.mask_copy[n][0], flt->w.spare_mask,
			MAX_MASK_SIZE);
		flt->w.mask_copy_valid |= (1u << n);
		fetched = true;
	}

	/* if nothing fetched, return */
	if (!fetched)
		return;

	/* re-calculate combined mask */
	memset(flt->w.combined_mask, 0, MAX_MASK_SIZE);
	remaining = flt->w.mask_copy_valid;

	for (n = 0; remaining && n < NUM_SLOTS; n++) {
		if (!(remaining & (1u << n)))
			continue;

		remaining &= ~(1u << n);

		s = (uint32_t *)&flt->w.mask_copy[n][0];
		d = (uint32_t *)flt->w.combined_mask;
		v = 0;

		for (i = 0; i < MAX_MASK_SIZE / 4; i++) {
			d[i] |= s[i];
			v |= s[i];
		}

		/* discover and discard full-zero slot copies */
		if (!v)
			flt->w.mask_copy_valid &= ~(1u << n);
	}
}

/* Reader: allocate slot */
static int flt_reader_alloc_slot(struct eventlib_flt_ctx *flt)
{
	shmptr struct eventlib_flt_slot *slot;
	uint32_t busy, all_slots_mask, seqlock;
	uint8_t n;

	all_slots_mask = (uint32_t)((1ull << NUM_SLOTS) - 1ull);

	while (1) {

		/* Barrier here ensures a new read of flt->r2w->busy from
		 * memory. Still it could be updated by other actors
		 * immediately after that read, but since it is used as a hint
		 * to choose bits to test, it's not a problem
		 */
		read_barrier();
		busy = flt->r2w->busy;

		if ((busy & all_slots_mask) == all_slots_mask) {
			/* There was no free slot at time of mask was
			 * read from memory. Threat that as no free slot
			 * error
			 */
			return -EBUSY;
		}

		/* Slot was available at time of mask read. Find it and
		 * try to synchronously allocate it. This can race against
		 * other reader doing the same and thus can fail - thus
		 * entire operation is executed in loop
		 */
		for (n = 0; n < NUM_SLOTS; n++) {
			if (busy & (1u << n))
				continue;
			if (sync_test_and_set_bit(n, &flt->r2w->busy) == 0)
				goto success;
		}
	}

success:
	flt->r.slot_index = n;
	slot = flt_r2w_slot(flt, n);

	/* Just for case, ensure clean state at init */

	/* This is the first access to seqlock of this slot by this reader.
	 * Per generic rule, have a read barrier before accessing it, to force
	 * read from memory. Although situation of no physical read here is
	 * absolutely exotic
	 */
	read_barrier();
	seqlock = slot->seqlock;

	if (seqlock & 1u) {
		slot->seqlock = ++seqlock;
		write_barrier();
	}

	return 0;
}

/* Reader: release slot */
static void flt_reader_release_slot(struct eventlib_flt_ctx *flt)
{
	/* push all-zero mask */
	memset(flt->r.mask, 0, MAX_MASK_SIZE);
	flt_reader_push(flt);

	/* clear busy bit */
	sync_clear_bit(flt->r.slot_index, &flt->r2w->busy);
}

static void flt_reader_set_all_bits(struct eventlib_flt_ctx *flt)
{
	struct eventlib_flt_domain_geo *geo;
	uint8_t *p;
	int i, j;

	for (i = 0; i < EVENTLIB_FILTER_DOMAIN_MAX; i++) {
		geo = &flt->geo[i];
		p = &flt->r.mask[geo->offset];
		for (j = 0; j < geo->bits / 8; j++)
			*p++ = 0xff;
		if (geo->bits % 8)
			*p = (uint8_t)((1u << (geo->bits % 8)) - 1u);
	}
}

/*****************************************************************************
 * Filter Mask Control Initialization
 *****************************************************************************/

/* Init constants in eventlib_flt_ctx */
static int flt_init_consts(struct eventlib_flt_ctx *flt, uint16_t *num_bits)
{
	struct eventlib_flt_domain_geo *geo;
	uint16_t bytes = 0;
	int i;

	for (i = 0; i < EVENTLIB_FILTER_DOMAIN_MAX; i++) {
		geo = &flt->geo[i];
		geo->bits = num_bits[i];
		geo->offset = bytes;
		bytes = (uint16_t)(bytes + EVENTLIB_FLT_MASK_SIZE(geo->bits));
	}

	if (bytes > MAX_MASK_SIZE)
		return -ENOMEM;

	flt->slot_size = (uint32_t)sizeof(struct eventlib_flt_slot) + bytes;
	return 0;
}

/* Get r2w memory size (after flt_init_consts() completed) */
static inline uint32_t flt_r2w_size(struct eventlib_flt_ctx *flt)
{
	return ((uint32_t)sizeof(struct eventlib_flt_r2w) +
		(NUM_SLOTS * flt->slot_size));
}

static int flt_writer_init(struct eventlib_ctx *ctx)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint32_t size;
	int ret;

	memset(flt, 0, sizeof(*flt));

	if (!(ctx->flags & EVENTLIB_FLAG_INIT_FILTERING))
		return 0;

	if (ctx->flt_num_bits[EVENTLIB_FILTER_DOMAIN_EVENT_TYPE] < 1)
		return -EINVAL;

	ret = flt_init_consts(flt, ctx->flt_num_bits);
	if (ret != 0)
		return ret;

	/* initialize w2r shared memory area */
	ret = subsys_w2r_carve(ctx, FILTERING,
		sizeof(struct eventlib_flt_w2r));
	if (ret != 0)
		return ret;

	flt->w2r = subsys_w2r_area(ctx, FILTERING);
	size = subsys_w2r_size(ctx, FILTERING);
	if (size != sizeof(struct eventlib_flt_w2r))
		return -ENOSPC;

	flt->w2r->compat = FLT_COMPAT_INFO;
	memcpy(flt->w2r->num_bits, ctx->flt_num_bits,
		sizeof(ctx->flt_num_bits));

	/* initialize r2w shared memory area */
	ret = subsys_r2w_carve(ctx, FILTERING, flt_r2w_size(flt));
	if (ret != 0)
		return ret;

	flt->r2w = subsys_r2w_area(ctx, FILTERING);
	size = subsys_r2w_size(ctx, FILTERING);
	if (size != flt_r2w_size(flt))
		return -ENOSPC;

	memset(flt->r2w, 0, size);

	flt->inited = true;
	return 0;
}

static int flt_reader_init(struct eventlib_ctx *ctx)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint32_t w2r_size = subsys_w2r_size(ctx, FILTERING);
	uint32_t r2w_size = subsys_r2w_size(ctx, FILTERING);
	int ret;

	memset(flt, 0, sizeof(*flt));

	/* check if writer disabled filtering */
	if ((w2r_size == 0) && (r2w_size == 0)) {
		memset(ctx->flt_num_bits, 0, sizeof(ctx->flt_num_bits));
		return 0;
	}

	/* process w2r shared memory area */
	if (w2r_size != sizeof(struct eventlib_flt_w2r))
		return -EPROTONOSUPPORT;

	flt->w2r = subsys_w2r_area(ctx, FILTERING);
	if (flt->w2r->compat != FLT_COMPAT_INFO)
		return -EPROTONOSUPPORT;

	memcpy(ctx->flt_num_bits, flt->w2r->num_bits,
		sizeof(flt->w2r->num_bits));

	if (ctx->flt_num_bits[EVENTLIB_FILTER_DOMAIN_EVENT_TYPE] < 1)
		return -EIO;

	ret = flt_init_consts(flt, ctx->flt_num_bits);
	if (ret != 0)
		return ret;

	/* process r2w shared memory area */
	if (r2w_size != flt_r2w_size(flt))
		return -EIO;

	flt->r2w = subsys_r2w_area(ctx, FILTERING);

	ret = flt_reader_alloc_slot(flt);
	if (ret != 0)
		return ret;

	if (!(ctx->flags & EVENTLIB_FLAG_INIT_FILTERING))
		flt_reader_set_all_bits(flt);

	flt->inited = true;
	return 0;
}

int flt_init(struct eventlib_ctx *ctx)
{
	int ret;

	switch (ctx->direction) {
	case EVENTLIB_DIRECTION_WRITER:
		ret = flt_writer_init(ctx);
		break;
	case EVENTLIB_DIRECTION_READER:
		ret = flt_reader_init(ctx);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

void flt_fini(struct eventlib_ctx *ctx)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;

	if (ctx->direction != EVENTLIB_DIRECTION_READER)
		return;

	if (flt->inited)
		flt_reader_release_slot(flt);
}

/*****************************************************************************
 * Filter Mask Access Implementation
 *****************************************************************************/

int eventlib_get_num_attached_readers(struct eventlib_ctx *ctx)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint32_t busy;
	int ret;

	if (!flt->inited)
		return -EPROTO;

	if (ctx->direction != EVENTLIB_DIRECTION_WRITER)
		return -EPROTO;

	read_barrier();
	busy = flt->r2w->busy;

	ret = 0;
	while (busy) {
		if (busy & 1u)
			ret++;
		busy >>= 1;
	}

	return ret;
}

int eventlib_get_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;

	if (!flt->inited)
		return -EPROTO;

	if (domain >= EVENTLIB_FILTER_DOMAIN_MAX)
		return -EINVAL;

	if (ctx->direction == EVENTLIB_DIRECTION_WRITER) {
		flt_writer_refresh(flt);
		memcpy(mask, flt->w.combined_mask + flt->geo[domain].offset,
			EVENTLIB_FLT_MASK_SIZE(flt->geo[domain].bits));
	} else
		memcpy(mask, flt->r.mask + flt->geo[domain].offset,
			EVENTLIB_FLT_MASK_SIZE(flt->geo[domain].bits));

	return 0;
}

int eventlib_check_filter_bit(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, uint16_t bit)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint8_t *p, m;

	if (!flt->inited)
		return -EPROTO;

	if (domain >= EVENTLIB_FILTER_DOMAIN_MAX)
		return -EINVAL;

	if (bit >= flt->geo[domain].bits)
		return -EINVAL;

	if (ctx->direction == EVENTLIB_DIRECTION_WRITER) {
		flt_writer_refresh(flt);
		p = flt->w.combined_mask;
	} else
		p = flt->r.mask;

	p += flt->geo[domain].offset + (bit / 8);
	m = (uint8_t)(1u << (bit % 8));

	return ((*p) & m) ? 1 : 0;
}

int eventlib_check_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint8_t *a, *b, m;
	uint32_t size;

	if (!flt->inited)
		return -EPROTO;

	if (domain >= EVENTLIB_FILTER_DOMAIN_MAX)
		return -EINVAL;

	if (ctx->direction == EVENTLIB_DIRECTION_WRITER) {
		flt_writer_refresh(flt);
		a = flt->w.combined_mask + flt->geo[domain].offset;
	} else
		a = flt->r.mask;

	size = flt->geo[domain].bits;
	b = mask;

	while (size >= 32) {
		if ((*((uint32_t *)a)) & (*((uint32_t *)b)))
			return 1;
		a += 4;
		b += 4;
		size -= 32;
	}

	while (size >= 8) {
		if ((*a) & (*b))
			return 1;
		a++;
		b++;
		size -= 8;
	}

	if (size > 0) {
		m = (uint8_t)((1u << size) - 1u);
		if (((*a) & (*b) & m) != 0)
			return 1;
	}

	return 0;
}

int eventlib_set_filter_bit(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, uint16_t bit, int val)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;
	uint8_t *p, m;

	if (ctx->direction != EVENTLIB_DIRECTION_READER)
		return -EPROTO;

	if (!flt->inited)
		return -EPROTO;

	if (domain >= EVENTLIB_FILTER_DOMAIN_MAX)
		return -EINVAL;

	if (bit >= flt->geo[domain].bits)
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	p = flt->r.mask + flt->geo[domain].offset + (bit / 8);
	m = (uint8_t)(1u << (bit % 8));

	if (val)
		*p |= m;
	else
		*p &= (uint8_t)(~m);

	flt_reader_push(flt);
	return 0;
}

int eventlib_set_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask)
{
	struct eventlib_flt_ctx *flt = &ctx->priv->flt;

	if (ctx->direction != EVENTLIB_DIRECTION_READER)
		return -EPROTO;

	if (!flt->inited)
		return -EPROTO;

	if (domain >= EVENTLIB_FILTER_DOMAIN_MAX)
		return -EINVAL;

	memcpy(flt->r.mask + flt->geo[domain].offset, mask,
		EVENTLIB_FLT_MASK_SIZE(flt->geo[domain].bits));

	flt_reader_push(flt);
	return 0;
}
