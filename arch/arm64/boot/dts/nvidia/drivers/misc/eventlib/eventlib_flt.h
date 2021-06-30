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

#ifndef EVENTLIB_FLT_H
#define EVENTLIB_FLT_H

#include "eventlib.h"

/* Readers' connects/updates/disconnects and writer's checks for updates, are
 * all fully asynchronous.
 *
 * Slot allocation by readers is based on bits of 'busy' word in r2w memory.
 * Readers acquire bits in this word using atomic test-and-set.
 * Writer is not involved in this.
 * - reader:
 *   - increment seqlock,
 *   - write new slot content,
 *   - increment seqlock;
 * - writer:
 *   - read seqlock
 *   - if seqlock is odd, retry later
 *   - read content into temporary buffer
 *   - read seqlock
 *   - if seqlock changed, retry later
 *   - accept temporary byffer as new content
 *
 * Notification of writer on updated slots, without need for unbounded loop
 * on writer side, is based on two words, 'notify' and 'ack', in r2w memory.
 * - reader:
 *   - update slot,
 *   - fetch 'ack' word,
 *   - atomically replace bit corresponding to this reader in 'notify' word
 *     with inverted corresponding bit of just fetched 'ack' word
 * - writer:
 *   - fetch 'notify' word,
 *   - calculate 'dirty' as XOR of fetched 'notify' and current 'ack',
 *   - bits set in 'dirty' correspond to updated slots
 * Correctness of this (i.e. if reader changes slot, writer always notices that)
 * follows from: (a) every reader only changes bit corresponding to this reader,
 * nobody else touches that bit, and (b) moment when writer writes to 'ack'
 * acts as a barrier:
 * - if a reader has already updated a slot, this update will be fetched by
 *   writer immediately after this write to 'ack',
 * - any newer update (including one currently happenning) will be followed by
 *   setting bit in 'notify' to value different from bit in just written 'ack',
 *   and thus will be noticed at the next check.
 *
 * On disconnect, reader sets it's mask to all zeroes. Writer fetches this
 * update following normal procedure, and thus cleans after disconnected
 * reader. As an optimization, writer notices all-zero mask copy, and marks
 * it as invalid, so it won't be included into future combined mask
 * calculations.
 */

#define MAX_MASK_SIZE 20
#define NUM_SLOTS 4

/* w2r shared memory subblock */
struct eventlib_flt_w2r {
	uint32_t compat;
	uint16_t num_bits[EVENTLIB_FILTER_DOMAIN_MAX];
} __attribute__((__packed__));

/* slot representation in r2w shared memory block */
struct eventlib_flt_slot {
	uint32_t seqlock;
	uint8_t mask[0];
	/* bit array for filters, padded at end to 32bit alignmemt */
} __attribute__((__packed__));

/* r2w shared memory block */
struct eventlib_flt_r2w {
	uint32_t notify;
	uint32_t ack;
	uint32_t busy;
	struct eventlib_flt_slot slots[0];
	/* slots here:
	 * - count of all slots is defined by NUM_SLOTS
	 * - size of each slot is total of 'num_bits' value from w2r subblock
	 *   rounded up to next multiple of four (see EVENTLIB_FLT_MASK_SIZE)
	 */
} __attribute__((__packed__));

/* helper structure used to represent domain's geometry
 * within all-domains mask
 */
struct eventlib_flt_domain_geo {
	/* total bit flags in domain */
	uint16_t bits;
	/* byte offset of domain in all-domains mask */
	uint16_t offset;
};

/* filtering context object */
struct eventlib_flt_ctx {
	/* Was filtering inited? */
	bool inited;

	/* Poiters to shared memory blocks */
	shmptr struct eventlib_flt_w2r *w2r;
	shmptr struct eventlib_flt_r2w *r2w;

	/* Parameters calculated at init time */
	struct eventlib_flt_domain_geo geo[EVENTLIB_FILTER_DOMAIN_MAX];
	uint32_t slot_size;

	/* The rest of structure is very different between writer and reader,
	 * thus using union here
	 */
	union {
		/* writer's version */
		struct {
			/* local copy of 'ack' word */
			uint32_t ack;
			/* current combined mask */
			uint8_t combined_mask[MAX_MASK_SIZE];
			/* local copies of masks in slots (row-major) */
			uint8_t mask_copy[32][MAX_MASK_SIZE];
			/* validity mask of local copies */
			uint32_t mask_copy_valid;
			/* extra space for temporary mask copy */
			uint8_t spare_mask[MAX_MASK_SIZE];
		} w;

		/* reader's version */
		struct {
			/* local copy of current mask */
			uint8_t mask[MAX_MASK_SIZE];
			/* index of allocated slot */
			uint8_t slot_index;
		} r;
	};
};

/* Below synchronization wrappers are based on GCC atomic builtins.
 * These functions are only used in the reader path.
 */

static inline bool sync_test_and_set_bit(unsigned int n, uint32_t *p)
{
	return !!(__sync_fetch_and_or(p, (1u << n)) & (1u << n));
}

static inline void sync_set_bit(unsigned int n, uint32_t *p)
{
	__sync_fetch_and_or(p, (1u << n));
}

static inline void sync_clear_bit(unsigned int n, uint32_t *p)
{
	__sync_fetch_and_and(p, ~(1u << n));
}

/* Below functions are implemented by the filter subsystem interface.
 * Linkage is optional to support writer environments w/out filtering.
 */

#pragma weak flt_init
extern int flt_init(struct eventlib_ctx *ctx);

#pragma weak flt_fini
extern void flt_fini(struct eventlib_ctx *ctx);

#endif
