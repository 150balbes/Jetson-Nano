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

#ifndef EVENTLIB_H
#define EVENTLIB_H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/stddef.h>

/* Possible init flags */
#define EVENTLIB_FLAG_INIT_FILTERING (1 << 0)

/* These are used to ensure binary compatibility between library and caller
 * If eventlib_ctx is ever changed in incompatible way, EVENTLIB_CTX_VERSION
 * must be increased.
 */
#define EVENTLIB_CTX_VERSION 1
#define EVENTLIB_CTX_SIZE    (sizeof(struct eventlib_ctx))

/* Mask size is aligned to 4-byte boundary */
#define EVENTLIB_FLT_MASK_SIZE(bit_count) ((((bit_count) + 31u) / 32u) * 4u)

enum eventlib_direction {
	EVENTLIB_DIRECTION_READER = 1,
	EVENTLIB_DIRECTION_WRITER = 2
};

enum eventlib_filter_domain {
	EVENTLIB_FILTER_DOMAIN_EVENT_TYPE,
	EVENTLIB_FILTER_DOMAIN_CUSTOM,
	EVENTLIB_FILTER_DOMAIN_MAX
};

typedef uint32_t event_type_t;
typedef uint64_t event_timestamp_t;
typedef uint8_t *eventlib_bitmask_t;

/* ========================================
 * Initialization and finalization
 * ========================================
 * Caller must allocate context structure, completely zero it, then fill
 * mandatory fields, and call eventlib_init().
 */

struct eventlib_init;

struct eventlib_ctx {
	/* Direction. Mandatory. */
	enum eventlib_direction direction;

	/* Init flags. Possible flags defined below. */
	uint32_t flags;

	/* Private storage space used for internal use; must not be touched.
	 * Below value corresponds with size of struct eventlib_init.
	 */
	char local_mem[0x400] __aligned(8);

	/* W2R shared memory. Mandatory. */
	void *w2r_shm;
	uint32_t w2r_shm_size;

	/* R2W shared memory.
	 * Mandatory if using subsystem that needs it, otherwise unneeded.
	 */
	void *r2w_shm;
	uint32_t r2w_shm_size;

	/* Private pointer must be NULL on init. */
	struct eventlib_init *priv;

	/* Filtering parameters */
	uint16_t flt_num_bits[EVENTLIB_FILTER_DOMAIN_MAX];

	/* Number of trace buffers
	 * Below value will be adjusted to one if set as zero.
	 * Reader context value is ignored
	 */
	uint32_t num_buffers;
};

/* Initialize communication
 *
 * Arguments:
 *   ctx - pointer to context with mandatory fields initialized by caller
 *
 * Possible return values:
 *   0 - ok
 *   -EINVAL - invalid data found
 *   -EFAULT - binary incompatibility between library and caller detected
 *   -ENOMEM - not enough space in local memory
 *   -ENOSPC (writer) - not enough space in shared memory
 *   -ENOSPC (reader) - size of shared memory is too small to cover all
 *      data expected to be in shared memory
 *   -EPROTONOSUPPORT (reader) - incompatibility with writer detected
 *   -EIO (reader) - inconsistent state of shared memory detected
 *   -EBUSY (reader) - unable to reserve filtering slot
 */

int _eventlib_init(struct eventlib_ctx *ctx, uint32_t ctx_version,
	uint32_t ctx_size);

#define eventlib_init(ctx) \
	_eventlib_init(ctx, EVENTLIB_CTX_VERSION, EVENTLIB_CTX_SIZE)

/* De-initialize communication.
 * This operation never fails.
 *
 * Arguments:
 *   ctx - library context object
 *
 * Context object must no longer be used after call to this routine.
 */

void eventlib_close(struct eventlib_ctx *ctx);

/* ========================================
 * Read and write
 * ========================================

 * Add an event to the trace buffer. To be called on writer side.
 * Event is added unconditionally, any filtering should be applied before
 * calling this.
 *
 * Arguments:
 *   ctx - library context
 *   idx - trace buffer id [indexed from 0]
 *   type - event type, not anyhow interpreted, just passed to the reader
 *   ts - event timestamp, not anyhow interpreted, just passed to the reader
 *   data - event data
 *   size - size of the data.
 *
 * This operation never fails.
 * Data may be truncated if too large. Old events may get overwritten.
 */

void eventlib_write(struct eventlib_ctx *ctx, uint32_t idx,
	event_type_t type, event_timestamp_t ts, void *data, uint32_t size);

/* Try to extract many events from trace buffer. To be called at reader side.
 * It is not guaranteed that any particular event will be delivered.
 * Delivery order is always preserved with newest events first (LIFO).
 * The other thing guaranteed is - same event won't be delivered to same
 * reader more than once.
 *
 * Arguments:
 *   ctx - library context
 *   buffer - where to store event data, represented as list of `record`,
 *     with event data following each `record` with size `record.size`
 *   size - on entry, available buffer size, on successful return, final size
 *     buffer size should match `w2r_shm_size` passed to eventlib_init()
 *   num_lost_events - where to store number of new lost events detected
 *     within this call (NULL = do not store)
 *
 * Possible return values:
 *   0 - successfully extracted events, see size output parameter
 *   -EPROTO - protocol usage violation (called on reader side)
 *   -EIO - encountered an internal data inconsistency, this should
 *     not happen, error is not expected to be recoverable
 *   -EINTR - attempt was repeatedly interrupted by a fast writer, this is
 *     not expected (perhaps peer is misbehaving), caller can retry later
 */

struct record {
	uint32_t size;
	uint32_t type;
	uint64_t ts;
	uint8_t  data[0];
} __packed;

int eventlib_read(struct eventlib_ctx *ctx, void *buffer, uint32_t *size,
	uint64_t *lost);

/* ========================================
 * Filtering feature
 * ========================================

 * Check how many readers have slots currently reserved.
 *
 * Arguments:
 *   ctx - library context
 *
 * Possible return values:
 *   >=0 - success, returned value is the result
 *   -EPROTO - protocol violation (not writer side, not inited)
 *
 * Note that due to readers are completely asynchronous with writer, returned
 * value can become obsolete an any moment, it can even be already obsolete
 * at return time.
 */

int eventlib_get_num_attached_readers(struct eventlib_ctx *ctx);

/* Get current mask for the given domain.
 * For reader, returns mask of this reader
 * For writer, returns ORed masks from all connected readers.
 *
 * Arguments:
 *   ctx - library context
 *   domain - domain in question
 *   mask - where to store mask, buffer size should be at least
 *     EVENTLIB_FLT_MASK_SIZE(<bit in domain>)
 *
 * Possible return values:
 *   0 - success, mask stored
 *   -EINVAL - invalid argument
 *   -EPROTO - filtering not inited at writer side
 *
 * Note that on writer side, mask is refreshed in all calls that use it,
 * thus returned mask could become obsolete soon.
 */

int eventlib_get_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask);

/* Check if given bit is set in the current mask for the given domain.
 * For reader, uses current mask for this reader
 * For writer, uses ORed masks from all connected readers.
 *
 * Arguments:
 *   ctx - library context
 *   domain - domain in question
 *   bit - bit to check, bits count from zero
 *
 * Possible return values:
 *   0 or 1 - success, returned bit value
 *   -EINVAL - invalid argument
 *   -EPROTO - filtering not inited at writer side
 */

int eventlib_check_filter_bit(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, uint16_t bit);

/* Check if any of set bits in the given mask is set in the current mask for
 * the given domain.
 * For reader, uses current mask for this reader
 * For writer, uses ORed masks from all connected readers.
 *
 * Arguments:
 *   ctx - library context
 *   domain - domain in question
 *   mask - mask with bits to check, size should be
 *     EVENTLIB_FLT_MASK_SIZE(<bit in domain>)
 *
 * Possible return values:
 *   0 - success, no bit is set
 *   1 - success, some bits are set
 *   -EINVAL - invalid argument
 *   -EPROTO - filtering not inited at writer side
 */

int eventlib_check_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask);

/* Update single bit in reader's current mask for the given domain
 * Changed mask is immediately synced to the writer.
 *
 * Arguments:
 *   ctx - library context
 *   domain - domain in question
 *   bit - bit to write to, bits count from zero
 *   val - value to write into bit
 *
 * Possible return values:
 *   0 - success,
 *   -EINVAL - invalid argument
 *   -EPROTO - called on writer side
 */

int eventlib_set_filter_bit(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, uint16_t bit, int val);

/* Replace entire reader's mask for the given domain
 * Changed mask is immediately synced to the writer.
 *
 * Arguments:
 *   ctx - library context
 *   domain - domain in question
 *   mask - new mask, size should be EVENTLIB_FLT_MASK_SIZE(<bits in domain>)
 *
 * Possible return values:
 *   0 - success,
 *   -EINVAL - invalid argument
 *   -EPROTO - called on writer side
 */

int eventlib_set_filter_mask(struct eventlib_ctx *ctx,
	enum eventlib_filter_domain domain, eventlib_bitmask_t mask);

/* ========================================
 * Timestamp access for CPU
 * ========================================
 */

#if defined(__aarch64__)

/* Returns the current system timer counter value.
 * This operation never fails.
 */
static inline event_timestamp_t eventlib_get_timer_counter(void)
{
	uint64_t value;

	asm volatile("mrs %0, CNTPCT_EL0" : "=r"(value) :: );
	return (event_timestamp_t)value;
}

/* Returns the system timer frequency in hertz.
 * This operation never fails.
 */
static inline uint32_t eventlib_get_timer_freq(void)
{
	uint64_t value;

	asm volatile("mrs %0, CNTFRQ_EL0" : "=r"(value) :: );
	return (uint32_t)value;
}

#elif defined(__arm__)

/* Returns the current system timer counter value.
 * This operation never fails.
 */
static inline event_timestamp_t eventlib_get_timer_counter(void)
{
	uint32_t value1, value2;

	asm volatile("mrrc p15, 0, %0, %1, c14"
		: "=r"(value1), "=r"(value2) :: );
	return ((event_timestamp_t)(
		(((uint64_t)value1) | ((uint64_t)value2 << 32))));
}

/* Returns the system timer frequency in hertz.
 * This operation never fails.
 */
static inline uint32_t eventlib_get_timer_freq(void)
{
	uint32_t value;

	asm volatile("mrc p15, 0, %0, c14, c0, 0" : "=r"(value) :: );
	return value;
}

#endif

#endif
