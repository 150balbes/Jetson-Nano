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

#ifndef EVENTLIB_INIT_H
#define EVENTLIB_INIT_H

/* All pointers targeted to shared memory are annotated with 'shmptr' */
#define shmptr /* empty, pure annotation */

#include "eventlib.h"
#include "utility.h"
#include "eventlib_tbuf.h"
#include "eventlib_flt.h"

#define ALIGN_64_DOWN(x)  ((x) & ~((typeof(x))(sizeof(uint64_t) - 1)))
#define ALIGN_64_UP(x)    ALIGN_64_DOWN((x) + ((uint32_t)sizeof(uint64_t) - 1))
#define ALIGN_64_CHECK(x) ((x) == ALIGN_64_DOWN(x))

/* Eventlib uses two shared memory blocks, one is read only for readers,
 * and other is read-write for readers.
 *
 * These blocks can be physically accessed by different eventlib builds
 * simultaneuosly. Thus special mesures have to be taken to ensure
 * compatibility.
 *
 * - Blocks are separated into subblocks corresponding to subsystems. At this
 *   time, subsystems are "tracebuf" and "filtering". One more could be -
 *   alive management. Room for subsystems costs 8 bytes per subsystem per
 *   shmem block - thus should not be an issue to reserve space for 8 of them.
 * - Each subblock holds some sort of version/compatibility information, that
 *   can be used in subsystem-specific way to find out if reader's
 *   implementation of this subsystem can understand writer's implementation
 *   of this subsystem. Details are subsystem-specific.
 * - Thus reader can understand some of writer's subsystems, but maybe not
 *   all. Understanding 'tracebuf' subsystem is required.Not understanding
 *   other subsystem is possible but then reader will have only limited
 *   information.
 */

#define EVENTLIB_SUBSYS_TRACEBUF  0
#define EVENTLIB_SUBSYS_FILTERING 1
#define EVENTLIB_SUBSYS_MAX       2

#define EVENTLIB_TBUFS_MAX        6

#define EVENTLIB_MAGIC_W2R 0x52574c45 /* 'ELWR' in little endian */
#define EVENTLIB_MAGIC_R2W 0x57524c45 /* 'ELRW' in little endian */

struct eventlib_shared {
	/* Different magic is used for two shared memory blocks */
	uint32_t magic;

	/* Number of trace buffers
	 * Below value is updated from writer context
	 */
	uint32_t num_buffers;

	/* Locatons of subsystem's information. If subsystem is not presented
	 * in particular block, corresponding offset/size are kept zeroed.
	 */
	struct {
		uint32_t offset;
		uint32_t size;
	} subsys[EVENTLIB_SUBSYS_MAX];
} __attribute__((__packed__));

struct eventlib_init {
	/* writer, init time: offsets of currently unallocated space */
	uint32_t w2r_offset, r2w_offset;

	/* Local copy of subsystem information is used to protect against
	 * unexpected asynchronous changes (not expected under normal use).
	 */
	struct eventlib_shared w2r_copy;
	struct eventlib_shared r2w_copy;

	struct eventlib_tbuf_ctx tbuf[EVENTLIB_TBUFS_MAX];
	struct eventlib_flt_ctx flt;
};

static inline void *subsys_shm_area(shmptr struct eventlib_shared *sh,
	struct eventlib_shared *copy, int subsys)
{
	if (sh == NULL)
		return NULL;

	return ((void *)((uintptr_t)sh + copy->subsys[subsys].offset));
}

#define subsys_w2r_area(ctx, subsys) \
	subsys_shm_area(ctx->w2r_shm, &ctx->priv->w2r_copy, \
		EVENTLIB_SUBSYS_ ## subsys)
#define subsys_r2w_area(ctx, subsys) \
	subsys_shm_area(ctx->r2w_shm, &ctx->priv->r2w_copy, \
		EVENTLIB_SUBSYS_ ## subsys)

static inline uint32_t subsys_shm_size(struct eventlib_shared *copy, int subsys)
{
	return copy->subsys[subsys].size;
}

#define subsys_w2r_size(ctx, subsys) \
	subsys_shm_size(&ctx->priv->w2r_copy, EVENTLIB_SUBSYS_ ## subsys)
#define subsys_r2w_size(ctx, subsys) \
	subsys_shm_size(&ctx->priv->r2w_copy, EVENTLIB_SUBSYS_ ## subsys)

static inline int subsys_shm_carve(struct eventlib_shared *copy,
	uint32_t *shm_size, uint32_t *shm_offset, int subsys, uint32_t size)
{
	if (size > *shm_size - *shm_offset)
		return -ENOSPC;

	copy->subsys[subsys].offset = *shm_offset;
	copy->subsys[subsys].size = size;

	*shm_offset += ALIGN_64_UP(size);

	return 0;
}

#define subsys_w2r_carve(ctx, subsys, size) \
	subsys_shm_carve(&ctx->priv->w2r_copy, &ctx->w2r_shm_size, \
		&ctx->priv->w2r_offset, EVENTLIB_SUBSYS_ ## subsys, size)
#define subsys_r2w_carve(ctx, subsys, size) \
	subsys_shm_carve(&ctx->priv->r2w_copy, &ctx->r2w_shm_size, \
		&ctx->priv->r2w_offset, EVENTLIB_SUBSYS_ ## subsys, size)

#endif
