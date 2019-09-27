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

#include "eventlib_init.h"

static int priv_init(struct eventlib_ctx *ctx)
{
	uint32_t priv_size = ALIGN_64_UP(sizeof(struct eventlib_init));

	if (sizeof(ctx->local_mem) < priv_size)
		return -ENOMEM;

	ctx->priv = (struct eventlib_init *)ctx->local_mem;
	memset(ctx->priv, 0, priv_size);
	return 0;
}

static int shm_writer_setup_start(shmptr struct eventlib_shared *shm,
	uint32_t size, uint32_t *offset)
{
	if (size < sizeof(struct eventlib_shared))
		return -ENOSPC;

	memset(shm, 0, sizeof(struct eventlib_shared));
	*offset = ALIGN_64_UP(sizeof(struct eventlib_shared));
	return 0;
}

static void shm_writer_setup_done(struct eventlib_shared *copy,
	shmptr struct eventlib_shared *shm, uint32_t magic)
{
	/* Publish subsystem config to shared memory. Note that all local
	 * accesses to subsystem config are done through local values to
	 * protect against unexpected asynchronous changes.
	 */
	memcpy(shm->subsys, copy->subsys, sizeof(shm->subsys));

	write_barrier();
	shm->magic = magic;
	write_barrier();
}

static int shm_reader_check(struct eventlib_shared *copy,
	shmptr struct eventlib_shared *shm, uint32_t size, uint32_t magic)
{
	int i;

	if (size < sizeof(struct eventlib_shared))
		return -ENOSPC;

	if (shm->magic != magic)
		return -EIO;

	read_barrier();

	/* Query subsystem config from shared memory and verify it. Further
	 * accesses to subsystem config must be done through these values in
	 * order to protect against unexpected asynchronous changes.
	 */
	memcpy(&copy->num_buffers, &shm->num_buffers,
	       sizeof(copy->num_buffers));
	memcpy(copy->subsys, shm->subsys, sizeof(copy->subsys));

	for (i = 0; i < EVENTLIB_SUBSYS_MAX; i++) {
		if (!copy->subsys[i].offset || !copy->subsys[i].size)
			continue;
		if (copy->subsys[i].offset > size)
			return -ENOSPC;
		if (copy->subsys[i].offset + copy->subsys[i].size > size)
			return -ENOSPC;
	}

	return 0;
}

static int shm_region_start(struct eventlib_ctx *ctx)
{
	int ret;
	struct eventlib_shared *shm;

	if (ctx->direction == EVENTLIB_DIRECTION_WRITER) {
		ret = shm_writer_setup_start(ctx->w2r_shm,
			ctx->w2r_shm_size,
			&ctx->priv->w2r_offset);
		if (!ret && ctx->r2w_shm)
			ret = shm_writer_setup_start(ctx->r2w_shm,
				ctx->r2w_shm_size,
				&ctx->priv->r2w_offset);

		/* Update shared memory do that value is visible to reader */
		shm = (struct eventlib_shared *)ctx->w2r_shm;
		shm->num_buffers = ctx->num_buffers;
	} else {
		ret = shm_reader_check(&ctx->priv->w2r_copy, ctx->w2r_shm,
			ctx->w2r_shm_size, EVENTLIB_MAGIC_W2R);
		if (!ret && ctx->r2w_shm)
			ret = shm_reader_check(&ctx->priv->r2w_copy,
				ctx->r2w_shm, ctx->r2w_shm_size,
				EVENTLIB_MAGIC_R2W);
	}

	return ret;
}

static void shm_region_done(struct eventlib_ctx *ctx)
{
	if (ctx->direction == EVENTLIB_DIRECTION_WRITER) {
		shm_writer_setup_done(&ctx->priv->w2r_copy,
			ctx->w2r_shm, EVENTLIB_MAGIC_W2R);
		if (ctx->r2w_shm)
			shm_writer_setup_done(&ctx->priv->r2w_copy,
				ctx->r2w_shm, EVENTLIB_MAGIC_R2W);
	}
}

static bool mem_ok(void *p, uint32_t size)
{
	if (!size)
		return true;
	else if (!p)
		return false;
	else if (!ALIGN_64_CHECK((uintptr_t)p))
		return false;
	else
		return true;
}

int _eventlib_init(struct eventlib_ctx *ctx, uint32_t ctx_version,
	uint32_t ctx_size)
{
	int ret;

	/* TODO: need a better size check */
	if (ctx_version != EVENTLIB_CTX_VERSION ||
		ctx_size == 0)
		return -EFAULT;

	if (ctx->direction != EVENTLIB_DIRECTION_WRITER &&
		ctx->direction != EVENTLIB_DIRECTION_READER)
		return -EINVAL;

	if (!ctx->w2r_shm ||
		!mem_ok(ctx->w2r_shm, ctx->w2r_shm_size) ||
		!mem_ok(ctx->r2w_shm, ctx->r2w_shm_size))
		return -EINVAL;

	if (ctx->priv != NULL)
		return -EINVAL;

	ctx->w2r_shm_size = ALIGN_64_DOWN(ctx->w2r_shm_size);
	ctx->r2w_shm_size = ALIGN_64_DOWN(ctx->r2w_shm_size);

	if (ctx->direction == EVENTLIB_DIRECTION_WRITER &&
	    ctx->num_buffers == 0)
		ctx->num_buffers = 1;

	ret = priv_init(ctx);
	if (ret)
		return ret;

	ret = shm_region_start(ctx);
	if (ret) {
		ctx->priv = NULL;
		return ret;
	}

	if (flt_init) {
		ret = flt_init(ctx);
		if (ret) {
			ctx->priv = NULL;
			return ret;
		}
	}

	ret = tbuf_init(ctx);
	if (ret) {
		eventlib_close(ctx);
		return ret;
	}

	shm_region_done(ctx);
	return 0;
}

void eventlib_close(struct eventlib_ctx *ctx)
{
	if (flt_fini)
		flt_fini(ctx);
	ctx->priv = NULL;
}
