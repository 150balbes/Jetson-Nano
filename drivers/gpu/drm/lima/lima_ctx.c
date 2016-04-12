// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright 2018 Qiang Yu <yuq825@gmail.com> */

#include <linux/slab.h>

#include "lima_device.h"
#include "lima_ctx.h"

int lima_ctx_create(struct lima_device *dev, struct lima_ctx_mgr *mgr, u32 *id)
{
	struct lima_ctx *ctx;
	int i, err;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;
	kref_init(&ctx->refcnt);

	for (i = 0; i < lima_pipe_num; i++) {
		err = lima_sched_context_init(dev->pipe + i, ctx->context + i, &ctx->guilty);
		if (err)
			goto err_out0;
	}

	idr_preload(GFP_KERNEL);
	spin_lock(&mgr->lock);
	err = idr_alloc(&mgr->handles, ctx, 1, 0, GFP_ATOMIC);
	spin_unlock(&mgr->lock);
	idr_preload_end();
	if (err < 0)
		goto err_out0;

	*id = err;
	return 0;

err_out0:
	for (i--; i >= 0; i--)
		lima_sched_context_fini(dev->pipe + i, ctx->context + i);
	kfree(ctx);
	return err;
}

static void lima_ctx_do_release(struct kref *ref)
{
	struct lima_ctx *ctx = container_of(ref, struct lima_ctx, refcnt);
	int i;

	for (i = 0; i < lima_pipe_num; i++)
		lima_sched_context_fini(ctx->dev->pipe + i, ctx->context + i);
	kfree(ctx);
}

int lima_ctx_free(struct lima_ctx_mgr *mgr, u32 id)
{
	struct lima_ctx *ctx;

	spin_lock(&mgr->lock);
	ctx = idr_remove(&mgr->handles, id);
	spin_unlock(&mgr->lock);

	if (ctx) {
		kref_put(&ctx->refcnt, lima_ctx_do_release);
		return 0;
	}
	return -EINVAL;
}

struct lima_ctx *lima_ctx_get(struct lima_ctx_mgr *mgr, u32 id)
{
	struct lima_ctx *ctx;

	spin_lock(&mgr->lock);
	ctx = idr_find(&mgr->handles, id);
	if (ctx)
		kref_get(&ctx->refcnt);
	spin_unlock(&mgr->lock);
	return ctx;
}

void lima_ctx_put(struct lima_ctx *ctx)
{
	kref_put(&ctx->refcnt, lima_ctx_do_release);
}

void lima_ctx_mgr_init(struct lima_ctx_mgr *mgr)
{
	spin_lock_init(&mgr->lock);
	idr_init(&mgr->handles);
}

void lima_ctx_mgr_fini(struct lima_ctx_mgr *mgr)
{
	struct lima_ctx *ctx;
	struct idr *idp;
	uint32_t id;

	idp = &mgr->handles;

	idr_for_each_entry(idp, ctx, id) {
	        kref_put(&ctx->refcnt, lima_ctx_do_release);
	}

	idr_destroy(&mgr->handles);
}
