/*
 * Tegra GK20A GPU Debugger Driver Register Ops
 *
 * Copyright (c) 2013-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a.h"
#include "gr_gk20a.h"
#include "dbg_gpu_gk20a.h"
#include "regops_gk20a.h"

#include <nvgpu/log.h>
#include <nvgpu/bsearch.h>
#include <nvgpu/bug.h>
#include <nvgpu/io.h>

static int regop_bsearch_range_cmp(const void *pkey, const void *pelem)
{
	u32 key = *(u32 *)pkey;
	struct regop_offset_range *prange = (struct regop_offset_range *)pelem;
	if (key < prange->base) {
		return -1;
	} else if (prange->base <= key && key < (prange->base +
					       (prange->count * 4U))) {
		return 0;
	}
	return 1;
}

static inline bool linear_search(u32 offset, const u32 *list, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		if (list[i] == offset) {
			return true;
		}
	}
	return false;
}

/*
 * In order to perform a context relative op the context has
 * to be created already... which would imply that the
 * context switch mechanism has already been put in place.
 * So by the time we perform such an opertation it should always
 * be possible to query for the appropriate context offsets, etc.
 *
 * But note: while the dbg_gpu bind requires the a channel fd,
 * it doesn't require an allocated gr/compute obj at that point...
 */
static bool gr_context_info_available(struct gr_gk20a *gr)
{
	int err;

	nvgpu_mutex_acquire(&gr->ctx_mutex);
	err = !gr->ctx_vars.golden_image_initialized;
	nvgpu_mutex_release(&gr->ctx_mutex);
	if (err) {
		return false;
	}

	return true;

}

static bool validate_reg_ops(struct dbg_session_gk20a *dbg_s,
			     u32 *ctx_rd_count, u32 *ctx_wr_count,
			     struct nvgpu_dbg_reg_op *ops,
			     u32 op_count);


int exec_regops_gk20a(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_reg_op *ops,
		      u64 num_ops,
		      bool *is_current_ctx)
{
	int err = 0;
	unsigned int i;
	struct channel_gk20a *ch = NULL;
	struct gk20a *g = dbg_s->g;
	/*struct gr_gk20a *gr = &g->gr;*/
	u32 data32_lo = 0, data32_hi = 0;
	u32 ctx_rd_count = 0, ctx_wr_count = 0;
	bool skip_read_lo, skip_read_hi;
	bool ok;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_gpu_dbg, " ");

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);

	/* For vgpu, the regops routines need to be handled in the
	 * context of the server and support for that does not exist.
	 *
	 * The two users of the regops interface are the compute driver
	 * and tools. The compute driver will work without a functional
	 * regops implementation, so we return -ENOSYS. This will allow
	 * compute apps to run with vgpu. Tools will not work in this
	 * configuration and are not required to work at this time. */
	if (g->is_virtual) {
		return -ENOSYS;
	}

	ok = validate_reg_ops(dbg_s,
			      &ctx_rd_count, &ctx_wr_count,
			      ops, num_ops);
	if (!ok) {
		nvgpu_err(g, "invalid op(s)");
		err = -EINVAL;
		/* each op has its own err/status */
		goto clean_up;
	}

	/* be sure that ctx info is in place if there are ctx ops */
	if (ctx_wr_count | ctx_rd_count) {
		if (!gr_context_info_available(&g->gr)) {
			nvgpu_err(g, "gr context data not available");
			return -ENODEV;
		}
	}

	for (i = 0; i < num_ops; i++) {
		/* if it isn't global then it is done in the ctx ops... */
		if (ops[i].type != REGOP(TYPE_GLOBAL)) {
			continue;
		}

		switch (ops[i].op) {

		case REGOP(READ_32):
			ops[i].value_hi = 0;
			ops[i].value_lo = gk20a_readl(g, ops[i].offset);
			nvgpu_log(g, gpu_dbg_gpu_dbg, "read_32 0x%08x from 0x%08x",
				   ops[i].value_lo, ops[i].offset);

			break;

		case REGOP(READ_64):
			ops[i].value_lo = gk20a_readl(g, ops[i].offset);
			ops[i].value_hi =
				gk20a_readl(g, ops[i].offset + 4);

			nvgpu_log(g, gpu_dbg_gpu_dbg, "read_64 0x%08x:%08x from 0x%08x",
				   ops[i].value_hi, ops[i].value_lo,
				   ops[i].offset);
		break;

		case REGOP(WRITE_32):
		case REGOP(WRITE_64):
			/* some of this appears wonky/unnecessary but
			   we've kept it for compat with existing
			   debugger code.  just in case... */
			skip_read_lo = skip_read_hi = false;
			if (ops[i].and_n_mask_lo == ~(u32)0) {
				data32_lo = ops[i].value_lo;
				skip_read_lo = true;
			}

			if ((ops[i].op == REGOP(WRITE_64)) &&
			    (ops[i].and_n_mask_hi == ~(u32)0)) {
				data32_hi = ops[i].value_hi;
				skip_read_hi = true;
			}

			/* read first 32bits */
			if (skip_read_lo == false) {
				data32_lo = gk20a_readl(g, ops[i].offset);
				data32_lo &= ~ops[i].and_n_mask_lo;
				data32_lo |= ops[i].value_lo;
			}

			/* if desired, read second 32bits */
			if ((ops[i].op == REGOP(WRITE_64)) &&
			    !skip_read_hi) {
				data32_hi = gk20a_readl(g, ops[i].offset + 4);
				data32_hi &= ~ops[i].and_n_mask_hi;
				data32_hi |= ops[i].value_hi;
			}

			/* now update first 32bits */
			gk20a_writel(g, ops[i].offset, data32_lo);
			nvgpu_log(g, gpu_dbg_gpu_dbg, "Wrote 0x%08x to 0x%08x ",
				   data32_lo, ops[i].offset);
			/* if desired, update second 32bits */
			if (ops[i].op == REGOP(WRITE_64)) {
				gk20a_writel(g, ops[i].offset + 4, data32_hi);
				nvgpu_log(g, gpu_dbg_gpu_dbg, "Wrote 0x%08x to 0x%08x ",
					   data32_hi, ops[i].offset + 4);

			}


			break;

		/* shouldn't happen as we've already screened */
		default:
			BUG();
			err = -EINVAL;
			goto clean_up;
			break;
		}
	}

	if (ctx_wr_count | ctx_rd_count) {
		err = gr_gk20a_exec_ctx_ops(ch, ops, num_ops,
					    ctx_wr_count, ctx_rd_count,
					    is_current_ctx);
		if (err) {
			nvgpu_warn(g, "failed to perform ctx ops\n");
			goto clean_up;
		}
	}

 clean_up:
	nvgpu_log(g, gpu_dbg_gpu_dbg, "ret=%d", err);
	return err;

}


static int validate_reg_op_info(struct dbg_session_gk20a *dbg_s,
				struct nvgpu_dbg_reg_op *op)
{
	int err = 0;

	op->status = REGOP(STATUS_SUCCESS);

	switch (op->op) {
	case REGOP(READ_32):
	case REGOP(READ_64):
	case REGOP(WRITE_32):
	case REGOP(WRITE_64):
		break;
	default:
		op->status |= REGOP(STATUS_UNSUPPORTED_OP);
		err = -EINVAL;
		break;
	}

	switch (op->type) {
	case REGOP(TYPE_GLOBAL):
	case REGOP(TYPE_GR_CTX):
	case REGOP(TYPE_GR_CTX_TPC):
	case REGOP(TYPE_GR_CTX_SM):
	case REGOP(TYPE_GR_CTX_CROP):
	case REGOP(TYPE_GR_CTX_ZROP):
	case REGOP(TYPE_GR_CTX_QUAD):
		break;
	/*
	case NVGPU_DBG_GPU_REG_OP_TYPE_FB:
	*/
	default:
		op->status |= REGOP(STATUS_INVALID_TYPE);
		err = -EINVAL;
		break;
	}

	return err;
}

static bool check_whitelists(struct dbg_session_gk20a *dbg_s,
			  struct nvgpu_dbg_reg_op *op, u32 offset)
{
	struct gk20a *g = dbg_s->g;
	bool valid = false;
	struct channel_gk20a *ch;

	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);

	if (op->type == REGOP(TYPE_GLOBAL)) {
		/* search global list */
		valid = g->ops.regops.get_global_whitelist_ranges &&
			!!bsearch(&offset,
			g->ops.regops.get_global_whitelist_ranges(),
			g->ops.regops.get_global_whitelist_ranges_count(),
			sizeof(*g->ops.regops.get_global_whitelist_ranges()),
			regop_bsearch_range_cmp);

		/* if debug session and channel is bound search context list */
		if ((!valid) && (!dbg_s->is_profiler && ch)) {
			/* binary search context list */
			valid = g->ops.regops.get_context_whitelist_ranges &&
				!!bsearch(&offset,
			g->ops.regops.get_context_whitelist_ranges(),
			g->ops.regops.get_context_whitelist_ranges_count(),
			sizeof(*g->ops.regops.get_context_whitelist_ranges()),
			regop_bsearch_range_cmp);
		}

		/* if debug session and channel is bound search runcontrol list */
		if ((!valid) && (!dbg_s->is_profiler && ch)) {
			valid = g->ops.regops.get_runcontrol_whitelist &&
				linear_search(offset,
				g->ops.regops.get_runcontrol_whitelist(),
				g->ops.regops.get_runcontrol_whitelist_count());
		}
	} else if (op->type == REGOP(TYPE_GR_CTX)) {
		/* it's a context-relative op */
		if (!ch) {
			nvgpu_err(dbg_s->g, "can't perform ctx regop unless bound");
			op->status = REGOP(STATUS_UNSUPPORTED_OP);
			return valid;
		}

		/* binary search context list */
		valid = g->ops.regops.get_context_whitelist_ranges &&
			!!bsearch(&offset,
			g->ops.regops.get_context_whitelist_ranges(),
			g->ops.regops.get_context_whitelist_ranges_count(),
			sizeof(*g->ops.regops.get_context_whitelist_ranges()),
			regop_bsearch_range_cmp);

		/* if debug session and channel is bound search runcontrol list */
		if ((!valid) && (!dbg_s->is_profiler && ch)) {
			valid = g->ops.regops.get_runcontrol_whitelist &&
				linear_search(offset,
				g->ops.regops.get_runcontrol_whitelist(),
				g->ops.regops.get_runcontrol_whitelist_count());
		}

	} else if (op->type == REGOP(TYPE_GR_CTX_QUAD)) {
		valid = g->ops.regops.get_qctl_whitelist &&
			linear_search(offset,
				g->ops.regops.get_qctl_whitelist(),
				g->ops.regops.get_qctl_whitelist_count());
	}

	return valid;
}

/* note: the op here has already been through validate_reg_op_info */
static int validate_reg_op_offset(struct dbg_session_gk20a *dbg_s,
				  struct nvgpu_dbg_reg_op *op)
{
	int err;
	u32 buf_offset_lo, buf_offset_addr, num_offsets, offset;
	bool valid = false;

	op->status = 0;
	offset = op->offset;

	/* support only 24-bit 4-byte aligned offsets */
	if (offset & 0xFF000003) {
		nvgpu_err(dbg_s->g, "invalid regop offset: 0x%x", offset);
		op->status |= REGOP(STATUS_INVALID_OFFSET);
		return -EINVAL;
	}

	valid = check_whitelists(dbg_s, op, offset);
	if ((op->op == REGOP(READ_64) || op->op == REGOP(WRITE_64)) && valid) {
		valid = check_whitelists(dbg_s, op, offset + 4);
	}

	if (valid && (op->type != REGOP(TYPE_GLOBAL))) {
		err = gr_gk20a_get_ctx_buffer_offsets(dbg_s->g,
						      op->offset,
						      1,
						      &buf_offset_lo,
						      &buf_offset_addr,
						      &num_offsets,
						      op->type == REGOP(TYPE_GR_CTX_QUAD),
						      op->quad);
		if (err) {
			err = gr_gk20a_get_pm_ctx_buffer_offsets(dbg_s->g,
							      op->offset,
							      1,
							      &buf_offset_lo,
							      &buf_offset_addr,
							      &num_offsets);

			if (err) {
				op->status |= REGOP(STATUS_INVALID_OFFSET);
				return -EINVAL;
			}
		}
		if (!num_offsets) {
			op->status |= REGOP(STATUS_INVALID_OFFSET);
			return -EINVAL;
		}
	}

	if (!valid) {
		nvgpu_err(dbg_s->g, "invalid regop offset: 0x%x", offset);
		op->status |= REGOP(STATUS_INVALID_OFFSET);
		return -EINVAL;
	}

	return 0;
}

static bool validate_reg_ops(struct dbg_session_gk20a *dbg_s,
			    u32 *ctx_rd_count, u32 *ctx_wr_count,
			    struct nvgpu_dbg_reg_op *ops,
			    u32 op_count)
{
	u32 i;
	bool ok = true;
	struct gk20a *g = dbg_s->g;

	/* keep going until the end so every op can get
	 * a separate error code if needed */
	for (i = 0; i < op_count; i++) {

		if (validate_reg_op_info(dbg_s, &ops[i]) != 0) {
			ok = false;
		}

		if (reg_op_is_gr_ctx(ops[i].type)) {
			if (reg_op_is_read(ops[i].op)) {
				(*ctx_rd_count)++;
			} else {
				(*ctx_wr_count)++;
			}
		}

		/* if "allow_all" flag enabled, dont validate offset */
		if (!g->allow_all) {
			if (validate_reg_op_offset(dbg_s, &ops[i]) != 0) {
				ok = false;
			}
		}
	}

	nvgpu_log(g, gpu_dbg_gpu_dbg, "ctx_wrs:%d ctx_rds:%d",
		   *ctx_wr_count, *ctx_rd_count);

	return ok;
}

/* exported for tools like cyclestats, etc */
bool is_bar0_global_offset_whitelisted_gk20a(struct gk20a *g, u32 offset)
{
	bool valid = !!bsearch(&offset,
			g->ops.regops.get_global_whitelist_ranges(),
			g->ops.regops.get_global_whitelist_ranges_count(),
			sizeof(*g->ops.regops.get_global_whitelist_ranges()),
			regop_bsearch_range_cmp);
	return valid;
}

bool reg_op_is_gr_ctx(u8 type)
{
	return  type == REGOP(TYPE_GR_CTX) ||
		type == REGOP(TYPE_GR_CTX_TPC) ||
		type == REGOP(TYPE_GR_CTX_SM) ||
		type == REGOP(TYPE_GR_CTX_CROP) ||
		type == REGOP(TYPE_GR_CTX_ZROP) ||
		type == REGOP(TYPE_GR_CTX_QUAD);
}

bool reg_op_is_read(u8 op)
{
	return  op == REGOP(READ_32) ||
		op == REGOP(READ_64);
}
