/*
 * GP10B GPU FECS traces
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/gk20a.h>

#include "gk20a/fecs_trace_gk20a.h"

#include "fecs_trace_gm20b.h"

#include <nvgpu/hw/gm20b/hw_ctxsw_prog_gm20b.h>
#include <nvgpu/hw/gm20b/hw_gr_gm20b.h>

#ifdef CONFIG_GK20A_CTXSW_TRACE
int gm20b_fecs_trace_flush(struct gk20a *g)
{
	struct fecs_method_op_gk20a op = {
		.mailbox = { .id = 0, .data = 0,
			.clr = ~0, .ok = 0, .fail = 0},
		.method.addr = gr_fecs_method_push_adr_write_timestamp_record_v(),
		.method.data = 0,
		.cond.ok = GR_IS_UCODE_OP_NOT_EQUAL,
		.cond.fail = GR_IS_UCODE_OP_SKIP,
	};
	int err;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, " ");

	err = gr_gk20a_elpg_protected_call(g,
			gr_gk20a_submit_fecs_method_op(g, op, false));
	if (err)
		nvgpu_err(g, "write timestamp record failed");

	return err;
}
#endif /* CONFIG_GK20A_CTXSW_TRACE */
