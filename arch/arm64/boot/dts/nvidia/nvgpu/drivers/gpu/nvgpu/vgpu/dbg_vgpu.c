/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/bug.h>
#include <nvgpu/channel.h>

#include "gk20a/gk20a.h"
#include "gk20a/dbg_gpu_gk20a.h"
#include "gk20a/regops_gk20a.h"
#include "dbg_vgpu.h"

int vgpu_exec_regops(struct dbg_session_gk20a *dbg_s,
		      struct nvgpu_dbg_reg_op *ops,
		      u64 num_ops,
		      bool *is_current_ctx)
{
	struct channel_gk20a *ch;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_reg_ops_params *p = &msg.params.reg_ops;
	void *oob;
	size_t oob_size, ops_size;
	void *handle = NULL;
	int err = 0;
	struct gk20a *g = dbg_s->g;

	nvgpu_log_fn(g, " ");
	BUG_ON(sizeof(*ops) != sizeof(struct tegra_vgpu_reg_op));

	handle = vgpu_ivc_oob_get_ptr(vgpu_ivc_get_server_vmid(),
					TEGRA_VGPU_QUEUE_CMD,
					&oob, &oob_size);
	if (!handle)
		return -EINVAL;

	ops_size = sizeof(*ops) * num_ops;
	if (oob_size < ops_size) {
		err = -ENOMEM;
		goto fail;
	}

	memcpy(oob, ops, ops_size);

	msg.cmd = TEGRA_VGPU_CMD_REG_OPS;
	msg.handle = vgpu_get_handle(dbg_s->g);
	ch = nvgpu_dbg_gpu_get_session_channel(dbg_s);
	p->handle = ch ? ch->virt_ctx : 0;
	p->num_ops = num_ops;
	p->is_profiler = dbg_s->is_profiler;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (!err)
		memcpy(ops, oob, ops_size);

fail:
	vgpu_ivc_oob_put_ptr(handle);
	return err;
}

int vgpu_dbg_set_powergate(struct dbg_session_gk20a *dbg_s, bool disable_powergate)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_set_powergate_params *p = &msg.params.set_powergate;
	int err = 0;
	u32 mode;
	struct gk20a *g = dbg_s->g;

	nvgpu_log_fn(g, " ");

	/* Just return if requested mode is the same as the session's mode */
	if (disable_powergate) {
		if (dbg_s->is_pg_disabled)
			return 0;
		dbg_s->is_pg_disabled = true;
		mode = TEGRA_VGPU_POWERGATE_MODE_DISABLE;
	} else {
		if (!dbg_s->is_pg_disabled)
			return 0;
		dbg_s->is_pg_disabled = false;
		mode = TEGRA_VGPU_POWERGATE_MODE_ENABLE;
	}

	msg.cmd = TEGRA_VGPU_CMD_SET_POWERGATE;
	msg.handle = vgpu_get_handle(dbg_s->g);
	p->mode = mode;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	return err;
}

static int vgpu_sendrecv_prof_cmd(struct dbg_session_gk20a *dbg_s, u32 mode)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_prof_mgt_params *p = &msg.params.prof_management;
	int err = 0;

	msg.cmd = TEGRA_VGPU_CMD_PROF_MGT;
	msg.handle = vgpu_get_handle(dbg_s->g);

	p->mode = mode;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	return err;
}

bool vgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	if (g->profiler_reservation_count > 0)
		return false;

	/* Check that another guest OS doesn't already have a reservation */
	if (!vgpu_sendrecv_prof_cmd(dbg_s, TEGRA_VGPU_PROF_GET_GLOBAL)) {
		g->global_profiler_reservation_held = true;
		g->profiler_reservation_count = 1;
		dbg_s->has_profiler_reservation = true;
		prof_obj->has_reservation = true;
		return true;
	}
	return false;
}

bool vgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	/* Assumes that we've already checked that no global reservation
	 * is in effect for this guest.
	 *
	 * If our reservation count is non-zero, then no other guest has the
	 * global reservation; if it is zero, need to check with RM server.
	 *
	 */
	if ((g->profiler_reservation_count != 0) ||
		!vgpu_sendrecv_prof_cmd(dbg_s, TEGRA_VGPU_PROF_GET_CONTEXT)) {
		g->profiler_reservation_count++;
		dbg_s->has_profiler_reservation = true;
		prof_obj->has_reservation = true;
		return true;
	}
	return false;
}

void vgpu_release_profiler_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj)
{
	struct gk20a *g = dbg_s->g;

	dbg_s->has_profiler_reservation = false;
	prof_obj->has_reservation = false;
	if (prof_obj->ch == NULL)
		g->global_profiler_reservation_held = false;

	/* If new reservation count is zero, notify server */
	g->profiler_reservation_count--;
	if (g->profiler_reservation_count == 0)
		vgpu_sendrecv_prof_cmd(dbg_s, TEGRA_VGPU_PROF_RELEASE);
}

static int vgpu_sendrecv_perfbuf_cmd(struct gk20a *g, u64 offset, u32 size)
{
	struct mm_gk20a *mm = &g->mm;
	struct vm_gk20a *vm = mm->perfbuf.vm;
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_perfbuf_mgt_params *p =
						&msg.params.perfbuf_management;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_PERFBUF_MGT;
	msg.handle = vgpu_get_handle(g);

	p->vm_handle = vm->handle;
	p->offset = offset;
	p->size = size;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	return err;
}

int vgpu_perfbuffer_enable(struct gk20a *g, u64 offset, u32 size)
{
	return vgpu_sendrecv_perfbuf_cmd(g, offset, size);
}

int vgpu_perfbuffer_disable(struct gk20a *g)
{
	return vgpu_sendrecv_perfbuf_cmd(g, 0, 0);
}
