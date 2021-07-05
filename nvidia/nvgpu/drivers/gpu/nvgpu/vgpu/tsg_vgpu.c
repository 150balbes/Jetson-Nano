/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include "gk20a/gk20a.h"
#include "fifo_vgpu.h"

#include <nvgpu/channel.h>
#include <nvgpu/tsg.h>
#include <nvgpu/bug.h>

#include <nvgpu/vgpu/tegra_vgpu.h>
#include <nvgpu/vgpu/vgpu.h>

int vgpu_tsg_open(struct tsg_gk20a *tsg)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_open_rel_params *p =
				&msg.params.tsg_open;
	int err;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_TSG_OPEN;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_open failed, tsgid %d", tsg->tsgid);
	}

	return err;
}

void vgpu_tsg_release(struct tsg_gk20a *tsg)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_open_rel_params *p =
				&msg.params.tsg_release;
	int err;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_TSG_RELEASE;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_release failed, tsgid %d", tsg->tsgid);
	}
}

int vgpu_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry)
		g->ops.fifo.enable_channel(ch);
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	return 0;
}

int vgpu_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_unbind_channel_params *p =
				&msg.params.tsg_bind_unbind_channel;
	int err;
	struct gk20a *g = ch->g;

	nvgpu_log_fn(g, " ");

	err = gk20a_tsg_bind_channel(tsg, ch);
	if (err)
		return err;

	msg.cmd = TEGRA_VGPU_CMD_TSG_BIND_CHANNEL;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->ch_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(tsg->g,
			"vgpu_tsg_bind_channel failed, ch %d tsgid %d",
			ch->chid, tsg->tsgid);
		gk20a_tsg_unbind_channel(ch);
	}

	return err;
}

int vgpu_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_tsg_bind_unbind_channel_params *p =
				&msg.params.tsg_bind_unbind_channel;
	int err;
	struct gk20a *g = ch->g;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_TSG_UNBIND_CHANNEL;
	msg.handle = vgpu_get_handle(g);
	p->ch_handle = ch->virt_ctx;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);

	return err;
}

int vgpu_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_tsg_timeslice_params *p =
				&msg.params.tsg_timeslice;
	int err;
	struct gk20a *g = tsg->g;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_TSG_SET_TIMESLICE;
	msg.handle = vgpu_get_handle(tsg->g);
	p->tsg_id = tsg->tsgid;
	p->timeslice_us = timeslice;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	WARN_ON(err);
	if (!err)
		tsg->timeslice_us = timeslice;

	return err;
}
