/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/vgpu/vgpu_ivc.h>
#include <nvgpu/vgpu/vgpu.h>
#include <nvgpu/timers.h>
#include <nvgpu/channel.h>
#include <nvgpu/clk_arb.h>

#include "gk20a/gk20a.h"
#include "fecs_trace_vgpu.h"

int vgpu_comm_init(struct gk20a *g)
{
	size_t queue_sizes[] = { TEGRA_VGPU_QUEUE_SIZES };

	return vgpu_ivc_init(g, 3, queue_sizes, TEGRA_VGPU_QUEUE_CMD,
				ARRAY_SIZE(queue_sizes));
}

void vgpu_comm_deinit(void)
{
	size_t queue_sizes[] = { TEGRA_VGPU_QUEUE_SIZES };

	vgpu_ivc_deinit(TEGRA_VGPU_QUEUE_CMD, ARRAY_SIZE(queue_sizes));
}

int vgpu_comm_sendrecv(struct tegra_vgpu_cmd_msg *msg, size_t size_in,
		size_t size_out)
{
	void *handle;
	size_t size = size_in;
	void *data = msg;
	int err;

	err = vgpu_ivc_sendrecv(vgpu_ivc_get_server_vmid(),
				TEGRA_VGPU_QUEUE_CMD, &handle, &data, &size);
	if (!err) {
		WARN_ON(size < size_out);
		memcpy(msg, data, size_out);
		vgpu_ivc_release(handle);
	}

	return err;
}

u64 vgpu_connect(void)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_connect_params *p = &msg.params.connect;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_CONNECT;
	p->module = TEGRA_VGPU_MODULE_GPU;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	return (err || msg.ret) ? 0 : p->handle;
}

int vgpu_get_attribute(u64 handle, u32 attrib, u32 *value)
{
	struct tegra_vgpu_cmd_msg msg;
	struct tegra_vgpu_attrib_params *p = &msg.params.attrib;
	int err;

	msg.cmd = TEGRA_VGPU_CMD_GET_ATTRIBUTE;
	msg.handle = handle;
	p->attrib = attrib;
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));

	if (err || msg.ret)
		return -1;

	*value = p->value;
	return 0;
}

static void vgpu_handle_channel_event(struct gk20a *g,
			struct tegra_vgpu_channel_event_info *info)
{
	struct tsg_gk20a *tsg;

	if (!info->is_tsg) {
		nvgpu_err(g, "channel event posted");
		return;
	}

	if (info->id >= g->fifo.num_channels ||
		info->event_id >= TEGRA_VGPU_CHANNEL_EVENT_ID_MAX) {
		nvgpu_err(g, "invalid channel event");
		return;
	}

	tsg = &g->fifo.tsg[info->id];

	gk20a_tsg_event_id_post_event(tsg, info->event_id);
}

static void vgpu_channel_abort_cleanup(struct gk20a *g, u32 chid)
{
	struct channel_gk20a *ch = gk20a_channel_from_id(g, chid);

	if (ch == NULL) {
		nvgpu_err(g, "invalid channel id %d", chid);
		return;
	}

	gk20a_channel_set_timedout(ch);
	g->ops.fifo.ch_abort_clean_up(ch);
	gk20a_channel_put(ch);
}

static void vgpu_set_error_notifier(struct gk20a *g,
		struct tegra_vgpu_channel_set_error_notifier *p)
{
	struct channel_gk20a *ch;

	if (p->chid >= g->fifo.num_channels) {
		nvgpu_err(g, "invalid chid %d", p->chid);
		return;
	}

	ch = &g->fifo.channel[p->chid];
	g->ops.fifo.set_error_notifier(ch, p->error);
}

int vgpu_intr_thread(void *dev_id)
{
	struct gk20a *g = dev_id;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	while (true) {
		struct tegra_vgpu_intr_msg *msg;
		u32 sender;
		void *handle;
		size_t size;
		int err;

		err = vgpu_ivc_recv(TEGRA_VGPU_QUEUE_INTR, &handle,
					(void **)&msg, &size, &sender);
		if (err == -ETIME)
			continue;
		if (WARN_ON(err))
			continue;

		if (msg->event == TEGRA_VGPU_EVENT_ABORT) {
			vgpu_ivc_release(handle);
			break;
		}

		switch (msg->event) {
		case TEGRA_VGPU_EVENT_INTR:
			if (msg->unit == TEGRA_VGPU_INTR_GR)
				vgpu_gr_isr(g, &msg->info.gr_intr);
			else if (msg->unit == TEGRA_VGPU_INTR_FIFO)
				vgpu_fifo_isr(g, &msg->info.fifo_intr);
			break;
#ifdef CONFIG_GK20A_CTXSW_TRACE
		case TEGRA_VGPU_EVENT_FECS_TRACE:
			vgpu_fecs_trace_data_update(g);
			break;
#endif
		case TEGRA_VGPU_EVENT_CHANNEL:
			vgpu_handle_channel_event(g, &msg->info.channel_event);
			break;
		case TEGRA_VGPU_EVENT_SM_ESR:
			vgpu_gr_handle_sm_esr_event(g, &msg->info.sm_esr);
			break;
		case TEGRA_VGPU_EVENT_SEMAPHORE_WAKEUP:
			g->ops.semaphore_wakeup(g,
					!!msg->info.sem_wakeup.post_events);
			break;
		case TEGRA_VGPU_EVENT_CHANNEL_CLEANUP:
			vgpu_channel_abort_cleanup(g,
					msg->info.ch_cleanup.chid);
			break;
		case TEGRA_VGPU_EVENT_SET_ERROR_NOTIFIER:
			vgpu_set_error_notifier(g,
						&msg->info.set_error_notifier);
			break;
		default:
			nvgpu_err(g, "unknown event %u", msg->event);
			break;
		}

		vgpu_ivc_release(handle);
	}

	while (!nvgpu_thread_should_stop(&priv->intr_handler))
		nvgpu_msleep(10);
	return 0;
}

void vgpu_remove_support_common(struct gk20a *g)
{
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	struct tegra_vgpu_intr_msg msg;
	int err;

	if (g->dbg_regops_tmp_buf)
		nvgpu_kfree(g, g->dbg_regops_tmp_buf);

	if (g->pmu.remove_support)
		g->pmu.remove_support(&g->pmu);

	if (g->acr.remove_support != NULL) {
		g->acr.remove_support(&g->acr);
	}

	if (g->gr.remove_support)
		g->gr.remove_support(&g->gr);

	if (g->fifo.remove_support)
		g->fifo.remove_support(&g->fifo);

	if (g->mm.remove_support)
		g->mm.remove_support(&g->mm);

	msg.event = TEGRA_VGPU_EVENT_ABORT;
	err = vgpu_ivc_send(vgpu_ivc_get_peer_self(), TEGRA_VGPU_QUEUE_INTR,
				&msg, sizeof(msg));
	if (err)
		nvgpu_log_info(g, "vgpu_ivc_send_returned %d\n", err);

	nvgpu_thread_stop(&priv->intr_handler);

	nvgpu_clk_arb_cleanup_arbiter(g);

	nvgpu_mutex_destroy(&g->clk_arb_enable_lock);
	nvgpu_mutex_destroy(&priv->vgpu_clk_get_freq_lock);

	nvgpu_kfree(g, priv->freqs);
}

void vgpu_detect_chip(struct gk20a *g)
{
	struct nvgpu_gpu_params *p = &g->params;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	p->gpu_arch = priv->constants.arch;
	p->gpu_impl = priv->constants.impl;
	p->gpu_rev = priv->constants.rev;

	nvgpu_log_info(g, "arch: %x, impl: %x, rev: %x\n",
			p->gpu_arch,
			p->gpu_impl,
			p->gpu_rev);
}

int vgpu_init_gpu_characteristics(struct gk20a *g)
{
	int err;

	nvgpu_log_fn(g, " ");

	err = gk20a_init_gpu_characteristics(g);
	if (err)
		return err;

	__nvgpu_set_enabled(g, NVGPU_SUPPORT_MAP_BUFFER_BATCH, false);

	/* features vgpu does not support */
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_RESCHEDULE_RUNLIST, false);
	__nvgpu_set_enabled(g, NVGPU_SUPPORT_SET_CTX_MMU_DEBUG_MODE, false);

	return 0;
}

int vgpu_read_ptimer(struct gk20a *g, u64 *value)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_read_ptimer_params *p = &msg.params.read_ptimer;
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_READ_PTIMER;
	msg.handle = vgpu_get_handle(g);

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (!err)
		*value = p->time;
	else
		nvgpu_err(g, "vgpu read ptimer failed, err=%d", err);

	return err;
}

int vgpu_get_timestamps_zipper(struct gk20a *g,
		u32 source_id, u32 count,
		struct nvgpu_cpu_time_correlation_sample *samples)
{
	struct tegra_vgpu_cmd_msg msg = {0};
	struct tegra_vgpu_get_timestamps_zipper_params *p =
			&msg.params.get_timestamps_zipper;
	int err;
	u32 i;

	nvgpu_log_fn(g, " ");

	if (count > TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_MAX_COUNT) {
		nvgpu_err(g, "count %u overflow", count);
		return -EINVAL;
	}

	msg.cmd = TEGRA_VGPU_CMD_GET_TIMESTAMPS_ZIPPER;
	msg.handle = vgpu_get_handle(g);
	p->source_id = TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_SRC_ID_TSC;
	p->count = count;

	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err) {
		nvgpu_err(g, "vgpu get timestamps zipper failed, err=%d", err);
		return err;
	}

	for (i = 0; i < count; i++) {
		samples[i].cpu_timestamp = p->samples[i].cpu_timestamp;
		samples[i].gpu_timestamp = p->samples[i].gpu_timestamp;
	}

	return err;
}

int vgpu_init_hal(struct gk20a *g)
{
	u32 ver = g->params.gpu_arch + g->params.gpu_impl;
	int err;

	switch (ver) {
	case NVGPU_GPUID_GP10B:
		nvgpu_log_info(g, "gp10b detected");
		err = vgpu_gp10b_init_hal(g);
		break;
	case NVGPU_GPUID_GV11B:
		err = vgpu_gv11b_init_hal(g);
		break;
	default:
		nvgpu_err(g, "no support for %x", ver);
		err = -ENODEV;
		break;
	}

	return err;
}

int vgpu_get_constants(struct gk20a *g)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct tegra_vgpu_constants_params *p = &msg.params.constants;
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);
	int err;

	nvgpu_log_fn(g, " ");

	msg.cmd = TEGRA_VGPU_CMD_GET_CONSTANTS;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;

	if (unlikely(err)) {
		nvgpu_err(g, "%s failed, err=%d", __func__, err);
		return err;
	}

	if (unlikely(p->gpc_count > TEGRA_VGPU_MAX_GPC_COUNT ||
		p->max_tpc_per_gpc_count > TEGRA_VGPU_MAX_TPC_COUNT_PER_GPC)) {
		nvgpu_err(g, "gpc_count %d max_tpc_per_gpc %d overflow",
			(int)p->gpc_count, (int)p->max_tpc_per_gpc_count);
		return -EINVAL;
	}

	priv->constants = *p;
	return 0;
}
