/*
 * Tegra GK20A GPU Debugger Driver
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
#ifndef DBG_GPU_H
#define DBG_GPU_H

#include <nvgpu/cond.h>
#include <nvgpu/lock.h>
#include <nvgpu/list.h>

struct gk20a;
struct channel_gk20a;
struct dbg_session_gk20a;

/* used by the interrupt handler to post events */
void gk20a_dbg_gpu_post_events(struct channel_gk20a *fault_ch);

struct channel_gk20a *
nvgpu_dbg_gpu_get_session_channel(struct dbg_session_gk20a *dbg_s);

struct dbg_gpu_session_events {
	struct nvgpu_cond wait_queue;
	bool events_enabled;
	int num_pending_events;
};

struct dbg_session_gk20a {
	/* dbg session id used for trace/prints */
	int id;

	/* profiler session, if any */
	bool is_profiler;

	/* has a valid profiler reservation */
	bool has_profiler_reservation;

	/* power enabled or disabled */
	bool is_pg_disabled;

	/* timeouts enabled or disabled */
	bool is_timeout_disabled;

	struct gk20a              *g;

	/* list of bound channels, if any */
	struct nvgpu_list_node ch_list;
	struct nvgpu_mutex ch_list_lock;

	/* event support */
	struct dbg_gpu_session_events dbg_events;

	bool broadcast_stop_trigger;

	struct nvgpu_mutex ioctl_lock;
};

struct dbg_session_data {
	struct dbg_session_gk20a *dbg_s;
	struct nvgpu_list_node dbg_s_entry;
};

static inline struct dbg_session_data *
dbg_session_data_from_dbg_s_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_session_data *)
	     ((uintptr_t)node - offsetof(struct dbg_session_data, dbg_s_entry));
};

struct dbg_session_channel_data {
	int channel_fd;
	u32 chid;
	struct nvgpu_list_node ch_entry;
	struct dbg_session_data *session_data;
	int (*unbind_single_channel)(struct dbg_session_gk20a *dbg_s,
			struct dbg_session_channel_data *ch_data);
};

static inline struct dbg_session_channel_data *
dbg_session_channel_data_from_ch_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_session_channel_data *)
	((uintptr_t)node - offsetof(struct dbg_session_channel_data, ch_entry));
};

struct dbg_profiler_object_data {
	int session_id;
	u32 prof_handle;
	struct channel_gk20a *ch;
	bool has_reservation;
	struct nvgpu_list_node prof_obj_entry;
};

static inline struct dbg_profiler_object_data *
dbg_profiler_object_data_from_prof_obj_entry(struct nvgpu_list_node *node)
{
	return (struct dbg_profiler_object_data *)
	((uintptr_t)node - offsetof(struct dbg_profiler_object_data, prof_obj_entry));
};

bool gk20a_dbg_gpu_broadcast_stop_trigger(struct channel_gk20a *ch);
int gk20a_dbg_gpu_clear_broadcast_stop_trigger(struct channel_gk20a *ch);

int dbg_set_powergate(struct dbg_session_gk20a *dbg_s, bool disable_powergate);
bool nvgpu_check_and_set_global_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
bool nvgpu_check_and_set_context_reservation(
				struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
void nvgpu_release_profiler_reservation(struct dbg_session_gk20a *dbg_s,
				struct dbg_profiler_object_data *prof_obj);
int gk20a_perfbuf_enable_locked(struct gk20a *g, u64 offset, u32 size);
int gk20a_perfbuf_disable_locked(struct gk20a *g);

void nvgpu_dbg_session_post_event(struct dbg_session_gk20a *dbg_s);
u32 nvgpu_set_powergate_locked(struct dbg_session_gk20a *dbg_s,
				bool mode);

 /* PM Context Switch Mode */
/*This mode says that the pms are not to be context switched. */
#define NVGPU_DBG_HWPM_CTXSW_MODE_NO_CTXSW               (0x00000000)
/* This mode says that the pms in Mode-B are to be context switched */
#define NVGPU_DBG_HWPM_CTXSW_MODE_CTXSW                  (0x00000001)
/* This mode says that the pms in Mode-E (stream out) are to be context switched. */
#define NVGPU_DBG_HWPM_CTXSW_MODE_STREAM_OUT_CTXSW       (0x00000002)

#endif /* DBG_GPU_GK20A_H */
