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
#ifndef TSG_GK20A_H
#define TSG_GK20A_H

#include <nvgpu/lock.h>
#include <nvgpu/kref.h>
#include <nvgpu/rwsem.h>

#include "gk20a/gr_gk20a.h"

#define NVGPU_INVALID_TSG_ID (U32_MAX)

struct channel_gk20a;

struct tsg_gk20a *gk20a_tsg_open(struct gk20a *g, pid_t pid);
void gk20a_tsg_release(struct nvgpu_ref *ref);

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid);
struct tsg_gk20a *tsg_gk20a_from_ch(struct channel_gk20a *ch);

struct nvgpu_tsg_sm_error_state {
	u32 hww_global_esr;
	u32 hww_warp_esr;
	u64 hww_warp_esr_pc;
	u32 hww_global_esr_report_mask;
	u32 hww_warp_esr_report_mask;
};

struct tsg_gk20a {
	struct gk20a *g;

	struct vm_gk20a *vm;
	struct nvgpu_mem *eng_method_buffers;


	struct nvgpu_gr_ctx gr_ctx;
	struct nvgpu_ref refcount;

	struct nvgpu_list_node ch_list;
	struct nvgpu_list_node event_id_list;
	struct nvgpu_rwsem ch_list_lock;
	struct nvgpu_mutex event_id_list_lock;
	int num_active_channels;

	unsigned int timeslice_us;
	unsigned int timeslice_timeout;
	unsigned int timeslice_scale;

	u32 interleave_level;
	u32 tsgid;

	u32 runlist_id;
	pid_t tgid;
	u32  num_active_tpcs;
	u8   tpc_pg_enabled;
	bool tpc_num_initialized;
	bool in_use;

	/* MMU debug mode enabled if mmu_debug_mode_refcnt > 0 */
	u32  mmu_debug_mode_refcnt;

	struct nvgpu_tsg_sm_error_state *sm_error_states;

#define NVGPU_SM_EXCEPTION_TYPE_MASK_NONE		(0x0U)
#define NVGPU_SM_EXCEPTION_TYPE_MASK_FATAL		(0x1U << 0)
	u32 sm_exception_mask_type;
};

int gk20a_enable_tsg(struct tsg_gk20a *tsg);
int gk20a_disable_tsg(struct tsg_gk20a *tsg);
int gk20a_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch);
int gk20a_tsg_unbind_channel(struct channel_gk20a *ch);

void gk20a_tsg_event_id_post_event(struct tsg_gk20a *tsg,
				       int event_id);
int gk20a_tsg_set_runlist_interleave(struct tsg_gk20a *tsg, u32 level);
int gk20a_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);
u32 gk20a_tsg_get_timeslice(struct tsg_gk20a *tsg);
int gk20a_tsg_set_priority(struct gk20a *g, struct tsg_gk20a *tsg,
				u32 priority);
int gk20a_tsg_alloc_sm_error_states_mem(struct gk20a *g,
					struct tsg_gk20a *tsg,
					u32 num_sm);
void gk20a_tsg_update_sm_error_state_locked(struct tsg_gk20a *tsg,
			u32 sm_id,
			struct nvgpu_tsg_sm_error_state *sm_error_state);

struct gk20a_event_id_data {
	struct gk20a *g;

	int id; /* ch or tsg */
	int pid;
	u32 event_id;

	bool event_posted;

	struct nvgpu_cond event_id_wq;
	struct nvgpu_mutex lock;
	struct nvgpu_list_node event_id_node;
};

static inline struct gk20a_event_id_data *
gk20a_event_id_data_from_event_id_node(struct nvgpu_list_node *node)
{
	return (struct gk20a_event_id_data *)
		((uintptr_t)node - offsetof(struct gk20a_event_id_data, event_id_node));
};

int nvgpu_tsg_set_mmu_debug_mode(struct channel_gk20a *ch, bool enable);

#endif /* TSG_GK20A_H */
