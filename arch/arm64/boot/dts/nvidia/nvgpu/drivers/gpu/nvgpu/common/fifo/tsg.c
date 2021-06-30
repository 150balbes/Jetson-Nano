/*
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/bug.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/channel.h>
#include <nvgpu/tsg.h>
#include <nvgpu/gk20a.h>

int gk20a_enable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;
	bool is_next, is_ctx_reload;

	gk20a_fifo_disable_tsg_sched(g, tsg);

	/*
	 * Due to h/w bug that exists in Maxwell and Pascal,
	 * we first need to enable all channels with NEXT and CTX_RELOAD set,
	 * and then rest of the channels should be enabled
	 */
	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		is_next = gk20a_fifo_channel_status_is_next(g, ch->chid);
		is_ctx_reload = gk20a_fifo_channel_status_is_ctx_reload(g, ch->chid);

		if (is_next || is_ctx_reload) {
			g->ops.fifo.enable_channel(ch);
		}
	}

	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		is_next = gk20a_fifo_channel_status_is_next(g, ch->chid);
		is_ctx_reload = gk20a_fifo_channel_status_is_ctx_reload(g, ch->chid);

		if (is_next || is_ctx_reload) {
			continue;
		}

		g->ops.fifo.enable_channel(ch);
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	gk20a_fifo_enable_tsg_sched(g, tsg);

	return 0;
}

int gk20a_disable_tsg(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;
	struct channel_gk20a *ch;

	nvgpu_rwsem_down_read(&tsg->ch_list_lock);
	nvgpu_list_for_each_entry(ch, &tsg->ch_list, channel_gk20a, ch_entry) {
		g->ops.fifo.disable_channel(ch);
	}
	nvgpu_rwsem_up_read(&tsg->ch_list_lock);

	return 0;
}

static bool gk20a_is_channel_active(struct gk20a *g, struct channel_gk20a *ch)
{
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_runlist_info_gk20a *runlist;
	unsigned int i;

	for (i = 0; i < f->max_runlists; ++i) {
		runlist = &f->runlist_info[i];
		if (test_bit(ch->chid, runlist->active_channels)) {
			return true;
		}
	}

	return false;
}

/*
 * API to mark channel as part of TSG
 *
 * Note that channel is not runnable when we bind it to TSG
 */
int gk20a_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;

	nvgpu_log_fn(g, " ");

	/* check if channel is already bound to some TSG */
	if (tsg_gk20a_from_ch(ch) != NULL) {
		return -EINVAL;
	}

	/* channel cannot be bound to TSG if it is already active */
	if (gk20a_is_channel_active(tsg->g, ch)) {
		return -EINVAL;
	}


	/* all the channel part of TSG should need to be same runlist_id */
	if (tsg->runlist_id == FIFO_INVAL_TSG_ID) {
		tsg->runlist_id = ch->runlist_id;
	} else if (tsg->runlist_id != ch->runlist_id) {
		nvgpu_err(tsg->g,
			"Error: TSG channel should be share same runlist ch[%d] tsg[%d]",
			ch->runlist_id, tsg->runlist_id);
		return -EINVAL;
	}

	nvgpu_rwsem_down_write(&tsg->ch_list_lock);
	nvgpu_list_add_tail(&ch->ch_entry, &tsg->ch_list);
	ch->tsgid = tsg->tsgid;
	/* channel is serviceable after it is bound to tsg */
	ch->ch_timedout = false;
	nvgpu_rwsem_up_write(&tsg->ch_list_lock);

	nvgpu_ref_get(&tsg->refcount);

	nvgpu_log(g, gpu_dbg_fn, "BIND tsg:%d channel:%d\n",
					tsg->tsgid, ch->chid);

	nvgpu_log_fn(g, "done");
	return 0;
}

/* The caller must ensure that channel belongs to a tsg */
int gk20a_tsg_unbind_channel(struct channel_gk20a *ch)
{
	struct gk20a *g = ch->g;
	struct tsg_gk20a *tsg = tsg_gk20a_from_ch(ch);
	int err;

	if (tsg == NULL) {
		return -EINVAL;
	}

	err = g->ops.fifo.tsg_unbind_channel(ch);
	if (err) {
		nvgpu_err(g, "Channel %d unbind failed, tearing down TSG %d",
			ch->chid, tsg->tsgid);

		gk20a_fifo_abort_tsg(ch->g, tsg, true);
		/* If channel unbind fails, channel is still part of runlist */
		channel_gk20a_update_runlist(ch, false);

		nvgpu_rwsem_down_write(&tsg->ch_list_lock);
		nvgpu_list_del(&ch->ch_entry);
		ch->tsgid = NVGPU_INVALID_TSG_ID;
		nvgpu_rwsem_up_write(&tsg->ch_list_lock);
	}
	nvgpu_log(g, gpu_dbg_fn, "UNBIND tsg:%d channel:%d",
					tsg->tsgid, ch->chid);

	nvgpu_ref_put(&tsg->refcount, gk20a_tsg_release);

	return 0;
}

int gk20a_init_tsg_support(struct gk20a *g, u32 tsgid)
{
	struct tsg_gk20a *tsg = NULL;
	int err;

	if (tsgid >= g->fifo.num_channels) {
		return -EINVAL;
	}

	tsg = &g->fifo.tsg[tsgid];

	tsg->in_use = false;
	tsg->tsgid = tsgid;

	nvgpu_init_list_node(&tsg->ch_list);
	nvgpu_rwsem_init(&tsg->ch_list_lock);

	nvgpu_init_list_node(&tsg->event_id_list);
	err = nvgpu_mutex_init(&tsg->event_id_list_lock);
	if (err) {
		tsg->in_use = true; /* make this TSG unusable */
		return err;
	}

	return 0;
}

int gk20a_tsg_set_runlist_interleave(struct tsg_gk20a *tsg, u32 level)
{
	struct gk20a *g = tsg->g;
	int ret;

	nvgpu_log(g, gpu_dbg_sched, "tsgid=%u interleave=%u", tsg->tsgid, level);

	switch (level) {
	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_LOW:
	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_MEDIUM:
	case NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_HIGH:
		ret = g->ops.fifo.set_runlist_interleave(g, tsg->tsgid,
							0, level);
		if (!ret) {
			tsg->interleave_level = level;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : g->ops.fifo.update_runlist(g, tsg->runlist_id, ~0, true, true);
}

int gk20a_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice)
{
	struct gk20a *g = tsg->g;

	nvgpu_log(g, gpu_dbg_sched, "tsgid=%u timeslice=%u us", tsg->tsgid, timeslice);

	return g->ops.fifo.tsg_set_timeslice(tsg, timeslice);
}

u32 gk20a_tsg_get_timeslice(struct tsg_gk20a *tsg)
{
	struct gk20a *g = tsg->g;

	if (!tsg->timeslice_us) {
		return g->ops.fifo.default_timeslice_us(g);
	}

	return tsg->timeslice_us;
}

static void release_used_tsg(struct fifo_gk20a *f, struct tsg_gk20a *tsg)
{
	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	f->tsg[tsg->tsgid].in_use = false;
	nvgpu_mutex_release(&f->tsg_inuse_mutex);
}

static struct tsg_gk20a *gk20a_tsg_acquire_unused_tsg(struct fifo_gk20a *f)
{
	struct tsg_gk20a *tsg = NULL;
	unsigned int tsgid;

	nvgpu_mutex_acquire(&f->tsg_inuse_mutex);
	for (tsgid = 0; tsgid < f->num_channels; tsgid++) {
		if (!f->tsg[tsgid].in_use) {
			f->tsg[tsgid].in_use = true;
			tsg = &f->tsg[tsgid];
			break;
		}
	}
	nvgpu_mutex_release(&f->tsg_inuse_mutex);

	return tsg;
}

struct tsg_gk20a *gk20a_tsg_open(struct gk20a *g, pid_t pid)
{
	struct tsg_gk20a *tsg;
	int err;

	tsg = gk20a_tsg_acquire_unused_tsg(&g->fifo);
	if (tsg == NULL) {
		return NULL;
	}

	/* we need to allocate this after g->ops.gr.init_fs_state() since
	 * we initialize gr->no_of_sm in this function
	 */
	if (g->gr.no_of_sm == 0U) {
		nvgpu_err(g, "no_of_sm %d not set, failed allocation",
				  g->gr.no_of_sm);
		return NULL;
	}

	err = gk20a_tsg_alloc_sm_error_states_mem(g, tsg, g->gr.no_of_sm);
	if (err != 0) {
		return NULL;
	}

	tsg->g = g;
	tsg->num_active_channels = 0;
	nvgpu_ref_init(&tsg->refcount);

	tsg->vm = NULL;
	tsg->interleave_level = NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_LOW;
	tsg->timeslice_us = 0;
	tsg->timeslice_timeout = 0;
	tsg->timeslice_scale = 0;
	tsg->runlist_id = ~0;
	tsg->tgid = pid;
	tsg->sm_exception_mask_type = NVGPU_SM_EXCEPTION_TYPE_MASK_NONE;

	if (g->ops.fifo.init_eng_method_buffers) {
		g->ops.fifo.init_eng_method_buffers(g, tsg);
	}

	if (g->ops.fifo.tsg_open) {
		err = g->ops.fifo.tsg_open(tsg);
		if (err != 0) {
			nvgpu_err(g, "tsg %d fifo open failed %d",
				  tsg->tsgid, err);
			goto clean_up;
		}
	}

	nvgpu_log(g, gpu_dbg_fn, "tsg opened %d\n", tsg->tsgid);

	return tsg;

clean_up:

	if(tsg->sm_error_states != NULL) {
		nvgpu_kfree(g, tsg->sm_error_states);
		tsg->sm_error_states = NULL;
	}

	nvgpu_ref_put(&tsg->refcount, gk20a_tsg_release);
	return NULL;
}

void gk20a_tsg_release(struct nvgpu_ref *ref)
{
	struct tsg_gk20a *tsg = container_of(ref, struct tsg_gk20a, refcount);
	struct gk20a *g = tsg->g;
	struct gk20a_event_id_data *event_id_data, *event_id_data_temp;

	if (g->ops.fifo.tsg_release != NULL) {
		g->ops.fifo.tsg_release(tsg);
	}

	if (nvgpu_mem_is_valid(&tsg->gr_ctx.mem)) {
		gr_gk20a_free_tsg_gr_ctx(tsg);
	}

	if (g->ops.fifo.deinit_eng_method_buffers != NULL) {
		g->ops.fifo.deinit_eng_method_buffers(g, tsg);
	}

	if (tsg->vm != NULL) {
		nvgpu_vm_put(tsg->vm);
		tsg->vm = NULL;
	}

	if(tsg->sm_error_states != NULL) {
		nvgpu_kfree(g, tsg->sm_error_states);
		tsg->sm_error_states = NULL;
	}

	/* unhook all events created on this TSG */
	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_for_each_entry_safe(event_id_data, event_id_data_temp,
				&tsg->event_id_list,
				gk20a_event_id_data,
				event_id_node) {
		nvgpu_list_del(&event_id_data->event_id_node);
	}
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	release_used_tsg(&g->fifo, tsg);

	tsg->runlist_id = ~0;
	tsg->sm_exception_mask_type = NVGPU_SM_EXCEPTION_TYPE_MASK_NONE;

	nvgpu_log(g, gpu_dbg_fn, "tsg released %d\n", tsg->tsgid);
}

struct tsg_gk20a *tsg_gk20a_from_ch(struct channel_gk20a *ch)
{
	struct tsg_gk20a *tsg = NULL;
	u32 tsgid = ch->tsgid;

	if (tsgid != NVGPU_INVALID_TSG_ID) {
		struct gk20a *g = ch->g;
		struct fifo_gk20a *f = &g->fifo;

		tsg = &f->tsg[tsgid];
	} else {
		nvgpu_log(ch->g, gpu_dbg_fn, "tsgid is invalid for chid: %d",
			ch->chid);
	}
	return tsg;
}

int gk20a_tsg_alloc_sm_error_states_mem(struct gk20a *g,
					struct tsg_gk20a *tsg,
					u32 num_sm)
{
	int err = 0;

	if (tsg->sm_error_states != NULL) {
		return err;
	}

	tsg->sm_error_states = nvgpu_kzalloc(g,
			sizeof(struct nvgpu_tsg_sm_error_state)
			* num_sm);
	if (tsg->sm_error_states == NULL) {
		nvgpu_err(g, "sm_error_states mem allocation failed");
		err = -ENOMEM;
	}

	return err;
}

void gk20a_tsg_update_sm_error_state_locked(struct tsg_gk20a *tsg,
				u32 sm_id,
				struct nvgpu_tsg_sm_error_state *sm_error_state)
{
	struct nvgpu_tsg_sm_error_state *tsg_sm_error_states;

	tsg_sm_error_states = tsg->sm_error_states + sm_id;

	tsg_sm_error_states->hww_global_esr =
			sm_error_state->hww_global_esr;
	tsg_sm_error_states->hww_warp_esr =
			sm_error_state->hww_warp_esr;
	tsg_sm_error_states->hww_warp_esr_pc =
			sm_error_state->hww_warp_esr_pc;
	tsg_sm_error_states->hww_global_esr_report_mask =
			sm_error_state->hww_global_esr_report_mask;
	tsg_sm_error_states->hww_warp_esr_report_mask =
			sm_error_state->hww_warp_esr_report_mask;
}
