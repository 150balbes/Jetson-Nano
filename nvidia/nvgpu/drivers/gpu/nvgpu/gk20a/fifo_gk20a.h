/*
 * GK20A graphics fifo (gr host)
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef FIFO_GK20A_H
#define FIFO_GK20A_H

#include <nvgpu/kref.h>

struct gk20a_debug_output;
struct mmu_fault_info;
struct nvgpu_semaphore;
struct channel_gk20a;
struct tsg_gk20a;

enum {
	NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_LOW = 0,
	NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_MEDIUM,
	NVGPU_FIFO_RUNLIST_INTERLEAVE_LEVEL_HIGH,
	NVGPU_FIFO_RUNLIST_INTERLEAVE_NUM_LEVELS,
};

#define MAX_RUNLIST_BUFFERS		2

#define FIFO_INVAL_ENGINE_ID		((u32)~0)
#define FIFO_INVAL_CHANNEL_ID		((u32)~0)
#define FIFO_INVAL_TSG_ID		((u32)~0)
#define FIFO_INVAL_RUNLIST_ID		((u32)~0)

#define ID_TYPE_CHANNEL			0
#define ID_TYPE_TSG			1
#define ID_TYPE_UNKNOWN			((u32)~0)

#define RC_YES				1
#define RC_NO				0

#define GRFIFO_TIMEOUT_CHECK_PERIOD_US 100000

#define RC_TYPE_NO_RC			0
#define RC_TYPE_MMU_FAULT		1
#define RC_TYPE_PBDMA_FAULT		2
#define RC_TYPE_GR_FAULT		3
#define RC_TYPE_PREEMPT_TIMEOUT		4
#define RC_TYPE_CTXSW_TIMEOUT		5
#define RC_TYPE_RUNLIST_UPDATE_TIMEOUT	6
#define RC_TYPE_FORCE_RESET		7
#define RC_TYPE_SCHED_ERR		8

#define NVGPU_FIFO_DEFAULT_TIMESLICE_TIMEOUT	128UL
#define NVGPU_FIFO_DEFAULT_TIMESLICE_SCALE	3UL

/*
 * Number of entries in the kickoff latency buffer, used to calculate
 * the profiling and histogram. This number is calculated to be statistically
 * significative on a histogram on a 5% step
 */
#ifdef CONFIG_DEBUG_FS
#define FIFO_PROFILING_ENTRIES	16384
#endif

#define	RUNLIST_DISABLED		0
#define	RUNLIST_ENABLED			1

/* generally corresponds to the "pbdma" engine */

struct fifo_runlist_info_gk20a {
	unsigned long *active_channels;
	unsigned long *active_tsgs;
	/* Each engine has its own SW and HW runlist buffer.*/
	struct nvgpu_mem mem[MAX_RUNLIST_BUFFERS];
	u32  cur_buffer;
	u32  total_entries;
	u32  pbdma_bitmask;      /* pbdmas supported for this runlist*/
	u32  eng_bitmask;        /* engines using this runlist */
	u32  reset_eng_bitmask;  /* engines to be reset during recovery */
	u32  count;              /* cached runlist_hw_submit parameter */
	bool stopped;
	bool support_tsg;
	/* protect ch/tsg/runlist preempt & runlist update */
	struct nvgpu_mutex runlist_lock;
};

enum {
	ENGINE_GR_GK20A	       = 0U,
	ENGINE_GRCE_GK20A      = 1U,
	ENGINE_ASYNC_CE_GK20A  = 2U,
	ENGINE_INVAL_GK20A     = 3U,
};

struct fifo_pbdma_exception_info_gk20a {
	u32 status_r; /* raw register value from hardware */
	u32 id, next_id;
	u32 chan_status_v; /* raw value from hardware */
	bool id_is_chid, next_id_is_chid;
	bool chsw_in_progress;
};

struct fifo_engine_exception_info_gk20a {
	u32 status_r; /* raw register value from hardware */
	u32 id, next_id;
	u32 ctx_status_v; /* raw value from hardware */
	bool id_is_chid, next_id_is_chid;
	bool faulted, idle, ctxsw_in_progress;
};

struct fifo_engine_info_gk20a {
	u32 engine_id;
	u32 runlist_id;
	u32 intr_mask;
	u32 reset_mask;
	u32 pbdma_id;
	u32 inst_id;
	u32 pri_base;
	u32 fault_id;
	u32 engine_enum;
	struct fifo_pbdma_exception_info_gk20a pbdma_exception_info;
	struct fifo_engine_exception_info_gk20a engine_exception_info;
};

enum {
	PROFILE_IOCTL_ENTRY = 0U,
	PROFILE_ENTRY,
	PROFILE_JOB_TRACKING,
	PROFILE_APPEND,
	PROFILE_END,
	PROFILE_IOCTL_EXIT,
	PROFILE_MAX
};

struct fifo_profile_gk20a {
	u64 timestamp[PROFILE_MAX];
};

struct fifo_gk20a {
	struct gk20a *g;
	unsigned int num_channels;
	unsigned int runlist_entry_size;
	unsigned int num_runlist_entries;

	unsigned int num_pbdma;
	u32 *pbdma_map;

	struct fifo_engine_info_gk20a *engine_info;
	u32 max_engines;
	u32 num_engines;
	u32 *active_engines_list;

	struct fifo_runlist_info_gk20a *runlist_info;
	u32 max_runlists;
#ifdef CONFIG_DEBUG_FS
	struct {
		struct fifo_profile_gk20a *data;
		nvgpu_atomic_t get;
		bool enabled;
		u64 *sorted;
		struct nvgpu_ref ref;
		struct nvgpu_mutex lock;
	} profile;
#endif
	struct nvgpu_mem userd;
	u32 userd_entry_size;

	unsigned int used_channels;
	struct channel_gk20a *channel;
	/* zero-kref'd channels here */
	struct nvgpu_list_node free_chs;
	struct nvgpu_mutex free_chs_mutex;
	struct nvgpu_mutex engines_reset_mutex;

	struct tsg_gk20a *tsg;
	struct nvgpu_mutex tsg_inuse_mutex;

	void (*remove_support)(struct fifo_gk20a *);
	bool sw_ready;
	struct {
		/* share info between isrs and non-isr code */
		struct {
			struct nvgpu_mutex mutex;
		} isr;
		struct {
			u32 device_fatal_0;
			u32 channel_fatal_0;
			u32 restartable_0;
		} pbdma;
		struct {

		} engine;


	} intr;

	unsigned long deferred_fault_engines;
	bool deferred_reset_pending;
	struct nvgpu_mutex deferred_reset_mutex;

	u32 max_subctx_count;
	u32 channel_base;
};

struct ch_state {
	int pid;
	int refs;
	bool deterministic;
	u32 inst_block[0];
};

int gk20a_init_fifo_support(struct gk20a *g);

int gk20a_init_fifo_setup_hw(struct gk20a *g);

void gk20a_fifo_isr(struct gk20a *g);
u32 gk20a_fifo_nonstall_isr(struct gk20a *g);

int gk20a_fifo_preempt_channel(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_fifo_preempt_tsg(struct gk20a *g, struct tsg_gk20a *tsg);
int gk20a_fifo_preempt(struct gk20a *g, struct channel_gk20a *ch);

int gk20a_fifo_enable_engine_activity(struct gk20a *g,
			struct fifo_engine_info_gk20a *eng_info);
int gk20a_fifo_enable_all_engine_activity(struct gk20a *g);
int gk20a_fifo_disable_engine_activity(struct gk20a *g,
			struct fifo_engine_info_gk20a *eng_info,
			bool wait_for_idle);
int gk20a_fifo_disable_all_engine_activity(struct gk20a *g,
				bool wait_for_idle);
void gk20a_fifo_enable_tsg_sched(struct gk20a *g, struct tsg_gk20a *tsg);
void gk20a_fifo_disable_tsg_sched(struct gk20a *g, struct tsg_gk20a *tsg);

u32 gk20a_fifo_engines_on_ch(struct gk20a *g, u32 chid);

int gk20a_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next);
int nvgpu_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next,
		bool wait_preempt);

int gk20a_fifo_update_runlist(struct gk20a *g, u32 engine_id, u32 chid,
			      bool add, bool wait_for_finish);

int gk20a_fifo_update_runlist_locked(struct gk20a *g, u32 runlist_id,
					    u32 chid, bool add,
					    bool wait_for_finish);
int gk20a_fifo_suspend(struct gk20a *g);

bool gk20a_fifo_mmu_fault_pending(struct gk20a *g);

void gk20a_fifo_recover(struct gk20a *g,
			u32 engine_ids, /* if zero, will be queried from HW */
			u32 hw_id, /* if ~0, will be queried from HW */
			bool id_is_tsg, /* ignored if hw_id == ~0 */
			bool id_is_known, bool verbose, int rc_type);
void gk20a_fifo_recover_ch(struct gk20a *g, struct channel_gk20a *ch,
	bool verbose, u32 rc_type);
void gk20a_fifo_recover_tsg(struct gk20a *g, struct tsg_gk20a *tsg,
			 bool verbose, u32 rc_type);
int gk20a_fifo_force_reset_ch(struct channel_gk20a *ch,
				u32 err_code, bool verbose);
void gk20a_fifo_reset_engine(struct gk20a *g, u32 engine_id);
int gk20a_init_fifo_reset_enable_hw(struct gk20a *g);
int gk20a_fifo_tsg_unbind_channel(struct channel_gk20a *ch);

void fifo_gk20a_finish_mmu_fault_handling(struct gk20a *g,
		unsigned long fault_id);
int gk20a_fifo_wait_engine_idle(struct gk20a *g);
bool gk20a_fifo_is_engine_busy(struct gk20a *g);
u32 gk20a_fifo_engine_interrupt_mask(struct gk20a *g);
u32 gk20a_fifo_act_eng_interrupt_mask(struct gk20a *g, u32 act_eng_id);
u32 gk20a_fifo_get_pbdma_signature(struct gk20a *g);
u32 gk20a_fifo_get_failing_engine_data(struct gk20a *g,
		int *__id, bool *__is_tsg);
void gk20a_fifo_set_ctx_mmu_error_tsg(struct gk20a *g,
		struct tsg_gk20a *tsg);
void gk20a_fifo_abort_tsg(struct gk20a *g, struct tsg_gk20a *tsg, bool preempt);
void gk20a_fifo_set_ctx_mmu_error_ch(struct gk20a *g,
		struct channel_gk20a *refch);
bool gk20a_fifo_error_tsg(struct gk20a *g, struct tsg_gk20a *tsg);
bool gk20a_fifo_error_ch(struct gk20a *g, struct channel_gk20a *refch);

void gk20a_fifo_issue_preempt(struct gk20a *g, u32 id, bool is_tsg);
int gk20a_fifo_set_runlist_interleave(struct gk20a *g,
				u32 id,
				u32 runlist_id,
				u32 new_level);
int gk20a_fifo_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);

const char *gk20a_fifo_interleave_level_name(u32 interleave_level);

int gk20a_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
		u32 *inst_id);

u32 gk20a_fifo_get_engine_ids(struct gk20a *g, u32 engine_id[],
				 u32 engine_id_sz, u32 engine_enum);

void gk20a_fifo_delete_runlist(struct fifo_gk20a *f);

struct fifo_engine_info_gk20a *gk20a_fifo_get_engine_info(struct gk20a *g,
							 u32 engine_id);

bool gk20a_fifo_is_valid_engine_id(struct gk20a *g, u32 engine_id);

u32 gk20a_fifo_get_gr_engine_id(struct gk20a *g);

int gk20a_fifo_deferred_reset(struct gk20a *g, struct channel_gk20a *ch);

u32 gk20a_fifo_get_all_ce_engine_reset_mask(struct gk20a *g);

u32 gk20a_fifo_get_fast_ce_runlist_id(struct gk20a *g);

u32 gk20a_fifo_get_gr_runlist_id(struct gk20a *g);

bool gk20a_fifo_is_valid_runlist_id(struct gk20a *g, u32 runlist_id);

int gk20a_fifo_update_runlist_ids(struct gk20a *g, u32 runlist_ids, u32 chid,
		bool add, bool wait_for_finish);

int gk20a_fifo_init_engine_info(struct fifo_gk20a *f);

void gk20a_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist);
void gk20a_get_ch_runlist_entry(struct channel_gk20a *ch, u32 *runlist);
void gk20a_fifo_set_runlist_state(struct gk20a *g, u32 runlists_mask,
		 u32 runlist_state);

u32 gk20a_fifo_userd_gp_get(struct gk20a *g, struct channel_gk20a *c);
void gk20a_fifo_userd_gp_put(struct gk20a *g, struct channel_gk20a *c);
u64 gk20a_fifo_userd_pb_get(struct gk20a *g, struct channel_gk20a *c);

bool gk20a_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid);
#ifdef CONFIG_DEBUG_FS
struct fifo_profile_gk20a *gk20a_fifo_profile_acquire(struct gk20a *g);
void gk20a_fifo_profile_release(struct gk20a *g,
	struct fifo_profile_gk20a *profile);
void gk20a_fifo_profile_snapshot(struct fifo_profile_gk20a *profile, int idx);
#else
static inline struct fifo_profile_gk20a *
gk20a_fifo_profile_acquire(struct gk20a *g)
{
	return NULL;
}
static inline void gk20a_fifo_profile_release(struct gk20a *g,
	struct fifo_profile_gk20a *profile)
{
}
static inline void gk20a_fifo_profile_snapshot(
		struct fifo_profile_gk20a *profile, int idx)
{
}
#endif

void gk20a_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 chid,
				     struct ch_state *ch_state);
void gk20a_debug_dump_all_channel_status_ramfc(struct gk20a *g,
		 struct gk20a_debug_output *o);
void gk20a_dump_pbdma_status(struct gk20a *g,
				 struct gk20a_debug_output *o);
void gk20a_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o);
const char *gk20a_decode_ccsr_chan_status(u32 index);
const char *gk20a_decode_pbdma_chan_eng_ctx_status(u32 index);
void gk20a_fifo_enable_channel(struct channel_gk20a *ch);
void gk20a_fifo_disable_channel(struct channel_gk20a *ch);

bool gk20a_fifo_channel_status_is_next(struct gk20a *g, u32 chid);
bool gk20a_fifo_channel_status_is_ctx_reload(struct gk20a *g, u32 chid);
int gk20a_fifo_tsg_unbind_channel_verify_status(struct channel_gk20a *ch);

struct channel_gk20a *gk20a_refch_from_inst_ptr(struct gk20a *g, u64 inst_ptr);
void gk20a_fifo_channel_unbind(struct channel_gk20a *ch_gk20a);

u32 gk20a_fifo_intr_0_error_mask(struct gk20a *g);

int gk20a_fifo_is_preempt_pending(struct gk20a *g, u32 id,
			unsigned int id_type);
int __locked_fifo_preempt(struct gk20a *g, u32 id, bool is_tsg);
void gk20a_fifo_preempt_timeout_rc_tsg(struct gk20a *g, struct tsg_gk20a *tsg);
void gk20a_fifo_preempt_timeout_rc(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_fifo_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long timeout, u32 flags);
void gk20a_fifo_setup_ramfc_for_privileged_channel(struct channel_gk20a *c);
int gk20a_fifo_alloc_inst(struct gk20a *g, struct channel_gk20a *ch);
void gk20a_fifo_free_inst(struct gk20a *g, struct channel_gk20a *ch);
int gk20a_fifo_setup_userd(struct channel_gk20a *c);
u32 gk20a_fifo_pbdma_acquire_val(u64 timeout);


u32 *gk20a_runlist_construct_locked(struct fifo_gk20a *f,
				struct fifo_runlist_info_gk20a *runlist,
				u32 cur_level,
				u32 *runlist_entry,
				bool interleave_enabled,
				bool prev_empty,
				u32 *entries_left);
void gk20a_fifo_runlist_hw_submit(struct gk20a *g, u32 runlist_id,
	u32 count, u32 buffer_index);
int gk20a_fifo_runlist_wait_pending(struct gk20a *g, u32 runlist_id);
int gk20a_init_fifo_setup_sw_common(struct gk20a *g);
int gk20a_init_fifo_setup_sw(struct gk20a *g);
void gk20a_fifo_handle_runlist_event(struct gk20a *g);
bool gk20a_fifo_should_defer_engine_reset(struct gk20a *g, u32 engine_id,
			u32 engine_subid, bool fake_fault);

void gk20a_fifo_teardown_ch_tsg(struct gk20a *g, u32 __engine_ids,
			u32 hw_id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault);

bool gk20a_fifo_check_ch_ctxsw_timeout(struct channel_gk20a *ch,
			bool *verbose, u32 *ms);
bool gk20a_fifo_check_tsg_ctxsw_timeout(struct tsg_gk20a *tsg,
			bool *verbose, u32 *ms);
void gk20a_fifo_teardown_mask_intr(struct gk20a *g);
void gk20a_fifo_teardown_unmask_intr(struct gk20a *g);
bool gk20a_fifo_handle_sched_error(struct gk20a *g);

void gk20a_fifo_reset_pbdma_method(struct gk20a *g, int pbdma_id,
			 int pbdma_method_index);
unsigned int gk20a_fifo_handle_pbdma_intr_0(struct gk20a *g, u32 pbdma_id,
			u32 pbdma_intr_0, u32 *handled, u32 *error_notifier);
unsigned int gk20a_fifo_handle_pbdma_intr_1(struct gk20a *g, u32 pbdma_id,
			u32 pbdma_intr_1, u32 *handled, u32 *error_notifier);
u32 gk20a_fifo_handle_pbdma_intr(struct gk20a *g, struct fifo_gk20a *f,
			u32 pbdma_id, unsigned int rc);

u32 gk20a_fifo_default_timeslice_us(struct gk20a *g);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
void gk20a_fifo_add_syncpt_wait_cmd(struct gk20a *g,
			struct priv_cmd_entry *cmd, u32 off,
			u32 id, u32 thresh, u64 gpu_va);
u32 gk20a_fifo_get_syncpt_wait_cmd_size(void);
u32 gk20a_fifo_get_syncpt_incr_per_release(void);
void gk20a_fifo_add_syncpt_incr_cmd(struct gk20a *g,
			bool wfi_cmd, struct priv_cmd_entry *cmd,
			u32 id, u64 gpu_va);
u32 gk20a_fifo_get_syncpt_incr_cmd_size(bool wfi_cmd);
void gk20a_fifo_free_syncpt_buf(struct channel_gk20a *c,
			struct nvgpu_mem *syncpt_buf);
int gk20a_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf);
#endif

void gk20a_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
				struct mmu_fault_info *mmfault);
void gk20a_fifo_get_mmu_fault_desc(struct mmu_fault_info *mmfault);
void gk20a_fifo_get_mmu_fault_client_desc(struct mmu_fault_info *mmfault);
void gk20a_fifo_get_mmu_fault_gpc_desc(struct mmu_fault_info *mmfault);
u32 gk20a_fifo_get_sema_wait_cmd_size(void);
u32 gk20a_fifo_get_sema_incr_cmd_size(void);
void gk20a_fifo_add_sema_cmd(struct gk20a *g,
	struct nvgpu_semaphore *s, u64 sema_va,
	struct priv_cmd_entry *cmd,
	u32 off, bool acquire, bool wfi);
#endif /* FIFO_GK20A_H */
