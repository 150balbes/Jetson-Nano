/*
 * GV11B Fifo
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

#ifndef FIFO_GV11B_H
#define FIFO_GV11B_H

#define PBDMA_SUBDEVICE_ID  1

#define FIFO_INVAL_PBDMA_ID	((u32)~0)
#define FIFO_INVAL_VEID		((u32)~0)

/* engine context-switch request occurred while the engine was in reset */
#define SCHED_ERROR_CODE_ENGINE_RESET      0x00000005

/*
* ERROR_CODE_BAD_TSG indicates that Host encountered a badly formed TSG header
* or a badly formed channel type runlist entry in the runlist. This is typically
* caused by encountering a new TSG entry in the middle of a TSG definition.
* A channel type entry having wrong runqueue selector can also cause this.
* Additionally this error code can indicate when a channel is encountered on
* the runlist which is outside of a TSG.
*/
#define SCHED_ERROR_CODE_BAD_TSG           0x00000020

/* can be removed after runque support is added */

#define GR_RUNQUE			0	/* pbdma 0 */
#define ASYNC_CE_RUNQUE			2	/* pbdma 2 */

#define CHANNEL_INFO_VEID0		0

#define MAX_PRE_SI_RETRIES		200000	/* 1G/500KHz * 100 */

struct gpu_ops;

void gv11b_fifo_reset_pbdma_and_eng_faulted(struct gk20a *g,
			struct tsg_gk20a *tsg,
			u32 faulted_pbdma, u32 faulted_engine);
void gv11b_mmu_fault_id_to_eng_pbdma_id_and_veid(struct gk20a *g,
	u32 mmu_fault_id, u32 *active_engine_id, u32 *veid, u32 *pbdma_id);

void gv11b_get_tsg_runlist_entry(struct tsg_gk20a *tsg, u32 *runlist);
void gv11b_get_ch_runlist_entry(struct channel_gk20a *c, u32 *runlist);
int channel_gv11b_setup_ramfc(struct channel_gk20a *c,
		u64 gpfifo_base, u32 gpfifo_entries,
		unsigned long acquire_timeout, u32 flags);
u32 gv11b_userd_gp_get(struct gk20a *g, struct channel_gk20a *c);
u64 gv11b_userd_pb_get(struct gk20a *g, struct channel_gk20a *c);
void gv11b_userd_gp_put(struct gk20a *g, struct channel_gk20a *c);
void channel_gv11b_unbind(struct channel_gk20a *ch);
u32 gv11b_fifo_get_num_fifos(struct gk20a *g);
bool gv11b_is_fault_engine_subid_gpc(struct gk20a *g, u32 engine_subid);
void gv11b_dump_channel_status_ramfc(struct gk20a *g,
				     struct gk20a_debug_output *o,
				     u32 chid,
				     struct ch_state *ch_state);
void gv11b_dump_eng_status(struct gk20a *g,
				 struct gk20a_debug_output *o);
u32 gv11b_fifo_intr_0_error_mask(struct gk20a *g);
int gv11b_fifo_reschedule_runlist(struct channel_gk20a *ch, bool preempt_next);
int gv11b_fifo_is_preempt_pending(struct gk20a *g, u32 id,
		 unsigned int id_type);
int gv11b_fifo_preempt_channel(struct gk20a *g, struct channel_gk20a *ch);
int gv11b_fifo_preempt_tsg(struct gk20a *g, struct tsg_gk20a *tsg);
int gv11b_fifo_enable_tsg(struct tsg_gk20a *tsg);
void gv11b_fifo_teardown_ch_tsg(struct gk20a *g, u32 act_eng_bitmask,
			u32 id, unsigned int id_type, unsigned int rc_type,
			 struct mmu_fault_info *mmfault);
void gv11b_fifo_teardown_mask_intr(struct gk20a *g);
void gv11b_fifo_teardown_unmask_intr(struct gk20a *g);
void gv11b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f);
int gv11b_init_fifo_reset_enable_hw(struct gk20a *g);
bool gv11b_fifo_handle_sched_error(struct gk20a *g);
bool gv11b_fifo_handle_ctxsw_timeout(struct gk20a *g, u32 fifo_intr);
unsigned int gv11b_fifo_handle_pbdma_intr_0(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_0,
			u32 *handled, u32 *error_notifier);
unsigned int gv11b_fifo_handle_pbdma_intr_1(struct gk20a *g,
			u32 pbdma_id, u32 pbdma_intr_1,
			u32 *handled, u32 *error_notifier);
void gv11b_fifo_init_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg);
void gv11b_fifo_deinit_eng_method_buffers(struct gk20a *g,
					struct tsg_gk20a *tsg);
int gv11b_fifo_alloc_syncpt_buf(struct channel_gk20a *c,
			u32 syncpt_id, struct nvgpu_mem *syncpt_buf);
void gv11b_fifo_free_syncpt_buf(struct channel_gk20a *c,
					struct nvgpu_mem *syncpt_buf);
int gv11b_fifo_get_sync_ro_map(struct vm_gk20a *vm,
	u64 *base_gpuva, u32 *sync_size);
u32 gv11b_fifo_get_sema_wait_cmd_size(void);
u32 gv11b_fifo_get_sema_incr_cmd_size(void);
void gv11b_fifo_add_sema_cmd(struct gk20a *g,
	struct nvgpu_semaphore *s, u64 sema_va,
	struct priv_cmd_entry *cmd,
	u32 off, bool acquire, bool wfi);
void gv11b_fifo_add_syncpt_wait_cmd(struct gk20a *g,
		struct priv_cmd_entry *cmd, u32 off,
		u32 id, u32 thresh, u64 gpu_va_base);
u32 gv11b_fifo_get_syncpt_wait_cmd_size(void);
u32 gv11b_fifo_get_syncpt_incr_per_release(void);
void gv11b_fifo_add_syncpt_incr_cmd(struct gk20a *g,
		bool wfi_cmd, struct priv_cmd_entry *cmd,
		u32 id, u64 gpu_va_base);
u32 gv11b_fifo_get_syncpt_incr_cmd_size(bool wfi_cmd);
int gv11b_init_fifo_setup_hw(struct gk20a *g);

void gv11b_fifo_tsg_verify_status_faulted(struct channel_gk20a *ch);
u32 gv11b_fifo_get_preempt_timeout(struct gk20a *g);

void gv11b_fifo_init_ramfc_eng_method_buffer(struct gk20a *g,
			struct channel_gk20a *ch, struct nvgpu_mem *mem);
void gv11b_userd_writeback_config(struct gk20a *g);
void gv11b_ring_channel_doorbell(struct channel_gk20a *c);
u64 gv11b_fifo_usermode_base(struct gk20a *g);
#endif
