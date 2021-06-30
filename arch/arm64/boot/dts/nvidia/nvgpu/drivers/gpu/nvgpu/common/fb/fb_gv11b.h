/*
 * GV11B FB
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

#ifndef NVGPU_FB_GV11B_H
#define NVGPU_FB_GV11B_H

#define	NONREPLAY_REG_INDEX		0
#define	REPLAY_REG_INDEX		1

struct gk20a;
struct gr_gk20a;

void gv11b_fb_init_hw(struct gk20a *g);

void gv11b_fb_init_fs_state(struct gk20a *g);
void gv11b_fb_init_cbc(struct gk20a *g, struct gr_gk20a *gr);
void gv11b_fb_reset(struct gk20a *g);
void gv11b_fb_hub_isr(struct gk20a *g);

bool gv11b_fb_is_fault_buf_enabled(struct gk20a *g, u32 index	);
void gv11b_fb_fault_buf_set_state_hw(struct gk20a *g,
		 u32 index, u32 state);
void gv11b_fb_fault_buf_configure_hw(struct gk20a *g, u32 index);
void gv11b_fb_enable_hub_intr(struct gk20a *g);
void gv11b_fb_disable_hub_intr(struct gk20a *g);
bool gv11b_fb_mmu_fault_pending(struct gk20a *g);
void gv11b_fb_handle_dropped_mmu_fault(struct gk20a *g, u32 fault_status);
void gv11b_fb_handle_other_fault_notify(struct gk20a *g,
			 u32 fault_status);
void gv11b_fb_handle_mmu_nonreplay_replay_fault(struct gk20a *g,
		 u32 fault_status, u32 index);
void gv11b_fb_handle_nonreplay_fault_overflow(struct gk20a *g,
			 u32 fault_status);
void gv11b_fb_handle_replay_fault_overflow(struct gk20a *g,
			 u32 fault_status);
void gv11b_fb_handle_replayable_mmu_fault(struct gk20a *g);
void gv11b_handle_l2tlb_ecc_isr(struct gk20a *g, u32 ecc_status);
void gv11b_handle_hubtlb_ecc_isr(struct gk20a *g, u32 ecc_status);
void gv11b_handle_fillunit_ecc_isr(struct gk20a *g, u32 ecc_status);

void fb_gv11b_write_mmu_fault_buffer_lo_hi(struct gk20a *g, u32 index,
	u32 addr_lo, u32 addr_hi);
u32 fb_gv11b_read_mmu_fault_buffer_get(struct gk20a *g, u32 index);
void fb_gv11b_write_mmu_fault_buffer_get(struct gk20a *g, u32 index,
	u32 reg_val);
u32 fb_gv11b_read_mmu_fault_buffer_put(struct gk20a *g, u32 index);
u32 fb_gv11b_read_mmu_fault_buffer_size(struct gk20a *g, u32 index);
void fb_gv11b_write_mmu_fault_buffer_size(struct gk20a *g, u32 index,
	u32 reg_val);
void fb_gv11b_read_mmu_fault_addr_lo_hi(struct gk20a *g,
	u32 *addr_lo, u32 *addr_hi);
void fb_gv11b_read_mmu_fault_inst_lo_hi(struct gk20a *g,
	u32 *inst_lo, u32 *inst_hi);
u32 fb_gv11b_read_mmu_fault_info(struct gk20a *g);
u32 fb_gv11b_read_mmu_fault_status(struct gk20a *g);
void fb_gv11b_write_mmu_fault_status(struct gk20a *g, u32 reg_val);

int gv11b_fb_mmu_invalidate_replay(struct gk20a *g,
			 u32 invalidate_replay_val);

#endif /* NVGPU_FB_GV11B_H */
