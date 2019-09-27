/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _FIFO_VGPU_H_
#define _FIFO_VGPU_H_

#include <nvgpu/types.h>

struct gk20a;
struct channel_gk20a;
struct fifo_gk20a;
struct tsg_gk20a;

int vgpu_init_fifo_setup_hw(struct gk20a *g);
void vgpu_channel_bind(struct channel_gk20a *ch);
void vgpu_channel_unbind(struct channel_gk20a *ch);
int vgpu_channel_alloc_inst(struct gk20a *g, struct channel_gk20a *ch);
void vgpu_channel_free_inst(struct gk20a *g, struct channel_gk20a *ch);
void vgpu_channel_enable(struct channel_gk20a *ch);
void vgpu_channel_disable(struct channel_gk20a *ch);
int vgpu_channel_setup_ramfc(struct channel_gk20a *ch, u64 gpfifo_base,
				u32 gpfifo_entries,
				unsigned long acquire_timeout, u32 flags);
int vgpu_fifo_init_engine_info(struct fifo_gk20a *f);
int vgpu_fifo_preempt_channel(struct gk20a *g, struct channel_gk20a *ch);
int vgpu_fifo_preempt_tsg(struct gk20a *g, struct tsg_gk20a *tsg);
int vgpu_fifo_update_runlist(struct gk20a *g, u32 runlist_id,
				u32 chid, bool add, bool wait_for_finish);
int vgpu_fifo_wait_engine_idle(struct gk20a *g);
int vgpu_fifo_set_runlist_interleave(struct gk20a *g,
					u32 id,
					u32 runlist_id,
					u32 new_level);
int vgpu_channel_set_timeslice(struct channel_gk20a *ch, u32 timeslice);
int vgpu_fifo_force_reset_ch(struct channel_gk20a *ch,
					u32 err_code, bool verbose);
u32 vgpu_fifo_default_timeslice_us(struct gk20a *g);
int vgpu_tsg_open(struct tsg_gk20a *tsg);
void vgpu_tsg_release(struct tsg_gk20a *tsg);
int vgpu_tsg_bind_channel(struct tsg_gk20a *tsg,
			struct channel_gk20a *ch);
int vgpu_tsg_unbind_channel(struct channel_gk20a *ch);
int vgpu_tsg_set_timeslice(struct tsg_gk20a *tsg, u32 timeslice);
int vgpu_enable_tsg(struct tsg_gk20a *tsg);

#endif
