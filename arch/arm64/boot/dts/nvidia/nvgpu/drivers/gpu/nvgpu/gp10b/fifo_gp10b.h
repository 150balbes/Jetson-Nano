/*
 * GP10B Fifo
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef FIFO_GP10B_H
#define FIFO_GP10B_H

struct gpu_ops;
struct channel_gk20a;
struct fifo_gk20a;
struct mmu_fault_info;

int channel_gp10b_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries,
			unsigned long acquire_timeout, u32 flags);
u32 gp10b_fifo_get_pbdma_signature(struct gk20a *g);
int gp10b_fifo_resetup_ramfc(struct channel_gk20a *c);
int gp10b_fifo_engine_enum_from_type(struct gk20a *g, u32 engine_type,
					u32 *inst_id);
void gp10b_device_info_data_parse(struct gk20a *g, u32 table_entry,
				u32 *inst_id, u32 *pri_base, u32 *fault_id);
void gp10b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f);
void gp10b_fifo_get_mmu_fault_info(struct gk20a *g, u32 mmu_fault_id,
	struct mmu_fault_info *mmfault);
void gp10b_fifo_get_mmu_fault_desc(struct mmu_fault_info *mmfault);
void gp10b_fifo_get_mmu_fault_client_desc(struct mmu_fault_info *mmfault);
int channel_gp10b_commit_userd(struct channel_gk20a *c);

#endif
