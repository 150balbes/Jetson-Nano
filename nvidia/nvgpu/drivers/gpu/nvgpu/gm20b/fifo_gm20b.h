/*
 * GM20B Fifo
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

#ifndef NVGPU_GM20B_FIFO_GM20B_H
#define NVGPU_GM20B_FIFO_GM20B_H
struct gk20a;
struct mmu_fault_info;

void channel_gm20b_bind(struct channel_gk20a *c);
void gm20b_fifo_trigger_mmu_fault(struct gk20a *g,
		unsigned long engine_ids);
u32 gm20b_fifo_get_num_fifos(struct gk20a *g);
void gm20b_device_info_data_parse(struct gk20a *g,
						u32 table_entry, u32 *inst_id,
						u32 *pri_base, u32 *fault_id);
void gm20b_fifo_init_pbdma_intr_descs(struct fifo_gk20a *f);
void gm20b_fifo_tsg_verify_status_ctx_reload(struct channel_gk20a *ch);
void gm20b_fifo_get_mmu_fault_gpc_desc(struct mmu_fault_info *mmfault);

#endif /* NVGPU_GM20B_FIFO_GM20B_H */
